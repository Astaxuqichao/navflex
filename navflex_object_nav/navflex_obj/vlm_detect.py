"""VLM object detector: is a named target visible in ONE real camera frame, where?

This is the object-grounding brain. It runs a vision LLM on a single live ego-view
JPEG (NOT an imagined rollout) and answers: is {target} visible, and if so at what
normalized position in the image. The grounder node turns that position into a
bearing; LIDAR then supplies the range.

Backend config mirrors navflex_wm.critic exactly (same kinds / env vars / params),
so the detector and the safety critic can share one aggregator/key:
  openai_compat : POST {base_url}/chat/completions  (what the CN aggregators speak)
  claude        : Anthropic Messages API (vision + schema-constrained JSON)
  null          : never detects — for wiring tests without burning a VLM call

Deliberately separate from critic.py (a different prompt + schema — detection, not
a safety verdict) and kept in this package so navflex_world_model is untouched.
"""

from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


# Lead with the task, not with "these may be predictions" — same lesson as the
# critic: warning about artifacts first makes models hallucinate them. Detection
# is on a REAL frame anyway, so there are no rollout artifacts to warn about.
DETECT_SYSTEM_PROMPT = """你是机器人第一人称相机的物体检测器。

给你一张【真实】相机画面和一个目标物体的描述（可能带地标，如"书架旁的绿色盆栽"——
地标只是帮你确认是不是同一个物体，真正要找的是那个【物体】本身）。

判断这个目标物体此刻是否【清晰可见】于画面中：
- 若可见：visible=true，并给出它中心在画面里的归一化位置
  x_frac(0=最左, 1=最右)、y_frac(0=最上, 1=最下)，以及 confidence(0~1)。
- 若不可见、只是猜它可能在别处、或画面无法辨认：visible=false。

只依据画面里【实际可见】的内容判断，绝不臆造看不到的物体。宁可 visible=false 也不误报。
如果画面几乎没有内容（纯色/全黑/无法辨认），visible=false 并在 reason 里说明看不清。
reason 用一句中文说明依据。只输出一个 JSON 对象。"""

DETECT_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'visible': {'type': 'boolean'},
        'confidence': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
        'x_frac': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
        'y_frac': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
        'reason': {'type': 'string'},
    },
    'required': ['visible', 'confidence', 'reason'],
}


class DetectorError(RuntimeError):
    pass


@dataclass
class Detection:
    visible: bool = False
    confidence: float = 0.0
    x_frac: float = 0.5
    y_frac: float = 0.5
    reason: str = ''
    source: str = ''
    raw: Dict[str, Any] = field(default_factory=dict)


def detection_from_json(parsed: Dict[str, Any], source: str) -> Detection:
    visible = bool(parsed.get('visible', False))

    def _clamp01(v, default):
        try:
            return max(0.0, min(1.0, float(v)))
        except (TypeError, ValueError):
            return default

    return Detection(
        visible=visible,
        confidence=_clamp01(parsed.get('confidence'), 0.0),
        # Only meaningful when visible; default to image centre otherwise.
        x_frac=_clamp01(parsed.get('x_frac'), 0.5),
        y_frac=_clamp01(parsed.get('y_frac'), 0.5),
        reason=str(parsed.get('reason', '')),
        source=source,
        raw=parsed,
    )


def _extract_json_object(text: str) -> Dict[str, Any]:
    """Best-effort: pull the first balanced {...} object out of an LLM reply."""
    if not text:
        raise DetectorError('empty VLM response')
    start = text.find('{')
    if start < 0:
        raise DetectorError(f'no JSON object in VLM response: {text[:200]!r}')
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(text)):
        c = text[i]
        if in_str:
            if esc:
                esc = False
            elif c == '\\':
                esc = True
            elif c == '"':
                in_str = False
        else:
            if c == '"':
                in_str = True
            elif c == '{':
                depth += 1
            elif c == '}':
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(text[start:i + 1])
                    except json.JSONDecodeError as exc:
                        raise DetectorError(f'malformed JSON: {exc}') from exc
    raise DetectorError('unterminated JSON object in VLM response')


class Detector:
    name = 'base'

    def health(self) -> str:  # pragma: no cover - overridden
        return 'ok'

    def detect(self, frame_jpeg: bytes, target: str) -> Detection:  # pragma: no cover
        raise NotImplementedError

    @staticmethod
    def _user_text(target: str) -> str:
        return f'目标物体：{target}\n判断它是否清晰可见于这张画面，可见则给出中心归一化位置。'


class NullDetector(Detector):
    name = 'null'

    def health(self) -> str:
        return 'ok (null detector — never reports visible)'

    def detect(self, frame_jpeg, target) -> Detection:
        return Detection(visible=False, confidence=0.0,
                         reason='null detector: no VLM configured', source='null')


class OpenAICompatDetector(Detector):
    """Detection through an OpenAI-compatible /v1/chat/completions endpoint."""

    name = 'openai_compat'

    def __init__(self, base_url: str, model: str, api_key: str = '',
                 proxy: str = '', timeout: float = 60.0, image_detail: str = 'auto'):
        if not base_url:
            raise DetectorError('openai_compat detector needs a base_url')
        if not model:
            raise DetectorError('openai_compat detector needs a model')
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_key = api_key
        self.proxy = proxy
        self.timeout = timeout
        self.image_detail = image_detail

    def _opener(self):
        handlers = []
        if self.proxy:
            handlers.append(urllib.request.ProxyHandler(
                {'http': self.proxy, 'https': self.proxy}))
        else:
            handlers.append(urllib.request.ProxyHandler({}))
        return urllib.request.build_opener(*handlers)

    def _headers(self) -> Dict[str, str]:
        h = {'Content-Type': 'application/json'}
        if self.api_key:
            h['Authorization'] = f'Bearer {self.api_key}'
        return h

    def _post(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        req = urllib.request.Request(
            self.base_url + '/chat/completions',
            data=json.dumps(payload).encode('utf-8'),
            headers=self._headers(), method='POST')
        try:
            with self._opener().open(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', 'replace')[:300]
            raise DetectorError(f'HTTP {exc.code}: {body}') from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise DetectorError(f'request failed: {exc}') from exc

    def health(self) -> str:
        try:
            req = urllib.request.Request(
                self.base_url + '/models', headers=self._headers(), method='GET')
            with self._opener().open(req, timeout=min(self.timeout, 15.0)) as resp:
                data = json.loads(resp.read().decode('utf-8'))
        except Exception as exc:  # noqa: BLE001 - health must never raise
            return f'unavailable: {exc}'
        ids = [m.get('id') for m in data.get('data', [])]
        if ids and self.model not in ids:
            return f"unavailable: model '{self.model}' not offered ({len(ids)} listed)"
        return f'ok ({self.model} @ {self.base_url})'

    def detect(self, frame_jpeg: bytes, target: str) -> Detection:
        if not frame_jpeg:
            raise DetectorError('no image bytes')
        image_url = 'data:image/jpeg;base64,' + base64.b64encode(frame_jpeg).decode('ascii')
        content = [
            {'type': 'text', 'text': self._user_text(target)},
            {'type': 'image_url',
             'image_url': {'url': image_url, 'detail': self.image_detail}},
        ]
        messages = [
            {'role': 'system', 'content': DETECT_SYSTEM_PROMPT},
            {'role': 'user', 'content': content},
        ]
        base = {'model': self.model, 'messages': messages,
                'max_tokens': 512, 'temperature': 0.0}
        # Degrade structured-output support: json_schema -> json_object -> plain.
        attempts = [
            {**base, 'response_format': {
                'type': 'json_schema',
                'json_schema': {'name': 'navflex_object_detection',
                                'schema': DETECT_SCHEMA, 'strict': True}}},
            {**base, 'response_format': {'type': 'json_object'}},
            base,
        ]
        last: Optional[Exception] = None
        for payload in attempts:
            try:
                data = self._post(payload)
                text = (data.get('choices', [{}])[0]
                            .get('message', {}).get('content', '')) or ''
                if isinstance(text, list):  # some providers return content parts
                    text = ''.join(p.get('text', '') for p in text if isinstance(p, dict))
                parsed = _extract_json_object(text)
                return detection_from_json(parsed, self.name)
            except DetectorError as exc:
                last = exc
                continue
        raise last or DetectorError('chat/completions produced no usable detection')


class ClaudeDetector(Detector):
    """Detection through the Anthropic Messages API (vision + schema)."""

    name = 'claude'

    def __init__(self, model: str = '', api_key: str = '', timeout: float = 60.0):
        try:
            import anthropic  # noqa: F401
        except ImportError as exc:
            raise DetectorError(f'anthropic SDK not installed: {exc}') from exc
        import anthropic
        self.model = model or 'claude-sonnet-5'
        self._client = anthropic.Anthropic(
            api_key=api_key or None, timeout=timeout)

    def health(self) -> str:
        try:
            self._client.models.retrieve(self.model)
        except Exception as exc:  # noqa: BLE001
            return f'unavailable: {exc}'
        return f'ok ({self.model})'

    def detect(self, frame_jpeg: bytes, target: str) -> Detection:
        if not frame_jpeg:
            raise DetectorError('no image bytes')
        content = [
            {'type': 'text', 'text': self._user_text(target)},
            {'type': 'image', 'source': {
                'type': 'base64', 'media_type': 'image/jpeg',
                'data': base64.b64encode(frame_jpeg).decode('ascii')}},
        ]
        try:
            resp = self._client.messages.create(
                model=self.model, max_tokens=512,
                system=DETECT_SYSTEM_PROMPT + '\n只输出一个 JSON 对象，不要额外文字。',
                messages=[{'role': 'user', 'content': content}])
        except Exception as exc:  # noqa: BLE001
            raise DetectorError(f'Claude request failed: {exc}') from exc
        text = ''.join(b.text for b in resp.content if getattr(b, 'type', '') == 'text')
        parsed = _extract_json_object(text)
        return detection_from_json(parsed, self.name)


def build_detector(kind: str, **kwargs) -> Detector:
    """Mirror navflex_wm.critic.build_critic's kinds/aliases."""
    kind = (kind or 'null').strip().lower()
    if kind in ('null', 'none', 'off', ''):
        return NullDetector()
    if kind in ('openai_compat', 'compat', 'aggregator'):
        return OpenAICompatDetector(
            base_url=kwargs.get('base_url', ''),
            model=kwargs.get('model', '') or 'gpt-4o',
            api_key=kwargs.get('api_key', ''),
            proxy=kwargs.get('proxy', ''),
            timeout=float(kwargs.get('timeout', 60.0)))
    if kind in ('claude', 'anthropic'):
        return ClaudeDetector(
            model=kwargs.get('model', ''),
            api_key=kwargs.get('api_key', ''),
            timeout=float(kwargs.get('timeout', 60.0)))
    raise DetectorError(f"unknown detector kind '{kind}'")
