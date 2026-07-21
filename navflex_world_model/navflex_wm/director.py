"""Director critic: score an imagined rollout for "does this direction lead toward
the target?" — the ranking brain of phase-A object-goal search.

Given a short world-model rollout of driving toward a candidate frontier viewpoint
(a handful of imagined ego-view frames) and a target description, return a 0..1
score for how likely that DIRECTION heads toward the region the target lives in.
It judges room-type / doorway / corridor cues — NOT whether the target object is
already rendered (it usually is not, and the world model cannot reliably imagine
an unseen object anyway). The orchestrator picks the highest-scoring candidate.

Backend config mirrors navflex_wm.critic (openai_compat / claude / null), so the
safety critic, the object detector, and this director can share one aggregator/key.
Separate from critic.py because it is a different question (progress-toward-target,
not safety) with a different prompt + schema.
"""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


DIRECTOR_SYSTEM_PROMPT = """你是移动机器人探索的方向评估器。

给你一小段【第一人称想象画面】——机器人若朝某个候选方向前进、会看到的景象（由世界模型生成，
按时间先后排列），以及一个目标物体的描述（可能带地标，如"厨房边的冰箱"）。

判断【沿这个方向前进，最终通往目标所在区域的可能性有多大】：
- 看画面里的房间类型、门/走廊/家具线索是否与目标所在的场景一致
  （例如找冰箱 -> 是否朝着像厨房的区域；找衣柜 -> 是否朝着像卧室/楼梯的区域）。
- ★注意：目标物体本身此刻很可能还看不到。你判断的是【方向/房间类型对不对】，
  不是目标是否已经出现。别因为"没看到目标"就给低分。

给一个 0~1 的分数：1 = 非常可能通往目标所在区域，0.5 = 说不准，0 = 明显不通往（如死胡同、
与目标场景相反的房间）。只依据画面【实际可见】的线索，画面太空/无法辨认就给 0.5 并说明。
reason 用一句中文写明依据。只输出一个 JSON 对象。"""

DIRECTOR_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'score': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
        'reason': {'type': 'string'},
    },
    'required': ['score', 'reason'],
}


class DirectorError(RuntimeError):
    pass


@dataclass
class DirectorScore:
    score: float = -1.0     # -1 = no rollout / not scored
    reason: str = ''
    source: str = ''


def _extract_json_object(text: str) -> Dict[str, Any]:
    if not text:
        raise DirectorError('empty VLM response')
    start = text.find('{')
    if start < 0:
        raise DirectorError(f'no JSON object: {text[:200]!r}')
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
                        raise DirectorError(f'malformed JSON: {exc}') from exc
    raise DirectorError('unterminated JSON object')


def _even_sample(frames: List[bytes], count: int) -> List[bytes]:
    if count <= 0 or len(frames) <= count:
        return frames
    if count == 1:
        return [frames[-1]]
    step = (len(frames) - 1) / (count - 1)
    return [frames[round(i * step)] for i in range(count)]


def score_from_json(parsed: Dict[str, Any], source: str) -> DirectorScore:
    try:
        s = max(0.0, min(1.0, float(parsed.get('score'))))
    except (TypeError, ValueError):
        s = 0.5
    return DirectorScore(score=s, reason=str(parsed.get('reason', '')), source=source)


class Director:
    name = 'base'
    max_frames = 6

    def health(self) -> str:  # pragma: no cover
        return 'ok'

    def score(self, frames: List[bytes], target: str) -> DirectorScore:
        """Return a DirectorScore; frames == [] -> score -1 (no rollout to judge)."""
        if not frames:
            return DirectorScore(score=-1.0, reason='no rollout frames', source=self.name)
        return self._score(_even_sample(frames, self.max_frames), target)

    def _score(self, frames, target) -> DirectorScore:  # pragma: no cover
        raise NotImplementedError

    @staticmethod
    def _user_text(target: str, n: int) -> str:
        return (f'目标物体：{target}\n以下 {n} 帧是沿某候选方向前进的想象画面，按时间先后排列。'
                f'请评估沿这个方向前进通往目标所在区域的可能性。')


class NullDirector(Director):
    name = 'null'

    def health(self) -> str:
        return 'ok (null director — never scores)'

    def _score(self, frames, target) -> DirectorScore:
        return DirectorScore(score=-1.0, reason='null director', source='null')


class OpenAICompatDirector(Director):
    name = 'openai_compat'

    def __init__(self, base_url: str, model: str, api_key: str = '',
                 proxy: str = '', timeout: float = 120.0, max_frames: int = 6,
                 max_tokens: int = 2048):
        if not base_url or not model:
            raise DirectorError('openai_compat director needs base_url + model')
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_key = api_key
        self.proxy = proxy
        self.timeout = timeout
        self.max_frames = max_frames
        # Reasoning models (e.g. kimi-k2.5) spend tokens thinking before emitting
        # the JSON; 512 was too few and came back empty. Match the safety critic.
        self.max_tokens = max_tokens

    def _opener(self):
        h = [urllib.request.ProxyHandler(
            {'http': self.proxy, 'https': self.proxy} if self.proxy else {})]
        return urllib.request.build_opener(*h)

    def _headers(self):
        d = {'Content-Type': 'application/json'}
        if self.api_key:
            d['Authorization'] = f'Bearer {self.api_key}'
        return d

    def health(self) -> str:
        try:
            req = urllib.request.Request(
                self.base_url + '/models', headers=self._headers(), method='GET')
            with self._opener().open(req, timeout=min(self.timeout, 15.0)) as resp:
                data = json.loads(resp.read().decode('utf-8'))
        except Exception as exc:  # noqa: BLE001
            return f'unavailable: {exc}'
        ids = [m.get('id') for m in data.get('data', [])]
        if ids and self.model not in ids:
            return f"unavailable: model '{self.model}' not offered"
        return f'ok ({self.model} @ {self.base_url})'

    def _score(self, frames, target) -> DirectorScore:
        content = [{'type': 'text', 'text': self._user_text(target, len(frames))}]
        for f in frames:
            content.append({
                'type': 'image_url',
                'image_url': {'url': 'data:image/jpeg;base64,'
                              + base64.b64encode(f).decode('ascii'), 'detail': 'auto'}})
        base = {'model': self.model,
                'messages': [{'role': 'system', 'content': DIRECTOR_SYSTEM_PROMPT},
                             {'role': 'user', 'content': content}],
                'max_tokens': self.max_tokens, 'temperature': 0.0}
        attempts = [
            {**base, 'response_format': {'type': 'json_schema', 'json_schema': {
                'name': 'navflex_director', 'schema': DIRECTOR_SCHEMA, 'strict': True}}},
            {**base, 'response_format': {'type': 'json_object'}},
            base,
        ]
        last: Optional[Exception] = None
        for payload in attempts:
            try:
                req = urllib.request.Request(
                    self.base_url + '/chat/completions',
                    data=json.dumps(payload).encode('utf-8'),
                    headers=self._headers(), method='POST')
                with self._opener().open(req, timeout=self.timeout) as resp:
                    data = json.loads(resp.read().decode('utf-8'))
                text = data.get('choices', [{}])[0].get('message', {}).get('content', '') or ''
                if isinstance(text, list):
                    text = ''.join(p.get('text', '') for p in text if isinstance(p, dict))
                return score_from_json(_extract_json_object(text), self.name)
            except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError,
                    OSError, DirectorError) as exc:
                last = exc
                continue
        # Honor the same public contract as the critic/backend: always raise a
        # DirectorError so callers (rank_viewpoints) can skip ONE bad candidate
        # instead of a raw urllib/OS error escaping the per-candidate handler.
        if last is not None:
            raise DirectorError(str(last)) from last
        raise DirectorError('chat/completions produced no usable score')


class ClaudeDirector(Director):
    name = 'claude'

    def __init__(self, model: str = '', api_key: str = '', timeout: float = 60.0,
                 max_frames: int = 6):
        try:
            import anthropic  # noqa: F401
        except ImportError as exc:
            raise DirectorError(f'anthropic SDK not installed: {exc}') from exc
        import anthropic
        self.model = model or 'claude-sonnet-5'
        self.max_frames = max_frames
        self._client = anthropic.Anthropic(api_key=api_key or None, timeout=timeout)

    def health(self) -> str:
        try:
            self._client.models.retrieve(self.model)
        except Exception as exc:  # noqa: BLE001
            return f'unavailable: {exc}'
        return f'ok ({self.model})'

    def _score(self, frames, target) -> DirectorScore:
        content = [{'type': 'text', 'text': self._user_text(target, len(frames))}]
        for f in frames:
            content.append({'type': 'image', 'source': {
                'type': 'base64', 'media_type': 'image/jpeg',
                'data': base64.b64encode(f).decode('ascii')}})
        try:
            resp = self._client.messages.create(
                model=self.model, max_tokens=512,
                system=DIRECTOR_SYSTEM_PROMPT + '\n只输出一个 JSON 对象。',
                messages=[{'role': 'user', 'content': content}])
        except Exception as exc:  # noqa: BLE001
            raise DirectorError(f'Claude request failed: {exc}') from exc
        text = ''.join(b.text for b in resp.content if getattr(b, 'type', '') == 'text')
        return score_from_json(_extract_json_object(text), self.name)


def build_director(kind: str, **kwargs) -> Director:
    kind = (kind or 'null').strip().lower()
    if kind in ('null', 'none', 'off', ''):
        return NullDirector()
    if kind in ('openai_compat', 'compat', 'aggregator'):
        return OpenAICompatDirector(
            base_url=kwargs.get('base_url', ''), model=kwargs.get('model', '') or 'gpt-4o',
            api_key=kwargs.get('api_key', ''), proxy=kwargs.get('proxy', ''),
            timeout=float(kwargs.get('timeout', 60.0)),
            max_frames=int(kwargs.get('max_frames', 6)))
    if kind in ('claude', 'anthropic'):
        return ClaudeDirector(
            model=kwargs.get('model', ''), api_key=kwargs.get('api_key', ''),
            timeout=float(kwargs.get('timeout', 60.0)),
            max_frames=int(kwargs.get('max_frames', 6)))
    raise DirectorError(f"unknown director kind '{kind}'")
