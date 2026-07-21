#!/usr/bin/env python3
"""Judges an imagined rollout and returns a verdict.

The generator can only show what the near future looks like; something still has
to decide whether that future is acceptable. LingBot-World is generation-only --
it has no reasoner -- so the judging is delegated to a vision LLM.

Backends: Claude (vision plus a schema-constrained verdict), any OpenAI-compatible
/v1/chat/completions endpoint (what the model aggregators speak), the OpenAI
Responses API and Codex CLI reused from `navflex_instruction_web.py`, and a null
critic that refuses to judge.

The critic never touches the robot. It returns one of ``approve``, ``reject``
or ``needs_confirmation``, and `navflex_task_server` maps that onto the
confirmation gate it already had.
"""

from __future__ import annotations

import base64
import json
import os
import shutil
import subprocess
import urllib.error
import urllib.request
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

VERDICTS = ('approve', 'reject', 'needs_confirmation')

VERDICT_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'verdict': {'type': 'string', 'enum': list(VERDICTS)},
        'confidence': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
        'reason': {'type': 'string'},
        'hazards': {'type': 'array', 'items': {'type': 'string'}},
    },
    'required': ['verdict', 'confidence', 'reason', 'hazards'],
}

# Do not open by warning about generation artifacts. An earlier version led with
# "these frames are predicted and may contain artifacts", and models anchored on
# it: they reported severe artifacts even in flat grey images, hallucinated a
# corridor that was not there, and returned needs_confirmation for every single
# rollout -- a gate that blocks everything is not a gate.
SYSTEM_PROMPT = """你是移动机器人导航的安全审查员。

输入是一组第一人称相机画面，按时间先后排列，表示机器人若执行当前规划路径将看到的景象。
最后一帧是路径终点附近。

判断按此路径行驶是否 (1) 安全，(2) 符合用户指令：

- approve：画面里前方通路可见且畅通，没有会撞上的障碍、台阶或跌落风险，且与指令一致。
- reject：出现明确的碰撞、跌落、驶入人群，或与指令明显冲突。
- needs_confirmation：证据不足以判断——看不清通路、信息不足、或风险无法排除。

只依据画面里实际可见的内容判断，不要臆测看不到的东西。
如果画面几乎没有内容（纯色、全黑、无法辨认场景），那就是证据不足，
直接给 needs_confirmation 并说明看不到东西，不要描述并不存在的物体。

画面可能带有轻微噪点或形变。只有当它严重到你无法辨认通路时，才因此给 needs_confirmation。
通路清晰且与指令一致时就给 approve——不要仅因为"这是预测而非真实观测"就额外保守。

reason 用一句中文写明关键依据。只输出一个 JSON 对象。"""


class CriticError(RuntimeError):
    pass


def even_sample(frames: List[bytes], count: int) -> List[bytes]:
    """Pick ``count`` frames spread evenly across the rollout, endpoints included.

    A rollout is dozens of frames; a critic reads a handful. Sampling evenly
    keeps the first and last frame, which is where the robot is now and where the
    plan takes it.
    """
    if count <= 0:
        return []
    if len(frames) <= count:
        return frames
    if count == 1:
        # The last frame says the most about where the plan ends up.
        return [frames[-1]]
    step = (len(frames) - 1) / (count - 1)
    return [frames[round(i * step)] for i in range(count)]


class Verdict:
    def __init__(
        self,
        verdict: str,
        confidence: float,
        reason: str,
        hazards: List[str] = None,
        source: str = '',
        confidence_reported: bool = True,
    ):
        if verdict not in VERDICTS:
            verdict = 'needs_confirmation'
        self.verdict = verdict
        self.confidence = max(0.0, min(1.0, float(confidence)))
        self.reason = reason
        self.hazards = list(hazards or [])
        self.source = source
        # False when the critic never gave a number and we filled in 0.0. A
        # fabricated 0.0 must not be read as "the critic was unsure".
        self.confidence_reported = confidence_reported

    def to_dict(self) -> Dict[str, Any]:
        return {
            'verdict': self.verdict,
            'confidence': self.confidence,
            'confidence_reported': self.confidence_reported,
            'reason': self.reason,
            'hazards': self.hazards,
            'source': self.source,
        }


class Critic(ABC):
    name = 'abstract'

    @abstractmethod
    def critique(self, frames: List[bytes], instruction: str, plan_summary: str) -> Verdict:
        ...

    def health(self) -> str:
        """Cheap reachability probe, run once at startup. Never raises."""
        return 'ok'


class NullCritic(Critic):
    """Refuses to judge. Never approves on no evidence."""

    name = 'null'

    def critique(self, frames, instruction, plan_summary) -> Verdict:
        del instruction, plan_summary
        return Verdict(
            'needs_confirmation', 0.0,
            f'no critic configured; {len(frames)} imagined frames were not reviewed',
            source='null')

    def health(self) -> str:
        return 'ok (null critic, never approves)'


class ClaudeCritic(Critic):
    """Vision critique through Claude. The default backend.

    Structured outputs (``output_config.format``) constrain the reply to
    VERDICT_SCHEMA, so a malformed verdict cannot reach the gate in the first
    place. Adaptive thinking is on: deciding whether an imagined rollout
    collides or drifts off-instruction is exactly the kind of judgement worth
    spending reasoning tokens on.
    """

    name = 'claude'

    def __init__(
        self,
        model: str = 'claude-opus-4-8',
        api_key: str = '',
        effort: str = 'medium',
        timeout: float = 120.0,
        max_frames: int = 6,
        max_tokens: int = 4096,
        base_url: str = '',
        proxy: str = '',
    ):
        try:
            import anthropic
        except ImportError as exc:
            raise CriticError(
                'claude critic needs the anthropic SDK: pip install anthropic') from exc

        # A bare Anthropic() also resolves ANTHROPIC_AUTH_TOKEN and an
        # `ant auth login` profile, so an unset ANTHROPIC_API_KEY is not fatal.
        options: Dict[str, Any] = {'timeout': timeout}
        if api_key:
            options['api_key'] = api_key
        if base_url:
            options['base_url'] = base_url
        if proxy:
            # httpx would read the proxy from the environment, but the ROS node
            # runs in a container that does not inherit it, so pass it through.
            #
            # Not `DefaultHttpxClient(proxy=...)`: that class builds its own
            # transport and mounts it from the *environment* proxies only, and
            # httpx gives `mounts` precedence over `proxy`, so an explicit
            # proxy= is silently ignored (anthropic 0.116.0). Overriding
            # `mounts` is the escape hatch its own source points at, and it
            # keeps the SDK's timeout / connection-pool / keepalive defaults.
            #
            # Mount both schemes explicitly: httpx picks the most specific key,
            # so a bare 'all://' would still lose to an 'https://' entry that
            # the environment contributed.
            import httpx
            transport = httpx.HTTPTransport(proxy=proxy)
            options['http_client'] = anthropic.DefaultHttpxClient(
                mounts={'http://': transport, 'https://': transport,
                        'all://': transport})
        try:
            self._client = anthropic.Anthropic(**options)
        except anthropic.AnthropicError as exc:
            # Surface as CriticError so the gate degrades to the null critic
            # rather than taking the ROS node down at startup.
            raise CriticError(f'could not construct the Claude client: {exc}') from exc
        self._errors = anthropic
        self.model = model
        self.effort = effort
        self.max_frames = max_frames
        self.max_tokens = max_tokens

    def health(self) -> str:
        """Probe with models.retrieve -- it is not billed and not rate-limited.

        Catches a bad key, an unreachable proxy, or a model ID that does not
        exist, at startup rather than on the first plan. It does NOT catch an
        exhausted credit balance: that surfaces as a 400 only on a billed
        request, and would otherwise turn every navigation task into a silent
        needs_confirmation.
        """
        try:
            model = self._client.models.retrieve(self.model)
        except self._errors.APIStatusError as exc:
            return f'unavailable: HTTP {exc.status_code} ({exc.message})'
        except self._errors.APIConnectionError as exc:
            return f'unreachable: {exc}'
        return f'ok ({model.id} reachable, effort={self.effort})'

    def critique(self, frames, instruction, plan_summary) -> Verdict:
        if not frames:
            return Verdict('needs_confirmation', 0.0,
                           'world model produced no frames to review', source='claude')

        sampled = even_sample(frames, self.max_frames)
        content: List[Dict[str, Any]] = [{
            'type': 'text',
            'text': (f'用户指令：{instruction}\n'
                     f'规划摘要：{plan_summary}\n'
                     f'以下 {len(sampled)} 帧按时间先后排列。'),
        }]
        for frame in sampled:
            content.append({
                'type': 'image',
                'source': {
                    'type': 'base64',
                    'media_type': 'image/jpeg',
                    'data': base64.b64encode(frame).decode('ascii'),
                },
            })

        try:
            response = self._client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=SYSTEM_PROMPT,
                thinking={'type': 'adaptive'},
                output_config={
                    'effort': self.effort,
                    'format': {'type': 'json_schema', 'schema': VERDICT_SCHEMA},
                },
                messages=[{'role': 'user', 'content': content}],
            )
        except self._errors.APIStatusError as exc:
            raise CriticError(f'Claude API {exc.status_code}: {exc.message}') from exc
        except self._errors.APIConnectionError as exc:
            raise CriticError(f'Claude request failed: {exc}') from exc

        # Claude declines some requests outright; that is not an approval.
        if response.stop_reason == 'refusal':
            return Verdict('needs_confirmation', 0.0,
                           'Claude declined to review this rollout', source='claude')
        if response.stop_reason == 'max_tokens':
            raise CriticError('Claude hit max_tokens before completing the verdict')

        text = next((b.text for b in response.content if b.type == 'text'), '')
        if not text:
            raise CriticError('Claude returned no text block')
        parsed = _parse_json_object(text)
        return verdict_from_json(parsed, 'claude')


class OpenAICompatCritic(Critic):
    """Vision critique through an OpenAI-compatible /v1/chat/completions endpoint.

    This is what the Chinese model aggregators speak -- not the Responses API
    that :class:`OpenAICritic` uses. Kept on urllib rather than the openai SDK
    to avoid another dependency inside the robot's ROS container; a chat
    completion is a single JSON POST.

    Structured output support varies by provider, so the request degrades:
    json_schema -> json_object -> plain prompting. Whatever comes back is still
    validated by :class:`Verdict`, so a provider that ignores all three cannot
    smuggle an approval through.
    """

    name = 'openai_compat'

    def __init__(
        self,
        base_url: str,
        model: str,
        api_key: str = '',
        proxy: str = '',
        timeout: float = 120.0,
        max_frames: int = 6,
        max_tokens: int = 2048,
        image_detail: str = 'auto',
    ):
        if not base_url:
            raise CriticError('openai_compat critic needs a base_url')
        if not model:
            raise CriticError('openai_compat critic needs a model')
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_key = api_key
        self.proxy = proxy
        self.timeout = timeout
        self.max_frames = max_frames
        self.max_tokens = max_tokens
        self.image_detail = image_detail

    def health(self) -> str:
        try:
            data = self._get('/models')
        except CriticError as exc:
            return f'unavailable: {exc}'
        ids = [m.get('id') for m in data.get('data', [])]
        if ids and self.model not in ids:
            return f"unavailable: model '{self.model}' not in /models ({len(ids)} offered)"
        return f'ok ({self.model} listed by {self.base_url})'

    def critique(self, frames, instruction, plan_summary) -> Verdict:
        if not frames:
            return Verdict('needs_confirmation', 0.0,
                           'world model produced no frames to review',
                           source=self.name)

        content: List[Dict[str, Any]] = [{
            'type': 'text',
            'text': (f'用户指令：{instruction}\n'
                     f'规划摘要：{plan_summary}\n'
                     f'以下 {min(len(frames), self.max_frames)} 帧按时间先后排列。'),
        }]
        for frame in even_sample(frames, self.max_frames):
            content.append({
                'type': 'image_url',
                'image_url': {
                    'url': 'data:image/jpeg;base64,'
                           + base64.b64encode(frame).decode('ascii'),
                    'detail': self.image_detail,
                },
            })

        base = {
            'model': self.model,
            'max_tokens': self.max_tokens,
            'messages': [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': content},
            ],
        }
        # Most permissive first; each provider rejects what it does not know.
        formats = [
            {'type': 'json_schema', 'json_schema': {
                'name': 'navflex_world_model_verdict',
                'strict': True,
                'schema': VERDICT_SCHEMA,
            }},
            {'type': 'json_object'},
            None,
        ]
        last: Optional[CriticError] = None
        for response_format in formats:
            payload = dict(base)
            if response_format:
                payload['response_format'] = response_format
            else:
                payload['messages'] = [
                    {'role': 'system',
                     'content': SYSTEM_PROMPT + '\n只输出一个 JSON 对象，不要输出 Markdown。'},
                    base['messages'][1],
                ]
            try:
                data = self._post('/chat/completions', payload)
            except CriticError as exc:
                last = exc
                continue
            # Providers vary: `choices` can be absent, `message` can be null, and
            # a reasoning model can spend the whole budget thinking and return
            # content: "". Never index blindly -- a crash here takes the ROS
            # node down instead of degrading to needs_confirmation.
            choices = data.get('choices') or []
            message = (choices[0] or {}).get('message') if choices else None
            text = (message or {}).get('content') or ''
            if not text.strip():
                last = CriticError('response had no message content')
                continue
            try:
                parsed = _parse_json_object(text)
            except CriticError as exc:
                last = exc
                continue
            return verdict_from_json(parsed, self.name)
        raise last or CriticError('chat/completions produced no usable verdict')

    def _opener(self):
        if self.proxy:
            return urllib.request.build_opener(urllib.request.ProxyHandler(
                {'http': self.proxy, 'https': self.proxy}))
        return urllib.request.build_opener()

    def _headers(self) -> Dict[str, str]:
        headers = {'Content-Type': 'application/json'}
        if self.api_key:
            headers['Authorization'] = f'Bearer {self.api_key}'
        return headers

    def _get(self, route: str) -> Dict[str, Any]:
        request = urllib.request.Request(
            f'{self.base_url}{route}', headers=self._headers(), method='GET')
        return self._send(request)

    def _post(self, route: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        request = urllib.request.Request(
            f'{self.base_url}{route}',
            data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
            headers=self._headers(), method='POST')
        return self._send(request)

    def _send(self, request) -> Dict[str, Any]:
        try:
            with self._opener().open(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')[:300]
            raise CriticError(f'HTTP {exc.code}: {detail}') from exc
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise CriticError(f'request to {self.base_url} failed: {exc}') from exc


class OpenAICritic(Critic):
    """Vision critique through the OpenAI Responses API.

    Mirrors `navflex_instruction_web.py`: same env-var names, same proxy
    handling, same strict-json_schema-then-retry-without-it degradation.
    """

    name = 'openai'

    def __init__(
        self,
        api_url: str = 'https://api.openai.com/v1/responses',
        api_key: str = '',
        model: str = 'gpt-5.5',
        proxy: str = '',
        timeout: float = 120.0,
        max_frames: int = 6,
    ):
        if not api_key:
            raise CriticError('OpenAI critic needs an API key')
        self.api_url = api_url
        self.api_key = api_key
        self.model = model
        self.proxy = proxy
        self.timeout = timeout
        self.max_frames = max_frames

    def critique(self, frames, instruction, plan_summary) -> Verdict:
        if not frames:
            return Verdict('needs_confirmation', 0.0,
                           'world model produced no frames to review', source='openai')

        content: List[Dict[str, Any]] = [{
            'type': 'input_text',
            'text': (f'用户指令：{instruction}\n'
                     f'规划摘要：{plan_summary}\n'
                     f'以下 {min(len(frames), self.max_frames)} 帧按时间先后排列。'),
        }]
        for frame in even_sample(frames, self.max_frames):
            content.append({
                'type': 'input_image',
                'image_url': 'data:image/jpeg;base64,'
                             + base64.b64encode(frame).decode('ascii'),
            })

        payload = {
            'model': self.model,
            'instructions': SYSTEM_PROMPT,
            'input': [{'role': 'user', 'content': content}],
            'text': {
                'format': {
                    'type': 'json_schema',
                    'name': 'navflex_world_model_verdict',
                    'strict': True,
                    'schema': VERDICT_SCHEMA,
                },
            },
        }
        try:
            data = self._post(payload)
        except CriticError:
            # Some deployments reject strict json_schema; ask for bare JSON.
            payload.pop('text', None)
            payload['instructions'] += '\n只输出一个 JSON 对象，不要输出 Markdown。'
            data = self._post(payload)

        parsed = _parse_json_object(_extract_response_text(data))
        return verdict_from_json(parsed, 'openai')

    def _post(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        request = urllib.request.Request(
            self.api_url,
            data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
            method='POST')
        opener = urllib.request.build_opener()
        if self.proxy:
            opener = urllib.request.build_opener(
                urllib.request.ProxyHandler({'http': self.proxy, 'https': self.proxy}))
        try:
            with opener.open(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')[:400]
            raise CriticError(f'OpenAI HTTP {exc.code}: {detail}') from exc
        except (urllib.error.URLError, OSError) as exc:
            raise CriticError(f'OpenAI request failed: {exc}') from exc


class CodexCliCritic(Critic):
    """Judges through a locally logged-in Codex CLI.

    Codex reads images from disk rather than from the request body, so frames
    are written out and referenced by path.
    """

    name = 'codex_cli'

    def __init__(
        self,
        command: str = 'codex',
        args: List[str] = None,
        timeout: float = 180.0,
        max_frames: int = 6,
    ):
        if shutil.which(command) is None:
            raise CriticError(f'codex command not found: {command}')
        self.command = command
        self.args = list(args or ['exec', '--skip-git-repo-check', '-'])
        self.timeout = timeout
        self.max_frames = max_frames

    def critique(self, frames, instruction, plan_summary) -> Verdict:
        if not frames:
            return Verdict('needs_confirmation', 0.0,
                           'world model produced no frames to review', source='codex_cli')

        import tempfile
        with tempfile.TemporaryDirectory(prefix='navflex_critic_') as workdir:
            paths = []
            sampled = even_sample(frames, self.max_frames)
            for index, frame in enumerate(sampled):
                path = os.path.join(workdir, f'frame_{index:02d}.jpg')
                with open(path, 'wb') as handle:
                    handle.write(frame)
                paths.append(path)

            prompt = (
                f'{SYSTEM_PROMPT}\n\n'
                f'用户指令：{instruction}\n'
                f'规划摘要：{plan_summary}\n'
                f'按时间顺序读取这些预测帧：\n'
                + '\n'.join(paths)
                + '\n\n只输出 JSON 对象，不要输出 Markdown，不要解释。')
            completed = subprocess.run(
                [self.command] + self.args,
                input=prompt, text=True, capture_output=True,
                timeout=self.timeout, check=False)

        output = (completed.stdout or '').strip()
        if not output:
            raise CriticError(
                f'codex produced no output: {(completed.stderr or "")[:200]}')
        parsed = _parse_json_object(output)
        return verdict_from_json(parsed, 'codex_cli')


def _extract_response_text(data: Dict[str, Any]) -> str:
    if data.get('output_text'):
        return str(data['output_text'])
    chunks = []
    for item in data.get('output', []):
        for content in item.get('content', []):
            if isinstance(content, dict) and content.get('text'):
                chunks.append(str(content['text']))
    if chunks:
        return ''.join(chunks)
    raise CriticError('response did not contain text output')


_VERDICT_KEYS = ('verdict', 'decision', 'judgement', 'judgment', 'result')
_CONFIDENCE_KEYS = ('confidence', 'score', 'certainty')
_HAZARD_KEYS = ('hazards', 'risks', 'dangers')

# What models write when they are not held to the schema.
_VERDICT_ALIASES = {
    'approve': 'approve', 'approved': 'approve', 'accept': 'approve',
    'pass': 'approve', 'safe': 'approve', 'go': 'approve', 'yes': 'approve',
    'reject': 'reject', 'rejected': 'reject', 'deny': 'reject',
    'block': 'reject', 'unsafe': 'reject', 'stop': 'reject', 'no': 'reject',
    'needs_confirmation': 'needs_confirmation',
    'needs confirmation': 'needs_confirmation',
    'needs-confirmation': 'needs_confirmation',
    'confirm': 'needs_confirmation', 'uncertain': 'needs_confirmation',
    'unclear': 'needs_confirmation',
}


def verdict_from_json(parsed: Dict[str, Any], source: str) -> Verdict:
    """Build a Verdict from whatever shape the endpoint actually returned.

    Aggregators fall back from `json_schema` to `json_object` without warning,
    and an unconstrained model answers `{"decision": "approve", "reason": ...}`
    -- no `verdict`, no `confidence`. Read that literally and the gate blocks a
    perfectly clear path while printing an approving reason: safe, but it looks
    broken, and an operator learns to ignore the gate.

    Unknown *verdict* still fails closed. A missing *confidence* is recorded as
    unreported rather than invented, because 0.0 is a judgement the critic never
    made.
    """
    raw = ''
    for key in _VERDICT_KEYS:
        if parsed.get(key):
            raw = str(parsed[key]).strip().lower()
            break
    verdict = _VERDICT_ALIASES.get(raw, 'needs_confirmation')

    confidence, reported = 0.0, False
    for key in _CONFIDENCE_KEYS:
        if isinstance(parsed.get(key), (int, float)):
            confidence, reported = float(parsed[key]), True
            break

    hazards: List[str] = []
    for key in _HAZARD_KEYS:
        value = parsed.get(key)
        if isinstance(value, list):
            hazards = [str(v) for v in value]
            break

    reason = str(parsed.get('reason') or parsed.get('explanation') or '')
    if raw and raw not in _VERDICT_ALIASES:
        reason = (f'critic returned an unrecognised verdict {raw!r}; '
                  f'failing closed. Its reason: {reason}')
    elif not raw:
        reason = (f'critic returned no verdict field (keys: '
                  f'{sorted(parsed)}); failing closed. Its reason: {reason}')

    return Verdict(verdict, confidence, reason, hazards, source=source,
                   confidence_reported=reported)


def _parse_json_object(text: str) -> Dict[str, Any]:
    text = text.strip()
    if text.startswith('```'):
        text = text.strip('`')
        if text.lower().startswith('json'):
            text = text[4:]
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        raise CriticError(f'no JSON object in critic output: {text[:200]}')
    try:
        parsed = json.loads(text[start:end + 1])
    except json.JSONDecodeError as exc:
        raise CriticError(f'critic output is not valid JSON: {exc}') from exc
    if not isinstance(parsed, dict):
        raise CriticError('critic output must be a JSON object')
    return parsed


def build_critic(kind: str, **kwargs) -> Critic:
    kind = (kind or 'null').strip().lower()
    if kind in ('null', 'none', 'off'):
        return NullCritic()
    if kind in ('claude', 'anthropic'):
        return ClaudeCritic(
            model=kwargs.get('model') or 'claude-opus-4-8',
            api_key=kwargs.get('api_key') or '',
            effort=kwargs.get('effort') or 'medium',
            timeout=float(kwargs.get('timeout') or 120.0),
            max_frames=int(kwargs.get('max_frames') or 6),
            max_tokens=int(kwargs.get('max_tokens') or 4096),
            base_url=kwargs.get('base_url') or '',
            proxy=kwargs.get('proxy') or '')
    if kind in ('openai_compat', 'compat', 'aggregator'):
        return OpenAICompatCritic(
            base_url=kwargs.get('base_url') or '',
            model=kwargs.get('model') or '',
            api_key=kwargs.get('api_key') or '',
            proxy=kwargs.get('proxy') or '',
            timeout=float(kwargs.get('timeout') or 120.0),
            max_frames=int(kwargs.get('max_frames') or 6),
            max_tokens=int(kwargs.get('max_tokens') or 2048))
    if kind == 'openai':
        return OpenAICritic(
            api_url=kwargs.get('api_url') or 'https://api.openai.com/v1/responses',
            api_key=kwargs.get('api_key') or '',
            model=kwargs.get('model') or 'gpt-5.5',
            proxy=kwargs.get('proxy') or '',
            timeout=float(kwargs.get('timeout') or 120.0),
            max_frames=int(kwargs.get('max_frames') or 6))
    if kind in ('codex_cli', 'codex'):
        return CodexCliCritic(
            command=kwargs.get('command') or 'codex',
            args=kwargs.get('args'),
            timeout=float(kwargs.get('timeout') or 180.0),
            max_frames=int(kwargs.get('max_frames') or 6))
    raise CriticError(f"unknown critic backend '{kind}'")
