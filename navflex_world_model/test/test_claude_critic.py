#!/usr/bin/env python3
"""Exercise the Claude critic against a stub Messages API.

Points the real anthropic SDK at a local stub via base_url, so the request the
SDK actually builds is inspected -- image blocks, structured-output schema,
adaptive thinking -- without a key or a live call. What this cannot check is
whether Claude judges the rollout well.
"""

import base64
import io
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from navflex_wm.critic import (  # noqa: E402
    ClaudeCritic,
    CriticError,
    VERDICT_SCHEMA,
    build_critic,
)

failures = []
seen = {'requests': []}
MODE = {'value': 'ok'}


def check(name, condition, detail=''):
    if condition:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        failures.append(name)


def jpeg(shade):
    from PIL import Image
    buffer = io.BytesIO()
    Image.new('RGB', (32, 24), (shade, 0, 0)).save(buffer, format='JPEG')
    return buffer.getvalue()


FRAMES = [jpeg(i * 20) for i in range(11)]


def _message(verdict, confidence, reason, hazards, stop_reason='end_turn'):
    return {
        'id': 'msg_stub', 'type': 'message', 'role': 'assistant',
        'model': 'claude-opus-4-8', 'stop_reason': stop_reason, 'stop_sequence': None,
        'content': [
            {'type': 'thinking', 'thinking': '', 'signature': 'sig'},
            {'type': 'text', 'text': json.dumps({
                'verdict': verdict, 'confidence': confidence,
                'reason': reason, 'hazards': hazards})},
        ],
        'usage': {'input_tokens': 10, 'output_tokens': 5},
    }


class StubAnthropic(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        # GET /v1/models/{id} -- what health() probes.
        seen['requests'].append({'GET': self.path})
        if MODE['value'] == 'bad_key':
            body = json.dumps({'type': 'error', 'error': {
                'type': 'authentication_error', 'message': 'invalid x-api-key'}}).encode()
            code = 401
        else:
            body = json.dumps({
                'id': self.path.rsplit('/', 1)[-1], 'type': 'model',
                'display_name': 'Claude Opus 4.8', 'created_at': '2026-01-01T00:00:00Z',
                'max_input_tokens': 1000000, 'max_tokens': 128000}).encode()
            code = 200
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        payload = json.loads(self.rfile.read(length).decode('utf-8'))
        seen['requests'].append(payload)
        mode = MODE['value']

        if mode == 'overloaded':
            body = json.dumps({'type': 'error', 'error': {
                'type': 'overloaded_error', 'message': 'overloaded'}}).encode()
            self.send_response(529)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        bodies = {
            'ok': _message('approve', 0.9, 'path is clear', []),
            'reject': _message('reject', 0.85, 'wall ahead', ['collision']),
            'refusal': _message('needs_confirmation', 0.0, '', [], stop_reason='refusal'),
            'truncated': _message('approve', 1.0, '', [], stop_reason='max_tokens'),
            'bogus': _message('looks_fine', 7.0, 'shrug', []),
        }
        body = json.dumps(bodies[mode]).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


server = ThreadingHTTPServer(('127.0.0.1', 8125), StubAnthropic)
threading.Thread(target=server.serve_forever, daemon=True).start()
time.sleep(0.3)
BASE = 'http://127.0.0.1:8125'


def critic(**kw):
    # max_retries=0: the SDK retries 529 twice by default, which would make the
    # failure test look like three requests and take ~2s of backoff.
    c = ClaudeCritic(api_key='test-key', base_url=BASE, timeout=10, **kw)
    c._client = c._client.with_options(max_retries=0)
    return c


print('== happy path ==')
MODE['value'] = 'ok'
seen['requests'].clear()
verdict = critic().critique(FRAMES, '去厨房', '20 路点，5.0 m')
check('verdict approve', verdict.verdict == 'approve', verdict.verdict)
check('confidence 0.9', abs(verdict.confidence - 0.9) < 1e-9)
check('source tagged claude', verdict.source == 'claude')

print('== request shape the SDK actually built ==')
request = seen['requests'][0]
content = request['messages'][0]['content']
images = [c for c in content if c['type'] == 'image']
texts = [c for c in content if c['type'] == 'text']

check('model defaults to claude-opus-4-8',
      request['model'] == 'claude-opus-4-8', request['model'])
check('system prompt sent top-level, not as a message',
      isinstance(request.get('system'), str) and '安全审查员' in request['system'])
check('adaptive thinking enabled',
      request['thinking'] == {'type': 'adaptive'}, request.get('thinking'))
check('no budget_tokens (removed on Opus 4.8)',
      'budget_tokens' not in json.dumps(request.get('thinking', {})))
check('no sampling params (rejected on Opus 4.8)',
      not any(k in request for k in ('temperature', 'top_p', 'top_k')))
check('effort inside output_config',
      request['output_config']['effort'] == 'medium', request.get('output_config'))
check('structured output pins the verdict schema',
      request['output_config']['format'] == {
          'type': 'json_schema', 'schema': VERDICT_SCHEMA})
check('schema forbids extra keys and requires all four',
      VERDICT_SCHEMA['additionalProperties'] is False
      and set(VERDICT_SCHEMA['required']) == {'verdict', 'confidence', 'reason', 'hazards'})
check('one text block', len(texts) == 1)
check('6 images (max_frames), not 11', len(images) == 6, len(images))
check('images are base64 jpeg blocks',
      all(i['source']['type'] == 'base64'
          and i['source']['media_type'] == 'image/jpeg'
          and base64.b64decode(i['source']['data'])[:2] == b'\xff\xd8'
          for i in images))
check('first and last frame retained',
      base64.b64decode(images[0]['source']['data']) == FRAMES[0]
      and base64.b64decode(images[-1]['source']['data']) == FRAMES[-1])
check('instruction reaches the prompt', '去厨房' in texts[0]['text'])
check('plan summary reaches the prompt', '5.0 m' in texts[0]['text'])
check('no assistant prefill (400s on Opus 4.8)',
      all(m['role'] != 'assistant' for m in request['messages']))

print('== effort is configurable ==')
MODE['value'] = 'ok'
seen['requests'].clear()
critic(effort='high').critique(FRAMES, 'x', 'y')
check("effort='high' forwarded",
      seen['requests'][0]['output_config']['effort'] == 'high')

print('== reject verdict ==')
MODE['value'] = 'reject'
verdict = critic().critique(FRAMES, 'x', 'y')
check('verdict reject', verdict.verdict == 'reject', verdict.verdict)
check('hazards parsed', verdict.hazards == ['collision'], verdict.hazards)

print('== a refusal is not an approval ==')
MODE['value'] = 'refusal'
verdict = critic().critique(FRAMES, 'x', 'y')
check('stop_reason=refusal -> needs_confirmation',
      verdict.verdict == 'needs_confirmation', verdict.verdict)
check('refusal has zero confidence', verdict.confidence == 0.0)

print('== truncation is an error, not a verdict ==')
MODE['value'] = 'truncated'
try:
    critic().critique(FRAMES, 'x', 'y')
    check('stop_reason=max_tokens raises', False, 'no exception')
except CriticError as exc:
    check('stop_reason=max_tokens raises CriticError', 'max_tokens' in str(exc))

print('== malformed verdict is clamped, never approved ==')
MODE['value'] = 'bogus'
verdict = critic().critique(FRAMES, 'x', 'y')
check('unknown verdict -> needs_confirmation',
      verdict.verdict == 'needs_confirmation', verdict.verdict)
check('confidence 7.0 clamped to 1.0', verdict.confidence == 1.0, verdict.confidence)

print('== API failures raise, never approve ==')
MODE['value'] = 'overloaded'
try:
    critic().critique(FRAMES, 'x', 'y')
    check('529 raises', False, 'no exception')
except CriticError as exc:
    check('529 raises CriticError', '529' in str(exc), str(exc)[:70])

print('== no evidence -> no approval, and no API call ==')
MODE['value'] = 'ok'
seen['requests'].clear()
verdict = critic().critique([], 'x', 'y')
check('no frames -> needs_confirmation', verdict.verdict == 'needs_confirmation')
check('no frames -> no API request', len(seen['requests']) == 0)

print('== health() probe: free, and it never raises ==')
MODE['value'] = 'ok'
seen['requests'].clear()
health = critic().health()
check('health reports ok', health.startswith('ok'), health)
check('health names the model', 'claude-opus-4-8' in health, health)
check('health probes GET /v1/models, not a billed POST',
      seen['requests'] and 'GET' in seen['requests'][0]
      and '/v1/models/claude-opus-4-8' in seen['requests'][0]['GET'],
      seen['requests'][:1])

MODE['value'] = 'bad_key'
health = critic().health()
check('bad key -> not ok, and no exception', not health.startswith('ok'), health)
check('bad key names the status', '401' in health, health)

print('== null critic health ==')
from navflex_wm.critic import NullCritic  # noqa: E402
check('null critic health says it never approves',
      'never approves' in NullCritic().health())

print('== proxy is passed to the SDK, not left to the environment ==')
import anthropic  # noqa: E402
proxied = ClaudeCritic(api_key='k', base_url=BASE, proxy='http://127.0.0.1:9', timeout=5)
http_client = proxied._client._client
check('proxy keeps the SDK DefaultHttpxClient',
      isinstance(http_client, anthropic.DefaultHttpxClient),
      type(http_client).__name__)
# DefaultHttpxClient(proxy=...) is silently ignored: it mounts transports built
# from the environment, and httpx prefers the more specific mount key. Assert
# the explicit proxy overrides both schemes, or a container with a stale
# https_proxy would quietly route the critic somewhere else.
mounts = {k.pattern: v for k, v in http_client._mounts.items()}
check('explicit proxy mounts https:// (beats any env entry)',
      mounts.get('https://') is not None, sorted(mounts))
check('explicit proxy mounts http://',
      mounts.get('http://') is not None, sorted(mounts))
check('the two schemes share one transport',
      mounts.get('https://') is mounts.get('http://'))

plain = ClaudeCritic(api_key='k', base_url=BASE, timeout=5)
check('no proxy -> SDK default client untouched',
      plain._client._client is not http_client)

print('== build_critic wiring ==')
os.environ.setdefault('ANTHROPIC_API_KEY', 'test-key')
check("build_critic('claude') selects ClaudeCritic",
      build_critic('claude', api_key='test-key').name == 'claude')
check("build_critic('anthropic') aliases to claude",
      build_critic('anthropic', api_key='test-key').name == 'claude')

server.shutdown()
print()
if failures:
    print(f'{len(failures)} FAILED: {failures}')
    sys.exit(1)
print('all checks passed')
