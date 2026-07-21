#!/usr/bin/env python3
"""Exercise the OpenAI-compatible critic against a stub /v1/chat/completions.

Aggregators differ in how much of the OpenAI schema they implement, so the
interesting behaviour is the degradation: json_schema -> json_object -> plain
prompting, and whether a provider that ignores all three can smuggle an
approval past the gate. (It cannot: Verdict clamps.)
"""

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
    CriticError,
    OpenAICompatCritic,
    VERDICT_SCHEMA,
    build_critic,
)

failures = []
seen = {'requests': []}
MODE = {'value': 'schema_ok'}


def check(name, condition, detail=''):
    if condition:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        failures.append(name)


def jpeg(shade):
    from PIL import Image
    buf = io.BytesIO()
    Image.new('RGB', (32, 24), (shade, 0, 0)).save(buf, format='JPEG')
    return buf.getvalue()


FRAMES = [jpeg(i * 20) for i in range(11)]


def completion(text):
    return {'id': 'c', 'object': 'chat.completion',
            'choices': [{'index': 0, 'message': {'role': 'assistant', 'content': text},
                         'finish_reason': 'stop'}]}


VERDICT_JSON = json.dumps({'verdict': 'approve', 'confidence': 0.8,
                           'reason': 'clear', 'hazards': []})


class Stub(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _reply(self, code, body):
        raw = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        seen['requests'].append({'GET': self.path})
        self._reply(200, {'object': 'list', 'data': [
            {'id': 'glm-5.2'}, {'id': 'qwen3.5-397b-a17b'}]})

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        payload = json.loads(self.rfile.read(n).decode())
        seen['requests'].append(payload)
        fmt = (payload.get('response_format') or {}).get('type')
        mode = MODE['value']

        if mode == 'schema_ok':
            return self._reply(200, completion(VERDICT_JSON))
        if mode == 'no_schema':  # rejects json_schema, accepts json_object
            if fmt == 'json_schema':
                return self._reply(400, {'error': {'message': 'response_format not supported'}})
            return self._reply(200, completion(VERDICT_JSON))
        if mode == 'no_format':  # rejects every response_format
            if fmt:
                return self._reply(400, {'error': {'message': 'response_format not supported'}})
            return self._reply(200, completion('```json\n' + VERDICT_JSON + '\n```'))
        if mode == 'ignores_schema':  # 200s, but replies with garbage
            return self._reply(200, completion(json.dumps(
                {'verdict': 'definitely fine', 'confidence': 9, 'reason': 'trust me',
                 'hazards': []})))
        if mode == 'always_500':
            return self._reply(500, {'error': {'message': 'boom'}})
        if mode == 'empty':
            return self._reply(200, {'choices': [{'message': {'content': ''}}]})
        # Seen in the wild on api.luchentech.com: 200 OK, message is null.
        if mode == 'null_message':
            return self._reply(200, {'choices': [{'message': None}]})
        if mode == 'no_choices':
            return self._reply(200, {'id': 'c'})
        # A reasoning model that spent its whole budget thinking.
        if mode == 'reasoning_only':
            return self._reply(200, {'choices': [{'message': {
                'content': '', 'reasoning_content': 'hmm...'}}]})
        # 200 OK, but prose instead of JSON on every attempt.
        if mode == 'prose':
            return self._reply(200, completion('I think it looks fine, honestly.'))


server = ThreadingHTTPServer(('127.0.0.1', 8126), Stub)
threading.Thread(target=server.serve_forever, daemon=True).start()
time.sleep(0.3)
BASE = 'http://127.0.0.1:8126/v1'


def critic(model='glm-5.2', **kw):
    return OpenAICompatCritic(BASE, model, api_key='k', timeout=10, **kw)


print('== happy path: provider honours json_schema ==')
MODE['value'] = 'schema_ok'
seen['requests'].clear()
v = critic().critique(FRAMES, '去厨房', '20 路点，5.0 m')
check('verdict approve', v.verdict == 'approve', v.verdict)
check('source tagged', v.source == 'openai_compat')
check('one request only (no needless retries)',
      len(seen['requests']) == 1, len(seen['requests']))

print('== request shape ==')
req = seen['requests'][0]
content = req['messages'][1]['content']
images = [c for c in content if c['type'] == 'image_url']
texts = [c for c in content if c['type'] == 'text']
check('model forwarded', req['model'] == 'glm-5.2')
check('system prompt is a system message', req['messages'][0]['role'] == 'system')
check('6 images, not 11', len(images) == 6, len(images))
check('images are data URIs',
      all(i['image_url']['url'].startswith('data:image/jpeg;base64,') for i in images))
check('json_schema requested first',
      req['response_format']['type'] == 'json_schema')
check('schema is the verdict schema',
      req['response_format']['json_schema']['schema'] == VERDICT_SCHEMA)
check('instruction reaches the prompt', '去厨房' in texts[0]['text'])

print('== degradation: json_schema -> json_object ==')
MODE['value'] = 'no_schema'
seen['requests'].clear()
v = critic().critique(FRAMES, 'x', 'y')
posts = [r for r in seen['requests'] if 'messages' in r]
check('two attempts', len(posts) == 2, len(posts))
check('second attempt uses json_object',
      posts[1]['response_format']['type'] == 'json_object')
check('verdict survives degradation', v.verdict == 'approve')

print('== degradation: -> plain prompting, markdown fence tolerated ==')
MODE['value'] = 'no_format'
seen['requests'].clear()
v = critic().critique(FRAMES, 'x', 'y')
posts = [r for r in seen['requests'] if 'messages' in r]
check('three attempts', len(posts) == 3, len(posts))
check('last attempt drops response_format', 'response_format' not in posts[2])
check('last attempt hardens the system prompt',
      '不要输出 Markdown' in posts[2]['messages'][0]['content'])
check('markdown-fenced JSON parsed', v.verdict == 'approve')

print('== a provider that ignores the schema cannot smuggle an approval ==')
MODE['value'] = 'ignores_schema'
v = critic().critique(FRAMES, 'x', 'y')
check("'definitely fine' -> needs_confirmation",
      v.verdict == 'needs_confirmation', v.verdict)
check('confidence 9 clamped to 1.0', v.confidence == 1.0, v.confidence)

print('== failures raise, never approve ==')
MODE['value'] = 'always_500'
try:
    critic().critique(FRAMES, 'x', 'y')
    check('500 raises', False, 'no exception')
except CriticError as exc:
    check('500 raises CriticError', '500' in str(exc), str(exc)[:60])

MODE['value'] = 'empty'
try:
    critic().critique(FRAMES, 'x', 'y')
    check('empty content raises', False, 'no exception')
except CriticError:
    check('empty content raises CriticError', True)

print('== malformed provider responses degrade, they do not crash the node ==')
for mode, label in (('null_message', 'message: null (seen on luchentech)'),
                    ('no_choices', 'no choices key'),
                    ('reasoning_only', 'reasoning_content only, content empty'),
                    ('prose', 'prose instead of JSON on every attempt')):
    MODE['value'] = mode
    try:
        critic().critique(FRAMES, 'x', 'y')
        check(f'{label} raises', False, 'no exception')
    except CriticError:
        check(f'{label} -> CriticError, not TypeError', True)
    except Exception as exc:  # noqa: BLE001
        check(f'{label} -> CriticError, not TypeError', False, type(exc).__name__)

print('== no evidence -> no approval, no request ==')
MODE['value'] = 'schema_ok'
seen['requests'].clear()
v = critic().critique([], 'x', 'y')
check('no frames -> needs_confirmation', v.verdict == 'needs_confirmation')
check('no frames -> no request', len(seen['requests']) == 0)

print('== health() checks the model is actually offered ==')
health = critic('glm-5.2').health()
check('listed model -> ok', health.startswith('ok'), health)
missing = critic('gpt-nonexistent').health()
check('unlisted model -> unavailable', missing.startswith('unavailable'), missing)
check('unlisted model says so', 'not in /models' in missing, missing)

print('== build_critic wiring / required args ==')
check("build_critic('openai_compat')",
      build_critic('openai_compat', base_url=BASE, model='glm-5.2',
                   api_key='k').name == 'openai_compat')
for missing_arg, kwargs in (('base_url', {'model': 'm'}), ('model', {'base_url': BASE})):
    try:
        build_critic('openai_compat', **kwargs)
        check(f'missing {missing_arg} raises', False, 'no exception')
    except CriticError:
        check(f'missing {missing_arg} raises CriticError', True)

server.shutdown()
print()
if failures:
    print(f'{len(failures)} FAILED: {failures}')
    sys.exit(1)
print('all checks passed')
