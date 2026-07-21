#!/usr/bin/env python3
"""Exercise the critic against a stub Responses API.

The one thing this cannot check is whether the model judges well. Everything
else -- request shape, frame sampling, response parsing, the strict-schema
retry, and the clamping that stops a malformed verdict from becoming an
approval -- is checkable without a key, and is where the bugs live.
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
    NullCritic,
    OpenAICritic,
    build_critic,
    even_sample,
)

failures = []
seen = {'requests': []}


def check(name, condition, detail=''):
    if condition:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        failures.append(name)


def jpeg(colour):
    from PIL import Image
    buffer = io.BytesIO()
    Image.new('RGB', (32, 24), colour).save(buffer, format='JPEG')
    return buffer.getvalue()


FRAMES = [jpeg((i * 20, 0, 0)) for i in range(11)]

# Behaviour of the stub, flipped per test.
MODE = {'value': 'ok'}


class StubOpenAI(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        payload = json.loads(self.rfile.read(length).decode('utf-8'))
        seen['requests'].append(payload)
        mode = MODE['value']

        if mode == 'reject_strict_schema' and 'text' in payload:
            self.send_response(400)
            body = b'{"error": "json_schema not supported"}'
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if mode == 'always_500':
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b'boom')
            return

        bodies = {
            'ok': {'output_text': json.dumps({
                'verdict': 'approve', 'confidence': 0.9,
                'reason': 'path is clear', 'hazards': []})},
            'reject_strict_schema': {'output_text': json.dumps({
                'verdict': 'reject', 'confidence': 0.8,
                'reason': 'wall ahead', 'hazards': ['collision']})},
            'nested_output': {'output': [{'content': [{'text': json.dumps({
                'verdict': 'reject', 'confidence': 0.7,
                'reason': 'stairs', 'hazards': ['fall']})}]}]},
            'markdown': {'output_text': '```json\n' + json.dumps({
                'verdict': 'needs_confirmation', 'confidence': 0.4,
                'reason': 'blurry', 'hazards': []}) + '\n```'},
            'bogus_verdict': {'output_text': json.dumps({
                'verdict': 'looks_fine_to_me', 'confidence': 5.0,
                'reason': 'shrug', 'hazards': []})},
            'no_text': {'output': []},
        }
        body = json.dumps(bodies[mode]).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


server = ThreadingHTTPServer(('127.0.0.1', 8124), StubOpenAI)
threading.Thread(target=server.serve_forever, daemon=True).start()
time.sleep(0.3)
URL = 'http://127.0.0.1:8124/v1/responses'


def critic(max_frames=6):
    return OpenAICritic(api_url=URL, api_key='test-key', model='stub',
                        timeout=10, max_frames=max_frames)


print('== even_sample ==')
check('keeps endpoints', even_sample(FRAMES, 3) == [FRAMES[0], FRAMES[5], FRAMES[10]])
check('count >= len returns all', even_sample(FRAMES, 50) == FRAMES)
check('count == 1 does not divide by zero', even_sample(FRAMES, 1) == [FRAMES[-1]])
check('count == 0 returns nothing', even_sample(FRAMES, 0) == [])

print('== happy path ==')
MODE['value'] = 'ok'
seen['requests'].clear()
verdict = critic().critique(FRAMES, '去厨房', '20 路点，5.0 m')
check('verdict approve', verdict.verdict == 'approve', verdict.verdict)
check('confidence 0.9', abs(verdict.confidence - 0.9) < 1e-9)
check('source tagged', verdict.source == 'openai')

print('== request shape ==')
request = seen['requests'][0]
content = request['input'][0]['content']
images = [c for c in content if c['type'] == 'input_image']
texts = [c for c in content if c['type'] == 'input_text']
check('one text block', len(texts) == 1)
check('6 images (max_frames), not 11', len(images) == 6, len(images))
check('images are jpeg data URIs',
      all(c['image_url'].startswith('data:image/jpeg;base64,') for c in images))
check('instruction reaches the prompt', '去厨房' in texts[0]['text'])
check('plan summary reaches the prompt', '5.0 m' in texts[0]['text'])
check('strict json_schema requested',
      request['text']['format']['type'] == 'json_schema'
      and request['text']['format']['strict'] is True)

print('== strict schema rejected -> retry without it ==')
MODE['value'] = 'reject_strict_schema'
seen['requests'].clear()
verdict = critic().critique(FRAMES, 'x', 'y')
check('two requests sent', len(seen['requests']) == 2, len(seen['requests']))
check('retry dropped the schema', 'text' not in seen['requests'][1])
check('verdict survives the retry', verdict.verdict == 'reject', verdict.verdict)

print('== response parsing ==')
MODE['value'] = 'nested_output'
check('output[].content[].text form', critic().critique(FRAMES, 'x', 'y').verdict == 'reject')
MODE['value'] = 'markdown'
check('markdown-fenced JSON', critic().critique(FRAMES, 'x', 'y').verdict == 'needs_confirmation')

print('== a malformed verdict must never become an approval ==')
MODE['value'] = 'bogus_verdict'
verdict = critic().critique(FRAMES, 'x', 'y')
check('unknown verdict -> needs_confirmation', verdict.verdict == 'needs_confirmation', verdict.verdict)
check('confidence 5.0 clamped to 1.0', verdict.confidence == 1.0, verdict.confidence)

print('== failures raise, never approve ==')
MODE['value'] = 'always_500'
try:
    critic().critique(FRAMES, 'x', 'y')
    check('HTTP 500 raises', False, 'no exception')
except CriticError as exc:
    check('HTTP 500 raises CriticError', '500' in str(exc), str(exc)[:60])

MODE['value'] = 'no_text'
try:
    critic().critique(FRAMES, 'x', 'y')
    check('empty output raises', False, 'no exception')
except CriticError:
    check('empty output raises CriticError', True)

print('== no evidence -> no approval, and no API call ==')
MODE['value'] = 'ok'
seen['requests'].clear()
verdict = critic().critique([], 'x', 'y')
check('no frames -> needs_confirmation', verdict.verdict == 'needs_confirmation')
check('no frames -> no HTTP request', len(seen['requests']) == 0, len(seen['requests']))

print('== null critic ==')
null = NullCritic().critique(FRAMES, 'x', 'y')
check('null critic never approves', null.verdict == 'needs_confirmation')
check('null critic has zero confidence', null.confidence == 0.0)

print('== build_critic ==')
try:
    build_critic('openai', api_key='')
    check('openai without key raises', False, 'no exception')
except CriticError:
    check('openai without key raises CriticError', True)
check('null is the default', build_critic('').name == 'null')
try:
    build_critic('telepathy')
    check('unknown critic raises', False, 'no exception')
except CriticError:
    check('unknown critic raises CriticError', True)

server.shutdown()
print()
print('== verdict_from_json: the shapes endpoints actually return ==')
from navflex_wm.critic import verdict_from_json  # noqa: E402

# What moonshotai/kimi-k2.5 returns on api.luchentech.com once the endpoint
# quietly drops from json_schema to json_object. Read literally, the gate
# blocked a clear path while printing an approving reason.
real = verdict_from_json(
    {'decision': 'approve', 'reason': '画面显示前方通路畅通，地板平整无跌落风险。'}, 'openai_compat')
check("'decision' is accepted as a verdict", real.verdict == 'approve', real.verdict)
check('a missing confidence is not invented', real.confidence_reported is False)
check('...and defaults to 0.0 without claiming it', real.confidence == 0.0)
check('the reason survives', '通路畅通' in real.reason)

check('schema-shaped replies still work',
      verdict_from_json({'verdict': 'reject', 'confidence': 0.9, 'reason': 'x',
                         'hazards': ['collision']}, 's').verdict == 'reject')
schema = verdict_from_json({'verdict': 'approve', 'confidence': 0.95, 'reason': 'x',
                            'hazards': []}, 's')
check('a reported confidence is marked reported', schema.confidence_reported is True)
check('...and preserved', schema.confidence == 0.95)

for word, want in (('approved', 'approve'), ('ACCEPT', 'approve'), ('safe', 'approve'),
                   ('rejected', 'reject'), ('unsafe', 'reject'), ('block', 'reject'),
                   ('needs confirmation', 'needs_confirmation'),
                   ('needs-confirmation', 'needs_confirmation')):
    check(f'{word!r} -> {want}',
          verdict_from_json({'decision': word, 'reason': ''}, 's').verdict == want)

# Fail closed, and say why -- an operator must be able to tell a cautious critic
# from a broken one.
odd = verdict_from_json({'decision': 'maybe', 'reason': 'r'}, 's')
check('an unrecognised verdict fails closed',
      odd.verdict == 'needs_confirmation', odd.verdict)
check('...and says the verdict was unrecognised', 'unrecognised' in odd.reason)
none = verdict_from_json({'reason': 'r'}, 's')
check('a missing verdict field fails closed', none.verdict == 'needs_confirmation')
check('...and names the keys it did get', 'reason' in none.reason)

check('hazards accept the risks alias',
      verdict_from_json({'decision': 'reject', 'risks': ['step']}, 's').hazards == ['step'])
check('a non-list hazards field does not crash',
      verdict_from_json({'decision': 'approve', 'hazards': 'none'}, 's').hazards == [])
check('confidence out of range is clamped',
      verdict_from_json({'verdict': 'approve', 'confidence': 3.0}, 's').confidence == 1.0)
check('a string confidence is treated as unreported',
      verdict_from_json({'verdict': 'approve', 'confidence': 'high'}, 's')
      .confidence_reported is False)

print()
if failures:
    print(f'{len(failures)} FAILED: {failures}')
    sys.exit(1)
print('all checks passed')
