#!/usr/bin/env python3
"""Exercise the LingBotHttpBackend <-> lingbot_server contract without the model.

Stubs out the 18.5B pipeline with a tensor of the right shape, then drives a real
HTTP round trip: npy encode -> POST /imagine -> generate -> frame sampling ->
base64 JPEG -> decode. This is where the two sides can silently disagree about
dtypes, axis order and array shapes; the weights add nothing to that question.
"""

import os
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

failures = []


def check(name, condition, detail=''):
    if condition:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        failures.append(name)


# --- stub the heavy imports the sidecar pulls in lazily -----------------------
import torch  # noqa: E402
from PIL import Image  # noqa: E402

import lingbot_server  # noqa: E402
from navflex_wm.backends import LingBotHttpBackend, WorldModelError  # noqa: E402
from navflex_wm.pose_utils import (  # noqa: E402
    CameraExtrinsic,
    plan_to_control_signal,
    quantize_frame_num,
)

FRAMES_OUT = 21  # what the stub "generates"
HEIGHT, WIDTH = 480, 832

captured = {}


class StubPipe:
    def generate(self, prompt, image, action_path, **kwargs):
        # Read back exactly what the sidecar wrote, so the on-disk contract with
        # wan/image2video.py (poses.npy + intrinsics.npy) is exercised too.
        captured['prompt'] = prompt
        captured['image_size'] = image.size
        captured['poses'] = np.load(os.path.join(action_path, 'poses.npy'))
        captured['intrinsics'] = np.load(os.path.join(action_path, 'intrinsics.npy'))
        captured['frame_num'] = kwargs.get('frame_num')
        captured['offload_model'] = kwargs.get('offload_model')
        # WanI2VCausal returns (C, T, H, W) in [-1, 1].
        return torch.zeros(3, FRAMES_OUT, HEIGHT, WIDTH).uniform_(-1.0, 1.0)


def fake_save_video(tensor, save_file, **kwargs):
    with open(save_file, 'wb') as handle:
        handle.write(b'fake mp4')


# lingbot_server.imagine() imports these from the lingbot repo at call time.
fake_configs = types.ModuleType('wan.configs')
fake_configs.MAX_AREA_CONFIGS = {'480*832': HEIGHT * WIDTH}
fake_utils = types.ModuleType('wan.utils.utils')
fake_utils.save_video = fake_save_video
fake_wan = types.ModuleType('wan')
fake_wan_utils = types.ModuleType('wan.utils')
sys.modules.setdefault('wan', fake_wan)
sys.modules.setdefault('wan.utils', fake_wan_utils)
sys.modules['wan.configs'] = fake_configs
sys.modules['wan.utils.utils'] = fake_utils

cfg = types.SimpleNamespace(sample_shift=5.0, sample_fps=16)
lingbot_server._STATE['pipe'] = StubPipe()
lingbot_server._STATE['cfg'] = cfg
lingbot_server._STATE['ready'] = True

args = types.SimpleNamespace(
    size='480*832', chunk_size=4, seed=42, offload_model=True,
    max_attention_size=32760, save_dir='/tmp/navflex_rollouts_test',
    host='127.0.0.1', port=8123)
lingbot_server.Handler.args = args

from http.server import ThreadingHTTPServer  # noqa: E402

server = ThreadingHTTPServer(('127.0.0.1', args.port), lingbot_server.Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
time.sleep(0.3)

backend = LingBotHttpBackend(f'http://127.0.0.1:{args.port}', timeout=60, sample_stride=4)

print('== health ==')
check('sidecar reports ready', backend.health() == 'ready', backend.health())

print('== round trip ==')
import io  # noqa: E402
buffer = io.BytesIO()
Image.new('RGB', (640, 360), (10, 20, 30)).save(buffer, format='JPEG')
image_jpeg = buffer.getvalue()

c2ws, intrinsics = plan_to_control_signal(
    [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]], 81,
    CameraExtrinsic(x=0.29, z=0.01, pitch=np.deg2rad(15)), 90.0, 1920, 1080)

rollout = backend.imagine(image_jpeg, c2ws, intrinsics, 'a robot drives forward')

check('frames returned', rollout.frame_count > 0, rollout.frame_count)
check('sample_stride=4 over 21 frames -> 6 frames',
      rollout.frame_count == 6, rollout.frame_count)
check('frames decode as JPEG',
      all(Image.open(io.BytesIO(f)).size == (WIDTH, HEIGHT) for f in rollout.frames))
check('video path returned', rollout.video_path.endswith('.mp4'), rollout.video_path)

print('== the sidecar wrote what wan/image2video.py expects ==')
# 81 is off the renderable grid, so plan_to_control_signal yields 77 poses.
check('poses.npy is (77,4,4) float32',
      captured['poses'].shape == (77, 4, 4) and captured['poses'].dtype == np.float32,
      (captured['poses'].shape, captured['poses'].dtype))
check('intrinsics.npy is (77,4) float32',
      captured['intrinsics'].shape == (77, 4) and captured['intrinsics'].dtype == np.float32,
      (captured['intrinsics'].shape, captured['intrinsics'].dtype))
check('poses survive the base64 npy hop bit-exact',
      np.array_equal(captured['poses'], c2ws))
check('intrinsics survive bit-exact', np.array_equal(captured['intrinsics'], intrinsics))
# The invariant that matters: one pose per rendered frame. generate() truncates
# c2ws to whatever it can render, so a mismatch here means the tail of the plan
# is dropped in silence -- the gate would vet a path it never imagined.
check('frame_num matches the pose count exactly',
      captured['frame_num'] == len(captured['poses']),
      (captured['frame_num'], len(captured['poses'])))
check('frame_num is on the renderable grid',
      captured['frame_num'] == quantize_frame_num(captured['frame_num']),
      captured['frame_num'])
# Group offloading owns residency now. offload_model=True would end generate()
# with self.model.cpu(), which reaches the params through Module._apply rather
# than .to(), slipping past the loader's guard and corrupting the hooks.
check('offload_model stays off (group offloading owns residency)',
      captured['offload_model'] is False, captured['offload_model'])
check('prompt forwarded', captured['prompt'] == 'a robot drives forward')
check('conditioning image decoded RGB', captured['image_size'] == (640, 360))

print('== failure surfaces as WorldModelError, not a silent approve ==')


class EmptyPipe:
    def generate(self, *a, **k):
        return torch.zeros(3, 0, HEIGHT, WIDTH)


lingbot_server._STATE['pipe'] = EmptyPipe()
try:
    backend.imagine(image_jpeg, c2ws, intrinsics, 'x')
    check('empty rollout raises', False, 'no exception')
except WorldModelError:
    check('empty rollout raises WorldModelError', True)

lingbot_server._STATE['ready'] = False
try:
    backend.imagine(image_jpeg, c2ws, intrinsics, 'x')
    check('not-ready sidecar raises', False, 'no exception')
except WorldModelError as exc:
    check('not-ready sidecar raises WorldModelError', '503' in str(exc), str(exc)[:80])

server.shutdown()
print()
if failures:
    print(f'{len(failures)} FAILED: {failures}')
    sys.exit(1)
print('all checks passed')
