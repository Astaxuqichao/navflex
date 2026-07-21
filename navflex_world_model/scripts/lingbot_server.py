#!/usr/bin/env python3
"""Persistent LingBot-World-v2 inference sidecar.

Runs outside ROS, in the conda env that owns torch/flash-attn, and keeps the
pipeline warm: constructing ``WanI2VCausal`` costs ~100 s, so paying it per plan
is not an option.

Why a sidecar at all: the DiT is 18.5B parameters stored fp32 (74 GB on disk,
~37 GB in bf16), which does not fit a 32 GB RTX 5090. ``lingbot_loader`` builds
it under diffusers group offloading, streaming blocks in from host RAM as they
are needed -- the pipeline's own ``offload_model`` flag cannot do this and in
fact corrupts the offload hooks, so it stays off. Cost: ~90 s for a 21-frame
rollout. Fine for a pre-execution gate, useless inside a control loop.

    conda activate lingbot_wm
    python3 lingbot_server.py \
        --ckpt_dir /home/jiangbz/project/world_model/models/lingbot-world-v2-14b-causal-fast \
        --lingbot_repo /home/jiangbz/project/world_model/lingbot-world-v2
"""

import argparse
import base64
import io
import json
import logging
import os
import sys
import tempfile
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

LOG = logging.getLogger('lingbot_server')

# One GPU, one 37 GB model: rollouts are strictly serialized.
_PIPE_LOCK = threading.Lock()
_STATE = {'pipe': None, 'cfg': None, 'ready': False, 'error': '', 'calls': 0}


def _decode_npy(encoded: str) -> np.ndarray:
    return np.load(io.BytesIO(base64.b64decode(encoded)), allow_pickle=False)


def build_pipeline(args):
    """Construct WanI2VCausal once, with the DiT offloaded off the GPU.

    Not `wan.WanI2VCausal(...)` directly: its constructor moves all 37 GB of
    bf16 weights onto the device, which a 32 GB card cannot hold, and
    `offload_model` does not help (see navflex_wm/lingbot_loader.py).
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from navflex_wm.lingbot_loader import build_pipeline as build

    LOG.info('constructing WanI2VCausal (group offload; ~40 s)...')
    started = time.perf_counter()
    pipe, cfg = build(
        ckpt_dir=args.ckpt_dir,
        lingbot_repo=args.lingbot_repo,
        device_id=args.device_id,
        blocks_per_group=args.blocks_per_group,
        local_attn_size=args.local_attn_size,
        sink_size=args.sink_size,
        use_stream=False,
        task=args.task,
    )
    LOG.info('pipeline ready in %.1f s', time.perf_counter() - started)
    return pipe, cfg


def imagine(args, payload):
    from wan.configs import MAX_AREA_CONFIGS
    from wan.utils.utils import save_video
    from PIL import Image

    image = Image.open(io.BytesIO(base64.b64decode(payload['image_b64']))).convert('RGB')
    poses = _decode_npy(payload['poses_npy_b64']).astype(np.float32)
    intrinsics = _decode_npy(payload['intrinsics_npy_b64']).astype(np.float32)
    prompt = payload.get('prompt') or 'A ground robot drives forward, ego-view camera.'
    frame_num = int(payload.get('frame_num') or len(poses))
    stride = max(1, int(payload.get('sample_stride') or 8))

    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError(f'poses must be (T,4,4), got {poses.shape}')
    if intrinsics.ndim != 2 or intrinsics.shape[1] != 4:
        raise ValueError(f'intrinsics must be (T,4), got {intrinsics.shape}')

    # WanI2VCausal reads poses.npy / intrinsics.npy from a directory.
    with tempfile.TemporaryDirectory(prefix='lingbot_action_') as action_path:
        np.save(os.path.join(action_path, 'poses.npy'), poses)
        np.save(os.path.join(action_path, 'intrinsics.npy'), intrinsics)

        started = time.perf_counter()
        with _PIPE_LOCK:
            video = _STATE['pipe'].generate(
                prompt,
                image,
                action_path=action_path,
                chunk_size=args.chunk_size,
                max_area=MAX_AREA_CONFIGS[args.size],
                frame_num=frame_num,
                shift=_STATE['cfg'].sample_shift,
                seed=args.seed,
                # Group offloading owns residency. offload_model=True would end
                # with self.model.cpu(), which bypasses the loader's .to() guard
                # via Module._apply and corrupts the offload hooks.
                offload_model=False,
                max_attention_size=args.max_attention_size,
            )
            _STATE['calls'] += 1
        elapsed = time.perf_counter() - started

    video_path = ''
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        video_path = os.path.join(
            args.save_dir, f'rollout_{_STATE["calls"]:05d}.mp4')
        save_video(tensor=video[None], save_file=video_path,
                   fps=_STATE['cfg'].sample_fps, nrow=1,
                   normalize=True, value_range=(-1, 1))

    frames_b64 = _sample_frames(video, stride)
    LOG.info('rollout: %d frames in %.1f s -> %d sampled',
             video.shape[1], elapsed, len(frames_b64))
    return {
        'frames_b64': frames_b64,
        'video_path': video_path,
        'message': f'{video.shape[1]} frames in {elapsed:.1f}s',
    }


def _sample_frames(video, stride):
    """Take every ``stride``-th frame as JPEG. ``video`` is (C, T, H, W) in [-1, 1]."""
    from PIL import Image

    array = video.detach().float().cpu().numpy()
    array = np.clip((array + 1.0) * 127.5, 0, 255).astype(np.uint8)
    array = np.transpose(array, (1, 2, 3, 0))  # -> (T, H, W, C)

    encoded = []
    for index in range(0, len(array), stride):
        buffer = io.BytesIO()
        Image.fromarray(array[index]).save(buffer, format='JPEG', quality=88)
        encoded.append(base64.b64encode(buffer.getvalue()).decode('ascii'))
    return encoded


class Handler(BaseHTTPRequestHandler):
    args = None

    def log_message(self, fmt, *fargs):
        LOG.debug(fmt, *fargs)

    def _reply(self, code, body):
        payload = json.dumps(body).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path.rstrip('/') != '/health':
            return self._reply(404, {'error': 'not found'})
        status = 'ready' if _STATE['ready'] else f"loading/failed: {_STATE['error'] or '...'}"
        self._reply(200, {'status': status, 'calls': _STATE['calls']})

    def do_POST(self):
        if self.path.rstrip('/') != '/imagine':
            return self._reply(404, {'error': 'not found'})
        if not _STATE['ready']:
            return self._reply(503, {'error': f"pipeline not ready: {_STATE['error']}"})
        try:
            length = int(self.headers.get('Content-Length', 0))
            payload = json.loads(self.rfile.read(length).decode('utf-8'))
            self._reply(200, imagine(self.args, payload))
        except Exception as exc:
            LOG.error('imagine failed:\n%s', traceback.format_exc())
            self._reply(500, {'error': str(exc)})


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--ckpt_dir', required=True)
    parser.add_argument('--lingbot_repo', required=True)
    parser.add_argument('--task', default='i2v-A14B')
    parser.add_argument('--size', default='480*832')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8100)
    parser.add_argument('--device_id', type=int, default=0)
    parser.add_argument('--chunk_size', type=int, default=4)
    parser.add_argument('--blocks_per_group', type=int, default=1,
                        help='DiT blocks onloaded to the GPU at a time')
    # The KV cache is preallocated as frame_seqlen * local_attn_size * 40 layers.
    # The upstream default of 18 costs ~23 GB at 480x832 -- fine sharded across
    # 8xH100 with ulysses, fatal on one card. 8 keeps it near 10 GB and leaves
    # headroom for activations and the VAE decode.
    #
    # -1 sizes the cache to the rollout's own latent frames instead. When
    # lat_f <= the window the sliding window never saturates and sink tokens are
    # inert, so the two settings produce *bit-identical* video -- verified. The
    # gate's rollouts cap at 29 frames (lat_f=8), so 8 and -1 agree there. Past
    # that -1 grows: a 61-frame rollout (lat_f=16) OOMs on a 32 GB card.
    parser.add_argument('--local_attn_size', type=int, default=8)
    parser.add_argument('--sink_size', type=int, default=6)
    parser.add_argument('--max_attention_size', type=int, default=32760)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--save_dir', default='/tmp/navflex_rollouts')
    # BooleanOptionalAction, not store_true: these default on for a 32 GB card
    # but must be switchable off on a GPU that can hold the model outright.
    parser.add_argument('--t5_cpu', action=argparse.BooleanOptionalAction, default=True,
                        help='keep umT5 on CPU; it runs once per plan and the '
                             'GPU budget belongs to the DiT')
    return parser.parse_args()


def main():
    logging.basicConfig(
        level=logging.INFO, format='[%(asctime)s] %(levelname)s %(message)s')
    args = parse_args()
    Handler.args = args

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    # Serve /health while the weights load, so the ROS gate can report why it is
    # unavailable instead of hanging on connect.
    threading.Thread(target=server.serve_forever, daemon=True).start()
    LOG.info('listening on http://%s:%d', args.host, args.port)

    try:
        _STATE['pipe'], _STATE['cfg'] = build_pipeline(args)
        _STATE['ready'] = True
    except Exception as exc:
        _STATE['error'] = str(exc)
        LOG.error('pipeline construction failed:\n%s', traceback.format_exc())

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        LOG.info('shutting down')
        server.shutdown()


if __name__ == '__main__':
    main()
