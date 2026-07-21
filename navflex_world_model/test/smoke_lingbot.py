#!/usr/bin/env python3
"""First real rollout: load LingBot-World-v2 and imagine one nav2 plan.

Run inside the sidecar's env, after the weights land:

    conda activate lingbot_wm
    python3 test/smoke_lingbot.py --ckpt_dir ... --lingbot_repo ...

Answers the questions the null backend cannot:
  - do 18.5B fp32 weights load into 32 GB of VRAM with offload,
  - does a camera trajectory built from a nav2 plan drive the model,
  - how long does one rollout actually take on this box.

Writes the mp4 so the imagined future can be watched, not just measured.
"""

import argparse
import io
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from navflex_wm.pose_utils import CameraExtrinsic, plan_to_control_signal  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--ckpt_dir',
        default='/home/jiangbz/project/world_model/models/lingbot-world-v2-14b-causal-fast')
    parser.add_argument(
        '--lingbot_repo', default='/home/jiangbz/project/world_model/lingbot-world-v2')
    parser.add_argument('--image', default='', help='conditioning frame; a LingBot example if unset')
    parser.add_argument('--frame_num', type=int, default=21,
                        help='keep small for the first run; must be ((n-1)//4)*4+1')
    parser.add_argument('--size', default='480*832')
    parser.add_argument('--chunk_size', type=int, default=4)
    parser.add_argument('--local_attn_size', type=int, default=18)
    parser.add_argument('--sink_size', type=int, default=6)
    parser.add_argument('--blocks_per_group', type=int, default=2,
                        help='DiT blocks resident on the GPU per offload group')
    parser.add_argument('--use_stream', action='store_true',
                        help='prefetch groups on a side stream; pins a second '
                             'copy of the weights in host memory (OOMs at 60 GB)')
    parser.add_argument('--offload_to_disk', default='',
                        help='page offloaded weights from this directory instead '
                             'of host RAM')
    parser.add_argument('--out', default='/tmp/navflex_rollouts/smoke.mp4')
    return parser.parse_args()


def _watch_memory(stop):
    """Report RAM every few seconds: an OOM kill leaves no traceback."""
    import threading
    import time as _time

    def loop():
        while not stop.is_set():
            try:
                with open('/proc/meminfo') as fh:
                    info = {k: int(v.split()[0]) for k, v in
                            (line.split(':') for line in fh)}
                avail = info['MemAvailable'] / 2**20
                swap = (info['SwapTotal'] - info['SwapFree']) / 2**20
                print(f'   [mem] RAM available {avail:.1f} GiB, swap used {swap:.1f} GiB',
                      flush=True)
            except OSError:
                pass
            stop.wait(15)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    return thread


def main():
    args = parse_args()
    sys.path.insert(0, args.lingbot_repo)

    import torch
    from PIL import Image

    from wan.configs import MAX_AREA_CONFIGS
    from wan.utils.utils import save_video

    from navflex_wm.lingbot_loader import build_pipeline, vram_report

    image_path = args.image or os.path.join(args.lingbot_repo, 'examples', '03', 'image.jpg')
    image = Image.open(image_path).convert('RGB')
    print(f'conditioning frame: {image_path} {image.size}')

    # A plan the robot could plausibly have been handed: 4 m ahead, veering left.
    plan = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.5, 1.0, 0.0], [4.0, 2.0, 0.0]]
    extrinsic = CameraExtrinsic(x=0.29, y=0.0, z=0.01, pitch=math.radians(15.0))
    c2ws, intrinsics = plan_to_control_signal(
        plan, args.frame_num, extrinsic, 90.0, 1920, 1080, final_yaw=math.pi / 2)
    print(f'camera trajectory: {c2ws.shape}, intrinsics {intrinsics[0]}')

    action_path = '/tmp/navflex_smoke_action'
    os.makedirs(action_path, exist_ok=True)
    np.save(os.path.join(action_path, 'poses.npy'), c2ws)
    np.save(os.path.join(action_path, 'intrinsics.npy'), intrinsics)

    import threading
    stop = threading.Event()
    _watch_memory(stop)

    print('constructing WanI2VCausal (bf16, block-level group offload)...')
    t0 = time.perf_counter()
    pipe, cfg = build_pipeline(
        ckpt_dir=args.ckpt_dir,
        lingbot_repo=args.lingbot_repo,
        blocks_per_group=args.blocks_per_group,
        local_attn_size=args.local_attn_size,
        sink_size=args.sink_size,
        use_stream=args.use_stream,
        offload_to_disk_path=args.offload_to_disk or None,
    )
    print(f'construct: {time.perf_counter() - t0:.1f} s')
    print(f'after load: {vram_report()}')

    print('generating...')
    t1 = time.perf_counter()
    video = pipe.generate(
        'A ground robot drives forward through an indoor environment, ego-view camera, steady motion.',
        image,
        action_path=action_path,
        chunk_size=args.chunk_size,
        max_area=MAX_AREA_CONFIGS[args.size],
        frame_num=args.frame_num,
        shift=cfg.sample_shift,
        seed=0,
        # Group offloading already owns residency. `offload_model=True` would end
        # by calling `self.model.cpu()`, which goes through Module._apply rather
        # than .to(), slips past the loader's guard, and corrupts the hooks'
        # bookkeeping.
        offload_model=False,
    )
    elapsed = time.perf_counter() - t1

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save_video(tensor=video[None], save_file=args.out, fps=cfg.sample_fps,
               nrow=1, normalize=True, value_range=(-1, 1))

    frames = video.shape[1]
    print()
    print(f'video tensor:  {tuple(video.shape)}  (C, T, H, W)')
    print(f'generate:      {elapsed:.1f} s for {frames} frames '
          f'({elapsed / max(frames, 1):.2f} s/frame)')
    print(f'peak VRAM:     {torch.cuda.max_memory_allocated() / 2**30:.1f} GiB')
    stop.set()
    print(f'saved:         {args.out}')


if __name__ == '__main__':
    main()
