#!/usr/bin/env python3
"""Is a bad rollout my camera trajectory, or the single-GPU config?

Loads the pipeline once, then generates the same clip under several control
signals. The repo's own `examples/03/poses.npy` is the control: if it produces a
coherent video and mine does not, the fault is in pose_utils. If the control is
also garbage, the fault is in how this box runs the model (offloading, KV window,
sink tokens) and no trajectory will save it.
"""

import argparse
import math
import os
import shutil
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from navflex_wm.pose_utils import CameraExtrinsic, plan_to_control_signal  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--ckpt_dir',
                   default='/home/jiangbz/project/world_model/models/lingbot-world-v2-14b-causal-fast')
    p.add_argument('--lingbot_repo', default='/home/jiangbz/project/world_model/lingbot-world-v2')
    p.add_argument('--frame_num', type=int, default=21)
    p.add_argument('--chunk_size', type=int, default=4)
    p.add_argument('--size', default='480*832')
    p.add_argument('--local_attn_size', type=int, default=-1)
    p.add_argument('--sink_size', type=int, default=0)
    p.add_argument('--out_dir', default='/tmp/navflex_traj')
    return p.parse_args()


def main():
    args = parse_args()
    sys.path.insert(0, args.lingbot_repo)

    import torch
    from PIL import Image

    from wan.configs import MAX_AREA_CONFIGS
    from wan.utils.utils import save_video

    from navflex_wm.lingbot_loader import build_pipeline

    os.makedirs(args.out_dir, exist_ok=True)
    example = os.path.join(args.lingbot_repo, 'examples', '03')
    image = Image.open(os.path.join(example, 'image.jpg')).convert('RGB')
    # examples/03 ships no prompt.txt; this is the one the v2 README pairs with
    # that image. The first smoke test captioned this lake as an indoor robot
    # corridor, which is a confound on top of the trajectory question.
    prompt_path = os.path.join(example, 'prompt.txt')
    prompt = (open(prompt_path).read().strip() if os.path.exists(prompt_path) else
              'A serene lakeside scene with a lone tree standing in calm water, '
              'surrounded by distant snow-capped mountains under a bright blue sky '
              'with drifting white clouds — gentle ripples reflect the tree and sky, '
              'creating a tranquil, meditative atmosphere.')

    extrinsic = CameraExtrinsic(x=0.29, y=0.0, z=0.01, pitch=math.radians(15.0))

    cases = {}

    # Control: the repo's own poses + its own prompt. Nothing of mine involved.
    cases['control_repo_poses'] = (example, prompt)

    # Mine, gentle: 1.5 m straight ahead, no turn.
    straight = f'{args.out_dir}/straight'
    os.makedirs(straight, exist_ok=True)
    c2ws, K = plan_to_control_signal(
        [[0.0, 0.0, 0.0], [1.5, 0.0, 0.0]], args.frame_num, extrinsic, 90.0, 1920, 1080)
    np.save(f'{straight}/poses.npy', c2ws)
    np.save(f'{straight}/intrinsics.npy', K)
    cases['mine_straight_1p5m'] = (straight, prompt)

    # Mine, the aggressive plan the first smoke test used: 4 m + a 90 deg turn.
    turning = f'{args.out_dir}/turning'
    os.makedirs(turning, exist_ok=True)
    c2ws, K = plan_to_control_signal(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.5, 1.0, 0.0], [4.0, 2.0, 0.0]],
        args.frame_num, extrinsic, 90.0, 1920, 1080, final_yaw=math.pi / 2)
    np.save(f'{turning}/poses.npy', c2ws)
    np.save(f'{turning}/intrinsics.npy', K)
    cases['mine_4m_turn90'] = (turning, prompt)

    print('loading pipeline once...')
    pipe, cfg = build_pipeline(
        ckpt_dir=args.ckpt_dir, lingbot_repo=args.lingbot_repo,
        blocks_per_group=1, local_attn_size=args.local_attn_size,
        sink_size=args.sink_size, use_stream=False)

    # How far the control signal actually moves the camera, in the model's own
    # frame -- the number that decides whether a rollout is an interpolation or
    # an extrapolation.
    for name, (path, _) in cases.items():
        poses = np.load(f'{path}/poses.npy')
        span = float(np.linalg.norm(poses[-1, :3, 3] - poses[0, :3, 3]))
        rot = poses[0, :3, :3].T @ poses[-1, :3, :3]
        angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rot) - 1) / 2))))
        print(f'  {name:22} frames={len(poses):3d}  translation={span:.2f}  '
              f'rotation={angle:.1f} deg')

    for name, (path, text) in cases.items():
        print(f'\n── {name}')
        torch.cuda.reset_peak_memory_stats()
        t0 = time.perf_counter()
        video = pipe.generate(
            text, image, action_path=path, chunk_size=args.chunk_size,
            max_area=MAX_AREA_CONFIGS[args.size], frame_num=args.frame_num,
            shift=cfg.sample_shift, seed=0, offload_model=False)
        elapsed = time.perf_counter() - t0
        out = f'{args.out_dir}/{name}.mp4'
        save_video(tensor=video[None], save_file=out, fps=cfg.sample_fps,
                   nrow=1, normalize=True, value_range=(-1, 1))
        print(f'   {elapsed:.1f}s  {tuple(video.shape)}  peak '
              f'{torch.cuda.max_memory_allocated() / 2**30:.1f} GiB  -> {out}')


if __name__ == '__main__':
    main()
