#!/usr/bin/env python3
"""Pick two seed frames for the gate demo: one blocked, one open.

Uses the MATRiX bag's own depth channel rather than eyeballing the RGB, so the
demo's premise ("there is a bookcase 0.91 m ahead") is a measurement, not a
claim. Depth is metres, packed into the PNG as ``depth = R + G/255`` -- MATRiX's
own decoder. It checks out geometrically: floor underfoot 1.05 m, ceiling 3.0 m,
the far doorway 4.6 m, sky through a window 23.3 m.

Two traps this avoids:

* ``0`` is the invalid sentinel, not "zero metres". It fires on glass, sheer
  curtains and sky -- exactly the surfaces a robot must not read as obstacles.
  Taking the raw minimum picks a *window* as the most blocked frame.
* the very nearest frames are degenerate: the camera is clipped inside the
  geometry (0.05 m). An obstacle worth imagining sits far enough to be rendered
  and near enough to fall inside the 5.26 m the rollout covers.

Run on the host, in the conda env that has rosbags + opencv:

    conda activate lingbot_wm
    python3 scripts/gate_demo_seeds.py \
        --bag /home/jiangbz/data/quadrupedwm_raw_data/MATRiX_dataset/0_zsl1_house2_odom_rgb_rgbd
"""

import argparse
import json
import pathlib
import sys

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

RGB_TOPIC = '/image_raw/compressed'
DEPTH_TOPIC = '/image_raw/compressed/depth'

# An obstacle the rollout can actually depict: far enough that the camera is not
# clipped inside it, near enough to sit well within the 5.26 m the model covers.
BLOCK_LO, BLOCK_HI = 0.9, 1.8
# The open case must be clear beyond everything the rollout will show.
OPEN_MIN = 5.5


def unpack_depth(png_bytes):
    """Metres. cv2 hands back BGRA, so R is channel 2 and G is channel 1."""
    a = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
    if a is None or a.ndim != 3 or a.shape[2] < 3:
        raise ValueError('depth frame did not decode as a colour PNG')
    return a[..., 2].astype(np.float32) + a[..., 1].astype(np.float32) / 255.0


def path_clearance(depth):
    """Distance to the nearest solid thing in the robot's path. NaN if unusable.

    The band is the lower-centre of the image: where the robot is about to
    drive, not the ceiling.
    """
    h, w = depth.shape
    band = depth[int(h * 0.52):int(h * 0.72), int(w * 0.35):int(w * 0.65)]
    valid = band[band > 0.01]
    if valid.size < 0.85 * band.size:
        return np.nan          # mostly glass or sky: not a usable obstacle frame
    return float(np.percentile(valid, 20))


def read_topic(bag, topic, stride):
    with AnyReader([bag]) as reader:
        conn = [c for c in reader.connections if c.topic == topic]
        if not conn:
            raise SystemExit(f'bag has no {topic}')
        return [bytes(reader.deserialize(raw, conn[0].msgtype).data)
                for i, (_, _, raw) in enumerate(reader.messages(connections=conn))
                if i % stride == 0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bag', required=True, type=pathlib.Path)
    ap.add_argument('--stride', type=int, default=20,
                    help='sample every Nth frame; 9473 frames is far more than needed')
    ap.add_argument('--out', type=pathlib.Path,
                    default=pathlib.Path(__file__).resolve().parent.parent / 'test' / 'data')
    args = ap.parse_args()

    rgbs = read_topic(args.bag, RGB_TOPIC, args.stride)
    depths = [unpack_depth(b) for b in read_topic(args.bag, DEPTH_TOPIC, args.stride)]
    if len(rgbs) != len(depths):
        raise SystemExit(f'{len(rgbs)} rgb frames but {len(depths)} depth frames')

    clear = np.array([path_clearance(d) for d in depths])
    usable = ~np.isnan(clear)
    print(f'扫描 {len(clear)} 帧 (每 {args.stride} 帧一张); '
          f'{(~usable).sum()} 帧深度大面积无效(窗/天空),已排除')
    print(f'路径净空: min {np.nanmin(clear):.2f} m  '
          f'中位 {np.nanmedian(clear):.2f} m  max {np.nanmax(clear):.2f} m')

    in_band = usable & (clear >= BLOCK_LO) & (clear <= BLOCK_HI)
    if not in_band.any():
        raise SystemExit(f'没有净空落在 {BLOCK_LO}-{BLOCK_HI} m 的帧')
    blocked = int(np.where(in_band)[0][np.argmin(clear[in_band])])
    open_ = int(np.nanargmax(np.where(usable, clear, -np.inf)))
    if clear[open_] < OPEN_MIN:
        raise SystemExit(f'最开阔的一帧只有 {clear[open_]:.2f} m,不足以做对照')

    args.out.mkdir(parents=True, exist_ok=True)
    meta = {}
    for tag, idx in (('blocked', blocked), ('open', open_)):
        path = args.out / f'seed_{tag}.jpg'
        path.write_bytes(rgbs[idx])
        meta[tag] = {'bag_frame': idx * args.stride,
                     'clearance_m': round(float(clear[idx]), 2),
                     'path': str(path)}
        print(f'  {tag:8s} bag 帧 {idx*args.stride:5d}  前方 {clear[idx]:.2f} m  -> {path}')

    meta_path = args.out / 'seed_meta.json'
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    print(f'  写入 {meta_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
