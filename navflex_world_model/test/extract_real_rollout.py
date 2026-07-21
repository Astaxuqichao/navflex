#!/usr/bin/env python3
"""Cut real ego-view rollouts out of a MATRiX rosbag.

The critic is going to judge frames that LingBot generated from the robot's own
camera, so the benchmark should start from that camera, not from a stock photo.
This finds stretches of odometry where the robot drove roughly straight ahead,
then pulls the `/image_raw/compressed` frames recorded along them.

    python3 test/extract_real_rollout.py <bag_dir> [--out /tmp/navflex_real]
"""

import argparse
import math
import os
import pathlib
import sys

from rosbags.highlevel import AnyReader

RGB_TOPIC = '/image_raw/compressed'
ODOM_TOPIC = '/odom'


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_odom(reader):
    conns = [c for c in reader.connections if c.topic == ODOM_TOPIC]
    track = []
    for conn, stamp, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        p = msg.pose.pose.position
        track.append((stamp, p.x, p.y, yaw_of(msg.pose.pose.orientation)))
    return track


def straight_segments(track, min_len=3.5, max_turn_deg=25.0, stride=200):
    """Windows where the robot advanced `min_len` metres without turning much.

    Straightness is measured two ways, because either alone is foolable: the
    net heading change must be small, AND the net displacement must be most of
    the distance actually travelled (otherwise a there-and-back counts).
    """
    out = []
    n = len(track)
    i = 0
    while i < n:
        t0, x0, y0, yaw0 = track[i]
        travelled = 0.0
        px, py = x0, y0
        j = i + 1
        while j < n:
            t1, x1, y1, yaw1 = track[j]
            travelled += math.dist((px, py), (x1, y1))
            px, py = x1, y1
            net = math.dist((x0, y0), (x1, y1))
            if net >= min_len:
                turn = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
                if (math.degrees(turn) <= max_turn_deg
                        and travelled > 0 and net / travelled >= 0.9):
                    out.append({'t0': t0, 't1': t1, 'net': net,
                                'turn_deg': math.degrees(turn),
                                'straightness': net / travelled})
                break
            if travelled > min_len * 2.5:  # wandered without getting anywhere
                break
            j += 1
        i += stride
    return out


def frames_between(reader, t0, t1, count):
    conns = [c for c in reader.connections if c.topic == RGB_TOPIC]
    stamps, blobs = [], []
    for conn, stamp, raw in reader.messages(connections=conns, start=t0, stop=t1):
        msg = reader.deserialize(raw, conn.msgtype)
        stamps.append(stamp)
        blobs.append(bytes(msg.data))
    if len(blobs) < count:
        return blobs
    step = (len(blobs) - 1) / (count - 1)
    return [blobs[round(k * step)] for k in range(count)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default='/tmp/navflex_real')
    ap.add_argument('--count', type=int, default=6)
    ap.add_argument('--segments', type=int, default=3)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    with AnyReader([pathlib.Path(args.bag)]) as reader:
        track = read_odom(reader)
        print(f'odom samples: {len(track)}')
        segs = straight_segments(track)
        if not segs:
            sys.exit('no straight segment found')
        segs.sort(key=lambda s: (-s['net'], -s['straightness']))
        print(f'straight segments: {len(segs)}\n')

        for k, seg in enumerate(segs[:args.segments]):
            blobs = frames_between(reader, seg['t0'], seg['t1'], args.count)
            for i, blob in enumerate(blobs):
                with open(f'{args.out}/seg{k}_{i}.jpg', 'wb') as fh:
                    fh.write(blob)
            secs = (seg['t1'] - seg['t0']) / 1e9
            print(f'seg{k}: {seg["net"]:.2f} m forward, turn {seg["turn_deg"]:.1f} deg, '
                  f'straightness {seg["straightness"]:.2f}, {secs:.1f} s, '
                  f'{len(blobs)} frames -> {args.out}/seg{k}_*.jpg')


if __name__ == '__main__':
    main()
