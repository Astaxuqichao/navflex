#!/usr/bin/env python3
"""Drive the gate through the demo scenarios and record what it decided.

Calls /navflex_world_model/evaluate once per scenario and writes a single JSON
file that the page builder reads. Parsing `ros2 service call`'s printed output
with regexes is how you end up believing a stale node -- ask the service
directly and serialise the response.

Runs inside the ROS container, with the gate and a fake_stack already up:

    source install/setup.bash
    python3 scripts/gate_demo_run.py --scenario blocked --out /tmp/gate_demo/blocked.json

The seed image is chosen by fake_stack via NAVFLEX_SEED, so this script does not
need to know about it -- it only records which scenario it was told it was in.
"""

import argparse
import json
import pathlib
import re
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from navflex_world_model.srv import EvaluatePlan

TEAR_RE = re.compile(
    r'rollout lost coherence at frame (\d+) \(~([\d.]+) m\); '
    r'judging only the (\d+) frames')
JUDGED_RE = re.compile(r'judging the first (\d+) of (\d+) frames')
TRAJ_RE = re.compile(r'camera trajectory: (\d+) frames (.+?), '
                     r'([\d.]+) deg of turning \(([\d.]+) deg/frame\)')


class DemoClient(Node):
    def __init__(self):
        super().__init__('gate_demo_client')
        self.client = self.create_client(EvaluatePlan, '/navflex_world_model/evaluate')

    def evaluate(self, x, y, instruction, timeout):
        if not self.client.wait_for_service(timeout_sec=30.0):
            raise SystemExit('gate service never appeared; is the node up?')
        request = EvaluatePlan.Request()
        request.goal = PoseStamped()
        request.goal.header.frame_id = 'map'
        request.goal.pose.position.x = float(x)
        request.goal.pose.position.y = float(y)
        request.goal.pose.orientation.w = 1.0
        request.instruction = instruction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise SystemExit(f'gate did not answer within {timeout:.0f} s')
        return future.result()


def summarise(steps):
    """Pull the numbers the page wants out of the gate's own step log."""
    out = {'tear_frame': 0, 'tear_m': 0.0, 'frames_judged': 0, 'frames_rendered': 0}
    for step in steps:
        tear = TEAR_RE.search(step)
        if tear:
            out['tear_frame'] = int(tear.group(1))
            out['tear_m'] = float(tear.group(2))
            out['frames_judged'] = int(tear.group(3))
        judged = JUDGED_RE.search(step)
        if judged and not out['frames_judged']:
            out['frames_judged'] = int(judged.group(1))
            out['frames_rendered'] = int(judged.group(2))
        traj = TRAJ_RE.search(step)
        if traj:
            out['frames_rendered'] = int(traj.group(1))
            out['coverage'] = traj.group(2)
            out['turn_deg'] = float(traj.group(3))
            out['deg_per_frame'] = float(traj.group(4))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--scenario', required=True, help='label recorded in the output')
    ap.add_argument('--instruction', required=True)
    ap.add_argument('--goal-x', type=float, default=4.0)
    ap.add_argument('--goal-y', type=float, default=0.0)
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--out', required=True, type=pathlib.Path)
    args = ap.parse_args()

    rclpy.init()
    try:
        response = DemoClient().evaluate(args.goal_x, args.goal_y,
                                         args.instruction, args.timeout)
    finally:
        rclpy.shutdown()

    steps = list(response.steps)
    record = {
        'scenario': args.scenario,
        'instruction': args.instruction,
        'goal': [args.goal_x, args.goal_y],
        'success': response.success,
        'verdict': response.verdict,
        'confidence': response.confidence,
        'reason': response.reason,
        'rollout_uri': response.rollout_uri,
        'critic': json.loads(response.critic_json) if response.critic_json else {},
        'steps': steps,
        'elapsed_s': response.elapsed_time.sec,
        **summarise(steps),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(record, indent=2, ensure_ascii=False))

    print(f"[{args.scenario}] {response.verdict} ({response.confidence:.2f})  "
          f"{response.elapsed_time.sec} s")
    if record['tear_frame']:
        print(f"  想象在第 {record['tear_frame']} 帧 (~{record['tear_m']:.2f} m) 撕裂,"
              f"只判前 {record['frames_judged']} 帧")
    print(f"  {response.reason[:110]}")
    print(f"  -> {args.out}")
    return 0 if response.success else 1


if __name__ == '__main__':
    sys.exit(main())
