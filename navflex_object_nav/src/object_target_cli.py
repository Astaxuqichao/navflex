#!/usr/bin/env python3
"""Interactive terminal for object-goal navigation.

Type a target description and press Enter — it is published to the orchestrator,
which searches for it and drives there. State updates (searching / spotted /
ranked / found / gave_up ...) print live in the same terminal, so one window
drives the whole thing.

  ros2 run navflex_object_nav object_target_cli.py

Then just type, e.g.:
  厨房边的冰箱
  楼梯旁的衣柜
  stop            # abort the current search
  quit / exit     # leave the CLI (does NOT stop the orchestrator)

Publishes:  navflex_object_nav/target (std_msgs/String)
            navflex_object_nav/stop   (std_msgs/Empty)  on "stop"
Subscribes: navflex_object_nav/state  (std_msgs/String, JSON) -> printed live
"""

import json
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

PROMPT = '\n目标> '


class TargetCli(Node):
    def __init__(self) -> None:
        super().__init__('object_target_cli')
        self.target_pub = self.create_publisher(String, 'navflex_object_nav/target', 10)
        self.stop_pub = self.create_publisher(Empty, 'navflex_object_nav/stop', 10)
        self.create_subscription(String, 'navflex_object_nav/state', self._on_state, 10)
        # Also surface the fine-grained activity so the long (~90s) world-model
        # imagines and each detect show up live instead of a silent gap.
        self.create_subscription(String, 'navflex_object_nav/grounder_events',
                                 self._on_grounder_event, 10)
        self.create_subscription(String, 'navflex_world_model/events',
                                 self._on_wm_event, 10)
        self.get_logger().info(
            "对象目标导航 CLI 就绪。输入目标描述回车即开始；'stop' 中止；'quit' 退出。")

    def _line(self, text: str) -> None:
        # Carriage-return keeps the live line above the prompt tidy-ish.
        sys.stdout.write(f'\r{text}{PROMPT}')
        sys.stdout.flush()

    def _on_state(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._line(f'[state] {msg.data}')
            return
        state = d.get('state', '?')
        extra = {k: v for k, v in d.items() if k not in ('stamp', 'state', 'target')}
        self._line(f'[{state}] {d.get("target", "")} '
                   + ' '.join(f'{k}={v}' for k, v in extra.items()))

    def _on_grounder_event(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        vis = '✓看到' if d.get('visible') else '·未见'
        self._line(f'[detect] {vis}  conf={d.get("confidence", 0):.2f} '
                   f'bearing={d.get("bearing_deg", 0):.0f}°')

    def _on_wm_event(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        # Only the director-rank progress is interesting here (not gate evaluates).
        if d.get('type') in ('ranking', 'ranked_one'):
            self._line(f'[world-model] {d.get("message", "")}')

    def send_target(self, text: str) -> None:
        self.target_pub.publish(String(data=text))
        print(f'  -> 开始搜索: {text}')
        # Catch the #1 gotcha: this terminal on a different RMW/domain than the
        # orchestrator, so the target reaches nobody and the dog never moves.
        if self.target_pub.get_subscription_count() == 0:
            print('  ⚠ 没有节点在监听 /navflex_object_nav/target！'
                  '这个终端很可能没设对环境——请确认已 '
                  'export RMW_IMPLEMENTATION=rmw_zenoh_cpp 和 ROS_DOMAIN_ID=89'
                  '（要和编排器一致），或确认编排器已启动。')

    def send_stop(self) -> None:
        self.stop_pub.publish(Empty())
        print('  -> 已发送中止')


def _stdin_loop(node: TargetCli) -> None:
    # Read RAW bytes and decode UTF-8 ourselves. input()/readline() use the
    # locale's encoding, and the container often runs a non-UTF-8 locale (C /
    # POSIX), so typing multi-byte Chinese interactively raises UnicodeDecodeError.
    # Reading sys.stdin.buffer sidesteps that entirely (cost: no line editing).
    while rclpy.ok():
        sys.stdout.write(PROMPT)
        sys.stdout.flush()
        try:
            raw = sys.stdin.buffer.readline()
        except (KeyboardInterrupt, EOFError):
            break
        if not raw:            # EOF (Ctrl-D / closed pipe)
            break
        cmd = raw.decode('utf-8', errors='replace').strip()
        if not cmd:
            continue
        low = cmd.lower()
        if low in ('quit', 'exit', 'q'):
            break
        if low == 'stop':
            node.send_stop()
        else:
            node.send_target(cmd)
    rclpy.shutdown()


def main(args=None) -> None:
    # Make our own writes UTF-8 too, so the Chinese prompt/state render regardless
    # of the container locale.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8')
        except (AttributeError, ValueError):
            pass
    rclpy.init(args=args)
    node = TargetCli()
    # ROS spins in a background thread; stdin drives the foreground.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        _stdin_loop(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
