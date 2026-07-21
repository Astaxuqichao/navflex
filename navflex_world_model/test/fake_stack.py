#!/usr/bin/env python3
"""Minimal stand-ins for the pieces the gate talks to.

Serves a compute_path_to_pose action that returns a straight line to the goal,
and publishes a CompressedImage on the real camera topic. Enough to drive the
gate through planning, pose conversion, the backend and the critic without nav2
or a robot.

The published frame is a real MATRiX ego-view (test/data/matrix_seed.jpg), not a
flat colour. This matters: LingBot conditions the whole rollout on it, and a
featureless image makes the critic invent structure that is not there -- during
the critic benchmark, minimax-m3 described a corridor with 'warped furniture'
in an image that was uniformly grey.
"""

import io
import math
import os
import pathlib
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# NAVFLEX_SEED lets a test drive the gate from a chosen frame -- e.g. one the
# bag's depth channel says has a cabinet 0.91 m ahead.
SEED_IMAGE = pathlib.Path(
    os.environ.get('NAVFLEX_SEED',
                   pathlib.Path(__file__).parent / 'data' / 'matrix_seed.jpg'))


def _jpeg(width=640, height=360):
    """The real ego frame if it is checked out, else a flat fallback."""
    if SEED_IMAGE.is_file():
        return SEED_IMAGE.read_bytes()
    from PIL import Image
    buffer = io.BytesIO()
    Image.new('RGB', (width, height), (90, 110, 130)).save(buffer, format='JPEG')
    return buffer.getvalue()


class FakeStack(Node):
    def __init__(self):
        super().__init__('fake_stack')
        self.spacing = 0.05  # costmap-resolution-ish, like a real global plan

        self.action_server = ActionServer(
            self, ComputePathToPose, 'compute_path_to_pose', self.on_plan)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(
            CompressedImage, '/image_raw/compressed', qos)
        self.jpeg = _jpeg()
        self.create_timer(0.1, self.publish_image)
        self.get_logger().info('fake planner + camera up')

    def publish_image(self):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = self.jpeg
        self.image_pub.publish(msg)

    def on_plan(self, goal_handle):
        goal = goal_handle.request.goal.pose.position
        distance = math.hypot(goal.x, goal.y)
        count = max(2, int(distance / self.spacing))

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(count):
            t = i / (count - 1)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = goal.x * t
            pose.pose.position.y = goal.y * t
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        goal_handle.succeed()
        result = ComputePathToPose.Result()
        result.path = path
        self.get_logger().info(f'planned {len(path.poses)} poses over {distance:.2f} m')
        return result


def main():
    rclpy.init(args=sys.argv[1:] or None)
    node = FakeStack()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
