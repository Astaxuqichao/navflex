#!/usr/bin/env python3
"""Phase B of object-goal navigation: approach a target the robot can already see.

Given a target the grounder reports visible at bearing theta (base_link, +left):
  1. range it — project theta into the forward LIDAR (/livox/lidar/filtered) and
     take the median range of the points near that bearing. The camera FOV (90 deg
     = +/-45) matches the filter cone (+/-45), both forward on base_link, so a
     camera bearing maps straight into the LIDAR cone.
  2. place the target in map = robot_pose (+) (range, theta);
  3. compute an approach pose ~approach_distance in FRONT of the target, facing it;
  4. (optional) ask the world-model gate whether driving there is safe;
  5. unless dry_run, drive there with compute_path_to_pose + follow_path.

The metric comes from a real range sensor, not from imagination — the world model
is only the safety verifier here (skippable, so phase B works without the GPU
backend). Service: navflex_object_nav/approach (navflex_object_nav/srv/ApproachTarget).
"""

import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from navflex_object_nav.srv import ApproachTarget, DetectObject

try:
    from navflex_world_model.srv import EvaluatePlan
    _HAVE_WM = True
except ImportError:
    _HAVE_WM = False


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]],
        dtype=np.float64)


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def cloud_to_xyz(msg: PointCloud2):
    """PointCloud2 -> (N,3) float64 array of finite points (assumes float32 x/y/z)."""
    offs = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= offs.keys():
        return None
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

    def col(name):
        o = offs[name]
        return raw[:, o:o + 4].copy().view(np.float32).reshape(n).astype(np.float64)

    pts = np.stack([col('x'), col('y'), col('z')], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


class ObjectApproach(Node):
    def __init__(self) -> None:
        super().__init__('object_approach')

        self.declare_parameter('lidar_topic', '/livox/lidar/filtered')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('detect_service', 'navflex_object_nav/detect')
        # Ranging window: keep LIDAR points within +/- this of the camera bearing.
        self.declare_parameter('angular_window_deg', 6.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_lidar_support', 3)   # points needed to trust a range
        self.declare_parameter('approach_distance', 0.8)  # stop this far in front
        self.declare_parameter('cloud_max_age', 1.0)
        # Drive backend: nav2 planner+controller actions (same servers exploration uses).
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('action_timeout', 120.0)
        self.declare_parameter('detect_timeout', 210.0)
        self.declare_parameter('detect_poll_interval', 0.25)
        # World-model safety gate (optional).
        self.declare_parameter('verify_with_world_model', False)
        self.declare_parameter('world_model_service', 'navflex_world_model/evaluate')
        self.declare_parameter('world_model_timeout', 900.0)

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.ang_win = math.radians(float(self.get_parameter('angular_window_deg').value))
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_support = int(self.get_parameter('min_lidar_support').value)
        self.approach_distance = float(self.get_parameter('approach_distance').value)
        self.cloud_max_age = float(self.get_parameter('cloud_max_age').value)
        self.planner_id = self.get_parameter('planner_id').value
        self.controller_id = self.get_parameter('controller_id').value
        self.action_timeout = float(self.get_parameter('action_timeout').value)
        self.detect_timeout = float(self.get_parameter('detect_timeout').value)
        self.detect_poll_interval = float(
            self.get_parameter('detect_poll_interval').value)
        self.verify_wm = bool(self.get_parameter('verify_with_world_model').value)

        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        cloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self._cloud = None
        self._cloud_stamp = 0.0
        self._cloud_lock = threading.Lock()
        self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value,
            self._on_cloud, cloud_qos, callback_group=self.cb)

        self.detect_client = self.create_client(
            DetectObject, self.get_parameter('detect_service').value,
            callback_group=self.cb)
        self.plan_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose', callback_group=self.cb)
        self.follow_client = ActionClient(
            self, FollowPath, 'follow_path', callback_group=self.cb)

        self.wm_client = None
        if self.verify_wm and _HAVE_WM:
            self.wm_client = self.create_client(
                EvaluatePlan, self.get_parameter('world_model_service').value,
                callback_group=self.cb)

        self.marker_pub = self.create_publisher(
            MarkerArray, 'navflex_object_nav/approach_markers', 1)
        self.create_service(
            ApproachTarget, 'navflex_object_nav/approach', self._on_approach,
            callback_group=self.cb)

        self.get_logger().info(
            f"Object approach ready. lidar='{self.get_parameter('lidar_topic').value}' "
            f"approach_dist={self.approach_distance:.2f}m "
            f"verify_wm={self.verify_wm and _HAVE_WM}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._cloud_lock:
            self._cloud = msg
            self._cloud_stamp = time.monotonic()

    def _take_cloud(self):
        with self._cloud_lock:
            if self._cloud is None:
                return None
            if time.monotonic() - self._cloud_stamp > self.cloud_max_age:
                return None
            return self._cloud

    def _wait(self, future, label: str, timeout: float = None):
        deadline = time.monotonic() + (
            timeout if timeout is not None else self.action_timeout)
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
            if time.monotonic() > deadline:
                self.get_logger().error(f'timed out waiting for {label}')
                return False
        return future.done()

    # ---- geometry -----------------------------------------------------------

    def _range_along_bearing(self, theta: float):
        """Median LIDAR range near bearing theta (base_link), and support count."""
        cloud = self._take_cloud()
        if cloud is None:
            return None, 0
        pts = cloud_to_xyz(cloud)
        if pts is None or len(pts) == 0:
            return None, 0
        # Transform points from the cloud frame into base_link.
        src = cloud.header.frame_id or 'lidar'
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_frame, src, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'TF {src}->{self.robot_frame} failed: {exc}')
            return None, 0
        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y,
                      tf.transform.translation.z])
        pb = pts @ R.T + t
        bearings = np.arctan2(pb[:, 1], pb[:, 0])
        ranges = np.hypot(pb[:, 0], pb[:, 1])
        d_ang = np.abs(np.arctan2(np.sin(bearings - theta), np.cos(bearings - theta)))
        mask = (d_ang <= self.ang_win) & (ranges >= self.min_range) & (ranges <= self.max_range)
        sel = ranges[mask]
        if len(sel) < self.min_support:
            return None, int(len(sel))
        return float(np.median(sel)), int(len(sel))

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF {self.robot_frame}->{self.global_frame} failed: {exc}')
            return None
        return (tf.transform.translation.x, tf.transform.translation.y,
                yaw_from_quat(tf.transform.rotation))

    # ---- service ------------------------------------------------------------

    def _on_approach(self, request, response):
        target = (request.target or '').strip()
        approach_dist = request.approach_distance or self.approach_distance
        response.message = ''

        # 1) detect + bearing
        det = self._detect(target)
        if det is None:
            response.found = False
            response.message = 'detect service unavailable'
            return response
        response.visible = det.visible
        response.confidence = det.confidence
        response.bearing_deg = det.bearing_deg
        if not det.visible:
            response.found = False
            response.message = f'target not visible: {det.notes}'
            return response

        # 2) range along the detected bearing
        rng, support = self._range_along_bearing(det.bearing_rad)
        response.range_m = rng or 0.0
        response.lidar_support = support
        if rng is None:
            response.found = False
            response.message = (f'no LIDAR return near bearing {det.bearing_deg:+.0f}deg '
                                f'({support} pts < {self.min_support})')
            return response

        # 3) target + approach pose in map
        pose = self._robot_pose()
        if pose is None:
            response.found = False
            response.message = 'no robot pose (TF)'
            return response
        rx, ry, r_yaw = pose
        world_bearing = r_yaw + det.bearing_rad
        tx = rx + rng * math.cos(world_bearing)
        ty = ry + rng * math.sin(world_bearing)
        # Approach pose: back off approach_dist from the target toward the robot.
        ax = tx - approach_dist * math.cos(world_bearing)
        ay = ty - approach_dist * math.sin(world_bearing)
        ayaw = world_bearing  # face the target
        response.found = True
        response.target_x, response.target_y = tx, ty
        response.approach_x, response.approach_y, response.approach_yaw = ax, ay, ayaw
        self._publish_markers(tx, ty, ax, ay, ayaw)
        self.get_logger().info(
            f"approach('{target}'): visible@{det.bearing_deg:+.0f}deg range={rng:.2f}m "
            f"({support}pts) -> target=({tx:.2f},{ty:.2f}) approach=({ax:.2f},{ay:.2f})")

        goal = self._make_pose(ax, ay, ayaw)

        # 4) optional world-model safety gate
        response.verified = True
        if self.wm_client is not None and not request.skip_verify:
            ok, reason = self._verify(goal, target)
            response.verified = ok
            response.verify_reason = reason
            if not ok:
                response.message = f'world model vetoed approach: {reason}'
                return response

        if request.dry_run:
            response.message = 'dry_run: pose computed, not driven'
            return response

        # 5) drive
        driven, msg = self._drive(goal)
        response.driven = driven
        response.message = msg
        return response

    def _detect(self, target: str):
        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            return None
        deadline = time.monotonic() + self.detect_timeout
        while time.monotonic() < deadline:
            req = DetectObject.Request()
            req.target = target
            fut = self.detect_client.call_async(req)
            remaining = max(0.1, deadline - time.monotonic())
            if not self._wait(fut, 'detect poll', timeout=remaining):
                return None
            result = fut.result()
            if result is None:
                return None
            if result.notes != 'detection pending':
                return result
            time.sleep(self.detect_poll_interval)
        self.get_logger().error('timed out waiting for detector result')
        return None

    def _verify(self, goal: PoseStamped, target: str):
        if not self.wm_client.wait_for_service(timeout_sec=5.0):
            return True, 'world model service unavailable; skipped'
        req = EvaluatePlan.Request()
        req.goal = goal
        req.instruction = (f'机器人要走到"{target}"跟前。请判断沿这条路径过去是否安全、'
                           f'可通行——不会撞上障碍或跌落。')
        req.frame_num = 0
        req.dry_run = False
        fut = self.wm_client.call_async(req)
        if not self._wait(fut, 'world model evaluate'):
            return True, 'world model timed out; skipped'
        res = fut.result()
        return (res.verdict == 'approve'), f'{res.verdict}: {res.reason}'

    def _drive(self, goal: PoseStamped):
        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            return False, 'compute_path_to_pose server not available'
        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = goal
        plan_goal.planner_id = self.planner_id
        plan_goal.use_start = False
        sf = self.plan_client.send_goal_async(plan_goal)
        if not self._wait(sf, 'plan send'):
            return False, 'plan send timed out'
        h = sf.result()
        if h is None or not h.accepted:
            return False, 'planner rejected goal'
        rf = h.get_result_async()
        if not self._wait(rf, 'plan result'):
            return False, 'plan result timed out'
        wrapped = rf.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, 'planning failed'
        path = wrapped.result.path
        if not path.poses:
            return False, 'planner returned empty path'

        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            return False, 'follow_path server not available'
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = self.controller_id
        sf = self.follow_client.send_goal_async(follow_goal)
        if not self._wait(sf, 'follow send'):
            return False, 'follow send timed out'
        h = sf.result()
        if h is None or not h.accepted:
            return False, 'controller rejected path'
        rf = h.get_result_async()
        if not self._wait(rf, 'follow result'):
            return False, 'follow result timed out'
        wrapped = rf.result()
        ok = wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED
        return ok, ('arrived' if ok else 'follow_path failed')

    def _make_pose(self, x, y, yaw) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.global_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw * 0.5)
        p.pose.orientation.w = math.cos(yaw * 0.5)
        return p

    def _publish_markers(self, tx, ty, ax, ay, ayaw):
        arr = MarkerArray()
        s = Marker()
        s.header.frame_id = self.global_frame
        s.header.stamp = self.get_clock().now().to_msg()
        s.ns = 'object_target'
        s.id = 0
        s.type = Marker.SPHERE
        s.action = Marker.ADD
        s.pose.position.x, s.pose.position.y, s.pose.position.z = tx, ty, 0.3
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = 0.25
        s.color.r, s.color.g, s.color.b, s.color.a = 1.0, 0.1, 0.1, 0.9
        arr.markers.append(s)
        a = Marker()
        a.header = s.header
        a.ns = 'object_approach'
        a.id = 1
        a.type = Marker.ARROW
        a.action = Marker.ADD
        a.pose.position.x, a.pose.position.y, a.pose.position.z = ax, ay, 0.1
        a.pose.orientation.z = math.sin(ayaw * 0.5)
        a.pose.orientation.w = math.cos(ayaw * 0.5)
        a.scale.x, a.scale.y, a.scale.z = 0.5, 0.08, 0.08
        a.color.r, a.color.g, a.color.b, a.color.a = 0.1, 1.0, 0.1, 0.9
        arr.markers.append(a)
        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectApproach()
    executor = MultiThreadedExecutor(num_threads=4)
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
