#!/usr/bin/env python3
"""Object-goal navigator — the two-state orchestrator.

Give it a target on `navflex_object_nav/target` (std_msgs/String) and it runs:

  SEARCHING (target not yet seen):
    · rotate-scan the current spot: the camera sees 90 deg, so rotate in
      ~60 deg steps, calling the grounder's detect() at each orientation, until
      the whole circle is covered;
    · if a full circle finds nothing, drive to the next frontier viewpoint
      (reusing the FrontierAStar planner as "where to look next" — WITHOUT the
      exploration BT), then rotate-scan there;
    · give up after max_vantages, or when there are no frontiers left.

  APPROACHING (target seen):
    · centre the target (rotate by its bearing), then hand off to phase B
      (navflex_object_nav/approach): LIDAR range -> approach pose -> drive there.
    · on arrival: FOUND.

This is the "simple search" — a placeholder for the world-model director that
will later rank frontier candidates by how likely each heads toward the target.
The search STRATEGY is swappable; the SEARCHING/APPROACHING skeleton is not.

State + progress are published as JSON on `navflex_object_nav/state`.
"""

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, DummyBehavior, FollowPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from navflex_object_nav.srv import ApproachTarget, DetectObject

try:
    from navflex_world_model.srv import RankViewpoints
    _HAVE_DIRECTOR = True
except ImportError:
    _HAVE_DIRECTOR = False


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObjectGoalNavigator(Node):
    def __init__(self) -> None:
        super().__init__('object_goal_navigator')

        self.declare_parameter('detect_service', 'navflex_object_nav/detect')
        self.declare_parameter('approach_service', 'navflex_object_nav/approach')
        self.declare_parameter('frontier_planner_id', 'FrontierAStar')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('scan_steps', 6)          # 6 x 60deg = 360
        self.declare_parameter('max_vantages', 8)         # frontier hops before giving up
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('center_tolerance_deg', 12.0)
        self.declare_parameter('settle_time', 1.2)        # s to wait after a rotate for a fresh frame
        self.declare_parameter('action_timeout', 120.0)
        # Includes a fresh VLM detection plus a Nav2 approach action.  It must
        # exceed action_timeout, otherwise the caller can abandon the approach
        # while the service is still legitimately polling the async detector.
        self.declare_parameter('approach_timeout', 330.0)
        self.declare_parameter('detect_timeout', 210.0)   # covers detector format fallbacks
        self.declare_parameter('detect_poll_interval', 0.25)
        self.declare_parameter('reacquire_last_seen', True)
        # Phase-A director: when true, "which frontier next" is chosen by the
        # world model (imagine top-K + rank toward target) instead of FrontierAStar
        # top-1. Falls back to FrontierAStar if the director is unavailable.
        self.declare_parameter('use_director', False)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('rank_service', 'navflex_world_model/rank_viewpoints')
        self.declare_parameter('candidates_topic', '/frontier_exploration/candidates')
        self.declare_parameter('max_rank_candidates', 3)   # imagines per relocation (each is slow)
        self.declare_parameter('min_candidate_dist', 0.8)  # ignore candidates closer than this
        self.declare_parameter('candidate_min_bearing_sep_deg', 45.0)
        self.declare_parameter('candidate_revisit_radius', 0.75)
        # ObjectNav needs a short look-ahead, not a full route video.  Fixing the
        # rollout horizon also prevents a small path-length change from snapping
        # a candidate from 13 to 29 frames and roughly doubling decision latency.
        self.declare_parameter('rank_frame_num', 13)
        # Predictions with only weak/ambiguous evidence must not override the
        # geometry baseline.  The director prompt assigns 0.5 to uncertainty,
        # while scores near zero mean the imagined direction is contradictory.
        self.declare_parameter('director_min_score', 0.25)
        # A rank does up to max_rank_candidates world-model imagines (~90s each) +
        # a director VLM call each — WAY more than action_timeout(120s). Give it its
        # own generous budget so the director isn't abandoned (and left pinning the
        # world model's GPU lock) every time.
        self.declare_parameter('rank_timeout', 900.0)

        self.detect_name = self.get_parameter('detect_service').value
        self.approach_name = self.get_parameter('approach_service').value
        self.frontier_planner_id = self.get_parameter('frontier_planner_id').value
        self.global_frame = self.get_parameter('global_frame').value
        self.scan_steps = int(self.get_parameter('scan_steps').value)
        self.max_vantages = int(self.get_parameter('max_vantages').value)
        self.conf_thresh = float(self.get_parameter('confidence_threshold').value)
        self.center_tol = math.radians(float(self.get_parameter('center_tolerance_deg').value))
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.action_timeout = float(self.get_parameter('action_timeout').value)
        self.approach_timeout = float(self.get_parameter('approach_timeout').value)
        self.detect_timeout = float(self.get_parameter('detect_timeout').value)
        self.detect_poll_interval = float(
            self.get_parameter('detect_poll_interval').value)
        self.reacquire_last_seen = bool(
            self.get_parameter('reacquire_last_seen').value)
        self.scan_step_rad = 2.0 * math.pi / max(1, self.scan_steps)
        self.robot_frame = self.get_parameter('robot_frame').value
        self.max_rank_candidates = int(self.get_parameter('max_rank_candidates').value)
        self.min_candidate_dist = float(self.get_parameter('min_candidate_dist').value)
        self.candidate_min_bearing_sep = math.radians(float(
            self.get_parameter('candidate_min_bearing_sep_deg').value))
        self.candidate_revisit_radius = float(
            self.get_parameter('candidate_revisit_radius').value)
        self.rank_frame_num = int(self.get_parameter('rank_frame_num').value)
        self.director_min_score = float(
            self.get_parameter('director_min_score').value)
        self.rank_timeout = float(self.get_parameter('rank_timeout').value)
        self.use_director = bool(self.get_parameter('use_director').value) and _HAVE_DIRECTOR

        self.cb = ReentrantCallbackGroup()
        self.detect_client = self.create_client(DetectObject, self.detect_name, callback_group=self.cb)
        self.approach_client = self.create_client(ApproachTarget, self.approach_name, callback_group=self.cb)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=self.cb)
        self.follow_client = ActionClient(self, FollowPath, 'follow_path', callback_group=self.cb)
        self.behavior_client = ActionClient(self, DummyBehavior, 'behavior_action', callback_group=self.cb)

        # Director path: TF (for robot pose + candidate bearings), the candidate
        # marker stream, and the rank service.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._latest_candidates = None
        self.rank_client = None
        if self.use_director:
            self.create_subscription(
                MarkerArray, self.get_parameter('candidates_topic').value,
                self._on_candidates, 1, callback_group=self.cb)
            self.rank_client = self.create_client(
                RankViewpoints, self.get_parameter('rank_service').value, callback_group=self.cb)

        self.state_pub = self.create_publisher(String, 'navflex_object_nav/state', 10)
        self.create_subscription(String, 'navflex_object_nav/target', self._on_target, 10, callback_group=self.cb)
        self.create_subscription(Empty, 'navflex_object_nav/stop', self._on_stop, 10, callback_group=self.cb)

        self._abort = threading.Event()
        self._busy = threading.Lock()
        self._ranked_candidate_xy = []

        # Some forks' DummyBehavior goal also carries a `behavior` selector.
        self._behavior_has_selector = 'behavior' in DummyBehavior.Goal.get_fields_and_field_types()

        self.get_logger().info(
            f"Object-goal navigator ready. Publish a target on 'navflex_object_nav/target'. "
            f"scan_steps={self.scan_steps} max_vantages={self.max_vantages} "
            f"relocate={'world-model director' if self.use_director else 'FrontierAStar top-1'}")

    def _on_candidates(self, msg: MarkerArray) -> None:
        self._latest_candidates = msg

    # ---- triggers -----------------------------------------------------------

    def _on_target(self, msg: String) -> None:
        target = (msg.data or '').strip()
        if not target:
            return
        if not self._busy.acquire(blocking=False):
            self._emit('busy', target, note='a search is already running; send stop first')
            return
        self._abort.clear()
        threading.Thread(target=self._run, args=(target,), daemon=True).start()

    def _on_stop(self, _msg: Empty) -> None:
        self._abort.set()
        self._emit('aborting', '', note='stop requested')

    # ---- state machine ------------------------------------------------------

    def _run(self, target: str) -> None:
        detection_target = self._detection_target(target)
        self._ranked_candidate_xy = []
        try:
            self._emit('searching', target, note='starting search',
                       detection_target=detection_target)
            for vantage in range(self.max_vantages):
                if self._abort.is_set():
                    return self._emit('aborted', target)
                # rotate-scan the current spot
                if self._scan_here(target, detection_target, vantage):
                    return  # FOUND (emitted inside)
                if self._abort.is_set():
                    return self._emit('aborted', target)
                # nothing here -> relocate to the next viewpoint
                self._emit('relocating', target, vantage=vantage,
                           note=('director' if self.use_director else 'frontier') + ': choosing next viewpoint')
                path = self._relocate(target, vantage)
                if path is None or not path.poses:
                    return self._emit('gave_up', target, vantage=vantage,
                                      note='no frontiers left (explored the reachable map)')
                if not self._follow(path):
                    return self._emit('gave_up', target, vantage=vantage, note='could not drive to next viewpoint')
            self._emit('gave_up', target, note=f'target not found after {self.max_vantages} vantages')
        finally:
            self._busy.release()

    @staticmethod
    def _detection_target(target: str) -> str:
        """Return the core object name from ``object｜spatial instruction``.

        The full string remains the world-model navigation instruction.  The
        real-frame grounder receives only the object before the delimiter, so a
        landmark need not be in the same camera frame to confirm the object.
        Legacy undelimited targets keep their previous behaviour.
        """
        core, separator, _context = target.partition('｜')
        return core.strip() if separator and core.strip() else target.strip()

    def _scan_here(self, target: str, detection_target: str, vantage: int) -> bool:
        """Rotate through the full circle; on detection, centre + approach. Returns True if FOUND."""
        for step in range(self.scan_steps):
            if self._abort.is_set():
                return False
            self._emit('scanning', target, vantage=vantage, step=f'{step + 1}/{self.scan_steps}')
            det = self._detect(detection_target)
            if det is not None and det.visible and det.confidence >= self.conf_thresh:
                self._emit('spotted', target, vantage=vantage, step=step,
                           confidence=det.confidence, bearing_deg=det.bearing_deg)
                # Centre the target so ranging/approach are dead ahead.  A
                # narrow doorway can occlude an edge detection after this turn;
                # retain the last *real-frame* visible heading for one grounded
                # recovery attempt rather than discarding a valid sighting.
                centred = False
                if abs(det.bearing_rad) > self.center_tol:
                    centred = self._rotate(det.bearing_rad)
                    if centred:
                        time.sleep(self.settle_time)
                approach = self._approach(detection_target)
                if self._approach_succeeded(approach):
                    self._emit('found', target, vantage=vantage,
                               note='arrived in front of the target')
                    return True
                if (self.reacquire_last_seen and centred and approach is not None
                        and not approach.visible and not self._abort.is_set()):
                    self._emit(
                        'reacquiring', target, vantage=vantage,
                        note='target lost after centering; restoring last visible heading',
                        restore_deg=round(-math.degrees(det.bearing_rad), 1))
                    if self._rotate(-det.bearing_rad):
                        time.sleep(self.settle_time)
                        approach = self._approach(detection_target)
                        if self._approach_succeeded(approach):
                            self._emit('found', target, vantage=vantage,
                                       note='reacquired at last visible heading and arrived')
                            return True
                self._emit('approach_failed', target, vantage=vantage,
                           note=('saw it but could not approach; resuming scan'
                                 + (f' ({approach.message})'
                                    if approach is not None and approach.message else '')))
            # rotate to the next scan orientation
            if step < self.scan_steps - 1:
                self._rotate(self.scan_step_rad)
                time.sleep(self.settle_time)
        return False

    # ---- action / service helpers ------------------------------------------

    def _wait(self, future, label: str, timeout: float = None) -> bool:
        deadline = time.monotonic() + (timeout if timeout is not None else self.action_timeout)
        while rclpy.ok() and not future.done() and not self._abort.is_set():
            time.sleep(0.02)
            if time.monotonic() > deadline:
                self.get_logger().error(f'timed out waiting for {label}')
                return False
        return future.done()

    def _detect(self, target: str):
        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            return None
        deadline = time.monotonic() + self.detect_timeout
        while not self._abort.is_set() and time.monotonic() < deadline:
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

    @staticmethod
    def _approach_succeeded(response) -> bool:
        return bool(response and response.found and response.driven)

    def _approach(self, target: str):
        if not self.approach_client.wait_for_service(timeout_sec=5.0):
            return None
        req = ApproachTarget.Request()
        req.target = target
        req.dry_run = False
        req.skip_verify = True   # world-model safety verify is optional; off in simple search
        fut = self.approach_client.call_async(req)
        if not self._wait(fut, 'approach', timeout=self.approach_timeout):
            return None
        return fut.result()

    def _rotate(self, angle_rad: float) -> bool:
        if abs(angle_rad) < 1e-3:
            return True
        if not self.behavior_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('behavior_action server not available; cannot rotate')
            return False
        goal = DummyBehavior.Goal()
        goal.command.data = f'rotate {angle_rad:.4f}'
        if self._behavior_has_selector:
            goal.behavior = 'cmd_behavior'
        sf = self.behavior_client.send_goal_async(goal)
        if not self._wait(sf, 'rotate send'):
            return False
        h = sf.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        if not self._wait(rf, 'rotate result'):
            return False
        return True

    def _plan_frontier(self):
        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            return None
        g = ComputePathToPose.Goal()
        # FrontierAStar ignores the requested goal and picks the top viewpoint itself.
        g.goal = PoseStamped()
        g.goal.header.frame_id = self.global_frame
        g.goal.header.stamp = self.get_clock().now().to_msg()
        g.planner_id = self.frontier_planner_id
        g.use_start = False
        sf = self.plan_client.send_goal_async(g)
        if not self._wait(sf, 'frontier plan send'):
            return None
        h = sf.result()
        if h is None or not h.accepted:
            return None
        rf = h.get_result_async()
        if not self._wait(rf, 'frontier plan result'):
            return None
        wrapped = rf.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        return wrapped.result.path

    # ---- director relocation ------------------------------------------------

    def _relocate(self, target: str, vantage: int):
        """Choose the next viewpoint: world-model director if enabled + available,
        else FrontierAStar top-1. Returns a path to follow (or None)."""
        if self.use_director and self.rank_client is not None:
            cands = self._get_candidates()
            if cands:
                pick = self._rank_and_pick(target, cands, vantage)
                if pick is not None:
                    path = self._plan_to(pick)
                    if path is not None and path.poses:
                        return path
            self._emit('director_fallback', target, vantage=vantage,
                       note='director unavailable/failed; using FrontierAStar top-1')
        return self._plan_frontier()

    def _get_candidates(self):
        """Refresh + read frontier candidate positions; return up to
        max_rank_candidates nearest reachable ones as PoseStamped (facing them)."""
        self._plan_frontier()          # triggers a fresh /candidates publish
        time.sleep(0.6)
        marker = self._latest_candidates
        pose = self._robot_pose()
        if marker is None or pose is None:
            return []
        rx, ry, _ = pose
        pts = []
        for m in marker.markers:
            if m.type == Marker.SPHERE_LIST:
                pts.extend(m.points)
        scored = [(math.hypot(p.x - rx, p.y - ry),
                   math.atan2(p.y - ry, p.x - rx), p) for p in pts]
        scored = [(d, b, p) for d, b, p in scored
                  if d >= self.min_candidate_dist and not any(
                      math.hypot(p.x - old_x, p.y - old_y)
                      < self.candidate_revisit_radius
                      for old_x, old_y in self._ranked_candidate_xy)]
        scored.sort(key=lambda t: t[0])
        selected = []
        # Prefer the nearest point in distinct angular sectors.  Pure nearest-K
        # repeatedly concentrated all rollouts in one open room and omitted
        # narrow doorways; directional diversity gives the director meaningful
        # spatial alternatives without changing K or the frontier generator.
        for item in scored:
            _d, bearing, _p = item
            if all(abs(math.atan2(math.sin(bearing - old_bearing),
                                  math.cos(bearing - old_bearing)))
                   >= self.candidate_min_bearing_sep
                   for _old_d, old_bearing, _old_p in selected):
                selected.append(item)
                if len(selected) >= self.max_rank_candidates:
                    break
        if len(selected) < self.max_rank_candidates:
            selected_ids = {id(item[2]) for item in selected}
            selected.extend(item for item in scored
                            if id(item[2]) not in selected_ids)
            selected = selected[:self.max_rank_candidates]
        out = []
        for _d, _bearing, p in selected:
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            yaw = math.atan2(p.y - ry, p.x - rx)   # face the candidate
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            out.append(ps)
        return out

    def _rank_and_pick(self, target, candidates, vantage):
        if not self.rank_client.wait_for_service(timeout_sec=5.0):
            return None
        req = RankViewpoints.Request()
        req.target = target
        req.candidates = candidates
        req.frame_num = self.rank_frame_num
        fut = self.rank_client.call_async(req)
        if not self._wait(fut, 'rank_viewpoints', timeout=self.rank_timeout):
            return None
        res = fut.result()
        if res is None or not res.ok or res.best_index < 0 or res.best_index >= len(candidates):
            return None
        scores = [float(s) for s in res.scores]
        reasons = list(res.reasons)
        best_score = scores[res.best_index]
        if best_score < self.director_min_score:
            self._emit(
                'director_rejected', target, vantage=vantage,
                best_index=res.best_index, best_score=round(best_score, 2),
                threshold=self.director_min_score,
                scores=[round(s, 2) for s in scores], reasons=reasons,
                note='all imagined directions were weak; using geometry fallback')
            return None
        # Only the viewpoint the robot will actually visit is suppressed later.
        # Removing every ranked-but-unvisited candidate could permanently hide
        # the correct doorway after one imperfect world-model prediction.
        chosen = candidates[res.best_index]
        self._ranked_candidate_xy.append(
            (chosen.pose.position.x, chosen.pose.position.y))
        self._emit('ranked', target, vantage=vantage, best_index=res.best_index,
                   scores=[round(s, 2) for s in scores], reasons=reasons,
                   rollout_frames=self.rank_frame_num)
        return chosen

    def _plan_to(self, goal: PoseStamped):
        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            return None
        g = ComputePathToPose.Goal()
        g.goal = goal
        g.planner_id = 'GridBased'
        g.use_start = False
        sf = self.plan_client.send_goal_async(g)
        if not self._wait(sf, 'plan_to send'):
            return None
        h = sf.result()
        if h is None or not h.accepted:
            return None
        rf = h.get_result_async()
        if not self._wait(rf, 'plan_to result'):
            return None
        wrapped = rf.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        return wrapped.result.path

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return (tf.transform.translation.x, tf.transform.translation.y,
                _yaw_from_quat(tf.transform.rotation))

    def _follow(self, path) -> bool:
        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            return False
        g = FollowPath.Goal()
        g.path = path
        g.controller_id = 'FollowPath'
        sf = self.follow_client.send_goal_async(g)
        if not self._wait(sf, 'follow send'):
            return False
        h = sf.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        if not self._wait(rf, 'follow result'):
            return False
        wrapped = rf.result()
        return wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED

    def _emit(self, state: str, target: str, **kw) -> None:
        payload = {'stamp': time.time(), 'state': state, 'target': target}
        payload.update(kw)
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(f'[{state}] {target} ' + ' '.join(f'{k}={v}' for k, v in kw.items()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectGoalNavigator()
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
