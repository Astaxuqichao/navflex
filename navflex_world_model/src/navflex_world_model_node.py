#!/usr/bin/env python3
"""Navflex world-model gate.

Sits between the task layer and execution. Given a grounded goal it:

  1. plans with compute_path_to_pose -- and never executes the plan,
  2. converts that plan into the camera trajectory the robot would fly,
  3. rolls a world model forward along it from the frame the robot sees now,
  4. asks a critic whether the imagined future is safe and on-instruction.

The verdict feeds `navflex_task_server`'s existing confirmation gate. Nothing
here drives the base: rolling a video diffusion model forward costs seconds to
minutes, which is four orders of magnitude off nav2's 20 Hz control loop. This
is a deliberation layer, not a controller.
"""

import json
import math
import threading
import time
import traceback
from dataclasses import replace
from typing import List, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from navflex_wm.backends import WorldModelError, build_backend
from navflex_wm.coherence import coherent_prefix
from navflex_wm.critic import CriticError, Verdict, build_critic
from navflex_wm.director import DirectorError, build_director
from navflex_wm.pose_utils import (
    MAX_COHERENT_FRAMES,
    MAX_DEG_PER_FRAME,
    METRES_PER_FRAME,
    MIN_COHERENT_FRAMES,
    CameraExtrinsic,
    PoseConversionError,
    budget_for_plan,
    plan_to_control_signal,
    quantize_frame_num,
    quaternion_to_yaw,
)
from navflex_world_model.srv import EvaluatePlan, RankViewpoints

# Below this many coherent frames there is no motion left to judge -- the critic
# would be reading a still image and calling it a prediction.
MIN_JUDGEABLE_FRAMES = 4


class NavflexWorldModel(Node):
    def __init__(self) -> None:
        super().__init__('navflex_world_model')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('action_timeout', 30.0)

        # MATRiX publishes the xgb ego camera here (config/config.json).
        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('image_max_age', 2.0)
        self.declare_parameter('camera_fov_deg', 90.0)
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('camera_offset_x', 0.29)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.01)
        self.declare_parameter('camera_roll_deg', 0.0)
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('camera_yaw_deg', 0.0)

        self.declare_parameter('backend', 'null')
        self.declare_parameter('lingbot_url', 'http://127.0.0.1:8100')
        self.declare_parameter('backend_timeout', 600.0)
        # 0 -> size the rollout from the plan's total turn. A fixed frame count
        # either wastes frames on a straight run or tumbles on a sharp one.
        self.declare_parameter('frame_num', 0)
        self.declare_parameter('max_deg_per_frame', MAX_DEG_PER_FRAME)
        # Both are snapped onto the frame-count grid LingBot renders in full
        # (13, 29, 45, ...) at startup -- an off-grid ceiling is a lie, because
        # the pipeline floors it and silently truncates the trajectory.
        self.declare_parameter('min_frame_num', MIN_COHERENT_FRAMES)
        # Longest rollout that stays coherent: past this the model drifts across
        # chunks even on a dead-straight path. See pose_utils.
        self.declare_parameter('max_frame_num', MAX_COHERENT_FRAMES)
        # Metres the imagined robot advances per frame. Calibrated against
        # MATRiX odometry; see pose_utils.METRES_PER_FRAME. Frame count is the
        # only distance knob the model has, so this is what lets the gate say
        # how much of the plan it actually imagined.
        self.declare_parameter('metres_per_frame', METRES_PER_FRAME)
        # 1 -> the sidecar returns every frame and `critic_max_frames` alone
        # decides what the critic sees. Two independent samplers compound: a
        # stride of 8 over a 13-frame rollout left the critic 2 frames, and it
        # was the *stride* that chose them, not the evenly-spaced sampler.
        # Rollouts are capped at 29 frames, so the whole set is a couple of MB.
        self.declare_parameter('frame_sample_stride', 1)
        self.declare_parameter('rollout_prompt', '')

        # claude | openai_compat | openai | codex_cli | null
        self.declare_parameter('critic', 'null')
        # Empty means "whatever the chosen backend defaults to".
        self.declare_parameter('critic_model', '')
        self.declare_parameter('critic_api_key_env', '')
        # openai_compat: the aggregator's OpenAI-compatible root, e.g.
        # https://example.com/v1  (chat/completions is appended).
        self.declare_parameter('critic_base_url', '')
        self.declare_parameter('critic_api_url', 'https://api.openai.com/v1/responses')
        self.declare_parameter('critic_proxy_env', 'HTTPS_PROXY')
        self.declare_parameter('critic_timeout', 120.0)
        self.declare_parameter('critic_max_frames', 6)
        # Claude only: how hard to think about the verdict.
        self.declare_parameter('critic_effort', 'medium')

        # What to answer when the gate cannot form an opinion. Fails closed:
        # a safety gate that approves on its own failure is not a safety gate.
        self.declare_parameter('unavailable_verdict', 'needs_confirmation')
        self.declare_parameter('min_confidence', 0.0)
        # A plan this short moves the camera nowhere, so the rollout would just
        # re-render the current frame. Not worth a diffusion pass.
        self.declare_parameter('min_path_length', 0.2)

        self.global_frame = self.get_parameter('global_frame').value
        self.planner_id = self.get_parameter('planner_id').value
        self.action_timeout = float(self.get_parameter('action_timeout').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.image_max_age = float(self.get_parameter('image_max_age').value)
        self.camera_fov_deg = float(self.get_parameter('camera_fov_deg').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.default_frame_num = int(self.get_parameter('frame_num').value)
        self.max_deg_per_frame = float(self.get_parameter('max_deg_per_frame').value)
        # Snap onto the grid before anything derives a turn budget from these.
        self.min_frame_num = quantize_frame_num(
            int(self.get_parameter('min_frame_num').value))
        self.max_frame_num = quantize_frame_num(
            int(self.get_parameter('max_frame_num').value))
        self.metres_per_frame = float(self.get_parameter('metres_per_frame').value)
        self.rollout_prompt = self.get_parameter('rollout_prompt').value
        self.unavailable_verdict = self.get_parameter('unavailable_verdict').value
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.min_path_length = float(self.get_parameter('min_path_length').value)

        self.extrinsic = CameraExtrinsic(
            x=float(self.get_parameter('camera_offset_x').value),
            y=float(self.get_parameter('camera_offset_y').value),
            z=float(self.get_parameter('camera_offset_z').value),
            roll=math.radians(float(self.get_parameter('camera_roll_deg').value)),
            pitch=math.radians(float(self.get_parameter('camera_pitch_deg').value)),
            yaw=math.radians(float(self.get_parameter('camera_yaw_deg').value)))

        self.backend = build_backend(
            self.get_parameter('backend').value,
            lingbot_url=self.get_parameter('lingbot_url').value,
            timeout=float(self.get_parameter('backend_timeout').value),
            sample_stride=int(self.get_parameter('frame_sample_stride').value))
        self.critic = self._build_critic()
        # Phase-A director: ranks candidate viewpoints by "heads toward target".
        # Reuses the critic's VLM backend (same base_url/model/key), different prompt.
        self.director = self._build_director()

        self.callback_group = ReentrantCallbackGroup()
        self.planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=self.callback_group)

        # Camera frames are large and lossy to buffer; keep only the newest.
        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            CompressedImage, self.image_topic, self._on_image, image_qos,
            callback_group=self.callback_group)

        self.event_pub = self.create_publisher(String, 'navflex_world_model/events', 10)
        self.create_service(
            EvaluatePlan, 'navflex_world_model/evaluate', self.evaluate_plan,
            callback_group=self.callback_group)
        self.create_service(
            RankViewpoints, 'navflex_world_model/rank_viewpoints', self.rank_viewpoints,
            callback_group=self.callback_group)

        self._image_lock = threading.Lock()
        self._latest_image: Optional[bytes] = None
        self._latest_image_stamp = 0.0
        # A rollout pins ~37 GB of GPU weights; overlapping calls would thrash.
        self._evaluate_lock = threading.Lock()

        critic_health = self.critic.health()
        if not critic_health.startswith('ok'):
            # Otherwise a bad key or dead proxy just makes every task ask for
            # confirmation, and nobody knows why.
            self.get_logger().error(
                f"critic '{self.critic.name}' is not usable: {critic_health}. "
                f"Every plan will come back as '{self.unavailable_verdict}'.")
        self.get_logger().info(
            f"World model gate ready. backend='{self.backend.name}' "
            f"({self.backend.health()}) critic='{self.critic.name}' "
            f"({critic_health}) "
            f"image_topic='{self.image_topic}' frame_num={self.default_frame_num} "
            f"unavailable_verdict='{self.unavailable_verdict}'")

    # Each backend reads its key from a different env var by default.
    DEFAULT_KEY_ENV = {
        'claude': 'ANTHROPIC_API_KEY',
        'openai': 'OPENAI_API_KEY',
        'openai_compat': 'NAVFLEX_CRITIC_API_KEY',
    }

    def _build_critic(self):
        import os

        kind = (self.get_parameter('critic').value or 'null').strip().lower()
        key_env = (self.get_parameter('critic_api_key_env').value
                   or self.DEFAULT_KEY_ENV.get(kind, ''))
        proxy_env = self.get_parameter('critic_proxy_env').value
        # Shells set https_proxy or HTTPS_PROXY interchangeably; honour both.
        proxy = (os.environ.get(proxy_env)
                 or os.environ.get(proxy_env.lower(), '')) if proxy_env else ''
        if kind in ('claude', 'openai', 'openai_compat'):
            self.get_logger().info(
                f"critic '{kind}': key_env='{key_env}' "
                f"({'set' if os.environ.get(key_env) else 'unset'}), "
                f"proxy={'yes' if proxy else 'no'}")
        try:
            return build_critic(
                kind,
                api_url=self.get_parameter('critic_api_url').value,
                base_url=self.get_parameter('critic_base_url').value,
                # A blank key lets the Anthropic SDK fall back to an `ant auth
                # login` profile; OpenAICritic rejects it, which is what we want.
                api_key=os.environ.get(key_env, '') if key_env else '',
                model=self.get_parameter('critic_model').value or None,
                effort=self.get_parameter('critic_effort').value,
                proxy=proxy,
                timeout=float(self.get_parameter('critic_timeout').value),
                max_frames=int(self.get_parameter('critic_max_frames').value))
        except CriticError as exc:
            self.get_logger().warn(
                f"critic '{kind}' unavailable ({exc}); falling back to null critic, "
                f'which never approves')
            return build_critic('null')

    def _build_director(self):
        import os

        # The director shares the critic's VLM backend (same base_url/model/key)
        # with a different prompt. director='' (default) -> follow the critic kind.
        self.declare_parameter('director', '')
        # ★ kimi-k2.5 (via luchentech) returns an EMPTY completion when handed 6
        #   images in one request; 3 works fine (measured). The rollout is short, so
        #   start/mid/end is plenty to judge "does this direction head toward target".
        self.declare_parameter('director_max_frames', 3)
        kind = (self.get_parameter('director').value
                or self.get_parameter('critic').value or 'null').strip().lower()
        key_env = (self.get_parameter('critic_api_key_env').value
                   or self.DEFAULT_KEY_ENV.get(kind, ''))
        proxy_env = self.get_parameter('critic_proxy_env').value
        proxy = (os.environ.get(proxy_env)
                 or os.environ.get(proxy_env.lower(), '')) if proxy_env else ''
        try:
            return build_director(
                kind,
                base_url=self.get_parameter('critic_base_url').value,
                model=self.get_parameter('critic_model').value or None,
                api_key=os.environ.get(key_env, '') if key_env else '',
                proxy=proxy,
                timeout=float(self.get_parameter('critic_timeout').value),
                max_frames=int(self.get_parameter('director_max_frames').value))
        except DirectorError as exc:
            self.get_logger().warn(
                f"director '{kind}' unavailable ({exc}); falling back to null director")
            return build_director('null')

    def _on_image(self, msg: CompressedImage) -> None:
        with self._image_lock:
            self._latest_image = bytes(msg.data)
            self._latest_image_stamp = time.monotonic()

    def _take_image(self) -> Optional[bytes]:
        with self._image_lock:
            if self._latest_image is None:
                return None
            age = time.monotonic() - self._latest_image_stamp
            if age > self.image_max_age:
                self.get_logger().warn(
                    f'latest camera frame is {age:.1f}s old (max {self.image_max_age:.1f}s)')
                return None
            return self._latest_image

    def evaluate_plan(self, request, response):
        start = self.get_clock().now()
        steps: List[str] = []

        if not self._evaluate_lock.acquire(blocking=False):
            return self._finish(
                response, steps, start, success=False, verdict='unavailable',
                reason='another plan is still being evaluated')

        try:
            goal = request.goal
            steps.append(
                f'evaluating goal ({goal.pose.position.x:.3f}, '
                f'{goal.pose.position.y:.3f}) in {goal.header.frame_id or self.global_frame}')

            path = self._plan(goal, steps)
            if path is None:
                return self._finish(
                    response, steps, start, success=False, verdict='unavailable',
                    reason='compute_path_to_pose failed; nothing to imagine')

            length = self._path_length(path)
            if length < self.min_path_length:
                # The robot is already there. An imagined rollout would just
                # redraw the current frame, so approve rather than burn a pass.
                steps.append(
                    f'plan is {length:.3f} m (< {self.min_path_length:.2f} m); '
                    f'no motion to imagine')
                return self._finish(
                    response, steps, start, success=True, verdict='approve',
                    reason=f'plan is only {length:.3f} m; robot is already at the goal',
                    confidence=1.0)

            try:
                c2ws, intrinsics, budget = self._plan_to_control_signal(
                    path, goal, int(request.frame_num) or self.default_frame_num)
            except PoseConversionError as exc:
                return self._finish(
                    response, steps, start, success=False, verdict='unavailable',
                    reason=f'pose conversion failed: {exc}')
            if budget.truncated:
                coverage = (f'reaching only {budget.covered_m:.2f} m of a '
                            f'{budget.plan_m:.2f} m plan')
            elif budget.covered_m > budget.plan_m + 0.05:
                coverage = (f'depicting {budget.covered_m:.2f} m, overshooting the '
                            f'{budget.plan_m:.2f} m plan by '
                            f'{budget.covered_m - budget.plan_m:.2f} m')
            else:
                coverage = f'depicting the whole {budget.plan_m:.2f} m plan'
            steps.append(
                f'camera trajectory: {budget.frame_num} frames {coverage}, '
                f'{budget.turn_deg:.0f} deg of turning '
                f'({budget.deg_per_frame:.2f} deg/frame)')

            # Frames are distance AND yaw rate at once: the rollout advances a
            # fixed ~0.19 m per frame, so a turn cannot be slowed by adding
            # frames without also walking further. Both failures below mean the
            # model cannot depict this plan. Say so, rather than spend minutes
            # generating something and then judging it as if it were evidence.
            if budget.too_curvy:
                return self._finish(
                    response, steps, start, success=True,
                    verdict='needs_confirmation',
                    reason=(f'plan turns {budget.turn_deg:.0f} deg over '
                            f'{budget.plan_m:.2f} m ({budget.deg_per_frame:.2f} '
                            f'deg/frame, limit {budget.max_deg_per_frame:.2f}). '
                            f'The world model cannot imagine a turn that tight: '
                            f'frames buy distance, not slower turning. No rollout '
                            f'was generated, so there is no evidence to judge.'))
            if budget.truncated:
                return self._finish(
                    response, steps, start, success=True,
                    verdict='needs_confirmation',
                    reason=(f'plan is {budget.plan_m:.2f} m but the world model '
                            f'stays coherent for only about {budget.covered_m:.2f} m '
                            f'({budget.frame_num} frames). The far end of the path '
                            f'would never be imagined, so approving it would mean '
                            f'vetting a route on partial evidence.'))

            image = self._take_image()
            if image is None:
                return self._finish(
                    response, steps, start, success=False, verdict='unavailable',
                    reason=f'no recent frame on {self.image_topic}')

            # The instruction goes to the critic, not to the world model.
            prompt = self.rollout_prompt or self._default_prompt()
            steps.append(f"imagining with backend '{self.backend.name}'")
            self._publish_event('imagining', prompt, {'frames': len(c2ws)})
            try:
                rollout = self.backend.imagine(image, c2ws, intrinsics, prompt)
            except WorldModelError as exc:
                return self._finish(
                    response, steps, start, success=False, verdict='unavailable',
                    reason=f'world model failed: {exc}')
            steps.append(
                f'imagined {rollout.frame_count} frames'
                + (f' -> {rollout.video_path}' if rollout.video_path else ''))
            response.rollout_uri = rollout.video_path

            if request.dry_run:
                steps.append('dry_run: skipping critic')
                return self._finish(
                    response, steps, start, success=True, verdict='needs_confirmation',
                    reason='dry_run: rollout produced, not judged')

            # The rollout overshoots the goal (13/29/45 frames are the only
            # renderable lengths, so 4.00 m renders as 5.26 m). Frames past the
            # goal show a journey nobody asked for; an obstacle there must not
            # veto this task.
            frames = self._frames_up_to_goal(rollout.frames, budget)
            if len(frames) < rollout.frame_count:
                steps.append(
                    f'judging the first {len(frames)} of {rollout.frame_count} '
                    f'frames: the rest overshoot the goal by '
                    f'{budget.covered_m - budget.plan_m:.2f} m')

            # The model has no physics of contact. Driven into a cabinet it
            # renders the cabinet, then -- with nothing plausible left to draw
            # -- cuts to an invented scene. Those frames are not evidence: a
            # critic once rejected a plan citing a hallucinated robot rather
            # than the cabinet that was actually there. Keep the continuous
            # prefix; never hand a critic frames the model made up.
            frames, tear = coherent_prefix(frames)
            torn_before_goal = tear > 0
            if torn_before_goal:
                torn_at_m = (tear - 1) * self.metres_per_frame
                steps.append(
                    f'rollout lost coherence at frame {tear} (~{torn_at_m:.2f} m); '
                    f'judging only the {len(frames)} frames before it')
                if len(frames) < MIN_JUDGEABLE_FRAMES:
                    return self._finish(
                        response, steps, start, success=True,
                        verdict='needs_confirmation',
                        reason=(f'the world model lost coherence after only '
                                f'{torn_at_m:.2f} m, leaving {len(frames)} usable '
                                f'frames. There is not enough evidence to judge '
                                f'this plan.'))

            steps.append(f"critiquing with '{self.critic.name}'")
            try:
                verdict = self.critic.critique(
                    frames, request.instruction, self._plan_summary(path))
            except CriticError as exc:
                return self._finish(
                    response, steps, start, success=False, verdict='unavailable',
                    reason=f'critic failed: {exc}')

            # A rollout that tore before the goal never showed the rest of the
            # path. The critic may honestly approve what it was shown; that is
            # not the same as the plan being safe. Reject stands -- the critic
            # saw a reason -- but approval does not.
            if torn_before_goal and verdict.verdict == 'approve':
                torn_at_m = (tear - 1) * self.metres_per_frame
                steps.append(
                    'approval downgraded: the rollout tore before reaching the goal')
                verdict = Verdict(
                    verdict='needs_confirmation', confidence=verdict.confidence,
                    reason=(f'the critic approved the first {torn_at_m:.2f} m, but the '
                            f'world model lost coherence there and never imagined the '
                            f'rest of the {budget.plan_m:.2f} m plan. '
                            f'Original reason: {verdict.reason}'),
                    hazards=verdict.hazards, source=verdict.source)

            # Only when the critic actually gave a number. An unconstrained model
            # returns no confidence at all, and Verdict fills 0.0 -- comparing
            # that against min_confidence would block every task on a figure the
            # critic never stated.
            if (verdict.verdict == 'approve' and verdict.confidence_reported
                    and verdict.confidence < self.min_confidence):
                steps.append(
                    f'approval downgraded: confidence {verdict.confidence:.2f} '
                    f'< min_confidence {self.min_confidence:.2f}')
                verdict = Verdict(
                    'needs_confirmation', verdict.confidence,
                    f'{verdict.reason} (confidence below threshold)',
                    verdict.hazards, verdict.source)

            response.critic_json = json.dumps(verdict.to_dict(), ensure_ascii=False)
            steps.append(f'verdict: {verdict.verdict} ({verdict.confidence:.2f})')
            self._publish_event(verdict.verdict, verdict.reason, verdict.to_dict())
            return self._finish(
                response, steps, start, success=True, verdict=verdict.verdict,
                reason=verdict.reason, confidence=verdict.confidence)
        except Exception as exc:  # keep the service alive
            self.get_logger().error(f'world model gate failed:\n{traceback.format_exc()}')
            return self._finish(
                response, steps, start, success=False, verdict='unavailable',
                reason=f'internal error: {exc}')
        finally:
            self._evaluate_lock.release()

    def rank_viewpoints(self, request, response):
        """Phase-A director: imagine a rollout toward each candidate viewpoint and
        score how likely it heads toward the target; return the best index."""
        start = self.get_clock().now()
        candidates = list(request.candidates)
        response.best_index = -1
        response.scores = []
        response.reasons = []
        if not candidates:
            response.ok = False
            response.message = 'no candidates'
            return response

        # A rank does several imagines; it must not overlap an evaluate or another
        # rank (both pin the GPU weights).
        if not self._evaluate_lock.acquire(blocking=False):
            response.ok = False
            response.message = 'world model busy'
            return response
        try:
            image = self._take_image()
            if image is None:
                response.ok = False
                response.message = f'no recent frame on {self.image_topic}'
                return response

            target = request.target or ''
            prompt = self.rollout_prompt or self._default_prompt()
            frame_num = int(request.frame_num) or self.default_frame_num
            scores: List[float] = []
            reasons: List[str] = []
            n = len(candidates)
            for i, cand in enumerate(candidates):
                candidate_started = time.monotonic()
                # Emit progress before the ~90s imagine so callers (and the CLI)
                # can see WHICH candidate is being worked on, not just silence.
                self._publish_event(
                    'ranking',
                    f'imagining candidate {i + 1}/{n} toward '
                    f'({cand.pose.position.x:.1f}, {cand.pose.position.y:.1f}) [~90s]',
                    {'i': i + 1, 'total': n})
                steps: List[str] = []
                path = self._plan(cand, steps)
                if path is None:
                    scores.append(-1.0)
                    reasons.append('plan failed')
                    self._publish_event('ranked_one', f'candidate {i + 1}/{n}: plan failed',
                                        {'i': i + 1, 'total': n, 'score': -1.0,
                                         'elapsed_s': round(
                                             time.monotonic() - candidate_started, 3),
                                         'frame_count': 0, 'reason': 'plan failed'})
                    continue
                frame_count = 0
                try:
                    c2ws, intrinsics, _budget = self._plan_to_control_signal(
                        path, cand, frame_num)
                    rollout = self.backend.imagine(image, c2ws, intrinsics, prompt)
                    frame_count = rollout.frame_count
                    # A flaky VLM response for ONE candidate must not kill the whole
                    # ranking — score it -1 and move on.
                    ds = self.director.score(rollout.frames, target)
                # OSError covers any backend that leaks a raw network/OS error;
                # one bad candidate must never abort the whole ranking.
                except (PoseConversionError, WorldModelError, DirectorError, OSError) as exc:
                    scores.append(-1.0)
                    reasons.append(f'failed: {exc}')
                    self._publish_event('ranked_one', f'candidate {i + 1}/{n}: failed ({exc})',
                                        {'i': i + 1, 'total': n, 'score': -1.0,
                                         'elapsed_s': round(
                                             time.monotonic() - candidate_started, 3),
                                         'frame_count': frame_count,
                                         'reason': f'failed: {exc}'})
                    continue
                scores.append(ds.score)
                reasons.append(ds.reason)
                self._publish_event(
                    'ranked_one', f'candidate {i + 1}/{n}: score {ds.score:.2f}',
                    {'i': i + 1, 'total': n, 'score': ds.score,
                     'elapsed_s': round(time.monotonic() - candidate_started, 3),
                     'frame_count': frame_count, 'reason': ds.reason})
                self.get_logger().info(
                    f'[rank] candidate {i} '
                    f'({cand.pose.position.x:.2f},{cand.pose.position.y:.2f}) '
                    f'score={ds.score:.2f} | {ds.reason}')

            valid = [(s, i) for i, s in enumerate(scores) if s >= 0.0]
            # Stable tie-break: Python's tuple max preferred the later index
            # whenever director scores were equal (e.g. [0.2, 0.1, 0.2]).
            # Keying only on score preserves the first candidate on a tie.
            best_index = max(valid, key=lambda item: item[0])[1] if valid else -1
            response.ok = bool(valid)
            response.best_index = best_index
            response.scores = scores
            response.reasons = reasons
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            response.message = (
                f'ranked {len(candidates)} candidates in {elapsed:.0f}s, best={best_index}'
                + (f' (score {scores[best_index]:.2f})' if best_index >= 0 else ' (none scored)'))
            self._publish_event(
                'rank_complete', response.message,
                {'total': len(candidates), 'elapsed_s': round(elapsed, 3),
                 'best_index': best_index, 'scores': scores, 'reasons': reasons})
            return response
        except Exception as exc:  # keep the service alive
            response.ok = False
            response.message = f'internal error: {exc}'
            self.get_logger().error(f'rank_viewpoints failed:\n{traceback.format_exc()}')
            return response
        finally:
            self._evaluate_lock.release()

    def _plan(self, goal, steps: List[str]):
        if not self.planner_client.server_is_ready():
            steps.append('waiting for compute_path_to_pose')
            if not self.planner_client.wait_for_server(timeout_sec=self.action_timeout):
                steps.append('compute_path_to_pose is not available')
                return None

        if not goal.header.frame_id:
            goal.header.frame_id = self.global_frame

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = goal
        plan_goal.planner_id = self.planner_id
        plan_goal.use_start = False

        steps.append(f"planning (no execution) with planner_id='{self.planner_id}'")
        send_future = self.planner_client.send_goal_async(plan_goal)
        if not self._wait(send_future, 'compute_path_to_pose send_goal'):
            return None
        handle = send_future.result()
        if handle is None or not handle.accepted:
            steps.append('planner rejected the goal')
            return None

        result_future = handle.get_result_async()
        if not self._wait(result_future, 'compute_path_to_pose result'):
            return None
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            steps.append(f'planner failed (status={getattr(wrapped, "status", "?")})')
            return None

        path = wrapped.result.path
        if not path.poses:
            steps.append('planner returned an empty path')
            return None
        steps.append(f'planner returned {len(path.poses)} poses')
        return path

    def _plan_to_control_signal(self, path, goal, frame_num):
        """Returns ``(c2ws, intrinsics, budget)``."""
        positions = np.array(
            [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
             for p in path.poses], dtype=np.float64)
        q = goal.pose.orientation
        final_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        budget = budget_for_plan(
            positions, final_yaw=final_yaw,
            max_deg_per_frame=self.max_deg_per_frame,
            min_frames=self.min_frame_num, max_frames=self.max_frame_num,
            metres_per_frame=self.metres_per_frame)
        if frame_num:
            # An explicit request overrides the sizing but not the grid, and the
            # coverage it implies is still reported honestly.
            frame_num = quantize_frame_num(frame_num)
            budget = replace(
                budget, frame_num=frame_num,
                covered_m=(frame_num - 1) * self.metres_per_frame,
                deg_per_frame=budget.turn_deg / max(frame_num - 1, 1))
        else:
            frame_num = budget.frame_num
        # level_camera=True: the 15 deg mount tilt is invisible to the model
        # (frame 0 is forced to identity) but its consequences are not -- it
        # reads the tilt's translation component as ascending, and flies into
        # the ceiling. See pose_utils.level_for_conditioning.
        c2ws, intrinsics = plan_to_control_signal(
            positions, frame_num, self.extrinsic, self.camera_fov_deg,
            self.camera_width, self.camera_height, final_yaw=final_yaw,
            level_camera=True)
        return c2ws, intrinsics, budget

    @staticmethod
    def _frames_up_to_goal(frames, budget):
        """Trim the returned frames to the stretch the plan actually covers.

        The backend may have subsampled, so work in fractions of the rollout
        rather than assuming one returned frame per rendered frame. Always keep
        at least two: a single frame carries no motion for the critic to judge.
        """
        if not frames or budget.frame_num <= 1:
            return frames
        fraction = budget.frames_to_goal / budget.frame_num
        keep = int(round(len(frames) * fraction))
        return frames[:max(2, min(len(frames), keep))]

    @staticmethod
    def _path_length(path) -> float:
        poses = path.poses
        return sum(
            math.dist(
                (poses[i].pose.position.x, poses[i].pose.position.y),
                (poses[i + 1].pose.position.x, poses[i + 1].pose.position.y))
            for i in range(len(poses) - 1))

    def _plan_summary(self, path) -> str:
        first, last = path.poses[0].pose.position, path.poses[-1].pose.position
        return (f'{len(path.poses)} 个路点，全长 {self._path_length(path):.2f} m，'
                f'从 ({first.x:.2f}, {first.y:.2f}) 到 ({last.x:.2f}, {last.y:.2f})')

    def _default_prompt(self) -> str:
        """What the world model is asked to imagine.

        Deliberately says nothing about the task. The forward model answers a
        question of geometry -- "what does the camera see if the robot follows
        this path?" -- and the critic then judges whether that matches the
        instruction. Conditioning the diffusion on the instruction as well
        invites it to draw the outcome the user asked for, and the gate becomes
        an echo chamber: ask it to climb a staircase and it may render one.

        Measured before this was removed: appending 'Task: {instruction}' to the
        prompt changed the late frames of the rollout by a mean of 28.4 (larger
        than the 11.6 mean difference between adjacent frames of one clip), for
        the same plan and the same seed image. It did not hallucinate a
        staircase that time. It is not a risk worth carrying for a safety gate.
        """
        return ('A ground robot drives forward through an indoor environment, '
                'ego-view camera, steady motion.')

    def _wait(self, future, label: str) -> bool:
        deadline = time.monotonic() + self.action_timeout
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
            if time.monotonic() > deadline:
                self.get_logger().error(f'timed out waiting for {label}')
                return False
        return future.done()

    def _publish_event(self, event_type: str, message: str, data=None) -> None:
        payload = {'stamp': time.time(), 'type': event_type, 'message': message}
        if data:
            payload.update(data)
        self.event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _finish(self, response, steps, start, success, verdict, reason,
                confidence: float = 0.0):
        if verdict == 'unavailable':
            # Report why, then answer with the configured fail-closed verdict so
            # callers never read "unavailable" as "safe".
            steps.append(f'gate unavailable: {reason}')
            self._publish_event('unavailable', reason)
            verdict = self.unavailable_verdict
            self.get_logger().warn(f'world model gate unavailable: {reason}')
        response.success = success
        response.verdict = verdict
        response.confidence = confidence
        response.reason = reason
        response.steps = steps
        if not response.critic_json:
            response.critic_json = '{}'
        response.elapsed_time = (self.get_clock().now() - start).to_msg()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavflexWorldModel()
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
