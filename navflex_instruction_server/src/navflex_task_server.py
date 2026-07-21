#!/usr/bin/env python3
"""High-level Navflex task server.

This layer keeps LLM/VLN output away from direct robot control. It accepts a
small task schema, grounds semantic targets through navflex_semantic_map, and
only then calls the existing text instruction executor.

When the world-model gate is enabled, a grounded navigation goal also gets
imagined before it is driven: navflex_world_model plans the path, rolls a video
world model forward along it, and returns a verdict that feeds the confirmation
gate below.
"""

import json
import math
import re
import time
import traceback
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from navflex_instruction_server.srv import (
    ExecuteInstruction,
    ExecuteTask,
    QuerySemanticTarget,
)

try:
    from navflex_world_model.srv import EvaluatePlan
except ImportError:  # navflex_world_model is an optional overlay
    EvaluatePlan = None


class TaskError(ValueError):
    pass


class NavflexTaskServer(Node):
    def __init__(self) -> None:
        super().__init__('navflex_task_server')
        self.declare_parameter('instruction_service', 'navflex_instruction/execute')
        self.declare_parameter('semantic_query_service', 'navflex_semantic_map/query_target')
        self.declare_parameter('use_semantic_map', True)
        self.declare_parameter('default_execute', False)
        self.declare_parameter('high_risk_constraints', ['enter_restricted_zone', 'ignore_obstacles'])
        self.declare_parameter('action_timeout', 30.0)
        self.declare_parameter('world_model_enabled', False)
        self.declare_parameter('world_model_service', 'navflex_world_model/evaluate')
        # Imagining a rollout is a video-diffusion pass: minutes on one GPU.
        self.declare_parameter('world_model_timeout', 900.0)
        self.declare_parameter('world_model_frame_num', 0)

        self.instruction_service = self.get_parameter('instruction_service').value
        self.semantic_query_service = self.get_parameter('semantic_query_service').value
        self.use_semantic_map = self._as_bool(
            self.get_parameter('use_semantic_map').value)
        self.default_execute = self._as_bool(
            self.get_parameter('default_execute').value)
        self.high_risk_constraints = set(self.get_parameter('high_risk_constraints').value or [])
        self.action_timeout = float(self.get_parameter('action_timeout').value)
        self.world_model_enabled = self._as_bool(
            self.get_parameter('world_model_enabled').value)
        self.world_model_service = self.get_parameter('world_model_service').value
        self.world_model_timeout = float(self.get_parameter('world_model_timeout').value)
        self.world_model_frame_num = int(self.get_parameter('world_model_frame_num').value)

        self.callback_group = ReentrantCallbackGroup()
        self.execute_client = self.create_client(
            ExecuteInstruction,
            self.instruction_service,
            callback_group=self.callback_group)
        self.semantic_client = self.create_client(
            QuerySemanticTarget,
            self.semantic_query_service,
            callback_group=self.callback_group)

        self.world_model_client = None
        if self.world_model_enabled:
            if EvaluatePlan is None:
                self.get_logger().error(
                    'world_model_enabled but navflex_world_model is not on the '
                    'ament prefix path; the gate will be skipped')
                self.world_model_enabled = False
            else:
                self.world_model_client = self.create_client(
                    EvaluatePlan,
                    self.world_model_service,
                    callback_group=self.callback_group)

        self.create_service(
            ExecuteTask,
            'navflex_task/execute',
            self.execute_task,
            callback_group=self.callback_group)
        self.get_logger().info(
            f"Task server ready: instruction_service='{self.instruction_service}', "
            f"semantic_query_service='{self.semantic_query_service}', "
            f"world_model={'on -> ' + self.world_model_service if self.world_model_enabled else 'off'}, "
            f"use_semantic_map={self.use_semantic_map}")

    def execute_task(self, request, response):
        start = self.get_clock().now()
        steps: List[str] = []
        try:
            task = self._load_task(request.instruction, request.task_json)
            steps.append(f"task parsed: action={task.get('action', '')}")
            task.setdefault('source_instruction', request.instruction)
            task.setdefault('constraints', [])
            task.setdefault('confirmation_required', False)

            command = self._ground_task(task, steps)
            task['normalized_command'] = command
            response.task_json = json.dumps(task, ensure_ascii=False, sort_keys=True)
            response.command_type = task.get('action', '')
            response.normalized_command = command

            requires_confirmation = bool(task.get('confirmation_required')) or request.require_confirmation
            risky = [c for c in task.get('constraints', []) if c in self.high_risk_constraints]
            if risky:
                requires_confirmation = True
                steps.append(f"confirmation required for high risk constraints: {','.join(risky)}")
            response.requires_confirmation = requires_confirmation

            should_execute = bool(request.execute or self.default_execute)
            if request.dry_run:
                should_execute = False
                steps.append('dry_run requested: execution skipped')
            if requires_confirmation and not request.execute:
                should_execute = False
                steps.append('confirmation required: execution skipped until execute=true')

            if not should_execute:
                response.success = True
                response.accepted = False
                # The gate only runs on plans that are about to be driven, so
                # say so rather than leaving the verdict field ambiguously empty.
                response.world_model_verdict = 'not_evaluated'
                response.message = 'task parsed and grounded; execution skipped'
                return self._finish(response, steps, start)

            # Everything above decided the plan is worth running. Imagine it
            # before driving it: the gate can still veto.
            verdict = self._consult_world_model(task, request, response, steps)
            if verdict == 'reject':
                response.success = False
                response.accepted = False
                response.requires_confirmation = False
                response.message = f'world model rejected the plan: {response.world_model_reason}'
                return self._finish(response, steps, start)
            if verdict == 'needs_confirmation':
                response.success = True
                response.accepted = False
                response.requires_confirmation = True
                response.message = (
                    'world model requires confirmation: '
                    f'{response.world_model_reason}. Re-send with '
                    'skip_world_model=true to execute anyway.')
                return self._finish(response, steps, start)

            steps.append(f"calling instruction executor: {command}")
            result = self._call_execute_instruction(command)
            if result is None:
                response.success = False
                response.accepted = False
                response.message = 'instruction executor unavailable or timed out'
                steps.append(response.message)
                return self._finish(response, steps, start)

            response.success = bool(result.success)
            response.accepted = True
            response.message = result.message
            steps.extend([f"executor: {step}" for step in result.steps])
            return self._finish(response, steps, start)
        except TaskError as exc:
            response.success = False
            response.accepted = False
            response.message = str(exc)
            steps.append(f'task rejected: {exc}')
            return self._finish(response, steps, start)
        except Exception as exc:
            response.success = False
            response.accepted = False
            response.message = f'internal error: {exc}'
            steps.append(response.message)
            self.get_logger().error(
                f"Task execution failed:\n{traceback.format_exc()}")
            return self._finish(response, steps, start)

    def _load_task(self, instruction: str, task_json: str) -> Dict:
        if task_json.strip():
            try:
                task = json.loads(task_json)
            except json.JSONDecodeError as exc:
                raise TaskError(f'invalid task_json: {exc}') from exc
            if not isinstance(task, dict):
                raise TaskError('task_json must be a JSON object')
            return task
        return self._parse_instruction(instruction)

    def _parse_instruction(self, instruction: str) -> Dict:
        text = self._normalize(instruction)
        if not text:
            raise TaskError('instruction or task_json is required')
        if self._looks_like_direct_motion(text):
            return {'action': 'direct_instruction', 'instruction': instruction}
        if self._contains_any(text, ['禁区', 'restricted', 'no go', 'no-go']):
            return {
                'action': 'semantic_navigate',
                'target': self._strip_nav_words(text),
                'constraints': ['enter_restricted_zone'],
                'confirmation_required': True,
            }
        return {
            'action': 'semantic_navigate',
            'target': self._strip_nav_words(text),
            'constraints': [],
        }

    def _ground_task(self, task: Dict, steps: List[str]) -> str:
        action = str(task.get('action', '')).strip().lower()
        if action in ['direct_instruction', 'instruction']:
            command = str(task.get('instruction') or task.get('source_instruction') or '').strip()
            if not command:
                raise TaskError('direct_instruction task needs instruction')
            return command
        if action in ['navigate', 'navigate_to_pose']:
            x = task.get('x')
            y = task.get('y')
            yaw = float(task.get('yaw', 0.0))
            if x is None or y is None:
                pose = task.get('pose') or {}
                x = pose.get('x')
                y = pose.get('y')
                yaw = float(pose.get('yaw', yaw))
            if x is None or y is None:
                raise TaskError('navigate task needs x/y or pose.x/pose.y')
            return f"go to {float(x):.3f} {float(y):.3f} {yaw:.6f}rad"
        if action in ['semantic_navigate', 'navigate_to_object_or_region', 'navigate_to_landmark']:
            target = str(task.get('target') or task.get('target_name') or '').strip()
            target_type = str(task.get('target_type') or '').strip()
            pose = self._pose_from_task(task)
            if pose is not None:
                task['grounded_target'] = {
                    'name': target or 'observed_target',
                    'target_type': target_type,
                    'x': pose['x'],
                    'y': pose['y'],
                    'yaw': pose['yaw'],
                    'source': pose['source'],
                }
                steps.append(
                    f"semantic target grounded from {pose['source']}: "
                    f"{task['grounded_target']['name']}")
                return f"go to {pose['x']:.3f} {pose['y']:.3f} {pose['yaw']:.6f}rad"
            if not target:
                raise TaskError('semantic navigation task needs target or target_pose')
            if not self.use_semantic_map:
                raise TaskError(
                    'semantic map disabled: semantic navigation needs target_pose, pose, nav_goal, or x/y/yaw')
            semantic = self._query_semantic_target(target, target_type)
            if semantic is None:
                raise TaskError(f"semantic target not found: {target}")
            task['grounded_target'] = {
                'name': semantic.target_name,
                'target_type': semantic.target_type,
                'x': semantic.pose.pose.position.x,
                'y': semantic.pose.pose.position.y,
                'yaw': self._quaternion_to_yaw(semantic.pose.pose.orientation),
            }
            steps.append(
                f"semantic target grounded: {semantic.target_name} ({semantic.target_type})")
            pose = task['grounded_target']
            return f"go to {pose['x']:.3f} {pose['y']:.3f} {pose['yaw']:.6f}rad"
        if action in ['wait', 'linear', 'rotate']:
            value = task.get('value')
            if value is None:
                raise TaskError(f'{action} task needs value')
            return f"{action} {float(value):.6f}"
        raise TaskError(f"unsupported task action '{action}'")

    def _consult_world_model(self, task: Dict, request, response, steps: List[str]) -> str:
        """Imagine the grounded plan and return the gate's verdict.

        Returns 'approve' whenever execution should proceed, which includes the
        cases where the gate does not apply -- a rotate/wait task has no path to
        imagine, and the gate is off by default.
        """
        response.world_model_verdict = 'disabled'
        response.world_model_reason = ''
        response.world_model_rollout_uri = ''

        if not self.world_model_enabled:
            return 'approve'
        if request.skip_world_model:
            response.world_model_verdict = 'skipped'
            response.world_model_reason = 'caller set skip_world_model'
            steps.append('world model gate skipped by caller')
            return 'approve'

        goal = self._task_goal_pose(task)
        if goal is None:
            response.world_model_verdict = 'skipped'
            response.world_model_reason = (
                f"action '{task.get('action', '')}' has no navigation goal to imagine")
            steps.append(f'world model gate skipped: {response.world_model_reason}')
            return 'approve'

        if not self.world_model_client.wait_for_service(timeout_sec=self.action_timeout):
            response.world_model_verdict = 'needs_confirmation'
            response.world_model_reason = (
                f'world model service unavailable: {self.world_model_service}')
            steps.append(response.world_model_reason)
            # Fail closed: an absent gate is not an approving gate.
            return 'needs_confirmation'

        req = EvaluatePlan.Request()
        req.goal = goal
        req.instruction = task.get('source_instruction', '') or request.instruction
        req.task_json = json.dumps(task, ensure_ascii=False, sort_keys=True)
        req.dry_run = False
        req.frame_num = self.world_model_frame_num

        steps.append('world model: imagining the plan before executing it')
        future = self.world_model_client.call_async(req)
        if not self._wait_for_future(future, 'world model evaluate',
                                     timeout=self.world_model_timeout):
            response.world_model_verdict = 'needs_confirmation'
            response.world_model_reason = 'world model evaluation timed out'
            steps.append(response.world_model_reason)
            return 'needs_confirmation'

        result = future.result()
        if result is None:
            response.world_model_verdict = 'needs_confirmation'
            response.world_model_reason = 'world model returned no result'
            steps.append(response.world_model_reason)
            return 'needs_confirmation'

        response.world_model_verdict = result.verdict
        response.world_model_reason = result.reason
        response.world_model_rollout_uri = result.rollout_uri
        steps.extend([f'world model: {step}' for step in result.steps])
        steps.append(
            f'world model verdict: {result.verdict} '
            f'(confidence {result.confidence:.2f})')
        return result.verdict if result.verdict in (
            'approve', 'reject', 'needs_confirmation') else 'needs_confirmation'

    def _task_goal_pose(self, task: Dict) -> Optional[PoseStamped]:
        """The pose the robot would drive to, or None if the task is not a drive."""
        grounded = task.get('grounded_target')
        if isinstance(grounded, dict) and grounded.get('x') is not None:
            x, y, yaw = grounded['x'], grounded['y'], grounded.get('yaw', 0.0)
        elif task.get('action') in ('navigate', 'navigate_to_pose'):
            pose = task.get('pose') or {}
            x = task.get('x', pose.get('x'))
            y = task.get('y', pose.get('y'))
            yaw = task.get('yaw', pose.get('yaw', 0.0))
            if x is None or y is None:
                return None
        else:
            return None

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(float(yaw) * 0.5)
        goal.pose.orientation.w = math.cos(float(yaw) * 0.5)
        return goal

    def _query_semantic_target(self, target: str, target_type: str):
        if not self.semantic_client.wait_for_service(timeout_sec=self.action_timeout):
            raise TaskError(f"semantic query service unavailable: {self.semantic_query_service}")
        req = QuerySemanticTarget.Request()
        req.query = target
        req.target_type = target_type
        future = self.semantic_client.call_async(req)
        if not self._wait_for_future(future, 'semantic target query'):
            return None
        result = future.result()
        if result is None or not result.success:
            return None
        return result

    def _pose_from_task(self, task: Dict) -> Optional[Dict]:
        for key in ['target_pose', 'goal_pose', 'nav_goal', 'pose']:
            parsed = self._parse_pose_dict(task.get(key), key)
            if parsed is not None:
                return parsed
        return self._parse_pose_dict(task, 'x/y/yaw')

    def _parse_pose_dict(self, value, source: str) -> Optional[Dict]:
        if not isinstance(value, dict):
            return None
        x = value.get('x')
        y = value.get('y')
        yaw = value.get('yaw', value.get('theta', 0.0))
        if x is None or y is None:
            position = value.get('position')
            if isinstance(position, dict):
                x = position.get('x')
                y = position.get('y')
            pose = value.get('pose')
            if isinstance(pose, dict):
                nested = self._parse_pose_dict(pose, source)
                if nested is not None:
                    nested['source'] = source
                    return nested
        if x is None or y is None:
            return None
        return {
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'source': source,
        }

    def _call_execute_instruction(self, instruction: str):
        if not self.execute_client.wait_for_service(timeout_sec=self.action_timeout):
            return None
        req = ExecuteInstruction.Request()
        req.instruction = instruction
        future = self.execute_client.call_async(req)
        if not self._wait_for_future(future, 'instruction execute'):
            return None
        return future.result()

    def _wait_for_future(self, future, label: str, timeout: Optional[float] = None) -> bool:
        deadline = time.monotonic() + (
            self.action_timeout if timeout is None else timeout)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.monotonic() > deadline:
                self.get_logger().error(f'Timed out waiting for {label}')
                return False
        return future.done()

    def _finish(self, response, steps: List[str], start):
        response.steps = steps
        response.elapsed_time = (self.get_clock().now() - start).to_msg()
        if not response.task_json:
            response.task_json = '{}'
        return response

    def _looks_like_direct_motion(self, text: str) -> bool:
        return self._contains_any(text, [
            'go to', 'goto', '去 ', '去到', '到 ', 'forward', 'backward',
            'rotate', 'turn', 'wait', '前进', '后退', '左转', '右转', '等待']) and bool(re.search(r'[-+]?\d', text))

    def _strip_nav_words(self, text: str) -> str:
        cleaned = text
        for token in ['帮我', '请', 'navigate to', 'go to', 'goto', 'move to', '去到', '去', '到', '附近', '旁边']:
            cleaned = cleaned.replace(token, ' ')
        return re.sub(r'\s+', ' ', cleaned).strip()

    def _contains_any(self, text: str, needles) -> bool:
        return any(needle in text for needle in needles)

    def _normalize(self, text: str) -> str:
        text = str(text or '').strip().lower()
        for old in ['，', ',', '。', ';', '；', '（', '）', '(', ')', ':']:
            text = text.replace(old, ' ')
        return re.sub(r'\s+', ' ', text).strip()

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ['1', 'true', 'yes', 'on']
        return bool(value)

    def _quaternion_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavflexTaskServer()
    executor = MultiThreadedExecutor(num_threads=2)
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
