#!/usr/bin/env python3
"""VLN bridge for Navflex.

The bridge accepts raw model output or deterministic inputs and converts them
into the constrained Navflex task schema before calling navflex_task/execute.
"""

import json
import re
import time
import traceback
from typing import Dict, List

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from navflex_instruction_server.srv import ExecuteTask, InterpretVln


class VlnBridgeError(ValueError):
    pass


class NavflexVlnBridge(Node):
    def __init__(self) -> None:
        super().__init__('navflex_vln_bridge')
        self.declare_parameter('task_service', 'navflex_task/execute')
        self.declare_parameter('action_timeout', 30.0)
        self.declare_parameter('allowed_actions', [
            'direct_instruction',
            'navigate',
            'semantic_navigate',
            'navigate_to_object_or_region',
            'navigate_to_landmark',
            'wait',
            'linear',
            'rotate',
        ])
        self.task_service = self.get_parameter('task_service').value
        self.action_timeout = float(self.get_parameter('action_timeout').value)
        self.allowed_actions = set(self.get_parameter('allowed_actions').value or [])
        self.callback_group = ReentrantCallbackGroup()
        self.task_client = self.create_client(
            ExecuteTask,
            self.task_service,
            callback_group=self.callback_group)
        self.create_service(
            InterpretVln,
            'navflex_vln/interpret',
            self.interpret,
            callback_group=self.callback_group)
        self.get_logger().info(f"VLN bridge ready: task_service='{self.task_service}'")

    def interpret(self, request, response):
        start = self.get_clock().now()
        steps: List[str] = []
        try:
            action = self._build_action(request, steps)
            response.action_json = json.dumps(action, ensure_ascii=False, sort_keys=True)
            steps.append(f"action normalized: {action.get('action', '')}")

            task_result = self._call_task(action, request.instruction, request.execute, request.dry_run)
            if task_result is None:
                response.success = False
                response.execution_success = False
                response.message = 'task service unavailable or timed out'
                steps.append(response.message)
                return self._finish(response, steps, start)
            response.success = True
            response.execution_success = bool(task_result.success)
            response.message = task_result.message
            steps.extend([f"task: {step}" for step in task_result.steps])
            return self._finish(response, steps, start)
        except VlnBridgeError as exc:
            response.success = False
            response.execution_success = False
            response.message = str(exc)
            steps.append(f'vln bridge rejected input: {exc}')
            return self._finish(response, steps, start)
        except Exception as exc:
            response.success = False
            response.execution_success = False
            response.message = f'internal error: {exc}'
            steps.append(response.message)
            self.get_logger().error(
                f"VLN interpretation failed:\n{traceback.format_exc()}")
            return self._finish(response, steps, start)

    def _build_action(self, request, steps: List[str]) -> Dict:
        if request.model_output_json.strip():
            model_output = self._parse_json(request.model_output_json, 'model_output_json')
            steps.append('using model_output_json')
            action = self._extract_action(model_output)
        else:
            steps.append('using deterministic fallback parser')
            action = self._fallback_action(request.instruction, request.perception_json, request.semantic_context_json)

        action = self._normalize_action(action)
        self._enrich_action_from_perception(action, request.perception_json, steps)
        self._validate_action(action)
        return action

    def _extract_action(self, model_output: Dict) -> Dict:
        if 'action' in model_output:
            return dict(model_output)
        if 'task' in model_output and isinstance(model_output['task'], dict):
            return dict(model_output['task'])
        if 'tool_call' in model_output and isinstance(model_output['tool_call'], dict):
            args = model_output['tool_call'].get('arguments', {})
            if isinstance(args, dict):
                return dict(args)
        raise VlnBridgeError('model output does not contain action/task/tool_call.arguments')

    def _fallback_action(self, instruction: str, perception_json: str, semantic_context_json: str) -> Dict:
        text = self._normalize_text(instruction)
        perception = self._parse_json(perception_json, 'perception_json') if perception_json.strip() else {}
        semantic_context = self._parse_json(semantic_context_json, 'semantic_context_json') if semantic_context_json.strip() else {}

        visible_targets = self._visible_targets(perception)
        for target in visible_targets:
            name = self._normalize_text(target.get('name') or target.get('label') or '')
            if name and (name in text or text in name):
                action = {
                    'action': 'semantic_navigate',
                    'target': name,
                    'target_type': target.get('type') or target.get('label') or '',
                    'evidence': {'source': 'perception', 'target': target},
                }
                pose = self._pose_from_target(target)
                if pose is not None:
                    action['target_pose'] = pose
                return action

        for landmark in semantic_context.get('landmarks', []):
            if isinstance(landmark, str):
                try:
                    landmark = json.loads(landmark)
                except json.JSONDecodeError:
                    continue
            name = self._normalize_text(landmark.get('name', ''))
            aliases = [self._normalize_text(a) for a in landmark.get('aliases', [])]
            if name and (name in text or text in name or any(alias and alias in text for alias in aliases)):
                return {
                    'action': 'semantic_navigate',
                    'target': name,
                    'target_type': landmark.get('target_type', ''),
                    'evidence': {'source': 'semantic_context'},
                }

        return {'action': 'semantic_navigate', 'target': self._strip_nav_words(text)}

    def _normalize_action(self, action: Dict) -> Dict:
        normalized = dict(action)
        normalized['action'] = self._normalize_text(normalized.get('action', ''))
        if normalized['action'] in ['navigate_to_object', 'navigate_to_region']:
            normalized['action'] = 'navigate_to_object_or_region'
        if normalized['action'] == 'goto':
            normalized['action'] = 'semantic_navigate'
        constraints = normalized.get('constraints', [])
        if isinstance(constraints, str):
            constraints = [constraints]
        normalized['constraints'] = [self._normalize_text(c) for c in constraints if self._normalize_text(c)]
        if 'target_name' in normalized and 'target' not in normalized:
            normalized['target'] = normalized['target_name']
        return normalized

    def _validate_action(self, action: Dict) -> None:
        name = action.get('action', '')
        if name not in self.allowed_actions:
            raise VlnBridgeError(f"action '{name}' is not allowed")
        if name in ['semantic_navigate', 'navigate_to_object_or_region', 'navigate_to_landmark']:
            if not str(action.get('target', '')).strip() and not self._action_has_pose(action):
                raise VlnBridgeError('semantic navigation action needs target or target_pose')
        if name in ['navigate']:
            if not self._action_has_pose(action):
                raise VlnBridgeError('navigate action needs x/y or pose')

    def _enrich_action_from_perception(self, action: Dict, perception_json: str, steps: List[str]) -> None:
        if self._action_has_pose(action) or not perception_json.strip():
            return
        if action.get('action') not in ['semantic_navigate', 'navigate_to_object_or_region', 'navigate_to_landmark']:
            return
        perception = self._parse_json(perception_json, 'perception_json')
        target_text = self._normalize_text(action.get('target', ''))
        if not target_text:
            return
        for target in self._visible_targets(perception):
            names = [
                self._normalize_text(target.get('name', '')),
                self._normalize_text(target.get('label', '')),
                self._normalize_text(target.get('type', '')),
            ]
            aliases = target.get('aliases', [])
            if isinstance(aliases, list):
                names.extend(self._normalize_text(alias) for alias in aliases)
            if not any(name and (name in target_text or target_text in name) for name in names):
                continue
            pose = self._pose_from_target(target)
            if pose is None:
                continue
            action['target_pose'] = pose
            action.setdefault('target_type', target.get('type') or target.get('label') or '')
            action['evidence'] = {'source': 'perception', 'target': target}
            steps.append(f"target pose attached from perception: {action.get('target', '')}")
            return

    def _action_has_pose(self, action: Dict) -> bool:
        if self._pose_from_target(action) is not None:
            return True
        return any(isinstance(action.get(key), dict) and self._parse_pose(action[key]) is not None
                   for key in ['target_pose', 'goal_pose', 'nav_goal', 'pose'])

    def _pose_from_target(self, target: Dict):
        for key in ['target_pose', 'goal_pose', 'nav_goal', 'pose', 'map_pose']:
            pose = self._parse_pose(target.get(key))
            if pose is not None:
                return pose
        return self._parse_pose(target)

    def _parse_pose(self, value):
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
                return self._parse_pose(pose)
        if x is None or y is None:
            return None
        return {'x': float(x), 'y': float(y), 'yaw': float(yaw)}

    def _call_task(self, action: Dict, instruction: str, execute: bool, dry_run: bool):
        if not self.task_client.wait_for_service(timeout_sec=self.action_timeout):
            return None
        req = ExecuteTask.Request()
        req.instruction = instruction
        req.task_json = json.dumps(action, ensure_ascii=False)
        req.execute = execute
        req.dry_run = dry_run
        req.require_confirmation = bool(action.get('confirmation_required', False))
        future = self.task_client.call_async(req)
        if not self._wait_for_future(future, 'task execute'):
            return None
        return future.result()

    def _parse_json(self, text: str, label: str) -> Dict:
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise VlnBridgeError(f'invalid {label}: {exc}') from exc
        if not isinstance(data, dict):
            raise VlnBridgeError(f'{label} must be a JSON object')
        return data

    def _visible_targets(self, perception: Dict) -> List[Dict]:
        objects = perception.get('objects', [])
        if isinstance(objects, list):
            return [obj for obj in objects if isinstance(obj, dict)]
        return []

    def _wait_for_future(self, future, label: str) -> bool:
        deadline = time.monotonic() + self.action_timeout
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.monotonic() > deadline:
                self.get_logger().error(f'Timed out waiting for {label}')
                return False
        return future.done()

    def _finish(self, response, steps: List[str], start):
        response.steps = steps
        response.elapsed_time = (self.get_clock().now() - start).to_msg()
        if not response.action_json:
            response.action_json = '{}'
        return response

    def _strip_nav_words(self, text: str) -> str:
        cleaned = text
        for token in ['帮我', '请', 'navigate to', 'go to', 'goto', 'move to', '去到', '去', '到', '附近', '旁边']:
            cleaned = cleaned.replace(token, ' ')
        return re.sub(r'\s+', ' ', cleaned).strip()

    def _normalize_text(self, text) -> str:
        text = str(text or '').strip().lower()
        for old in ['，', ',', '。', ';', '；', '（', '）', '(', ')', ':']:
            text = text.replace(old, ' ')
        return re.sub(r'\s+', ' ', text).strip()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavflexVlnBridge()
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
