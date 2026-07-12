#!/usr/bin/env python3
"""Navflex text instruction service.

This node intentionally stays small: it parses abstract text commands and
forwards them to the existing Navflex action surface:
  - ComputePathToPose + FollowPath for navigation to a pose
  - DummyBehavior(cmd_behavior) for relative motion commands
"""

import math
import json
import re
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, DummyBehavior, FollowPath
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterUninitializedException
from std_msgs.msg import String

from navflex_instruction_server.srv import ExecuteInstruction, GetCapabilities


@dataclass(frozen=True)
class ParsedInstruction:
    command_type: str
    normalized_command: str
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: float = 0.0
    behavior_command: str = ""


class InstructionParseError(ValueError):
    pass


class RealtimeSteps(list):
    def __init__(self, publish_fn):
        super().__init__()
        self._publish_fn = publish_fn

    def append(self, item):
        super().append(item)
        self._publish_fn('step', str(item), {'step': str(item)})


class NavflexInstructionServer(Node):
    def __init__(self) -> None:
        super().__init__('navflex_instruction_server')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('xy_goal_tolerance', 0.0)
        self.declare_parameter('yaw_goal_tolerance', 0.0)
        self.declare_parameter('behavior_id', 'cmd_behavior')
        self.declare_parameter('action_timeout', 60.0)
        self.declare_parameter('feedback_publish_period', 0.5)
        self.declare_parameter(
            'named_locations',
            Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Named locations formatted as name:x,y[,yaw_rad]'))

        self.global_frame = self.get_parameter('global_frame').value
        self.planner_id = self.get_parameter('planner_id').value
        self.controller_id = self.get_parameter('controller_id').value
        self.xy_goal_tolerance = float(
            self.get_parameter('xy_goal_tolerance').value)
        self.yaw_goal_tolerance = float(
            self.get_parameter('yaw_goal_tolerance').value)
        self.behavior_id = self.get_parameter('behavior_id').value
        self.action_timeout = float(self.get_parameter('action_timeout').value)
        self.feedback_publish_period = float(
            self.get_parameter('feedback_publish_period').value)
        self.named_locations = self._load_named_locations(
            self._get_string_array_parameter('named_locations'))

        self.callback_group = ReentrantCallbackGroup()

        self.planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=self.callback_group)
        self.follow_client = ActionClient(
            self, FollowPath, 'follow_path', callback_group=self.callback_group)
        self.behavior_client = ActionClient(
            self, DummyBehavior, 'behavior_action', callback_group=self.callback_group)
        self.event_pub = self.create_publisher(
            String, 'navflex_instruction/events', 10)

        self.execute_srv = self.create_service(
            ExecuteInstruction, 'navflex_instruction/execute', self.execute_instruction,
            callback_group=self.callback_group)
        self.capabilities_srv = self.create_service(
            GetCapabilities, 'navflex_instruction/capabilities', self.get_capabilities,
            callback_group=self.callback_group)
        self._execute_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._feedback_last_emit = {}

        self.get_logger().info(
            "Navflex instruction server ready. "
            f"planner_id='{self.planner_id}' controller_id='{self.controller_id}' "
            f"goal_tolerance=({self.xy_goal_tolerance:.3f}, {self.yaw_goal_tolerance:.3f}) "
            f"behavior_id='{self.behavior_id}' "
            f"feedback_publish_period={self.feedback_publish_period:.2f}s "
            f"named_locations={len(self.named_locations)}")

    def execute_instruction(self, request, response):
        start = self.get_clock().now()
        instruction = request.instruction.strip()
        steps = RealtimeSteps(self._publish_event)
        if not instruction:
            response.success = False
            response.command_type = 'invalid'
            response.normalized_command = ''
            response.message = 'instruction is empty'
            response.steps = ['received empty instruction', 'rejected before parsing']
            self._publish_event('failed', response.message, {
                'command_type': response.command_type,
                'steps': list(response.steps),
            })
            response.elapsed_time = (self.get_clock().now() - start).to_msg()
            return response

        if not self._execute_lock.acquire(blocking=False):
            response.success = False
            response.command_type = 'busy'
            response.normalized_command = ''
            response.message = 'another instruction is still running'
            response.steps = [
                f"received instruction: {instruction}",
                'rejected because another instruction is still running',
            ]
            self._publish_event('failed', response.message, {
                'command_type': response.command_type,
                'steps': list(response.steps),
            })
            response.elapsed_time = (self.get_clock().now() - start).to_msg()
            return response

        try:
            steps.append(f"received instruction: {instruction}")
            parsed = self._parse_instruction(instruction)
            response.command_type = parsed.command_type
            response.normalized_command = parsed.normalized_command
            steps.append(
                f"parsed command: type={parsed.command_type}, "
                f"normalized='{parsed.normalized_command}'")
            self.get_logger().info(
                f"Executing instruction: raw='{instruction}' type={parsed.command_type} "
                f"normalized='{parsed.normalized_command}'")

            if parsed.command_type == 'navigate':
                success, message = self._execute_navigate(parsed, steps)
            else:
                success, message = self._execute_behavior(parsed, steps)

            response.success = success
            response.message = message
            steps.append(('completed' if success else 'failed') + f": {message}")
            self._publish_event('completed' if success else 'failed', message, {
                'command_type': response.command_type,
                'normalized_command': response.normalized_command,
                'success': success,
            })
        except InstructionParseError as exc:
            response.success = False
            response.command_type = 'invalid'
            response.normalized_command = ''
            response.message = str(exc)
            steps.append(f'parse failed: {exc}')
            self._publish_event('failed', response.message, {
                'command_type': response.command_type,
                'normalized_command': response.normalized_command,
                'success': False,
            })
            self.get_logger().warn(
                f"Failed to parse instruction '{instruction}': {exc}")
        except Exception as exc:  # Keep service alive on unexpected action/client errors.
            response.success = False
            if not response.command_type:
                response.command_type = 'internal_error'
            response.message = f'internal error: {exc}'
            steps.append(f'internal error: {exc}')
            self._publish_event('failed', response.message, {
                'command_type': response.command_type,
                'normalized_command': response.normalized_command,
                'success': False,
            })
            self.get_logger().error(
                f"Instruction execution failed:\n{traceback.format_exc()}")
        finally:
            response.steps = steps
            response.elapsed_time = (self.get_clock().now() - start).to_msg()
            self._execute_lock.release()

        return response

    def get_capabilities(self, request, response):
        del request
        response.capabilities = [
            'navigate_to_pose: go/goto/navigate/去/到 <x> <y> [yaw]',
            'navigate_to_named_location: go/goto/navigate/去/到 <name>',
            'relative_linear_motion: forward/backward/前进/后退 <distance_m>',
            'relative_rotation: rotate/turn/spin/旋转/左转/右转 <angle_deg|angle_rad>',
            'wait: wait/等待 <duration_s>',
        ]
        response.examples = [
            'go to 1.0 2.0 90deg',
            '去 1.0 2.0',
            'goto kitchen',
            'forward 0.5m',
            'backward 0.3',
            'rotate 90deg',
            '原地旋转 -1.57rad',
            '左转 90 度',
            'wait 2.0',
        ]
        response.schema = (
            'ExecuteInstruction.srv: string instruction -> bool success, '
            'string command_type, string normalized_command, string message, '
            'string[] steps, '
            'builtin_interfaces/Duration elapsed_time. '
            'Navigation calls compute_path_to_pose then follow_path. '
            'Relative motion calls behavior_action with behavior_id parameter.'
        )
        return response

    def _execute_navigate(self, parsed: ParsedInstruction, steps) -> Tuple[bool, str]:
        if parsed.x is None or parsed.y is None:
            raise InstructionParseError('navigate instruction is missing x/y')

        steps.append(
            f"target pose: frame='{self.global_frame}', x={parsed.x:.3f}, "
            f"y={parsed.y:.3f}, yaw={parsed.yaw:.3f} rad")
        steps.append("checking action server: compute_path_to_pose")
        if not self._wait_for_action(self.planner_client, 'compute_path_to_pose'):
            steps.append('compute_path_to_pose action server is not available')
            return False, 'compute_path_to_pose action server is not available'
        steps.append("checking action server: follow_path")
        if not self._wait_for_action(self.follow_client, 'follow_path'):
            steps.append('follow_path action server is not available')
            return False, 'follow_path action server is not available'

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(parsed.x)
        goal_pose.pose.position.y = float(parsed.y)
        goal_pose.pose.orientation = self._yaw_to_quaternion(parsed.yaw)

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = goal_pose
        plan_goal.planner_id = self.planner_id
        plan_goal.use_start = False

        self.get_logger().info(
            f"Calling planner action: planner_id='{self.planner_id}' "
            f"goal=({parsed.x:.3f}, {parsed.y:.3f}, {parsed.yaw:.3f} rad)")
        steps.append(f"calling planner plugin: planner_id='{self.planner_id}'")
        plan_result = self._send_goal_and_wait(self.planner_client, plan_goal, 'compute_path_to_pose')
        if plan_result is None:
            steps.append(f"planner failed or timed out: planner_id='{self.planner_id}'")
            return False, 'compute_path_to_pose failed or timed out'
        if not self._action_result_ok(plan_result, 'compute_path_to_pose', success_upper_bound=10):
            message = self._result_message(plan_result, 'planner failed')
            steps.append(f"planner failed: planner_id='{self.planner_id}', {message}")
            return False, message

        path = plan_result.result.path
        if not path.poses:
            message = f'planner returned empty path: {getattr(plan_result.result, "message", "")}'
            steps.append(f"planner returned empty path: planner_id='{self.planner_id}'")
            return False, message
        steps.append(
            f"planner succeeded: planner_id='{self.planner_id}', poses={len(path.poses)}")

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = self.controller_id
        follow_goal.xy_goal_tolerance = self.xy_goal_tolerance
        follow_goal.yaw_goal_tolerance = self.yaw_goal_tolerance

        self.get_logger().info(
            f"Calling follow action: controller_id='{self.controller_id}' "
            f"goal_tolerance=({self.xy_goal_tolerance:.3f}, "
            f"{self.yaw_goal_tolerance:.3f}) poses={len(path.poses)}")
        steps.append(
            f"calling controller plugin: controller_id='{self.controller_id}', "
            f"goal_tolerance=({self.xy_goal_tolerance:.3f}, "
            f"{self.yaw_goal_tolerance:.3f}), poses={len(path.poses)}")
        follow_result = self._send_goal_and_wait(self.follow_client, follow_goal, 'follow_path')
        if follow_result is None:
            steps.append(
                f"follow_path failed or timed out: controller_id='{self.controller_id}'")
            return False, 'follow_path failed or timed out'
        if not self._action_result_ok(follow_result, 'follow_path', success_upper_bound=10):
            message = self._result_message(follow_result, 'follow_path failed')
            steps.append(
                f"controller failed: controller_id='{self.controller_id}', {message}")
            return False, message

        steps.append(
            f"controller succeeded: controller_id='{self.controller_id}', poses={len(path.poses)}")
        return True, (
            f"navigation completed: planner_id='{self.planner_id}', "
            f"controller_id='{self.controller_id}', poses={len(path.poses)}")

    def _execute_behavior(self, parsed: ParsedInstruction, steps) -> Tuple[bool, str]:
        if not parsed.behavior_command:
            raise InstructionParseError('behavior instruction is missing command')
        steps.append("checking action server: behavior_action")
        if not self._wait_for_action(self.behavior_client, 'behavior_action'):
            steps.append('behavior_action server is not available')
            return False, 'behavior_action server is not available'

        goal = DummyBehavior.Goal()
        goal.command = String(data=parsed.behavior_command)
        # navflex_nav extends this action server code to use goal.behavior
        # when available in the local nav2_msgs build. Set it defensively.
        if hasattr(goal, 'behavior'):
            goal.behavior = self.behavior_id

        self.get_logger().info(
            f"Calling behavior action: behavior_id='{self.behavior_id}' "
            f"command='{parsed.behavior_command}'")
        steps.append(
            f"calling behavior plugin: behavior_id='{self.behavior_id}', "
            f"command='{parsed.behavior_command}'")
        result = self._send_goal_and_wait(self.behavior_client, goal, 'behavior_action')
        if result is None:
            steps.append(
                f"behavior_action failed or timed out: behavior_id='{self.behavior_id}'")
            return False, 'behavior_action failed or timed out'
        if not self._action_result_ok(result, 'behavior_action', success_upper_bound=1):
            message = self._result_message(result, 'behavior failed')
            steps.append(
                f"behavior failed: behavior_id='{self.behavior_id}', {message}")
            return False, message
        message = getattr(result.result, 'message', '') or 'behavior completed'
        used_plugin = getattr(result.result, 'used_plugin', '') or self.behavior_id
        steps.append(
            f"behavior succeeded: behavior_id='{self.behavior_id}', "
            f"used_plugin='{used_plugin}'")
        if used_plugin and used_plugin not in message:
            message = f"{message} (used_plugin='{used_plugin}')"
        return True, message

    def _wait_for_action(self, client: ActionClient, name: str) -> bool:
        if client.server_is_ready():
            return True
        self.get_logger().info(f"Waiting for action server '{name}'...")
        return client.wait_for_server(timeout_sec=self.action_timeout)

    def _send_goal_and_wait(self, client: ActionClient, goal_msg, name: str):
        with self._feedback_lock:
            self._feedback_last_emit.pop(name, None)

        send_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self._on_action_feedback(name, msg.feedback))
        if not self._wait_for_future(send_future, f'{name} send_goal'):
            return None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Action goal rejected by '{name}'")
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, f'{name} result'):
            return None
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error(f"Action '{name}' returned no result")
            return None
        return wrapped

    def _on_action_feedback(self, action_name: str, feedback) -> None:
        now = time.monotonic()
        with self._feedback_lock:
            last_emit = self._feedback_last_emit.get(action_name, 0.0)
            if now - last_emit < self.feedback_publish_period:
                return
            self._feedback_last_emit[action_name] = now

        data = self._feedback_to_dict(feedback)
        self._publish_event('feedback', self._feedback_message(action_name, data), {
            'action': action_name,
            'feedback': data,
        })

    def _feedback_to_dict(self, feedback) -> Dict:
        data = {}
        for field in ['distance_to_goal', 'dist_to_goal', 'speed', 'angle_to_goal',
                      'outcome', 'message']:
            if hasattr(feedback, field):
                value = getattr(feedback, field)
                if isinstance(value, float):
                    value = round(value, 4)
                data[field] = value

        if hasattr(feedback, 'current_pose'):
            data['current_pose'] = self._pose_stamped_to_dict(feedback.current_pose)
        if hasattr(feedback, 'last_cmd_vel'):
            data['last_cmd_vel'] = {
                'linear_x': round(feedback.last_cmd_vel.twist.linear.x, 4),
                'angular_z': round(feedback.last_cmd_vel.twist.angular.z, 4),
            }
        return data

    def _feedback_message(self, action_name: str, data: Dict) -> str:
        parts = [f'{action_name} feedback']
        if 'distance_to_goal' in data:
            parts.append(f"distance_to_goal={data['distance_to_goal']:.3f}m")
        if 'dist_to_goal' in data:
            parts.append(f"dist_to_goal={data['dist_to_goal']:.3f}m")
        if 'speed' in data:
            parts.append(f"speed={data['speed']:.3f}m/s")
        if data.get('message'):
            parts.append(str(data['message']))
        return ', '.join(parts)

    def _pose_stamped_to_dict(self, pose_stamped) -> Dict:
        pose = pose_stamped.pose
        return {
            'frame_id': pose_stamped.header.frame_id,
            'x': round(pose.position.x, 4),
            'y': round(pose.position.y, 4),
            'z': round(pose.position.z, 4),
            'yaw': round(self._quaternion_to_yaw(pose.orientation), 4),
        }

    def _publish_event(self, event_type: str, message: str, data=None) -> None:
        payload = {
            'stamp': time.time(),
            'type': event_type,
            'message': message,
        }
        if data:
            payload.update(data)
        self.event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _action_result_ok(self, wrapped_result, name: str, success_upper_bound: int) -> bool:
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Action '{name}' finished with status={wrapped_result.status}")
            return False
        outcome = getattr(wrapped_result.result, 'outcome', 0)
        return outcome < success_upper_bound

    def _result_message(self, wrapped_result, default: str) -> str:
        outcome = getattr(wrapped_result.result, 'outcome', None)
        message = getattr(wrapped_result.result, 'message', '') or default
        if outcome is None:
            return message
        return f'{message} (outcome={outcome}, status={wrapped_result.status})'

    def _wait_for_future(self, future, label: str) -> bool:
        deadline = time.monotonic() + self.action_timeout
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.monotonic() > deadline:
                self.get_logger().error(f"Timed out waiting for {label}")
                return False
        if not future.done():
            return False
        return True

    def _parse_instruction(self, text: str) -> ParsedInstruction:
        cleaned = self._normalize_text(text)
        if not cleaned:
            raise InstructionParseError('instruction is empty after normalization')

        if self._looks_like_navigation(cleaned):
            return self._parse_navigation(cleaned)
        if self._looks_like_rotation(cleaned):
            return self._parse_rotation(cleaned)
        if self._looks_like_backward(cleaned):
            return self._parse_linear(cleaned, default_sign=-1.0)
        if self._looks_like_forward(cleaned):
            return self._parse_linear(cleaned, default_sign=1.0)
        if self._looks_like_wait(cleaned):
            return self._parse_wait(cleaned)

        raise InstructionParseError(
            "unknown instruction. Try 'go to 1.0 2.0', 'rotate 90deg', or 'forward 0.5m'")

    def _parse_navigation(self, text: str) -> ParsedInstruction:
        stripped = self._strip_navigation_prefix(text)
        if not stripped:
            raise InstructionParseError('navigate command needs a target')

        if stripped in self.named_locations:
            x, y, yaw = self.named_locations[stripped]
            return ParsedInstruction(
                command_type='navigate',
                normalized_command=f'navigate {stripped} ({x:.3f}, {y:.3f}, {yaw:.3f})',
                x=x, y=y, yaw=yaw)

        numbers = self._numbers_with_units(stripped)
        if len(numbers) < 2:
            raise InstructionParseError(
                f"unknown named location '{stripped}' and no x/y coordinates found")
        x = numbers[0][0]
        y = numbers[1][0]
        yaw = 0.0
        if len(numbers) >= 3:
            yaw = self._angle_to_rad(numbers[2][0], numbers[2][1])
        return ParsedInstruction(
            command_type='navigate',
            normalized_command=f'navigate {x:.3f} {y:.3f} {yaw:.3f}rad',
            x=x, y=y, yaw=yaw)

    def _parse_rotation(self, text: str) -> ParsedInstruction:
        numbers = self._numbers_with_units(text)
        if not numbers:
            raise InstructionParseError('rotate command needs an angle')
        value, unit = numbers[0]
        if self._contains_any(text, ['right', 'clockwise', '右转', '顺时针']) and value > 0.0:
            value = -value
        angle_rad = self._angle_to_rad(value, unit)
        return ParsedInstruction(
            command_type='rotate',
            normalized_command=f'rotate {angle_rad:.6f}',
            behavior_command=f'rotate {angle_rad:.6f}')

    def _parse_linear(self, text: str, default_sign: float) -> ParsedInstruction:
        numbers = self._numbers_with_units(text)
        if not numbers:
            raise InstructionParseError('linear command needs a distance')
        distance = numbers[0][0]
        if distance >= 0.0:
            distance *= default_sign
        return ParsedInstruction(
            command_type='linear',
            normalized_command=f'linear {distance:.6f}',
            behavior_command=f'linear {distance:.6f}')

    def _parse_wait(self, text: str) -> ParsedInstruction:
        numbers = self._numbers_with_units(text)
        if not numbers:
            raise InstructionParseError('wait command needs a duration')
        seconds = numbers[0][0]
        if seconds <= 0.0:
            raise InstructionParseError('wait duration must be > 0')
        return ParsedInstruction(
            command_type='wait',
            normalized_command=f'wait {seconds:.6f}',
            behavior_command=f'wait {seconds:.6f}')

    def _looks_like_navigation(self, text: str) -> bool:
        return self._strip_navigation_prefix(text) != text

    def _looks_like_rotation(self, text: str) -> bool:
        return self._contains_any(text, ['rotate', 'turn', 'spin', '旋转', '原地旋转', '左转', '右转'])

    def _looks_like_forward(self, text: str) -> bool:
        return self._contains_any(text, ['forward', 'ahead', '前进', '向前', '往前'])

    def _looks_like_backward(self, text: str) -> bool:
        return self._contains_any(text, ['backward', 'back', 'reverse', '后退', '向后', '往后'])

    def _looks_like_wait(self, text: str) -> bool:
        return self._contains_any(text, ['wait', 'sleep', '等待', '停留'])

    def _contains_any(self, text: str, needles) -> bool:
        return any(needle in text for needle in needles)

    def _numbers_with_units(self, text: str):
        pattern = re.compile(
            r'([-+]?\d+(?:\.\d+)?)\s*(degrees?|degree|deg|度|radians?|radian|rad|米|m|秒|s)?')
        return [(float(match.group(1)), match.group(2) or '') for match in pattern.finditer(text)]

    def _angle_to_rad(self, value: float, unit: str) -> float:
        if unit in ['degree', 'degrees', 'deg', '度']:
            return math.radians(value)
        if unit in ['radian', 'radians', 'rad']:
            return value
        # Human text commonly omits units for heading commands; degrees are the
        # friendlier default unless the value already looks like radians.
        if abs(value) > (2.0 * math.pi):
            return math.radians(value)
        return value

    def _normalize_text(self, text: str) -> str:
        text = text.strip().lower()
        replacements = {
            '，': ' ', ',': ' ', '。': ' ', ';': ' ', '；': ' ',
            '（': ' ', '）': ' ', '(': ' ', ')': ' ', ':': ' ',
        }
        for old, new in replacements.items():
            text = text.replace(old, new)
        return re.sub(r'\s+', ' ', text).strip()

    def _strip_navigation_prefix(self, text: str) -> str:
        english_prefixes = ['navigate to', 'move to', 'go to', 'goto', 'nav to']
        for prefix in english_prefixes:
            if text == prefix:
                return ''
            if text.startswith(prefix + ' '):
                return text[len(prefix):].strip()

        for prefix in ['去到', '去', '到']:
            if text.startswith(prefix):
                return text[len(prefix):].strip()
        return text

    def _load_named_locations(self, values) -> Dict[str, Tuple[float, float, float]]:
        locations: Dict[str, Tuple[float, float, float]] = {}
        if not values:
            return locations
        for item in values:
            parts = str(item).split(':')
            if len(parts) != 2:
                self.get_logger().warn(
                    f"Ignoring named location '{item}'; expected name:x,y[,yaw]")
                continue
            name = parts[0].strip().lower()
            nums = [p.strip() for p in parts[1].split(',')]
            if len(nums) not in [2, 3]:
                self.get_logger().warn(
                    f"Ignoring named location '{item}'; expected two or three numbers")
                continue
            try:
                x = float(nums[0])
                y = float(nums[1])
                yaw = float(nums[2]) if len(nums) == 3 else 0.0
            except ValueError:
                self.get_logger().warn(
                    f"Ignoring named location '{item}'; failed to parse numbers")
                continue
            locations[name] = (x, y, yaw)
        return locations

    def _get_string_array_parameter(self, name: str):
        try:
            return self.get_parameter(name).value or []
        except ParameterUninitializedException:
            return []

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavflexInstructionServer()
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
