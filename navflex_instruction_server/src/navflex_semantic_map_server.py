#!/usr/bin/env python3
"""Semantic landmark registry for Navflex task/VLN layers."""

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterUninitializedException

from navflex_instruction_server.srv import (
    ListSemanticLandmarks,
    QuerySemanticTarget,
    UpdateLandmark,
)


@dataclass
class Landmark:
    name: str
    target_type: str
    pose: PoseStamped
    aliases: List[str] = field(default_factory=list)

    def matches(self, query: str, target_type: str) -> bool:
        if target_type and self.target_type != target_type:
            return False
        if not query:
            return True
        terms = [self.name, self.target_type] + self.aliases
        return any(query in term or term in query for term in terms)

    def to_json(self) -> str:
        pose = self.pose.pose
        return json.dumps({
            'name': self.name,
            'target_type': self.target_type,
            'aliases': self.aliases,
            'frame_id': self.pose.header.frame_id,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': quaternion_to_yaw(pose.orientation),
        }, ensure_ascii=False)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavflexSemanticMapServer(Node):
    def __init__(self) -> None:
        super().__init__('navflex_semantic_map_server')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter(
            'landmarks',
            Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Landmarks: name|type|x|y|yaw|alias1,alias2'))
        self.global_frame = self.get_parameter('global_frame').value
        self.landmarks: Dict[str, Landmark] = {}
        self._load_landmarks(self._get_string_array_parameter('landmarks'))

        self.create_service(
            QuerySemanticTarget,
            'navflex_semantic_map/query_target',
            self.query_target)
        self.create_service(
            UpdateLandmark,
            'navflex_semantic_map/update_landmark',
            self.update_landmark)
        self.create_service(
            ListSemanticLandmarks,
            'navflex_semantic_map/list_landmarks',
            self.list_landmarks)
        self.get_logger().info(
            f"Semantic map ready: frame='{self.global_frame}', landmarks={len(self.landmarks)}")

    def query_target(self, request, response):
        query = self._normalize(request.query)
        target_type = self._normalize(request.target_type)
        candidates = [lm for lm in self.landmarks.values() if lm.matches(query, target_type)]
        if not candidates:
            response.success = False
            response.message = f"no semantic target matched query='{request.query}' type='{request.target_type}'"
            response.candidates = [lm.to_json() for lm in self.landmarks.values()]
            return response

        best = self._rank(candidates, request.near_x, request.near_y, request.use_near)[0]
        response.success = True
        response.target_name = best.name
        response.target_type = best.target_type
        response.pose = best.pose
        response.message = f"matched semantic target '{best.name}' ({best.target_type})"
        response.candidates = [lm.to_json() for lm in candidates]
        response.score = self._score(best, request.near_x, request.near_y, request.use_near)
        return response

    def update_landmark(self, request, response):
        name = self._normalize(request.name)
        if not name:
            response.success = False
            response.message = 'landmark name is required'
            return response
        if request.remove:
            existed = self.landmarks.pop(name, None) is not None
            response.success = existed
            response.message = 'removed' if existed else 'landmark not found'
            return response

        target_type = self._normalize(request.target_type) or 'landmark'
        pose = request.pose
        if not pose.header.frame_id:
            pose.header.frame_id = self.global_frame
        aliases = [self._normalize(alias) for alias in request.aliases if self._normalize(alias)]
        self.landmarks[name] = Landmark(name, target_type, pose, aliases)
        response.success = True
        response.message = f"updated semantic landmark '{name}'"
        return response

    def list_landmarks(self, request, response):
        target_type = self._normalize(request.target_type)
        response.landmarks = [
            lm.to_json() for lm in self.landmarks.values()
            if not target_type or lm.target_type == target_type
        ]
        return response

    def _load_landmarks(self, values) -> None:
        for item in values:
            parts = str(item).split('|')
            if len(parts) < 5:
                self.get_logger().warn(
                    f"Ignoring landmark '{item}'; expected name|type|x|y|yaw|aliases")
                continue
            try:
                name = self._normalize(parts[0])
                target_type = self._normalize(parts[1]) or 'landmark'
                x = float(parts[2])
                y = float(parts[3])
                yaw = float(parts[4])
            except ValueError:
                self.get_logger().warn(f"Ignoring landmark '{item}'; invalid pose")
                continue
            aliases = []
            if len(parts) >= 6:
                aliases = [self._normalize(alias) for alias in parts[5].split(',') if self._normalize(alias)]
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = yaw_to_quaternion(yaw)
            self.landmarks[name] = Landmark(name, target_type, pose, aliases)

    def _rank(self, candidates, near_x: float, near_y: float, use_near: bool):
        return sorted(candidates, key=lambda lm: self._score(lm, near_x, near_y, use_near))

    def _score(self, landmark: Landmark, near_x: float, near_y: float, use_near: bool) -> float:
        if not use_near:
            return 0.0
        dx = landmark.pose.pose.position.x - near_x
        dy = landmark.pose.pose.position.y - near_y
        return math.hypot(dx, dy)

    def _normalize(self, text: str) -> str:
        return str(text or '').strip().lower()

    def _get_string_array_parameter(self, name: str):
        try:
            return self.get_parameter(name).value or []
        except ParameterUninitializedException:
            return []


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavflexSemanticMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
