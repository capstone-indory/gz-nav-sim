#!/usr/bin/env python3
"""Publish Nav2 /goal_pose from named destinations or compact 2D goals."""

from __future__ import annotations

import json
import math
import os
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PointStamped, Pose2D, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    import yaml
except ImportError:  # pragma: no cover - checked by setup scripts
    yaml = None


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _coerce_goal(name: str, value: Any) -> Optional[tuple[float, float, float]]:
    if isinstance(value, dict):
        try:
            x = float(value['x'])
            y = float(value['y'])
        except (KeyError, TypeError, ValueError):
            return None
        yaw_value = value.get('yaw', value.get('theta', 0.0))
        try:
            yaw = float(yaw_value)
        except (TypeError, ValueError):
            return None
        return x, y, yaw

    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            x = float(value[0])
            y = float(value[1])
            yaw = float(value[2]) if len(value) >= 3 else 0.0
        except (TypeError, ValueError):
            return None
        return x, y, yaw

    return None


class NavDestinationNode(Node):
    def __init__(self) -> None:
        super().__init__('nav_destination_node')

        self.declare_parameter('destinations_file', '')
        self.declare_parameter('destination_topic', '/nav/destination')
        self.declare_parameter('goal_pose2d_topic', '/nav/goal_pose2d')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('enable_clicked_point_goal', False)
        self.declare_parameter('clicked_point_yaw', 0.0)

        g = lambda name: self.get_parameter(name).value
        self._frame_id = str(g('frame_id'))
        self._clicked_point_yaw = float(g('clicked_point_yaw'))
        self._destinations = self._load_destinations(str(g('destinations_file')))

        self._pub = self.create_publisher(PoseStamped, str(g('goal_topic')), 10)
        self.create_subscription(String, str(g('destination_topic')), self._on_destination, 10)
        self.create_subscription(Pose2D, str(g('goal_pose2d_topic')), self._on_pose2d, 10)
        if bool(g('enable_clicked_point_goal')):
            self.create_subscription(PointStamped, str(g('clicked_point_topic')), self._on_clicked_point, 10)

        names = ', '.join(sorted(self._destinations.keys())) or '(none)'
        self.get_logger().info(
            f'nav_destination: {g("destination_topic")} and {g("goal_pose2d_topic")} '
            f'-> {g("goal_topic")}; destinations={names}')

    def _load_destinations(self, path: str) -> dict[str, tuple[float, float, float]]:
        if not path:
            return {}
        expanded = os.path.expanduser(path)
        if not os.path.exists(expanded):
            self.get_logger().warn(f'destinations file not found: {expanded}')
            return {}

        try:
            with open(expanded, 'r', encoding='utf-8') as f:
                if expanded.endswith('.json'):
                    raw = json.load(f)
                else:
                    if yaml is None:
                        raise RuntimeError('python3-yaml/PyYAML is required for YAML destination files')
                    raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f'failed to load destinations from {expanded}: {exc}')
            return {}

        entries = raw.get('destinations', raw) if isinstance(raw, dict) else {}
        goals: dict[str, tuple[float, float, float]] = {}
        if not isinstance(entries, dict):
            self.get_logger().warn(f'destinations file has no mapping: {expanded}')
            return goals

        for name, value in entries.items():
            goal = _coerce_goal(str(name), value)
            if goal is None:
                self.get_logger().warn(f'ignoring invalid destination: {name}')
                continue
            goals[str(name)] = goal
        return goals

    def _on_destination(self, msg: String) -> None:
        name = msg.data.strip()
        if not name:
            return
        goal = self._destinations.get(name)
        if goal is None:
            self.get_logger().warn(f'unknown destination "{name}"')
            return
        self._publish_goal(*goal, source=f'destination:{name}')

    def _on_pose2d(self, msg: Pose2D) -> None:
        self._publish_goal(float(msg.x), float(msg.y), float(msg.theta), source='pose2d')

    def _on_clicked_point(self, msg: PointStamped) -> None:
        frame_id = msg.header.frame_id or self._frame_id
        self._publish_goal(
            float(msg.point.x),
            float(msg.point.y),
            self._clicked_point_yaw,
            frame_id=frame_id,
            source='clicked_point')

    def _publish_goal(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        frame_id: Optional[str] = None,
        source: str,
    ) -> None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = frame_id or self._frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        qz, qw = _yaw_to_quaternion(yaw)
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self._pub.publish(goal)
        self.get_logger().info(
            f'published {source} goal: frame={goal.header.frame_id} '
            f'x={x:.3f} y={y:.3f} yaw={yaw:.3f}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavDestinationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
