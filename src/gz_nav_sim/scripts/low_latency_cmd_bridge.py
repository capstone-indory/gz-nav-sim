#!/usr/bin/env python3
"""Small command-only bridge for low-latency teleop forwarding."""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def split_topics(value) -> list[str]:
    if isinstance(value, (list, tuple)):
        raw = []
        for item in value:
            raw.extend(str(item).replace(';', ',').split(','))
    else:
        raw = str(value).replace(';', ',').split(',')
    topics: list[str] = []
    seen: set[str] = set()
    for topic in raw:
        topic = topic.strip()
        if not topic or topic in seen:
            continue
        seen.add(topic)
        topics.append(topic)
    return topics


class LowLatencyCmdBridge(Node):
    def __init__(self) -> None:
        super().__init__('low_latency_cmd_bridge')
        self.declare_parameter(
            'cmd_vel_in_topics',
            '/cmd_vel_teleop,/cmd_teleop,/cmd_vel_mux,/cmd_vel',
        )
        self.declare_parameter('cmd_vel_out_topic', '/xlerobot/cmd_vel')
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('repeat_rate_hz', 250.0)
        self.declare_parameter('max_linear_x', 0.30)
        self.declare_parameter('max_linear_y', 0.30)
        self.declare_parameter('max_angular_z', 1.00)

        g = lambda name: self.get_parameter(name).value
        self._cmd_in_topics = split_topics(g('cmd_vel_in_topics'))
        self._cmd_timeout_sec = float(g('cmd_timeout_sec'))
        self._max_linear_x = float(g('max_linear_x'))
        self._max_linear_y = float(g('max_linear_y'))
        self._max_angular_z = float(g('max_angular_z'))
        self._lock = threading.Lock()
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._sent_stale_zero = True
        self._cmd_by_topic: dict[str, tuple[Twist, float]] = {}
        self._topic_priority = {
            topic: priority
            for priority, topic in enumerate(self._cmd_in_topics)
        }

        out_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub_cmd = self.create_publisher(Twist, str(g('cmd_vel_out_topic')), out_qos)
        for topic in self._cmd_in_topics:
            self.create_subscription(
                Twist,
                topic,
                lambda msg, source_topic=topic: self._on_cmd_vel(msg, source_topic),
                10,
            )
        period = 1.0 / max(1.0, float(g('repeat_rate_hz')))
        self.create_timer(period, self._tick_cmd)
        self.get_logger().info(
            f'low_latency_cmd_bridge active: {self._cmd_in_topics} -> {g("cmd_vel_out_topic")} '
            f'at repeat_rate_hz={1.0 / period:.1f}')

    def _on_cmd_vel(self, msg: Twist, source_topic: str) -> None:
        out = self._clamp_twist(msg)
        now = time.monotonic()
        with self._lock:
            self._cmd_by_topic[source_topic] = (out, now)
            self._last_cmd, self._last_cmd_time = self._select_active_locked(now)
            self._sent_stale_zero = False
            selected = self._last_cmd
        self._pub_cmd.publish(selected)

    def _tick_cmd(self) -> None:
        now = time.monotonic()
        with self._lock:
            msg, stamp = self._select_active_locked(now)
            self._last_cmd = msg
            self._last_cmd_time = stamp
            if stamp <= 0.0:
                if self._sent_stale_zero:
                    return
                msg = Twist()
                self._sent_stale_zero = True
            else:
                self._sent_stale_zero = False
        self._pub_cmd.publish(msg)

    def _select_active_locked(self, now: float) -> tuple[Twist, float]:
        active: list[tuple[int, float, Twist]] = []
        for topic, (msg, stamp) in list(self._cmd_by_topic.items()):
            age = now - stamp
            if self._cmd_timeout_sec > 0.0 and age > self._cmd_timeout_sec:
                continue
            active.append((self._topic_priority.get(topic, len(self._topic_priority)), stamp, msg))
        if not active:
            return Twist(), 0.0
        priority, stamp, msg = min(active, key=lambda item: (item[0], -item[1]))
        return msg, stamp

    @staticmethod
    def _clip_abs(value: float, limit: float) -> float:
        limit = abs(float(limit))
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, float(value)))

    def _clamp_twist(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = self._clip_abs(msg.linear.x, self._max_linear_x)
        out.linear.y = self._clip_abs(msg.linear.y, self._max_linear_y)
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = self._clip_abs(msg.angular.z, self._max_angular_z)
        return out


def main() -> None:
    rclpy.init()
    node = LowLatencyCmdBridge()
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
