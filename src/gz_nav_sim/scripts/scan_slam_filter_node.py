#!/usr/bin/env python3
"""Filter LaserScan input for SLAM or Nav2 obstacle avoidance."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanSlamFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_slam_filter_node')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_slam')
        self.declare_parameter('min_range_m', 0.20)
        self.declare_parameter('max_range_m', 0.0)
        self.declare_parameter('remove_isolated_clusters', True)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('cluster_jump_m', 0.30)
        self.declare_parameter('cluster_max_range_m', 2.5)
        self.declare_parameter('temporal_static_filter', False)
        self.declare_parameter('stable_observations', 4)
        self.declare_parameter('stable_tolerance_m', 0.22)
        self.declare_parameter('stable_max_misses', 8)
        self.declare_parameter('stable_alpha', 0.15)
        self.declare_parameter('stable_seconds', 2.0)

        g = lambda name: self.get_parameter(name).value
        self._min_range = max(0.0, float(g('min_range_m')))
        self._max_range = max(0.0, float(g('max_range_m')))
        self._remove_clusters = bool(g('remove_isolated_clusters'))
        self._min_cluster_points = max(1, int(g('min_cluster_points')))
        self._cluster_jump = max(0.01, float(g('cluster_jump_m')))
        self._cluster_max_range = max(0.0, float(g('cluster_max_range_m')))
        self._temporal_static = bool(g('temporal_static_filter'))
        self._stable_observations = max(1, int(g('stable_observations')))
        self._stable_tolerance = max(0.01, float(g('stable_tolerance_m')))
        self._stable_max_misses = max(1, int(g('stable_max_misses')))
        self._stable_alpha = min(1.0, max(0.0, float(g('stable_alpha'))))
        self._stable_seconds = max(0.0, float(g('stable_seconds')))
        self._stable: list[float] = []
        self._candidate: list[float] = []
        self._candidate_count: list[int] = []
        self._candidate_first_seen: list[float] = []
        self._miss_count: list[int] = []

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(LaserScan, str(g('output_topic')), qos)
        self.create_subscription(LaserScan, str(g('input_topic')), self._on_scan, qos)

        self.get_logger().info(
            f'scan_filter: {g("input_topic")} -> {g("output_topic")}, '
            f'min_range={self._min_range:.2f}m')

    def _on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(float(msg.range_min), self._min_range)
        out.range_max = float(msg.range_max)
        out.intensities = list(msg.intensities)

        ranges = []
        max_range = self._max_range if self._max_range > 0.0 else float(msg.range_max)
        for value in msg.ranges:
            r = float(value)
            if not math.isfinite(r) or r < self._min_range or r > max_range:
                ranges.append(math.inf)
            else:
                ranges.append(r)

        if self._remove_clusters:
            self._remove_small_near_clusters(ranges)
        if self._temporal_static:
            ranges = self._temporal_static_ranges(ranges, out.range_min, out.range_max)

        out.ranges = ranges
        self._pub.publish(out)

    def _temporal_static_ranges(
        self, ranges: list[float], range_min: float, range_max: float
    ) -> list[float]:
        n = len(ranges)
        if len(self._stable) != n:
            self._stable = [math.inf] * n
            self._candidate = [math.inf] * n
            self._candidate_count = [0] * n
            self._candidate_first_seen = [0.0] * n
            self._miss_count = [0] * n

        out = [math.inf] * n
        now = self.get_clock().now().nanoseconds / 1e9
        for index, value in enumerate(ranges):
            r = float(value)
            valid = math.isfinite(r) and range_min <= r <= range_max
            old = float(self._stable[index])
            old_valid = math.isfinite(old)

            if not valid:
                if old_valid:
                    self._miss_count[index] += 1
                    if self._miss_count[index] <= self._stable_max_misses:
                        out[index] = old
                        continue
                    self._stable[index] = math.inf
                self._candidate[index] = math.inf
                self._candidate_count[index] = 0
                self._candidate_first_seen[index] = 0.0
                self._miss_count[index] = 0
                continue

            self._miss_count[index] = 0
            if old_valid and abs(r - old) <= self._stable_tolerance:
                self._stable[index] = (
                    old * (1.0 - self._stable_alpha) + r * self._stable_alpha)
                self._candidate[index] = math.inf
                self._candidate_count[index] = 0
                self._candidate_first_seen[index] = 0.0
                out[index] = self._stable[index]
                continue

            cand = float(self._candidate[index])
            if math.isfinite(cand) and abs(r - cand) <= self._stable_tolerance:
                self._candidate[index] = (cand + r) * 0.5
                self._candidate_count[index] += 1
            else:
                self._candidate[index] = r
                self._candidate_count[index] = 1
                self._candidate_first_seen[index] = now

            stable_for_s = now - (self._candidate_first_seen[index] or now)
            if (
                self._candidate_count[index] >= self._stable_observations
                and stable_for_s >= self._stable_seconds
            ):
                self._stable[index] = float(self._candidate[index])
                self._candidate[index] = math.inf
                self._candidate_count[index] = 0
                self._candidate_first_seen[index] = 0.0
                out[index] = self._stable[index]
            elif old_valid:
                out[index] = old

        return out

    def _remove_small_near_clusters(self, ranges: list[float]) -> None:
        start: Optional[int] = None
        previous: Optional[float] = None
        clusters: list[tuple[int, int]] = []

        for index, value in enumerate(ranges):
            valid = math.isfinite(value)
            if not valid:
                if start is not None:
                    clusters.append((start, index))
                start = None
                previous = None
                continue
            if start is None:
                start = index
            elif previous is not None and abs(value - previous) > self._cluster_jump:
                clusters.append((start, index))
                start = index
            previous = value

        if start is not None:
            clusters.append((start, len(ranges)))

        for start, end in clusters:
            count = end - start
            if count >= self._min_cluster_points:
                continue
            finite = [ranges[i] for i in range(start, end) if math.isfinite(ranges[i])]
            if not finite:
                continue
            if sum(finite) / len(finite) <= self._cluster_max_range:
                for i in range(start, end):
                    ranges[i] = math.inf


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ScanSlamFilterNode()
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
