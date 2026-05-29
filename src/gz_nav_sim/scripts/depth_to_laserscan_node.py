#!/usr/bin/env python3
"""Convert a mounted depth sensor depth image into a horizontal LaserScan.

This is a hardware fallback for when the physical RPLIDAR is present but not
producing frames. It uses RGB-D depth only, not wheel/base odometry.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class DepthToLaserScanNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_to_laserscan_node')

        self.declare_parameter('depth_topic', '/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth/camera_info')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('scan_frame', 'base_link')
        self.declare_parameter('scan_height_px', 24)
        self.declare_parameter('range_min_m', 0.20)
        self.declare_parameter('range_max_m', 4.50)
        self.declare_parameter('publish_rate_hz', 10.0)

        g = lambda name: self.get_parameter(name).value
        self._scan_topic = str(g('scan_topic'))
        self._scan_frame = str(g('scan_frame'))
        self._scan_height = max(1, int(g('scan_height_px')))
        self._range_min = max(0.0, float(g('range_min_m')))
        self._range_max = max(self._range_min + 0.01, float(g('range_max_m')))
        self._min_period = 1.0 / max(1.0, float(g('publish_rate_hz')))
        self._last_pub_time = 0.0
        self._info: Optional[CameraInfo] = None

        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(LaserScan, self._scan_topic, qos)
        self.create_subscription(CameraInfo, str(g('camera_info_topic')), self._on_info, qos)
        self.create_subscription(Image, str(g('depth_topic')), self._on_depth, qos)

        self.get_logger().info(
            f'depth_to_laserscan: {g("depth_topic")} -> {self._scan_topic}, '
            f'frame={self._scan_frame}')

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_depth(self, msg: Image) -> None:
        now = self.get_clock().now()
        now_sec = float(now.nanoseconds) * 1e-9
        if now_sec - self._last_pub_time < self._min_period:
            return

        depth = self._decode_depth(msg)
        if depth is None or depth.size == 0:
            return

        height, width = depth.shape
        row_mid = height // 2
        half = max(0, self._scan_height // 2)
        row0 = max(0, row_mid - half)
        row1 = min(height, row_mid + half + 1)
        band = depth[row0:row1, :]

        valid = np.isfinite(band) & (band >= self._range_min) & (band <= self._range_max)
        ranges_by_col = np.full(width, math.inf, dtype=np.float32)
        if np.any(valid):
            masked = np.where(valid, band, np.inf)
            ranges_by_col = np.min(masked, axis=0).astype(np.float32)
        else:
            return

        fx, cx = self._intrinsics(width, height)
        cols = np.arange(width, dtype=np.float32)
        x_right = (cols - cx) / fx * ranges_by_col
        forward = ranges_by_col
        angles = np.arctan2(-x_right, forward)
        planar_ranges = np.hypot(x_right, forward)

        finite = np.isfinite(planar_ranges) & (planar_ranges >= self._range_min) & (planar_ranges <= self._range_max)
        if int(np.count_nonzero(finite)) < 20:
            return

        order = np.argsort(angles)
        angles = angles[order]
        planar_ranges = planar_ranges[order]
        finite = finite[order]
        output = np.where(finite, planar_ranges, np.inf).astype(np.float32)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self._scan_frame
        scan.angle_min = float(angles[0])
        scan.angle_max = float(angles[-1])
        scan.angle_increment = (scan.angle_max - scan.angle_min) / float(max(1, width - 1))
        scan.time_increment = 0.0
        scan.scan_time = self._min_period
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = output.tolist()
        self._pub.publish(scan)
        self._last_pub_time = now_sec

    def _intrinsics(self, width: int, height: int) -> tuple[float, float]:
        if self._info is not None and self._info.k[0] > 0.0:
            return float(self._info.k[0]), float(self._info.k[2])
        return float(max(width, height)), (float(width) - 1.0) * 0.5

    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                array = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                return array.astype(np.float32) * 0.001
            if msg.encoding in ('32FC1',):
                return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        except Exception as exc:
            self.get_logger().warn(f'depth decode failed: {exc}')
            return None
        self.get_logger().warn(f'unsupported depth encoding: {msg.encoding}')
        return None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
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
