#!/usr/bin/env python3
"""Latest-only downsampled point cloud republisher for Foxglove.

Internal navigation can keep using the full `/camera/points` stream, while
remote monitoring subscribes to a cheaper visualization topic.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointcloudVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_visualizer_node')

        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('output_topic', '/camera/points_visual')
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('stride', 6)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('voxel_size_m', 0.10)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            out_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self._on_cloud,
            sensor_qos,
        )
        self._last_pub_time = 0.0

        self.get_logger().info(
            'pointcloud visualizer ready. %s -> %s'
            % (
                self.get_parameter('input_topic').value,
                self.get_parameter('output_topic').value,
            )
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        max_rate_hz = max(0.1, float(self.get_parameter('max_rate_hz').value))
        if now - self._last_pub_time < 1.0 / max_rate_hz:
            return

        points = point_cloud2.read_points(msg, skip_nans=True)
        if points.size == 0:
            return

        stride = max(1, int(self.get_parameter('stride').value))
        sampled = points[::stride]
        sampled = self._voxel_downsample(sampled)

        max_points = max(1000, int(self.get_parameter('max_points').value))
        if sampled.shape[0] > max_points:
            indices = np.linspace(0, sampled.shape[0] - 1, max_points, dtype=np.int64)
            sampled = sampled[indices]

        cloud = point_cloud2.create_cloud(msg.header, msg.fields, sampled.tolist())
        cloud.is_dense = False
        self._pub.publish(cloud)
        self._last_pub_time = now

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        voxel = max(0.0, float(self.get_parameter('voxel_size_m').value))
        if voxel <= 0.0 or points.shape[0] < 2:
            return points
        names = points.dtype.names or ()
        if not {'x', 'y', 'z'}.issubset(names):
            return points
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
        keys = np.floor(xyz / voxel).astype(np.int32)
        _, keep = np.unique(keys, axis=0, return_index=True)
        keep.sort()
        return points[keep]


def main() -> None:
    rclpy.init()
    node = PointcloudVisualizerNode()
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
