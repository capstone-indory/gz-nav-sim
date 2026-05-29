#!/usr/bin/env python3
"""Convert a metric depth image into a sparse PointCloud2 for Nav2 costmaps."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


class DepthToPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_to_pointcloud_node')
        self.declare_parameter('depth_topic', '/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth/camera_info')
        self.declare_parameter('points_topic', '/depth/points')
        self.declare_parameter('stride', 4)
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 4.50)
        self.declare_parameter('max_points', 12000)
        self.declare_parameter('frame_id', '')

        self._bridge = CvBridge()
        self._camera_info: Optional[CameraInfo] = None
        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        out_qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('points_topic').value),
            out_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            sensor_qos,
        )
        self.get_logger().info(
            'depth_to_pointcloud ready: '
            f'{self.get_parameter("depth_topic").value} + '
            f'{self.get_parameter("camera_info_topic").value} -> '
            f'{self.get_parameter("points_topic").value}'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_depth(self, msg: Image) -> None:
        info = self._camera_info
        if info is None:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'depth decode failed: {exc}')
            return
        depth_m = self._to_meters(np.asarray(depth), msg.encoding)
        if depth_m is None:
            return

        stride = max(1, int(self.get_parameter('stride').value))
        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        max_points = max(1, int(self.get_parameter('max_points').value))

        k = np.asarray(info.k, dtype=np.float32).reshape(3, 3)
        fx, fy = float(k[0, 0]), float(k[1, 1])
        cx, cy = float(k[0, 2]), float(k[1, 2])
        if fx == 0.0 or fy == 0.0:
            return

        sampled = depth_m[::stride, ::stride]
        rows, cols = np.indices(sampled.shape)
        z = sampled.reshape(-1)
        valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
        if not np.any(valid):
            return

        u = (cols.reshape(-1).astype(np.float32) * stride)[valid]
        v = (rows.reshape(-1).astype(np.float32) * stride)[valid]
        z = z[valid].astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.column_stack((x, y, z))
        if points.shape[0] > max_points:
            step = int(math.ceil(points.shape[0] / max_points))
            points = points[::step]

        header = msg.header
        frame_id = str(self.get_parameter('frame_id').value).strip()
        if frame_id:
            header.frame_id = frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self._pub.publish(cloud)

    def _to_meters(self, depth: np.ndarray, encoding: str) -> Optional[np.ndarray]:
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        encoding = (encoding or '').upper()
        if depth.dtype == np.uint16 or encoding == '16UC1':
            return depth.astype(np.float32) * 0.001
        if depth.dtype == np.float32 or encoding == '32FC1':
            return depth.astype(np.float32)
        self.get_logger().warn(f'unsupported depth encoding for pointcloud: {encoding or depth.dtype}')
        return None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
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
