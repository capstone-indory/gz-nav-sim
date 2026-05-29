#!/usr/bin/env python3
"""LiDAR-only scan-to-scan odometry for feeding slam_toolbox/Nav2.

This deliberately does not use wheel odometry. It estimates base motion by
matching consecutive 2D LaserScan frames and publishes odom->base_link.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LaserScanOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('laser_scan_odometry_node')

        self.declare_parameter('scan_topic', '/scan_slam')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('min_range_m', 0.20)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('max_points', 240)
        self.declare_parameter('icp_iterations', 8)
        self.declare_parameter('max_correspondence_distance_m', 0.35)
        self.declare_parameter('min_pairs', 35)
        self.declare_parameter('max_translation_per_scan_m', 0.35)
        self.declare_parameter('max_rotation_per_scan_rad', 0.60)
        self.declare_parameter('invert_delta', False)

        g = lambda name: self.get_parameter(name).value
        self._odom_topic = str(g('odom_topic'))
        self._odom_frame = str(g('odom_frame'))
        self._base_frame = str(g('base_frame'))
        self._publish_tf = bool(g('publish_tf'))
        self._min_range = max(0.0, float(g('min_range_m')))
        self._max_range = max(self._min_range, float(g('max_range_m')))
        self._max_points = max(40, int(g('max_points')))
        self._icp_iterations = max(1, int(g('icp_iterations')))
        self._max_corr = max(0.05, float(g('max_correspondence_distance_m')))
        self._min_pairs = max(6, int(g('min_pairs')))
        self._max_translation = max(0.01, float(g('max_translation_per_scan_m')))
        self._max_rotation = max(0.01, float(g('max_rotation_per_scan_rad')))
        self._invert_delta = bool(g('invert_delta'))

        self._prev_points: Optional[np.ndarray] = None
        self._prev_stamp = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._wz = 0.0
        self._rejected = 0

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._tf = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(LaserScan, str(g('scan_topic')), self._on_scan, qos)

        self.get_logger().info(
            f'laser_scan_odometry: {g("scan_topic")} -> {self._odom_topic}, '
            'wheel odom disabled')

    def _on_scan(self, msg: LaserScan) -> None:
        points = self._scan_to_points(msg)
        if points.shape[0] < self._min_pairs:
            self.get_logger().warn('not enough valid scan points for lidar odom')
            return

        if self._prev_points is None:
            self._prev_points = points
            self._prev_stamp = msg.header.stamp
            self._publish(msg, 0.0, 0.0)
            return

        result = self._estimate_delta(points, self._prev_points)
        if result is None:
            self._rejected += 1
            if self._rejected % 20 == 1:
                self.get_logger().warn('scan matching rejected; holding last lidar odom')
            self._publish(msg, 0.0, 0.0)
            return

        dx, dy, dyaw, pairs, rmse = result
        if self._invert_delta:
            c = math.cos(dyaw)
            s = math.sin(dyaw)
            inv_x = -(c * dx + s * dy)
            inv_y = -(-s * dx + c * dy)
            dx, dy, dyaw = inv_x, inv_y, -dyaw

        if math.hypot(dx, dy) > self._max_translation or abs(dyaw) > self._max_rotation:
            self._rejected += 1
            if self._rejected % 20 == 1:
                self.get_logger().warn(
                    f'scan delta too large; rejected dx={dx:.3f}, dy={dy:.3f}, '
                    f'dyaw={dyaw:.3f}, pairs={pairs}, rmse={rmse:.3f}')
            self._publish(msg, 0.0, 0.0)
            return

        dt = self._stamp_dt(self._prev_stamp, msg.header.stamp)
        world_dx = math.cos(self._yaw) * dx - math.sin(self._yaw) * dy
        world_dy = math.sin(self._yaw) * dx + math.cos(self._yaw) * dy
        self._x += world_dx
        self._y += world_dy
        self._yaw = normalize_angle(self._yaw + dyaw)
        if dt > 1e-4:
            self._vx = dx / dt
            self._wz = dyaw / dt

        self._prev_points = points
        self._prev_stamp = msg.header.stamp
        self._publish(msg, self._vx, self._wz)

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        points = []
        angle = float(msg.angle_min)
        for value in msg.ranges:
            r = float(value)
            if math.isfinite(r) and self._min_range <= r <= self._max_range:
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += float(msg.angle_increment)

        if len(points) > self._max_points:
            step = max(1, len(points) // self._max_points)
            points = points[::step][:self._max_points]
        return np.asarray(points, dtype=np.float64)

    def _estimate_delta(self, current: np.ndarray, previous: np.ndarray) -> Optional[tuple[float, float, float, int, float]]:
        rotation = np.eye(2)
        translation = np.zeros(2)
        pair_count = 0
        rmse = math.inf

        for _ in range(self._icp_iterations):
            transformed = current @ rotation.T + translation
            diff = transformed[:, None, :] - previous[None, :, :]
            dist2 = np.sum(diff * diff, axis=2)
            nearest = np.argmin(dist2, axis=1)
            nearest_dist2 = dist2[np.arange(dist2.shape[0]), nearest]
            mask = nearest_dist2 < self._max_corr * self._max_corr
            pair_count = int(np.count_nonzero(mask))
            if pair_count < self._min_pairs:
                return None

            src = transformed[mask]
            dst = previous[nearest[mask]]
            delta_r, delta_t = self._fit_transform(src, dst)
            rotation = delta_r @ rotation
            translation = delta_r @ translation + delta_t
            rmse = float(math.sqrt(np.mean(nearest_dist2[mask])))
            if np.linalg.norm(delta_t) < 1e-4 and abs(math.atan2(delta_r[1, 0], delta_r[0, 0])) < 1e-4:
                break

        dyaw = math.atan2(rotation[1, 0], rotation[0, 0])
        return float(translation[0]), float(translation[1]), float(dyaw), pair_count, rmse

    @staticmethod
    def _fit_transform(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        src_mean = np.mean(src, axis=0)
        dst_mean = np.mean(dst, axis=0)
        src_centered = src - src_mean
        dst_centered = dst - dst_mean
        h = src_centered.T @ dst_centered
        u, _s, vt = np.linalg.svd(h)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1
            rotation = vt.T @ u.T
        translation = dst_mean - rotation @ src_mean
        return rotation, translation

    @staticmethod
    def _stamp_dt(prev_stamp, stamp) -> float:
        if prev_stamp is None:
            return 0.0
        prev = float(prev_stamp.sec) + float(prev_stamp.nanosec) * 1e-9
        now = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return max(0.0, min(now - prev, 1.0))

    def _publish(self, scan: LaserScan, linear_x: float, angular_z: float) -> None:
        quat = yaw_to_quaternion(self._yaw)
        msg = Odometry()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = quat
        msg.pose.covariance[0] = 0.08
        msg.pose.covariance[7] = 0.08
        msg.pose.covariance[35] = 0.20
        msg.twist.twist.linear.x = linear_x
        msg.twist.twist.angular.z = angular_z
        msg.twist.covariance[0] = 0.20
        msg.twist.covariance[35] = 0.30
        self._pub.publish(msg)

        if self._tf is not None:
            tf = TransformStamped()
            tf.header.stamp = scan.header.stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = quat
            self._tf.sendTransform(tf)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LaserScanOdometryNode()
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
