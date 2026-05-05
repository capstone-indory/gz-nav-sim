#!/usr/bin/env python3
"""Publish a lightweight accumulated trajectory as nav_msgs/Path."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class TrajectoryPathNode(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_path_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/trajectory')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('max_poses', 2000)
        self.declare_parameter('min_translation_m', 0.05)
        self.declare_parameter('min_rotation_rad', 0.05)
        self.declare_parameter('tf_timeout_s', 0.1)
        g = lambda name: self.get_parameter(name).value
        self._target_frame = str(g('target_frame'))
        self._max_poses = max(1, int(g('max_poses')))
        self._min_translation_m = max(0.0, float(g('min_translation_m')))
        self._min_rotation_rad = max(0.0, float(g('min_rotation_rad')))
        self._tf_timeout = Duration(seconds=float(g('tf_timeout_s')))

        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._pub = self.create_publisher(Path, str(g('path_topic')), latched_qos)
        self.create_subscription(Odometry, str(g('odom_topic')), self._on_odom, sensor_qos)

        self._path = Path()
        self._path.header.frame_id = self._target_frame
        self._last_pose: PoseStamped | None = None

        self.get_logger().info(
            f'trajectory path ready. odom={g("odom_topic")} path={g("path_topic")} '
            f'target_frame={self._target_frame}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        if pose.header.frame_id != self._target_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    pose.header.frame_id,
                    pose.header.stamp,
                    timeout=self._tf_timeout,
                )
                pose = do_transform_pose_stamped(pose, transform)
            except TransformException as exc:
                self.get_logger().debug(f'failed to transform odom pose: {exc}')
                return

        if self._last_pose is not None and not self._should_append(pose, self._last_pose):
            return

        self._last_pose = pose
        self._path.header.stamp = pose.header.stamp
        self._path.header.frame_id = self._target_frame
        self._path.poses.append(pose)
        if len(self._path.poses) > self._max_poses:
            self._path.poses = self._path.poses[-self._max_poses:]
        self._pub.publish(self._path)

    def _should_append(self, current: PoseStamped, last: PoseStamped) -> bool:
        dx = current.pose.position.x - last.pose.position.x
        dy = current.pose.position.y - last.pose.position.y
        dz = current.pose.position.z - last.pose.position.z
        translation = math.sqrt(dx * dx + dy * dy + dz * dz)
        if translation >= self._min_translation_m:
            return True

        yaw_current = self._yaw_from_quaternion(current.pose.orientation)
        yaw_last = self._yaw_from_quaternion(last.pose.orientation)
        delta_yaw = math.atan2(math.sin(yaw_current - yaw_last), math.cos(yaw_current - yaw_last))
        return abs(delta_yaw) >= self._min_rotation_rad

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main() -> None:
    rclpy.init()
    node = TrajectoryPathNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
