#!/usr/bin/env python3
"""ROS topic bridge for the XLeRobot Hospital Isaac Sim v2 app.

The v2 Isaac app connects to rosbridge_server itself and publishes native ROS 2
topics under /xlerobot. This node keeps the rest of gz_nav_sim unchanged by
aliasing that interface to the legacy topics used by Nav2/RTAB-Map:

  /xlerobot/cmd_vel                         <- /cmd_vel_mux
  /xlerobot/odom                            -> /odom
  /xlerobot/head_camera/color/image_raw       -> /camera/image_raw
  /xlerobot/head_camera/color/image           -> /camera/image_raw/compressed
  /xlerobot/head_camera/color/camera_info     -> /camera/camera_info
  /xlerobot/head_camera/depth/image_rect_raw  -> /depth/image_raw
  /xlerobot/head_camera/depth/camera_info     -> /depth/camera_info
  /xlerobot/head_camera/imu                    -> /imu/data
  /xlerobot/imu/data                        -> /imu/data
  /xlerobot/scan                            -> /scan or /scan_raw

Frame ids are normalized to the local stack's short names (odom/base_link/
camera_optical_frame) so existing Nav2 and semantic nodes do not need a second
set of parameters for xlerobot_* frames.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, LaserScan
from tf2_ros import TransformBroadcaster


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


class XLeRobotV2Bridge(Node):
    def __init__(self) -> None:
        super().__init__('xlerobot_v2_bridge')

        self.declare_parameter('namespace', '/xlerobot')
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('imu_frame', 'camera_imu_frame')
        self.declare_parameter('scan_frame', 'base_link')
        self.declare_parameter('enable_odom_bridge', True)
        self.declare_parameter('enable_scan_bridge', True)
        self.declare_parameter('enable_imu_bridge', True)
        self.declare_parameter('enable_cmd_bridge', True)

        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_mux')
        self.declare_parameter('cmd_vel_in_topics', '')
        self.declare_parameter('cmd_vel_out_topic', '/xlerobot/cmd_vel')
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('max_linear_x', 0.30)
        self.declare_parameter('max_linear_y', 0.30)
        self.declare_parameter('max_angular_z', 1.00)
        self.declare_parameter('odom_in_topic', '/xlerobot/odom')
        self.declare_parameter('odom_out_topic', '/odom')
        self.declare_parameter('rgb_image_in_topic', '')
        self.declare_parameter('rgb_compressed_image_in_topic', '/xlerobot/head_camera/color/image')
        self.declare_parameter('rgb_compressed_image_in_topics', '')
        self.declare_parameter('rgb_info_in_topic', '/xlerobot/head_camera/color/camera_info')
        self.declare_parameter('rgb_info_in_topics', '')
        self.declare_parameter('rgb_image_out_topic', '/camera/image_raw')
        self.declare_parameter('rgb_compressed_out_topic', '/camera/image_raw/compressed')
        self.declare_parameter('rgb_info_out_topic', '/camera/camera_info')
        self.declare_parameter('publish_rgb_raw', True)
        self.declare_parameter('depth_image_in_topic', '/xlerobot/head_camera/depth/image_rect_raw')
        self.declare_parameter('depth_image_in_topics', '')
        self.declare_parameter('depth_compressed_image_in_topic', '/xlerobot/head_camera/depth/image')
        self.declare_parameter('depth_compressed_image_in_topics', '')
        self.declare_parameter('depth_info_in_topic', '/xlerobot/head_camera/depth/camera_info')
        self.declare_parameter('depth_info_in_topics', '')
        self.declare_parameter('depth_image_out_topic', '/depth/image_raw')
        self.declare_parameter('depth_info_out_topic', '/depth/camera_info')
        self.declare_parameter('publish_depth_raw', True)
        self.declare_parameter('imu_in_topic', '/xlerobot/head_camera/imu')
        self.declare_parameter('imu_in_topics', '')
        self.declare_parameter('imu_out_topic', '/imu/data')
        self.declare_parameter('synthesize_camera_info', True)
        self.declare_parameter('scan_in_topic', '/xlerobot/scan')
        self.declare_parameter('scan_out_topic', '/scan')

        g = lambda name: self.get_parameter(name).value
        self._odom_frame = str(g('odom_frame'))
        self._base_frame = str(g('base_frame'))
        self._camera_frame = str(g('camera_optical_frame'))
        self._imu_frame = str(g('imu_frame'))
        self._scan_frame = str(g('scan_frame'))
        self._enable_odom_bridge = bool(g('enable_odom_bridge'))
        self._enable_scan_bridge = bool(g('enable_scan_bridge'))
        self._enable_imu_bridge = bool(g('enable_imu_bridge'))
        self._enable_cmd_bridge = bool(g('enable_cmd_bridge'))

        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        cmd_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self._cmd_group = MutuallyExclusiveCallbackGroup()
        self._sensor_group = MutuallyExclusiveCallbackGroup()

        self._pub_cmd = (
            self.create_publisher(Twist, str(g('cmd_vel_out_topic')), cmd_qos)
            if self._enable_cmd_bridge else None
        )
        self._pub_odom = (
            self.create_publisher(Odometry, str(g('odom_out_topic')), rel_qos)
            if self._enable_odom_bridge else None
        )
        self._pub_scan = (
            self.create_publisher(LaserScan, str(g('scan_out_topic')), rel_qos)
            if self._enable_scan_bridge else None
        )
        self._publish_rgb_raw = bool(g('publish_rgb_raw'))
        self._publish_depth_raw = bool(g('publish_depth_raw'))
        self._pub_rgb = (
            self.create_publisher(Image, str(g('rgb_image_out_topic')), sensor_qos)
            if self._publish_rgb_raw else None
        )
        self._pub_rgb_compressed = self.create_publisher(
            CompressedImage, str(g('rgb_compressed_out_topic')), sensor_qos)
        self._pub_rgb_info = self.create_publisher(CameraInfo, str(g('rgb_info_out_topic')), rel_qos)
        self._pub_depth = (
            self.create_publisher(Image, str(g('depth_image_out_topic')), sensor_qos)
            if self._publish_depth_raw else None
        )
        self._pub_depth_info = (
            self.create_publisher(CameraInfo, str(g('depth_info_out_topic')), rel_qos)
            if self._publish_depth_raw else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, str(g('imu_out_topic')), sensor_qos)
            if self._enable_imu_bridge else None
        )

        self._tf_bcast = TransformBroadcaster(self) if self._enable_odom_bridge else None
        self._cv_bridge = CvBridge()
        self._cmd_lock = threading.Lock()
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._sent_stale_zero = True
        self._cmd_timeout_sec = float(g('cmd_timeout_sec'))
        self._cmd_in_topics = split_topics(g('cmd_vel_in_topics') or g('cmd_vel_in_topic'))
        if not self._cmd_in_topics:
            self._cmd_in_topics = ['/cmd_vel_mux']
        self._cmd_in_topic = self._cmd_in_topics[0]
        self._max_linear_x = float(g('max_linear_x'))
        self._max_linear_y = float(g('max_linear_y'))
        self._max_angular_z = float(g('max_angular_z'))
        self._last_odom_stamp = None
        self._synthesize_camera_info = bool(g('synthesize_camera_info'))
        self._last_rgb_info_time = 0.0
        self._last_depth_info_time = 0.0
        self._synthetic_camera_size: Optional[tuple[int, int]] = None
        self._synthetic_depth_size: Optional[tuple[int, int]] = None
        self._synthetic_info_warned = False
        self._synthetic_depth_info_warned = False
        rgb_compressed_topics = split_topics(
            g('rgb_compressed_image_in_topics') or g('rgb_compressed_image_in_topic'))
        rgb_info_topics = split_topics(g('rgb_info_in_topics') or g('rgb_info_in_topic'))
        depth_image_topics = split_topics(g('depth_image_in_topics') or g('depth_image_in_topic'))
        depth_compressed_topics = split_topics(
            g('depth_compressed_image_in_topics') or g('depth_compressed_image_in_topic'))
        depth_info_topics = split_topics(g('depth_info_in_topics') or g('depth_info_in_topic'))
        imu_topics = split_topics(g('imu_in_topics') or g('imu_in_topic'))

        if self._enable_cmd_bridge:
            for topic in self._cmd_in_topics:
                self.create_subscription(
                    Twist, topic, self._on_cmd_vel, 10,
                    callback_group=self._cmd_group)
        if self._enable_odom_bridge:
            self.create_subscription(
                Odometry, str(g('odom_in_topic')), self._on_odom, sensor_qos,
                callback_group=self._sensor_group)
        rgb_image_in_topic = str(g('rgb_image_in_topic')).strip()
        if rgb_image_in_topic:
            self.create_subscription(
                Image, rgb_image_in_topic, self._on_rgb, sensor_qos,
                callback_group=self._sensor_group)
        for topic in rgb_compressed_topics:
            self.create_subscription(
                CompressedImage, topic, self._on_rgb_compressed, sensor_qos,
                callback_group=self._sensor_group)
        for topic in rgb_info_topics:
            self.create_subscription(
                CameraInfo, topic, self._on_rgb_info, sensor_qos,
                callback_group=self._sensor_group)
        if self._publish_depth_raw:
            for topic in depth_image_topics:
                self.create_subscription(
                    Image, topic, self._on_depth, sensor_qos,
                    callback_group=self._sensor_group)
            for topic in depth_compressed_topics:
                self.create_subscription(
                    CompressedImage, topic, self._on_depth_compressed, sensor_qos,
                    callback_group=self._sensor_group)
            for topic in depth_info_topics:
                self.create_subscription(
                    CameraInfo, topic, self._on_depth_info, sensor_qos,
                    callback_group=self._sensor_group)
        if self._enable_imu_bridge:
            for topic in imu_topics:
                self.create_subscription(
                    Imu, topic, self._on_imu, sensor_qos,
                    callback_group=self._sensor_group)
        if self._enable_scan_bridge:
            self.create_subscription(
                LaserScan, str(g('scan_in_topic')), self._on_scan, sensor_qos,
                callback_group=self._sensor_group)

        cmd_period = 1.0 / max(1.0, float(g('cmd_rate_hz')))
        if self._enable_cmd_bridge:
            self.create_timer(cmd_period, self._tick_cmd, callback_group=self._cmd_group)

        scan_out = str(g('scan_out_topic')) if self._enable_scan_bridge else ''
        odom_out = str(g('odom_out_topic')) if self._enable_odom_bridge else ''
        self.get_logger().info(
            'xlerobot_v2_bridge active: /xlerobot ROS topics -> '
            f'{odom_out} {scan_out} /camera/image_raw/compressed'
            f'{" /camera/image_raw" if self._publish_rgb_raw else ""}'
            f'{" /depth/image_raw" if self._publish_depth_raw else ""}'
            f'{" /imu/data" if self._enable_imu_bridge else ""}, '
            f'{self._cmd_in_topics} -> {g("cmd_vel_out_topic")}'
            f'{" disabled" if not self._enable_cmd_bridge else ""}')
        if self._enable_cmd_bridge:
            self.get_logger().info(
                'xlerobot_v2_bridge command limits: '
                f'linear.x={self._max_linear_x:.3f} m/s, '
                f'linear.y={self._max_linear_y:.3f} m/s, '
                f'angular.z={self._max_angular_z:.3f} rad/s')
        self.get_logger().info(
            'xlerobot_v2_bridge camera inputs: compressed='
            f'{rgb_compressed_topics}, info={rgb_info_topics}')
        if self._publish_depth_raw:
            self.get_logger().info(
                'xlerobot_v2_bridge depth inputs: raw='
                f'{depth_image_topics}, compressed={depth_compressed_topics}, '
                f'info={depth_info_topics}')
        if self._enable_imu_bridge:
            self.get_logger().info(f'xlerobot_v2_bridge imu inputs: {imu_topics}')

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._pub_cmd is None:
            return
        out = self._clamp_twist(msg)
        with self._cmd_lock:
            self._last_cmd = out
            self._last_cmd_time = time.monotonic()
            self._sent_stale_zero = False
        self._pub_cmd.publish(out)

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

    def _tick_cmd(self) -> None:
        if self._pub_cmd is None:
            return
        with self._cmd_lock:
            msg = self._last_cmd
            age = time.monotonic() - self._last_cmd_time
            stale = self._last_cmd_time <= 0.0 or (
                self._cmd_timeout_sec > 0.0 and age > self._cmd_timeout_sec
            )
            has_cmd_publisher = any(self.count_publishers(topic) > 0 for topic in self._cmd_in_topics)
            if not has_cmd_publisher or stale:
                if self._sent_stale_zero:
                    return
                msg = Twist()
                self._sent_stale_zero = True
            else:
                self._sent_stale_zero = False
        self._pub_cmd.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        if self._pub_odom is None or self._tf_bcast is None:
            return
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        self._last_odom_stamp = msg.header.stamp
        self._pub_odom.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = Quaternion(
            x=msg.pose.pose.orientation.x,
            y=msg.pose.pose.orientation.y,
            z=msg.pose.pose.orientation.z,
            w=msg.pose.pose.orientation.w,
        )
        self._tf_bcast.sendTransform(tf)

    def _on_rgb(self, msg: Image) -> None:
        msg.header.frame_id = self._camera_frame
        self._publish_synthetic_camera_info_if_needed(msg.header, msg.width, msg.height)
        if self._pub_rgb is not None:
            self._pub_rgb.publish(msg)
        try:
            image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        except Exception as exc:
            self.get_logger().warn(f'rgb compression failed: {exc}')
            return
        if not ok:
            return
        comp = CompressedImage()
        comp.header = msg.header
        comp.format = 'jpeg'
        comp.data = encoded.tobytes()
        self._pub_rgb_compressed.publish(comp)

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        self._last_rgb_info_time = time.monotonic()
        msg.header.frame_id = self._camera_frame
        self._pub_rgb_info.publish(msg)

    def _on_rgb_compressed(self, msg: CompressedImage) -> None:
        msg.header.frame_id = self._camera_frame
        self._pub_rgb_compressed.publish(msg)
        width_height = self._synthetic_camera_size
        if width_height is None and self._synthesize_camera_info:
            width_height = self._read_compressed_size(msg)
            self._synthetic_camera_size = width_height
        if width_height is not None:
            width, height = width_height
            self._publish_synthetic_camera_info_if_needed(msg.header, width, height)
        if self._pub_rgb is not None:
            raw = self._decode_rgb_compressed(msg)
            if raw is not None:
                self._pub_rgb.publish(raw)

    def _decode_rgb_compressed(self, msg: CompressedImage) -> Optional[Image]:
        try:
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        except Exception as exc:
            self.get_logger().warn(f'rgb compressed decode failed: {exc}')
            return None
        if image is None:
            self.get_logger().warn('rgb compressed decode returned None')
            return None
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        raw_msg = Image()
        raw_msg.header = msg.header
        raw_msg.height = int(image.shape[0])
        raw_msg.width = int(image.shape[1])
        raw_msg.encoding = 'rgb8'
        raw_msg.is_bigendian = 0
        raw_msg.step = int(image.shape[1] * 3)
        raw_msg.data = image.tobytes()
        return raw_msg

    def _read_compressed_size(self, msg: CompressedImage) -> Optional[tuple[int, int]]:
        try:
            array = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(array, cv2.IMREAD_UNCHANGED)
        except Exception as exc:
            self.get_logger().warn(f'camera_info synthesis decode failed: {exc}')
            return None
        if image is None or len(image.shape) < 2:
            return None
        height, width = image.shape[:2]
        return int(width), int(height)

    def _publish_synthetic_camera_info_if_needed(self, header, width: int, height: int) -> None:
        if not self._synthesize_camera_info or width <= 0 or height <= 0:
            return
        if self._last_rgb_info_time > 0.0 and time.monotonic() - self._last_rgb_info_time < 2.0:
            return
        if not self._synthetic_info_warned:
            self.get_logger().warn(
                'no upstream camera_info received; publishing approximate /camera/camera_info')
            self._synthetic_info_warned = True

        fx = fy = float(max(width, height))
        cx = (float(width) - 1.0) * 0.5
        cy = (float(height) - 1.0) * 0.5

        info = CameraInfo()
        info.header = header
        info.header.frame_id = self._camera_frame
        info.width = int(width)
        info.height = int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self._pub_rgb_info.publish(info)

    def _publish_synthetic_depth_info_if_needed(self, header, width: int, height: int) -> None:
        if (
            self._pub_depth_info is None
            or not self._synthesize_camera_info
            or width <= 0
            or height <= 0
        ):
            return
        if (
            self._last_depth_info_time > 0.0
            and time.monotonic() - self._last_depth_info_time < 2.0
        ):
            return
        if not self._synthetic_depth_info_warned:
            self.get_logger().warn(
                'no upstream depth camera_info received; publishing approximate /depth/camera_info')
            self._synthetic_depth_info_warned = True

        fx = fy = float(max(width, height))
        cx = (float(width) - 1.0) * 0.5
        cy = (float(height) - 1.0) * 0.5

        info = CameraInfo()
        info.header = header
        info.header.frame_id = self._camera_frame
        info.width = int(width)
        info.height = int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self._pub_depth_info.publish(info)

    def _on_depth(self, msg: Image) -> None:
        if self._pub_depth is None:
            return
        msg.header.frame_id = self._camera_frame
        self._publish_synthetic_depth_info_if_needed(msg.header, msg.width, msg.height)
        self._pub_depth.publish(msg)

    def _on_depth_compressed(self, msg: CompressedImage) -> None:
        if self._pub_depth is None:
            return
        msg.header.frame_id = self._camera_frame
        try:
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        except Exception as exc:
            self.get_logger().warn(f'depth compressed decode failed: {exc}')
            return
        if image is None:
            self.get_logger().warn('depth compressed decode returned None')
            return
        if image.ndim == 3:
            image = image[:, :, 0]
        if image.dtype != np.uint16:
            image = np.clip(image, 0, 65535).astype(np.uint16)
        image = np.ascontiguousarray(image)

        raw_msg = Image()
        raw_msg.header = msg.header
        raw_msg.height = int(image.shape[0])
        raw_msg.width = int(image.shape[1])
        raw_msg.encoding = '16UC1'
        raw_msg.is_bigendian = 0
        raw_msg.step = int(image.shape[1] * 2)
        raw_msg.data = image.tobytes()
        self._synthetic_depth_size = (raw_msg.width, raw_msg.height)
        self._publish_synthetic_depth_info_if_needed(raw_msg.header, raw_msg.width, raw_msg.height)
        self._pub_depth.publish(raw_msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        if self._pub_depth_info is None:
            return
        self._last_depth_info_time = time.monotonic()
        msg.header.frame_id = self._camera_frame
        self._pub_depth_info.publish(msg)

    def _on_imu(self, msg: Imu) -> None:
        if self._pub_imu is None:
            return
        msg.header.frame_id = self._imu_frame
        self._pub_imu.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        if self._pub_scan is None:
            return
        # Isaac native LiDAR and rosbridge odom can arrive with about 100 ms
        # skew. Stamp scans at the latest odom TF time so slam_toolbox can look
        # up base_link->odom instead of dropping every scan as pre-cache data.
        msg.header.stamp = self._last_odom_stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = self._scan_frame
        self._normalize_scan_metadata(msg)
        self._pub_scan.publish(msg)

    def _normalize_scan_metadata(self, msg: LaserScan) -> None:
        count = len(msg.ranges)
        if count <= 1 or msg.angle_increment == 0.0:
            return
        expected = int(round((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
        if expected == count:
            return
        if abs(expected - count) > 2:
            self.get_logger().warn(
                f'large LaserScan size mismatch: ranges={count}, expected={expected}; '
                'leaving angle metadata unchanged')
            return
        msg.angle_max = msg.angle_min + msg.angle_increment * float(count - 1)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = XLeRobotV2Bridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
