#!/usr/bin/env python3
"""ROS topic bridge for the XLeRobot Hospital Isaac Sim v2 app.

The v2 Isaac app connects to rosbridge_server itself and publishes native ROS 2
topics under /xlerobot. This node keeps the rest of gz_nav_sim unchanged by
aliasing that interface to the legacy topics used by Nav2/RTAB-Map:

  /xlerobot/cmd_vel                         <- /cmd_vel_mux
  /xlerobot/odom                            -> /odom
  /xlerobot/head/d456/color/image_raw       -> /camera/image_raw
  /xlerobot/head/d456/color/camera_info     -> /camera/camera_info
  /xlerobot/head/d456/depth/image_rect_raw  -> /d456/depth/image_raw
  /xlerobot/head/d456/depth/camera_info     -> /d456/depth/camera_info
  /xlerobot/scan                            -> /scan

Frame ids are normalized to the local stack's short names (odom/base_link/
camera_optical_frame) so existing Nav2 and semantic nodes do not need a second
set of parameters for xlerobot_* frames.
"""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from tf2_ros import TransformBroadcaster


class XLeRobotV2Bridge(Node):
    def __init__(self) -> None:
        super().__init__('xlerobot_v2_bridge')

        self.declare_parameter('namespace', '/xlerobot')
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('scan_frame', 'base_link')

        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_mux')
        self.declare_parameter('cmd_vel_out_topic', '/xlerobot/cmd_vel')
        self.declare_parameter('odom_in_topic', '/xlerobot/odom')
        self.declare_parameter('odom_out_topic', '/odom')
        self.declare_parameter('rgb_image_in_topic', '/xlerobot/head/d456/color/image_raw')
        self.declare_parameter('rgb_compressed_image_in_topic', '/xlerobot/head/d456/color/image')
        self.declare_parameter('rgb_info_in_topic', '/xlerobot/head/d456/color/camera_info')
        self.declare_parameter('rgb_image_out_topic', '/camera/image_raw')
        self.declare_parameter('rgb_compressed_out_topic', '/camera/image_raw/compressed')
        self.declare_parameter('rgb_info_out_topic', '/camera/camera_info')
        self.declare_parameter('depth_image_in_topic', '/xlerobot/head/d456/depth/image_rect_raw')
        self.declare_parameter('depth_compressed_image_in_topic', '/xlerobot/head/d456/depth/image')
        self.declare_parameter('depth_info_in_topic', '/xlerobot/head/d456/depth/camera_info')
        self.declare_parameter('depth_image_out_topic', '/d456/depth/image_raw')
        self.declare_parameter('depth_info_out_topic', '/d456/depth/camera_info')
        self.declare_parameter('scan_in_topic', '/xlerobot/scan')
        self.declare_parameter('scan_out_topic', '/scan')

        g = lambda name: self.get_parameter(name).value
        self._odom_frame = str(g('odom_frame'))
        self._base_frame = str(g('base_frame'))
        self._camera_frame = str(g('camera_optical_frame'))
        self._scan_frame = str(g('scan_frame'))

        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._pub_cmd = self.create_publisher(Twist, str(g('cmd_vel_out_topic')), rel_qos)
        self._pub_odom = self.create_publisher(Odometry, str(g('odom_out_topic')), rel_qos)
        self._pub_scan = self.create_publisher(LaserScan, str(g('scan_out_topic')), rel_qos)
        self._pub_rgb = self.create_publisher(Image, str(g('rgb_image_out_topic')), sensor_qos)
        self._pub_rgb_compressed = self.create_publisher(
            CompressedImage, str(g('rgb_compressed_out_topic')), sensor_qos)
        self._pub_rgb_info = self.create_publisher(CameraInfo, str(g('rgb_info_out_topic')), rel_qos)
        self._pub_depth = self.create_publisher(Image, str(g('depth_image_out_topic')), sensor_qos)
        self._pub_depth_info = self.create_publisher(CameraInfo, str(g('depth_info_out_topic')), rel_qos)

        self._tf_bcast = TransformBroadcaster(self)
        self._cv_bridge = CvBridge()
        self._cmd_lock = threading.Lock()
        self._last_cmd = Twist()
        self._cmd_in_topic = str(g('cmd_vel_in_topic'))

        self.create_subscription(Twist, self._cmd_in_topic, self._on_cmd_vel, 10)
        self.create_subscription(Odometry, str(g('odom_in_topic')), self._on_odom, sensor_qos)
        self.create_subscription(Image, str(g('rgb_image_in_topic')), self._on_rgb, sensor_qos)
        self.create_subscription(CompressedImage, str(g('rgb_compressed_image_in_topic')), self._on_rgb_compressed, sensor_qos)
        self.create_subscription(CameraInfo, str(g('rgb_info_in_topic')), self._on_rgb_info, sensor_qos)
        self.create_subscription(Image, str(g('depth_image_in_topic')), self._on_depth, sensor_qos)
        self.create_subscription(CompressedImage, str(g('depth_compressed_image_in_topic')), self._on_depth_compressed, sensor_qos)
        self.create_subscription(CameraInfo, str(g('depth_info_in_topic')), self._on_depth_info, sensor_qos)
        self.create_subscription(LaserScan, str(g('scan_in_topic')), self._on_scan, sensor_qos)

        cmd_period = 1.0 / max(1.0, float(g('cmd_rate_hz')))
        self.create_timer(cmd_period, self._tick_cmd)

        self.get_logger().info(
            'xlerobot_v2_bridge active: /xlerobot ROS topics -> /odom /scan /camera/*, '
            f'{self._cmd_in_topic} -> {g("cmd_vel_out_topic")}')

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._last_cmd = msg

    def _tick_cmd(self) -> None:
        with self._cmd_lock:
            msg = self._last_cmd
            if self.count_publishers(self._cmd_in_topic) == 0:
                msg = Twist()
        self._pub_cmd.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
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
        msg.header.frame_id = self._camera_frame
        self._pub_rgb_info.publish(msg)

    def _on_rgb_compressed(self, msg: CompressedImage) -> None:
        msg.header.frame_id = self._camera_frame
        self._pub_rgb_compressed.publish(msg)
        try:
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        except Exception as exc:
            self.get_logger().warn(f'rgb compressed decode failed: {exc}')
            return
        if image is None:
            self.get_logger().warn('rgb compressed decode returned None')
            return
        raw_msg = Image()
        raw_msg.header = msg.header
        raw_msg.height = int(image.shape[0])
        raw_msg.width = int(image.shape[1])
        raw_msg.encoding = 'bgr8'
        raw_msg.is_bigendian = 0
        raw_msg.step = int(image.shape[1] * 3)
        raw_msg.data = image.tobytes()
        self._pub_rgb.publish(raw_msg)

    def _on_depth(self, msg: Image) -> None:
        msg.header.frame_id = self._camera_frame
        self._pub_depth.publish(msg)

    def _on_depth_compressed(self, msg: CompressedImage) -> None:
        msg.header.frame_id = self._camera_frame
        fmt = msg.format.lower()
        try:
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        except Exception as exc:
            self.get_logger().warn(f'depth compressed decode failed: {exc}')
            return
        if image is None:
            self.get_logger().warn('depth compressed decode returned None')
            return
        if image.ndim == 3 and image.shape[2] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image.dtype != np.uint16:
            try:
                image = image.astype(np.uint16)
            except Exception as exc:
                self.get_logger().warn(f'depth compressed cast failed: {exc}')
                return
        raw_msg = Image()
        raw_msg.header = msg.header
        raw_msg.height = int(image.shape[0])
        raw_msg.width = int(image.shape[1])
        raw_msg.encoding = '16UC1'
        raw_msg.is_bigendian = 0
        raw_msg.step = int(image.shape[1] * 2)
        raw_msg.data = image.tobytes()
        self._pub_depth.publish(raw_msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self._camera_frame
        self._pub_depth_info.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = self._scan_frame
        self._pub_scan.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = XLeRobotV2Bridge()
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
