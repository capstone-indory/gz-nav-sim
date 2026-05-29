#!/usr/bin/env python3
"""ROS 2 ↔ Isaac Sim (xlerobot_v1 ZMQ) bridge — multi-robot fleet aware.

Legacy bridge for the older Isaac ZMQ sim_server. The downstream stack
(slam_toolbox, Nav2, foxglove, ros_adapter) sees the same ROS topic interface.
We bind ONE ROS stack to ONE Isaac robot in the fleet
(parameter `robot_id`, default 0).

Wire spec (indoory_isaac_sim, multi-robot):
  * SUB :5555  sim → robot   sensor PUB    multipart [topic, msgpack]
                             topics suffixed `.<robot_id>`
                             e.g. proprio.0, rgb.front.0, scan.0
  * PUSH :5556 robot → sim   action frame  msgpack {robot_id,
                                                    arm_joint_pos_target(14),
                                                    base_cmd_vel(3)}
  * REQ :5557  RPC           reset / set_pose(robot_id) / enable_stream / ...

Topics consumed (for our robot_id only):
  proprio.<id>      → /odom + odom→base_link TF + /clock
  rgb.front.<id>    → /camera/image_raw + /camera/image_raw/compressed
                       + /camera/camera_info
  depth.front.<id>  → /depth/image_raw + /depth/camera_info
  scan.<id>         → /scan
  (rgb.wrist / depth.wrist / scan.mid / tf.links — ignored, not used by SLAM/Nav)

Subscribed (from ROS):
  /cmd_vel_mux → base_cmd_vel = [vx, vy, wz],
  /xlerobot/teleop/joint_targets → arm_joint_pos_target(14),
              robot_id = our `robot_id`. `frame` field omitted
              → defaults to "body" (sim yaw-rotates per current pose,
                 which is what Nav2 controller_server already expects).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

import msgpack
import numpy as np
import rclpy
import zmq
from cv_bridge import CvBridge
import cv2

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    import zstandard as zstd
    _ZSTD_DECOMPRESS = zstd.ZstdDecompressor().decompress
except ImportError:
    _ZSTD_DECOMPRESS = None


# sim_server 가 sensor (proprio/rgb/depth/scan/tf.links) payload 에 박는 schema.
# 모든 sensor 메시지의 'schema' 필드를 이 값과 비교해 검증 — 안 맞으면 drop.
SCHEMA = 'xlerobot_v1'
# command (PUSH :5556) frame 의 schema. sim_server 가 fleet_info 응답으로
# `command_schema = 'xlerobot_v1.1'` 을 노출 (action_dim_per_robot=23, vr_mode=True).
# sensor schema 와 분리되어 있어 send 측만 'xlerobot_v1.1' 로 박아야 sim 이 수락.
# 'xlerobot_v1' 박으면 wire validation 통과 못 해서 sim 이 silently drop → 로봇 안 움직임.
COMMAND_SCHEMA = 'xlerobot_v1.1'
ARM_DOF = 14
BASE_DOF = 3


def _ns_to_time(ns: int) -> TimeMsg:
    sec, nanosec = divmod(int(ns), 1_000_000_000)
    msg = TimeMsg()
    msg.sec = int(sec)
    msg.nanosec = int(nanosec)
    return msg


class IsaacBridge(Node):
    def __init__(self) -> None:
        super().__init__('isaac_bridge')

        # ── connection ──────────────────────────────────────────────────
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('pub_port', 5555)
        self.declare_parameter('push_port', 5556)
        self.declare_parameter('rep_port', 5557)
        self.declare_parameter('cmd_rate_hz', 20.0)
        # Which robot in the Isaac fleet we drive. Default 1 (we usually run
        # robot 1 — robot 0 is reserved for other peers in the lab fleet).
        # sim_server caps at MAX_NUM_ROBOTS=16 and rejects out-of-fleet ids
        # at runtime. Use RPC `fleet_info` to confirm sim's --num-robots.
        self.declare_parameter('robot_id', 1)

        # ── frames ──────────────────────────────────────────────────────
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('lidar_frame', 'base_link')
        # tf.links.<id> payload camera link name. Dynamic camera TF is disabled
        # by default; the launch file publishes the fixed XLeRobot head chain.
        self.declare_parameter('camera_link_name', 'head_tilt')

        # ── camera intrinsics (RGB) — Isaac default profile 1280×720 ───
        # Defaults approximate depth sensor 1280×720 RealSense intrinsics
        # (HFOV ~87° → fx ~644). Override via params if Isaac config differs.
        self.declare_parameter('rgb_width', 1280)
        self.declare_parameter('rgb_height', 720)
        self.declare_parameter('rgb_fx', 644.5)
        self.declare_parameter('rgb_fy', 644.5)
        self.declare_parameter('rgb_cx', 640.0)
        self.declare_parameter('rgb_cy', 360.0)

        # depth intrinsics — Isaac default 1280×720 (same as RGB front)
        self.declare_parameter('depth_width', 1280)
        self.declare_parameter('depth_height', 720)
        self.declare_parameter('depth_fx', 644.5)
        self.declare_parameter('depth_fy', 644.5)
        self.declare_parameter('depth_cx', 640.0)
        self.declare_parameter('depth_cy', 360.0)
        # mm → m. Isaac sends uint16 mm by default (depth_scale_m=0.001).
        self.declare_parameter('depth_scale_m', 0.001)

        # ── lidar geometry (-π .. π, 12 m) ─────────────────────────────
        # scan_range_min defaults to 0.20 m: drop self-returns where the lidar
        # picks up the robot's own chassis. Anything closer is rewritten to
        # inf so slam_toolbox / nav2 obstacle_layer treat it as no-return.
        self.declare_parameter('scan_angle_min', -3.14159)
        self.declare_parameter('scan_angle_max', 3.14159)
        self.declare_parameter('scan_range_min', 0.20)
        self.declare_parameter('scan_range_max', 12.0)

        g = lambda n: self.get_parameter(n).value
        self._host = str(g('host'))
        self._pub_port = int(g('pub_port'))
        self._push_port = int(g('push_port'))
        self._rep_port = int(g('rep_port'))
        self._cmd_period = 1.0 / max(1.0, float(g('cmd_rate_hz')))
        self._robot_id = int(g('robot_id'))
        # Per-robot topic names — match against incoming SUB frames.
        self._t_proprio = f'proprio.{self._robot_id}'
        self._t_rgb = f'rgb.front.{self._robot_id}'
        self._t_depth = f'depth.front.{self._robot_id}'
        self._t_scan = f'scan.{self._robot_id}'
        self._t_tflinks = f'tf.links.{self._robot_id}'
        self._camera_link_name = str(g('camera_link_name'))

        self._odom_frame = str(g('odom_frame'))
        self._base_frame = str(g('base_frame'))
        self._cam_frame = str(g('camera_optical_frame'))
        self._lidar_frame = str(g('lidar_frame'))

        self._scan_angle_min = float(g('scan_angle_min'))
        self._scan_angle_max = float(g('scan_angle_max'))
        self._scan_range_min = float(g('scan_range_min'))
        self._scan_range_max = float(g('scan_range_max'))
        self._depth_scale = float(g('depth_scale_m'))

        # Pre-build CameraInfo templates — only stamp/frame change per frame.
        self._rgb_info_tmpl = self._build_camera_info(
            int(g('rgb_width')), int(g('rgb_height')),
            float(g('rgb_fx')), float(g('rgb_fy')),
            float(g('rgb_cx')), float(g('rgb_cy')),
        )
        self._depth_info_tmpl = self._build_camera_info(
            int(g('depth_width')), int(g('depth_height')),
            float(g('depth_fx')), float(g('depth_fy')),
            float(g('depth_cx')), float(g('depth_cy')),
        )

        # ── ROS pubs / subs ─────────────────────────────────────────────
        # /odom + /scan are RELIABLE because slam_toolbox, nav2 costmaps, and
        # telemetry adapters expect reliable. Camera/depth stay
        # BEST_EFFORT for throughput.
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._pub_clock = self.create_publisher(Clock, '/clock', 10)
        self._pub_odom = self.create_publisher(Odometry, '/odom', rel_qos)
        self._pub_scan = self.create_publisher(LaserScan, '/scan', rel_qos)
        self._pub_rgb = self.create_publisher(Image, '/camera/image_raw', sensor_qos)
        self._pub_rgb_compressed = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', sensor_qos)
        self._pub_rgb_info = self.create_publisher(CameraInfo, '/camera/camera_info', rel_qos)
        self._pub_depth = self.create_publisher(Image, '/depth/image_raw', sensor_qos)
        self._pub_depth_info = self.create_publisher(CameraInfo, '/depth/camera_info', rel_qos)

        self._tf_bcast = TransformBroadcaster(self)
        self._cv_bridge = CvBridge()

        self._cmd_lock = threading.Lock()
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._arm_lock = threading.Lock()
        self._arm_target = [0.0] * ARM_DOF
        # twist_mux 가 /cmd_vel_teleop (수동) 와 /cmd_vel (Nav2) 둘을 우선순위로
        # 묶어 /cmd_vel_mux 로 forward. isaac_bridge 는 그것만 sub — SLAM 못
        # 잡혀 Nav2 가 cmd_vel=0 도배해도 teleop pri 100 으로 통과.
        self.create_subscription(Twist, '/cmd_vel_mux', self._on_cmd_vel, 10)
        self.create_subscription(
            Float64MultiArray,
            '/xlerobot/teleop/joint_targets',
            self._on_joint_targets,
            10,
        )

        # ── ZMQ ──────────────────────────────────────────────────────────
        self._zmq_ctx = zmq.Context.instance()
        self._sub = self._zmq_ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RCVHWM, 8)
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.connect(f'tcp://{self._host}:{self._pub_port}')
        # Subscribe selectively to *our* robot's 4 topics. ZMQ does prefix
        # matching, but the 4 strings are exact-name and don't share prefixes
        # with other-robot or wrist/mid/tf topics, so the kernel filters out
        # the 1280×720 jpeg traffic for the other N-1 robots automatically.
        for t in (self._t_proprio, self._t_rgb, self._t_depth, self._t_scan,
                  self._t_tflinks):
            self._sub.setsockopt(zmq.SUBSCRIBE, t.encode('ascii'))

        self._push = self._zmq_ctx.socket(zmq.PUSH)
        self._push.setsockopt(zmq.SNDHWM, 4)
        self._push.setsockopt(zmq.LINGER, 0)
        self._push.connect(f'tcp://{self._host}:{self._push_port}')

        self.get_logger().info(
            f'isaac_bridge[robot_id={self._robot_id}] connected: '
            f'SUB tcp://{self._host}:{self._pub_port} '
            f'PUSH tcp://{self._host}:{self._push_port}  '
            f'topics=[{self._t_proprio}, {self._t_rgb}, {self._t_depth}, {self._t_scan}]')
        if _ZSTD_DECOMPRESS is None:
            self.get_logger().warn(
                "zstandard not installed — depth.front frames will be skipped. "
                "Install via `pip3 install zstandard`.")

        # SUB thread runs blocking poll in background; ROS pubs are thread-safe.
        self._stop = threading.Event()
        self._seen_topics: set[str] = set()
        self._sub_thread = threading.Thread(target=self._sub_loop, daemon=True)
        self._sub_thread.start()

        # PUSH on a timer so base and joint targets stay fresh.
        self.create_timer(self._cmd_period, self._tick_push)

    # ── ZMQ → ROS ────────────────────────────────────────────────────────
    def _sub_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        while not self._stop.is_set():
            try:
                socks = dict(poller.poll(timeout=200))
            except zmq.ContextTerminated:
                return
            if self._sub not in socks:
                continue
            try:
                topic_b, payload = self._sub.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                continue
            except zmq.ZMQError as exc:
                self.get_logger().warn(f'zmq recv error: {exc}')
                continue

            try:
                msg = msgpack.unpackb(payload, raw=False)
            except Exception as exc:  # malformed
                self.get_logger().warn(f'msgpack decode failed: {exc}')
                continue
            if not isinstance(msg, dict):
                continue
            # Schema only enforced when the payload explicitly carries one.
            # In practice sim_server omits it on sensor topics (rgb/depth/scan)
            # and only adds it to proprio + RPC. Dropping unconditionally here
            # silently swallowed every camera/lidar frame.
            schema = msg.get('schema')
            if schema is not None and schema != SCHEMA:
                continue

            topic = topic_b.decode('ascii', errors='replace')
            if topic not in self._seen_topics:
                self._seen_topics.add(topic)
                keys = sorted(k for k in msg.keys() if k != 'data')
                self.get_logger().info(
                    f'first frame on {topic!r}: keys={keys}')
            try:
                self._dispatch(topic, msg)
            except Exception as exc:
                self.get_logger().error(
                    f'dispatch {topic}: {type(exc).__name__}: {exc}')

    def _dispatch(self, topic: str, msg: dict) -> None:
        if topic == self._t_proprio:
            self._handle_proprio(msg)
        elif topic == self._t_rgb:
            self._handle_rgb(msg)
        elif topic == self._t_depth:
            self._handle_depth(msg)
        elif topic == self._t_scan:
            # Main LiDAR scan for SLAM and Nav2 costmaps.
            self._handle_scan(msg)
        # tf.links.<id> dynamic TF is disabled. The launch file now publishes
        # the fixed XLeRobot head chain through camera_frame/camera_optical_frame.
        # elif topic == self._t_tflinks:
        #     self._handle_tf_links(msg)
        # scan.mid.<id> intentionally dropped: feeding two scans into the
        # same /scan topic alternates frame_ids and saturates slam_toolbox's
        # tf2 message filter. rgb.wrist.<id>, depth.wrist.<id>, tf.links.<id>
        # also ignored — not used by SLAM/Nav.

    def _handle_proprio(self, msg: dict) -> None:
        stamp_ns = int(msg.get('stamp_ns', 0))
        stamp = _ns_to_time(stamp_ns)

        # /clock — wire the rest of the stack to sim time.
        clock = Clock()
        clock.clock = stamp
        self._pub_clock.publish(clock)

        pose = msg.get('base_pose')
        if not pose or len(pose) < 7:
            return
        x, y, z, qx, qy, qz, qw = (float(v) for v in pose[:7])

        # base_forward_w 는 base_link 의 +X 축을 world 좌표계로 박은 unit vector
        # (가이드 §3.4). 2D 평면 robot (Reg/Force3DoF=true) 이라 z 회전만 필요 →
        # base_forward_w 의 (fx, fy) 로 yaw 계산이 quaternion 해석/좌표계 오차에
        # 완전 면역. 90도 오프셋 디버깅 시점에 base_pose quaternion 만 쓰면 sim
        # 의 robot URDF forward 정의가 ROS 표준 (+X 정면) 과 다를 때 회전된 odom
        # 으로 발행 → 화살표 90도 어긋남 + cmd_vel body frame 적용 시 직진하면
        # 옆으로 움직이는 증상. base_forward_w 우선이 가장 안전.
        import math as _math
        fw = msg.get('base_forward_w')
        if fw and len(fw) >= 2:
            yaw = _math.atan2(float(fw[1]), float(fw[0]))
            qx, qy = 0.0, 0.0
            qz = _math.sin(yaw * 0.5)
            qw = _math.cos(yaw * 0.5)

        twist = msg.get('base_twist') or [0.0] * 6
        vx, vy, vz, wx, wy, wz = (float(v) for v in (list(twist) + [0.0] * 6)[:6])

        # /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
        odom.twist.twist.angular = Vector3(x=wx, y=wy, z=wz)
        self._pub_odom.publish(odom)

        # odom → base_link TF
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self._tf_bcast.sendTransform(tf)

    def _handle_rgb(self, msg: dict) -> None:
        data = msg.get('data')
        if not data:
            return
        encoding = msg.get('encoding', 'jpeg')
        if encoding != 'jpeg':
            self.get_logger().warn(f'rgb.front: unsupported encoding {encoding!r}')
            return
        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))

        # JPEG → numpy → Image (rgb8). Also republish the raw JPEG as
        # CompressedImage for foxglove/bandwidth-sensitive consumers.
        buf = np.frombuffer(bytes(data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn('rgb.front: jpeg decode failed')
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        img = self._cv_bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        img.header.stamp = stamp
        img.header.frame_id = self._cam_frame
        self._pub_rgb.publish(img)

        comp = CompressedImage()
        comp.header.stamp = stamp
        comp.header.frame_id = self._cam_frame
        comp.format = 'jpeg'
        comp.data = bytes(data)
        self._pub_rgb_compressed.publish(comp)

        self._pub_rgb_info.publish(
            self._stamp_camera_info(self._rgb_info_tmpl, stamp, self._cam_frame))

    def _handle_depth(self, msg: dict) -> None:
        if _ZSTD_DECOMPRESS is None:
            return
        raw = msg.get('data')
        if not raw:
            return
        encoding = msg.get('encoding', 'u16_zstd')
        if encoding != 'u16_zstd':
            self.get_logger().warn(f'depth.front: unsupported encoding {encoding!r}')
            return
        width = int(msg.get('width', 0))
        height = int(msg.get('height', 0))
        if width <= 0 or height <= 0:
            return
        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))
        scale_m = float(msg.get('depth_scale_m', self._depth_scale))

        try:
            decompressed = _ZSTD_DECOMPRESS(bytes(raw))
        except Exception as exc:
            self.get_logger().warn(f'depth.front zstd decode failed: {exc}')
            return

        arr = np.frombuffer(decompressed, dtype=np.uint16)
        if arr.size != width * height:
            self.get_logger().warn(
                f'depth.front size mismatch: got {arr.size}, '
                f'expected {width * height} ({width}x{height})')
            return
        depth = arr.reshape(height, width)

        # Publish as 16UC1 millimetres — what RTAB-Map / nvblox expect by
        # default. If sim sends a non-mm scale, callers can convert via the
        # `depth_scale_m` param downstream (most consumers assume mm).
        if abs(scale_m - 0.001) > 1e-9:
            # Re-express as mm uint16 (clip to range to avoid overflow).
            depth = np.clip(
                depth.astype(np.float32) * scale_m * 1000.0,
                0.0, 65535.0).astype(np.uint16)

        img = self._cv_bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        img.header.stamp = stamp
        img.header.frame_id = self._cam_frame
        self._pub_depth.publish(img)

        info = self._stamp_camera_info(self._depth_info_tmpl, stamp, self._cam_frame)
        self._pub_depth_info.publish(info)

    def _handle_scan(self, msg: dict) -> None:
        ranges_raw = msg.get('ranges')
        if ranges_raw is None:
            return
        # sim_server packs ranges as msgpack bin (raw float32 bytes), not as
        # a Python list. Iterating bytes directly yields 0-255 ints — which
        # gets silently published as garbage. Detect both shapes.
        if isinstance(ranges_raw, (bytes, bytearray, memoryview)):
            ranges_arr = np.frombuffer(bytes(ranges_raw), dtype=np.float32)
        else:
            ranges_arr = np.asarray(list(ranges_raw), dtype=np.float32)
        n = int(ranges_arr.size)
        if n == 0:
            return

        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))
        scan = LaserScan()
        scan.header.stamp = stamp
        # Force our own lidar_frame ("base_link" by default). sim_server tags
        # scans with internal frames like "mid_scan" / "lidar_link" that have
        # no static TF in our launch — slam_toolbox's tf2 message filter then
        # drops every scan with "queue is full" and SLAM never advances.
        scan.header.frame_id = self._lidar_frame
        # Allow per-message overrides; otherwise fall back to the params.
        amin = float(msg.get('angle_min', self._scan_angle_min))
        amax = float(msg.get('angle_max', self._scan_angle_max))
        scan.angle_min = amin
        scan.angle_max = amax
        scan.angle_increment = (amax - amin) / n
        scan.time_increment = 0.0
        scan.scan_time = float(msg.get('scan_time', 0.1))
        # Force our self-filter range_min instead of sim's (sim sends 0.05).
        # Anything closer than range_min is rewritten to inf so consumers
        # that look at raw ranges (not just header.range_min) drop it too.
        scan.range_min = self._scan_range_min
        scan.range_max = float(msg.get('range_max', self._scan_range_max))
        if self._scan_range_min > 0.0:
            ranges_arr = np.where(
                ranges_arr < self._scan_range_min,
                np.float32(np.inf), ranges_arr)
        scan.ranges = ranges_arr.tolist()
        self._pub_scan.publish(scan)

    def _handle_tf_links(self, msg: dict) -> None:
        """Optional legacy dynamic camera TF handler for ZMQ payloads.

        The active XLeRobot path uses the fixed head chain from the launch file,
        so this callback is intentionally not subscribed in normal runs.
        """
        if not hasattr(self, '_tflinks_dbg_done'):
            names = [t.get('name','?') for t in msg.get('targets',[]) or []]
            self.get_logger().info(
                f'[DBG] _handle_tf_links 첫 호출. camera_link_name={self._camera_link_name!r} '
                f'targets={names}')
            self._tflinks_dbg_done = True
        targets = msg.get('targets')
        if not targets:
            return
        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))
        for t in targets:
            if t.get('name') != self._camera_link_name:
                continue
            pose = t.get('pose')
            if not pose or len(pose) < 7:
                return
            x, y, z, qx, qy, qz, qw = (float(v) for v in pose[:7])
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._base_frame
            tf.child_frame_id = 'camera_frame'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self._tf_bcast.sendTransform(tf)
            return

    # ── ROS → ZMQ ────────────────────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._cmd_vx = float(msg.linear.x)
            self._cmd_vy = float(msg.linear.y)
            self._cmd_wz = float(msg.angular.z)

    def _on_joint_targets(self, msg: Float64MultiArray) -> None:
        with self._arm_lock:
            for index, raw_value in enumerate(list(msg.data)[:ARM_DOF]):
                value = float(raw_value)
                if np.isfinite(value):
                    self._arm_target[index] = value

    def _tick_push(self) -> None:
        # publisher-count 기반 deadman: /cmd_vel_mux 에 publisher 가 0 이면 마지막
        # cached cmd 무시하고 (0,0,0) 송출. ROS graph 에 publisher 가 살아있으면
        # 신뢰 (Nav2 controller, adapter teleop, behavior_server 모두 자체 watchdog
        # 가짐). publisher 가 사라진 경우 = 누가 죽거나 SIGKILL 됨 = 안전하게 정지.
        # timing 의존 watchdog 보다 시맨틱하게 정확하고 false-positive 가 적음.
        with self._cmd_lock:
            if self.count_publishers('/cmd_vel_mux') == 0:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_wz = 0.0
            base = [self._cmd_vx, self._cmd_vy, self._cmd_wz]
        with self._arm_lock:
            arm = list(self._arm_target)
        # No `frame` key → defaults to "body" on sim side (yaw-rotated).
        # That matches Nav2 controller_server / teleop conventions where
        # cmd_vel is already in the robot's body frame.
        frame = {
            'schema': COMMAND_SCHEMA,
            'stamp_ns': time.time_ns(),
            'robot_id': self._robot_id,
            'arm_joint_pos_target': arm,
            'base_cmd_vel': base,
        }
        try:
            self._push.send(msgpack.packb(frame, use_bin_type=True), zmq.NOBLOCK)
        except zmq.Again:
            # sim PULL backed up — drop, will resend next tick.
            pass
        except zmq.ZMQError as exc:
            self.get_logger().warn(f'push send error: {exc}')

    # ── helpers ──────────────────────────────────────────────────────────
    @staticmethod
    def _build_camera_info(
            width: int, height: int, fx: float, fy: float,
            cx: float, cy: float) -> CameraInfo:
        info = CameraInfo()
        info.width = int(width)
        info.height = int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    @staticmethod
    def _stamp_camera_info(tmpl: CameraInfo, stamp: TimeMsg, frame: str) -> CameraInfo:
        info = CameraInfo()
        info.width = tmpl.width
        info.height = tmpl.height
        info.distortion_model = tmpl.distortion_model
        info.d = list(tmpl.d)
        info.k = list(tmpl.k)
        info.r = list(tmpl.r)
        info.p = list(tmpl.p)
        info.header.stamp = stamp
        info.header.frame_id = frame
        return info

    def destroy_node(self) -> None:
        self._stop.set()
        try:
            self._sub.close(linger=0)
        except Exception:
            pass
        try:
            self._push.close(linger=0)
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = IsaacBridge()
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
