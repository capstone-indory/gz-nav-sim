#!/usr/bin/env python3
"""ROS 2 ↔ VGGT-SLAM bridge (runs in the Humble Python 3.10 env).

VGGT-SLAM needs gtsam-develop (Python ≥ 3.11 only), so we run the heavy
solver in a sibling Python 3.11 process and exchange messages over ZeroMQ.

This node:
  • subscribes to compressed RGB frames,
  • forwards every frame as JPEG bytes to the server (PUSH),
  • listens on a SUB socket for SLAM updates,
  • republishes them as PoseStamped / Path / PointCloud2.

The server binary can also be launched from here — set
`launch_server:=true` and point `server_python` / `server_script` at the
Python 3.11 interpreter and script.
"""

from __future__ import annotations

import os
import subprocess
import threading
from dataclasses import dataclass

import msgpack
import numpy as np
import rclpy
import zmq
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import (CameraInfo, CompressedImage, LaserScan,
                             PointCloud2, PointField)
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class ServerHandle:
    proc: subprocess.Popen | None


class VGGTSlamBridge(Node):
    def __init__(self) -> None:
        super().__init__('vggt_slam_bridge')

        p = self.declare_parameter
        p('image_topic', '/camera/image_raw/compressed')
        p('pose_topic', '/vggt_slam/pose')
        p('trajectory_topic', '/vggt_slam/trajectory')
        p('pointcloud_topic', '/vggt_slam/pointcloud')
        p('world_frame', 'map')
        p('pull_port', 5555)
        p('pub_port', 5556)
        p('server_host', '127.0.0.1')
        p('launch_server', True)
        p('server_python', '/root/gz-nav-sim/venv_vggt/bin/python')
        p('server_script',
          '/root/gz-nav-sim/src/gz_nav_sim/scripts/vggt_slam_server.py')
        p('server_repo', '/root/gz-nav-sim/src/VGGT-SLAM')
        p('submap_size', 8)
        p('min_disparity', 50.0)
        p('pointcloud_stride', 8)

        # LiDAR scale anchor 관련
        p('scan_topic', '/scan')
        p('camera_info_topic', '/camera/camera_info')
        p('camera_frame', 'camera_optical_frame')
        p('lidar_anchor_enabled', True)
        p('lidar_anchor_sigma', 0.05)
        p('lidar_anchor_min_points', 20)

        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        host = g('server_host')
        push_ep = f'tcp://{host}:{int(g("pull_port"))}'
        sub_ep = f'tcp://{host}:{int(g("pub_port"))}'

        self._server = ServerHandle(None)
        if bool(g('launch_server')):
            self._start_server(
                python_bin=g('server_python'),
                script=g('server_script'),
                repo=g('server_repo'),
                pull_port=int(g('pull_port')),
                pub_port=int(g('pub_port')),
                submap_size=int(g('submap_size')),
                min_disparity=float(g('min_disparity')),
                pointcloud_stride=int(g('pointcloud_stride')),
            )

        ctx = zmq.Context.instance()
        self._push = ctx.socket(zmq.PUSH)
        self._push.setsockopt(zmq.SNDHWM, 4)
        self._push.connect(push_ep)
        self._sub = ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, b'')
        self._sub.connect(sub_ep)
        self.get_logger().info(f'zmq PUSH → {push_ep}')
        self.get_logger().info(f'zmq SUB  ← {sub_ep}')

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CompressedImage, g('image_topic'),
            self._on_image, sensor_qos,
        )

        # LiDAR scale anchor 입력
        self._camera_frame = str(g('camera_frame'))
        self._lidar_anchor_enabled = bool(g('lidar_anchor_enabled'))
        self._lidar_anchor_sigma = float(g('lidar_anchor_sigma'))
        self._lidar_anchor_min_points = int(g('lidar_anchor_min_points'))
        self._latest_scan: LaserScan | None = None
        self._latest_camera_info: CameraInfo | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_subscription(
            LaserScan, g('scan_topic'),
            self._on_scan, sensor_qos,
        )
        self.create_subscription(
            CameraInfo, g('camera_info_topic'),
            self._on_camera_info, sensor_qos,
        )

        self._pub_pose = self.create_publisher(PoseStamped, g('pose_topic'), 10)
        self._pub_path = self.create_publisher(Path, g('trajectory_topic'), 10)
        self._pub_cloud = self.create_publisher(PointCloud2, g('pointcloud_topic'), 5)

        # 글로벌 누적 publishers (TRANSIENT_LOCAL = 새 구독자도 마지막 메시지 수신)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_global_cloud = self.create_publisher(
            PointCloud2, '/vggt_slam/global_pointcloud', latched_qos)
        self._pub_global_path = self.create_publisher(
            Path, '/vggt_slam/global_trajectory', latched_qos)

        # submap_id → (N,3) world-frame points + RGB(uint8 N,3)
        self._submap_pts: dict[int, np.ndarray] = {}
        self._submap_colors: dict[int, np.ndarray] = {}
        # submap_id → list of pose dicts (server에서 온 형식)
        self._submap_frame_poses: dict[int, list[dict]] = {}

        self._frame_id = 0
        self._stop = False
        self._sub_thread = threading.Thread(target=self._sub_loop, daemon=True)
        self._sub_thread.start()
        self.get_logger().info('vggt_slam_bridge ready')

    # ── server lifecycle ──────────────────────────────────────────────
    def _start_server(self, python_bin, script, repo, pull_port, pub_port,
                      submap_size, min_disparity, pointcloud_stride):
        if not os.path.exists(python_bin):
            self.get_logger().error(f'server_python not found: {python_bin}')
            return
        if not os.path.exists(script):
            self.get_logger().error(f'server_script not found: {script}')
            return
        cmd = [
            python_bin, script,
            '--repo', repo,
            '--pull-port', str(pull_port),
            '--pub-port', str(pub_port),
            '--submap-size', str(submap_size),
            '--min-disparity', str(min_disparity),
            '--pointcloud-stride', str(pointcloud_stride),
        ]
        self.get_logger().info('spawning VGGT-SLAM server: ' + ' '.join(cmd))
        self._server.proc = subprocess.Popen(cmd)

    # ── subscribers ───────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _on_image(self, msg: CompressedImage) -> None:
        payload = {
            'type': 'frame',
            'frame_id': self._frame_id,
            'stamp_ns': int(msg.header.stamp.sec) * 1_000_000_000
                        + int(msg.header.stamp.nanosec),
            'format': msg.format,
            'jpeg': bytes(msg.data),
        }
        # LiDAR anchor 패키징 (best-effort, 실패해도 frame은 보냄)
        if self._lidar_anchor_enabled:
            anchor = self._build_lidar_anchor(msg.header.stamp, msg.header.frame_id)
            if anchor is not None:
                payload['lidar_anchor'] = anchor

        try:
            self._push.send(msgpack.packb(payload, use_bin_type=True),
                            flags=zmq.NOBLOCK)
        except zmq.Again:
            pass  # server slow; drop frame to avoid backpressure
        self._frame_id += 1

    def _build_lidar_anchor(self, stamp, image_frame_id: str) -> dict | None:
        """첫 submap의 SIM(3) 글로벌 변환에 필요한 데이터 한 묶음.

        - scan + K + T_camera_lidar: 카메라 좌표계로 lidar 점 투영 → scale 측정
        - T_world_camera: world(map/odom)에서 본 첫 카메라 pose → R, t 추출
        """
        if self._latest_scan is None or self._latest_camera_info is None:
            return None
        scan = self._latest_scan
        cam_info = self._latest_camera_info
        cam_frame = image_frame_id or self._camera_frame

        # T_camera_lidar: camera_optical_frame ← lidar frame
        try:
            tf_cl = self._tf_buffer.lookup_transform(
                cam_frame, scan.header.frame_id,
                Time.from_msg(stamp), timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        # T_world_camera: world frame ← camera_optical_frame
        # SIM(3) 변환의 R + t. 첫 frame에서만 쓰지만 매번 lookup은 낮은 비용.
        try:
            tf_wc = self._tf_buffer.lookup_transform(
                self.world_frame, cam_frame,
                Time.from_msg(stamp), timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        T_camera_lidar = self._tf_to_matrix(tf_cl)
        T_world_camera = self._tf_to_matrix(tf_wc)
        return {
            'scan': {
                'angle_min': float(scan.angle_min),
                'angle_increment': float(scan.angle_increment),
                'range_min': float(scan.range_min),
                'range_max': float(scan.range_max),
                'ranges': np.asarray(scan.ranges, dtype=np.float32).tobytes(),
                'ranges_count': int(len(scan.ranges)),
            },
            'K': np.asarray(cam_info.k, dtype=np.float64).tobytes(),
            'image_width': int(cam_info.width),
            'image_height': int(cam_info.height),
            'T_camera_lidar': T_camera_lidar.astype(np.float64).tobytes(),
            'T_world_camera': T_world_camera.astype(np.float64).tobytes(),
            'min_points': int(self._lidar_anchor_min_points),
        }

    def _tf_to_matrix(self, tf) -> np.ndarray:
        """geometry_msgs/TransformStamped → 4x4 ndarray."""
        q = tf.transform.rotation
        t = tf.transform.translation
        # quaternion → rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        R = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = (t.x, t.y, t.z)
        return T

    # ── subscriber loop ───────────────────────────────────────────────
    def _sub_loop(self) -> None:
        while not self._stop:
            try:
                raw = self._sub.recv()
            except zmq.ContextTerminated:
                break
            except Exception as e:
                self.get_logger().warn(f'sub recv error: {e}')
                continue
            try:
                msg = msgpack.unpackb(raw, raw=False)
            except Exception:
                continue
            kind = msg.get('type')
            if kind == 'submap':
                self._publish_submap(msg)
            elif kind == 'trajectory':
                self._publish_trajectory(msg)

    def _make_header(self, stamp_ns: int) -> Header:
        h = Header()
        h.frame_id = self.world_frame
        h.stamp.sec = stamp_ns // 1_000_000_000
        h.stamp.nanosec = stamp_ns % 1_000_000_000
        return h

    def _pose_stamped(self, pose: dict, stamp_ns: int) -> PoseStamped:
        p = PoseStamped()
        p.header = self._make_header(stamp_ns)
        p.pose.position.x = float(pose['tx'])
        p.pose.position.y = float(pose['ty'])
        p.pose.position.z = float(pose['tz'])
        p.pose.orientation.x = float(pose['qx'])
        p.pose.orientation.y = float(pose['qy'])
        p.pose.orientation.z = float(pose['qz'])
        p.pose.orientation.w = float(pose['qw'])
        return p

    def _publish_submap(self, msg: dict) -> None:
        stamp_ns = int(msg.get('stamp_ns', 0))
        submap_id = int(msg.get('submap_id', -1))

        self._pub_pose.publish(self._pose_stamped(msg['pose'], stamp_ns))

        # 현재 submap만 보여주는 기존 토픽 (디버그용 유지)
        pts = None
        colors = None
        if 'pointcloud' in msg and msg.get('pointcloud_count', 0) > 0:
            pts = np.frombuffer(msg['pointcloud'], dtype=np.float32).reshape(-1, 3)
            cloud = self._build_pointcloud2(pts, stamp_ns)
            self._pub_cloud.publish(cloud)
            if 'colors' in msg and msg.get('colors_count', 0) > 0:
                colors = np.frombuffer(
                    msg['colors'], dtype=np.uint8).reshape(-1, 3)

        # 누적 dict 갱신 + 글로벌 토픽 재발행
        if submap_id >= 0 and pts is not None:
            self._submap_pts[submap_id] = pts
            if colors is not None:
                n = min(len(colors), len(pts))
                self._submap_colors[submap_id] = colors[:n]
            self._publish_global_cloud(stamp_ns)

        # frame_poses (이번 submap)도 누적 dict에 저장 → trajectory msg가 갱신
        if submap_id >= 0 and 'frame_poses' in msg:
            self._submap_frame_poses[submap_id] = list(msg['frame_poses'])

    def _publish_trajectory(self, msg: dict) -> None:
        """server의 trajectory broadcast — 모든 submap의 모든 frame pose를 갱신."""
        stamp_ns = int(msg.get('stamp_ns', 0))

        # 신규 포맷: 'submaps' = [{'submap_id', 'poses': [...]}]
        if 'submaps' in msg:
            for entry in msg['submaps']:
                sid = int(entry.get('submap_id', -1))
                if sid >= 0:
                    self._submap_frame_poses[sid] = list(entry.get('poses', []))
        else:
            # 구 포맷 backward compat: 'poses' = submap별 마지막 pose
            poses = msg.get('poses', [])
            for i, pose in enumerate(poses):
                self._submap_frame_poses.setdefault(i, [pose])

        self._publish_global_path(stamp_ns)

        # 기존 단일 trajectory 토픽도 발행 (각 submap의 마지막 pose만)
        path = Path()
        path.header = self._make_header(stamp_ns)
        for sid in sorted(self._submap_frame_poses.keys()):
            poses = self._submap_frame_poses[sid]
            if poses:
                path.poses.append(self._pose_stamped(poses[-1], stamp_ns))
        self._pub_path.publish(path)

    def _publish_global_cloud(self, stamp_ns: int) -> None:
        if not self._submap_pts:
            return
        keys = sorted(self._submap_pts.keys())
        all_pts = np.concatenate([self._submap_pts[k] for k in keys], axis=0)
        all_colors = None
        if self._submap_colors:
            color_parts = []
            ok = True
            for k in keys:
                if k in self._submap_colors:
                    color_parts.append(self._submap_colors[k])
                else:
                    ok = False
                    break
            if ok and color_parts:
                all_colors = np.concatenate(color_parts, axis=0)
                if len(all_colors) != len(all_pts):
                    all_colors = None  # 안전장치
        cloud = self._build_pointcloud2(all_pts, stamp_ns, colors=all_colors)
        self._pub_global_cloud.publish(cloud)

    def _publish_global_path(self, stamp_ns: int) -> None:
        path = Path()
        path.header = self._make_header(stamp_ns)
        for sid in sorted(self._submap_frame_poses.keys()):
            for pose in self._submap_frame_poses[sid]:
                path.poses.append(self._pose_stamped(pose, stamp_ns))
        self._pub_global_path.publish(path)

    def _build_pointcloud2(
        self,
        pts: np.ndarray,
        stamp_ns: int,
        colors: np.ndarray | None = None,
    ) -> PointCloud2:
        """colors가 있으면 PointCloud2에 'rgb' float 패킹된 4번째 필드로 추가."""
        cloud = PointCloud2()
        cloud.header = self._make_header(stamp_ns)
        cloud.height = 1
        cloud.width = int(pts.shape[0])
        cloud.is_bigendian = False
        cloud.is_dense = True

        if colors is not None and len(colors) == len(pts):
            # XYZ + RGB(packed float). 16바이트 stride.
            cloud.point_step = 16
            cloud.fields = [
                PointField(name='x', offset=0,
                           datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,
                           datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,
                           datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12,
                           datatype=PointField.UINT32, count=1),
            ]
            buf = np.zeros((len(pts), 4), dtype=np.float32)
            buf[:, 0:3] = pts.astype(np.float32)
            # RGB → 0xRRGGBB packed into uint32, viewed as float32
            r, g, b = colors[:, 0].astype(np.uint32), colors[:, 1].astype(np.uint32), colors[:, 2].astype(np.uint32)
            packed = (r << 16) | (g << 8) | b
            buf[:, 3] = packed.view(np.float32)
            cloud.row_step = cloud.point_step * cloud.width
            cloud.data = buf.tobytes()
            return cloud

        # XYZ only path (기존 포맷)
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = pts.astype(np.float32).tobytes()
        return cloud

    # ── shutdown ──────────────────────────────────────────────────────
    def destroy_node(self) -> bool:
        self._stop = True
        try:
            zmq.Context.instance().term()
        except Exception:
            pass
        if self._server.proc is not None and self._server.proc.poll() is None:
            self._server.proc.terminate()
            try:
                self._server.proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._server.proc.kill()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = VGGTSlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
