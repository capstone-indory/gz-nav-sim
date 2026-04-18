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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Header


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

        self._pub_pose = self.create_publisher(PoseStamped, g('pose_topic'), 10)
        self._pub_path = self.create_publisher(Path, g('trajectory_topic'), 10)
        self._pub_cloud = self.create_publisher(PointCloud2, g('pointcloud_topic'), 5)

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
    def _on_image(self, msg: CompressedImage) -> None:
        payload = {
            'type': 'frame',
            'frame_id': self._frame_id,
            'stamp_ns': int(msg.header.stamp.sec) * 1_000_000_000
                        + int(msg.header.stamp.nanosec),
            'format': msg.format,
            'jpeg': bytes(msg.data),
        }
        try:
            self._push.send(msgpack.packb(payload, use_bin_type=True),
                            flags=zmq.NOBLOCK)
        except zmq.Again:
            pass  # server slow; drop frame to avoid backpressure
        self._frame_id += 1

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
        self._pub_pose.publish(self._pose_stamped(msg['pose'], stamp_ns))
        if 'pointcloud' in msg and msg.get('pointcloud_count', 0) > 0:
            pts = np.frombuffer(msg['pointcloud'], dtype=np.float32).reshape(-1, 3)
            cloud = self._build_pointcloud2(pts, stamp_ns)
            self._pub_cloud.publish(cloud)

    def _publish_trajectory(self, msg: dict) -> None:
        stamp_ns = int(msg.get('stamp_ns', 0))
        path = Path()
        path.header = self._make_header(stamp_ns)
        for pose in msg.get('poses', []):
            path.poses.append(self._pose_stamped(pose, stamp_ns))
        self._pub_path.publish(path)

    def _build_pointcloud2(self, pts: np.ndarray, stamp_ns: int) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = self._make_header(stamp_ns)
        cloud.height = 1
        cloud.width = int(pts.shape[0])
        cloud.is_bigendian = False
        cloud.is_dense = True
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
