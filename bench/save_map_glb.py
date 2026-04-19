#!/usr/bin/env python3
"""현재 매핑 결과를 glb로 저장.

VGGT 모드: /vggt_slam/global_pointcloud → trimesh PointCloud → glb
nvblox 모드: /nvblox_node/mesh (MarkerArray) → trimesh Trimesh → glb (구현 추가 필요)

사용법:
  python3 bench/save_map_glb.py --topic /vggt_slam/global_pointcloud --out map.glb
  python3 bench/save_map_glb.py --auto --out map.glb   # 둘 중 발견되는 토픽
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Saver(Node):
    def __init__(self, topic: str, out_path: str):
        super().__init__('save_map_glb')
        self.topic = topic
        self.out = out_path
        self.captured = False
        # global_pointcloud는 TRANSIENT_LOCAL latched
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.get_logger().info(f'subscribed {topic}, out={out_path}')

    def _cb(self, msg: PointCloud2):
        if self.captured:
            return
        try:
            import trimesh
        except ImportError:
            self.get_logger().error('trimesh 필요: pip install trimesh')
            self.captured = True
            return

        pts_iter = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array(list(pts_iter), dtype=np.float32)
        if pts.ndim == 1:
            pts = pts.view(np.float32).reshape(-1, 3)
        elif pts.shape[1] != 3:
            pts = pts[:, :3].astype(np.float32)
        n = len(pts)
        if n == 0:
            self.get_logger().warn('empty pointcloud')
            return
        # color 추출 시도 (rgb field 있을 때만)
        colors = None
        for f in msg.fields:
            if f.name == 'rgb':
                rgb_iter = point_cloud2.read_points(
                    msg, field_names=('rgb',), skip_nans=True)
                rgb_raw = np.array(list(rgb_iter), dtype=np.uint32)
                if rgb_raw.ndim > 1:
                    rgb_raw = rgb_raw.flatten()
                # rgb_raw는 packed uint32 (0xRRGGBB)
                r = ((rgb_raw >> 16) & 0xFF).astype(np.uint8)
                g = ((rgb_raw >> 8) & 0xFF).astype(np.uint8)
                b = (rgb_raw & 0xFF).astype(np.uint8)
                colors = np.stack([r, g, b, np.full_like(r, 255)], axis=-1)
                break

        cloud = trimesh.PointCloud(pts, colors=colors)
        scene = trimesh.Scene([cloud])
        scene.export(self.out)
        self.get_logger().info(
            f'saved {n} points → {self.out} ({"with colors" if colors is not None else "xyz only"})')
        self.captured = True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/vggt_slam/global_pointcloud')
    p.add_argument('--out', required=True)
    p.add_argument('--timeout', type=float, default=10.0)
    args = p.parse_args()

    rclpy.init()
    node = Saver(args.topic, args.out)
    deadline = time.time() + args.timeout
    try:
        while rclpy.ok() and not node.captured and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    sys.exit(0 if node.captured else 1)


if __name__ == '__main__':
    main()
