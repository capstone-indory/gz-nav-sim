#!/usr/bin/env python3
"""VGGT-SLAM server — runs in Python 3.11 venv, communicates via ZeroMQ.

The ROS bridge node (Python 3.10) pushes JPEG frames here, this server
feeds them to VGGT-SLAM's Solver and publishes incremental SLAM results
(pose, trajectory, submap pointcloud) back over a PUB socket.

Launch from the Python 3.11 venv:
  /root/gz-nav-sim/venv_vggt/bin/python vggt_slam_server.py \
      --repo /root/gz-nav-sim/src/VGGT-SLAM \
      --pull-port 5555 --pub-port 5556
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
import traceback

import cv2
import msgpack
import numpy as np
import torch
import zmq


def _setup_paths(repo_root: str) -> None:
    for p in (
        repo_root,
        os.path.join(repo_root, 'third_party', 'vggt'),
        os.path.join(repo_root, 'third_party', 'salad'),
    ):
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


def _load_vggt(device: str):
    from vggt.models.vggt import VGGT

    model = VGGT()
    url = 'https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt'
    state = torch.hub.load_state_dict_from_url(url)
    model.load_state_dict(state)
    model.eval()
    model = model.to(torch.bfloat16)
    return model.to(device)


def _pack_pose_from_extrinsic(extrinsic: np.ndarray) -> dict:
    R = extrinsic[:3, :3]
    t = extrinsic[:3, 3]
    q = _rotmat_to_quat(R)
    return {
        'tx': float(t[0]), 'ty': float(t[1]), 'tz': float(t[2]),
        'qx': float(q[0]), 'qy': float(q[1]), 'qz': float(q[2]), 'qw': float(q[3]),
    }


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = (tr + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = (1.0 + m00 - m11 - m22) ** 0.5 * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = (1.0 + m11 - m00 - m22) ** 0.5 * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = (1.0 + m22 - m00 - m11) ** 0.5 * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return np.array([qx, qy, qz, qw])


class VGGTSlamServer:
    def __init__(self, args):
        _setup_paths(args.repo)
        from vggt_slam.solver import Solver

        self.submap_size = args.submap_size
        self.overlap = args.overlap
        self.min_disparity = args.min_disparity
        self.max_loops = args.max_loops
        self.conf_threshold = args.conf_threshold
        self.lc_thres = args.lc_thres
        self.pointcloud_stride = args.pointcloud_stride

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f'[vggt_slam_server] device={self.device}', flush=True)

        self.solver = Solver(
            init_conf_threshold=self.conf_threshold,
            lc_thres=self.lc_thres,
            vis_voxel_size=None,
        )
        print('[vggt_slam_server] loading VGGT weights…', flush=True)
        self.model = _load_vggt(self.device)
        print('[vggt_slam_server] VGGT ready', flush=True)

        ctx = zmq.Context.instance()
        self.pull = ctx.socket(zmq.PULL)
        self.pull.bind(f'tcp://127.0.0.1:{args.pull_port}')
        self.pub = ctx.socket(zmq.PUB)
        self.pub.bind(f'tcp://127.0.0.1:{args.pub_port}')
        self.pull.setsockopt(zmq.RCVHWM, 10)
        print(f'[vggt_slam_server] PULL tcp://127.0.0.1:{args.pull_port}', flush=True)
        print(f'[vggt_slam_server] PUB  tcp://127.0.0.1:{args.pub_port}', flush=True)

        self.tmp_dir = tempfile.mkdtemp(prefix='vggt_slam_frames_')
        self.image_paths: list[str] = []
        self.submap_count = 0
        self.last_stamp_ns = 0

    def _save_frame_to_tmp(self, jpeg_bytes: bytes, frame_id: int) -> tuple[str, np.ndarray | None]:
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return '', None
        path = os.path.join(self.tmp_dir, f'frame_{frame_id:08d}.jpg')
        cv2.imwrite(path, img)
        return path, img

    def _publish_submap(self, submap_id: int):
        """Latest submap's camera pose + transformed world-frame pointcloud."""
        try:
            submap = self.solver.map.get_submap(submap_id)
            pose_world = submap.get_last_pose_world(self.solver.graph)
            msg = {
                'type': 'submap',
                'submap_id': int(submap_id),
                'stamp_ns': int(self.last_stamp_ns),
                'pose': _pack_pose_from_extrinsic(pose_world),
            }
            pts = submap.get_points_in_world_frame(self.solver.graph)
            if pts is not None and len(pts) > 0:
                stride = max(1, self.pointcloud_stride)
                pts = pts[::stride].astype(np.float32)
                msg['pointcloud'] = pts.tobytes()
                msg['pointcloud_count'] = int(len(pts))
            self.pub.send(msgpack.packb(msg, use_bin_type=True))
        except Exception:
            traceback.print_exc()

    def _publish_trajectory(self):
        """All submap last-pose waypoints in world frame."""
        try:
            poses = []
            for i in range(self.solver.map.get_num_submaps()):
                sm = self.solver.map.get_submap(i)
                poses.append(_pack_pose_from_extrinsic(
                    sm.get_last_pose_world(self.solver.graph)))
            self.pub.send(msgpack.packb({
                'type': 'trajectory',
                'stamp_ns': int(self.last_stamp_ns),
                'poses': poses,
            }, use_bin_type=True))
        except Exception:
            traceback.print_exc()

    def run(self):
        print('[vggt_slam_server] waiting for frames…', flush=True)
        while True:
            try:
                raw = self.pull.recv()
            except KeyboardInterrupt:
                break

            try:
                req = msgpack.unpackb(raw, raw=False)
            except Exception:
                print('[vggt_slam_server] bad msgpack, skipping', flush=True)
                continue

            if req.get('type') != 'frame':
                continue

            stamp_ns = int(req.get('stamp_ns', time.time_ns()))
            self.last_stamp_ns = stamp_ns
            jpeg = req['jpeg']
            frame_id = int(req.get('frame_id', len(self.image_paths)))

            path, img = self._save_frame_to_tmp(jpeg, frame_id)
            if img is None:
                continue

            try:
                enough = self.solver.flow_tracker.compute_disparity(
                    img, self.min_disparity, False)
            except Exception:
                traceback.print_exc()
                continue

            if not enough:
                try:
                    os.remove(path)
                except OSError:
                    pass
                continue

            self.image_paths.append(path)

            if len(self.image_paths) < self.submap_size + self.overlap:
                continue

            try:
                predictions = self.solver.run_predictions(
                    self.image_paths, self.model, self.max_loops, None, None)
                self.solver.add_points(predictions)
                self.solver.graph.optimize()
                self.submap_count += 1
                print(f'[vggt_slam_server] submap {self.submap_count} '
                      f'processed (frames={len(self.image_paths)})', flush=True)
                # 서브맵은 frame_id로 키잉돼서 0,9,18,…처럼 점프함.
                # 마지막 키를 직접 가져와 publish.
                last_id = self.solver.map.get_largest_key()
                if last_id is not None:
                    self._publish_submap(last_id)
                self._publish_trajectory()
            except Exception:
                traceback.print_exc()

            keep = self.image_paths[-self.overlap:]
            for p in self.image_paths[:-self.overlap]:
                try:
                    os.remove(p)
                except OSError:
                    pass
            self.image_paths = keep


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', default='/root/gz-nav-sim/src/VGGT-SLAM')
    parser.add_argument('--pull-port', type=int, default=5555)
    parser.add_argument('--pub-port', type=int, default=5556)
    parser.add_argument('--submap-size', type=int, default=8)
    parser.add_argument('--overlap', type=int, default=1)
    parser.add_argument('--min-disparity', type=float, default=50.0)
    parser.add_argument('--max-loops', type=int, default=1)
    parser.add_argument('--conf-threshold', type=float, default=25.0)
    parser.add_argument('--lc-thres', type=float, default=0.95)
    parser.add_argument('--pointcloud-stride', type=int, default=8)
    args = parser.parse_args()

    server = VGGTSlamServer(args)
    try:
        server.run()
    except KeyboardInterrupt:
        print('\n[vggt_slam_server] shutdown', flush=True)


if __name__ == '__main__':
    main()
