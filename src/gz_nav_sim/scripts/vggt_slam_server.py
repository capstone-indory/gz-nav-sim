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


def _pack_RT(R: np.ndarray, t: np.ndarray) -> dict:
    """3x3 R + 3-vec t → 직렬화 가능한 dict."""
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
        self._prev_num_submaps = 0  # loop closure 검출용 (submap 수 점프)
        # frame buffer에 같은 인덱스로 lidar anchor 보관 (None이면 anchor 없음).
        # image_paths가 keyframe만 보관하므로 1:1 매칭 유지.
        self._lidar_anchors: list[dict | None] = []
        # 첫 submap 생성 시 한 번만 측정해서 SIM(3) 글로벌 변환 저장.
        # P_world = R_global @ (scale * P_vggt) + t_global
        # VGGT-SLAM 그래프 자체는 건드리지 않음 (normalized 좌표 그대로 유지).
        # 모든 publish에서 위 변환만 적용 → 한 번 calibrate 후 글로벌 일관성.
        self._global_scale: float | None = None
        self._global_R: np.ndarray | None = None  # (3,3) optical → world rotation
        self._global_t: np.ndarray | None = None  # (3,) world translation

    def _loop_closure_occurred(self) -> bool:
        """add_points 후 submap 수가 1보다 많이 늘었으면 loop closure submap이 추가된 것."""
        cur = self.solver.map.get_num_submaps()
        delta = cur - self._prev_num_submaps
        self._prev_num_submaps = cur
        # 정상은 delta==1 (새 submap 1개). loop closure 발생 시 추가 submap이 더 들어옴.
        return delta > 1

    def _measure_initial_global_transform(self, submap_id: int) -> None:
        """**첫 keyframe** 기준으로 한 번 SIM(3) 글로벌 변환 측정.

        VGGT-world 원점 = 첫 keyframe (정의상). 따라서:
          R_g = R_w  (첫 keyframe의 world orientation)
          t_g = t_w  (첫 keyframe의 world position)
        scale = lidar median(z_cam) / VGGT median(depth_at_pixel) — 첫 frame에서 측정.

        실패해도 첫 keyframe의 anchor가 동일하게 유지되므로 다음 submap에서 그대로 재시도.
        (다음 submap이 와도 self._lidar_anchors[0]는 같은 첫 keyframe.)
        """
        if not self._lidar_anchors:
            return
        first_anchor = self._lidar_anchors[0]
        if first_anchor is None or 'T_world_camera' not in first_anchor:
            return

        try:
            scale, n_pts = self._measure_lidar_scale_at_frame(
                submap_id, first_anchor, frame_index=0)
        except Exception:
            traceback.print_exc()
            return

        min_pts = int(first_anchor.get('min_points', 20))
        if n_pts < min_pts or scale is None:
            print(f'[global_xform] skip (waiting for valid first-frame): '
                  f'n_pts={n_pts} < min={min_pts}', flush=True)
            return
        if scale < 0.1 or scale > 10.0:
            print(f'[global_xform] skip extreme scale={scale:.3f}', flush=True)
            return

        # 첫 keyframe의 world pose가 곧 SIM(3) 변환 (VGGT origin = first cam)
        T_w_cam = np.frombuffer(
            first_anchor['T_world_camera'], dtype=np.float64).reshape(4, 4)
        self._global_R = T_w_cam[:3, :3].copy()
        self._global_t = T_w_cam[:3, 3].copy()
        self._global_scale = float(scale)
        print(f'[global_xform] LOCKED at first keyframe: '
              f'scale={scale:.4f} t_g={self._global_t.tolist()} (n={n_pts})',
              flush=True)

    def _apply_global_xform_pose(self, T_vggt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """VGGT pose (camera-to-vggt-world) → (R_world, t_world) in odom frame."""
        scale = self._global_scale or 1.0
        R_g = self._global_R if self._global_R is not None else np.eye(3)
        t_g = self._global_t if self._global_t is not None else np.zeros(3)
        R_v = T_vggt[:3, :3]
        t_v = T_vggt[:3, 3] * scale
        # Compose: T_world = T_g @ T_v_scaled
        R_world = R_g @ R_v
        t_world = R_g @ t_v + t_g
        return R_world, t_world

    def _apply_global_xform_points(self, pts: np.ndarray) -> np.ndarray:
        """(N, 3) VGGT-world 점들 → odom 프레임으로 SIM(3) 변환."""
        scale = self._global_scale or 1.0
        R_g = self._global_R if self._global_R is not None else np.eye(3)
        t_g = self._global_t if self._global_t is not None else np.zeros(3)
        return ((R_g @ (pts.T * scale)).T + t_g).astype(np.float32)

    def _measure_lidar_scale_at_frame(self, submap_id: int, anchor: dict,
                                      frame_index: int = 0
                                      ) -> tuple[float | None, int]:
        """DA3 _apply_lidar_scale 패턴: 라이다 빔을 지정 frame 이미지에 픽셀 투영,
        VGGT가 같은 픽셀에서 추정한 depth와 비교해 scale ratio (median).

        VGGT submap 구조:
          submap.pointclouds[i]  : (H, W, 3)  — first-camera frame 좌표
          submap.poses[i]        : 4x4        — first_cam → cam_i 변환
        frame_index 카메라 기준 depth = (poses[i] @ pointclouds[i])[..., 2]
        frame_index=0이면 poses[0]≈identity → depth = pointclouds[0][..., 2]
        """
        scan_d = anchor['scan']
        ranges = np.frombuffer(scan_d['ranges'], dtype=np.float32)
        n_total = int(scan_d.get('ranges_count', len(ranges)))
        ranges = ranges[:n_total]
        angle_min = float(scan_d['angle_min'])
        angle_inc = float(scan_d['angle_increment'])
        rmin = float(scan_d['range_min'])
        rmax = float(scan_d['range_max'])

        valid = np.isfinite(ranges) & (ranges > rmin) & (ranges < rmax)
        if not np.any(valid):
            return None, 0

        r = ranges[valid]
        a = angle_min + np.arange(len(ranges))[valid] * angle_inc
        # 라이다 로컬: x=r*cos(a), y=r*sin(a), z=0 (2D scanner) — DA3와 동일
        lidar_pts = np.stack(
            [r * np.cos(a), r * np.sin(a), np.zeros_like(r), np.ones_like(r)],
            axis=-1).astype(np.float64)  # (N, 4)

        T_camera_lidar = np.frombuffer(
            anchor['T_camera_lidar'], dtype=np.float64).reshape(4, 4)
        cam_pts = (T_camera_lidar @ lidar_pts.T).T[:, :3]  # (N, 3) optical
        # 카메라 +Z forward (optical convention)
        z_cam = cam_pts[:, 2]
        front = z_cam > 0.1
        n_lidar_total = int(len(z_cam))
        n_lidar_front = int(np.sum(front))
        if n_lidar_front == 0:
            print(f'[lidar_diag] submap {submap_id}: '
                  f'lidar={n_lidar_total} front=0 — '
                  f'check T_camera_lidar (z range: '
                  f'{z_cam.min():.2f}..{z_cam.max():.2f})', flush=True)
            return None, 0
        cam_pts = cam_pts[front]
        z_cam = z_cam[front]

        # VGGT submap의 frame_index 카메라 기준 per-pixel depth_map
        try:
            submap = self.solver.map.get_submap(submap_id)
            pcs = submap.pointclouds  # (S, H, W, 3) in first-cam frame
            poses = submap.poses      # (S, 4, 4) first_cam → cam_i
            if pcs is None or len(pcs) == 0:
                return None, 0
            idx = int(frame_index)
            if idx < 0 or idx >= len(pcs):
                return None, 0
            pc_first = np.asarray(pcs[idx])         # (H, W, 3)
            T_first_to_i = np.asarray(poses[idx])   # (4,4)
            H, W, _ = pc_first.shape
            flat = pc_first.reshape(-1, 3)
            ones = np.ones((flat.shape[0], 1), dtype=flat.dtype)
            flat_h = np.concatenate([flat, ones], axis=1)
            pc_cam = (T_first_to_i @ flat_h.T).T[:, :3].reshape(H, W, 3)
            depth_map = pc_cam[..., 2].astype(np.float32)
        except Exception:
            traceback.print_exc()
            return None, 0

        # camera_info K → VGGT 처리 해상도(H, W)에 맞게 스케일
        K_full = np.frombuffer(anchor['K'], dtype=np.float64).reshape(3, 3).copy()
        img_w = int(anchor.get('image_width', 0))
        img_h = int(anchor.get('image_height', 0))
        sx = (W / img_w) if img_w > 0 else 1.0
        sy = (H / img_h) if img_h > 0 else 1.0
        fx = float(K_full[0, 0]) * sx
        fy = float(K_full[1, 1]) * sy
        cx = float(K_full[0, 2]) * sx
        cy = float(K_full[1, 2]) * sy

        # 라이다 점을 이미지 픽셀 좌표로 투영 (DA3 동일)
        u = (cam_pts[:, 0] * fx / z_cam + cx).astype(int)
        v = (cam_pts[:, 1] * fy / z_cam + cy).astype(int)
        in_image = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        if not np.any(in_image):
            return None, int(np.sum(front))
        u, v, z_cam = u[in_image], v[in_image], z_cam[in_image]

        # 같은 픽셀에서 VGGT depth 가져와서 비교
        vggt_d = depth_map[v, u]
        valid_d = np.isfinite(vggt_d) & (vggt_d > 0.05) & (vggt_d < 50.0)
        n_pts = int(np.sum(valid_d))
        if n_pts < 5:
            return None, n_pts

        scale = float(np.median(z_cam[valid_d] / vggt_d[valid_d]))
        return scale, n_pts

    def _save_frame_to_tmp(self, jpeg_bytes: bytes, frame_id: int) -> tuple[str, np.ndarray | None]:
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return '', None
        path = os.path.join(self.tmp_dir, f'frame_{frame_id:08d}.jpg')
        cv2.imwrite(path, img)
        return path, img

    def _publish_submap(self, submap_id: int):
        """Submap의 모든 frame pose + world-frame 점군 + 색상 송신.

        self._global_scale이 잡혀 있으면 점군 xyz와 pose translation에 곱.
        """
        try:
            submap = self.solver.map.get_submap(submap_id)
            graph = self.solver.graph
            pose_world = submap.get_last_pose_world(graph)
            R_w, t_w = self._apply_global_xform_pose(np.asarray(pose_world))
            msg = {
                'type': 'submap',
                'submap_id': int(submap_id),
                'stamp_ns': int(self.last_stamp_ns),
                'pose': _pack_RT(R_w, t_w),
            }

            # 모든 프레임의 world-frame extrinsic
            try:
                all_extr = submap.get_all_poses_world(graph)
                packed = []
                for e in all_extr:
                    R_e, t_e = self._apply_global_xform_pose(np.asarray(e))
                    packed.append(_pack_RT(R_e, t_e))
                msg['frame_poses'] = packed
            except Exception:
                msg['frame_poses'] = [msg['pose']]

            pts = submap.get_points_in_world_frame(graph)
            colors = None
            try:
                colors = submap.get_points_colors()
            except Exception:
                colors = None

            if pts is not None and len(pts) > 0:
                pts = pts.reshape(-1, 3) if pts.ndim > 2 else pts
                stride = max(1, self.pointcloud_stride)
                pts = pts[::stride].astype(np.float32)
                pts = self._apply_global_xform_points(pts)
                msg['pointcloud'] = pts.tobytes()
                msg['pointcloud_count'] = int(len(pts))
                if colors is not None and len(colors) > 0:
                    colors = np.asarray(colors).reshape(-1, 3)[::stride]
                    if colors.dtype != np.uint8:
                        colors = (colors * 255 if colors.max() <= 1.0 + 1e-6
                                  else colors).clip(0, 255).astype(np.uint8)
                    n = min(len(colors), len(pts))
                    msg['colors'] = colors[:n].tobytes()
                    msg['colors_count'] = int(n)
            self.pub.send(msgpack.packb(msg, use_bin_type=True))
        except Exception:
            traceback.print_exc()

    def _publish_trajectory(self):
        """모든 submap의 모든 frame pose 송신 (loop closure 후 재정렬 반영)."""
        try:
            graph = self.solver.graph
            entries = []  # (submap_id, [pose, ...])
            # GraphMap.submaps는 frame_id 점프 키 (0, 9, 18, ...) → 키 정렬 필요
            for sid in sorted(self.solver.map.submaps.keys()):
                sm = self.solver.map.get_submap(sid)
                try:
                    extr = sm.get_all_poses_world(graph)
                    packed = []
                    for e in extr:
                        R_e, t_e = self._apply_global_xform_pose(np.asarray(e))
                        packed.append(_pack_RT(R_e, t_e))
                    poses = packed
                except Exception:
                    R_e, t_e = self._apply_global_xform_pose(
                        np.asarray(sm.get_last_pose_world(graph)))
                    poses = [_pack_RT(R_e, t_e)]
                entries.append({'submap_id': int(sid), 'poses': poses})
            self.pub.send(msgpack.packb({
                'type': 'trajectory',
                'stamp_ns': int(self.last_stamp_ns),
                'submaps': entries,
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
            self._lidar_anchors.append(req.get('lidar_anchor'))

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
                    # 첫 submap에서 한 번만 SIM(3) 글로벌 변환 측정 (scale + R + t).
                    # VGGT 그래프는 normalized 상태 유지 — 출력 단계에서만 변환.
                    if self._global_scale is None:
                        self._measure_initial_global_transform(last_id)

                    # viser에 점군 + 카메라 frustum push
                    # add_points 내부에서 frames만 그려져서, 점군 별도 호출 필요.
                    try:
                        latest = self.solver.map.get_submap(last_id)
                        self.solver.set_submap_point_cloud(latest)
                    except Exception:
                        traceback.print_exc()
                    self._publish_submap(last_id)
                self._publish_trajectory()

                # Loop closure가 발생했으면 모든 submap의 viser scene 재구성
                # (loop closure는 _publish_submap 후 graph.optimize에서 발생 가능)
                if self._loop_closure_occurred():
                    try:
                        self.solver.update_all_submap_vis()
                    except Exception:
                        traceback.print_exc()
            except Exception:
                traceback.print_exc()

            keep = self.image_paths[-self.overlap:]
            keep_anchors = self._lidar_anchors[-self.overlap:]
            for p in self.image_paths[:-self.overlap]:
                try:
                    os.remove(p)
                except OSError:
                    pass
            self.image_paths = keep
            self._lidar_anchors = keep_anchors


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
