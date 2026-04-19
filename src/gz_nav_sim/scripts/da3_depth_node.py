#!/usr/bin/env python3

"""ROS 2 wrapper for Depth Anything 3 monocular depth inference.

Multi-view batch + keyframe selection + per-frame world projection + accumulated RGB cloud.
"""

from __future__ import annotations

from collections import deque
import importlib.util
import os
import sys
import threading
import traceback
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class Da3DepthNode(Node):
    """Subscribe to RGB + camera_info and publish DA3 depth and point cloud."""

    def __init__(self) -> None:
        super().__init__('da3_depth_node')

        self.declare_parameter('image_topic', '/front_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/front_camera/camera_info')
        self.declare_parameter('depth_topic', '/front_camera/depth/image_raw')
        self.declare_parameter('depth_camera_info_topic', '/front_camera/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/front_camera/depth/points')
        self.declare_parameter('da3_repo_path', '')
        self.declare_parameter('model_id', 'depth-anything/DA3-Large')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('process_res', 336)
        self.declare_parameter('process_res_method', 'upper_bound_resize')
        self.declare_parameter('inference_rate_hz', 1.0)
        self.declare_parameter('input_views', 1)
        self.declare_parameter('da3_log_level', 'WARN')
        self.declare_parameter('ref_view_strategy', 'middle')
        # batch 사이 overlap (input_views 중 다음 batch와 공유할 frame 수)
        self.declare_parameter('keyframe_overlap', 0)
        self.declare_parameter('point_cloud_stride', 4)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 20.0)
        self.declare_parameter('camera_frame', 'front_camera_optical_frame')
        self.declare_parameter('point_cloud_frame', 'front_camera_optical_frame')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lidar_scale', True)
        self.declare_parameter('lidar_scale_min_points', 5)
        # Keyframe 선별 (VGGT 패턴: 충분히 움직인 frame만 큐에 적재)
        self.declare_parameter('keyframe_min_disparity', 30.0)
        self.declare_parameter('keyframe_max_buffer_age_s', 5.0)
        # 누적 world-frame RGB pointcloud
        self.declare_parameter('world_pointcloud_topic', '/camera/depth/world_points')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('world_pc_max_points', 500000)
        self.declare_parameter('world_pc_voxel_size', 0.05)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._input_views = max(int(self.get_parameter('input_views').value), 1)
        # Keyframe 버퍼 — disparity 통과 frame 적재.
        # maxlen은 input_views의 여러 batch 분 (추론 중 쌓이는 거 흡수). 초과 시 가장 오래된 것 drop.
        self._image_buffer: deque[tuple[Image, CameraInfo]] = deque(maxlen=self._input_views * 4)
        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_scan: Optional[LaserScan] = None
        self._processing = False
        self._model_ready = False
        self._model_error_logged = False
        self._waited_for_views_logged = False

        # Disparity-based keyframe gate (VGGT 스타일)
        self._kf_min_disparity = float(self.get_parameter('keyframe_min_disparity').value)
        self._kf_max_age_s = float(self.get_parameter('keyframe_max_buffer_age_s').value)
        self._last_kf_gray: Optional[np.ndarray] = None  # 마지막 채택 keyframe의 grayscale (작은 해상도)
        self._kf_disp_resize = (320, 240)  # disparity 계산용 다운샘플 해상도

        # 매 batch (5 keyframe)의 world-frame RGB pointcloud (누적 X)
        self._world_voxel = float(self.get_parameter('world_pc_voxel_size').value)
        self._world_frame = str(self.get_parameter('world_frame').value)
        # confidence/sky 필터 (DA3 prediction에 conf/sky 출력 활용)
        self.declare_parameter('min_confidence', 0.5)

        # Temporal smoothing of (s, t) — batch별 scale jitter 흡수
        # alpha 작을수록 smooth ↑ (변화 둔감, 안정적)
        # alpha 크면 새 측정 빨리 반영하지만 jitter 그대로
        # 0.3 = 새 batch가 30% 영향, 옛 값 70% 유지
        self.declare_parameter('affine_smooth_alpha', 0.3)
        self.declare_parameter('affine_smooth_min_inliers', 20)
        self._s_smooth = 1.0
        self._t_smooth = 0.0
        self._affine_initialized = False

        self._torch = None
        self._model = None
        self._tf_buffer = Buffer()
        # spin_thread=True → TF subscription이 자체 쓰레드로 동작 →
        # 메인 콜백이 바빠도 TF buffer가 꾸준히 업데이트됨
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_info_topic = self.get_parameter('depth_camera_info_topic').value
        point_cloud_topic = self.get_parameter('point_cloud_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._depth_pub = self.create_publisher(Image, depth_topic, output_qos)
        self._depth_info_pub = self.create_publisher(CameraInfo, depth_info_topic, output_qos)
        self._points_pub = self.create_publisher(PointCloud2, point_cloud_topic, output_qos)
        # 누적 world-frame RGB cloud
        world_pc_topic = str(self.get_parameter('world_pointcloud_topic').value)
        self._world_pc_pub = self.create_publisher(PointCloud2, world_pc_topic, output_qos)
        self.create_subscription(Image, image_topic, self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)

        # event-driven: timer 없음. _on_image에서 5장 채워지면 즉시 _trigger_inference.
        # 추론 끝난 직후에도 buffer 확인 (밀린 frame 처리).
        threading.Thread(target=self._load_model, daemon=True).start()

        self.get_logger().info(
            f'DA3 wrapper ready. image={image_topic}, info={camera_info_topic}, '
            f'scan={scan_topic}, depth={depth_topic}, points={point_cloud_topic}'
        )

    def _on_image(self, msg: Image) -> None:
        if self._latest_camera_info is None:
            return  # camera_info 없으면 버림

        # disparity 기반 keyframe gate
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            return
        gray = cv2.cvtColor(
            cv2.resize(rgb, self._kf_disp_resize), cv2.COLOR_RGB2GRAY)

        accepted = False
        with self._lock:
            if self._last_kf_gray is None:
                self._last_kf_gray = gray
                self._image_buffer.append((msg, self._latest_camera_info))
                accepted = True
            else:
                disp = self._compute_mean_disparity(self._last_kf_gray, gray)
                if disp >= self._kf_min_disparity:
                    self._last_kf_gray = gray
                    self._image_buffer.append((msg, self._latest_camera_info))
                    accepted = True

        # 5장 모이고 미추론 상태면 즉시 trigger
        if accepted:
            self._maybe_trigger_inference()

    def _compute_mean_disparity(self, prev: np.ndarray, curr: np.ndarray) -> float:
        """Shi-Tomasi corners + LK optical flow 평균 픽셀 변위. 실패 시 0."""
        try:
            corners = cv2.goodFeaturesToTrack(
                prev, maxCorners=100, qualityLevel=0.01, minDistance=8)
            if corners is None or len(corners) < 5:
                return 0.0
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                prev, curr, corners, None, winSize=(21, 21), maxLevel=2)
            if new_pts is None or status is None:
                return 0.0
            ok = status.flatten() == 1
            if not np.any(ok):
                return 0.0
            d = np.linalg.norm(new_pts[ok] - corners[ok], axis=-1)
            return float(np.median(d.ravel()))
        except Exception:
            return 0.0

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._latest_camera_info = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    def _maybe_trigger_inference(self) -> None:
        """input_views 만큼 모이고 미추론이면 batch를 꺼내 추론 thread 시작.

        - input_views 장을 추론 입력으로 사용 (sliding window)
        - 추론 후 (input_views - keyframe_overlap) 개의 가장 오래된 frame만 buffer에서 drop
        - keyframe_overlap 만큼은 buffer에 남아 다음 batch와 공유 → 연속성 향상

        호출처:
        - _on_image: 새 keyframe 적재 직후 (event-driven 트리거)
        - _infer_and_publish 종료: 추론 중 쌓인 frame 처리 (back-to-back 가능)
        """
        if not self._model_ready or self._processing:
            return

        with self._lock:
            if len(self._image_buffer) < self._input_views:
                return
            frames = list(self._image_buffer)[:self._input_views]
            scan = self._latest_scan
            self._processing = True
            # overlap 만큼 남기고 가장 오래된 frame들을 drop.
            overlap = max(0, min(int(self.get_parameter('keyframe_overlap').value),
                                 self._input_views - 1))
            n_drop = self._input_views - overlap
            for _ in range(n_drop):
                self._image_buffer.popleft()

        image_msgs = [f[0] for f in frames]
        camera_info_msgs = [f[1] for f in frames]

        threading.Thread(
            target=self._infer_and_publish,
            args=(image_msgs, camera_info_msgs, scan),
            daemon=True,
        ).start()

    def _load_model(self) -> None:
        try:
            repo_path = str(self.get_parameter('da3_repo_path').value).strip()
            self._append_da3_repo_path(repo_path)
            missing = self._missing_python_modules()
            if missing:
                raise RuntimeError(
                    'Missing DA3 runtime dependencies: '
                    + ', '.join(missing)
                    + '. Install them into the ros2 environment first.'
                )

            import torch  # noqa: WPS433

            os.environ['DA3_LOG_LEVEL'] = str(self.get_parameter('da3_log_level').value).upper()
            from depth_anything_3.api import DepthAnything3  # noqa: WPS433

            model_id = str(self.get_parameter('model_id').value)
            device = self._resolve_device(torch, str(self.get_parameter('device').value))
            self.get_logger().info(f'Loading DA3 model {model_id} on {device}...')

            model = DepthAnything3.from_pretrained(model_id)
            model = model.to(device=device)
            model.eval()

            self._torch = torch
            self._model = model
            self._model_ready = True
            self.get_logger().info('DA3 model loaded successfully.')
        except Exception as exc:  # pragma: no cover - runtime dependency path
            self._model_ready = False
            if not self._model_error_logged:
                self._model_error_logged = True
                self.get_logger().error(f'Failed to initialize DA3: {exc}')
                self.get_logger().error(traceback.format_exc())

    def _append_da3_repo_path(self, repo_path: str) -> None:
        candidates = []
        if repo_path:
            candidates.extend([repo_path, os.path.join(repo_path, 'src')])
        for candidate in candidates:
            package_root = os.path.join(candidate, 'depth_anything_3')
            if os.path.isdir(package_root) and candidate not in sys.path:
                sys.path.insert(0, candidate)
                return
        raise FileNotFoundError(
            f'Unable to find depth_anything_3 package under {repo_path!r}. '
            'Expected a cloned Depth-Anything-3 repository.'
        )

    def _missing_python_modules(self) -> list[str]:
        modules = [
            'torch',
            'torchvision',
            'huggingface_hub',
            'einops',
            'omegaconf',
            'addict',
            'safetensors',
            'depth_anything_3',
        ]
        return [module for module in modules if importlib.util.find_spec(module) is None]

    def _resolve_device(self, torch_module, requested: str) -> str:
        requested = requested.lower()
        if requested != 'auto':
            return requested
        if torch_module.cuda.is_available():
            return 'cuda'
        if hasattr(torch_module.backends, 'mps') and torch_module.backends.mps.is_available():
            return 'mps'
        return 'cpu'

    def _infer_and_publish(
        self,
        image_msgs: list[Image],
        camera_info_msgs: list[CameraInfo],
        scan: Optional[LaserScan],
    ) -> None:
        import time as _time
        t_start = _time.perf_counter()
        try:
            rgbs = [self._bridge.imgmsg_to_cv2(m, desired_encoding='rgb8') for m in image_msgs]
            t_decode = _time.perf_counter()
            intrinsics_batch = np.stack(
                [self._camera_info_to_matrix(ci) for ci in camera_info_msgs],
                axis=0,
            ).astype(np.float32)

            # Per-frame extrinsics — DA3는 world-to-camera (CV 표준) convention.
            # _build_extrinsics_batch는 c2w (TF lookup 결과) → 여기서 invert.
            extrinsics_c2w = self._build_extrinsics_batch(image_msgs)
            extrinsics_w2c = None
            if extrinsics_c2w is not None:
                extrinsics_w2c = np.stack(
                    [np.linalg.inv(T) for T in extrinsics_c2w], axis=0
                ).astype(np.float32)

            inf_kwargs = dict(
                intrinsics=intrinsics_batch,
                process_res=int(self.get_parameter('process_res').value),
                process_res_method=str(self.get_parameter('process_res_method').value),
                ref_view_strategy=str(self.get_parameter('ref_view_strategy').value),
            )
            if extrinsics_w2c is not None:
                inf_kwargs['extrinsics'] = extrinsics_w2c
            t_tf = _time.perf_counter()
            prediction = self._model.inference(rgbs, **inf_kwargs)
            t_infer = _time.perf_counter()

            n = len(image_msgs)
            depths = [np.asarray(prediction.depth[i], dtype=np.float32) for i in range(n)]
            apply_lidar = bool(self.get_parameter('lidar_scale').value) and scan is not None

            # K (depth 해상도 기준) 미리 계산
            K_list = [self._gt_intrinsics(camera_info_msgs[i], depths[i].shape) for i in range(n)]
            # metric scaling (모델별 hack — 보통 안 쓰임)
            if self._needs_metric_scaling(prediction):
                depths = [self._apply_metric_scaling(depths[i], K_list[i]) for i in range(n)]

            # ── BATCH 단위 lidar 2-DOF affine fit (disparity space) ─────
            # DA3는 MiDaS 계열 scale-shift invariant loss로 학습됨 →
            # 모델 출력은 DISPARITY 공간에서 정확히 2-DOF affine 모호함.
            # 따라서 1/d_metric = s × (1/d_pred) + t 로 fit해야 학습 도메인과 일치.
            #
            # 1) frame별 (z_lidar, z_da3) pair 수집 (conf + edge 필터)
            # 2) 11m hard cut (라이다 effective range)
            # 3) RANSAC + stratified sampling (거리 bin별 균등) → best (s,t)
            # 4) 모든 frame depth에 동일 affine 적용
            batch_s, batch_t = 1.0, 0.0
            if apply_lidar:
                all_lidar = []
                all_da3 = []
                for i in range(n):
                    conf_i = (np.asarray(prediction.conf[i], dtype=np.float32)
                              if getattr(prediction, 'conf', None) is not None else None)
                    pairs = self._collect_lidar_pairs(
                        depths[i], K_list[i], image_msgs[i], scan, conf=conf_i)
                    if pairs is not None:
                        all_lidar.append(pairs[0])
                        all_da3.append(pairs[1])
                min_total = int(self.get_parameter('lidar_scale_min_points').value)
                if all_lidar:
                    z_l = np.concatenate(all_lidar).astype(np.float64)
                    z_d = np.concatenate(all_da3).astype(np.float64)

                    # 1차 hard cut: 라이다 effective range (sensor max 10m × 1.1 마진)
                    # 라이다 spec: range 0.05~12m, resolution 1.5cm
                    keep = ((z_l > 0.1) & (z_l < 11.0)
                            & (z_d > 0.05) & (z_d < 30.0)
                            & np.isfinite(z_l) & np.isfinite(z_d))
                    z_l = z_l[keep]; z_d = z_d[keep]
                    n_initial = len(z_l)

                    if n_initial >= min_total:
                        s_raw, t_raw = self._ransac_affine_disparity(
                            z_l, z_d, n_initial)
                        # Temporal smoothing — batch jitter 흡수, wall ghosting 차단
                        batch_s, batch_t = self._smooth_affine(
                            s_raw, t_raw, n_inliers=n_initial)
                if batch_s != 1.0 or batch_t != 0.0:
                    depths = [self._apply_affine_disparity(d, batch_s, batch_t)
                              for d in depths]
            t_lidar = _time.perf_counter()

            batch_xyz: list[np.ndarray] = []
            batch_rgb: list[np.ndarray] = []
            for i in range(n):
                d = depths[i]
                K_i = K_list[i]
                # nvblox 입력
                self._publish_depth(image_msgs[i], d, K_i, camera_info_msgs[i])
                self._publish_point_cloud(image_msgs[i], d, K_i)
                # world 투영 — 항상 TF만 사용 (Gazebo ground truth).
                # DA3 refined extrinsics는 BA로 batch 사이 표류 → ghosting 원인이라
                # 의도적으로 무시. TF lookup 실패 시 frame skip.
                conf_i = (np.asarray(prediction.conf[i], dtype=np.float32)
                          if getattr(prediction, 'conf', None) is not None else None)
                sky_i = (np.asarray(prediction.sky[i])
                         if getattr(prediction, 'sky', None) is not None else None)
                xyz_rgb = self._project_frame_to_world(
                    d, K_i, image_msgs[i], rgbs[i],
                    conf=conf_i, sky=sky_i, T_world_camera=T_world_cam_i)
                if xyz_rgb is not None:
                    batch_xyz.append(xyz_rgb[0])
                    batch_rgb.append(xyz_rgb[1])

            self._publish_world_batch(batch_xyz, batch_rgb, image_msgs[-1].header.stamp)
            t_publish = _time.perf_counter()

            # 단계별 timing 로그 (어디가 느린지 정확히 보이게)
            with self._lock:
                buf_after = len(self._image_buffer)
            self.get_logger().info(
                f'[timing] decode={t_decode-t_start:.3f}s '
                f'tf={t_tf-t_decode:.3f}s '
                f'infer={t_infer-t_tf:.3f}s '
                f'lidar+affine={t_lidar-t_infer:.3f}s '
                f'publish={t_publish-t_lidar:.3f}s '
                f'TOTAL={t_publish-t_start:.3f}s '
                f'frames={n} buf_after={buf_after}')
        except Exception as exc:
            self.get_logger().error(f'DA3 inference failed: {exc}')
            self.get_logger().error(traceback.format_exc())
        finally:
            with self._lock:
                self._processing = False
            # 추론 중 buffer에 5장 이상 쌓였으면 즉시 다음 batch 시작 (back-to-back)
            self._maybe_trigger_inference()

    def _project_frame_to_world(
        self,
        depth: np.ndarray,
        intrinsics: np.ndarray,
        image_msg: Image,
        rgb: np.ndarray,
        conf: Optional[np.ndarray] = None,
        sky: Optional[np.ndarray] = None,
        T_world_camera: Optional[np.ndarray] = None,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """한 frame: depth → world frame XYZ + RGB. 실패 시 None.

        - T_world_camera 주어지면 그걸 사용 (DA3 prediction.extrinsics 등),
          없으면 TF lookup (image_msg.stamp 기준)
        - conf 임계값 이하 픽셀 제외
        - sky로 표시된 픽셀 제외
        - 3D 통계적 outlier (z-score 기반) 제외
        """
        stride = max(int(self.get_parameter('point_cloud_stride').value), 1)
        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)
        conf_min = float(self.get_parameter('min_confidence').value)

        fy = float(intrinsics[1, 1]); fx = float(intrinsics[0, 0])
        cx = float(intrinsics[0, 2]); cy = float(intrinsics[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            return None

        z = depth[::stride, ::stride]
        rows = np.arange(0, depth.shape[0], stride, dtype=np.float32)
        cols = np.arange(0, depth.shape[1], stride, dtype=np.float32)
        u, v = np.meshgrid(cols, rows)

        valid = np.isfinite(z) & (z > min_d) & (z < max_d)

        # conf 임계값 필터
        if conf is not None and conf_min > 0:
            c = conf[::stride, ::stride]
            valid &= c >= conf_min

        # sky 마스크 필터 (sky=True인 픽셀 제외)
        if sky is not None:
            s = sky[::stride, ::stride]
            valid &= ~s.astype(bool)

        if not np.any(valid):
            return None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        cam_pts = np.stack((x[valid], y[valid], z[valid]), axis=-1).astype(np.float32)

        rgb_resized = cv2.resize(
            rgb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_LINEAR)
        rgb_pts = rgb_resized[::stride, ::stride][valid].astype(np.uint8)

        if T_world_camera is not None:
            T = T_world_camera
        else:
            src_frame = self._source_frame_id(image_msg.header.frame_id)
            T = self._lookup_tf_matrix(src_frame, self._world_frame, image_msg.header.stamp)
            if T is None:
                return None
        R = T[:3, :3].astype(np.float32)
        t = T[:3, 3].astype(np.float32)
        world_pts = (cam_pts @ R.T) + t

        # 3D 통계적 outlier 제거 (depth z-score)
        if len(world_pts) > 100:
            z_med = np.median(world_pts[:, 2])
            z_mad = np.median(np.abs(world_pts[:, 2] - z_med)) + 1e-6
            z_score = np.abs(world_pts[:, 2] - z_med) / z_mad
            inlier = z_score < 5.0  # 5 MAD = 약 3.5 sigma
            world_pts = world_pts[inlier]
            rgb_pts = rgb_pts[inlier]

        return world_pts, rgb_pts

    def _ransac_affine_disparity(
        self, z_l: np.ndarray, z_d: np.ndarray, n_initial: int,
    ) -> tuple[float, float]:
        """RANSAC 2-DOF affine fit in DISPARITY space.

        모델: 1/d_metric = s × (1/d_pred) + t
        →    y_l = s × y_p + t  (y = 1/d, 즉 disparity)

        DA3는 MiDaS 계열 scale-shift invariant loss로 학습 → 출력 DOF가
        정확히 disparity 공간 affine 2개. 1-DOF (scale only)는 shift bias를
        흡수 못 해 거리별 잔차가 갈림 → 멀리/가까이 동시 정확 불가.

        Stratified sampling:
            - z_l을 거리 bin (1.5/3/5/7.5/11m)으로 나눠 bin당 2개씩 샘플
            - 원/근 균형 → s,t가 한쪽에 휘지 않음

        Inlier 판정 (depth space, 물리적 threshold):
            tol = lidar_res + noise_factor × z_l
            (1m → 6.5cm, 5m → 26.5cm, 10m → 51.5cm)

        Final: best inlier set에 2-DOF closed-form 재fit.
        """
        rng = np.random.default_rng()
        n = n_initial
        n_trials = 100
        lidar_res = 0.015
        noise_factor = 0.05

        # Disparity 변환
        y_l = 1.0 / z_l
        y_p = 1.0 / z_d

        # Stratified bins
        bin_edges = np.array([0.0, 1.5, 3.0, 5.0, 7.5, 11.0])
        bin_idx = np.digitize(z_l, bin_edges) - 1
        bin_lists = [np.where(bin_idx == k)[0]
                     for k in range(len(bin_edges) - 1)]
        nonempty = [b for b in bin_lists if len(b) > 0]
        use_strat = len(nonempty) >= 2

        best_count = 0
        best_mask = None
        best_s, best_t = 1.0, 0.0

        for _ in range(n_trials):
            if use_strat:
                idx_list = []
                for b in nonempty:
                    k = min(2, len(b))
                    idx_list.extend(rng.choice(b, size=k, replace=False))
                idx = np.array(idx_list)
            else:
                k = min(10, max(4, n // 50))
                idx = rng.choice(n, size=k, replace=False)
            if len(idx) < 3:
                continue

            # 2-DOF closed form LSQ in disparity space
            xs = y_p[idx]; ys = y_l[idx]
            N = len(idx)
            Sxx = float(np.sum(xs * xs))
            Sx = float(np.sum(xs))
            Sxy = float(np.sum(xs * ys))
            Sy = float(np.sum(ys))
            denom = N * Sxx - Sx * Sx
            if abs(denom) < 1e-12:
                continue
            s_trial = (N * Sxy - Sx * Sy) / denom
            t_trial = (Sy - s_trial * Sx) / N

            # Sanity: s in reasonable range, |t| < 1.0 1/m
            if not (0.1 < s_trial < 10.0) or abs(t_trial) > 1.0:
                continue

            # 모든 점에 적용 → depth space에서 물리적 inlier check
            disp_metric = s_trial * y_p + t_trial
            valid = disp_metric > 1e-3   # depth < 1000m
            if not np.any(valid):
                continue
            d_metric = np.where(valid, 1.0 / np.maximum(disp_metric, 1e-3), 1e6)
            residual = np.abs(d_metric - z_l)
            tol = lidar_res + noise_factor * z_l
            mask = (residual < tol) & valid
            count = int(np.sum(mask))
            if count > best_count:
                best_count = count
                best_mask = mask
                best_s, best_t = s_trial, t_trial

        min_pts = int(self.get_parameter('lidar_scale_min_points').value)
        if best_mask is None or best_count < min_pts:
            self.get_logger().warn(
                f'[batch_affine] RANSAC fail: best inliers={best_count}/{n_initial} '
                f'→ identity (s=1, t=0)')
            return 1.0, 0.0

        # Final 2-DOF refit on best inlier set
        xs = y_p[best_mask]; ys = y_l[best_mask]
        N = len(xs)
        Sxx = float(np.sum(xs * xs))
        Sx = float(np.sum(xs))
        Sxy = float(np.sum(xs * ys))
        Sy = float(np.sum(ys))
        denom = N * Sxx - Sx * Sx
        if abs(denom) > 1e-12:
            final_s = (N * Sxy - Sx * Sy) / denom
            final_t = (Sy - final_s * Sx) / N
        else:
            final_s, final_t = best_s, best_t
        final_s = float(np.clip(final_s, 0.1, 10.0))
        final_t = float(np.clip(final_t, -1.0, 1.0))

        zl_in = z_l[best_mask]
        self.get_logger().info(
            f'[batch_affine] s={final_s:.4f}, t={final_t:+.4f} (1/m) '
            f'inliers={best_count}/{n_initial} '
            f'({100*best_count/n_initial:.0f}%) '
            f'z_l=[{zl_in.min():.1f},{zl_in.max():.1f}]m')
        return final_s, final_t

    def _smooth_affine(
        self, s_raw: float, t_raw: float, n_inliers: int,
    ) -> tuple[float, float]:
        """EMA smoothing of (s, t). Inlier 적으면 갱신 스킵 (옛 값 유지).

        - 첫 fit: 그대로 채택 (warm start)
        - 이후: alpha = 0.3 기본 → 새 batch 30% + 옛 70%
        - n_inliers < min_inliers: skip update (RANSAC 신뢰 낮음)
        """
        min_in = int(self.get_parameter('affine_smooth_min_inliers').value)
        if n_inliers < min_in:
            self.get_logger().warn(
                f'[smooth] inliers={n_inliers} < {min_in} → skip update '
                f'(keep s={self._s_smooth:.4f}, t={self._t_smooth:+.4f})')
            return self._s_smooth, self._t_smooth
        if not self._affine_initialized:
            self._s_smooth = s_raw
            self._t_smooth = t_raw
            self._affine_initialized = True
            self.get_logger().info(
                f'[smooth] init s={s_raw:.4f}, t={t_raw:+.4f}')
            return s_raw, t_raw
        a = float(self.get_parameter('affine_smooth_alpha').value)
        s_new = (1.0 - a) * self._s_smooth + a * s_raw
        t_new = (1.0 - a) * self._t_smooth + a * t_raw
        ds = abs(s_new - self._s_smooth)
        dt = abs(t_new - self._t_smooth)
        self._s_smooth = s_new
        self._t_smooth = t_new
        self.get_logger().info(
            f'[smooth] s_raw={s_raw:.4f}→s={s_new:.4f} (Δ{ds:.4f}) '
            f't_raw={t_raw:+.4f}→t={t_new:+.4f} (Δ{dt:.4f})')
        return s_new, t_new

    def _apply_affine_disparity(
        self, depth: np.ndarray, s: float, t: float,
    ) -> np.ndarray:
        """1/d_metric = s × (1/d_pred) + t  →  d_metric.

        - 원본 depth ≤ 0 또는 inf인 픽셀은 0으로
        - 결과 disparity ≤ 1e-3 (depth > 1000m)인 픽셀도 0으로 (무효)
        """
        out = np.zeros_like(depth, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 1e-6)
        if not np.any(valid):
            return out
        disp_pred = np.where(valid, 1.0 / np.maximum(depth, 1e-6), 0.0)
        disp_metric = s * disp_pred + t
        final_valid = valid & (disp_metric > 1e-3)
        out[final_valid] = (1.0 / disp_metric[final_valid]).astype(np.float32)
        return out

    def _collect_lidar_pairs(
        self,
        depth: np.ndarray,
        intrinsics: np.ndarray,
        image_msg: Image,
        scan: LaserScan,
        conf: Optional[np.ndarray] = None,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """라이다 점을 frame 이미지에 투영 → 매칭된 (z_lidar, z_da3) 배열 반환.

        - conf 임계값 낮은 픽셀 제외 (DA3 신뢰도)
        - depth edge (gradient 큰 곳) 제외 — DA3 hallucination 방지
        - scale은 여기서 안 구함; 호출처가 batch 전체 pair를 모아 한 번에 fit.
        매칭점 부족하면 None.
        """
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not np.any(valid):
            return None

        r = ranges[valid]; a = angles[valid]
        lidar_pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=-1)

        cam_frame = self._source_frame_id(image_msg.header.frame_id)
        cam_pts = self._transform_points(
            lidar_pts, scan.header.frame_id, cam_frame, image_msg.header.stamp)
        if cam_pts is None:
            return None

        z_cam = cam_pts[:, 2]
        front = z_cam > 0.1
        if not np.any(front):
            return None
        cam_pts = cam_pts[front]; z_cam = z_cam[front]

        fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
        cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
        u = (cam_pts[:, 0] * fx / z_cam + cx).astype(int)
        v = (cam_pts[:, 1] * fy / z_cam + cy).astype(int)
        h, w = depth.shape
        in_image = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        if not np.any(in_image):
            return None
        u, v, z_cam = u[in_image], v[in_image], z_cam[in_image]

        da3_d = depth[v, u]
        valid_d = (da3_d > 0.0) & np.isfinite(da3_d)

        # Confidence 임계
        conf_min = float(self.get_parameter('min_confidence').value)
        if conf is not None and conf_min > 0:
            valid_d &= (conf[v, u] >= conf_min)

        # Depth edge 필터 — Sobel gradient 큰 픽셀 (depth 불연속)은 신뢰도 낮음
        try:
            gx = cv2.Sobel(depth, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(depth, cv2.CV_32F, 0, 1, ksize=3)
            grad = np.sqrt(gx * gx + gy * gy)
            # gradient 픽셀당 (m / px) 단위. depth ~3m면 0.05 ~ 0.2 정도 정상
            edge_thresh = max(0.5, 0.1 * float(np.median(np.abs(depth[depth > 0]))))
            valid_d &= (grad[v, u] < edge_thresh)
        except Exception:
            pass

        if np.sum(valid_d) < 1:
            return None
        return z_cam[valid_d].astype(np.float32), da3_d[valid_d].astype(np.float32)

    def _build_extrinsics_batch(self, image_msgs: list) -> Optional[np.ndarray]:
        """각 frame의 world ← camera_optical 4x4 행렬 (N, 4, 4)을 만들어 반환.

        하나라도 TF lookup 실패하면 None 반환 → DA3가 internal estimate 사용.
        성공 시 DA3 inference에 extrinsics로 전달 → multi-view consistency 강화.
        """
        mats = []
        for msg in image_msgs:
            src = self._source_frame_id(msg.header.frame_id)
            T = self._lookup_tf_matrix(src, self._world_frame, msg.header.stamp)
            if T is None:
                return None
            mats.append(T)
        return np.stack(mats, axis=0).astype(np.float32)

    def _lookup_tf_matrix(self, source: str, target: str, stamp) -> Optional[np.ndarray]:
        try:
            tf = self._tf_buffer.lookup_transform(
                target, source, Time.from_msg(stamp),
                timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        q = tf.transform.rotation
        tr = tf.transform.translation
        R = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = (tr.x, tr.y, tr.z)
        return T

    def _publish_world_batch(self, xyz_list: list, rgb_list: list, stamp) -> None:
        """현재 batch (최대 5 frame)의 world XYZ+RGB를 1개 PointCloud2로 publish.

        누적 X — 매 batch마다 새 메시지로 교체.
        """
        if not xyz_list:
            return
        xyz = np.concatenate(xyz_list, axis=0)
        rgb = np.concatenate(rgb_list, axis=0)

        # batch 내 voxel down-sample (인접 frame 간 중복 제거)
        if self._world_voxel > 0 and len(xyz) > 5000:
            keys = np.floor(xyz / self._world_voxel).astype(np.int64)
            _, idx = np.unique(
                keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791,
                return_index=True)
            xyz = xyz[idx]
            rgb = rgb[idx]

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self._world_frame
        msg.height = 1
        msg.width = int(xyz.shape[0])
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 16
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        buf = np.zeros((xyz.shape[0], 4), dtype=np.float32)
        buf[:, 0:3] = xyz.astype(np.float32)
        r, g, b = (rgb[:, 0].astype(np.uint32),
                   rgb[:, 1].astype(np.uint32),
                   rgb[:, 2].astype(np.uint32))
        packed = (r << 16) | (g << 8) | b
        buf[:, 3] = packed.view(np.float32)
        msg.row_step = msg.point_step * msg.width
        msg.data = buf.tobytes()
        self._world_pc_pub.publish(msg)

    def _apply_lidar_scale(
        self,
        depth: np.ndarray,
        intrinsics: np.ndarray,
        image_msg: Image,
        scan: LaserScan,
    ) -> np.ndarray:
        """라이다 스캔을 카메라 이미지에 투영해 DA3 depth의 metric scale을 복원."""
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not np.any(valid):
            return depth

        # 라이다 로컬 프레임에서 3D 점 (z=0: 2D 스캐너)
        r = ranges[valid]
        a = angles[valid]
        lidar_pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=-1)

        # 카메라 optical 프레임으로 변환
        cam_frame = self._source_frame_id(image_msg.header.frame_id)
        cam_pts = self._transform_points(lidar_pts, scan.header.frame_id, cam_frame, image_msg.header.stamp)
        if cam_pts is None:
            return depth

        # 카메라 앞쪽 점만 사용
        z_cam = cam_pts[:, 2]
        front = z_cam > 0.1
        if not np.any(front):
            return depth
        cam_pts = cam_pts[front]
        z_cam = z_cam[front]

        # 이미지 평면에 투영
        fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
        cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
        u = (cam_pts[:, 0] * fx / z_cam + cx).astype(int)
        v = (cam_pts[:, 1] * fy / z_cam + cy).astype(int)

        h, w = depth.shape
        in_image = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        if not np.any(in_image):
            return depth
        u, v, z_cam = u[in_image], v[in_image], z_cam[in_image]

        # DA3 depth와 라이다 GT 비교
        da3_d = depth[v, u]
        valid_d = da3_d > 0.0
        if np.sum(valid_d) < int(self.get_parameter('lidar_scale_min_points').value):
            self.get_logger().warn(
                f'LiDAR scale: only {np.sum(valid_d)} valid points, skipping scale correction.'
            )
            return depth

        scale = float(np.median(z_cam[valid_d] / da3_d[valid_d]))
        self.get_logger().debug(f'LiDAR scale factor: {scale:.4f}')
        return depth * scale

    def _camera_info_to_matrix(self, camera_info_msg: CameraInfo) -> np.ndarray:
        return np.asarray(camera_info_msg.k, dtype=np.float32).reshape(3, 3)

    def _gt_intrinsics(self, camera_info_msg: CameraInfo, depth_shape: tuple[int, int]) -> np.ndarray:
        """GT camera_info를 depth 해상도에 맞게 스케일한 intrinsics."""
        height, width = depth_shape
        scale_x = float(width) / float(max(camera_info_msg.width, 1))
        scale_y = float(height) / float(max(camera_info_msg.height, 1))
        K = self._camera_info_to_matrix(camera_info_msg)
        K[0, 0] *= scale_x
        K[0, 2] *= scale_x
        K[1, 1] *= scale_y
        K[1, 2] *= scale_y
        return K

    def _needs_metric_scaling(self, prediction) -> bool:
        """Metric 후처리 필요 여부.

        DA3 prediction.is_metric이 일차 권위 (모델이 직접 알려줌).
        없으면 model_id 문자열 fallback. NESTED 같은 신모델도 자동 인식.
        """
        is_metric_attr = getattr(prediction, 'is_metric', None)
        if is_metric_attr is not None:
            return not bool(is_metric_attr)
        model_id = str(self.get_parameter('model_id').value).lower()
        return 'metric' in model_id

    def _apply_metric_scaling(self, depth: np.ndarray, intrinsics: np.ndarray) -> np.ndarray:
        focal = 0.5 * float(intrinsics[0, 0] + intrinsics[1, 1])
        return depth * (focal / 300.0)

    def _publish_depth(
        self,
        image_msg: Image,
        depth: np.ndarray,
        intrinsics: np.ndarray,
        rgb_camera_info: CameraInfo,
    ) -> None:
        frame_id = self._output_frame_id(image_msg.header.frame_id)
        header = Header(stamp=image_msg.header.stamp, frame_id=frame_id)

        depth_msg = self._bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        depth_msg.header = header
        self._depth_pub.publish(depth_msg)

        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.height = int(depth.shape[0])
        info_msg.width = int(depth.shape[1])
        info_msg.distortion_model = rgb_camera_info.distortion_model or 'plumb_bob'
        info_msg.d = [0.0] * 5
        k = intrinsics.astype(np.float64).flatten().tolist()
        info_msg.k = k
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [
            k[0], k[1], k[2], 0.0,
            k[3], k[4], k[5], 0.0,
            k[6], k[7], k[8], 0.0,
        ]
        self._depth_info_pub.publish(info_msg)

    def _publish_point_cloud(
        self,
        image_msg: Image,
        depth: np.ndarray,
        intrinsics: np.ndarray,
    ) -> None:
        stride = max(int(self.get_parameter('point_cloud_stride').value), 1)
        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)

        fy = float(intrinsics[1, 1])
        fx = float(intrinsics[0, 0])
        cx = float(intrinsics[0, 2])
        cy = float(intrinsics[1, 2])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn('Skipping point cloud publish because intrinsics are invalid.')
            return

        z = depth[::stride, ::stride]
        rows = np.arange(0, depth.shape[0], stride, dtype=np.float32)
        cols = np.arange(0, depth.shape[1], stride, dtype=np.float32)
        u, v = np.meshgrid(cols, rows)

        valid = np.isfinite(z) & (z > min_depth) & (z < max_depth)
        if not np.any(valid):
            return

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.stack((x[valid], y[valid], z[valid]), axis=-1).astype(np.float32)

        source_frame = self._source_frame_id(image_msg.header.frame_id)
        target_frame = str(self.get_parameter('point_cloud_frame').value).strip() or source_frame
        if target_frame != source_frame:
            points = self._transform_points(points, source_frame, target_frame, image_msg.header.stamp)
            if points is None:
                return

        header = Header(
            stamp=image_msg.header.stamp,
            frame_id=target_frame,
        )
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self._points_pub.publish(cloud_msg)

    def _source_frame_id(self, incoming_frame_id: str) -> str:
        default_frame = str(self.get_parameter('camera_frame').value).strip() or 'front_camera_optical_frame'
        return incoming_frame_id or default_frame

    def _output_frame_id(self, incoming_frame_id: str) -> str:
        return self._source_frame_id(incoming_frame_id)

    def _transform_points(
        self,
        points: np.ndarray,
        source_frame: str,
        target_frame: str,
        stamp,
    ) -> Optional[np.ndarray]:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Failed to transform {source_frame} -> {target_frame}: {exc}'
            )
            return None

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        rot = self._quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        trans = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return (points @ rot.T) + trans

    def _quat_to_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float32)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Da3DepthNode()
    # MultiThreadedExecutor — image/scan/camera_info/TF 콜백을 분리 쓰레드로.
    # SingleThreaded면 _on_image의 LK optical flow (~20-30ms)가 TF 콜백 starve →
    # 추론 thread에서 lookup 실패 → frame drop → nvblox 통합 누락.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
