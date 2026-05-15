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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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
        # 이미지 버퍼 최대 크기. 0 또는 음수면 input_views*4 (back-compat default).
        # 1이면 항상 최신 frame만 유지 (single-view 실시간 모드).
        self.declare_parameter('image_buffer_maxlen', 0)
        self.declare_parameter('point_cloud_stride', 4)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 20.0)
        self.declare_parameter('camera_frame', 'front_camera_optical_frame')
        self.declare_parameter('point_cloud_frame', 'front_camera_optical_frame')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lidar_scale', True)
        self.declare_parameter('lidar_scale_min_points', 5)
        # ── Per-chunk absolute scale (Floor plane primary) ──────────────
        # Floor plane: DA3 3D 포인트클라우드에서 floor plane 검출 → z=0 비교 → scale.
        #   Lidar 불필요. Camera height + floor geometry prior 사용.
        # Horizon bearing: lidar scan과 수평선 bearing 매칭 → scale.
        #   Cross-check 전용 (diag only, apply X).
        self.declare_parameter('floor_scale_as_primary', True)  # primary 적용
        self.declare_parameter('horizon_scale_enable', True)    # cross-check 계산만
        self.declare_parameter('floor_crosscheck_enable', True)
        # ── DA3-Streaming Stage 1: Inter-chunk Sim3 alignment ──────────
        # 매 chunk 추론 후, 이전 chunk와 공유되는 overlap frame들의 3D 포인트로
        # Umeyama Sim3 fit → 현재 chunk를 이전 chunk 좌표계로 정렬 → scale 일관성.
        # DA3-Streaming의 핵심 아이디어 (loop closure 제외).
        self.declare_parameter('interchunk_align_enable', True)
        self.declare_parameter('interchunk_max_residual_m', 0.30)  # Sim3 fit residual 상한
        self.declare_parameter('floor_camera_height_m', 0.5)      # base_link에서 카메라 높이
        self.declare_parameter('floor_true_z_m', 0.0)             # base_link에서 진짜 floor z
        self.declare_parameter('floor_max_z_search_m', 0.4)       # base_z < this 만 floor 후보
        self.declare_parameter('floor_plane_thickness_m', 0.08)   # plane inlier slab 두께
        self.declare_parameter('floor_min_inliers', 100)          # plane 검출 최소 점 수
        self.declare_parameter('floor_residual_threshold_m', 0.05) # plane 두께 > 이 값이면 reject
        self.declare_parameter('floor_stride', 4)                  # pixel 샘플 stride
        # ── Floor+Ceiling 합산 scale (camera height 가정 불필요) ─────────
        # Floor와 ceiling 두 평면을 동시에 검출 → 그 사이 거리(=방 높이)와 비교.
        # 카메라 높이 의존 X. 두 평면 다 잡혀야 사용. floor만 잡혔으면 fallback.
        self.declare_parameter('floor_ceiling_enable', True)
        self.declare_parameter('room_height_m', 2.5)               # 진짜 floor↔ceiling 거리
        self.declare_parameter('ceiling_min_z_m', 1.0)             # base_z > 이 값만 ceiling 후보
        self.declare_parameter('ceiling_min_inliers', 100)
        # ── Motion scale (DA3 BA pose vs TF) ─────────────────────────────
        # DA3가 multi-view BA로 추정한 카메라 이동 거리 ‖t_DA3‖와
        # TF c2w 기반 실제 이동 거리 ‖t_TF‖ 비교 → scale = ‖t_TF‖ / ‖t_DA3‖.
        # 정지 중 (baseline 작음)이면 SNR 낮아 skip.
        self.declare_parameter('motion_scale_enable', True)
        self.declare_parameter('motion_scale_min_baseline_m', 0.05)
        # Scale 허용 범위
        self.declare_parameter('scale_min', 0.3)
        self.declare_parameter('scale_max', 3.0)
        self.declare_parameter('scale_smooth_alpha', 0.3)
        # 기존 ICP 파라미터는 유지 (코드 다른 곳 참조 가능)
        self.declare_parameter('icp_residual_threshold_m', 0.10)
        self.declare_parameter('icp_lidar_z_m', 0.1)
        self.declare_parameter('icp_slice_slab_m', 0.08)
        self.declare_parameter('icp_min_points', 20)
        self.declare_parameter('icp_stride', 4)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_max_yaw_deg', 30.0)
        self.declare_parameter('icp_max_translation_m', 1.0)
        # Keyframe 선별 (VGGT 패턴: 충분히 움직인 frame만 큐에 적재)
        self.declare_parameter('keyframe_min_disparity', 30.0)
        self.declare_parameter('keyframe_max_buffer_age_s', 5.0)

        # ── L1: Frame-level rejection ────────────────────────────────────
        # DISABLED. 30%/10% 모두 over-kill. RANSAC fit + smoothing이 이미 batch jitter
        # 흡수. Frame 통째 버리는 것보다 smooth affine 유지 쪽이 덜 해로움.
        # 0.0 = 항상 통과. 문제 재발견 시 재활성화 (신중하게).
        self.declare_parameter('min_inlier_pct', 0.0)

        # ── L3-A: Temporal voxel consistency ─────────────────────────────
        # DISABLED. stride 8 block masking이 좋은 pixel 64개씩 zero-out →
        # point cloud/mesh 잘림. 원래 의도는 일관성 없는 관측 제거였지만
        # nvblox TSDF의 weighted averaging이 이미 이걸 하고 있음.
        self.declare_parameter('temporal_check_enable', False)
        self.declare_parameter('temporal_voxel_size_m', 0.1)
        self.declare_parameter('temporal_history_len', 10)
        self.declare_parameter('temporal_reject_ratio', 0.25)  # 25% 이상 차이면 reject
        self.declare_parameter('temporal_min_history', 3)     # 최소 3회 관측 후부터 체크
        self.declare_parameter('temporal_max_voxels', 300000) # 메모리 상한

        # ── L3-B: ESDF-based residual check (DISABLED by default) ──────
        # 현재 nvblox ESDF는 2D slice (esdf_slice_height=0.5m 한 층)이라
        # 3D reference로 쓸 수 없음. 이걸 쓰면 robot 높이 근방 얇은 band 만 통과 →
        # point cloud가 10-50cm z band로 잘리고 mesh 생성 불가.
        # 3D ESDF (esdf_2d: false) 또는 mesh vertices를 reference로 전환해야 재활성.
        self.declare_parameter('esdf_check_enable', False)
        self.declare_parameter('esdf_reference_topic', '/nvblox_node/static_esdf_pointcloud')
        self.declare_parameter('esdf_max_residual_m', 0.25)
        self.declare_parameter('esdf_min_reference_points', 500)
        # 누적 world-frame RGB pointcloud
        self.declare_parameter('world_pointcloud_topic', '/camera/depth/world_points')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('world_pc_max_points', 500000)
        self.declare_parameter('world_pc_voxel_size', 0.05)
        # ── Global accumulated point cloud (webSocket 친화 delta publishing) ──
        # Raw point cloud 누적 (voxel dedup 없음). 오래된 점부터 rolling drop으로 메모리 관리.
        # Delta: 신규 chunk의 점들만 publish (작음, 자주).
        # Full: 전체 누적 점 주기적 snapshot (늦게 붙은 client sync용).
        self.declare_parameter('global_map_enable', True)
        self.declare_parameter('global_map_topic', '/camera/depth/global_map')
        self.declare_parameter('global_map_delta_topic', '/camera/depth/global_map_delta')
        self.declare_parameter('global_map_full_period_s', 10.0)   # full snapshot 주기
        self.declare_parameter('global_map_delta_period_s', 1.0)   # delta publish 주기
        self.declare_parameter('global_map_max_points', 2000000)   # 메모리 상한 (~32MB)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._input_views = max(int(self.get_parameter('input_views').value), 1)
        # Keyframe 버퍼 — disparity 통과 frame 적재.
        # maxlen 파라미터 우선, 0/음수면 input_views*4 (추론 중 쌓이는 거 흡수).
        # trigger 조건 `len >= input_views` 유지를 위해 하한 = input_views.
        _buf_param = int(self.get_parameter('image_buffer_maxlen').value)
        _buf_maxlen = _buf_param if _buf_param > 0 else self._input_views * 4
        _buf_maxlen = max(_buf_maxlen, self._input_views)
        self._image_buffer: deque[tuple[Image, CameraInfo]] = deque(maxlen=_buf_maxlen)
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
        # ── Edge bleeding filter (DA3 single-view artifact 제거) ─────
        # 인접 pixel 간 depth 차이 > threshold (m) → 실루엣 경계에서 생긴
        # floating 점으로 간주하여 제거. 0 = disable.
        # 0.3 = 30cm 차이. 정상 벽은 연속적이라 영향 없음.
        self.declare_parameter('edge_gradient_threshold_m', 0.3)

        # ── Isolated point removal (Radius Outlier) ─────────────────
        # world_points 3D 공간에서 반경 내 이웃 수 적은 point = isolated → 제거.
        # 허공에 홀로 떠있는 잘못된 관측 정리. nvblox 입력엔 미적용 (viz 전용).
        # enable=False로 끄고 싶을 때.
        self.declare_parameter('outlier_filter_enable', True)
        self.declare_parameter('outlier_radius_m', 0.15)       # 15cm 반경 내
        self.declare_parameter('outlier_min_neighbors', 3)     # 최소 3개 이웃

        # Temporal smoothing of (s, t) — batch별 scale jitter 흡수
        # alpha 작을수록 smooth ↑ (변화 둔감, 안정적)
        # alpha 크면 새 측정 빨리 반영하지만 jitter 그대로
        # 0.3 = 새 batch가 30% 영향, 옛 값 70% 유지
        self.declare_parameter('affine_smooth_alpha', 0.3)
        self.declare_parameter('affine_smooth_min_inliers', 20)
        self._s_smooth = 1.0
        self._t_smooth = 0.0
        self._affine_initialized = False

        # ── Scale state (floor plane 방식) ──────────────────────────────
        self._scale_smooth = 1.0
        self._scale_initialized = False
        self._last_floor_z_m = 0.0          # DA3 검출 floor plane z (base_link)
        self._last_floor_thickness_m = 0.0  # plane 두께 (residual)
        self._last_floor_inliers = 0
        self._total_floor_rejects = 0
        # Cross-check: horizon bearing scale
        self._last_scale_floor = 1.0
        self._last_scale_horizon = 1.0
        self._last_horizon_matched = 0

        # ── DA3-Streaming Stage 1: Inter-chunk Sim3 state ───────────────
        # 이전 chunk의 overlap frame들의 world XYZ (frame_key → (N, 3) numpy).
        self._prev_chunk_xyz_by_frame: dict = {}
        self._chunk_index = 0
        self._last_interchunk_scale = 1.0
        self._last_interchunk_residual = 0.0
        self._last_interchunk_corr = 0
        self._cumulative_scale = 1.0  # 초기 chunk 대비 누적 scale

        # ── Global accumulated point cloud state (raw, voxel dedup 없음) ─
        # List of (xyz (N,3) float32, rgb (N,3) uint8) tuples — chunk 단위 저장.
        # max_points 초과 시 앞(오래된 것)부터 drop. Delta는 last delta 이후 추가분.
        self._global_map_lock = threading.Lock()
        self._global_chunks_xyz: list = []   # list of (N, 3) float32
        self._global_chunks_rgb: list = []   # list of (N, 3) uint8
        self._global_new_xyz: list = []      # delta buffer (since last delta publish)
        self._global_new_rgb: list = []
        self._global_total_points = 0
        self._global_last_full_pub_wall = 0.0
        self._global_delta_pub_count = 0
        self._global_full_pub_count = 0
        # 이전 ICP 지표들 (호환용, 사용 안 함)
        self._last_icp_residual_m = float('inf')
        self._last_icp_yaw_deg = 0.0
        self._last_icp_tx = 0.0
        self._last_icp_ty = 0.0
        self._last_icp_matched = 0
        self._total_icp_rejects = 0

        # ── L3-A: temporal voxel history ─────────────────────────────────
        # key = 정수 hash of (vx, vy, vz), value = deque of recent depths
        self._voxel_history_lock = threading.Lock()
        self._voxel_depth_history: dict[int, deque] = {}
        self._temporal_rejected_pixels = 0  # diagnostic 카운터
        self._temporal_total_pixels = 0

        # ── L3-B: ESDF reference cloud (from nvblox) ────────────────────
        # ref cloud를 받으면 KDTree 구축, query에 사용
        self._esdf_ref_lock = threading.Lock()
        self._esdf_ref_xyz: Optional[np.ndarray] = None  # (N, 3)
        self._esdf_ref_tree = None  # scipy.spatial.cKDTree
        self._esdf_rejected_pixels = 0
        self._esdf_total_pixels = 0
        # 상위에서 frame 통째로 reject 되었는지 카운트
        self._total_frames_rejected = 0

        # ── Diagnostics state ─────────────────────────────────────────────
        # sliding window(최근 N초)로 각 rate 계산. 타임스탬프 deque로 간단히.
        import time as _time_import
        self._wall_time = _time_import.time
        self._cam_ts: deque[float] = deque(maxlen=200)       # camera msg 도착 시각
        self._kf_accept_ts: deque[float] = deque(maxlen=50)  # keyframe으로 채택된 시각
        self._chunk_ts: deque[float] = deque(maxlen=50)      # chunk 추론 완료 시각
        self._last_inference_wall = 0.0
        self._last_infer_ms = 0.0
        self._last_tf_ms = 0.0
        self._last_publish_ms = 0.0
        self._last_lidar_inliers_pct = 0.0
        self._last_lidar_inliers_n = 0
        self._total_chunks = 0
        self._total_errors = 0
        self._last_error_msg = ""

        self._torch = None
        self._model = None
        self._infer_device = None
        # _project_frame_to_world 단계별 timing accumulator (chunk마다 reset)
        self._proj_acc = {'gpu': 0.0, 'd2h': 0.0, 'kdtree': 0.0, 'zscore': 0.0}
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

        # BEST_EFFORT로 publish: Foxglove 등 느린 subscriber 때문에 publish()가
        # block되는 문제 방지 (관측: 24 chunk 후 send queue full → 전 스레드 stall).
        # depth/pointcloud는 실시간 센서 데이터라 drop 허용 가능.
        # nvblox subscriber도 BEST_EFFORT로 받고 있음 → compatibility OK.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._depth_pub = self.create_publisher(Image, depth_topic, output_qos)
        self._depth_info_pub = self.create_publisher(CameraInfo, depth_info_topic, output_qos)
        self._points_pub = self.create_publisher(PointCloud2, point_cloud_topic, output_qos)
        # 누적 world-frame RGB cloud
        world_pc_topic = str(self.get_parameter('world_pointcloud_topic').value)
        self._world_pc_pub = self.create_publisher(PointCloud2, world_pc_topic, output_qos)
        # Rejected chunk 전용 world cloud (디버깅/관찰용).
        # nvblox는 accepted(_world_pc_pub 토픽 아닌 /camera/depth/image_raw)만 받음.
        # 이 토픽은 Foxglove에서 "DA3가 이런 이상한 depth를 냈다"를 눈으로 확인용.
        world_pc_rejected_topic = world_pc_topic + '_rejected'
        self._world_pc_rejected_pub = self.create_publisher(
            PointCloud2, world_pc_rejected_topic, output_qos)

        # Global accumulated map (webSocket 친화: delta + periodic full)
        if bool(self.get_parameter('global_map_enable').value):
            gm_topic = str(self.get_parameter('global_map_topic').value)
            gm_delta_topic = str(self.get_parameter('global_map_delta_topic').value)
            # Full은 TRANSIENT_LOCAL + RELIABLE (late-join client에도 마지막 snapshot 전달)
            gm_full_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._global_map_full_pub = self.create_publisher(
                PointCloud2, gm_topic, gm_full_qos)
            # Delta는 BEST_EFFORT (작은 메시지 자주)
            self._global_map_delta_pub = self.create_publisher(
                PointCloud2, gm_delta_topic, output_qos)
            # Timers
            delta_period = float(self.get_parameter('global_map_delta_period_s').value)
            full_period = float(self.get_parameter('global_map_full_period_s').value)
            self.create_timer(delta_period, self._publish_global_map_delta)
            self.create_timer(full_period, self._publish_global_map_full)
        else:
            self._global_map_full_pub = None
            self._global_map_delta_pub = None
        self.create_subscription(Image, image_topic, self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)

        # L3-B: nvblox ESDF pointcloud 구독 (3D reference)
        if bool(self.get_parameter('esdf_check_enable').value):
            esdf_topic = str(self.get_parameter('esdf_reference_topic').value)
            self.create_subscription(PointCloud2, esdf_topic,
                                     self._on_esdf_reference, sensor_qos)
            self.get_logger().info(f'L3-B: subscribed to ESDF reference {esdf_topic}')

        # ── Diagnostics publisher ─────────────────────────────────────────
        # RELIABLE: Foxglove Diagnostics panel + aggregator 호환.
        # /diagnostics aggregate에 직접 publish (표준 ROS2 패턴).
        diag_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.RELIABLE)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', diag_qos)
        # 1Hz로 self health 발행. stall 시에도 계속 tick → Foxglove에서 "X초 동안 멈춤" 즉시 보임.
        self.create_timer(1.0, self._publish_diagnostics)

        # event-driven: timer 없음. _on_image에서 5장 채워지면 즉시 _trigger_inference.
        # 추론 끝난 직후에도 buffer 확인 (밀린 frame 처리).
        threading.Thread(target=self._load_model, daemon=True).start()

        self.get_logger().info(
            f'DA3 wrapper ready. image={image_topic}, info={camera_info_topic}, '
            f'scan={scan_topic}, depth={depth_topic}, points={point_cloud_topic}'
        )

    def _on_image(self, msg: Image) -> None:
        # Diagnostic: camera 수신 시각 기록 (camera_info 없어도 카운트)
        self._cam_ts.append(self._wall_time())
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
            self._kf_accept_ts.append(self._wall_time())
            self._maybe_trigger_inference()

    def _compute_mean_disparity(self, prev: np.ndarray, curr: np.ndarray) -> float:
        """Shi-Tomasi corners + LK optical flow 평균 픽셀 변위.

        실패 시맨틱 — 중요:
        - LK는 작은 motion 전제 (윈도우 21px). 큰 움직임엔 tracking 전부 실패.
        - 옛 버그: 실패 시 0 반환 → "움직임 없음"으로 오해석 → frame reject →
          `_last_kf_gray` 업데이트 안 됨 → 다음 frame은 더 오래된 것과 비교 →
          더 큰 motion → 더 많이 실패 → 영구 stuck (관측: 5분 후 추론 완전 정지).
        - 수정: 실패 = "큰 motion" → float('inf') 반환 → threshold 무조건 통과 →
          _last_kf_gray 업데이트 → 다음 frame은 방금 받은 것과 비교 → 복귀.
        """
        try:
            corners = cv2.goodFeaturesToTrack(
                prev, maxCorners=100, qualityLevel=0.01, minDistance=8)
            if corners is None or len(corners) < 5:
                return float('inf')  # 텍스처 부족 — 판단 불가 → accept
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                prev, curr, corners, None, winSize=(21, 21), maxLevel=2)
            if new_pts is None or status is None:
                return float('inf')  # LK API 실패 → accept
            ok = status.flatten() == 1
            if not np.any(ok):
                return float('inf')  # 모든 tracking 실패 = motion 너무 큼 → accept
            d = np.linalg.norm(new_pts[ok] - corners[ok], axis=-1)
            return float(np.median(d.ravel()))
        except Exception:
            return float('inf')  # 예외 = fallback accept (stall 예방)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._latest_camera_info = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    # ── L3-B: ESDF reference cloud callback ─────────────────────────────
    def _on_esdf_reference(self, msg: PointCloud2) -> None:
        """Nvblox의 ESDF pointcloud 수신 → KDTree 갱신. frame_id는 odom/world."""
        try:
            # read_points는 generator of tuples. float32 3D array로 collect.
            iter_pts = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts_list = [(p[0], p[1], p[2]) for p in iter_pts]
            if not pts_list:
                return
            pts = np.asarray(pts_list, dtype=np.float32)  # (N, 3)
            if pts.ndim != 2 or pts.shape[1] != 3:
                return
            min_pts = int(self.get_parameter('esdf_min_reference_points').value)
            if len(pts) < min_pts:
                return
            from scipy.spatial import cKDTree  # lazy import
            tree = cKDTree(pts)
            with self._esdf_ref_lock:
                self._esdf_ref_xyz = pts
                self._esdf_ref_tree = tree
        except Exception as exc:
            self.get_logger().debug(f'ESDF ref callback: {exc}')

    # ── L3-A: temporal voxel consistency ────────────────────────────────
    @staticmethod
    def _voxel_hash(vx: np.ndarray, vy: np.ndarray, vz: np.ndarray) -> np.ndarray:
        """정수 voxel 좌표를 int64 hash로. 같은 voxel에 대해 항상 같은 값."""
        # 73856093, 19349663, 83492791는 spatial hashing에 흔히 쓰는 큰 prime들
        return ((vx.astype(np.int64) * 73856093) ^
                (vy.astype(np.int64) * 19349663) ^
                (vz.astype(np.int64) * 83492791)).astype(np.int64)

    def _temporal_check_mask(self, xyz_world: np.ndarray, d_pred: np.ndarray) -> np.ndarray:
        """xyz_world: (N,3) DA3 world 좌표. d_pred: (N,) 해당 point의 DA3 depth.

        각 point가 속한 voxel의 과거 median과 비교. 편차 큰 point → False.
        그 후 통과한 point의 depth를 voxel history에 append.
        """
        if xyz_world.size == 0:
            return np.zeros(0, dtype=bool)
        vs = float(self.get_parameter('temporal_voxel_size_m').value)
        reject_ratio = float(self.get_parameter('temporal_reject_ratio').value)
        hist_len = int(self.get_parameter('temporal_history_len').value)
        min_hist = int(self.get_parameter('temporal_min_history').value)
        max_voxels = int(self.get_parameter('temporal_max_voxels').value)

        vx = np.floor(xyz_world[:, 0] / vs).astype(np.int64)
        vy = np.floor(xyz_world[:, 1] / vs).astype(np.int64)
        vz = np.floor(xyz_world[:, 2] / vs).astype(np.int64)
        keys = self._voxel_hash(vx, vy, vz)

        mask = np.ones(len(d_pred), dtype=bool)
        with self._voxel_history_lock:
            # 메모리 상한 도달 시 통째로 절반 drop (간단 LRU 근사)
            if len(self._voxel_depth_history) > max_voxels:
                # Python 3.7+ dict는 insertion-order 유지 → 앞쪽이 오래된 것
                drop_n = len(self._voxel_depth_history) // 2
                for k in list(self._voxel_depth_history.keys())[:drop_n]:
                    del self._voxel_depth_history[k]

            for idx in range(len(d_pred)):
                k = int(keys[idx])
                hist = self._voxel_depth_history.get(k)
                d_new = float(d_pred[idx])
                if hist is not None and len(hist) >= min_hist:
                    med = float(np.median(hist))
                    if med > 0.1:
                        if abs(d_new - med) / med > reject_ratio:
                            mask[idx] = False
                            continue
                # 통과한 (또는 초기 관측) point를 history에 추가
                if hist is None:
                    hist = deque(maxlen=hist_len)
                    self._voxel_depth_history[k] = hist
                hist.append(d_new)

        return mask

    def _esdf_residual_mask(self, xyz_world: np.ndarray) -> np.ndarray:
        """각 point의 ESDF reference 최근접 거리. threshold 초과 → False."""
        with self._esdf_ref_lock:
            tree = self._esdf_ref_tree
        if tree is None or xyz_world.size == 0:
            return np.ones(len(xyz_world), dtype=bool)  # reference 없으면 skip
        thr = float(self.get_parameter('esdf_max_residual_m').value)
        dists, _ = tree.query(xyz_world, k=1, workers=1)
        return dists < thr

    def _apply_3d_consistency_masks(
        self, depth: np.ndarray, K: np.ndarray,
        T_world_cam: np.ndarray, stride: int = 4,
        use_temporal: bool = True, use_esdf: bool = True,
    ) -> np.ndarray:
        """L3-A + L3-B 통합 mask.

        stride 샘플링 → world XYZ → temporal voxel check + ESDF residual check
        → 통과 못한 샘플의 stride×stride block을 depth에서 0으로 만듦.
        """
        H, W = depth.shape
        stride = max(1, int(stride))
        z = depth[::stride, ::stride]
        H_s, W_s = z.shape

        fx = float(K[0, 0]); fy = float(K[1, 1])
        cx = float(K[0, 2]); cy = float(K[1, 2])
        if fx <= 0 or fy <= 0:
            return depth

        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)
        valid = np.isfinite(z) & (z > min_d) & (z < max_d)
        if not np.any(valid):
            return depth

        rows = np.arange(0, H, stride, dtype=np.float32)[:H_s]
        cols = np.arange(0, W, stride, dtype=np.float32)[:W_s]
        u, v = np.meshgrid(cols, rows)

        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        pts_cam = np.stack([x_cam.ravel(), y_cam.ravel(), z.ravel()], axis=-1)  # (N_s, 3)

        # camera → world: T_world_cam (c2w)
        R = T_world_cam[:3, :3]
        t = T_world_cam[:3, 3]
        pts_world = (pts_cam @ R.T) + t

        valid_flat = valid.ravel()
        mask_flat = np.ones(valid_flat.size, dtype=bool)
        mask_flat[~valid_flat] = False

        if np.any(valid_flat):
            pts_v = pts_world[valid_flat]
            z_v = z.ravel()[valid_flat]
            m = np.ones(len(z_v), dtype=bool)
            if use_temporal:
                m_t = self._temporal_check_mask(pts_v, z_v)
                m &= m_t
                self._temporal_total_pixels += len(z_v)
                self._temporal_rejected_pixels += int(np.sum(~m_t))
            if use_esdf:
                m_e = self._esdf_residual_mask(pts_v)
                m &= m_e
                self._esdf_total_pixels += len(z_v)
                self._esdf_rejected_pixels += int(np.sum(~m_e))
            # valid 영역의 mask 업데이트
            idx = np.flatnonzero(valid_flat)
            mask_flat[idx] = m

        mask_2d = mask_flat.reshape(H_s, W_s)
        # stride×stride block으로 upsample — 샘플 실패 블록 전체 zero-out
        mask_full = np.repeat(np.repeat(mask_2d, stride, axis=0), stride, axis=1)[:H, :W]
        return np.where(mask_full, depth, 0.0).astype(depth.dtype)

    # ── Floor plane scale estimation ───────────────────────────────────
    def _estimate_scale_from_floor(
        self, depth: np.ndarray, K: np.ndarray, T_base_cam: np.ndarray,
    ) -> tuple[Optional[float], float, int]:
        """DA3 pointcloud에서 floor plane 검출 → scale factor 계산.

        수학:
            진짜 floor 픽셀: base_z_DA3 = 0.5*(1-α)  (카메라 높이 0.5m, floor z=0 전제)
            → α = 1 - 2*z_plane_DA3
            → scale_to_multiply_depth = 1/α
        scale_to_apply * d_da3 = d_true

        Returns: (scale or None if detection fail, thickness_m, n_inliers)
        """
        stride = max(1, int(self.get_parameter('floor_stride').value))
        cam_h = float(self.get_parameter('floor_camera_height_m').value)
        floor_true_z = float(self.get_parameter('floor_true_z_m').value)
        max_z_search = float(self.get_parameter('floor_max_z_search_m').value)
        thickness = float(self.get_parameter('floor_plane_thickness_m').value)
        min_inliers = int(self.get_parameter('floor_min_inliers').value)
        s_min = float(self.get_parameter('scale_min').value)
        s_max = float(self.get_parameter('scale_max').value)
        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)

        H, W = depth.shape
        fx = float(K[0, 0]); fy = float(K[1, 1])
        cx = float(K[0, 2]); cy = float(K[1, 2])
        if fx <= 0 or fy <= 0:
            return None, 0.0, 0

        # image 하반부 (v > cy)만 사용: floor 후보는 카메라 아래를 보는 픽셀
        v_start = int(cy)  # 이미지 중앙선부터 아래
        z_s = depth[v_start::stride, ::stride]
        if z_s.size == 0:
            return None, 0.0, 0
        rows = np.arange(v_start, H, stride, dtype=np.float32)[:z_s.shape[0]]
        cols = np.arange(0, W, stride, dtype=np.float32)[:z_s.shape[1]]
        u, v = np.meshgrid(cols, rows)

        valid = np.isfinite(z_s) & (z_s > min_d) & (z_s < max_d)
        if not np.any(valid):
            return None, 0.0, 0

        x_opt = (u[valid] - cx) * z_s[valid] / fx
        y_opt = (v[valid] - cy) * z_s[valid] / fy
        z_opt = z_s[valid]
        pts_opt = np.stack([x_opt, y_opt, z_opt], axis=-1).astype(np.float32)

        R = T_base_cam[:3, :3].astype(np.float32)
        t = T_base_cam[:3, 3].astype(np.float32)
        pts_base = pts_opt @ R.T + t
        base_z = pts_base[:, 2]

        # 카메라 아래 범위만 (z < max_z_search, 하지만 너무 낮은 건 artifact 제외)
        search_mask = (base_z < max_z_search) & (base_z > -max_z_search)
        if int(np.sum(search_mask)) < min_inliers:
            return None, 0.0, int(np.sum(search_mask))

        z_candidates = base_z[search_mask]
        # 초기 추정: median
        z_plane = float(np.median(z_candidates))
        # Refine: thickness 안 inlier들의 median
        for _ in range(3):  # 3회 반복 refine
            inlier_mask = np.abs(z_candidates - z_plane) < thickness
            if int(np.sum(inlier_mask)) < min_inliers:
                break
            z_plane = float(np.median(z_candidates[inlier_mask]))

        inlier_mask = np.abs(z_candidates - z_plane) < thickness
        n_inliers = int(np.sum(inlier_mask))
        if n_inliers < min_inliers:
            return None, 0.0, n_inliers

        # Residual: inlier들의 plane 두께 (MAD)
        residual = float(np.median(np.abs(z_candidates[inlier_mask] - z_plane)))

        # Scale 계산
        # base_z_DA3 = cam_h * (1 - α) + floor_true_z * α 가 일반 형태.
        # 우리 기본: floor_true_z = 0, cam_h = 0.5 → base_z = 0.5(1-α) → α = 1 - 2*z_plane
        alpha = (cam_h - z_plane) / (cam_h - floor_true_z)
        # → α = 1 - z_plane / 0.5 (floor_true_z=0, cam_h=0.5일 때)
        if alpha <= 0 or not np.isfinite(alpha):
            return None, residual, n_inliers
        scale_to_apply = 1.0 / alpha

        if not (s_min <= scale_to_apply <= s_max):
            return None, residual, n_inliers

        self._last_floor_z_m = z_plane
        return scale_to_apply, residual, n_inliers

    # ── Floor+Ceiling joint scale (방 높이 가정으로 chunk-independent) ────
    def _detect_horizontal_plane(
        self, depth: np.ndarray, K: np.ndarray, T_base_cam: np.ndarray,
        v_range: tuple[int, int],
        z_min: float, z_max: float,
        min_inliers: int,
        thickness: float,
    ) -> tuple[Optional[float], int]:
        """이미지 v 범위 내 픽셀의 base-frame 3D 점에서 horizontal plane 검출.

        z_min < z_plane < z_max 범위만 고려. floor (v_range=하반, z_min=음수)와
        ceiling (v_range=상반, z_min=양수) 검출 둘 다 커버.

        Returns: (z_plane in base frame, n_inliers) or (None, n_seen).
        """
        stride = max(1, int(self.get_parameter('floor_stride').value))
        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)

        H, W = depth.shape
        v0, v1 = v_range
        v0 = max(0, min(H, v0)); v1 = max(0, min(H, v1))
        if v1 <= v0:
            return None, 0
        fx = float(K[0, 0]); fy = float(K[1, 1])
        cx = float(K[0, 2]); cy = float(K[1, 2])
        if fx <= 0 or fy <= 0:
            return None, 0

        z_s = depth[v0:v1:stride, ::stride]
        if z_s.size == 0:
            return None, 0
        rows = np.arange(v0, v1, stride, dtype=np.float32)[:z_s.shape[0]]
        cols = np.arange(0, W, stride, dtype=np.float32)[:z_s.shape[1]]
        u_g, v_g = np.meshgrid(cols, rows)

        valid = np.isfinite(z_s) & (z_s > min_d) & (z_s < max_d)
        if not np.any(valid):
            return None, 0

        x_opt = (u_g[valid] - cx) * z_s[valid] / fx
        y_opt = (v_g[valid] - cy) * z_s[valid] / fy
        z_opt = z_s[valid]
        pts_opt = np.stack([x_opt, y_opt, z_opt], axis=-1).astype(np.float32)
        R = T_base_cam[:3, :3].astype(np.float32)
        t = T_base_cam[:3, 3].astype(np.float32)
        base_z = (pts_opt @ R.T + t)[:, 2]

        in_range = (base_z >= z_min) & (base_z <= z_max)
        if int(np.sum(in_range)) < min_inliers:
            return None, int(np.sum(in_range))

        z_cand = base_z[in_range]
        z_plane = float(np.median(z_cand))
        for _ in range(3):
            mask = np.abs(z_cand - z_plane) < thickness
            if int(np.sum(mask)) < min_inliers:
                break
            z_plane = float(np.median(z_cand[mask]))

        mask = np.abs(z_cand - z_plane) < thickness
        n_in = int(np.sum(mask))
        if n_in < min_inliers:
            return None, n_in
        return z_plane, n_in

    def _estimate_scale_from_floor_ceiling(
        self, depth: np.ndarray, K: np.ndarray, T_base_cam: np.ndarray,
    ) -> tuple[Optional[float], Optional[float], Optional[float], int, int]:
        """Floor와 ceiling 두 평면 검출 → 그 사이 거리로 scale.

        장점:
          - 카메라 높이 정확히 몰라도 됨
          - floor 단독 검출의 천장-혼동 outlier (z_DA3 > 0인 case) 자동 배제
          - 두 평면 다 잡혀야 통과 → 신뢰도 자체 검증

        수학:
          z_floor_DA3, z_ceil_DA3 (base frame, DA3 gauge)
          h_DA3 = z_ceil_DA3 - z_floor_DA3   (positive)
          h_true = room_height_m
          scale = h_true / h_DA3

        Returns: (scale, z_floor, z_ceil, n_floor, n_ceil) or (None, ...)
        """
        H, _ = depth.shape
        cam_h = float(self.get_parameter('floor_camera_height_m').value)
        thickness = float(self.get_parameter('floor_plane_thickness_m').value)
        min_in_floor = int(self.get_parameter('floor_min_inliers').value)
        min_in_ceil = int(self.get_parameter('ceiling_min_inliers').value)
        ceil_min_z = float(self.get_parameter('ceiling_min_z_m').value)
        max_z_search = float(self.get_parameter('floor_max_z_search_m').value)
        room_h = float(self.get_parameter('room_height_m').value)
        s_min = float(self.get_parameter('scale_min').value)
        s_max = float(self.get_parameter('scale_max').value)

        v_mid = int(K[1, 2])  # cy
        # Floor: 이미지 하반 + base_z 음수 ~ 0 근처
        # base_z 검색 범위: -max_z_search ~ 0.05 (천장 후보 배제하기 위해 윗쪽 제한)
        z_floor, n_floor = self._detect_horizontal_plane(
            depth, K, T_base_cam,
            v_range=(v_mid, H),
            z_min=-max_z_search, z_max=0.05,
            min_inliers=min_in_floor, thickness=thickness)
        # Ceiling: 이미지 상반 + base_z 양수 (cam 위)
        # ceil_min_z 부터 cam_h + room_h * 1.5 까지
        z_ceil, n_ceil = self._detect_horizontal_plane(
            depth, K, T_base_cam,
            v_range=(0, v_mid),
            z_min=ceil_min_z, z_max=cam_h + room_h * 1.5,
            min_inliers=min_in_ceil, thickness=thickness)

        if z_floor is None or z_ceil is None:
            return None, z_floor, z_ceil, n_floor, n_ceil

        h_da3 = z_ceil - z_floor
        if h_da3 <= 0.1:  # 두 평면이 너무 가까우면 의미 없음
            return None, z_floor, z_ceil, n_floor, n_ceil
        scale = room_h / h_da3
        if not (s_min <= scale <= s_max):
            return None, z_floor, z_ceil, n_floor, n_ceil
        return scale, z_floor, z_ceil, n_floor, n_ceil

    # ── Motion scale: DA3 BA pose translation vs TF c2w translation ─────
    def _estimate_scale_from_motion(
        self, prediction, extrinsics_c2w_tf: Optional[np.ndarray],
    ) -> tuple[Optional[float], float, float]:
        """DA3가 BA로 산출한 카메라 이동 거리 vs 실제 (TF) 이동 거리.

        - DA3 prediction.extrinsics: (N, 3, 4) w2c. cam_center = -R^T @ t.
        - TF c2w: (N, 4, 4). cam_center = T[:3, 3].
        - chunk 안 첫 frame과 마지막 frame 사이 baseline 비교.
        - 정지 중이면 baseline 너무 작아 SNR 낮음 → skip.

        Returns: (scale, baseline_tf_m, baseline_da3_m) or (None, 0, 0)
        """
        if extrinsics_c2w_tf is None or len(extrinsics_c2w_tf) < 2:
            return None, 0.0, 0.0
        ext_da3 = getattr(prediction, 'extrinsics', None)
        if ext_da3 is None or len(ext_da3) < 2:
            return None, 0.0, 0.0
        try:
            ext_da3 = np.asarray(ext_da3, dtype=np.float32)
        except Exception:
            return None, 0.0, 0.0

        # DA3 cam centers: w2c → cam center = -R^T @ t
        # ext shape이 (N, 3, 4)일 수도 (N, 4, 4)일 수도 → 통일
        if ext_da3.shape[-2:] == (4, 4):
            R_da3 = ext_da3[:, :3, :3]
            t_da3 = ext_da3[:, :3, 3]
        elif ext_da3.shape[-2:] == (3, 4):
            R_da3 = ext_da3[:, :3, :3]
            t_da3 = ext_da3[:, :3, 3]
        else:
            return None, 0.0, 0.0
        cam_da3 = -np.einsum('nij,nj->ni', R_da3.transpose(0, 2, 1), t_da3)
        baseline_da3 = float(np.linalg.norm(cam_da3[-1] - cam_da3[0]))

        cam_tf = extrinsics_c2w_tf[:, :3, 3]
        baseline_tf = float(np.linalg.norm(cam_tf[-1] - cam_tf[0]))

        min_bl = float(self.get_parameter('motion_scale_min_baseline_m').value)
        if baseline_tf < min_bl or baseline_da3 < 1e-4:
            return None, baseline_tf, baseline_da3
        scale = baseline_tf / baseline_da3
        s_min = float(self.get_parameter('scale_min').value)
        s_max = float(self.get_parameter('scale_max').value)
        if not (s_min <= scale <= s_max):
            return None, baseline_tf, baseline_da3
        return scale, baseline_tf, baseline_da3

    # ── Cross-check: Horizon bearing matching (scale 추정 교차검증용) ──
    def _estimate_scale_from_horizon_bearing(
        self, depth: np.ndarray, K: np.ndarray, T_base_cam: np.ndarray,
        scan: Optional[LaserScan],
    ) -> tuple[Optional[float], int]:
        """이미지 수평선 근처 픽셀의 bearing ↔ lidar bearing 매칭.

        수평선 픽셀(v ≈ cy)은 카메라 광축 수평 방향 ray → 벽까지 XY 거리가
        lidar reading과 같음 (벽은 height-invariant). bearing 매칭으로 range ratio
        = scale 독립적으로 얻음.

        Returns: (scale or None, matched_bins)
        """
        if scan is None:
            return None, 0
        lidar_xy = self._extract_lidar_xy_base(scan)
        if lidar_xy is None:
            return None, 0

        stride = max(1, int(self.get_parameter('floor_stride').value))
        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)
        H, W = depth.shape
        fx = float(K[0, 0]); fy = float(K[1, 1])
        cx = float(K[0, 2]); cy = float(K[1, 2])

        # Horizon band: v ≈ cy ± 10 pixel
        v_lo = max(0, int(cy) - 10)
        v_hi = min(H, int(cy) + 11)
        z_s = depth[v_lo:v_hi:stride, ::stride]
        if z_s.size == 0:
            return None, 0
        rows = np.arange(v_lo, v_hi, stride, dtype=np.float32)[:z_s.shape[0]]
        cols = np.arange(0, W, stride, dtype=np.float32)[:z_s.shape[1]]
        u, v = np.meshgrid(cols, rows)
        valid = np.isfinite(z_s) & (z_s > min_d) & (z_s < max_d)
        if not np.any(valid):
            return None, 0

        x_opt = (u[valid] - cx) * z_s[valid] / fx
        y_opt = (v[valid] - cy) * z_s[valid] / fy
        z_opt = z_s[valid]
        pts_opt = np.stack([x_opt, y_opt, z_opt], axis=-1).astype(np.float32)
        R = T_base_cam[:3, :3].astype(np.float32)
        t = T_base_cam[:3, 3].astype(np.float32)
        pts_base = pts_opt @ R.T + t
        da3_xy = pts_base[:, :2]

        # 카메라 위치 (base_link) = T_base_cam translation의 xy
        cam_xy = t[:2]

        # Lidar / DA3 bearings & ranges (camera 중심)
        def _bearing_range(xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            rel = xy - cam_xy
            return np.arctan2(rel[:, 1], rel[:, 0]), np.linalg.norm(rel, axis=1)
        bear_l, rng_l = _bearing_range(lidar_xy)
        bear_d, rng_d = _bearing_range(da3_xy)

        # 1° bin. 각 bin에서 DA3 min range (벽이 가장 가까운 것) vs lidar min range
        bin_deg = 2.0
        n_bins = int(360 / bin_deg)
        bin_edges = np.linspace(-np.pi, np.pi, n_bins + 1)
        lidar_min = np.full(n_bins, np.inf, dtype=np.float32)
        da3_min = np.full(n_bins, np.inf, dtype=np.float32)

        li_idx = np.clip(((bear_l + np.pi) / (2 * np.pi) * n_bins).astype(np.int32),
                         0, n_bins - 1)
        di_idx = np.clip(((bear_d + np.pi) / (2 * np.pi) * n_bins).astype(np.int32),
                         0, n_bins - 1)
        # vectorized min per bin
        for i in range(n_bins):
            if li_idx.size > 0:
                sel_l = rng_l[li_idx == i]
                if sel_l.size > 0:
                    lidar_min[i] = float(sel_l.min())
            if di_idx.size > 0:
                sel_d = rng_d[di_idx == i]
                if sel_d.size > 0:
                    da3_min[i] = float(sel_d.min())

        matched = np.isfinite(lidar_min) & np.isfinite(da3_min) & (lidar_min > 0.3) & (da3_min > 0.3)
        if int(np.sum(matched)) < 10:
            return None, int(np.sum(matched))
        ratios = lidar_min[matched] / da3_min[matched]
        scale_est = float(np.median(ratios))

        s_min = float(self.get_parameter('scale_min').value)
        s_max = float(self.get_parameter('scale_max').value)
        if not (s_min <= scale_est <= s_max):
            return None, int(np.sum(matched))
        return scale_est, int(np.sum(matched))

    # ── 2D ICP lidar scale helpers (legacy, 호환용 유지) ─────────────────
    def _extract_lidar_xy_base(self, scan: LaserScan) -> Optional[np.ndarray]:
        """LaserScan → (N, 2) 점 in base_link frame.

        XLeRobot stack에서 라이다가 base_link 기준 오프셋 + 회전 없음으로 들어온다고 가정
        → x,y 그대로 base_link 평면 좌표. (z=0.1 offset은 2D 슬라이스에 무관)
        """
        if scan is None or len(scan.ranges) == 0:
            return None
        angles = scan.angle_min + np.arange(len(scan.ranges), dtype=np.float32) * scan.angle_increment
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        valid = (np.isfinite(ranges)
                 & (ranges > float(scan.range_min))
                 & (ranges < float(scan.range_max)))
        if not np.any(valid):
            return None
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        return np.stack([xs, ys], axis=-1).astype(np.float32)

    def _extract_da3_slice_xy_base(
        self, depth: np.ndarray, K: np.ndarray, T_base_cam: np.ndarray,
    ) -> Optional[np.ndarray]:
        """DA3 depth → 3D base_link 좌표 → lidar 높이 slice → (M, 2).

        stride 샘플링으로 속도 확보. lidar_z ± slab 범위만 추출.
        """
        stride = max(1, int(self.get_parameter('icp_stride').value))
        lidar_z = float(self.get_parameter('icp_lidar_z_m').value)
        slab = float(self.get_parameter('icp_slice_slab_m').value)
        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)

        H, W = depth.shape
        fx = float(K[0, 0]); fy = float(K[1, 1])
        cx = float(K[0, 2]); cy = float(K[1, 2])
        if fx <= 0 or fy <= 0:
            return None

        z = depth[::stride, ::stride]
        rows = np.arange(0, H, stride, dtype=np.float32)[:z.shape[0]]
        cols = np.arange(0, W, stride, dtype=np.float32)[:z.shape[1]]
        u, v = np.meshgrid(cols, rows)
        valid = np.isfinite(z) & (z > min_d) & (z < max_d)
        if not np.any(valid):
            return None

        x_cam = (u[valid] - cx) * z[valid] / fx
        y_cam = (v[valid] - cy) * z[valid] / fy
        z_cam = z[valid]
        pts_cam = np.stack([x_cam, y_cam, z_cam], axis=-1).astype(np.float32)

        R = T_base_cam[:3, :3].astype(np.float32)
        t = T_base_cam[:3, 3].astype(np.float32)
        pts_base = pts_cam @ R.T + t

        z_mask = np.abs(pts_base[:, 2] - lidar_z) < slab
        if not np.any(z_mask):
            return None
        return pts_base[z_mask, :2]

    @staticmethod
    def _umeyama_2d_similarity_from_points(
        src: np.ndarray, tgt: np.ndarray,
    ) -> Optional[tuple[float, np.ndarray, np.ndarray]]:
        """Umeyama closed-form 2D similarity (source → target).

        최소 2쌍 대응 필요. degenerate (같은 점) 케이스 None 반환.
        """
        if len(src) < 2 or len(tgt) < 2:
            return None
        c_s = src.mean(axis=0)
        c_t = tgt.mean(axis=0)
        src_c = src - c_s
        tgt_c = tgt - c_t
        var_s = float(np.sum(src_c * src_c) / len(src_c))
        if var_s < 1e-10:
            return None
        H = (src_c.T @ tgt_c) / len(src_c)
        U, S_vec, Vt = np.linalg.svd(H)
        S_reflect = np.ones(2, dtype=np.float32)
        if np.linalg.det(U) * np.linalg.det(Vt) < 0:
            S_reflect[-1] = -1
        R = (U * S_reflect) @ Vt
        scale = float(np.sum(S_vec * S_reflect) / var_s)
        t_vec = c_t - scale * (R @ c_s)
        return scale, R.astype(np.float32), t_vec.astype(np.float32)

    def _icp_similarity_2d(
        self, source: np.ndarray, target: np.ndarray,
    ) -> tuple[float, np.ndarray, np.ndarray, float, bool]:
        """RANSAC-based 2D similarity fit (globally optimal, 초기값 의존 없음).

        ICP 반복 대신 무작위 2-쌍 sampling → Umeyama closed-form → inlier count.
        DA3 scale이 2-5배 어긋나도 수렴. 로컬 미니마 문제 원천 제거.

        Returns:
            scale, R, t, residual(m), converged
        """
        if source is None or target is None:
            return 1.0, np.eye(2), np.zeros(2), float('inf'), False
        min_pts = int(self.get_parameter('icp_min_points').value)
        if len(source) < min_pts or len(target) < min_pts:
            return 1.0, np.eye(2), np.zeros(2), float('inf'), False

        from scipy.spatial import cKDTree  # lazy
        tree = cKDTree(target)
        n_iter = int(self.get_parameter('icp_max_iterations').value) * 10  # 여유 있게 많이
        s_min = float(self.get_parameter('scale_min').value)
        s_max = float(self.get_parameter('scale_max').value)
        # inlier threshold = final residual threshold × 2 (초기 선별용 관대)
        inlier_thr = float(self.get_parameter('icp_residual_threshold_m').value) * 2.0

        rng = np.random.default_rng()
        best_inlier_count = 0
        best_params: tuple[float, np.ndarray, np.ndarray] = (1.0, np.eye(2, dtype=np.float32),
                                                             np.zeros(2, dtype=np.float32))

        src_len = len(source)
        tgt_len = len(target)

        for _ in range(n_iter):
            # 2쌍 무작위 대응
            if src_len < 2 or tgt_len < 2:
                break
            src_idx = rng.choice(src_len, 2, replace=False)
            tgt_idx = rng.choice(tgt_len, 2, replace=False)
            result = self._umeyama_2d_similarity_from_points(
                source[src_idx], target[tgt_idx])
            if result is None:
                continue
            s, R, t = result
            # scale sanity: 범위 밖은 즉시 skip
            if not (s_min <= s <= s_max):
                continue
            # 전체 source 변환 후 target nearest distance
            source_t = s * (source @ R.T) + t
            dists, _ = tree.query(source_t, k=1)
            inlier_count = int(np.sum(dists < inlier_thr))
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_params = (s, R, t)

        # 최종 refit: best에서 inlier 추려서 전체 Umeyama 재계산
        s_best, R_best, t_best = best_params
        source_t = s_best * (source @ R_best.T) + t_best
        dists, nn_idx = tree.query(source_t, k=1)
        inlier_mask = dists < inlier_thr
        n_inliers = int(np.sum(inlier_mask))
        if n_inliers >= max(10, int(0.05 * src_len)):
            refit = self._umeyama_2d_similarity_from_points(
                source[inlier_mask], target[nn_idx[inlier_mask]])
            if refit is not None and s_min <= refit[0] <= s_max:
                s_best, R_best, t_best = refit

        # 최종 residual (best 변환 적용 후 평균 NN 거리, inlier만)
        source_final = s_best * (source @ R_best.T) + t_best
        dists_final, _ = tree.query(source_final, k=1)
        inlier_final = dists_final < inlier_thr
        if int(np.sum(inlier_final)) > 0:
            residual = float(np.mean(dists_final[inlier_final]))
        else:
            residual = float('inf')
        return s_best, R_best, t_best, residual, True

    # ── DA3-Streaming Stage 1: Inter-chunk Sim3 alignment ─────────────
    @staticmethod
    def _umeyama_3d_similarity(
        src: np.ndarray, tgt: np.ndarray, weights: Optional[np.ndarray] = None,
    ) -> Optional[tuple[float, np.ndarray, np.ndarray, float]]:
        """Weighted Umeyama 3D similarity: tgt = s * R @ src + t (least-squares).

        Args:
            src, tgt: (N, 3) corresponding 3D points
            weights: (N,) optional per-pair weights
        Returns: (scale, R(3,3), t(3,), residual) or None if degenerate
        """
        if len(src) < 3 or len(tgt) < 3:
            return None
        if weights is None:
            weights = np.ones(len(src), dtype=np.float64)
        w_sum = float(np.sum(weights))
        if w_sum < 1e-8:
            return None
        w = weights[:, None] / w_sum
        mu_s = np.sum(src * w, axis=0)
        mu_t = np.sum(tgt * w, axis=0)
        src_c = src - mu_s
        tgt_c = tgt - mu_t
        var_s = float(np.sum((src_c * src_c) * w))
        if var_s < 1e-10:
            return None
        H = (src_c * w).T @ tgt_c  # 3x3
        U, S_vec, Vt = np.linalg.svd(H)
        D_sign = np.ones(3)
        if np.linalg.det(U) * np.linalg.det(Vt) < 0:
            D_sign[-1] = -1
        R = U @ np.diag(D_sign) @ Vt
        scale = float(np.sum(S_vec * D_sign) / var_s)
        t_vec = mu_t - scale * (R @ mu_s)
        residual = float(np.mean(np.linalg.norm(
            scale * (src @ R.T) + t_vec - tgt, axis=1)))
        return scale, R, t_vec, residual

    def _chunk_frames_to_world_xyz(
        self, depths: list, K_list: list, extrinsics_c2w: Optional[np.ndarray],
        image_msgs: list, frame_indices: list,
    ) -> Optional[dict]:
        """선택된 frame들의 depth → world XYZ (stride 샘플링).

        Returns: dict[frame_idx] → (M, 3) world-frame points.
        """
        if extrinsics_c2w is None:
            return None
        stride = max(1, int(self.get_parameter('point_cloud_stride').value))
        min_d = float(self.get_parameter('min_depth_m').value)
        max_d = float(self.get_parameter('max_depth_m').value)
        result: dict[int, np.ndarray] = {}
        for i in frame_indices:
            if i >= len(depths):
                continue
            d = depths[i]
            K = K_list[i]
            T_wc = extrinsics_c2w[i] if i < len(extrinsics_c2w) else None
            if T_wc is None:
                continue
            H, W = d.shape
            fx, fy = float(K[0, 0]), float(K[1, 1])
            cx, cy = float(K[0, 2]), float(K[1, 2])
            if fx <= 0 or fy <= 0:
                continue
            z = d[::stride, ::stride]
            rows = np.arange(0, H, stride, dtype=np.float32)[:z.shape[0]]
            cols = np.arange(0, W, stride, dtype=np.float32)[:z.shape[1]]
            u, v = np.meshgrid(cols, rows)
            valid = np.isfinite(z) & (z > min_d) & (z < max_d)
            if not np.any(valid):
                continue
            x_cam = (u[valid] - cx) * z[valid] / fx
            y_cam = (v[valid] - cy) * z[valid] / fy
            pts_cam = np.stack([x_cam, y_cam, z[valid]], axis=-1).astype(np.float32)
            R = T_wc[:3, :3].astype(np.float32)
            t = T_wc[:3, 3].astype(np.float32)
            pts_world = pts_cam @ R.T + t
            result[i] = pts_world
        return result if result else None

    def _align_current_chunk_to_prev(
        self, curr_xyz_by_frame: dict,
    ) -> tuple[float, np.ndarray, np.ndarray, float, int]:
        """Inter-chunk Sim3 alignment (DA3-Streaming 핵심).

        현재 chunk의 처음 overlap frame들과 이전 chunk의 끝 overlap frame들을
        같은 world XYZ로 매칭 → Umeyama Sim3 → (s, R, t).

        현재 chunk 처음 overlap frame == 이전 chunk 끝 overlap frame
        (sliding window의 공유 구간).

        Returns: (scale, R, t, residual, n_corr)
        """
        if not self._prev_chunk_xyz_by_frame:
            return 1.0, np.eye(3), np.zeros(3), 0.0, 0

        # 공유 frame key 찾기. 단순화: 이전 chunk의 "뒤쪽 overlap 개" +
        # 현재 chunk의 "앞쪽 overlap 개" 는 time 순 연속이라 같은 실제 관측.
        overlap = int(self.get_parameter('keyframe_overlap').value)
        if overlap <= 0:
            return 1.0, np.eye(3), np.zeros(3), 0.0, 0
        # 키 기반 매칭: 공유 frame key 사용
        shared_keys = sorted(set(self._prev_chunk_xyz_by_frame.keys())
                             & set(curr_xyz_by_frame.keys()))
        if len(shared_keys) < 2:
            return 1.0, np.eye(3), np.zeros(3), 0.0, 0

        # 공유 frame의 point 집합
        src_list = []
        tgt_list = []
        max_pts_per_frame = 2000  # Umeyama는 많은 점 필요 X, 속도 우선
        for k in shared_keys:
            src_pts = curr_xyz_by_frame[k]  # current chunk 좌표
            tgt_pts = self._prev_chunk_xyz_by_frame[k]  # previous chunk 좌표
            n = min(len(src_pts), len(tgt_pts), max_pts_per_frame)
            if n < 10:
                continue
            # 같은 pixel index 대응 (stride 같으면 index 매칭 OK)
            src_list.append(src_pts[:n])
            tgt_list.append(tgt_pts[:n])
        if not src_list:
            return 1.0, np.eye(3), np.zeros(3), 0.0, 0

        src = np.vstack(src_list)
        tgt = np.vstack(tgt_list)
        result = self._umeyama_3d_similarity(src, tgt)
        if result is None:
            return 1.0, np.eye(3), np.zeros(3), 0.0, len(src)
        s, R, t, residual = result
        return s, R, t, residual, len(src)

    def _smooth_scale(self, scale_raw: float) -> float:
        """EMA smoothing of scalar scale."""
        if not self._scale_initialized:
            self._scale_smooth = scale_raw
            self._scale_initialized = True
            return scale_raw
        a = float(self.get_parameter('scale_smooth_alpha').value)
        self._scale_smooth = (1.0 - a) * self._scale_smooth + a * scale_raw
        return self._scale_smooth

    def _rate_in_window(self, ts_deque: deque, window_s: float) -> float:
        """최근 window_s 초 사이의 이벤트 수 / window_s = Hz."""
        if not ts_deque:
            return 0.0
        now = self._wall_time()
        cnt = sum(1 for t in ts_deque if now - t <= window_s)
        return cnt / window_s

    def _publish_diagnostics(self) -> None:
        """1Hz로 DA3 건강 상태를 /diagnostics에 publish.

        Foxglove의 Diagnostics 패널에서 실시간 지표 + WARN/ERROR 색상 표시됨.
        stall, camera 멈춤 같은 문제를 grep 없이 즉시 판별 가능.
        """
        try:
            now = self._wall_time()
            # 각종 rate (sliding window)
            cam_rate_1s  = self._rate_in_window(self._cam_ts, 1.0)
            cam_rate_5s  = self._rate_in_window(self._cam_ts, 5.0)
            kf_rate_5s   = self._rate_in_window(self._kf_accept_ts, 5.0)
            chunk_rate_5s = self._rate_in_window(self._chunk_ts, 5.0)
            last_chunk_ago = (now - self._last_inference_wall
                              if self._last_inference_wall > 0 else 999.0)
            last_cam_ago = (now - self._cam_ts[-1]) if self._cam_ts else 999.0
            buf_fill = len(self._image_buffer)
            buf_max = self._image_buffer.maxlen or 0

            # GPU 메모리 (모델 로드됐을 때만)
            gpu_mem_mb = 0
            if self._torch is not None:
                try:
                    gpu_mem_mb = int(self._torch.cuda.memory_allocated() // (1 << 20))
                except Exception:
                    pass

            # 건강 판정 로직
            level = DiagnosticStatus.OK
            msg_text = "OK"
            if not self._model_ready:
                level = DiagnosticStatus.WARN
                msg_text = "Model loading..."
            elif last_cam_ago > 3.0:
                level = DiagnosticStatus.ERROR
                msg_text = f"No camera input for {last_cam_ago:.1f}s"
            elif last_chunk_ago > 5.0 and self._total_chunks > 0:
                level = DiagnosticStatus.ERROR
                msg_text = f"Inference stalled: no chunk for {last_chunk_ago:.1f}s"
            elif last_chunk_ago > 2.0 and self._total_chunks > 0:
                level = DiagnosticStatus.WARN
                msg_text = f"Inference slow: last chunk {last_chunk_ago:.1f}s ago"
            elif cam_rate_5s < 3.0 and self._total_chunks > 0:
                level = DiagnosticStatus.WARN
                msg_text = f"Camera rate low: {cam_rate_5s:.1f} Hz"
            elif self._total_errors > 0 and self._last_error_msg:
                level = DiagnosticStatus.WARN
                msg_text = f"{self._total_errors} errors so far"

            stat = DiagnosticStatus()
            stat.level = level
            stat.name = "da3_depth_node"
            stat.message = msg_text
            stat.hardware_id = "cuda:0"
            stat.values = [
                KeyValue(key="camera_rate_1s_hz",   value=f"{cam_rate_1s:.2f}"),
                KeyValue(key="camera_rate_5s_hz",   value=f"{cam_rate_5s:.2f}"),
                KeyValue(key="keyframe_rate_5s_hz", value=f"{kf_rate_5s:.2f}"),
                KeyValue(key="chunk_rate_5s_hz",    value=f"{chunk_rate_5s:.2f}"),
                KeyValue(key="last_chunk_ago_s",    value=f"{last_chunk_ago:.2f}"),
                KeyValue(key="last_camera_ago_s",   value=f"{last_cam_ago:.2f}"),
                KeyValue(key="buffer_fill",         value=f"{buf_fill}/{buf_max}"),
                KeyValue(key="input_views",         value=str(self._input_views)),
                KeyValue(key="total_chunks",        value=str(self._total_chunks)),
                KeyValue(key="total_errors",        value=str(self._total_errors)),
                KeyValue(key="last_infer_ms",       value=f"{self._last_infer_ms:.1f}"),
                KeyValue(key="last_tf_ms",          value=f"{self._last_tf_ms:.1f}"),
                KeyValue(key="last_publish_ms",     value=f"{self._last_publish_ms:.1f}"),
                KeyValue(key="gpu_mem_mb",          value=str(gpu_mem_mb)),
                KeyValue(key="lidar_inlier_pairs",  value=str(self._last_lidar_inliers_n)),
                KeyValue(key="scale_smooth",         value=f"{self._scale_smooth:.3f}"),
                KeyValue(key="scale_floor",          value=f"{self._last_scale_floor:.3f}"),
                KeyValue(key="scale_horizon",        value=f"{self._last_scale_horizon:.3f}"),
                KeyValue(key="interchunk_scale",     value=f"{self._last_interchunk_scale:.4f}"),
                KeyValue(key="interchunk_residual_cm", value=f"{self._last_interchunk_residual*100:.1f}"),
                KeyValue(key="interchunk_corr",      value=str(self._last_interchunk_corr)),
                KeyValue(key="cumulative_scale",     value=f"{self._cumulative_scale:.4f}"),
                KeyValue(key="chunk_index",          value=str(self._chunk_index)),
                KeyValue(key="global_map_points",    value=str(self._global_total_points)),
                KeyValue(key="global_map_chunks",    value=str(len(self._global_chunks_xyz))),
                KeyValue(key="global_map_new_buf",   value=str(sum(len(x) for x in self._global_new_xyz))),
                KeyValue(key="scale_ratio(floor/hz)", value=(f"{self._last_scale_floor/self._last_scale_horizon:.3f}"
                                                              if self._last_scale_horizon > 1e-3 else "-")),
                KeyValue(key="floor_plane_z_m",       value=f"{self._last_floor_z_m:+.3f}"),
                KeyValue(key="floor_thickness_cm",    value=f"{self._last_floor_thickness_m*100:.1f}"),
                KeyValue(key="floor_inliers",         value=str(self._last_floor_inliers)),
                KeyValue(key="horizon_matched",       value=str(self._last_horizon_matched)),
                KeyValue(key="floor_total_rejects",   value=str(self._total_floor_rejects)),
                KeyValue(key="frames_rejected_L1",    value=str(self._total_frames_rejected)),
                KeyValue(key="temporal_reject_pct",
                         value=(f"{100.0*self._temporal_rejected_pixels/max(1,self._temporal_total_pixels):.1f}"
                                if self._temporal_total_pixels > 0 else "-")),
                KeyValue(key="esdf_reject_pct",
                         value=(f"{100.0*self._esdf_rejected_pixels/max(1,self._esdf_total_pixels):.1f}"
                                if self._esdf_total_pixels > 0 else "-")),
                KeyValue(key="voxel_history_size",  value=str(len(self._voxel_depth_history))),
                KeyValue(key="esdf_ref_points",
                         value=(str(len(self._esdf_ref_xyz)) if self._esdf_ref_xyz is not None else "0")),
                KeyValue(key="last_error",          value=self._last_error_msg or "-"),
            ]

            arr = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status = [stat]
            self._diag_pub.publish(arr)
        except Exception as exc:
            # diagnostics 실패가 메인 flow에 영향 주면 안 됨
            self.get_logger().debug(f'diagnostics publish failed: {exc}')

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
            # _project_frame_to_world의 GPU 가속에서 사용. cuda면 reprojection이
            # CPU numpy 대비 10배+ 빠름 (8 frames × 76k pts × stride 2 기준).
            self._infer_device = torch.device(device) if isinstance(device, str) else device
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
                # use_ray_pose=True는 내부에서 torch.inverse(intrinsics) 호출 →
                # libtorch_cuda_linalg.so lazy dlopen 실패 (torch 2.11+cu130 환경 이슈).
                # 해결 전까지 False. extrinsics 직접 전달하는 방식으로 pose feedback 우회.
                use_ray_pose=False,
            )
            # Umeyama Sim(3) pose alignment는 최소 2개 non-collinear pose 필요.
            # single-view (N=1)면 degenerate covariance → GeometryException.
            # → single-view 땐 extrinsics 아예 생략 (어차피 metric scale은 lidar_scale
            # 로 보정하니 DA3의 pose alignment가 필요 없음).
            if extrinsics_w2c is not None and len(image_msgs) >= 2:
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

            # ── Floor plane scale (primary) + Horizon bearing (cross-check) ──
            # Primary: DA3 3D points에서 floor plane 검출 → z=0 비교 → scale.
            #   Lidar 불필요. Geometric prior (camera_h=0.5, floor_z=0) 기반.
            # Cross-check: Horizon bearing (lidar scan 기반) → scale_horizon diag 전용.
            # 두 값이 비슷하면 교차검증 통과.
            batch_scale = 1.0
            if apply_lidar:
                ref_idx = n // 2
                src_frame_ref = self._source_frame_id(image_msgs[ref_idx].header.frame_id)
                T_base_cam = self._lookup_tf_matrix(
                    src_frame_ref, 'base_link', image_msgs[ref_idx].header.stamp)

                if T_base_cam is not None and bool(self.get_parameter('floor_scale_as_primary').value):
                    # ── Scale chain: 여러 estimator의 median으로 합산 ──
                    estimates = []   # (이름, 값)
                    diag_parts = []

                    # 1) Floor + Ceiling joint span (camera height 의존 X)
                    if bool(self.get_parameter('floor_ceiling_enable').value):
                        sc_fc, z_f_fc, z_c_fc, n_f_fc, n_c_fc = (
                            self._estimate_scale_from_floor_ceiling(
                                depths[ref_idx], K_list[ref_idx], T_base_cam))
                        if sc_fc is not None:
                            estimates.append(('fc', sc_fc))
                            diag_parts.append(
                                f'fc={sc_fc:.3f}(zf={z_f_fc:+.2f} zc={z_c_fc:+.2f} '
                                f'h={z_c_fc - z_f_fc:.2f}m)')
                        else:
                            diag_parts.append(
                                f'fc=- (zf={z_f_fc} zc={z_c_fc} '
                                f'nf={n_f_fc} nc={n_c_fc})')

                    # 2) Motion scale: DA3 BA pose translation vs TF
                    if bool(self.get_parameter('motion_scale_enable').value):
                        sc_m, bl_tf, bl_da3 = self._estimate_scale_from_motion(
                            prediction, extrinsics_c2w)
                        if sc_m is not None:
                            estimates.append(('motion', sc_m))
                            diag_parts.append(
                                f'motion={sc_m:.3f}(tf={bl_tf:.2f}m da3={bl_da3:.2f}m)')
                        else:
                            diag_parts.append(f'motion=- (tf={bl_tf:.2f} da3={bl_da3:.2f})')

                    # 3) Floor only (fallback / cross-check)
                    scale_floor, thickness, n_floor = self._estimate_scale_from_floor(
                        depths[ref_idx], K_list[ref_idx], T_base_cam)
                    self._last_floor_thickness_m = thickness
                    self._last_floor_inliers = n_floor
                    if scale_floor is not None:
                        self._last_scale_floor = scale_floor
                        # Floor 단독은 z_DA3 sanity check 통과한 것만 chain에 추가
                        # (천장이 floor로 잡힌 경우 z_DA3 > 0 → 이미 위 함수에서 reject됨)
                        if self._last_floor_z_m < -0.05:  # floor는 카메라 아래 5cm 이상
                            estimates.append(('floor', scale_floor))
                        diag_parts.append(
                            f'floor={scale_floor:.3f}(z={self._last_floor_z_m:+.2f}m '
                            f'thick={thickness*100:.1f}cm n={n_floor})')
                    else:
                        diag_parts.append(f'floor=- (n={n_floor})')

                    # 4) Horizon bearing (cross-check, chain 미참여 — DA3 depth 의존이라 noisy)
                    if bool(self.get_parameter('horizon_scale_enable').value):
                        scale_horizon, n_match = self._estimate_scale_from_horizon_bearing(
                            depths[ref_idx], K_list[ref_idx], T_base_cam, scan)
                        self._last_horizon_matched = n_match
                        if scale_horizon is not None:
                            self._last_scale_horizon = scale_horizon
                            diag_parts.append(f'hz={scale_horizon:.3f}(n={n_match})')

                    # ── Chain median ─────────────────────────────────────
                    if estimates:
                        vals = np.array([v for _, v in estimates])
                        scale_raw = float(np.median(vals))
                        batch_scale = self._smooth_scale(scale_raw)
                        names = ','.join(n for n, _ in estimates)
                        self.get_logger().info(
                            f'[scale] {" ".join(diag_parts)} '
                            f'| chain[{names}] median={scale_raw:.3f} → smooth={batch_scale:.3f}')
                    else:
                        # 모든 estimator 실패 → reject chunk (nvblox 오염 방지)
                        self.get_logger().warn(
                            f'[scale] REJECT chunk: all estimators failed | {" ".join(diag_parts)}')
                        self._total_floor_rejects += 1
                        fallback_scale = self._scale_smooth if self._scale_initialized else 1.0
                        self._publish_rejected_world_cloud(
                            depths, K_list, image_msgs, rgbs, extrinsics_c2w,
                            prediction, fallback_scale)
                        return

                # 모든 frame depth에 chain-derived scale 적용
                if self._scale_initialized and abs(batch_scale - 1.0) > 1e-4:
                    depths = [d * batch_scale for d in depths]

            # ── DA3-Streaming Stage 1: Inter-chunk Sim3 alignment ──────
            # 이전 chunk와 overlap 공유 frame들의 3D 포인트를 매칭하여 Sim3 fit.
            # 현재 chunk의 scale을 이전 chunk 좌표계로 align → 전 sequence 일관.
            interchunk_scale = 1.0
            if (bool(self.get_parameter('interchunk_align_enable').value)
                    and extrinsics_c2w is not None):
                overlap = int(self.get_parameter('keyframe_overlap').value)
                if overlap > 0:
                    # 현재 chunk의 "앞쪽 overlap 개" (이전 chunk와 공유되는 frame)
                    shared_curr_idx = list(range(min(overlap, n)))
                    # 이들의 world XYZ 계산
                    curr_shared_xyz = self._chunk_frames_to_world_xyz(
                        depths, K_list, extrinsics_c2w, image_msgs, shared_curr_idx)

                    if curr_shared_xyz is not None and self._prev_chunk_xyz_by_frame:
                        # Umeyama Sim3: curr → prev
                        s, R, t, residual, n_corr = self._align_current_chunk_to_prev(
                            curr_shared_xyz)
                        self._last_interchunk_scale = s
                        self._last_interchunk_residual = residual
                        self._last_interchunk_corr = n_corr

                        res_thr = float(self.get_parameter('interchunk_max_residual_m').value)
                        s_min = float(self.get_parameter('scale_min').value)
                        s_max = float(self.get_parameter('scale_max').value)
                        if (n_corr >= 20 and residual <= res_thr
                                and s_min <= s <= s_max):
                            # Scale만 depth에 곱 (R, t는 TF로 이미 관리 중)
                            interchunk_scale = s
                            self._cumulative_scale *= s
                            depths = [d * s for d in depths]
                            self.get_logger().info(
                                f'[interchunk] chunk#{self._chunk_index} '
                                f's={s:.4f} residual={residual*100:.1f}cm '
                                f'n_corr={n_corr} cumulative_s={self._cumulative_scale:.4f}')
                        else:
                            self.get_logger().warn(
                                f'[interchunk] skip alignment: '
                                f's={s:.3f} residual={residual*100:.1f}cm n_corr={n_corr}')

                    # 현재 chunk의 "뒤쪽 overlap 개" (다음 chunk와 공유)
                    # → depth가 이미 scale 적용된 상태로 저장
                    next_shared_idx = list(range(max(0, n - overlap), n))
                    next_shared_xyz = self._chunk_frames_to_world_xyz(
                        depths, K_list, extrinsics_c2w, image_msgs, next_shared_idx)
                    if next_shared_xyz is not None:
                        # frame key 재매핑: 현재 chunk의 "뒤쪽 N" → 다음 chunk의 "앞쪽 N"
                        remapped = {i - (n - overlap): v
                                    for i, v in next_shared_xyz.items()}
                        self._prev_chunk_xyz_by_frame = remapped

            self._chunk_index += 1
            t_lidar = _time.perf_counter()

            # ── Depth masking: nvblox와 world_points 둘 다 같은 필터 거침 ──
            # 2D depth image에서 가능한 필터:
            #   1) Range clip (max_depth_m 초과 = lidar 미검증 외삽)
            #   2) Conf filter (DA3 prediction.conf < min_confidence)
            #   3) Sky mask (DA3 sky=True = 하늘/창문 등 가짜 depth)
            #   4) Edge gradient (|Δdepth| > threshold = DA3 실루엣 bleeding)
            # 무효 pixel은 0으로 set → nvblox 자동 skip, project_frame도 같은 0 필터링.
            # 3D radius outlier는 2D에선 불가 → world_points에서만 추가 적용.
            min_d = float(self.get_parameter('min_depth_m').value)
            max_d = float(self.get_parameter('max_depth_m').value)
            conf_min = float(self.get_parameter('min_confidence').value)
            edge_thr = float(self.get_parameter('edge_gradient_threshold_m').value)

            # conf/sky는 torch tensor일 수 있음 → np.asarray()가 GPU→CPU sync 유발.
            # _mask_depth + _project_frame_to_world에서 frame마다 각각 호출하면
            # chunk당 32 syncs. chunk 시작 시 한 번만 다 가져와서 cache.
            has_conf = getattr(prediction, 'conf', None) is not None
            has_sky = getattr(prediction, 'sky', None) is not None
            confs = ([np.asarray(prediction.conf[i], dtype=np.float32) for i in range(n)]
                     if has_conf else [None] * n)
            skys = ([np.asarray(prediction.sky[i]) for i in range(n)]
                    if has_sky else [None] * n)

            def _mask_depth(i: int) -> np.ndarray:
                d = depths[i].astype(np.float32, copy=True)
                # 1. Range clip
                d = np.where(np.isfinite(d) & (d >= min_d) & (d <= max_d), d, 0.0)
                # 2. Conf filter (DA3 prediction에 conf 있을 때만)
                if conf_min > 0 and confs[i] is not None and confs[i].shape == d.shape:
                    d = np.where(confs[i] >= conf_min, d, 0.0)
                # 3. Sky mask
                if skys[i] is not None and skys[i].shape == d.shape:
                    d = np.where(skys[i].astype(bool), 0.0, d)
                # 4. Edge gradient — 실루엣 bleeding 제거 (nvblox TSDF 오염 차단)
                if edge_thr > 0 and d.shape[0] >= 3 and d.shape[1] >= 3:
                    dx = np.zeros_like(d); dy = np.zeros_like(d)
                    dx[:, 1:-1] = np.abs(d[:, 2:] - d[:, :-2]) * 0.5
                    dy[1:-1, :] = np.abs(d[2:, :] - d[:-2, :]) * 0.5
                    grad = np.maximum(dx, dy)
                    d = np.where(grad < edge_thr, d, 0.0)
                return d

            # ── L3-A/B 3D consistency check params ─────────────────────
            tc_enable = bool(self.get_parameter('temporal_check_enable').value)
            esdf_enable = bool(self.get_parameter('esdf_check_enable').value)
            # stride 8: 640x480 → 80x60 = 4800 points. stride 4 대비 4배 빠름.
            # block 크기 8px = 실제 world 수 cm 수준이라 mask 해상도 충분.
            check_stride = 8

            batch_xyz: list[np.ndarray] = []
            batch_rgb: list[np.ndarray] = []
            t_mask_sum = 0.0
            t_cons_sum = 0.0
            t_pubd_sum = 0.0
            t_pubpc_sum = 0.0
            t_proj_sum = 0.0
            self._proj_acc = {'gpu': 0.0, 'd2h': 0.0, 'kdtree': 0.0, 'zscore': 0.0}
            for i in range(n):
                _ts = _time.perf_counter()
                d = _mask_depth(i)
                t_mask_sum += _time.perf_counter() - _ts
                K_i = K_list[i]
                T_world_cam_i = (extrinsics_c2w[i]
                                 if extrinsics_c2w is not None and i < len(extrinsics_c2w)
                                 else None)

                # ── L3-A + L3-B: 3D consistency masking ─────────────────
                # stride 샘플링으로 world-frame xyz 생성 → temporal + ESDF 검사.
                # 검사 통과 못한 샘플 위치의 depth block을 0으로 만듦.
                _ts = _time.perf_counter()
                if (tc_enable or esdf_enable) and T_world_cam_i is not None:
                    d = self._apply_3d_consistency_masks(
                        d, K_i, T_world_cam_i, stride=check_stride,
                        use_temporal=tc_enable, use_esdf=esdf_enable)
                t_cons_sum += _time.perf_counter() - _ts

                depths[i] = d  # 후속 project에도 일관된 masked depth 사용
                # nvblox 입력 (masked)
                _ts = _time.perf_counter()
                self._publish_depth(image_msgs[i], d, K_i, camera_info_msgs[i])
                t_pubd_sum += _time.perf_counter() - _ts
                _ts = _time.perf_counter()
                self._publish_point_cloud(image_msgs[i], d, K_i)
                t_pubpc_sum += _time.perf_counter() - _ts
                # world 투영 — TF 기반 c2w 행렬 사용 (위에서 이미 계산).
                # DA3 refined extrinsics는 BA로 batch 사이 표류 → ghosting 원인이라
                # 의도적으로 무시. TF lookup 실패 시 frame skip (T_world_cam_i=None).
                _ts = _time.perf_counter()
                xyz_rgb = self._project_frame_to_world(
                    d, K_i, image_msgs[i], rgbs[i],
                    conf=confs[i], sky=skys[i], T_world_camera=T_world_cam_i)
                t_proj_sum += _time.perf_counter() - _ts
                if xyz_rgb is not None:
                    batch_xyz.append(xyz_rgb[0])
                    batch_rgb.append(xyz_rgb[1])

            t_pubw_start = _time.perf_counter()
            self._publish_world_batch(batch_xyz, batch_rgb, image_msgs[-1].header.stamp)
            t_pubw = _time.perf_counter() - t_pubw_start

            # Global accumulated map에 추가 (accepted chunk의 world points)
            t_gmap_start = _time.perf_counter()
            if (bool(self.get_parameter('global_map_enable').value)
                    and batch_xyz and batch_rgb):
                try:
                    xyz_concat = np.concatenate(batch_xyz, axis=0)
                    rgb_concat = np.concatenate(batch_rgb, axis=0)
                    self._add_to_global_map(xyz_concat, rgb_concat)
                except Exception as exc:
                    self.get_logger().debug(f'global_map add fail: {exc}')
            t_gmap = _time.perf_counter() - t_gmap_start
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
                f'[mask={t_mask_sum:.3f} cons={t_cons_sum:.3f} '
                f'pubd={t_pubd_sum:.3f} pubpc={t_pubpc_sum:.3f} '
                f'proj={t_proj_sum:.3f}(gpu={self._proj_acc["gpu"]:.3f} '
                f'd2h={self._proj_acc["d2h"]:.3f} '
                f'kd={self._proj_acc["kdtree"]:.3f} '
                f'zs={self._proj_acc["zscore"]:.3f}) '
                f'pubw={t_pubw:.3f} gmap={t_gmap:.3f}] '
                f'TOTAL={t_publish-t_start:.3f}s '
                f'frames={n} buf_after={buf_after}')
            # Diagnostic 업데이트
            self._chunk_ts.append(self._wall_time())
            self._total_chunks += 1
            self._last_inference_wall = self._wall_time()
            self._last_infer_ms = (t_infer - t_tf) * 1000.0
            self._last_tf_ms = (t_tf - t_decode) * 1000.0
            self._last_publish_ms = (t_publish - t_lidar) * 1000.0
        except Exception as exc:
            self.get_logger().error(f'DA3 inference failed: {exc}')
            self.get_logger().error(traceback.format_exc())
            self._total_errors += 1
            self._last_error_msg = str(exc)[:100]
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

        Reprojection 핵심 경로 (meshgrid, valid mask, edge gradient, 카메라→
        월드 변환, RGB 리샘플)는 cuda 가능 시 torch GPU에서 수행.
        76k+ 픽셀에서 numpy 대비 ~10배 빠름.

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
        edge_thr = float(self.get_parameter('edge_gradient_threshold_m').value)

        fx = float(intrinsics[0, 0]); fy = float(intrinsics[1, 1])
        cx = float(intrinsics[0, 2]); cy = float(intrinsics[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            return None

        # T 결정 (TF lookup if needed)
        if T_world_camera is not None:
            T = T_world_camera
        else:
            src_frame = self._source_frame_id(image_msg.header.frame_id)
            T = self._lookup_tf_matrix(src_frame, self._world_frame, image_msg.header.stamp)
            if T is None:
                return None

        # GPU path: cuda 가능 + 모델 cuda에 있을 때
        torch = self._torch
        use_gpu = (torch is not None
                   and self._infer_device is not None
                   and self._infer_device.type == 'cuda')

        import time as _time
        _t0 = _time.perf_counter()
        if use_gpu:
            with torch.no_grad():
                device = self._infer_device
                d_t = torch.from_numpy(np.ascontiguousarray(depth, dtype=np.float32)).to(
                    device, non_blocking=True)
                z = d_t[::stride, ::stride].contiguous()
                H_s, W_s = z.shape

                rows = torch.arange(
                    0, depth.shape[0], stride, dtype=torch.float32, device=device)[:H_s]
                cols = torch.arange(
                    0, depth.shape[1], stride, dtype=torch.float32, device=device)[:W_s]
                v_grid, u_grid = torch.meshgrid(rows, cols, indexing='ij')

                valid = torch.isfinite(z) & (z > min_d) & (z < max_d)
                if conf is not None and conf_min > 0:
                    c = torch.from_numpy(np.ascontiguousarray(conf, dtype=np.float32)).to(
                        device, non_blocking=True)[::stride, ::stride]
                    valid &= c >= conf_min
                if sky is not None:
                    s = torch.from_numpy(np.ascontiguousarray(sky)).to(
                        device, non_blocking=True)[::stride, ::stride]
                    valid &= ~s.bool()

                # Edge bleeding filter — DA3 single-view 전경/배경 경계 floating points 제거
                if edge_thr > 0 and H_s >= 3 and W_s >= 3:
                    z_safe = torch.where(
                        torch.isfinite(z) & (z > 0), z, torch.zeros_like(z))
                    grad_x = torch.zeros_like(z_safe)
                    grad_y = torch.zeros_like(z_safe)
                    grad_x[:, 1:-1] = torch.abs(z_safe[:, 2:] - z_safe[:, :-2]) * 0.5
                    grad_y[1:-1, :] = torch.abs(z_safe[2:, :] - z_safe[:-2, :]) * 0.5
                    grad = torch.maximum(grad_x, grad_y)
                    valid &= grad < edge_thr

                if not bool(valid.any()):
                    return None

                z_v = z[valid]
                x_cam = (u_grid[valid] - cx) * z_v / fx
                y_cam = (v_grid[valid] - cy) * z_v / fy

                # RGB → depth shape (필요 시)으로 리샘플 → stride → mask
                rgb_t = torch.from_numpy(np.ascontiguousarray(rgb)).to(
                    device, non_blocking=True).float()
                if rgb_t.shape[:2] != depth.shape:
                    import torch.nn.functional as F
                    rgb_t = F.interpolate(
                        rgb_t.permute(2, 0, 1).unsqueeze(0),
                        size=(depth.shape[0], depth.shape[1]),
                        mode='bilinear', align_corners=False,
                    ).squeeze(0).permute(1, 2, 0)
                rgb_pts_t = rgb_t[::stride, ::stride][valid]

                # 카메라 → 월드 변환
                T_t = torch.from_numpy(np.ascontiguousarray(T, dtype=np.float32)).to(
                    device, non_blocking=True)
                R = T_t[:3, :3]
                tvec = T_t[:3, 3]
                cam_pts = torch.stack((x_cam, y_cam, z_v), dim=-1)
                world_pts_t = cam_pts @ R.T + tvec

                _t_gpu = _time.perf_counter()
                self._proj_acc['gpu'] += _t_gpu - _t0
                world_pts = world_pts_t.detach().cpu().numpy().astype(np.float32, copy=False)
                rgb_pts = rgb_pts_t.detach().cpu().numpy().astype(np.uint8)
                self._proj_acc['d2h'] += _time.perf_counter() - _t_gpu
        else:
            # CPU fallback (cuda 사용 불가 환경)
            z = depth[::stride, ::stride]
            rows = np.arange(0, depth.shape[0], stride, dtype=np.float32)
            cols = np.arange(0, depth.shape[1], stride, dtype=np.float32)
            u, v = np.meshgrid(cols, rows)
            valid = np.isfinite(z) & (z > min_d) & (z < max_d)
            if conf is not None and conf_min > 0:
                valid &= conf[::stride, ::stride] >= conf_min
            if sky is not None:
                valid &= ~sky[::stride, ::stride].astype(bool)
            if edge_thr > 0 and z.shape[0] >= 3 and z.shape[1] >= 3:
                z_safe = np.where(np.isfinite(z) & (z > 0), z, 0.0).astype(np.float32)
                dx = np.zeros_like(z_safe); dy = np.zeros_like(z_safe)
                dx[:, 1:-1] = np.abs(z_safe[:, 2:] - z_safe[:, :-2]) * 0.5
                dy[1:-1, :] = np.abs(z_safe[2:, :] - z_safe[:-2, :]) * 0.5
                valid &= np.maximum(dx, dy) < edge_thr
            if not np.any(valid):
                return None
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            cam_pts = np.stack((x[valid], y[valid], z[valid]), axis=-1).astype(np.float32)
            rgb_resized = cv2.resize(
                rgb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_LINEAR)
            rgb_pts = rgb_resized[::stride, ::stride][valid].astype(np.uint8)
            R = T[:3, :3].astype(np.float32); tvec = T[:3, 3].astype(np.float32)
            world_pts = (cam_pts @ R.T) + tvec
            self._proj_acc['gpu'] += _time.perf_counter() - _t0

        _t_kd0 = _time.perf_counter()
        # ── Radius Outlier Removal (혼자 떠있는 점 제거) ─────────────
        # 3D 공간에서 반경 R 내 이웃 수 < threshold인 point는 isolated → 제거.
        # Open3D의 C++ KDTree (threading) 사용 — scipy cKDTree 대비 ~5배 빠름.
        # 76k pts × 8 frames per chunk에서 scipy는 ~1.85s, Open3D는 ~0.4s.
        if bool(self.get_parameter('outlier_filter_enable').value) and len(world_pts) > 0:
            r = float(self.get_parameter('outlier_radius_m').value)
            min_nb = int(self.get_parameter('outlier_min_neighbors').value)
            if r > 0 and min_nb > 0:
                try:
                    import open3d as o3d  # lazy
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(world_pts.astype(np.float64))
                    # nb_points = self 포함 최소 이웃 수 → scipy의 min_nb+1과 동일 의미
                    _, ind = pcd.remove_radius_outlier(nb_points=min_nb + 1, radius=r)
                    if len(ind) > 0:
                        keep = np.asarray(ind, dtype=np.int64)
                        world_pts = world_pts[keep]
                        rgb_pts = rgb_pts[keep]
                except Exception:
                    pass  # 실패 시 그냥 통과 (filter skip)
        _t_kd1 = _time.perf_counter()
        self._proj_acc['kdtree'] += _t_kd1 - _t_kd0

        # 3D 통계적 outlier 제거 (depth z-score)
        if len(world_pts) > 100:
            z_med = np.median(world_pts[:, 2])
            z_mad = np.median(np.abs(world_pts[:, 2] - z_med)) + 1e-6
            z_score = np.abs(world_pts[:, 2] - z_med) / z_mad
            inlier = z_score < 5.0  # 5 MAD = 약 3.5 sigma
            world_pts = world_pts[inlier]
            rgb_pts = rgb_pts[inlier]
        self._proj_acc['zscore'] += _time.perf_counter() - _t_kd1

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
        # noise_factor 0.05 → 0.02: 거리별 tolerance 엄격화.
        # 1m: 3.5cm, 5m: 11.5cm, 10m: 21.5cm (이전 0.05면 10m 51cm 허용 → 육안 안 맞음).
        # 라이다 실제 정밀도와 DA3 depth noise 고려한 물리적 한계치.
        noise_factor = 0.02

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
            # 실패 시 identity 반환 + caller가 EMA smoothing에 넣음 → 점진 blend.
            # 이전 값 유지는 위험: 한번 나쁜 값 고착되면 지속적 영향.
            self.get_logger().warn(
                f'[batch_affine] RANSAC fail: best inliers={best_count}/{n_initial} '
                f'→ identity (s=1, t=0) to be blended by smoothing')
            return 1.0, 0.0, 0

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

        # s 물리적 허용 범위 [1.0, 2.5] — 범위 밖은 RANSAC 실패로 간주.
        # DA3는 보통 depth를 약간 overestimate → lidar로 1-2.5배 scaling 필요.
        # 이 범위 벗어나면 (복도 단면 bin imbalance 등) fit 신뢰 불가 → identity fallback.
        S_MIN, S_MAX = 1.0, 2.5
        T_MAX = 1.0  # disparity shift 상한 (1/m 단위)
        if not (S_MIN <= final_s <= S_MAX) or abs(final_t) > T_MAX:
            self.get_logger().warn(
                f'[batch_affine] OUT-OF-RANGE: s={final_s:.4f} '
                f'(허용 [{S_MIN},{S_MAX}]) t={final_t:+.4f} → identity')
            return 1.0, 0.0, 0   # → smooth_affine이 min_inliers 미달로 skip update

        final_t = float(np.clip(final_t, -T_MAX, T_MAX))

        zl_in = z_l[best_mask]
        self.get_logger().info(
            f'[batch_affine] s={final_s:.4f}, t={final_t:+.4f} (1/m) '
            f'inliers={best_count}/{n_initial} '
            f'({100*best_count/n_initial:.0f}%) '
            f'z_l=[{zl_in.min():.1f},{zl_in.max():.1f}]m')
        return final_s, final_t, best_count

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

    # ── Global accumulated raw point cloud ────────────────────────────
    def _add_to_global_map(self, xyz: np.ndarray, rgb: np.ndarray) -> None:
        """Accepted chunk의 world points를 raw point cloud buffer에 추가.

        Voxel dedup 없이 원본 점 그대로 누적. max_points 초과 시 앞쪽(오래된) chunk drop.
        """
        if xyz.size == 0 or rgb.size == 0:
            return
        xyz32 = xyz.astype(np.float32, copy=False)
        rgb8 = rgb.astype(np.uint8, copy=False)
        max_pts = int(self.get_parameter('global_map_max_points').value)

        with self._global_map_lock:
            # Add new chunk
            self._global_chunks_xyz.append(xyz32)
            self._global_chunks_rgb.append(rgb8)
            self._global_new_xyz.append(xyz32)
            self._global_new_rgb.append(rgb8)
            self._global_total_points += len(xyz32)

            # Memory cap: drop oldest chunks until under cap
            while self._global_total_points > max_pts and len(self._global_chunks_xyz) > 1:
                old_xyz = self._global_chunks_xyz.pop(0)
                self._global_chunks_rgb.pop(0)
                self._global_total_points -= len(old_xyz)

    def _pack_points_to_msg(self, xyz: np.ndarray, rgb: np.ndarray, stamp) -> Optional[PointCloud2]:
        """(N, 3) xyz + (N, 3) rgb → PointCloud2 메시지."""
        if xyz is None or len(xyz) == 0:
            return None
        n = len(xyz)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self._world_frame
        msg.height = 1
        msg.width = n
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 16
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        buf = np.zeros((n, 4), dtype=np.float32)
        buf[:, 0:3] = xyz.astype(np.float32)
        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
        packed = (r << 16) | (g << 8) | b
        buf[:, 3] = packed.view(np.float32)
        msg.row_step = msg.point_step * msg.width
        msg.data = buf.tobytes()
        return msg

    def _publish_global_map_delta(self) -> None:
        """Delta: 마지막 publish 이후 추가된 chunk의 점들만 publish (작은 메시지)."""
        if self._global_map_delta_pub is None:
            return
        try:
            with self._global_map_lock:
                if not self._global_new_xyz:
                    return
                xyz_list = self._global_new_xyz
                rgb_list = self._global_new_rgb
                self._global_new_xyz = []
                self._global_new_rgb = []
            xyz = np.concatenate(xyz_list, axis=0)
            rgb = np.concatenate(rgb_list, axis=0)
            msg = self._pack_points_to_msg(xyz, rgb, self.get_clock().now().to_msg())
            if msg is not None:
                self._global_map_delta_pub.publish(msg)
                self._global_delta_pub_count += 1
                if self._global_delta_pub_count <= 3 or self._global_delta_pub_count % 30 == 0:
                    self.get_logger().info(
                        f'[global_map] delta #{self._global_delta_pub_count} '
                        f'pts={len(xyz)} ({msg.row_step/1024:.1f}KB)')
        except Exception as exc:
            self.get_logger().error(f'global_map_delta publish fail: {exc}')

    def _publish_global_map_full(self) -> None:
        """Full: 전체 누적 point cloud 주기적 발행 (late-join client sync용)."""
        if self._global_map_full_pub is None:
            return
        try:
            with self._global_map_lock:
                if not self._global_chunks_xyz:
                    return
                xyz_list = list(self._global_chunks_xyz)
                rgb_list = list(self._global_chunks_rgb)
            xyz = np.concatenate(xyz_list, axis=0)
            rgb = np.concatenate(rgb_list, axis=0)
            msg = self._pack_points_to_msg(xyz, rgb, self.get_clock().now().to_msg())
            if msg is not None:
                self._global_map_full_pub.publish(msg)
                self._global_last_full_pub_wall = self._wall_time()
                self._global_full_pub_count += 1
                if self._global_full_pub_count <= 3 or self._global_full_pub_count % 6 == 0:
                    self.get_logger().info(
                        f'[global_map] full #{self._global_full_pub_count} '
                        f'pts={len(xyz)} ({msg.row_step/1024/1024:.1f}MB) '
                        f'chunks={len(xyz_list)}')
        except Exception as exc:
            self.get_logger().error(f'global_map_full publish fail: {exc}')

    def _publish_rejected_world_cloud(
        self, depths: list, K_list: list, image_msgs: list, rgbs: list,
        extrinsics_c2w: Optional[np.ndarray], prediction, fallback_scale: float,
    ) -> None:
        """Rejected chunk의 world-frame cloud를 별도 topic으로 발행 (디버깅용).

        scale 보정 실패 frame도 "DA3가 뭘 냈나" 시각 확인 가능하게.
        nvblox에는 들어가지 않음 (oil pollution 방지).
        """
        try:
            n = len(depths)
            batch_xyz: list[np.ndarray] = []
            batch_rgb: list[np.ndarray] = []
            for i in range(n):
                d = depths[i] * fallback_scale  # fallback scale만 적용
                K_i = K_list[i]
                T_wc_i = (extrinsics_c2w[i]
                          if extrinsics_c2w is not None and i < len(extrinsics_c2w)
                          else None)
                conf_i = (np.asarray(prediction.conf[i], dtype=np.float32)
                          if getattr(prediction, 'conf', None) is not None else None)
                sky_i = (np.asarray(prediction.sky[i])
                         if getattr(prediction, 'sky', None) is not None else None)
                xyz_rgb = self._project_frame_to_world(
                    d, K_i, image_msgs[i], rgbs[i],
                    conf=conf_i, sky=sky_i, T_world_camera=T_wc_i)
                if xyz_rgb is not None:
                    batch_xyz.append(xyz_rgb[0])
                    batch_rgb.append(xyz_rgb[1])
            if batch_xyz:
                self._publish_world_batch_to(
                    self._world_pc_rejected_pub, batch_xyz, batch_rgb,
                    image_msgs[-1].header.stamp)
        except Exception as exc:
            self.get_logger().debug(f'rejected cloud publish failed: {exc}')

    def _publish_world_batch_to(
        self, publisher, xyz_list: list, rgb_list: list, stamp,
    ) -> None:
        """주어진 publisher로 world cloud 발행 (내부 헬퍼)."""
        # _publish_world_batch의 구현을 그대로 사용하되 publisher만 다르게.
        # 이하 로직은 _publish_world_batch와 동일해야 하므로 delegation 하지 않고
        # 파라미터로 publisher 주입. 중복 대신 내부에서 call.
        if not xyz_list:
            return
        xyz = np.concatenate(xyz_list, axis=0)
        rgb = np.concatenate(rgb_list, axis=0)
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
        publisher.publish(msg)

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
