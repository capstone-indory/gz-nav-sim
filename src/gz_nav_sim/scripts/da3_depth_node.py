#!/usr/bin/env python3

"""ROS 2 wrapper for Depth Anything 3 monocular depth inference."""

from __future__ import annotations

from collections import deque
import importlib.util
import os
import sys
import threading
import traceback
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
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
        self.declare_parameter('point_cloud_topic', '/front_camera/depth/points')
        self.declare_parameter('da3_repo_path', '')
        self.declare_parameter('model_id', 'depth-anything/DA3-Large')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('process_res', 336)
        self.declare_parameter('process_res_method', 'upper_bound_resize')
        self.declare_parameter('inference_rate_hz', 1.0)
        self.declare_parameter('input_views', 1)
        self.declare_parameter('da3_log_level', 'WARN')
        self.declare_parameter('point_cloud_stride', 4)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 20.0)
        self.declare_parameter('camera_frame', 'front_camera_optical_frame')
        self.declare_parameter('point_cloud_frame', 'front_camera_optical_frame')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lidar_scale', True)
        self.declare_parameter('lidar_scale_min_points', 5)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._input_views = max(int(self.get_parameter('input_views').value), 1)
        # (Image, CameraInfo) 쌍으로 버퍼링 — 이미지 수신 시점의 camera_info를 함께 저장
        self._image_buffer: deque[tuple[Image, CameraInfo]] = deque(maxlen=self._input_views)
        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_scan: Optional[LaserScan] = None
        self._processing = False
        self._model_ready = False
        self._model_error_logged = False
        self._waited_for_views_logged = False

        self._torch = None
        self._model = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
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
        self._points_pub = self.create_publisher(PointCloud2, point_cloud_topic, output_qos)
        self.create_subscription(Image, image_topic, self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)

        rate_hz = max(float(self.get_parameter('inference_rate_hz').value), 0.1)
        self.create_timer(1.0 / rate_hz, self._process_latest)
        threading.Thread(target=self._load_model, daemon=True).start()

        self.get_logger().info(
            f'DA3 wrapper ready. image={image_topic}, info={camera_info_topic}, '
            f'scan={scan_topic}, depth={depth_topic}, points={point_cloud_topic}'
        )

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            # camera_info가 아직 없으면 이 프레임은 버림
            if self._latest_camera_info is not None:
                self._image_buffer.append((msg, self._latest_camera_info))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._latest_camera_info = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    def _process_latest(self) -> None:
        if not self._model_ready or self._processing:
            return

        with self._lock:
            if len(self._image_buffer) < self._input_views:
                if not self._waited_for_views_logged:
                    self._waited_for_views_logged = True
                    self.get_logger().info(
                        f'Waiting for {self._input_views} frames before first DA3 inference '
                        f'(currently {len(self._image_buffer)}).'
                    )
                return
            frames = list(self._image_buffer)
            scan = self._latest_scan
            self._processing = True

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
        try:
            rgbs = [self._bridge.imgmsg_to_cv2(m, desired_encoding='rgb8') for m in image_msgs]
            # 각 프레임의 camera_info를 per-frame으로 사용
            intrinsics_batch = np.stack(
                [self._camera_info_to_matrix(ci) for ci in camera_info_msgs],
                axis=0,
            ).astype(np.float32)
            prediction = self._model.inference(
                rgbs,
                intrinsics=intrinsics_batch,
                process_res=int(self.get_parameter('process_res').value),
                process_res_method=str(self.get_parameter('process_res_method').value),
            )

            # 출력은 마지막 프레임 기준
            image_msg = image_msgs[-1]
            camera_info_msg = camera_info_msgs[-1]
            depth = np.asarray(prediction.depth[-1], dtype=np.float32)
            # GT intrinsics — depth 해상도에 맞게 스케일
            intrinsics = self._gt_intrinsics(camera_info_msg, depth.shape)
            if self._needs_metric_scaling(prediction):
                depth = self._apply_metric_scaling(depth, intrinsics)

            # 라이다 스케일 보정
            if self.get_parameter('lidar_scale').value and scan is not None:
                depth = self._apply_lidar_scale(depth, intrinsics, image_msg, scan)

            self._publish_depth(image_msg, depth)
            self._publish_point_cloud(image_msg, depth, intrinsics)
        except Exception as exc:  # pragma: no cover - runtime inference path
            self.get_logger().error(f'DA3 inference failed: {exc}')
            self.get_logger().error(traceback.format_exc())
        finally:
            with self._lock:
                self._processing = False

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
        model_id = str(self.get_parameter('model_id').value).lower()
        return 'metric' in model_id and not bool(getattr(prediction, 'is_metric', 0))

    def _apply_metric_scaling(self, depth: np.ndarray, intrinsics: np.ndarray) -> np.ndarray:
        focal = 0.5 * float(intrinsics[0, 0] + intrinsics[1, 1])
        return depth * (focal / 300.0)

    def _publish_depth(self, image_msg: Image, depth: np.ndarray) -> None:
        depth_msg = self._bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        depth_msg.header = Header(
            stamp=image_msg.header.stamp,
            frame_id=self._output_frame_id(image_msg.header.frame_id),
        )
        self._depth_pub.publish(depth_msg)

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
    try:
        rclpy.spin(node)
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
