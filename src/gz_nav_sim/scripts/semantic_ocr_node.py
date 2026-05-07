#!/usr/bin/env python3
"""Semantic OCR inspection from the robot RGB camera stream.

This node is intentionally separate from semantic_vlm_node.py. It samples RGB
frames, runs OCR only, filters low-confidence/non-floor-compatible room IDs,
tracks repeated physical signs, and publishes JSON plus Foxglove annotations.
"""

from __future__ import annotations

import copy
import json
import math
import re
import subprocess
import threading
import time
import uuid
from collections import Counter, deque
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from foxglove_msgs.msg import Color, ImageAnnotations, KeyValuePair, Point2, PointsAnnotation, TextAnnotation
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


ROOM_ID_RE = re.compile(r'(?<![0-9A-Z])(?:[A-Z][ -]?)?\d{3,4}(?![0-9A-Z])', re.IGNORECASE)

OCR_REASON = (
    'PaddleOCR is the primary OCR because it returns text, confidence, and '
    'quadrilateral boxes in one pass, supports angle classification for tilted '
    'hallway text, and works well with multi-scale RGB frames.'
)


@dataclass
class OcrJob:
    rgb: np.ndarray
    ocr_rgb: np.ndarray
    coord_scale_x: float
    coord_scale_y: float
    stamp: object
    frame_id: str
    depth_msg: Optional[Image]
    camera_info: Optional[CameraInfo]
    seq: int


@dataclass
class OcrObservation:
    room_id: str
    raw_text: str
    confidence: float
    bbox_xyxy: Optional[list[int]]
    source: str
    frame_seq: int
    timestamp_s: float
    depth_m: Optional[float] = None
    world_xyz: Optional[tuple[float, float, float]] = None
    world_frame_id: str = ''


@dataclass
class OcrTrack:
    id: str
    first_seen_sec: float
    last_seen_sec: float
    first_frame_seq: int
    last_frame_seq: int
    last_bbox_xyxy: Optional[list[int]]
    selected_room_id: str
    selected_confidence: float
    selected_frame_seq: int
    selected_bbox_xyxy: Optional[list[int]]
    selected_depth_m: Optional[float]
    world_xyz: Optional[tuple[float, float, float]]
    frame_id: str
    observations: int = 1
    confirmed: bool = False
    candidate_room_ids: Counter[str] = field(default_factory=Counter)
    evidence_frames: list[int] = field(default_factory=list)


_FLOOR_HINT_RE = re.compile(r'^(B|BASEMENT-?)?\s*(\d+)\s*(F|TH|ST|ND|RD)?$')


def _normalize_floor_hint(value: str | None) -> str | None:
    """Accept any floor input shape (e.g., '4', '4F', 'F4', '13', 'B3', '5F',
    '-3') and produce canonical OCR form: '<n>F' for above-ground, 'B<n>F' for
    basement. Returns None when input is empty/invalid."""
    if value is None:
        return None
    text = str(value).strip().upper().replace(' ', '')
    if not text:
        return None
    # Negative integer = basement (e.g., '-3' → 'B3F')
    if text.startswith('-'):
        rest = text[1:]
        if rest.isdigit() and int(rest) > 0:
            return f'B{int(rest)}F'
        return text
    # 'F4' style → drop the leading F
    if text.startswith('F') and text[1:].isdigit():
        return f'{int(text[1:])}F'
    m = _FLOOR_HINT_RE.match(text)
    if not m:
        return text
    basement = bool(m.group(1))
    n = int(m.group(2))
    if n <= 0:
        return text
    return f'B{n}F' if basement else f'{n}F'


def _apply_floor_prior(room_id: str, floor_hint: str | None, floor_prior_mode: str) -> str | None:
    """Generic prefix filter for any floor.

    Convention: room number = floor digits + 2 trailing digits. So:
        4F   matches 4XX     (3 digits)
        13F  matches 13XX    (4 digits)
        25F  matches 25XX    (4 digits)
        B3F  matches B3XX    ('B' letter + 3 digits) or 3XX in `complete` mode
        B12F matches B12XX   ('B' letter + 4 digits) or 12XX in `complete` mode

    `complete` mode for above-ground floors >=10 also accepts the trailing-N-1
    digits (e.g., 13F + OCR '305' → '1305') in case the leading floor digit
    was occluded — preserves the original VID_*_13F recovery behavior.
    """
    hint = _normalize_floor_hint(floor_hint)
    compact = re.sub(r'[\s-]+', '', room_id.upper())
    match = re.match(r'^([A-Z])?(\d{3,4})$', compact)
    if not match:
        return None
    letter, digits = match.groups()
    letter = letter or ''
    complete = floor_prior_mode == 'complete'

    if not hint:
        return compact

    hm = re.match(r'^(B?)(\d+)F$', hint)
    if not hm:
        return compact  # malformed hint — pass through
    basement = hm.group(1) == 'B'
    floor_str = hm.group(2)
    expected_len = len(floor_str) + 2

    if basement:
        if letter == 'B' and len(digits) == expected_len and digits.startswith(floor_str):
            return f'B{digits}'
        if complete and not letter and len(digits) == expected_len and digits.startswith(floor_str):
            return f'B{digits}'
        return None

    # Above ground
    if not letter and len(digits) == expected_len and digits.startswith(floor_str):
        return digits
    # Trailing-digit recovery (multi-digit floors only): OCR caught the last
    # floor digit + room number but missed the leading floor digits.
    if (
        complete
        and len(floor_str) >= 2
        and not letter
        and len(digits) == expected_len - len(floor_str) + 1
        and digits.startswith(floor_str[-1])
    ):
        return floor_str[:-1] + digits
    return None


def _normalize_room_id(
    text: str | None,
    floor_hint: str | None,
    floor_prior_mode: str,
) -> str | None:
    if text is None:
        return None
    cleaned = re.sub(r'[^0-9A-Za-z가-힣 -]+', ' ', str(text).upper())
    match = ROOM_ID_RE.search(cleaned)
    if not match:
        return None
    return _apply_floor_prior(
        re.sub(r'[\s-]+', '', match.group(0).upper()),
        floor_hint,
        floor_prior_mode,
    )


def _parse_scales(text: str) -> list[float]:
    scales: list[float] = []
    for part in str(text).split(','):
        part = part.strip()
        if not part:
            continue
        try:
            value = max(0.25, min(4.0, float(part)))
        except ValueError:
            continue
        if value not in scales:
            scales.append(value)
    return scales or [1.0]


def _clean_text(text: str) -> str:
    return re.sub(r'\s+', ' ', text.strip())


def _resize_rgb(rgb: np.ndarray, max_side: int) -> tuple[np.ndarray, float, float]:
    h, w = rgb.shape[:2]
    max_side = max(1, int(max_side))
    scale = min(1.0, float(max_side) / float(max(h, w)))
    if scale >= 1.0:
        return rgb, 1.0, 1.0
    out_w = max(1, int(round(w * scale)))
    out_h = max(1, int(round(h * scale)))
    resized = cv2.resize(rgb, (out_w, out_h), interpolation=cv2.INTER_AREA)
    return resized, float(w) / float(out_w), float(h) / float(out_h)


def _scaled_rgb(rgb: np.ndarray, scale: float) -> np.ndarray:
    if abs(scale - 1.0) < 1e-6:
        return rgb
    h, w = rgb.shape[:2]
    return cv2.resize(
        rgb,
        (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
        interpolation=cv2.INTER_CUBIC if scale > 1.0 else cv2.INTER_AREA,
    )


def _clamp_bbox(bbox: Any, w: int, h: int) -> Optional[list[int]]:
    if bbox is None or len(bbox) != 4:
        return None
    try:
        x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
    except Exception:
        return None
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(0, min(w - 1, x2))
    y2 = max(0, min(h - 1, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def _bbox_center(bbox: list[int]) -> tuple[float, float]:
    return (0.5 * float(bbox[0] + bbox[2]), 0.5 * float(bbox[1] + bbox[3]))


def _bbox_diag(bbox: list[int]) -> float:
    return math.hypot(max(1.0, float(bbox[2] - bbox[0])), max(1.0, float(bbox[3] - bbox[1])))


def _bbox_center_distance(a: list[int], b: list[int]) -> float:
    ax, ay = _bbox_center(a)
    bx, by = _bbox_center(b)
    return math.hypot(ax - bx, ay - by)


def _bbox_iou(a: list[int], b: list[int]) -> float:
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return float(inter) / float(max(1, area_a + area_b - inter))


def _confidence_label(confidence: float) -> str:
    if confidence >= 0.85:
        return 'high'
    if confidence >= 0.70:
        return 'medium'
    return 'low'


class SemanticOcrNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_ocr_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/d456/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d456/depth/camera_info')
        self.declare_parameter('detections_topic', '/semantic_ocr/detections')
        self.declare_parameter('markers_topic', '/semantic_ocr/markers')
        self.declare_parameter('image_annotations_topic', '/semantic_ocr/image_annotations')
        self.declare_parameter('ocr_backend', 'paddle')
        self.declare_parameter('ocr_use_gpu', False)
        self.declare_parameter('frame_interval', 5)
        self.declare_parameter('max_queue_size', 32)
        self.declare_parameter('ocr_max_side', 1280)
        self.declare_parameter('ocr_scales', '1.0,2.0')
        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('floor_hint', '')
        self.declare_parameter('floor_prior_mode', 'reject')
        self.declare_parameter('max_depth_m', 12.0)
        self.declare_parameter('depth_window_px', 7)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('fallback_target_frame', 'odom')
        self.declare_parameter('candidate_ttl_s', 300.0)
        self.declare_parameter('confirm_min_observations', 2)
        self.declare_parameter('track_max_gap_frames', 60)
        self.declare_parameter('track_max_center_distance_px', 90.0)
        self.declare_parameter('track_distance_scale', 4.0)
        self.declare_parameter('track_min_iou', 0.05)
        self.declare_parameter('track_max_depth_diff_m', 0.0)
        self.declare_parameter('publish_raw_ocr_output', False)

        # Snapshot tunable parameters once. Hot paths read from self._cfg instead
        # of calling get_parameter() per frame/observation.
        gp = lambda name: self.get_parameter(name).value
        self._cfg = SimpleNamespace(
            image_topic=str(gp('image_topic')),
            depth_topic=str(gp('depth_topic')),
            camera_info_topic=str(gp('camera_info_topic')),
            detections_topic=str(gp('detections_topic')),
            markers_topic=str(gp('markers_topic')),
            image_annotations_topic=str(gp('image_annotations_topic')),
            ocr_backend=str(gp('ocr_backend')).strip().lower() or 'paddle',
            ocr_use_gpu=bool(gp('ocr_use_gpu')),
            frame_interval=max(1, int(gp('frame_interval'))),
            max_queue_size=max(1, int(gp('max_queue_size'))),
            ocr_max_side=int(gp('ocr_max_side')),
            ocr_scales=_parse_scales(str(gp('ocr_scales'))),
            min_confidence=max(0.0, float(gp('min_confidence'))),
            floor_hint=str(gp('floor_hint')).strip() or None,
            floor_prior_mode=(str(gp('floor_prior_mode')).strip().lower()
                              if str(gp('floor_prior_mode')).strip().lower() in ('reject', 'complete')
                              else 'reject'),
            max_depth_m=max(0.1, float(gp('max_depth_m'))),
            depth_window_px=max(1, int(gp('depth_window_px'))),
            target_frame=str(gp('target_frame')),
            fallback_target_frame=str(gp('fallback_target_frame')),
            candidate_ttl_s=max(1.0, float(gp('candidate_ttl_s'))),
            confirm_min_observations=max(1, int(gp('confirm_min_observations'))),
            track_max_gap_frames=max(1, int(gp('track_max_gap_frames'))),
            track_max_center_distance_px=max(1.0, float(gp('track_max_center_distance_px'))),
            track_distance_scale=max(0.1, float(gp('track_distance_scale'))),
            track_min_iou=max(0.0, float(gp('track_min_iou'))),
            track_max_depth_diff_m=max(0.0, float(gp('track_max_depth_diff_m'))),
            publish_raw_ocr_output=bool(gp('publish_raw_ocr_output')),
            gpu_sample_interval=10,  # nvidia-smi every N OCR jobs
        )

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_depth: Optional[Image] = None
        self._latest_info: Optional[CameraInfo] = None
        self._frame_seq = 0
        self._backend = None
        self._backend_name = ''
        self._backend_version: Optional[str] = None
        self._backend_error: Optional[str] = None
        self._load_lock = threading.Lock()
        self._job_cv = threading.Condition()
        self._jobs: deque[OcrJob] = deque()
        self._stop_worker = False
        self._tracks: list[OcrTrack] = []
        self._started_monotonic = time.monotonic()
        self._processed_frames = 0
        self._dropped_jobs = 0
        self._total_inference_s = 0.0
        self._gpu_memory_samples: list[float] = []
        self._last_gpu_snapshot: Optional[dict[str, float]] = None

        # Live-updatable params: clients (e.g., the ros_adapter web bridge)
        # call `ros2 param set /semantic_ocr_node floor_hint 4F` whenever a
        # new web session picks a floor. Refresh the cache so subsequent OCR
        # frames apply the new prior without restart.
        self.add_on_set_parameters_callback(self._on_param_change)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Image, self._cfg.image_topic, self._on_image, sensor_qos)
        self.create_subscription(Image, self._cfg.depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self._cfg.camera_info_topic, self._on_camera_info, sensor_qos)

        self._pub_json = self.create_publisher(String, self._cfg.detections_topic, out_qos)
        self._pub_markers = self.create_publisher(MarkerArray, self._cfg.markers_topic, out_qos)
        self._pub_image_annotations = self.create_publisher(
            ImageAnnotations, self._cfg.image_annotations_topic, out_qos)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'semantic OCR ready. image=%s depth=%s interval=%s backend=%s conf>%.2f scales=%s' % (
                self._cfg.image_topic,
                self._cfg.depth_topic,
                self._cfg.frame_interval,
                self._cfg.ocr_backend,
                self._cfg.min_confidence,
                self._cfg.ocr_scales,
            )
        )

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_info = msg

    def _on_image(self, msg: Image) -> None:
        self._frame_seq += 1
        if self._frame_seq % self._cfg.frame_interval != 0:
            return

        try:
            # cv_bridge ndarray may share memory with the message buffer; copy
            # once so the worker thread is safe after this callback returns.
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8').copy()
        except Exception as exc:
            self.get_logger().warn(f'failed to convert RGB frame for OCR: {exc}')
            return

        ocr_rgb, coord_scale_x, coord_scale_y = _resize_rgb(rgb, self._cfg.ocr_max_side)
        job = OcrJob(
            rgb=rgb,
            ocr_rgb=ocr_rgb,
            coord_scale_x=coord_scale_x,
            coord_scale_y=coord_scale_y,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
            depth_msg=self._latest_depth,
            camera_info=self._latest_info,
            seq=self._frame_seq,
        )
        with self._job_cv:
            if len(self._jobs) >= self._cfg.max_queue_size:
                self._jobs.popleft()
                self._dropped_jobs += 1
            self._jobs.append(job)
            self._job_cv.notify()

    def _worker_loop(self) -> None:
        while True:
            with self._job_cv:
                while not self._jobs and not self._stop_worker:
                    self._job_cv.wait(timeout=0.2)
                if self._stop_worker and not self._jobs:
                    return
                job = self._jobs.popleft()
            self._run_job(job)

    def _run_job(self, job: OcrJob) -> None:
        start = time.monotonic()
        raw_ocr: list[dict[str, Any]] = []
        try:
            observations, raw_ocr = self._infer_ocr(job)
        except Exception as exc:
            self.get_logger().error(f'OCR inference failed: {exc}')
            return

        observations = self._dedupe_same_frame_observations(observations)
        now_sec = self._stamp_sec(job.stamp)

        # Decode depth once per job; reused across all observations.
        depth_array = self._decode_depth(job.depth_msg)
        depth_scale = self._depth_scale(job.depth_msg)

        enriched_objects: list[dict[str, Any]] = []
        for obs in observations:
            obs.depth_m = (
                self._sample_depth(obs.bbox_xyxy, depth_array, depth_scale)
                if obs.bbox_xyxy is not None else None
            )
            obs.world_xyz, obs.world_frame_id = self._position_observation(obs, job)
            track = self._update_track(obs, now_sec)
            enriched_objects.append(self._observation_to_dict(obs, track))

        self._prune_tracks(now_sec)
        inference_s = time.monotonic() - start
        self._processed_frames += 1
        self._total_inference_s += inference_s

        # nvidia-smi is slow (50–100 ms). Sample every N OCR jobs and reuse.
        if self._processed_frames % max(1, self._cfg.gpu_sample_interval) == 1:
            self._last_gpu_snapshot = self._gpu_snapshot()
        vram = self._last_gpu_snapshot
        if vram and vram.get('memory_used_mb') is not None:
            self._gpu_memory_samples.append(float(vram['memory_used_mb']))

        height, width = job.rgb.shape[:2]
        payload = {
            'task_mode': 'ocr_room_ids',
            'has_text_object': bool(enriched_objects),
            'objects': enriched_objects,
            'control_summary_ko': self._control_summary_ko(enriched_objects),
            'need_human_check': False,
            'annotations': [self._track_to_dict(track) for track in self._tracks],
            'metadata': {
                'source_stamp': self._stamp_dict(job.stamp),
                'source_frame': job.frame_id,
                'image_width': width,
                'image_height': height,
                'frame_seq': job.seq,
                'ocr_backend': self._backend_name,
                'ocr_backend_version': self._backend_version,
                'ocr_model': self._ocr_model_description(),
                'why_this_ocr': OCR_REASON if self._backend_name == 'paddle' else 'Fallback OCR backend.',
                'min_confidence': self._cfg.min_confidence,
                'floor_hint': _normalize_floor_hint(self._cfg.floor_hint),
                'floor_prior_mode': self._cfg.floor_prior_mode,
                'latency_s': round(inference_s, 3),
                'stats': self._stats(),
                'vram': vram,
            },
        }
        if self._backend_error:
            payload['metadata']['backend_error'] = self._backend_error
        if self._cfg.publish_raw_ocr_output:
            payload['raw_ocr_output'] = raw_ocr

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._pub_json.publish(msg)
        self._pub_markers.publish(self._make_markers(job.stamp))
        self._pub_image_annotations.publish(self._make_image_annotations(job.stamp, enriched_objects))
        self.get_logger().info(
            f'OCR frame={job.seq} objects={len(enriched_objects)} tracks={len(self._tracks)} '
            f'latency={inference_s:.2f}s queue={len(self._jobs)}'
        )

    def _ensure_backend(self) -> None:
        if self._backend_name:
            return
        with self._load_lock:
            if self._backend_name:
                return

            requested = self._cfg.ocr_backend
            if requested not in ('paddle', 'tesseract'):
                requested = 'paddle'
            if requested == 'paddle':
                try:
                    from paddleocr import PaddleOCR
                    import paddleocr

                    try:
                        self._backend = PaddleOCR(
                            use_angle_cls=True,
                            lang='en',
                            use_gpu=self._cfg.ocr_use_gpu,
                            show_log=False,
                        )
                    except TypeError:
                        self._backend = PaddleOCR(use_angle_cls=True, lang='en')
                    self._backend_name = 'paddle'
                    version = getattr(paddleocr, '__version__', None)
                    self._backend_version = str(version) if version is not None else None
                    return
                except Exception as exc:
                    self._backend_error = str(exc)
                    self.get_logger().warn(f'PaddleOCR unavailable, falling back to tesseract: {exc}')

            try:
                import pytesseract

                self._backend = pytesseract
                self._backend_name = 'tesseract'
                try:
                    self._backend_version = str(pytesseract.get_tesseract_version())
                except Exception:
                    self._backend_version = None
            except Exception as exc:
                self._backend_name = 'none'
                self._backend_error = str(exc)
                raise RuntimeError(f'no OCR backend available: {exc}') from exc

    def _infer_ocr(self, job: OcrJob) -> tuple[list[OcrObservation], list[dict[str, Any]]]:
        self._ensure_backend()
        h_ocr, w_ocr = job.ocr_rgb.shape[:2]
        h, w = job.rgb.shape[:2]
        raw_detections: list[dict[str, Any]] = []
        if self._backend_name == 'paddle':
            for scale in self._cfg.ocr_scales:
                rgb_in = _scaled_rgb(job.ocr_rgb, scale)
                out = self._backend.ocr(rgb_in, cls=True)
                lines = out[0] if out and isinstance(out[0], list) else out
                for line in lines or []:
                    try:
                        pts = np.asarray(line[0], dtype=np.float32) / float(scale)
                        text = _clean_text(str(line[1][0]))
                        conf = float(line[1][1])
                    except Exception:
                        continue
                    bbox_ocr = _clamp_bbox([
                        float(pts[:, 0].min()),
                        float(pts[:, 1].min()),
                        float(pts[:, 0].max()),
                        float(pts[:, 1].max()),
                    ], w_ocr, h_ocr)
                    bbox = None
                    if bbox_ocr is not None:
                        bbox = _clamp_bbox([
                            bbox_ocr[0] * job.coord_scale_x,
                            bbox_ocr[1] * job.coord_scale_y,
                            bbox_ocr[2] * job.coord_scale_x,
                            bbox_ocr[3] * job.coord_scale_y,
                        ], w, h)
                    raw_detections.append({
                        'source': f'paddleocr@{scale:g}x',
                        'text': text,
                        'confidence': conf,
                        'bbox_xyxy': bbox,
                    })
        elif self._backend_name == 'tesseract':
            from PIL import Image as PILImage

            data = self._backend.image_to_data(
                PILImage.fromarray(job.ocr_rgb),
                lang='eng',
                config='--oem 3 --psm 11 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-',
                output_type=self._backend.Output.DICT,
            )
            for i, text in enumerate(data.get('text', [])):
                text = _clean_text(str(text))
                if not text:
                    continue
                try:
                    conf = float(data.get('conf', ['-1'])[i])
                except Exception:
                    conf = -1.0
                if conf < 25.0:
                    continue
                x = int(data['left'][i])
                y = int(data['top'][i])
                bw = int(data['width'][i])
                bh = int(data['height'][i])
                bbox_ocr = _clamp_bbox([x, y, x + bw, y + bh], w_ocr, h_ocr)
                bbox = None
                if bbox_ocr is not None:
                    bbox = _clamp_bbox([
                        bbox_ocr[0] * job.coord_scale_x,
                        bbox_ocr[1] * job.coord_scale_y,
                        bbox_ocr[2] * job.coord_scale_x,
                        bbox_ocr[3] * job.coord_scale_y,
                    ], w, h)
                raw_detections.append({
                    'source': 'pytesseract',
                    'text': text,
                    'confidence': conf / 100.0,
                    'bbox_xyxy': bbox,
                })

        floor_hint = self._cfg.floor_hint
        prior_mode = self._cfg.floor_prior_mode
        min_confidence = self._cfg.min_confidence
        observations: list[OcrObservation] = []
        stamp_s = self._stamp_sec(job.stamp)
        for det in raw_detections:
            try:
                conf = float(det.get('confidence'))
            except Exception:
                conf = 0.0
            if conf <= min_confidence:
                continue
            raw_text = str(det.get('text') or '')
            room_id = _normalize_room_id(raw_text, floor_hint, prior_mode)
            if not room_id:
                continue
            observations.append(OcrObservation(
                room_id=room_id,
                raw_text=raw_text,
                confidence=conf,
                bbox_xyxy=copy.deepcopy(det.get('bbox_xyxy')),
                source=str(det.get('source') or self._backend_name),
                frame_seq=job.seq,
                timestamp_s=stamp_s,
            ))
        return observations, raw_detections

    def _dedupe_same_frame_observations(self, observations: list[OcrObservation]) -> list[OcrObservation]:
        groups: list[list[OcrObservation]] = []
        for obs in sorted(observations, key=lambda item: -item.confidence):
            if not obs.bbox_xyxy:
                groups.append([obs])
                continue
            matched: Optional[list[OcrObservation]] = None
            for group in groups:
                anchor = group[0]
                if not anchor.bbox_xyxy:
                    continue
                center_distance = _bbox_center_distance(obs.bbox_xyxy, anchor.bbox_xyxy)
                allowed_distance = max(
                    8.0,
                    0.75 * max(_bbox_diag(obs.bbox_xyxy), _bbox_diag(anchor.bbox_xyxy)),
                )
                if center_distance <= allowed_distance or _bbox_iou(obs.bbox_xyxy, anchor.bbox_xyxy) >= 0.35:
                    matched = group
                    break
            if matched is None:
                groups.append([obs])
            else:
                matched.append(obs)
        kept = [max(group, key=lambda item: item.confidence) for group in groups]
        return sorted(kept, key=lambda item: (item.frame_seq, item.room_id, -item.confidence))

    def _update_track(self, obs: OcrObservation, now_sec: float) -> OcrTrack:
        best_track: Optional[OcrTrack] = None
        best_score = float('inf')
        max_gap = self._cfg.track_max_gap_frames
        max_center = self._cfg.track_max_center_distance_px
        distance_scale = self._cfg.track_distance_scale
        min_iou = self._cfg.track_min_iou
        max_depth_diff = self._cfg.track_max_depth_diff_m

        if obs.bbox_xyxy:
            for track in self._tracks:
                if track.last_bbox_xyxy is None:
                    continue
                gap = obs.frame_seq - track.last_frame_seq
                if gap <= 0 or gap > max_gap:
                    continue
                depth_penalty = 0.0
                if max_depth_diff > 0.0 and obs.depth_m is not None and track.selected_depth_m is not None:
                    depth_diff = abs(float(obs.depth_m) - float(track.selected_depth_m))
                    if depth_diff > max_depth_diff:
                        continue
                    depth_penalty = depth_diff / max(1e-6, max_depth_diff)
                center_distance = _bbox_center_distance(obs.bbox_xyxy, track.last_bbox_xyxy)
                allowed_distance = max(
                    max_center,
                    distance_scale * max(_bbox_diag(obs.bbox_xyxy), _bbox_diag(track.last_bbox_xyxy)),
                )
                overlap = _bbox_iou(obs.bbox_xyxy, track.last_bbox_xyxy)
                if center_distance <= allowed_distance or overlap >= min_iou:
                    score = (
                        (center_distance / max(1.0, allowed_distance))
                        + (gap / max(1, max_gap))
                        + depth_penalty
                    )
                    if score < best_score:
                        best_score = score
                        best_track = track

        if best_track is None:
            track = OcrTrack(
                id=uuid.uuid4().hex[:12],
                first_seen_sec=now_sec,
                last_seen_sec=now_sec,
                first_frame_seq=obs.frame_seq,
                last_frame_seq=obs.frame_seq,
                last_bbox_xyxy=copy.deepcopy(obs.bbox_xyxy),
                selected_room_id=obs.room_id,
                selected_confidence=obs.confidence,
                selected_frame_seq=obs.frame_seq,
                selected_bbox_xyxy=copy.deepcopy(obs.bbox_xyxy),
                selected_depth_m=obs.depth_m,
                world_xyz=obs.world_xyz,
                frame_id=obs.world_frame_id,
                candidate_room_ids=Counter({obs.room_id: 1}),
                evidence_frames=[obs.frame_seq],
            )
            self._tracks.append(track)
            best_track = track
        else:
            best_track.observations += 1
            best_track.last_seen_sec = now_sec
            best_track.last_frame_seq = obs.frame_seq
            best_track.last_bbox_xyxy = copy.deepcopy(obs.bbox_xyxy)
            best_track.candidate_room_ids.update([obs.room_id])
            best_track.evidence_frames = sorted(set(best_track.evidence_frames + [obs.frame_seq]))[-120:]
            if obs.confidence > best_track.selected_confidence:
                best_track.selected_room_id = obs.room_id
                best_track.selected_confidence = obs.confidence
                best_track.selected_frame_seq = obs.frame_seq
                best_track.selected_bbox_xyxy = copy.deepcopy(obs.bbox_xyxy)
                best_track.selected_depth_m = obs.depth_m
            if obs.world_xyz is not None:
                if best_track.world_xyz is None:
                    best_track.world_xyz = obs.world_xyz
                else:
                    old = np.asarray(best_track.world_xyz, dtype=np.float64)
                    new = np.asarray(obs.world_xyz, dtype=np.float64)
                    best_track.world_xyz = tuple(float(v) for v in (0.7 * old + 0.3 * new))
                best_track.frame_id = obs.world_frame_id

        if best_track.observations >= self._cfg.confirm_min_observations:
            best_track.confirmed = True
        return best_track

    def _prune_tracks(self, now_sec: float) -> None:
        ttl = self._cfg.candidate_ttl_s
        self._tracks = [
            track for track in self._tracks
            if track.confirmed or now_sec - track.last_seen_sec <= ttl
        ]

    def _position_observation(
        self,
        obs: OcrObservation,
        job: OcrJob,
    ) -> tuple[Optional[tuple[float, float, float]], str]:
        if obs.bbox_xyxy is None or obs.depth_m is None or job.camera_info is None:
            return None, ''
        k = job.camera_info.k
        fx, fy = float(k[0]), float(k[4])
        cx, cy = float(k[2]), float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None, ''
        x1, y1, x2, y2 = obs.bbox_xyxy
        u = (float(x1) + float(x2)) * 0.5
        v = (float(y1) + float(y2)) * 0.5
        z = float(obs.depth_m)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        source_frame = job.camera_info.header.frame_id or job.frame_id
        for target in (self._cfg.target_frame, self._cfg.fallback_target_frame):
            if not target:
                continue
            world = self._transform_point((x, y, z), source_frame, target, job.stamp)
            if world is not None:
                return world, target
        return (float(x), float(y), float(z)), source_frame

    def _decode_depth(self, depth_msg: Optional[Image]) -> Optional[np.ndarray]:
        if depth_msg is None:
            return None
        try:
            return self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'failed to convert depth image: {exc}')
            return None

    @staticmethod
    def _depth_scale(depth_msg: Optional[Image]) -> float:
        if depth_msg is None:
            return 1.0
        encoding = (depth_msg.encoding or '').lower()
        return 0.001 if ('16u' in encoding or 'mono16' in encoding) else 1.0

    def _sample_depth(
        self,
        bbox: Optional[list[int]],
        depth: Optional[np.ndarray],
        scale: float,
    ) -> Optional[float]:
        if bbox is None or depth is None:
            return None
        x1, y1, x2, y2 = bbox
        u = int(round((x1 + x2) * 0.5))
        v = int(round((y1 + y2) * 0.5))
        radius = self._cfg.depth_window_px
        y0, y3 = max(0, v - radius), min(depth.shape[0], v + radius + 1)
        x0, x3 = max(0, u - radius), min(depth.shape[1], u + radius + 1)
        patch = depth[y0:y3, x0:x3]
        if patch.size == 0:
            return None
        values = patch.astype(np.float32, copy=False).reshape(-1) * scale
        max_depth = self._cfg.max_depth_m
        values = values[np.isfinite(values)]
        values = values[(values > 0.05) & (values <= max_depth)]
        if values.size == 0:
            return None
        return float(np.median(values))

    def _transform_point(
        self,
        xyz: tuple[float, float, float],
        source_frame: str,
        target_frame: str,
        stamp,
    ) -> Optional[tuple[float, float, float]]:
        if not source_frame or not target_frame:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException:
                return None
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = self._quat_to_rot(q.x, q.y, q.z, q.w)
        point = rot @ np.asarray(xyz, dtype=np.float64) + np.asarray(
            [t.x, t.y, t.z], dtype=np.float64)
        return tuple(float(v) for v in point)

    def _observation_to_dict(self, obs: OcrObservation, track: OcrTrack) -> dict[str, Any]:
        return {
            'type': 'room_id_sign',
            'room_id': obs.room_id,
            'text': obs.room_id,
            'raw_text': obs.raw_text,
            'confidence': round(float(obs.confidence), 4),
            'confidence_label': _confidence_label(obs.confidence),
            'bbox_xyxy': obs.bbox_xyxy,
            'source': obs.source,
            'track_id': track.id,
            'annotation_status': 'confirmed' if track.confirmed else 'candidate',
            'track_observations': track.observations,
            'depth_m': round(obs.depth_m, 3) if obs.depth_m is not None else None,
            'world_xyz': list(obs.world_xyz) if obs.world_xyz is not None else None,
            'frame_id': obs.world_frame_id,
        }

    def _track_to_dict(self, track: OcrTrack) -> dict[str, Any]:
        return {
            'id': track.id,
            'type': 'room_id_sign',
            'selected_room_id': track.selected_room_id,
            'selected_confidence': round(float(track.selected_confidence), 4),
            'selected_confidence_label': _confidence_label(track.selected_confidence),
            'selected_frame_seq': track.selected_frame_seq,
            'selected_bbox_xyxy': track.selected_bbox_xyxy,
            'selected_depth_m': round(track.selected_depth_m, 3) if track.selected_depth_m is not None else None,
            'annotation_status': 'confirmed' if track.confirmed else 'candidate',
            'observations': track.observations,
            'candidate_room_ids': dict(sorted(track.candidate_room_ids.items())),
            'first_frame_seq': track.first_frame_seq,
            'last_frame_seq': track.last_frame_seq,
            'evidence_frames': track.evidence_frames,
            'world_xyz': list(track.world_xyz) if track.world_xyz is not None else None,
            'frame_id': track.frame_id,
        }

    def _control_summary_ko(self, objects: list[dict[str, Any]]) -> str:
        if not objects:
            return 'OCR에서 신뢰도 기준을 넘는 표지판 텍스트가 없습니다.'
        ids = ', '.join(str(obj.get('room_id')) for obj in objects[:5])
        extra = '' if len(objects) <= 5 else f' 외 {len(objects) - 5}개'
        return f'OCR 표지판 후보: {ids}{extra}'

    def _make_markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        target_frame = self._cfg.target_frame
        delete = Marker()
        delete.header.frame_id = target_frame
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        marker_id = 1
        for track in self._tracks:
            if track.world_xyz is None:
                continue
            x, y, z = track.world_xyz
            color = (0.1, 0.9, 0.2, 0.9) if track.confirmed else (1.0, 0.75, 0.1, 0.75)

            sphere = Marker()
            sphere.header.frame_id = track.frame_id or target_frame
            sphere.header.stamp = stamp
            sphere.ns = 'semantic_ocr_points'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=x, y=y, z=z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            markers.markers.append(sphere)

            label = Marker()
            label.header = sphere.header
            label.ns = 'semantic_ocr_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=x, y=y, z=z + 0.35)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.24
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            status = 'confirmed' if track.confirmed else 'candidate'
            label.text = f'{status}: OCR {track.selected_room_id} ({track.selected_confidence:.2f})'
            markers.markers.append(label)
        return markers

    def _make_image_annotations(self, stamp, objects: list[dict[str, Any]]) -> ImageAnnotations:
        msg = ImageAnnotations()
        msg.timestamp = stamp

        for index, obj in enumerate(objects):
            bbox = obj.get('bbox_xyxy')
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [float(v) for v in bbox]
            confidence = float(obj.get('confidence') or 0.0)
            outline = self._annotation_color(confidence)
            fill = Color(r=outline.r, g=outline.g, b=outline.b, a=0.10)

            box = PointsAnnotation()
            box.timestamp = stamp
            box.type = PointsAnnotation.LINE_LOOP
            box.points = [
                Point2(x=x1, y=y1),
                Point2(x=x2, y=y1),
                Point2(x=x2, y=y2),
                Point2(x=x1, y=y2),
            ]
            box.outline_color = outline
            box.fill_color = fill
            box.thickness = 2.0
            box.metadata = [
                KeyValuePair(key='type', value='room_id_sign'),
                KeyValuePair(key='room_id', value=str(obj.get('room_id') or '')),
                KeyValuePair(key='confidence', value=f'{confidence:.4f}'),
                KeyValuePair(key='source', value=str(obj.get('source') or '')),
                KeyValuePair(key='track_id', value=str(obj.get('track_id') or '')),
            ]
            msg.points.append(box)

            label = TextAnnotation()
            label.timestamp = stamp
            label.position = Point2(x=x1, y=max(0.0, y1 - 6.0))
            label.text = f"{obj.get('room_id', 'OCR')} {confidence:.2f}"
            label.font_size = 14.0
            label.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
            label.background_color = Color(r=0.0, g=0.0, b=0.0, a=0.65)
            label.metadata = [KeyValuePair(key='index', value=str(index))]
            msg.texts.append(label)
        return msg

    @staticmethod
    def _annotation_color(confidence: float) -> Color:
        if confidence >= 0.85:
            return Color(r=0.10, g=0.90, b=0.20, a=1.0)
        if confidence >= 0.70:
            return Color(r=1.00, g=0.75, b=0.10, a=1.0)
        return Color(r=0.95, g=0.25, b=0.25, a=1.0)

    def _ocr_model_description(self) -> str:
        if self._backend_name == 'paddle':
            suffix = f' {self._backend_version}' if self._backend_version else ''
            return f'PaddleOCR{suffix}(lang=en, use_angle_cls=True)'
        if self._backend_name == 'tesseract':
            suffix = f' {self._backend_version}' if self._backend_version else ''
            return f'Tesseract OCR{suffix}(lang=eng, psm=11, alphanumeric whitelist)'
        return self._backend_name or 'not_loaded'

    def _stats(self) -> dict[str, Any]:
        elapsed_s = max(0.0, time.monotonic() - self._started_monotonic)
        processed = float(self._processed_frames)
        fps_effective = processed / elapsed_s if elapsed_s > 0.0 else 0.0
        fps_inference = processed / self._total_inference_s if self._total_inference_s > 0.0 else 0.0
        return {
            'processed_frames': self._processed_frames,
            'sampled_frame_interval': self._cfg.frame_interval,
            'dropped_jobs': self._dropped_jobs,
            'queue_depth': len(self._jobs),
            'elapsed_s': round(elapsed_s, 3),
            'ocr_inference_s': round(self._total_inference_s, 3),
            'effective_fps': round(fps_effective, 3),
            'ocr_inference_fps': round(fps_inference, 3),
            'gpu_memory_used_mb': self._gpu_memory_stats(),
        }

    def _gpu_memory_stats(self) -> dict[str, Optional[float]]:
        if not self._gpu_memory_samples:
            return {'peak': None, 'mean': None}
        return {
            'peak': round(max(self._gpu_memory_samples), 1),
            'mean': round(float(np.mean(self._gpu_memory_samples)), 1),
        }

    @staticmethod
    def _gpu_snapshot() -> Optional[dict[str, float]]:
        try:
            proc = subprocess.run(
                [
                    'nvidia-smi',
                    '--query-gpu=memory.used,utilization.gpu',
                    '--format=csv,noheader,nounits',
                ],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=1.0,
            )
        except Exception:
            return None
        if proc.returncode != 0:
            return None
        mem_total = 0.0
        util_vals: list[float] = []
        for line in proc.stdout.strip().splitlines():
            parts = [part.strip() for part in line.split(',')]
            if len(parts) >= 2:
                try:
                    mem_total += float(parts[0])
                    util_vals.append(float(parts[1]))
                except ValueError:
                    continue
        return {
            'memory_used_mb': round(mem_total, 1),
            'util_pct': round(float(np.mean(util_vals)), 1) if util_vals else 0.0,
        }

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _stamp_dict(stamp) -> dict[str, int]:
        return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}

    @staticmethod
    def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
        n = x * x + y * y + z * z + w * w
        if n <= 0.0:
            return np.eye(3)
        s = 2.0 / n
        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s
        return np.asarray([
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ], dtype=np.float64)

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'floor_hint':
                self._cfg.floor_hint = str(p.value).strip() or None
                self.get_logger().info(f'floor_hint updated: {self._cfg.floor_hint!r}')
            elif p.name == 'floor_prior_mode':
                mode = str(p.value).strip().lower()
                self._cfg.floor_prior_mode = mode if mode in ('reject', 'complete') else 'reject'
                self.get_logger().info(f'floor_prior_mode updated: {self._cfg.floor_prior_mode!r}')
        return SetParametersResult(successful=True)

    def destroy_node(self) -> None:
        with self._job_cv:
            self._stop_worker = True
            self._job_cv.notify_all()
        if hasattr(self, '_worker') and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SemanticOcrNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
