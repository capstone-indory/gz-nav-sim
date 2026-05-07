"""Browser-side Isaac Sim sensor viewer.

This stays outside ROS. It subscribes to the Isaac ZMQ PUB socket, renders the
live sensor topics over HTTP, and optionally runs OCR on rgb.front. OCR results
are fused with depth.front and projected into a robot-local 2D scan map.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import msgpack
import zmq

try:
    import cv2
    import numpy as np

    HAVE_CV = True
except ImportError:
    HAVE_CV = False

try:
    import zstandard

    HAVE_ZSTD = True
except ImportError:
    HAVE_ZSTD = False


ROOM_ID_RE = re.compile(r"(?<![0-9A-Z])(?:[A-Z][ -]?)?\d{2,4}(?![0-9A-Z])", re.IGNORECASE)


def normalize_floor_hint(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip().upper().replace(" ", "")
    if not text:
        return None
    b_match = re.fullmatch(r"(?:B|BASEMENT)(\d+)(?:F)?", text)
    if b_match:
        return f"B{int(b_match.group(1))}F"
    f_match = re.fullmatch(r"(?:F)?(\d{1,2})(?:F|TH)?", text)
    if f_match:
        return f"{int(f_match.group(1))}F"
    return text


def apply_floor_prior(room_id: str, floor_hint: Optional[str], floor_prior_mode: str) -> Optional[str]:
    hint = normalize_floor_hint(floor_hint)
    compact = re.sub(r"[\s-]+", "", str(room_id).upper())
    match = re.fullmatch(r"([A-Z])?(\d{2,4})", compact)
    if not match:
        return None
    letter, digits = match.groups()
    letter = letter or ""
    complete = floor_prior_mode == "complete"

    if hint and hint.startswith("B") and hint.endswith("F"):
        floor_digits = hint[1:-1]
        if letter == "B" and digits.startswith(floor_digits):
            return f"B{digits}"
        if complete and not letter and digits.startswith(floor_digits):
            return f"B{digits}"
        return None

    if hint and hint.endswith("F"):
        floor_digits = hint[:-1]
        if not floor_digits.isdigit() or letter:
            return None
        if len(digits) >= 3 and digits.startswith(floor_digits):
            return digits
        if complete:
            if len(floor_digits) == 1 and len(digits) == 2:
                return f"{floor_digits}{digits}"
            if len(floor_digits) == 2 and len(digits) == 3 and digits.startswith(floor_digits[-1]):
                return f"{floor_digits}{digits[1:]}"
        return None

    return compact if len(digits) >= 3 else None


def normalize_room_id(text: Optional[str], floor_hint: Optional[str], floor_prior_mode: str) -> Optional[str]:
    if text is None:
        return None
    cleaned = re.sub(r"[^0-9A-Za-z가-힣 -]+", " ", str(text).upper())
    match = ROOM_ID_RE.search(cleaned)
    if not match:
        return None
    return apply_floor_prior(match.group(0), floor_hint, floor_prior_mode)


def clean_ocr_text(text: str) -> str:
    return re.sub(r"\s+", " ", str(text).strip())


def parse_scales(text: str) -> list[float]:
    scales: list[float] = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        try:
            scale = max(0.25, min(6.0, float(part)))
        except ValueError:
            continue
        if scale not in scales:
            scales.append(scale)
    return scales or [1.0]


def scaled_rgb_for_ocr(rgb: "np.ndarray", scale: float) -> "np.ndarray":
    if abs(scale - 1.0) < 1e-6:
        return rgb
    h, w = rgb.shape[:2]
    return cv2.resize(
        rgb,
        (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
        interpolation=cv2.INTER_CUBIC if scale > 1.0 else cv2.INTER_AREA,
    )


def clamp_bbox(bbox: list[float] | tuple[float, ...] | None, w: int, h: int) -> Optional[list[int]]:
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


def bbox_iou(a: list[int], b: list[int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1, (bx2 - bx1) * (by2 - by1))
    return float(inter) / float(area_a + area_b - inter)


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, dict] = {}
        self._analysis: dict = {"status": "starting", "detections": []}
        self._overlay_jpeg: Optional[bytes] = None
        self._map_jpeg: Optional[bytes] = None
        self._derived_stamp_ns = 0
        self._reset_generation = 0

    def put(self, topic: str, msg: dict) -> None:
        with self._lock:
            self._frames[topic] = msg

    def get(self, topic: str) -> Optional[dict]:
        with self._lock:
            return self._frames.get(topic)

    def snapshot(self) -> dict[str, dict]:
        with self._lock:
            return dict(self._frames)

    def keys(self) -> list[str]:
        with self._lock:
            return sorted(self._frames)

    def get_first_prefix(self, prefix: str) -> Optional[dict]:
        with self._lock:
            for key in sorted(self._frames):
                if key.startswith(prefix):
                    return self._frames[key]
            return None

    def set_derived(
        self,
        analysis: dict,
        overlay_jpeg: Optional[bytes],
        map_jpeg: Optional[bytes],
        stamp_ns: int,
    ) -> None:
        with self._lock:
            self._analysis = analysis
            self._overlay_jpeg = overlay_jpeg
            self._map_jpeg = map_jpeg
            self._derived_stamp_ns = int(stamp_ns)

    def get_analysis(self) -> dict:
        with self._lock:
            return dict(self._analysis)

    def get_derived_jpeg(self, name: str) -> tuple[Optional[bytes], int]:
        with self._lock:
            if name == "rgb.front.ocr":
                return self._overlay_jpeg, self._derived_stamp_ns
            if name == "local_map":
                return self._map_jpeg, self._derived_stamp_ns
            return None, 0

    def reset_local(self) -> int:
        with self._lock:
            self._frames.clear()
            self._analysis = {
                "status": "reset",
                "detections": [],
                "map_annotations": [],
                "topics": [],
            }
            self._overlay_jpeg = None
            self._map_jpeg = None
            self._derived_stamp_ns = time.time_ns()
            self._reset_generation += 1
            return self._reset_generation

    def reset_generation(self) -> int:
        with self._lock:
            return self._reset_generation


def reader_thread(host: str, port: int, store: FrameStore, stop: threading.Event) -> None:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{host}:{port}")
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVTIMEO, 500)
    sock.setsockopt(zmq.RCVHWM, 8)
    try:
        while not stop.is_set():
            try:
                topic_b, payload = sock.recv_multipart()
            except zmq.Again:
                continue
            try:
                msg = msgpack.unpackb(payload, raw=False)
            except Exception:
                continue
            store.put(topic_b.decode("utf-8", errors="replace"), msg)
    finally:
        sock.close(linger=0)


def rgb_array(msg: Optional[dict]) -> Optional["np.ndarray"]:
    if not HAVE_CV or msg is None or msg.get("encoding") != "jpeg":
        return None
    data = msg.get("data")
    if not isinstance(data, (bytes, bytearray)):
        return None
    arr = np.frombuffer(bytes(data), dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def depth_u16_mm(msg: Optional[dict]) -> Optional["np.ndarray"]:
    if not (HAVE_CV and HAVE_ZSTD) or msg is None:
        return None
    if msg.get("encoding") != "u16_zstd":
        return None
    try:
        raw = zstandard.decompress(bytes(msg["data"]))
        arr = np.frombuffer(raw, dtype=np.uint16)
        return arr.reshape(int(msg["height"]), int(msg["width"]))
    except Exception:
        return None


def rgb_jpeg(msg: dict) -> Optional[bytes]:
    if msg.get("encoding") != "jpeg":
        return None
    data = msg.get("data")
    return bytes(data) if isinstance(data, (bytes, bytearray)) else None


def depth_jpeg(msg: dict) -> Optional[bytes]:
    arr = depth_u16_mm(msg)
    if arr is None:
        return None
    valid_mask = arr > 0
    if valid_mask.any():
        valid = arr[valid_mask]
        lo, hi = float(valid.min()), float(np.percentile(valid, 99))
        if hi - lo < 1.0:
            hi = lo + 1.0
        norm = np.clip((arr.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    else:
        norm = np.zeros_like(arr, dtype=np.uint8)
    color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    color[~valid_mask] = 0
    ok, buf = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return buf.tobytes() if ok else None


def status_jpeg(title: str, detail: str, size: int = 480) -> Optional[bytes]:
    if not HAVE_CV:
        return None
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = (22, 22, 22)
    cv2.putText(img, title[:32], (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 220, 255), 2, cv2.LINE_AA)
    y = 92
    for line in detail.splitlines()[:8]:
        cv2.putText(img, line[:48], (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 210, 210), 1, cv2.LINE_AA)
        y += 28
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return buf.tobytes() if ok else None


def ranges_array(value) -> Optional["np.ndarray"]:
    if not HAVE_CV:
        return None
    try:
        if isinstance(value, (bytes, bytearray, memoryview)):
            return np.frombuffer(bytes(value), dtype=np.float32).copy()
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        return arr.copy()
    except Exception:
        return None


def primary_scan(frames: dict[str, dict]) -> Optional[dict]:
    if "scan" in frames:
        return frames["scan"]
    scan_topics = [name for name in frames if name.startswith("scan")]
    if not scan_topics:
        return None
    return frames[sorted(scan_topics)[0]]


def scan_points(msg: Optional[dict]) -> tuple[Optional["np.ndarray"], Optional["np.ndarray"], float]:
    if not HAVE_CV or msg is None:
        return None, None, 12.0
    try:
        a0, a1 = float(msg["angle_min"]), float(msg["angle_max"])
        rmax = float(msg.get("range_max", 12.0))
    except Exception:
        return None, None, 12.0
    ranges = ranges_array(msg.get("ranges"))
    if ranges is None:
        return None, None, rmax
    if ranges.size == 0:
        return None, None, rmax
    angles = np.linspace(a0, a1, ranges.size, endpoint=True, dtype=np.float32)
    keep = np.isfinite(ranges) & (ranges > 0.05) & (ranges < rmax - 0.01)
    x = ranges[keep] * np.cos(angles[keep])
    y = ranges[keep] * np.sin(angles[keep])
    return x, y, rmax


def scan_ranges_angles(
    msg: Optional[dict],
) -> tuple[Optional["np.ndarray"], Optional["np.ndarray"], float, float]:
    if not HAVE_CV or msg is None:
        return None, None, 12.0, 0.05
    try:
        a0, a1 = float(msg["angle_min"]), float(msg["angle_max"])
        rmin = float(msg.get("range_min", 0.05))
        rmax = float(msg.get("range_max", 12.0))
    except Exception:
        return None, None, 12.0, 0.05
    ranges = ranges_array(msg.get("ranges"))
    if ranges is None:
        return None, None, rmax, rmin
    if ranges.size == 0:
        return None, None, rmax, rmin
    angles = np.linspace(a0, a1, ranges.size, endpoint=True, dtype=np.float32)
    return ranges, angles, rmax, rmin


def scan_jpeg(msg: dict, size: int = 480) -> Optional[bytes]:
    x, y, rmax = scan_points(msg)
    if x is None or y is None:
        return status_jpeg("scan", "received scan topic\nbut ranges are empty\nor not decodable", size)
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cx, cy = size // 2, size // 2
    scale = (size // 2 - 12) / max(rmax, 1e-3)
    for r in (1.0, 3.0, 5.0, 10.0):
        if r <= rmax:
            cv2.circle(img, (cx, cy), int(r * scale), (50, 50, 50), 1)
    px = (cx + x * scale).astype(np.int32)
    py = (cy - y * scale).astype(np.int32)
    keep = (px >= 0) & (px < size) & (py >= 0) & (py < size)
    img[py[keep], px[keep]] = (0, 255, 0)
    cv2.line(img, (cx, cy), (cx + int(scale * 0.4), cy), (0, 255, 255), 2)
    cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return buf.tobytes() if ok else None


def render_jpeg(topic: str, msg: dict) -> Optional[bytes]:
    if topic.startswith("rgb."):
        return rgb_jpeg(msg)
    if topic.startswith("depth."):
        return depth_jpeg(msg)
    if topic.startswith("scan"):
        return scan_jpeg(msg)
    return None


def _json_default(o):
    if isinstance(o, (bytes, bytearray)):
        return f"<{len(o)} bytes>"
    raise TypeError


def yaw_from_xyzw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def latest_stamp_ns(frames: dict[str, dict], names: tuple[str, ...]) -> int:
    stamps = []
    for name in names:
        msg = frames.get(name)
        if msg is None:
            continue
        try:
            stamps.append(int(msg.get("stamp_ns", 0)))
        except Exception:
            continue
    return max(stamps) if stamps else time.time_ns()


def latest_frame_stamp_ns(frames: dict[str, dict]) -> int:
    stamps = []
    for msg in frames.values():
        try:
            stamps.append(int(msg.get("stamp_ns", 0)))
        except Exception:
            continue
    return max(stamps) if stamps else time.time_ns()


class OcrProjector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.backend = args.ocr_backend
        self.langs = [v.strip() for v in args.ocr_langs.split(",") if v.strip()]
        self.min_conf = float(args.ocr_min_confidence)
        self.ocr_interval = max(0.05, float(args.ocr_interval))
        self.floor_hint = normalize_floor_hint(args.floor_hint)
        self.floor_prior_mode = args.floor_prior_mode
        self.ocr_scales = parse_scales(args.ocr_scales)
        self.ocr_max_side = int(args.ocr_max_side)
        self.camera_x = float(args.camera_x_m)
        self.camera_y = float(args.camera_y_m)
        self.camera_z = float(args.camera_z_m)
        self.camera_yaw = float(args.camera_yaw_offset_rad)
        self.hfov = float(args.camera_hfov_rad)
        self.merge_radius_m = float(args.ocr_merge_radius_m)
        self._easy_reader = None
        self._paddle_reader = None
        self._backend_error = ""
        self._last_rgb_stamp: Optional[int] = None
        self._last_raw_detections: list[dict] = []
        self._last_ocr_status = "starting"
        self._last_ocr_wall_s = 0.0
        self._world_scan = np.empty((0, 2), dtype=np.float32) if HAVE_CV else None
        self._path: list[tuple[float, float]] = []
        self.grid_resolution = max(0.03, float(args.grid_resolution_m))
        self.grid_size_m = max(5.0, float(args.grid_size_m))
        self.grid_view_m = max(2.0, float(args.grid_view_m))
        self.grid_auto_fit = not bool(args.disable_grid_auto_fit)
        self._last_view_m = self.grid_view_m
        self._grid_center: Optional[tuple[float, float]] = None
        self._grid_dim = int(round(self.grid_size_m / self.grid_resolution)) if HAVE_CV else 0
        if HAVE_CV:
            self._grid = np.zeros((self._grid_dim, self._grid_dim), dtype=np.int16)
        else:
            self._grid = None
        self._ocr_annotations: list[dict] = []
        self._last_grid_scan_stamp: Optional[int] = None

    def run(self, frames: dict[str, dict]) -> tuple[dict, Optional[bytes], Optional[bytes], int]:
        rgb_msg = frames.get("rgb.front")
        depth_msg = frames.get("depth.front")
        scan_msg = primary_scan(frames)
        proprio = frames.get("proprio")
        stamp_ns = latest_frame_stamp_ns(frames)

        rgb = rgb_array(rgb_msg)
        if rgb is None:
            analysis = {
                "status": "no_rgb",
                "ocr_backend": self.backend,
                "backend_error": self._backend_error,
                "floor_hint": self.floor_hint,
                "detections": [],
                "map_annotations": list(self._ocr_annotations),
                "topics": sorted(frames),
                "grid_map": {
                    "resolution_m": self.grid_resolution,
                    "size_m": self.grid_size_m,
                    "center_xy_m": list(self._grid_center) if self._grid_center else None,
                    "annotation_count": len(self._ocr_annotations),
                    "path_points": len(self._path),
                    "center_mode": "fixed_world",
                    "view_m": self._last_view_m,
                    "auto_fit": self.grid_auto_fit,
                    "pose_source": self._pose_source(proprio),
                },
            }
            self._append_world_scan(scan_msg, proprio)
            return analysis, None, self._draw_map(scan_msg, [], proprio), stamp_ns

        try:
            rgb_stamp = int(rgb_msg.get("stamp_ns", stamp_ns)) if rgb_msg is not None else stamp_ns
        except Exception:
            rgb_stamp = stamp_ns
        if self.backend == "none":
            raw_detections: list[dict] = []
            status = "ocr_disabled"
            new_ocr = False
        else:
            now = time.monotonic()
            should_ocr = self._last_rgb_stamp is None or (
                rgb_stamp != self._last_rgb_stamp and now - self._last_ocr_wall_s >= self.ocr_interval
            )
            if should_ocr:
                raw_detections, status = self._ocr(rgb)
                self._last_rgb_stamp = rgb_stamp
                self._last_raw_detections = [dict(det) for det in raw_detections]
                self._last_ocr_status = status
                self._last_ocr_wall_s = now
                new_ocr = True
            else:
                raw_detections = [dict(det) for det in self._last_raw_detections]
                status = self._last_ocr_status
                new_ocr = False
        depth = depth_u16_mm(depth_msg)
        detections = [
            self._enrich_detection(det, rgb.shape[1], rgb.shape[0], depth, proprio)
            for det in raw_detections
        ]
        if new_ocr:
            self._remember_detections(detections, rgb_stamp)
        overlay = self._draw_overlay(rgb, detections)
        self._append_world_scan(scan_msg, proprio)
        local_map = self._draw_map(scan_msg, detections, proprio)
        analysis = {
            "status": status,
            "ocr_backend": self.backend,
            "backend_error": self._backend_error,
            "stamp_ns": stamp_ns,
            "rgb_stamp_ns": rgb_stamp,
            "detections_fresh_for_map": new_ocr,
            "detections": detections,
            "map_annotations": list(self._ocr_annotations),
            "topics": sorted(frames),
            "grid_map": {
                "resolution_m": self.grid_resolution,
                "size_m": self.grid_size_m,
                "center_xy_m": list(self._grid_center) if self._grid_center else None,
                "annotation_count": len(self._ocr_annotations),
                "path_points": len(self._path),
                "center_mode": "fixed_world",
                "view_m": self._last_view_m,
                "auto_fit": self.grid_auto_fit,
                "pose_source": self._pose_source(proprio),
            },
            "camera": {
                "x_m": self.camera_x,
                "y_m": self.camera_y,
                "z_m": self.camera_z,
                "yaw_offset_rad": self.camera_yaw,
                "hfov_rad": self.hfov,
            },
            "ocr_policy": {
                "backend": self.backend,
                "langs": self.langs,
                "min_confidence": self.min_conf,
                "floor_hint": self.floor_hint,
                "floor_prior_mode": self.floor_prior_mode,
                "scales": self.ocr_scales,
                "ocr_max_side": self.ocr_max_side,
                "pixel_anchor": "bbox_center",
                "depth_source": "depth.front",
                "depth_units": "uint16_mm",
                "depth_sample": "median_valid_9x9_patch",
                "projection": "pinhole_hfov_camera_forward_to_base_xy_then_pose_to_world_xy",
                "map_frame": "world_xy_when_proprio_available_else_base_xy",
                "merge_radius_m": self.merge_radius_m,
            },
        }
        return analysis, overlay, local_map, stamp_ns

    @staticmethod
    def _pose_xy_yaw(proprio: Optional[dict]) -> Optional[tuple[float, float, float]]:
        if proprio is None:
            return None
        joint_pose = proprio.get("joint_vel_arm_sample")
        if isinstance(joint_pose, list) and len(joint_pose) >= 3:
            return float(joint_pose[0]), float(joint_pose[1]), float(joint_pose[2])
        pose = proprio.get("base_pose")
        if not isinstance(pose, list) or len(pose) < 7:
            return None
        yaw = yaw_from_xyzw(float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]))
        return float(pose[0]), float(pose[1]), yaw

    @staticmethod
    def _pose_source(proprio: Optional[dict]) -> str:
        if proprio is None:
            return "none"
        joint_pose = proprio.get("joint_vel_arm_sample")
        if isinstance(joint_pose, list) and len(joint_pose) >= 3:
            return "joint_vel_arm_sample"
        pose = proprio.get("base_pose")
        if isinstance(pose, list) and len(pose) >= 7:
            return "base_pose"
        return "none"

    def _append_world_scan(self, scan_msg: Optional[dict], proprio: Optional[dict]) -> None:
        if not HAVE_CV or self._world_scan is None:
            return
        pose = self._pose_xy_yaw(proprio)
        if pose is None:
            return
        rx, ry, yaw = pose
        if self._grid_center is None:
            self._grid_center = (rx, ry)
        robot_cell = self._world_to_cell(rx, ry)
        if robot_cell is not None:
            self._mark_cell(robot_cell, -2)
        if not self._path or math.hypot(rx - self._path[-1][0], ry - self._path[-1][1]) > 0.03:
            self._path.append((rx, ry))
            if len(self._path) > 4000:
                self._path = self._path[-4000:]

        self._update_occupancy_grid(scan_msg, pose)
        sx, sy, _ = scan_points(scan_msg)
        if sx is None or sy is None or sx.size == 0:
            return
        cy = math.cos(yaw)
        syaw = math.sin(yaw)
        local = np.stack([sx[::4], sy[::4]], axis=1).astype(np.float32)
        world = np.empty_like(local)
        world[:, 0] = rx + cy * local[:, 0] - syaw * local[:, 1]
        world[:, 1] = ry + syaw * local[:, 0] + cy * local[:, 1]
        self._world_scan = np.concatenate([self._world_scan, world], axis=0)
        if self._world_scan.shape[0] > 60000:
            self._world_scan = self._world_scan[-60000:]

    def _update_occupancy_grid(
        self,
        scan_msg: Optional[dict],
        pose: tuple[float, float, float],
    ) -> None:
        if self._grid is None:
            return
        stamp = scan_msg.get("stamp_ns") if scan_msg is not None else None
        if stamp is not None and stamp == self._last_grid_scan_stamp:
            return
        self._last_grid_scan_stamp = int(stamp or time.time_ns())

        ranges, angles, rmax, rmin = scan_ranges_angles(scan_msg)
        if ranges is None or angles is None:
            return

        rx, ry, yaw = pose
        if self._grid_center is None:
            self._grid_center = (rx, ry)
        robot_cell = self._world_to_cell(rx, ry)
        if robot_cell is None:
            return

        max_free = min(float(rmax), self.grid_view_m)
        beam_step = max(1, int(ranges.size // 180))
        for r, a in zip(ranges[::beam_step], angles[::beam_step]):
            if not math.isfinite(float(r)) or float(r) < rmin:
                ray_len = max_free
                hit = False
            else:
                ray_len = min(float(r), max_free)
                hit = float(r) < min(float(rmax) - 0.05, max_free)
            if ray_len <= rmin:
                continue
            wx = rx + ray_len * math.cos(yaw + float(a))
            wy = ry + ray_len * math.sin(yaw + float(a))
            end_cell = self._world_to_cell(wx, wy)
            if end_cell is None:
                continue
            self._mark_free_line(robot_cell, end_cell)
            if hit:
                self._mark_cell(end_cell, 5)

        self._mark_cell(robot_cell, -2)

    def _world_to_cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        if self._grid_center is None or self._grid is None:
            return None
        cx, cy = self._grid_center
        gx = int(math.floor((x - (cx - self.grid_size_m * 0.5)) / self.grid_resolution))
        gy = int(math.floor((y - (cy - self.grid_size_m * 0.5)) / self.grid_resolution))
        if gx < 0 or gy < 0 or gx >= self._grid_dim or gy >= self._grid_dim:
            return None
        return gx, gy

    def _cell_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        assert self._grid_center is not None
        cx, cy = self._grid_center
        wx = (cx - self.grid_size_m * 0.5) + (gx + 0.5) * self.grid_resolution
        wy = (cy - self.grid_size_m * 0.5) + (gy + 0.5) * self.grid_resolution
        return wx, wy

    def _mark_cell(self, cell: tuple[int, int], delta: int) -> None:
        if self._grid is None:
            return
        gx, gy = cell
        self._grid[gy, gx] = int(np.clip(int(self._grid[gy, gx]) + delta, -24, 42))

    def _mark_free_line(self, start: tuple[int, int], end: tuple[int, int]) -> None:
        if self._grid is None:
            return
        x0, y0 = start
        x1, y1 = end
        steps = max(abs(x1 - x0), abs(y1 - y0))
        if steps <= 0:
            return
        xs = np.linspace(x0, x1, steps + 1, dtype=np.int32)
        ys = np.linspace(y0, y1, steps + 1, dtype=np.int32)
        xs = xs[:-1]
        ys = ys[:-1]
        keep = (xs >= 0) & (ys >= 0) & (xs < self._grid_dim) & (ys < self._grid_dim)
        if np.any(keep):
            self._grid[ys[keep], xs[keep]] = np.maximum(self._grid[ys[keep], xs[keep]] - 1, -24)

    def _remember_detections(self, detections: list[dict], stamp_ns: int) -> None:
        for det in detections:
            text = str(det.get("text") or "").strip()
            world_xy = det.get("world_xy_m")
            base_xy = det.get("base_xy_m")
            if not text:
                continue
            if isinstance(world_xy, list) and len(world_xy) == 2:
                map_xy = [float(world_xy[0]), float(world_xy[1])]
                coord_frame = "world"
            elif isinstance(base_xy, list) and len(base_xy) == 2:
                map_xy = [float(base_xy[0]), float(base_xy[1])]
                coord_frame = "base"
            else:
                continue
            wx, wy = map_xy
            best = None
            best_dist = float("inf")
            for ann in self._ocr_annotations:
                if ann.get("coord_frame", "world") != coord_frame:
                    continue
                ax, ay = ann.get("map_xy_m", ann.get("world_xy_m", [None, None]))
                if ax is None or ay is None:
                    continue
                dist = math.hypot(wx - float(ax), wy - float(ay))
                if dist < best_dist and dist <= self.merge_radius_m:
                    best = ann
                    best_dist = dist
            if best is not None:
                n = int(best.get("observations", 1))
                old_x, old_y = [float(v) for v in best.get("map_xy_m", best.get("world_xy_m"))]
                new_conf = float(det.get("confidence", 0.0))
                old_conf = float(best.get("confidence", best.get("best_confidence", 0.0)))
                new_weight = max(0.25, min(1.0, new_conf)) * 4.0
                old_weight = max(1.0, min(20.0, float(n)))
                fused_xy = [
                    round((old_x * old_weight + wx * new_weight) / (old_weight + new_weight), 3),
                    round((old_y * old_weight + wy * new_weight) / (old_weight + new_weight), 3),
                ]
                best["map_xy_m"] = fused_xy
                if coord_frame == "world":
                    best["world_xy_m"] = fused_xy
                best["last_text"] = text
                best["last_confidence"] = new_conf
                best["last_source"] = det.get("source")
                best["base_xy_m"] = det.get("base_xy_m")
                if new_conf >= old_conf:
                    best["text"] = text
                    best["source"] = det.get("source")
                    best["confidence"] = new_conf
                    best["best_confidence"] = new_conf
                    best["bbox_xyxy"] = det.get("bbox_xyxy")
                    best["depth_m"] = det.get("depth_m")
                    best["raw_text"] = det.get("raw_text")
                else:
                    best["confidence"] = old_conf
                    best["best_confidence"] = max(float(best.get("best_confidence", old_conf)), old_conf)
                best["observations"] = n + 1
                best["last_stamp_ns"] = int(stamp_ns)
                continue

            self._ocr_annotations.append({
                "id": len(self._ocr_annotations) + 1,
                "text": text,
                "raw_text": det.get("raw_text"),
                "source": det.get("source"),
                "confidence": float(det.get("confidence", 0.0)),
                "best_confidence": float(det.get("confidence", 0.0)),
                "last_text": text,
                "last_confidence": float(det.get("confidence", 0.0)),
                "last_source": det.get("source"),
                "bbox_xyxy": det.get("bbox_xyxy"),
                "depth_m": det.get("depth_m"),
                "base_xy_m": det.get("base_xy_m"),
                "world_xy_m": [round(wx, 3), round(wy, 3)] if coord_frame == "world" else det.get("world_xy_m"),
                "map_xy_m": [round(wx, 3), round(wy, 3)],
                "coord_frame": coord_frame,
                "observations": 1,
                "first_stamp_ns": int(stamp_ns),
                "last_stamp_ns": int(stamp_ns),
            })
            if len(self._ocr_annotations) > 200:
                self._ocr_annotations = self._ocr_annotations[-200:]

    def _ocr(self, rgb: "np.ndarray") -> tuple[list[dict], str]:
        if self.backend in ("gazebo", "paddle"):
            dets, status = self._gazebo_ocr(rgb, include_tesseract=self.backend == "gazebo")
            return dets, status
        if self.backend == "easyocr":
            dets, err = self._easyocr(rgb)
            if err:
                self._backend_error = err
                fallback, status = self._tesseract(rgb)
                return fallback, "easyocr_failed_tesseract_fallback" if fallback else "ocr_error"
            return dets, "ok"
        if self.backend == "tesseract":
            return self._tesseract(rgb)
        return [], "ocr_disabled"

    def _scale_for_ocr(self, rgb: "np.ndarray", requested: float) -> float:
        if self.ocr_max_side <= 0:
            return requested
        h, w = rgb.shape[:2]
        return max(0.25, min(requested, float(self.ocr_max_side) / float(max(h, w))))

    def _postprocess_ocr_candidates(self, candidates: list[dict], w: int, h: int) -> list[dict]:
        out: list[dict] = []
        for det in candidates:
            try:
                conf = float(det.get("confidence", 0.0))
            except Exception:
                conf = 0.0
            if conf < self.min_conf:
                continue
            raw_text = clean_ocr_text(str(det.get("raw_text") or det.get("text") or ""))
            room_id = normalize_room_id(raw_text, self.floor_hint, self.floor_prior_mode)
            if not room_id:
                continue
            bbox = clamp_bbox(det.get("bbox_xyxy"), w, h)
            if bbox is None:
                continue
            out.append({
                **det,
                "raw_text": raw_text,
                "text": room_id,
                "room_id": room_id,
                "confidence": conf,
                "bbox_xyxy": bbox,
            })

        kept: list[dict] = []
        for det in sorted(out, key=lambda d: float(d.get("confidence", 0.0)), reverse=True):
            bbox = det.get("bbox_xyxy")
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue
            cx = 0.5 * (bbox[0] + bbox[2])
            cy = 0.5 * (bbox[1] + bbox[3])
            duplicate = False
            for prev in kept:
                prev_bbox = prev.get("bbox_xyxy")
                if not isinstance(prev_bbox, list) or len(prev_bbox) != 4:
                    continue
                same_text = str(prev.get("text")) == str(det.get("text"))
                pcx = 0.5 * (prev_bbox[0] + prev_bbox[2])
                pcy = 0.5 * (prev_bbox[1] + prev_bbox[3])
                center_dist = math.hypot(cx - pcx, cy - pcy)
                if same_text and (bbox_iou(bbox, prev_bbox) > 0.15 or center_dist < 32.0):
                    duplicate = True
                    break
            if not duplicate:
                kept.append(det)
        return sorted(kept, key=lambda d: (d["bbox_xyxy"][1], d["bbox_xyxy"][0]))

    def _gazebo_ocr(self, rgb: "np.ndarray", include_tesseract: bool) -> tuple[list[dict], str]:
        h, w = rgb.shape[:2]
        candidates: list[dict] = []
        errors: list[str] = []

        paddle_dets, paddle_err = self._paddleocr(rgb)
        candidates.extend(paddle_dets)
        if paddle_err:
            errors.append(f"paddleocr: {paddle_err}")

        if include_tesseract or paddle_err:
            tess_dets, tess_status = self._tesseract_multiscale(rgb)
            candidates.extend(tess_dets)
            if tess_status == "ocr_error" and self._backend_error:
                errors.append(f"tesseract: {self._backend_error}")

        roi_count = 0
        for roi, offset_xy, roi_source in self._dark_sign_rois(rgb):
            roi_count += 1
            rx, ry = offset_xy
            roi_paddle, roi_paddle_err = self._paddleocr(roi, scales=[4.0, 8.0, 12.0])
            if roi_paddle_err and not paddle_err:
                errors.append(f"roi_paddleocr: {roi_paddle_err}")
            roi_tess, roi_tess_status = self._tesseract_multiscale(roi, scales=[6.0, 8.0, 12.0, 16.0, 24.0])
            for det in roi_paddle + roi_tess:
                bbox = det.get("bbox_xyxy")
                if isinstance(bbox, list) and len(bbox) == 4:
                    det["bbox_xyxy"] = [int(bbox[0] + rx), int(bbox[1] + ry), int(bbox[2] + rx), int(bbox[3] + ry)]
                det["source"] = f"{det.get('source', 'ocr')}:{roi_source}"
                candidates.append(det)
            if roi_tess_status == "ocr_error" and self._backend_error:
                errors.append(f"roi_tesseract: {self._backend_error}")

        dets = self._postprocess_ocr_candidates(candidates, w, h)
        self._backend_error = "; ".join(errors)
        if dets:
            return dets, "ok"
        if errors:
            return [], "ocr_error"
        return [], f"no_room_id_detections_roi={roi_count}"

    def _dark_sign_rois(self, rgb: "np.ndarray") -> list[tuple["np.ndarray", tuple[int, int], str]]:
        if not HAVE_CV:
            return []
        h, w = rgb.shape[:2]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[(hsv[:, :, 2] < 70) & (hsv[:, :, 1] < 150)] = 255
        mask[:4, :] = 0
        mask[int(h * 0.72):, :] = 0
        kernel = np.ones((2, 2), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rois: list[tuple[np.ndarray, tuple[int, int], str]] = []
        seen: list[list[int]] = []
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = bw * bh
            aspect = bw / max(float(bh), 1.0)
            if not (6 <= bw <= 90 and 4 <= bh <= 48 and 36 <= area <= 1800 and 0.6 <= aspect <= 8.0):
                continue
            pad_x = max(10, int(round(bw * 1.8)))
            pad_y = max(8, int(round(bh * 1.6)))
            x1 = max(0, x - pad_x)
            y1 = max(0, y - pad_y)
            x2 = min(w, x + bw + pad_x)
            y2 = min(h, y + bh + pad_y)
            bbox = [x1, y1, x2, y2]
            if any(bbox_iou(bbox, prev) > 0.35 for prev in seen):
                continue
            seen.append(bbox)
            rois.append((rgb[y1:y2, x1:x2].copy(), (x1, y1), f"dark_roi[{x},{y},{bw},{bh}]"))
        rois.sort(key=lambda item: item[0].shape[0] * item[0].shape[1])
        return rois[-8:]

    def _paddleocr(self, rgb: "np.ndarray", scales: Optional[list[float]] = None) -> tuple[list[dict], str]:
        try:
            if self._paddle_reader is None:
                from paddleocr import PaddleOCR

                try:
                    self._paddle_reader = PaddleOCR(use_angle_cls=True, lang="en", use_gpu=False, show_log=False)
                except TypeError:
                    self._paddle_reader = PaddleOCR(lang="en")
        except Exception as exc:
            return [], str(exc)

        h, w = rgb.shape[:2]
        detections: list[dict] = []
        for requested_scale in (scales or self.ocr_scales):
            scale = self._scale_for_ocr(rgb, requested_scale)
            try:
                rgb_in = scaled_rgb_for_ocr(rgb, scale)
                out = self._paddle_reader.ocr(rgb_in, cls=True)
                lines = out[0] if out and isinstance(out[0], list) else out
            except Exception as exc:
                return detections, str(exc)
            for line in lines or []:
                try:
                    pts = np.asarray(line[0], dtype=np.float32) / float(scale)
                    text = clean_ocr_text(str(line[1][0]))
                    conf = float(line[1][1])
                except Exception:
                    continue
                if not text:
                    continue
                bbox = clamp_bbox([
                    float(pts[:, 0].min()),
                    float(pts[:, 1].min()),
                    float(pts[:, 0].max()),
                    float(pts[:, 1].max()),
                ], w, h)
                detections.append({
                    "source": f"paddleocr@{scale:g}x",
                    "raw_text": text,
                    "text": text,
                    "confidence": conf,
                    "bbox_xyxy": bbox,
                })
        return detections, ""

    def _easyocr(self, rgb: "np.ndarray") -> tuple[list[dict], str]:
        try:
            if self._easy_reader is None:
                import easyocr

                self._easy_reader = easyocr.Reader(self.langs or ["ko", "en"], gpu=False)
            out = self._easy_reader.readtext(rgb)
        except Exception as exc:
            return [], str(exc)

        detections = []
        for box, text, conf in out:
            if float(conf) < self.min_conf:
                continue
            pts = np.asarray(box, dtype=np.float32)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)
            detections.append({
                "source": "easyocr",
                "text": str(text).strip(),
                "confidence": float(conf),
                "bbox_xyxy": [int(x1), int(y1), int(x2), int(y2)],
            })
        return detections, ""

    def _tesseract(self, rgb: "np.ndarray") -> tuple[list[dict], str]:
        detections, status = self._tesseract_multiscale(rgb)
        if status != "ok":
            return [], status
        h, w = rgb.shape[:2]
        return self._postprocess_ocr_candidates(detections, w, h), "ok"

    def _tesseract_multiscale(self, rgb: "np.ndarray", scales: Optional[list[float]] = None) -> tuple[list[dict], str]:
        try:
            import pytesseract
            from PIL import Image
        except Exception as exc:
            self._backend_error = str(exc)
            return [], "ocr_error"

        detections = []
        config = "--oem 3 --psm 11 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-"
        for requested_scale in (scales or self.ocr_scales):
            scale = self._scale_for_ocr(rgb, requested_scale)
            rgb_in = scaled_rgb_for_ocr(rgb, scale)
            gray = cv2.cvtColor(rgb_in, cv2.COLOR_RGB2GRAY)
            variants = [rgb_in]
            try:
                clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
                variants.append(clahe)
                otsu = cv2.threshold(clahe, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
                variants.append(otsu)
                variants.append(255 - otsu)
            except Exception:
                variants.append(gray)
            for variant_index, image in enumerate(variants):
                try:
                    data = pytesseract.image_to_data(
                        Image.fromarray(image),
                        lang="eng",
                        config=config,
                        output_type=pytesseract.Output.DICT,
                    )
                except Exception as exc:
                    self._backend_error = str(exc)
                    continue
                for i, raw_text in enumerate(data.get("text", [])):
                    text = clean_ocr_text(str(raw_text))
                    if not text:
                        continue
                    try:
                        conf = float(data.get("conf", ["-1"])[i]) / 100.0
                    except Exception:
                        conf = -1.0
                    if conf < 0.0:
                        continue
                    x = int(round(float(data["left"][i]) / scale))
                    y = int(round(float(data["top"][i]) / scale))
                    bw = int(round(float(data["width"][i]) / scale))
                    bh = int(round(float(data["height"][i]) / scale))
                    detections.append({
                        "source": f"pytesseract@{scale:g}x:v{variant_index}",
                        "raw_text": text,
                        "text": text,
                        "confidence": conf,
                        "bbox_xyxy": [x, y, x + bw, y + bh],
                    })
        return detections, "ok"

    def _enrich_detection(
        self,
        det: dict,
        rgb_w: int,
        rgb_h: int,
        depth: Optional["np.ndarray"],
        proprio: Optional[dict],
    ) -> dict:
        out = dict(det)
        bbox = det.get("bbox_xyxy")
        if not isinstance(bbox, list) or len(bbox) != 4:
            out["depth_m"] = None
            out["base_xy_m"] = None
            out["world_xy_m"] = None
            return out
        x1, y1, x2, y2 = [int(v) for v in bbox]
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        out["pixel_xy"] = [round(cx, 1), round(cy, 1)]
        depth_m = self._sample_depth(depth, cx / max(rgb_w, 1), cy / max(rgb_h, 1))
        out["depth_m"] = depth_m
        out["depth_sample"] = "median_valid_9x9_patch"
        if depth_m is None:
            out["base_xy_m"] = None
            out["world_xy_m"] = None
            out["map_status"] = "ocr_text_only_no_depth"
            return out

        fx = float(rgb_w) / (2.0 * math.tan(max(0.1, self.hfov) * 0.5))
        fy = fx
        ox = (cx - (rgb_w - 1) * 0.5) * depth_m / fx
        oy = (cy - (rgb_h - 1) * 0.5) * depth_m / fy
        forward_x = math.cos(self.camera_yaw)
        forward_y = math.sin(self.camera_yaw)
        pixel_right_x = math.sin(self.camera_yaw)
        pixel_right_y = -math.cos(self.camera_yaw)
        base_x = self.camera_x + depth_m * forward_x + ox * pixel_right_x
        base_y = self.camera_y + depth_m * forward_y + ox * pixel_right_y
        base_z = self.camera_z - oy
        out["base_xyz_m"] = [round(base_x, 3), round(base_y, 3), round(base_z, 3)]
        out["base_xy_m"] = [round(base_x, 3), round(base_y, 3)]
        world_xy = self._world_xy(base_x, base_y, proprio)
        out["world_xy_m"] = [round(world_xy[0], 3), round(world_xy[1], 3)] if world_xy else None
        out["map_status"] = "world_xy" if world_xy else "base_xy_only_no_proprio"
        return out

    @staticmethod
    def _sample_depth(depth: Optional["np.ndarray"], nx: float, ny: float) -> Optional[float]:
        if depth is None or depth.size == 0:
            return None
        h, w = depth.shape
        u = max(0, min(w - 1, int(round(nx * (w - 1)))))
        v = max(0, min(h - 1, int(round(ny * (h - 1)))))
        r = 4
        patch = depth[max(0, v - r):min(h, v + r + 1), max(0, u - r):min(w, u + r + 1)]
        valid = patch[(patch > 50) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return round(float(np.median(valid.astype(np.float32)) * 0.001), 3)

    def _world_xy(self, base_x: float, base_y: float, proprio: Optional[dict]) -> Optional[tuple[float, float]]:
        pose = self._pose_xy_yaw(proprio)
        if pose is None:
            return None
        x, y, yaw = pose
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return x + cy * base_x - sy * base_y, y + sy * base_x + cy * base_y

    @staticmethod
    def _draw_overlay(rgb: "np.ndarray", detections: list[dict]) -> Optional[bytes]:
        if not HAVE_CV:
            return None
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for det in detections:
            bbox = det.get("bbox_xyxy")
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [int(v) for v in bbox]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 180, 255), 2)
            label = str(det.get("text") or "text")[:40]
            depth_m = det.get("depth_m")
            if depth_m is not None:
                label = f"{label} {depth_m:.2f}m"
            cv2.putText(img, label, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 180, 255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        return buf.tobytes() if ok else None

    def _auto_view_m(
        self,
        center_x: float,
        center_y: float,
        pose: Optional[tuple[float, float, float]],
        detections: list[dict],
    ) -> float:
        view_m = float(self.grid_view_m)
        if not self.grid_auto_fit:
            self._last_view_m = view_m
            return view_m

        points: list[tuple[float, float]] = []
        if pose is not None:
            points.append((pose[0], pose[1]))
        points.extend(self._path[-4000:])

        for ann in self._ocr_annotations:
            map_xy = ann.get("map_xy_m", ann.get("world_xy_m"))
            if not isinstance(map_xy, list) or len(map_xy) != 2:
                continue
            mx, my = float(map_xy[0]), float(map_xy[1])
            if ann.get("coord_frame", "world") == "base" and pose is not None:
                rx, ry, yaw = pose
                cyaw = math.cos(yaw)
                syaw = math.sin(yaw)
                points.append((rx + cyaw * mx - syaw * my, ry + syaw * mx + cyaw * my))
            else:
                points.append((mx, my))

        for det in detections:
            if isinstance(det.get("world_xy_m"), list) and len(det["world_xy_m"]) == 2:
                points.append((float(det["world_xy_m"][0]), float(det["world_xy_m"][1])))
            elif isinstance(det.get("base_xy_m"), list) and len(det["base_xy_m"]) == 2 and pose is not None:
                bx, by = [float(v) for v in det["base_xy_m"]]
                rx, ry, yaw = pose
                cyaw = math.cos(yaw)
                syaw = math.sin(yaw)
                points.append((rx + cyaw * bx - syaw * by, ry + syaw * bx + cyaw * by))

        if points:
            max_extent = max(max(abs(px - center_x), abs(py - center_y)) for px, py in points)
            view_m = max(view_m, max_extent * 1.18 + 2.0)

        view_cap = max(self.grid_view_m, self.grid_size_m * 0.5 - self.grid_resolution)
        self._last_view_m = min(view_m, view_cap)
        return self._last_view_m

    def _draw_map(self, scan_msg: Optional[dict], detections: list[dict], proprio: Optional[dict]) -> Optional[bytes]:
        if not HAVE_CV:
            return None
        size = 640
        img = np.zeros((size, size, 3), dtype=np.uint8)
        pose = self._pose_xy_yaw(proprio)

        # Keep the map in a stable world frame. The robot moves inside this
        # viewport; the viewport must not chase the robot.
        if self._grid_center is not None:
            center_x, center_y = self._grid_center
        elif pose is not None:
            center_x, center_y, _ = pose
        else:
            center_x = center_y = 0.0
        view_m = self._auto_view_m(center_x, center_y, pose, detections)
        scale = (size - 40) / max(view_m * 2.0, 1e-6)

        def world_to_px(wx: float, wy: float) -> tuple[int, int]:
            px = int(round(size * 0.5 + (wx - center_x) * scale))
            py = int(round(size * 0.5 - (wy - center_y) * scale))
            return px, py

        if self._grid is not None and self._grid_center is not None:
            center_cell = self._world_to_cell(center_x, center_y)
            radius_cells = max(4, int(math.ceil(view_m / self.grid_resolution)))
            if center_cell is not None:
                gx, gy = center_cell
                x0 = max(0, gx - radius_cells)
                x1 = min(self._grid_dim, gx + radius_cells)
                y0 = max(0, gy - radius_cells)
                y1 = min(self._grid_dim, gy + radius_cells)
                crop = self._grid[y0:y1, x0:x1]
                gray = np.full(crop.shape, 92, dtype=np.uint8)
                gray[crop <= -3] = 218
                gray[crop >= 3] = 28
                bgr = cv2.cvtColor(np.flipud(gray), cv2.COLOR_GRAY2BGR)
                img = cv2.resize(bgr, (size, size), interpolation=cv2.INTER_NEAREST)
            else:
                img[:] = (92, 92, 92)
        else:
            img[:] = (92, 92, 92)

        for r in range(1, int(view_m) + 1):
            if r % 2 == 0:
                d = int(round(r * scale))
                cv2.rectangle(
                    img,
                    (max(0, size // 2 - d), max(0, size // 2 - d)),
                    (min(size - 1, size // 2 + d), min(size - 1, size // 2 + d)),
                    (120, 120, 120),
                    1,
                )

        if pose is not None:
            ranges, angles, rmax, rmin = scan_ranges_angles(scan_msg)
            if ranges is not None and angles is not None:
                rx, ry, yaw = pose
                finite = np.isfinite(ranges) & (ranges > rmin) & (ranges < min(rmax, view_m))
                rr = ranges[finite]
                aa = angles[finite]
                wx = rx + rr * np.cos(yaw + aa)
                wy = ry + rr * np.sin(yaw + aa)
                px = (size * 0.5 + (wx - center_x) * scale).astype(np.int32)
                py = (size * 0.5 - (wy - center_y) * scale).astype(np.int32)
                keep = (px >= 0) & (px < size) & (py >= 0) & (py < size)
                img[py[keep], px[keep]] = (0, 180, 80)

        else:
            cx, cy = size // 2, size // 2
            sx, sy, _ = scan_points(scan_msg)
            if sx is None or sy is None:
                sx = np.asarray([], dtype=np.float32)
                sy = np.asarray([], dtype=np.float32)
            px = (cx + sx * scale).astype(np.int32)
            py = (cy - sy * scale).astype(np.int32)
            keep = (px >= 0) & (px < size) & (py >= 0) & (py < size)
            img[py[keep], px[keep]] = (0, 220, 70)

        if self._path:
            pts = [world_to_px(wx, wy) for wx, wy in self._path[-1600:]]
            pts_arr = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
            if len(pts) >= 2:
                cv2.polylines(img, [pts_arr], False, (0, 165, 255), 5, cv2.LINE_AA)
                cv2.polylines(img, [pts_arr], False, (255, 245, 180), 2, cv2.LINE_AA)
            sx, sy = pts[0]
            if 0 <= sx < size and 0 <= sy < size:
                cv2.circle(img, (sx, sy), 8, (0, 210, 0), -1)
                cv2.putText(img, "START", (sx + 10, max(18, sy - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(img, "START", (sx + 10, max(18, sy - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 255, 120), 1, cv2.LINE_AA)
            ex, ey = pts[-1]
            if 0 <= ex < size and 0 <= ey < size:
                cv2.circle(img, (ex, ey), 7, (0, 220, 255), -1)

        current_xy = {
            tuple(round(float(v), 2) for v in det.get("world_xy_m", det.get("base_xy_m", [])))
            for det in detections
            if (
                isinstance(det.get("world_xy_m"), list) and len(det["world_xy_m"]) == 2
            ) or (
                isinstance(det.get("base_xy_m"), list) and len(det["base_xy_m"]) == 2
            )
        }
        for ann in self._ocr_annotations:
            map_xy = ann.get("map_xy_m", ann.get("world_xy_m"))
            if not isinstance(map_xy, list) or len(map_xy) != 2:
                continue
            mx, my = float(map_xy[0]), float(map_xy[1])
            if ann.get("coord_frame", "world") == "base" and pose is not None:
                rx, ry, yaw = pose
                cyaw = math.cos(yaw)
                syaw = math.sin(yaw)
                wx = rx + cyaw * mx - syaw * my
                wy = ry + syaw * mx + cyaw * my
            else:
                wx, wy = mx, my
            px, py = world_to_px(wx, wy)
            if not (0 <= px < size and 0 <= py < size):
                continue
            is_current = (round(wx, 2), round(wy, 2)) in current_xy
            color = (0, 0, 255) if not is_current else (0, 220, 255)
            cv2.circle(img, (px, py), 7 if not is_current else 9, color, -1)
            text = str(ann.get("text") or "OCR")[:24]
            obs = int(ann.get("observations", 1))
            label = f"{text} ({obs})"
            tx = min(px + 9, size - 180)
            ty = max(18, py - 8)
            cv2.putText(img, label, (tx + 1, ty + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(img, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 180, 255), 1, cv2.LINE_AA)

        for det in detections:
            if isinstance(det.get("world_xy_m"), list) and len(det["world_xy_m"]) == 2:
                wx, wy = [float(v) for v in det["world_xy_m"]]
            elif isinstance(det.get("base_xy_m"), list) and len(det["base_xy_m"]) == 2:
                bx, by = [float(v) for v in det["base_xy_m"]]
                if pose is not None:
                    rx, ry, yaw = pose
                    cyaw = math.cos(yaw)
                    syaw = math.sin(yaw)
                    wx = rx + cyaw * bx - syaw * by
                    wy = ry + syaw * bx + cyaw * by
                else:
                    wx, wy = bx, by
            else:
                continue
            px, py = world_to_px(wx, wy)
            if 0 <= px < size and 0 <= py < size:
                cv2.drawMarker(img, (px, py), (0, 255, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)

        if pose is not None:
            rx, ry, yaw = pose
            yaw = yaw + self.camera_yaw
            px, py = world_to_px(rx, ry)
            tip = (
                int(round(px + math.cos(yaw) * 18)),
                int(round(py - math.sin(yaw) * 18)),
            )
            left = (
                int(round(px + math.cos(yaw + 2.45) * 11)),
                int(round(py - math.sin(yaw + 2.45) * 11)),
            )
            right = (
                int(round(px + math.cos(yaw - 2.45) * 11)),
                int(round(py - math.sin(yaw - 2.45) * 11)),
            )
            cv2.fillConvexPoly(img, np.asarray([tip, left, right], dtype=np.int32), (255, 0, 0))

        if proprio is not None:
            active_pose = self._pose_xy_yaw(proprio)
            if active_pose is not None:
                px_m, py_m, _ = active_pose
                status = (
                    f"grid {self.grid_resolution:.2f}m view=+/-{self._last_view_m:.1f}m "
                    f"path={len(self._path)} "
                    f"ann={len(self._ocr_annotations)} "
                    f"pose x={px_m:.2f} y={py_m:.2f} "
                    f"src={self._pose_source(proprio)}"
                )
                cv2.putText(img, status, (12, size - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 2, cv2.LINE_AA)
                cv2.putText(img, status, (12, size - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        return buf.tobytes() if ok else None


def ocr_thread(store: FrameStore, args: argparse.Namespace, stop: threading.Event) -> None:
    projector = OcrProjector(args)
    last_key = None
    last_generation = store.reset_generation()
    while not stop.is_set():
        generation = store.reset_generation()
        if generation != last_generation:
            projector = OcrProjector(args)
            last_key = None
            last_generation = generation

        frames = store.snapshot()
        key_parts = []
        watched_topics = [
            topic for topic in sorted(frames)
            if topic == "proprio" or topic.startswith(("rgb.", "depth.", "scan"))
        ]
        for topic in watched_topics:
            msg = frames.get(topic)
            if msg is None:
                continue
            key_parts.append((topic, msg.get("stamp_ns")))
        key = tuple(key_parts)
        if not key or key == last_key:
            time.sleep(max(0.02, float(args.map_interval)))
            continue
        last_key = key

        analysis, overlay, local_map, stamp_ns = projector.run(frames)
        store.set_derived(analysis, overlay, local_map, stamp_ns)
        time.sleep(max(0.02, float(args.map_interval)))


INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Isaac Sim sensor viewer</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
* { box-sizing:border-box; }
html, body { height:100%; }
body {
  font-family:system-ui, sans-serif; background:#111; color:#ddd;
  margin:0; padding:10px; overflow:hidden;
}
h2 { margin:0; line-height:1.15; }
h3 { margin:0 0 6px; font-weight:600; font-size:13px; color:#9ad; }
.topbar {
  height:58px; display:flex; flex-direction:column; justify-content:center;
  gap:5px; min-width:0;
}
.topics {
  font-size:11px; color:#777; white-space:nowrap; overflow:hidden;
  text-overflow:ellipsis;
}
.grid {
  height:calc(100vh - 78px); min-height:0;
  display:grid; grid-template-columns:minmax(420px, 1.35fr) minmax(360px, .85fr);
  grid-template-rows:repeat(3, minmax(0, 1fr));
  grid-template-areas:
    "map rgb"
    "map depth"
    "map data";
  gap:10px;
}
.cell {
  background:#1a1a1a; padding:8px; border-radius:6px;
  min-width:0; min-height:0; overflow:hidden;
}
.map { grid-area:map; }
.rgb { grid-area:rgb; }
.depth { grid-area:depth; }
.data { grid-area:data; display:grid; grid-template-columns:1fr 1fr; gap:10px; padding:0; background:transparent; }
.data .cell { height:100%; }
.cell img {
  width:100%; height:calc(100% - 24px); display:block;
  background:#000; object-fit:contain; min-height:0;
}
.map img { height:calc(100% - 24px); }
pre {
  background:#1a1a1a; padding:8px; border-radius:6px;
  height:calc(100% - 24px); overflow:auto; font-size:11px;
  line-height:1.35; margin:0; white-space:pre-wrap; overflow-wrap:anywhere;
}
@media (max-width: 980px) {
  body { overflow:auto; padding:8px; }
  .topbar { height:auto; margin-bottom:8px; }
  .grid {
    height:auto; display:grid; grid-template-columns:1fr;
    grid-template-rows:none;
    grid-template-areas:"map" "rgb" "depth" "data";
  }
  .map { height:58vh; min-height:360px; }
  .rgb, .depth { height:34vh; min-height:220px; }
  .data { grid-template-columns:1fr; }
  .data .cell { height:260px; }
}
</style></head>
<body>
<div class="topbar">
  <h2>Isaac Sim sensor viewer</h2>
  <div class="topics" id="topics">topics: ...</div>
</div>
<div class="grid">
  <div class="cell map"><h3>2D grid map + trajectory + OCR</h3><img src="/local_map.mjpg" alt="local_map"></div>
  <div class="cell rgb"><h3>rgb.front + OCR</h3><img src="/rgb.front.ocr.mjpg" alt="rgb.front.ocr"></div>
  <div class="cell depth"><h3>depth.front</h3><img src="/depth.front.mjpg" alt="depth.front"></div>
  <div class="data">
    <div class="cell"><h3>proprio</h3><pre id="proprio">loading...</pre></div>
    <div class="cell"><h3>OCR/depth fusion</h3><pre id="analysis">loading...</pre></div>
  </div>
</div>
<script>
async function poll() {
  try {
    const [pr, tr, ar] = await Promise.all([fetch('/proprio.json'), fetch('/topics.json'), fetch('/analysis.json')]);
    if (pr.ok) document.getElementById('proprio').textContent = await pr.text();
    if (ar.ok) document.getElementById('analysis').textContent = await ar.text();
    if (tr.ok) {
      const ts = await tr.json();
      document.getElementById('topics').textContent = 'live topics: ' + ts.join(', ');
    }
  } catch (e) {}
  setTimeout(poll, 1000);
}
poll();
</script>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    store: FrameStore

    def log_message(self, fmt, *args):
        return

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path in ("/reset_local", "/reset_local.json"):
            return self._serve_reset_local()
        self.send_error(404, f"unknown path {path!r}")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            return self._send_bytes(INDEX_HTML.encode(), "text/html; charset=utf-8")
        if path in ("/reset_local", "/reset_local.json"):
            return self._serve_reset_local()
        if path == "/topics.json":
            return self._send_bytes(json.dumps(self.store.keys()).encode(), "application/json")
        if path == "/analysis.json":
            return self._send_bytes(
                json.dumps(self.store.get_analysis(), indent=2, ensure_ascii=False).encode(),
                "application/json",
            )
        if path == "/proprio.json":
            return self._serve_proprio()
        if path.endswith(".mjpg"):
            return self._serve_mjpeg(path[1:-len(".mjpg")])
        self.send_error(404, f"unknown path {path!r}")

    def _serve_reset_local(self) -> None:
        generation = self.store.reset_local()
        body = json.dumps({"ok": True, "reset_generation": generation}).encode()
        self._send_bytes(body, "application/json")

    def _send_bytes(self, body: bytes, ctype: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_proprio(self) -> None:
        msg = self.store.get("proprio")
        if msg is None:
            self.send_error(503, "no proprio frame yet")
            return
        body = json.dumps(msg, indent=2, default=_json_default).encode()
        self._send_bytes(body, "application/json")

    def _serve_mjpeg(self, topic: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_stamp = None
        try:
            while True:
                if topic == "rgb.front.ocr":
                    jpg, stamp = self.store.get_derived_jpeg(topic)
                    raw_msg = self.store.get("rgb.front")
                    raw_stamp = raw_msg.get("stamp_ns") if raw_msg is not None else None
                    stale = (
                        raw_stamp is not None
                        and stamp is not None
                        and int(raw_stamp) - int(stamp) > 500_000_000
                    )
                    if jpg is None or stale:
                        raw_jpg = render_jpeg("rgb.front", raw_msg) if raw_msg is not None else None
                        if raw_jpg is not None:
                            jpg = raw_jpg
                            stamp = raw_stamp
                elif topic == "local_map":
                    jpg, stamp = self.store.get_derived_jpeg(topic)
                else:
                    msg = self.store.get(topic)
                    if msg is None and topic == "scan":
                        msg = self.store.get_first_prefix("scan")
                    jpg = render_jpeg(topic, msg) if msg is not None else None
                    stamp = msg.get("stamp_ns") if msg is not None else None
                if jpg is None:
                    jpg = status_jpeg(topic, "waiting for live data\ncheck /topics.json")
                    stamp = int(time.time())
                if jpg is None or stamp == last_stamp:
                    time.sleep(0.02)
                    continue
                last_stamp = stamp
                hdr = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(hdr)
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-host", default="localhost", help="ZMQ PUB host")
    parser.add_argument("--pub-port", type=int, default=5555)
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--ocr-backend", choices=("gazebo", "paddle", "easyocr", "tesseract", "none"), default="gazebo")
    parser.add_argument("--ocr-langs", default="ko,en")
    parser.add_argument("--ocr-interval", type=float, default=2.0)
    parser.add_argument("--map-interval", type=float, default=0.20)
    parser.add_argument("--ocr-min-confidence", type=float, default=0.25)
    parser.add_argument("--ocr-merge-radius-m", type=float, default=0.75)
    parser.add_argument("--floor-hint", default=os.environ.get("FLOOR_HINT", "5"))
    parser.add_argument("--floor-prior-mode", choices=("reject", "complete"), default=os.environ.get("FLOOR_PRIOR_MODE", "complete"))
    parser.add_argument("--ocr-scales", default=os.environ.get("OCR_SCALES", "1.0,2.0,3.0,4.0,6.0"))
    parser.add_argument("--ocr-max-side", type=int, default=int(os.environ.get("OCR_MAX_SIDE", "2400")))
    parser.add_argument("--camera-x-m", type=float, default=-0.147)
    parser.add_argument("--camera-y-m", type=float, default=0.0)
    parser.add_argument("--camera-z-m", type=float, default=1.19065)
    parser.add_argument("--camera-yaw-offset-rad", type=float, default=0.0)
    parser.add_argument("--camera-hfov-rad", type=float, default=1.518)
    parser.add_argument("--grid-resolution-m", type=float, default=0.10)
    parser.add_argument("--grid-size-m", type=float, default=80.0)
    parser.add_argument("--grid-view-m", type=float, default=18.0)
    parser.add_argument("--disable-grid-auto-fit", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    store = FrameStore()
    stop = threading.Event()
    thread = threading.Thread(
        target=reader_thread, args=(args.sim_host, args.pub_port, store, stop), daemon=True
    )
    thread.start()
    derived_thread = threading.Thread(target=ocr_thread, args=(store, args, stop), daemon=True)
    derived_thread.start()

    Handler.store = store
    server = ThreadingHTTPServer((args.http_host, args.http_port), Handler)
    print(f"viewer http://{args.http_host}:{args.http_port}/  sim PUB tcp://{args.sim_host}:{args.pub_port}")
    print(
        f"ocr backend={args.ocr_backend} langs={args.ocr_langs} "
        f"floor={args.floor_hint} prior={args.floor_prior_mode} scales={args.ocr_scales}"
    )
    if not HAVE_CV:
        print("[warn] numpy/opencv missing: depth, lidar, OCR overlay disabled")
    if not HAVE_ZSTD:
        print("[warn] zstandard missing: depth disabled")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        stop.set()
        server.server_close()


if __name__ == "__main__":
    main()
