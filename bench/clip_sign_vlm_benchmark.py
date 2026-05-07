#!/usr/bin/env python3
"""Offline multi-frame OCR/VLM benchmark for room-ID signs.

This is deliberately video/clip oriented.  It does not use ROS 2 topics:
MP4 RGB frames are read directly, grouped into short clips, and recognized
slowly for accuracy-oriented inspection.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import threading
import time
from collections import Counter
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOM_ID_RE = re.compile(r'(?<![0-9A-Z])(?:[A-Z][ -]?)?\d{3,4}(?![0-9A-Z])', re.IGNORECASE)

DEFAULT_FLOOR_HINTS = {
    'VID_20260429_221116_981': '4F',
    'VID_20260429_221947_605': '13F',
    'VID_20260429_222101_976': '13F',
    '310_b3': 'B3F',
}

DEFAULT_ROTATION_HINTS = {
    '310_b3': 'none',
}


class GpuMonitor:
    def __init__(self, interval_s: float = 0.5):
        self.interval_s = interval_s
        self.samples: list[dict[str, float]] = []
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.samples.clear()
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> dict[str, Any]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        mem = [s['memory_used_mb'] for s in self.samples]
        util = [s['util_pct'] for s in self.samples]
        return {
            'samples': len(self.samples),
            'memory_used_mb': {
                'max': max(mem) if mem else None,
                'mean': float(np.mean(mem)) if mem else None,
                'median': float(np.median(mem)) if mem else None,
            },
            'util_pct': {
                'max': max(util) if util else None,
                'mean': float(np.mean(util)) if util else None,
            },
            'error': self.error,
        }

    def _run(self) -> None:
        while not self._stop.is_set():
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
                    timeout=2.0,
                )
                if proc.returncode != 0:
                    self.error = proc.stderr.strip() or f'nvidia-smi exited {proc.returncode}'
                else:
                    mem_total = 0.0
                    util_vals = []
                    for line in proc.stdout.strip().splitlines():
                        parts = [p.strip() for p in line.split(',')]
                        if len(parts) >= 2:
                            mem_total += float(parts[0])
                            util_vals.append(float(parts[1]))
                    self.samples.append({
                        't': time.time(),
                        'memory_used_mb': mem_total,
                        'util_pct': float(np.mean(util_vals)) if util_vals else 0.0,
                    })
            except Exception as exc:
                self.error = str(exc)
            self._stop.wait(self.interval_s)


def _resize_rgb(rgb: np.ndarray, max_side: int) -> np.ndarray:
    h, w = rgb.shape[:2]
    scale = min(1.0, float(max_side) / float(max(h, w)))
    if scale >= 1.0:
        return rgb
    return cv2.resize(rgb, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_AREA)


def _rotate_rgb(rgb: np.ndarray, rotation: str) -> np.ndarray:
    if rotation == 'none':
        return rgb
    if rotation == '90ccw':
        return np.ascontiguousarray(np.rot90(rgb, 1))
    if rotation == '90cw':
        return np.ascontiguousarray(np.rot90(rgb, -1))
    if rotation == '180':
        return np.ascontiguousarray(np.rot90(rgb, 2))
    if rotation == 'auto':
        return np.ascontiguousarray(np.rot90(rgb, 1)) if rgb.shape[0] > rgb.shape[1] else rgb
    raise ValueError(f'unknown rotation: {rotation}')


def _normalize_floor_hint(value: str | None) -> str | None:
    if value is None:
        return None
    text = str(value).strip().upper().replace(' ', '')
    if not text:
        return None
    if text in ('4', '4F', 'F4', '4TH'):
        return '4F'
    if text in ('13', '13F', 'F13', '13TH'):
        return '13F'
    if text in ('B3', 'B3F', 'FB3', 'BASEMENT3', 'BASEMENT-3'):
        return 'B3F'
    return text


def _parse_floor_hints(text: str | None) -> dict[str, str]:
    hints = dict(DEFAULT_FLOOR_HINTS)
    if not text:
        return hints
    for part in text.split(','):
        if not part.strip() or '=' not in part:
            continue
        key, value = part.split('=', 1)
        hint = _normalize_floor_hint(value)
        if key.strip() and hint:
            hints[key.strip()] = hint
    return hints


def _parse_rotation_hints(text: str | None) -> dict[str, str]:
    hints = dict(DEFAULT_ROTATION_HINTS)
    if not text:
        return hints
    valid = {'auto', 'none', '90ccw', '90cw', '180'}
    for part in text.split(','):
        if not part.strip() or '=' not in part:
            continue
        key, value = part.split('=', 1)
        value = value.strip()
        if key.strip() and value in valid:
            hints[key.strip()] = value
    return hints


def _hint_for_video(video: Path, hints: dict[str, str]) -> str | None:
    stem = video.stem
    lower = stem.lower()
    for key, hint in hints.items():
        if key == stem or key in stem or key.lower() in lower:
            return hint
    return None


def _floor_hint_for_video(video: Path, hints: dict[str, str]) -> str | None:
    hint = _hint_for_video(video, hints)
    if hint is not None:
        return _normalize_floor_hint(hint)
    lower = video.stem.lower()
    if '310_b3' in lower or lower.endswith('_b3'):
        return 'B3F'
    return None


def _rotation_for_video(video: Path, global_rotation: str, hints: dict[str, str]) -> str:
    if global_rotation != 'auto':
        return global_rotation
    return _hint_for_video(video, hints) or global_rotation


def _apply_floor_prior(room_id: str, floor_hint: str | None, floor_prior_mode: str = 'reject') -> str | None:
    hint = _normalize_floor_hint(floor_hint)
    complete = floor_prior_mode == 'complete'
    compact = re.sub(r'[\s-]+', '', room_id.upper())
    match = re.match(r'^([A-Z])?(\d{3,4})$', compact)
    if not match:
        return None
    letter, digits = match.groups()
    letter = letter or ''
    if hint == '4F':
        return digits if not letter and digits.startswith('4') else None
    if hint == '13F':
        if not letter and digits.startswith('13') and len(digits) == 4:
            return digits
        if complete and not letter and len(digits) == 3 and digits.startswith('3'):
            return f'13{digits[1:]}'
        return None
    if hint == 'B3F':
        if letter == 'B' and len(digits) == 3 and digits.startswith('3'):
            return f'B{digits}'
        if complete and not letter and len(digits) == 3 and digits.startswith('3'):
            return f'B{digits}'
        return None
    return compact


def _normalize_room_id(
    text: str | None,
    floor_hint: str | None = None,
    floor_prior_mode: str = 'reject',
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
        floor_prior_mode=floor_prior_mode,
    )


def _clamp_bbox(bbox: list[float] | tuple[float, ...] | None, w: int, h: int) -> list[int] | None:
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


def _extract_json(raw: str) -> Any | None:
    text = raw.strip()
    if text.startswith('```'):
        text = re.sub(r'^```(?:json)?\s*', '', text)
        text = re.sub(r'\s*```$', '', text)
    try:
        return json.loads(text)
    except Exception:
        pass
    start = text.find('{')
    end = text.rfind('}')
    arr_start = text.find('[')
    arr_end = text.rfind(']')
    if arr_start >= 0 and arr_end > arr_start and (start < 0 or arr_start < start):
        try:
            return json.loads(text[arr_start:arr_end + 1])
        except Exception:
            pass
    if start >= 0 and end > start:
        try:
            return json.loads(text[start:end + 1])
        except Exception:
            return None
    return None


def _load_vlm(model_name: str, device: str, dtype: str, local_files_only: bool):
    _debug_marker('load_vlm import torch')
    import torch
    _debug_marker('load_vlm import transformers')
    from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration

    _debug_marker('load_vlm choose device')
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    if dtype == 'auto':
        torch_dtype: str | Any = 'auto'
    elif dtype in ('float16', 'fp16'):
        torch_dtype = torch.float16
    elif dtype in ('bfloat16', 'bf16'):
        torch_dtype = torch.bfloat16
    else:
        torch_dtype = torch.float32
    _debug_marker('load_vlm processor from_pretrained')
    processor = AutoProcessor.from_pretrained(
        model_name,
        trust_remote_code=True,
        local_files_only=local_files_only,
    )
    _debug_marker('load_vlm model from_pretrained')
    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        model_name,
        torch_dtype=torch_dtype,
        trust_remote_code=True,
        local_files_only=local_files_only,
    ).eval()
    _debug_marker('load_vlm model to device')
    if device != 'cpu':
        model.to(device)
    _debug_marker('load_vlm done')
    return torch, processor, model, device


def _parse_scales(text: str) -> list[float]:
    scales: list[float] = []
    for part in text.split(','):
        part = part.strip()
        if not part:
            continue
        value = max(0.25, min(4.0, float(part)))
        if value not in scales:
            scales.append(value)
    return scales or [1.0]


def _scaled_rgb(rgb: np.ndarray, scale: float) -> np.ndarray:
    if abs(scale - 1.0) < 1e-6:
        return rgb
    h, w = rgb.shape[:2]
    return cv2.resize(
        rgb,
        (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
        interpolation=cv2.INTER_CUBIC if scale > 1.0 else cv2.INTER_AREA,
    )


def _clean_text(text: str) -> str:
    return re.sub(r'\s+', ' ', text.strip())


def _append_paddle_detections(detections: list[dict], lines: Any, scale: float, w: int, h: int) -> None:
    for line in lines or []:
        try:
            pts = np.asarray(line[0], dtype=np.float32) / float(scale)
            text = _clean_text(str(line[1][0]))
            conf = float(line[1][1])
        except Exception:
            continue
        if not text:
            continue
        bbox = _clamp_bbox([
            float(pts[:, 0].min()),
            float(pts[:, 1].min()),
            float(pts[:, 0].max()),
            float(pts[:, 1].max()),
        ], w, h)
        detections.append({
            'source': f'paddleocr@{scale:g}x',
            'text': text,
            'confidence': conf,
            'bbox_xyxy': bbox,
        })


def _postprocess_detections(
    detections: list[dict],
    w: int,
    h: int,
    rgb: np.ndarray | None = None,
    method: str = 'generic',
    floor_hint: str | None = None,
    floor_prior_mode: str = 'reject',
    min_confidence: float = 0.0,
) -> list[dict]:
    del rgb, method
    out: list[dict] = []
    for det in detections:
        try:
            conf = float(det.get('confidence'))
        except Exception:
            conf = 0.0
        if min_confidence > 0.0 and conf <= min_confidence:
            continue
        text = det.get('room_id') or det.get('text')
        room_id = _normalize_room_id(text, floor_hint=floor_hint, floor_prior_mode=floor_prior_mode)
        if not room_id:
            continue
        bbox = _clamp_bbox(det.get('bbox_xyxy'), w, h)
        out.append({**det, 'text': room_id, 'room_id': room_id, 'bbox_xyxy': bbox})
    return out


def _font(size: int) -> ImageFont.ImageFont:
    for path in (
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    ):
        try:
            return ImageFont.truetype(path, size=size)
        except Exception:
            continue
    return ImageFont.load_default()


def _debug_marker(message: str) -> None:
    if os.environ.get('CLIP_BENCH_DEBUG'):
        print(f'[clip-debug] {message}', flush=True)


@dataclass
class FrameItem:
    frame_index: int
    timestamp_s: float
    ocr_rgb: np.ndarray
    vlm_rgb: np.ndarray


@dataclass
class Detection:
    method: str
    video: str
    video_stem: str
    clip_index: int
    clip_start_frame: int
    clip_end_frame: int
    frame_index: int
    timestamp_s: float
    room_id: str
    raw_text: str
    confidence: float | str | None
    bbox_xyxy: list[int] | None
    source: str
    evidence_frames: list[int]
    depth_m: float | None = None
    error: str | None = None


def _parse_ground_truth(text: str | None) -> dict[str, int]:
    out: dict[str, int] = {}
    if not text:
        return out
    for part in text.split(','):
        part = part.strip()
        if not part or '=' not in part:
            continue
        key, value = part.split('=', 1)
        try:
            out[key.strip()] = int(value)
        except ValueError:
            continue
    return out


def _hint_for_stem(stem: str, values: dict[str, int]) -> int | None:
    lower = stem.lower()
    for key, value in values.items():
        key_l = key.lower()
        if key == stem or key in stem or key_l in lower:
            return value
    return None


def _conf_score(conf: float | str | None) -> float:
    if isinstance(conf, (int, float)):
        return float(conf)
    text = str(conf or '').lower()
    if text == 'high':
        return 0.95
    if text == 'medium':
        return 0.65
    if text == 'low':
        return 0.35
    return 0.0


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


def _sample_clip_frames(frames: list[FrameItem], step: int) -> list[FrameItem]:
    step = max(1, int(step))
    sampled = frames[::step]
    if frames and frames[-1] not in sampled:
        sampled.append(frames[-1])
    return sampled or frames


def _clip_prompt(floor_hint: str | None) -> tuple[str, str]:
    hint = _normalize_floor_hint(floor_hint)
    floor_line = ''
    if hint == '4F':
        floor_line = 'Floor prior: 4th floor. Use this only to reject incompatible readings; never invent a room ID from this prior.'
    elif hint == '13F':
        floor_line = 'Floor prior: 13th floor. Use this only to reject incompatible readings; never invent a room ID from this prior.'
    elif hint == 'B3F':
        floor_line = 'Floor prior: basement B3 floor. Use this only to reject incompatible readings; never invent a room ID from this prior.'

    system = """You are a high-precision video text recognition module.

Input: an ordered short clip of RGB camera frames from the same hallway traversal.

Task:
Use multiple adjacent frames together to detect every distinct physical room-ID door plate or wall sign visible anywhere in the clip. The same sign may be blurry in one frame and clear in another; use the clearest frame for reading the text.

Rules:
- Return only valid JSON.
- Do not wrap JSON in Markdown fences.
- Output one object per distinct physical sign in this clip, not one object per frame.
- Return at most 3 objects. Do not duplicate the same physical sign.
- Target room IDs are 3 or 4 digits, or one optional Latin letter followed by 3 or 4 digits.
- Do not infer missing digits and do not complete text from hallway context alone.
- If a prefix letter is not visibly printed, do not invent it.
- Prefer false negatives over hallucinated text.
- If no room-ID sign is clearly readable in these frames, return {"objects": [], "need_human_check": true}.
- Do not output a candidate unless the digits are visibly printed on a sign/plate in at least one supplied frame.
- The floor prior is a rejection filter only. It is not evidence that any specific number is visible.
- Ignore posters, generic notices, walls, blank doors, reflections, and ceiling/floor structure.
- Use best_frame_number as the 1-based input image number among the supplied images.
- evidence_frame_numbers must contain only 1-based input image numbers among the supplied images, at most 5 values.
- Do not output video frame numbers inside best_frame_number or evidence_frame_numbers.
- bbox_xyxy is optional. If present, it must be in that best frame's pixel coordinates.
- For bbox_xyxy, output either a numeric JSON list or null. Never output schema text such as "| null".
- The schema below uses placeholder names only. Do not copy placeholder values as detections.

Output schema:
{
  "objects": [
    {
      "best_frame_number": 1,
      "evidence_frame_numbers": [1, 2],
      "room_id": "VISIBLE_ROOM_ID_STRING",
      "text": "VISIBLE_ROOM_ID_STRING",
      "bbox_xyxy": [x1, y1, x2, y2] | null,
      "confidence": "low|medium|high",
      "failure_reason": "none|blur|small_text|occlusion|glare|low_resolution|motion|angle|unknown"
    }
  ],
  "need_human_check": boolean
}"""
    user = (
        'The following images are ordered video frames from one short clip. '
        'Read room-ID signs using all frames jointly. '
        f'{floor_line} '
        'Return only the JSON object.'
    )
    return system, user


def _crop_verify_prompt(floor_hint: str | None) -> tuple[str, str]:
    hint = _normalize_floor_hint(floor_hint)
    floor_line = ''
    if hint == '4F':
        floor_line = 'Floor prior: 4th floor. Use only to reject incompatible text.'
    elif hint == '13F':
        floor_line = 'Floor prior: 13th floor. Use only to reject incompatible text.'
    elif hint == 'B3F':
        floor_line = 'Floor prior: B3 floor. Use only to reject incompatible text.'
    system = """You are verifying a cropped image of a possible room-ID sign.

Rules:
- Return only valid JSON.
- Read only text visibly printed inside the crop.
- Do not use hallway context or floor prior to guess missing digits.
- If the crop is not a room-ID sign, or the digits are not clearly readable, set readable=false and room_id=null.
- If readable=true, room_id must be exactly the visible room ID string.

Return:
{"readable": boolean, "room_id": string|null, "confidence": "low|medium|high"}"""
    user = f'Read the room-ID sign in this crop. {floor_line} Return only JSON.'
    return system, user


def _extract_json_relaxed(raw: str) -> Any | None:
    parsed = _extract_json(raw)
    if isinstance(parsed, (dict, list)):
        return parsed
    # Qwen sometimes copies the schema union marker, e.g.
    # "bbox_xyxy": [1, 2, 3, 4] | null.  Keep the concrete list.
    fixed = re.sub(r'(\[[^\[\]]{1,160}\])\s*\|\s*null', r'\1', raw)
    fixed = re.sub(r'null\s*\|\s*(\[[^\[\]]{1,160}\])', r'\1', fixed)
    fixed = re.sub(r'^```(?:json)?\s*', '', fixed.strip())
    fixed = re.sub(r'\s*`+\s*$', '', fixed.strip())
    return _extract_json(fixed)


def _extract_object_fragments(raw: str) -> list[dict[str, Any]]:
    objects: list[dict[str, Any]] = []
    text = re.sub(r'(\[[^\[\]]{1,160}\])\s*\|\s*null', r'\1', raw)
    text = re.sub(r'null\s*\|\s*(\[[^\[\]]{1,160}\])', r'\1', text)
    for match in re.finditer(r'\{[^{}]*"room_id"[^{}]*\}', text, flags=re.DOTALL):
        fragment = match.group(0)
        try:
            parsed = json.loads(fragment)
        except Exception:
            continue
        if isinstance(parsed, dict):
            objects.append(parsed)
    return objects


def _read_clip(
    cap: cv2.VideoCapture,
    start: int,
    clip_size: int,
    fps: float,
    rotation: str,
    ocr_max_side: int,
    vlm_max_side: int,
) -> list[FrameItem]:
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(start))
    frames: list[FrameItem] = []
    for offset in range(clip_size):
        ok, bgr = cap.read()
        if not ok:
            break
        idx = start + offset
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = _rotate_rgb(rgb, rotation)
        frames.append(FrameItem(
            frame_index=int(idx),
            timestamp_s=(float(idx) / fps) if fps > 0 else 0.0,
            ocr_rgb=_resize_rgb(rgb, ocr_max_side),
            vlm_rgb=_resize_rgb(rgb, vlm_max_side),
        ))
    return frames


def _init_paddle(use_gpu: bool):
    try:
        from paddleocr import PaddleOCR

        return PaddleOCR(use_angle_cls=True, lang='en', use_gpu=use_gpu, show_log=False), None
    except Exception as exc:
        return None, str(exc)


def _ocr_backend_version(backend: str) -> str | None:
    if backend == 'paddle':
        try:
            import paddleocr

            version = getattr(paddleocr, '__version__', None)
            return str(version) if version is not None else None
        except Exception:
            return None
    if backend == 'tesseract':
        try:
            import pytesseract

            return str(pytesseract.get_tesseract_version())
        except Exception:
            return None
    return None


def _ocr_frame(
    paddle,
    rgb: np.ndarray,
    floor_hint: str | None,
    scales: list[float],
    floor_prior_mode: str,
    min_confidence: float,
) -> tuple[list[dict], str | None]:
    h, w = rgb.shape[:2]
    detections: list[dict] = []
    frame_error: str | None = None
    if paddle is not None:
        for scale in scales:
            try:
                rgb_in = _scaled_rgb(rgb, scale)
                out = paddle.ocr(rgb_in, cls=True)
                lines = out[0] if out and isinstance(out[0], list) else out
                _append_paddle_detections(detections, lines, scale, w, h)
            except Exception as exc:
                frame_error = str(exc)
    if paddle is None or frame_error:
        try:
            import pytesseract

            data = pytesseract.image_to_data(
                Image.fromarray(rgb),
                lang='eng',
                config='--oem 3 --psm 11 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-',
                output_type=pytesseract.Output.DICT,
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
                detections.append({
                    'source': 'pytesseract',
                    'text': text,
                    'confidence': conf / 100.0,
                    'bbox_xyxy': _clamp_bbox([x, y, x + bw, y + bh], w, h),
                })
        except Exception as exc:
            frame_error = f'{frame_error}; tesseract fallback failed: {exc}' if frame_error else str(exc)
    return _postprocess_detections(
        detections,
        w,
        h,
        rgb,
        method='ocr',
        floor_hint=floor_hint,
        floor_prior_mode=floor_prior_mode,
        min_confidence=min_confidence,
    ), frame_error


def _run_ocr_clip(
    paddle,
    frames: list[FrameItem],
    video: Path,
    clip_index: int,
    clip_start: int,
    floor_hint: str | None,
    scales: list[float],
    floor_prior_mode: str,
    min_confidence: float,
) -> tuple[list[Detection], list[str]]:
    detections: list[Detection] = []
    errors: list[str] = []
    clip_end = frames[-1].frame_index if frames else clip_start
    for frame in frames:
        frame_dets, err = _ocr_frame(
            paddle,
            frame.ocr_rgb,
            floor_hint,
            scales,
            floor_prior_mode,
            min_confidence,
        )
        if err:
            errors.append(f'frame {frame.frame_index}: {err}')
        for det in frame_dets:
            room_id = det.get('room_id') or _normalize_room_id(
                det.get('text'),
                floor_hint,
                floor_prior_mode=floor_prior_mode,
            )
            if not room_id:
                continue
            detections.append(Detection(
                method='ocr',
                video=str(video),
                video_stem=video.stem,
                clip_index=clip_index,
                clip_start_frame=clip_start,
                clip_end_frame=clip_end,
                frame_index=frame.frame_index,
                timestamp_s=frame.timestamp_s,
                room_id=room_id,
                raw_text=str(det.get('raw_text') or det.get('text') or room_id),
                confidence=det.get('confidence'),
                bbox_xyxy=det.get('bbox_xyxy'),
                source=str(det.get('source') or 'ocr'),
                evidence_frames=[frame.frame_index],
            ))
    return detections, errors


def _run_vlm_clip(
    torch,
    processor,
    model,
    device: str,
    frames: list[FrameItem],
    video: Path,
    clip_index: int,
    clip_start: int,
    floor_hint: str | None,
    max_new_tokens: int,
) -> tuple[list[Detection], str | None, str]:
    if not frames:
        return [], None, ''
    system, user = _clip_prompt(floor_hint)
    pil_frames = [Image.fromarray(f.vlm_rgb).convert('RGB') for f in frames]
    content: list[dict[str, Any]] = [{'type': 'text', 'text': user}]
    for i, (frame, pil) in enumerate(zip(frames, pil_frames), 1):
        content.append({
            'type': 'text',
            'text': f'\nFrame {i} (video_frame={frame.frame_index}, t={frame.timestamp_s:.2f}s):',
        })
        content.append({'type': 'image', 'image': pil})
    messages = [
        {'role': 'system', 'content': system},
        {'role': 'user', 'content': content},
    ]
    decoded = ''
    try:
        text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        inputs = processor(text=[text], images=pil_frames, padding=True, return_tensors='pt')
        inputs = {k: v.to(device) if hasattr(v, 'to') else v for k, v in inputs.items()}
        input_len = int(inputs['input_ids'].shape[-1])
        with torch.no_grad():
            generated = model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                do_sample=False,
            )
        decoded = processor.batch_decode(
            generated[:, input_len:],
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0].strip()
    except Exception as exc:
        if hasattr(torch, 'cuda') and torch.cuda.is_available():
            torch.cuda.empty_cache()
        return [], str(exc), decoded

    parsed = _extract_json_relaxed(decoded)
    if isinstance(parsed, list):
        parsed = {'objects': parsed}
    if isinstance(parsed, dict):
        objects = parsed.get('objects') or []
    else:
        objects = _extract_object_fragments(decoded)
    if not isinstance(objects, list):
        return [], 'json_parse_failed', decoded

    detections: list[Detection] = []
    clip_end = frames[-1].frame_index
    for obj in objects[:6]:
        if not isinstance(obj, dict):
            continue
        raw_text = obj.get('room_id') or obj.get('text')
        if raw_text is None or str(raw_text).strip().lower() == 'null':
            continue
        room_id = _normalize_room_id(str(raw_text), floor_hint)
        if not room_id:
            continue
        try:
            best_n = int(obj.get('best_frame_number') or obj.get('frame_number') or 1)
        except Exception:
            best_n = 1
        best_n = max(1, min(len(frames), best_n))
        best_frame = frames[best_n - 1]
        h, w = best_frame.vlm_rgb.shape[:2]
        bbox = _clamp_bbox(obj.get('bbox_xyxy'), w, h)
        ev_frames: list[int] = []
        for ev in obj.get('evidence_frame_numbers') or [best_n]:
            try:
                ev_n = max(1, min(len(frames), int(ev)))
            except Exception:
                continue
            ev_frames.append(frames[ev_n - 1].frame_index)
        if not ev_frames:
            ev_frames = [best_frame.frame_index]
        detections.append(Detection(
            method='vlm',
            video=str(video),
            video_stem=video.stem,
            clip_index=clip_index,
            clip_start_frame=clip_start,
            clip_end_frame=clip_end,
            frame_index=best_frame.frame_index,
            timestamp_s=best_frame.timestamp_s,
            room_id=room_id,
            raw_text=str(raw_text),
            confidence=obj.get('confidence'),
            bbox_xyxy=bbox,
            source='vlm_clip20',
            evidence_frames=sorted(set(ev_frames)),
        ))
    return detections, None, decoded


def _crop_from_bbox(rgb: np.ndarray, bbox: list[int], pad_ratio: float, max_side: int) -> Image.Image | None:
    h, w = rgb.shape[:2]
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    pad = int(round(max(bw, bh) * pad_ratio))
    x1 = max(0, x1 - pad)
    y1 = max(0, y1 - pad)
    x2 = min(w - 1, x2 + pad)
    y2 = min(h - 1, y2 + pad)
    if x2 <= x1 or y2 <= y1:
        return None
    crop = Image.fromarray(rgb[y1:y2 + 1, x1:x2 + 1]).convert('RGB')
    side = max(crop.width, crop.height)
    if side > 0:
        scale = min(8.0, max(1.0, float(max_side) / float(side)))
        if scale > 1.01:
            crop = crop.resize(
                (max(1, int(round(crop.width * scale))), max(1, int(round(crop.height * scale)))),
                Image.Resampling.LANCZOS,
            )
    return crop


def _verify_vlm_crop(
    torch,
    processor,
    model,
    device: str,
    crop: Image.Image,
    floor_hint: str | None,
    max_new_tokens: int,
) -> tuple[str | None, str | None, str | None, str]:
    system, user = _crop_verify_prompt(floor_hint)
    messages = [
        {'role': 'system', 'content': system},
        {'role': 'user', 'content': [
            {'type': 'image', 'image': crop},
            {'type': 'text', 'text': user},
        ]},
    ]
    decoded = ''
    try:
        text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        inputs = processor(text=[text], images=[crop], padding=True, return_tensors='pt')
        inputs = {k: v.to(device) if hasattr(v, 'to') else v for k, v in inputs.items()}
        input_len = int(inputs['input_ids'].shape[-1])
        with torch.no_grad():
            generated = model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                do_sample=False,
            )
        decoded = processor.batch_decode(
            generated[:, input_len:],
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0].strip()
    except Exception as exc:
        if hasattr(torch, 'cuda') and torch.cuda.is_available():
            torch.cuda.empty_cache()
        return None, None, str(exc), decoded
    parsed = _extract_json_relaxed(decoded)
    if not isinstance(parsed, dict):
        return None, None, 'json_parse_failed', decoded
    readable = parsed.get('readable')
    if isinstance(readable, str):
        readable = readable.strip().lower() == 'true'
    if not readable:
        return None, parsed.get('confidence'), None, decoded
    room_id = _normalize_room_id(parsed.get('room_id'), floor_hint=floor_hint)
    if not room_id:
        return None, parsed.get('confidence'), None, decoded
    return room_id, parsed.get('confidence'), None, decoded


def _verify_clip_detections(
    torch,
    processor,
    model,
    device: str,
    detections: list[Detection],
    frames: list[FrameItem],
    floor_hint: str | None,
    max_new_tokens: int,
    crop_pad_ratio: float,
    crop_max_side: int,
) -> tuple[list[Detection], list[dict[str, Any]]]:
    if not detections:
        return [], []
    frame_by_index = {frame.frame_index: frame for frame in frames}
    verified: list[Detection] = []
    errors: list[dict[str, Any]] = []
    for det in detections:
        if not det.bbox_xyxy:
            errors.append({
                'stage': 'verify_crop',
                'room_id': det.room_id,
                'frame_index': det.frame_index,
                'error': 'missing_bbox',
            })
            continue
        frame = frame_by_index.get(det.frame_index)
        if frame is None:
            errors.append({
                'stage': 'verify_crop',
                'room_id': det.room_id,
                'frame_index': det.frame_index,
                'error': 'missing_frame',
            })
            continue
        crop = _crop_from_bbox(frame.vlm_rgb, det.bbox_xyxy, crop_pad_ratio, crop_max_side)
        if crop is None:
            errors.append({
                'stage': 'verify_crop',
                'room_id': det.room_id,
                'frame_index': det.frame_index,
                'error': 'empty_crop',
            })
            continue
        room_id, confidence, error, raw = _verify_vlm_crop(
            torch,
            processor,
            model,
            device,
            crop,
            floor_hint,
            max_new_tokens,
        )
        if error:
            errors.append({
                'stage': 'verify_crop',
                'room_id': det.room_id,
                'frame_index': det.frame_index,
                'error': error,
                'raw': raw[:800],
            })
            continue
        if room_id is None:
            continue
        if room_id != det.room_id:
            errors.append({
                'stage': 'verify_crop',
                'room_id': det.room_id,
                'verified_room_id': room_id,
                'frame_index': det.frame_index,
                'error': 'room_id_mismatch',
                'raw': raw[:800],
            })
            continue
        verified.append(Detection(
            method=det.method,
            video=det.video,
            video_stem=det.video_stem,
            clip_index=det.clip_index,
            clip_start_frame=det.clip_start_frame,
            clip_end_frame=det.clip_end_frame,
            frame_index=det.frame_index,
            timestamp_s=det.timestamp_s,
            room_id=det.room_id,
            raw_text=f'{det.raw_text} | verified={room_id}',
            confidence=confidence or det.confidence,
            bbox_xyxy=det.bbox_xyxy,
            source=f'{det.source}+crop_verify',
            evidence_frames=det.evidence_frames,
        ))
    return _dedupe_clip(verified), errors


def _dedupe_clip(detections: list[Detection]) -> list[Detection]:
    best: dict[tuple[str, str], Detection] = {}
    for det in detections:
        key = (det.method, det.room_id)
        old = best.get(key)
        if old is None or _conf_score(det.confidence) > _conf_score(old.confidence):
            best[key] = det
        elif old is not None and det.frame_index not in old.evidence_frames:
            old.evidence_frames = sorted(set(old.evidence_frames + det.evidence_frames))
    return sorted(best.values(), key=lambda d: (d.method, d.room_id, d.frame_index))


def _track_instances(detections: list[Detection], max_gap_frames: int) -> list[dict[str, Any]]:
    tracks: list[dict[str, Any]] = []
    by_id: dict[str, list[Detection]] = {}
    for det in detections:
        by_id.setdefault(det.room_id, []).append(det)
    for room_id, dets in by_id.items():
        cur: dict[str, Any] | None = None
        for det in sorted(dets, key=lambda d: d.frame_index):
            if cur is None or det.frame_index - int(cur['last_frame']) > max_gap_frames:
                cur = {
                    'room_id': room_id,
                    'first_frame': det.frame_index,
                    'last_frame': det.frame_index,
                    'best_frame': det.frame_index,
                    'best_confidence': det.confidence,
                    'observations': 1,
                    'evidence_frames': list(det.evidence_frames),
                }
                tracks.append(cur)
            else:
                cur['last_frame'] = det.frame_index
                cur['observations'] = int(cur['observations']) + 1
                cur['evidence_frames'] = sorted(set(cur['evidence_frames'] + det.evidence_frames))
                if _conf_score(det.confidence) > _conf_score(cur.get('best_confidence')):
                    cur['best_frame'] = det.frame_index
                    cur['best_confidence'] = det.confidence
    return sorted(tracks, key=lambda t: (t['first_frame'], t['room_id']))


def _dedupe_same_frame_observations(detections: list[Detection]) -> list[Detection]:
    """Merge duplicate OCR observations caused by overlapping clip windows."""
    by_frame: dict[int, list[Detection]] = {}
    for det in detections:
        by_frame.setdefault(det.frame_index, []).append(det)

    kept: list[Detection] = []
    for frame_index in sorted(by_frame):
        groups: list[list[Detection]] = []
        for det in sorted(by_frame[frame_index], key=lambda d: -_conf_score(d.confidence)):
            if not det.bbox_xyxy:
                groups.append([det])
                continue
            matched: list[Detection] | None = None
            for group in groups:
                anchor = group[0]
                if not anchor.bbox_xyxy:
                    continue
                center_distance = _bbox_center_distance(det.bbox_xyxy, anchor.bbox_xyxy)
                allowed_distance = max(8.0, 0.75 * max(_bbox_diag(det.bbox_xyxy), _bbox_diag(anchor.bbox_xyxy)))
                if center_distance <= allowed_distance or _bbox_iou(det.bbox_xyxy, anchor.bbox_xyxy) >= 0.35:
                    matched = group
                    break
            if matched is None:
                groups.append([det])
            else:
                matched.append(det)
        for group in groups:
            best = max(group, key=lambda d: _conf_score(d.confidence))
            evidence = sorted(set(f for det in group for f in det.evidence_frames))
            if evidence != best.evidence_frames:
                best = Detection(**{**asdict(best), 'evidence_frames': evidence})
            kept.append(best)
    return sorted(kept, key=lambda d: (d.frame_index, d.room_id, -_conf_score(d.confidence)))


def _track_physical_signs(
    detections: list[Detection],
    max_gap_frames: int,
    max_center_distance_px: float,
    distance_scale: float,
    min_iou: float,
    max_depth_diff_m: float,
) -> dict[str, Any]:
    detections = _dedupe_same_frame_observations(detections)
    live_tracks: list[dict[str, Any]] = []
    skipped = 0
    for det in sorted(detections, key=lambda d: (d.frame_index, -_conf_score(d.confidence))):
        if not det.bbox_xyxy:
            skipped += 1
            continue
        best_track: dict[str, Any] | None = None
        best_score = float('inf')
        for track in live_tracks:
            last: Detection = track['last_detection']
            if det.frame_index <= last.frame_index:
                continue
            gap = det.frame_index - last.frame_index
            if gap > max_gap_frames:
                continue
            last_bbox = last.bbox_xyxy
            if not last_bbox:
                continue
            depth_penalty = 0.0
            if max_depth_diff_m > 0.0 and det.depth_m is not None and last.depth_m is not None:
                depth_diff = abs(float(det.depth_m) - float(last.depth_m))
                if depth_diff > max_depth_diff_m:
                    continue
                depth_penalty = depth_diff / max(1e-6, max_depth_diff_m)
            center_distance = _bbox_center_distance(det.bbox_xyxy, last_bbox)
            allowed_distance = max(
                float(max_center_distance_px),
                float(distance_scale) * max(_bbox_diag(det.bbox_xyxy), _bbox_diag(last_bbox)),
            )
            overlap = _bbox_iou(det.bbox_xyxy, last_bbox)
            if center_distance <= allowed_distance or overlap >= min_iou:
                score = (
                    (center_distance / max(1.0, allowed_distance))
                    + (gap / max(1, max_gap_frames))
                    + depth_penalty
                )
                if score < best_score:
                    best_score = score
                    best_track = track
        if best_track is None:
            live_tracks.append({'detections': [det], 'last_detection': det})
        else:
            best_track['detections'].append(det)
            best_track['last_detection'] = det

    tracks: list[dict[str, Any]] = []
    for index, track in enumerate(live_tracks, 1):
        dets: list[Detection] = track['detections']
        best = max(dets, key=lambda d: _conf_score(d.confidence))
        id_counts = Counter(d.room_id for d in dets)
        frames = sorted(set(f for d in dets for f in d.evidence_frames))
        tracks.append({
            'track_index': index,
            'selected_room_id': best.room_id,
            'selected_confidence': best.confidence,
            'selected_frame': best.frame_index,
            'first_frame': min(d.frame_index for d in dets),
            'last_frame': max(d.frame_index for d in dets),
            'observations': len(dets),
            'candidate_room_ids': dict(sorted(id_counts.items())),
            'selected_depth_m': best.depth_m,
            'depth_observations': sum(1 for d in dets if d.depth_m is not None),
            'evidence_frames': frames[:80],
            'evidence_frame_count': len(frames),
        })
    selected_ids = sorted(set(track['selected_room_id'] for track in tracks))
    return {
        'unique_room_ids': len(selected_ids),
        'room_ids': selected_ids,
        'tracks': sorted(tracks, key=lambda t: (t['first_frame'], t['selected_room_id'])),
        'temporal_tracks': len(tracks),
        'skipped_no_bbox': skipped,
    }


def _rank_room_id_candidates(detections: list[Detection]) -> list[dict[str, Any]]:
    by_id: dict[str, list[Detection]] = {}
    for det in detections:
        by_id.setdefault(det.room_id, []).append(det)
    ranked: list[dict[str, Any]] = []
    for room_id, dets in by_id.items():
        conf_votes = [_conf_score(det.confidence) for det in dets]
        evidence_frames = sorted(set(f for det in dets for f in det.evidence_frames))
        methods = sorted(set(det.method for det in dets))
        first_frame = min(det.frame_index for det in dets)
        last_frame = max(det.frame_index for det in dets)
        score = float(sum(conf_votes) + 0.20 * len(dets) + 0.03 * len(evidence_frames))
        ranked.append({
            'room_id': room_id,
            'score': round(score, 3),
            'observations': len(dets),
            'methods': methods,
            'first_frame': first_frame,
            'last_frame': last_frame,
            'evidence_frames': evidence_frames[:40],
            'evidence_frame_count': len(evidence_frames),
            'best_confidence': dets[int(np.argmax(conf_votes))].confidence if conf_votes else None,
        })
    return sorted(ranked, key=lambda item: (-float(item['score']), item['first_frame'], item['room_id']))


def _save_contact_sheet(
    frames: list[FrameItem],
    ocr_dets: list[Detection],
    vlm_dets: list[Detection],
    out_path: Path,
    cell_w: int = 320,
) -> None:
    if not frames:
        return
    imgs = [Image.fromarray(f.vlm_rgb).convert('RGB') for f in frames]
    thumbs = []
    for img in imgs:
        scale = cell_w / float(img.width)
        thumbs.append(img.resize((cell_w, max(1, int(round(img.height * scale)))), Image.Resampling.LANCZOS))
    cols = 5
    rows = int(math.ceil(len(thumbs) / cols))
    label_h = 24
    cell_h = max(t.height for t in thumbs) + label_h
    canvas = Image.new('RGB', (cols * cell_w, rows * cell_h), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    font = _font(13)
    frame_to_ids: dict[int, list[str]] = {}
    for det in ocr_dets + vlm_dets:
        frame_to_ids.setdefault(det.frame_index, []).append(f'{det.method}:{det.room_id}')
    for i, (frame, thumb) in enumerate(zip(frames, thumbs)):
        x = (i % cols) * cell_w
        y = (i // cols) * cell_h
        canvas.paste(thumb, (x, y + label_h))
        label = f'{i + 1}: f={frame.frame_index}'
        ids = ', '.join(sorted(set(frame_to_ids.get(frame.frame_index, [])))[:3])
        if ids:
            label += f' {ids}'
        draw.rectangle((x, y, x + cell_w - 1, y + label_h - 1), fill=(245, 247, 250), outline=(209, 213, 219))
        draw.text((x + 5, y + 5), label, fill=(17, 24, 39), font=font)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)


def _summarize_video(
    video: Path,
    gt_total: int | None,
    detections: dict[str, list[Detection]],
    max_gap_frames: int,
    ocr_physical_tracking: bool,
    ocr_track_max_gap_frames: int,
    ocr_track_max_center_distance_px: float,
    ocr_track_distance_scale: float,
    ocr_track_min_iou: float,
    ocr_track_max_depth_diff_m: float,
) -> dict[str, Any]:
    methods: dict[str, Any] = {}
    for method in ('ocr', 'vlm'):
        dets = detections.get(method, [])
        ids = sorted(set(d.room_id for d in dets))
        tracks = _track_instances(dets, max_gap_frames=max_gap_frames)
        methods[method] = {
            'raw_detections': len(dets),
            'unique_room_ids': len(ids),
            'room_ids': ids,
            'ranked_candidates': _rank_room_id_candidates(dets),
            'temporal_tracks': len(tracks),
            'tracks': tracks,
        }
        if method == 'ocr' and ocr_physical_tracking:
            methods[method]['physical_track_best'] = _track_physical_signs(
                dets,
                max_gap_frames=ocr_track_max_gap_frames,
                max_center_distance_px=ocr_track_max_center_distance_px,
                distance_scale=ocr_track_distance_scale,
                min_iou=ocr_track_min_iou,
                max_depth_diff_m=ocr_track_max_depth_diff_m,
            )
    return {
        'video': str(video),
        'video_stem': video.stem,
        'ground_truth_sign_total': gt_total,
        'ocr': methods['ocr'],
        'vlm': methods['vlm'],
    }


def _write_outputs(
    out_dir: Path,
    config: dict[str, Any],
    video_summaries: list[dict[str, Any]],
    all_detections: list[Detection],
    errors: list[dict[str, Any]],
    metrics: dict[str, Any],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        'config': config,
        'videos': video_summaries,
        'detections': [asdict(d) for d in all_detections],
        'errors': errors,
        'metrics': metrics,
    }
    (out_dir / 'clip_text_recognition_results.json').write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding='utf-8',
    )

    ocr_backend = str(config.get('ocr_backend') or '-')
    ocr_version = config.get('ocr_backend_version')
    version_suffix = f' {ocr_version}' if ocr_version else ''
    if ocr_backend == 'paddle':
        ocr_model = f'PaddleOCR{version_suffix}(lang=en, use_angle_cls=True)'
        ocr_reason = (
            'Chosen as the main OCR because it returns text, confidence, and quadrilateral boxes in one pass, '
            'supports angle classification for tilted/rotated hallway text, and works with simple multi-scale RGB frames.'
        )
    elif ocr_backend == 'tesseract':
        ocr_model = 'Tesseract OCR(lang=eng, psm=11, alphanumeric whitelist)'
        ocr_reason = 'Kept as a lightweight fallback OCR path when PaddleOCR is unavailable.'
    else:
        ocr_model = ocr_backend
        ocr_reason = '-'
    frames_seen = float(metrics.get('rgb_frames_seen') or 0.0)
    elapsed_s = float(metrics.get('elapsed_s') or 0.0)
    ocr_inference_s = float(metrics.get('ocr_inference_s') or 0.0)
    total_fps = frames_seen / elapsed_s if elapsed_s > 0.0 else 0.0
    ocr_inference_fps = frames_seen / ocr_inference_s if ocr_inference_s > 0.0 else 0.0
    gpu_mem = ((metrics.get('gpu') or {}).get('memory_used_mb') or {})
    gpu_peak_mb = gpu_mem.get('max')
    gpu_mean_mb = gpu_mem.get('mean')
    gpu_note = 'nvidia-smi observed GPU memory during the run; OCR was run without VLM in OCR-only runs.'

    lines = [
        '# Multi-Frame Text Recognition Benchmark',
        '',
        'OCR engine:',
        f'- Model/backend: {ocr_model}',
        f'- Why this OCR: {ocr_reason}',
        f'- OCR confidence filter: discard confidence <= {config.get("ocr_min_confidence", 0.0)}',
        f'- OCR scales: {", ".join(str(s) for s in config.get("ocr_scales", [])) or "-"}',
        f'- Floor prior mode: {config.get("floor_prior_mode", "-")}',
        '',
        f'Clip size: {config["clip_size"]} frames',
        f'Clip stride: {config["clip_stride"]} frames',
        f'Inference frame step within clip: {config["clip_frame_step"]}',
        f'OCR max side: {config["ocr_max_side"]}',
        f'VLM frame max side: {config["vlm_max_side"]}',
        '',
        '| Video | GT signs | OCR unique IDs | OCR tracks | OCR track-best IDs | OCR physical tracks | VLM unique IDs | VLM tracks |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for item in video_summaries:
        gt = item.get('ground_truth_sign_total')
        ocr_physical = item['ocr'].get('physical_track_best') or {}
        lines.append(
            f'| {item["video_stem"]} | {gt if gt is not None else "-"} | '
            f'{item["ocr"]["unique_room_ids"]} | {item["ocr"]["temporal_tracks"]} | '
            f'{ocr_physical.get("unique_room_ids", "-")} | {ocr_physical.get("temporal_tracks", "-")} | '
            f'{item["vlm"]["unique_room_ids"]} | {item["vlm"]["temporal_tracks"]} |'
        )
    lines.extend(['', 'Detected Room IDs:'])
    for item in video_summaries:
        lines.append(f'- `{item["video_stem"]}`')
        lines.append(f'  - OCR: {", ".join(item["ocr"]["room_ids"]) or "-"}')
        ocr_physical = item['ocr'].get('physical_track_best') or {}
        if ocr_physical:
            lines.append(f'  - OCR track-best: {", ".join(ocr_physical.get("room_ids") or []) or "-"}')
        lines.append(f'  - VLM: {", ".join(item["vlm"]["room_ids"]) or "-"}')
        ocr_top = item['ocr'].get('ranked_candidates') or []
        if ocr_top:
            ranked_text = ', '.join(
                f'{cand["room_id"]}({cand["observations"]}x,{cand["score"]})'
                for cand in ocr_top[:16]
            )
            lines.append(f'  - OCR ranked candidates: {ranked_text}')
        vlm_top = item['vlm'].get('ranked_candidates') or []
        if vlm_top:
            ranked_text = ', '.join(
                f'{cand["room_id"]}({cand["observations"]}x,{cand["score"]})'
                for cand in vlm_top[:16]
            )
            lines.append(f'  - VLM ranked candidates: {ranked_text}')
    lines.extend([
        '',
        'Timing:',
        f'- OCR setup: {metrics.get("ocr_setup_s", 0.0):.2f}s',
        f'- OCR inference: {metrics.get("ocr_inference_s", 0.0):.2f}s',
        f'- OCR input frames processed: {int(frames_seen)}',
        f'- Effective FPS including I/O/output: {total_fps:.3f}',
        f'- OCR inference FPS: {ocr_inference_fps:.3f}',
        f'- VLM setup: {metrics.get("vlm_setup_s", 0.0):.2f}s',
        f'- VLM inference: {metrics.get("vlm_inference_s", 0.0):.2f}s',
        f'- Total elapsed: {metrics.get("elapsed_s", 0.0):.2f}s',
        f'- Peak GPU memory: {gpu_peak_mb if gpu_peak_mb is not None else "-"} MB',
        f'- Mean GPU memory: {gpu_mean_mb if gpu_mean_mb is not None else "-"} MB',
        f'- GPU memory note: {gpu_note}',
        '',
        'Files:',
        '- `clip_text_recognition_results.json`',
        '- `summary.md`',
    ])
    if errors:
        lines.extend(['', f'Errors/warnings: {len(errors)} entries. See JSON for details.'])
    (out_dir / 'summary.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('videos', nargs='+', type=Path)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--clip-size', type=int, default=20)
    parser.add_argument('--clip-stride', type=int, default=20)
    parser.add_argument(
        '--clip-frame-step',
        type=int,
        default=5,
        help='Use every Nth frame inside each clip for OCR/VLM inference. With clip-size=20 and step=5, uses frames 0,5,10,15 plus the final frame.',
    )
    parser.add_argument(
        '--start-clip',
        type=int,
        default=1,
        help='1-based clip index to start from within each video.',
    )
    parser.add_argument(
        '--clip-count',
        type=int,
        default=0,
        help='Maximum number of clips to process from --start-clip. 0 means no limit.',
    )
    parser.add_argument('--max-clips-per-video', type=int, default=0)
    parser.add_argument('--ocr-max-side', type=int, default=1280)
    parser.add_argument('--vlm-max-side', type=int, default=768)
    parser.add_argument('--rotate-frames', choices=('auto', 'none', '90ccw', '90cw', '180'), default='auto')
    parser.add_argument('--floor-hints', default='')
    parser.add_argument(
        '--floor-prior-mode',
        choices=('reject', 'complete'),
        default='reject',
        help='reject only accepts visible IDs compatible with the floor; complete also repairs common partial OCR reads such as 303->1303 on 13F.',
    )
    parser.add_argument('--rotation-hints', default='')
    parser.add_argument('--ground-truth', default='')
    parser.add_argument('--ocr-scales', default='1.0,2.0')
    parser.add_argument(
        '--ocr-min-confidence',
        type=float,
        default=0.6,
        help='Discard OCR detections with confidence <= this threshold. Set 0 to disable.',
    )
    parser.add_argument(
        '--ocr-backend',
        choices=('paddle', 'tesseract'),
        default='paddle',
        help='OCR backend. tesseract avoids PaddleOCR native runtime crashes.',
    )
    parser.add_argument('--ocr-gpu', action='store_true')
    parser.add_argument('--skip-ocr', action='store_true')
    parser.add_argument('--skip-vlm', action='store_true')
    parser.add_argument('--model-name', default='Qwen/Qwen2.5-VL-3B-Instruct')
    parser.add_argument('--device', default='auto')
    parser.add_argument('--dtype', default='auto')
    parser.add_argument('--allow-network', action='store_true')
    parser.add_argument('--max-new-tokens', type=int, default=768)
    parser.add_argument('--track-gap-frames', type=int, default=150)
    parser.add_argument('--progress-every', type=int, default=10)
    parser.add_argument('--ocr-physical-tracking', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--ocr-track-max-gap-frames', type=int, default=30)
    parser.add_argument('--ocr-track-max-center-distance-px', type=float, default=90.0)
    parser.add_argument('--ocr-track-distance-scale', type=float, default=4.0)
    parser.add_argument('--ocr-track-min-iou', type=float, default=0.05)
    parser.add_argument(
        '--ocr-track-max-depth-diff-m',
        type=float,
        default=0.0,
        help='Future RGBD hook: if Detection.depth_m is populated, reject same-sign associations with depth difference above this value. 0 disables depth gating.',
    )
    parser.add_argument('--verify-crops', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--verify-crop-pad-ratio', type=float, default=0.75)
    parser.add_argument('--verify-crop-max-side', type=int, default=640)
    parser.add_argument('--verify-max-new-tokens', type=int, default=128)
    parser.add_argument('--save-detected-contact-sheets', action='store_true')
    args = parser.parse_args()
    _debug_marker('parsed args')

    args.out_dir.mkdir(parents=True, exist_ok=True)
    _debug_marker('created out dir')
    floor_hints = _parse_floor_hints(args.floor_hints)
    rotation_hints = _parse_rotation_hints(args.rotation_hints)
    ground_truth = _parse_ground_truth(args.ground_truth)
    scales = _parse_scales(args.ocr_scales)
    _debug_marker('parsed hints')

    config = {
        'clip_size': max(1, int(args.clip_size)),
        'clip_stride': max(1, int(args.clip_stride)),
        'clip_frame_step': max(1, int(args.clip_frame_step)),
        'start_clip': max(1, int(args.start_clip)),
        'clip_count': max(0, int(args.clip_count)),
        'ocr_max_side': int(args.ocr_max_side),
        'vlm_max_side': int(args.vlm_max_side),
        'rotate_frames': args.rotate_frames,
        'floor_hints': floor_hints,
        'floor_prior_mode': args.floor_prior_mode,
        'rotation_hints': rotation_hints,
        'ground_truth': ground_truth,
        'ocr_scales': scales,
        'ocr_min_confidence': float(args.ocr_min_confidence),
        'ocr_backend': args.ocr_backend,
        'ocr_backend_version': _ocr_backend_version(args.ocr_backend),
        'model_name': args.model_name,
        'track_gap_frames': int(args.track_gap_frames),
        'ocr_physical_tracking': bool(args.ocr_physical_tracking),
        'ocr_track_max_gap_frames': int(args.ocr_track_max_gap_frames),
        'ocr_track_max_center_distance_px': float(args.ocr_track_max_center_distance_px),
        'ocr_track_distance_scale': float(args.ocr_track_distance_scale),
        'ocr_track_min_iou': float(args.ocr_track_min_iou),
        'ocr_track_max_depth_diff_m': float(args.ocr_track_max_depth_diff_m),
        'verify_crops': bool(args.verify_crops),
        'verify_crop_pad_ratio': float(args.verify_crop_pad_ratio),
        'verify_crop_max_side': int(args.verify_crop_max_side),
    }

    start_t = time.perf_counter()
    ocr_setup_s = 0.0
    ocr_inference_s = 0.0
    vlm_setup_s = 0.0
    vlm_inference_s = 0.0
    errors: list[dict[str, Any]] = []

    paddle = None
    if not args.skip_ocr and args.ocr_backend == 'paddle':
        _debug_marker('loading paddle')
        t = time.perf_counter()
        paddle, err = _init_paddle(args.ocr_gpu)
        ocr_setup_s = time.perf_counter() - t
        if err:
            errors.append({'stage': 'ocr_setup', 'error': err})

    torch = processor = model = None
    device = args.device
    monitor = GpuMonitor()
    monitor.start()
    _debug_marker('started gpu monitor')
    if not args.skip_vlm:
        try:
            t = time.perf_counter()
            _debug_marker('loading vlm')
            torch, processor, model, device = _load_vlm(
                args.model_name,
                args.device,
                args.dtype,
                local_files_only=not args.allow_network,
            )
            vlm_setup_s = time.perf_counter() - t
            _debug_marker('loaded vlm')
            if torch.cuda.is_available():
                torch.cuda.reset_peak_memory_stats()
        except Exception as exc:
            errors.append({'stage': 'vlm_setup', 'error': str(exc)})
            args.skip_vlm = True

    all_detections: list[Detection] = []
    video_summaries: list[dict[str, Any]] = []
    total_clips = 0
    total_frames = 0

    for video in args.videos:
        _debug_marker(f'opening video {video}')
        if not video.exists():
            errors.append({'stage': 'open_video', 'video': str(video), 'error': 'file not found'})
            continue
        cap = cv2.VideoCapture(str(video))
        if not cap.isOpened():
            errors.append({'stage': 'open_video', 'video': str(video), 'error': 'cv2 failed to open video'})
            continue
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        floor_hint = _floor_hint_for_video(video, floor_hints)
        rotation = _rotation_for_video(video, args.rotate_frames, rotation_hints)
        gt_total = _hint_for_stem(video.stem, ground_truth)
        all_starts = list(range(0, frame_count, config['clip_stride']))
        start_offset = max(0, config['start_clip'] - 1)
        starts = all_starts[start_offset:]
        if config['clip_count'] > 0:
            starts = starts[:config['clip_count']]
        if args.max_clips_per_video > 0:
            starts = starts[:args.max_clips_per_video]
        per_video: dict[str, list[Detection]] = {'ocr': [], 'vlm': []}
        print(
            f'[clip] {video.name}: frames={frame_count} fps={fps:.3f} clips={len(starts)} '
            f'floor={floor_hint} rotation={rotation}',
            flush=True,
        )
        for clip_i, start in enumerate(starts, config['start_clip']):
            _debug_marker(f'reading clip {clip_i}')
            frames = _read_clip(
                cap,
                start,
                config['clip_size'],
                fps,
                rotation,
                config['ocr_max_side'],
                config['vlm_max_side'],
            )
            if not frames:
                continue
            inference_frames = _sample_clip_frames(frames, config['clip_frame_step'])
            total_clips += 1
            total_frames += len(inference_frames)
            clip_dets: list[Detection] = []
            if not args.skip_ocr:
                t = time.perf_counter()
                ocr_dets, ocr_errors = _run_ocr_clip(
                    paddle,
                    inference_frames,
                    video,
                    clip_i,
                    start,
                    floor_hint,
                    scales,
                    args.floor_prior_mode,
                    max(0.0, float(args.ocr_min_confidence)),
                )
                ocr_inference_s += time.perf_counter() - t
                ocr_dets = _dedupe_clip(ocr_dets)
                per_video['ocr'].extend(ocr_dets)
                clip_dets.extend(ocr_dets)
                if ocr_errors:
                    errors.append({
                        'stage': 'ocr_clip',
                        'video': str(video),
                        'clip_index': clip_i,
                        'errors': ocr_errors[:3],
                        'error_count': len(ocr_errors),
                    })
            if not args.skip_vlm and torch is not None and processor is not None and model is not None:
                _debug_marker(f'running vlm clip {clip_i}')
                t = time.perf_counter()
                vlm_dets, err, raw = _run_vlm_clip(
                    torch,
                    processor,
                    model,
                    device,
                    inference_frames,
                    video,
                    clip_i,
                    start,
                    floor_hint,
                    args.max_new_tokens,
                )
                vlm_inference_s += time.perf_counter() - t
                vlm_dets = _dedupe_clip(vlm_dets)
                if args.verify_crops:
                    t_verify = time.perf_counter()
                    vlm_dets, verify_errors = _verify_clip_detections(
                        torch,
                        processor,
                        model,
                        device,
                        vlm_dets,
                        inference_frames,
                        floor_hint,
                        args.verify_max_new_tokens,
                        max(0.0, float(args.verify_crop_pad_ratio)),
                        max(64, int(args.verify_crop_max_side)),
                    )
                    vlm_inference_s += time.perf_counter() - t_verify
                    for verify_error in verify_errors:
                        errors.append({
                            'video': str(video),
                            'clip_index': clip_i,
                            **verify_error,
                        })
                _debug_marker(f'finished vlm clip {clip_i}')
                per_video['vlm'].extend(vlm_dets)
                clip_dets.extend(vlm_dets)
                if err:
                    errors.append({
                        'stage': 'vlm_clip',
                        'video': str(video),
                        'clip_index': clip_i,
                        'error': err,
                        'raw': raw[:1000],
                    })
            all_detections.extend(clip_dets)
            if args.save_detected_contact_sheets and clip_dets:
                out = args.out_dir / 'contact_sheets' / f'{video.stem}_clip_{clip_i:04d}_{start:06d}.jpg'
                _save_contact_sheet(
                    frames,
                    [d for d in clip_dets if d.method == 'ocr'],
                    [d for d in clip_dets if d.method == 'vlm'],
                    out,
                )
            progress_every = max(1, int(args.progress_every))
            if clip_i == 1 or clip_i % progress_every == 0 or clip_i == len(starts):
                ocr_ids = len(set(d.room_id for d in per_video['ocr']))
                vlm_ids = len(set(d.room_id for d in per_video['vlm']))
                print(
                    f'[clip] {video.name} {clip_i}/{len(starts)} '
                    f'ocr_ids={ocr_ids} vlm_ids={vlm_ids}',
                    flush=True,
                )
        cap.release()
        video_summaries.append(_summarize_video(
            video,
            gt_total,
            per_video,
            max_gap_frames=max(1, int(args.track_gap_frames)),
            ocr_physical_tracking=bool(args.ocr_physical_tracking),
            ocr_track_max_gap_frames=max(1, int(args.ocr_track_max_gap_frames)),
            ocr_track_max_center_distance_px=max(1.0, float(args.ocr_track_max_center_distance_px)),
            ocr_track_distance_scale=max(0.1, float(args.ocr_track_distance_scale)),
            ocr_track_min_iou=max(0.0, float(args.ocr_track_min_iou)),
            ocr_track_max_depth_diff_m=max(0.0, float(args.ocr_track_max_depth_diff_m)),
        ))

    gpu = monitor.stop()
    torch_peak_mb = None
    if torch is not None:
        try:
            if torch.cuda.is_available():
                torch_peak_mb = float(torch.cuda.max_memory_allocated() / (1024.0 * 1024.0))
        except Exception:
            pass
    elapsed_s = time.perf_counter() - start_t
    metrics = {
        'videos': len(video_summaries),
        'clips': total_clips,
        'rgb_frames_seen': total_frames,
        'ocr_setup_s': ocr_setup_s,
        'ocr_inference_s': ocr_inference_s,
        'vlm_setup_s': vlm_setup_s,
        'vlm_inference_s': vlm_inference_s,
        'elapsed_s': elapsed_s,
        'gpu': gpu,
        'torch_peak_allocated_mb': torch_peak_mb,
    }
    _write_outputs(args.out_dir, config, video_summaries, all_detections, errors, metrics)
    print(args.out_dir / 'summary.md', flush=True)
    print(args.out_dir / 'clip_text_recognition_results.json', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
