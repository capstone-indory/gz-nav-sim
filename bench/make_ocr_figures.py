#!/usr/bin/env python3
"""Render OCR-only result figures from clip_sign_vlm_benchmark JSON."""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


EXPECTED_COUNTS = {
    'VID_20260429_221116_981': 19,
    'VID_20260429_222101_976': 17,
    'VID_20260429_221947_605': 12,
    '310_b3': 4,
}


def _font(size: int) -> ImageFont.ImageFont:
    for path in (
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    ):
        try:
            return ImageFont.truetype(path, size=size)
        except Exception:
            continue
    return ImageFont.load_default()


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
    return rgb


def _resize_rgb(rgb: np.ndarray, max_side: int) -> np.ndarray:
    h, w = rgb.shape[:2]
    scale = min(1.0, float(max_side) / float(max(h, w)))
    if scale >= 1.0:
        return rgb
    return cv2.resize(rgb, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_AREA)


def _rotation_for(stem: str, config: dict[str, Any]) -> str:
    hints = config.get('rotation_hints') or {}
    if stem in hints:
        return str(hints[stem])
    if '310_b3' in stem:
        return 'none'
    return str(config.get('rotate_frames') or 'auto')


def _read_frame(video: Path, stem: str, frame_index: int, config: dict[str, Any]) -> np.ndarray | None:
    cap = cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(frame_index))
    ok, bgr = cap.read()
    cap.release()
    if not ok:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = _rotate_rgb(rgb, _rotation_for(stem, config))
    return _resize_rgb(rgb, int(config.get('ocr_max_side') or 1280))


def _draw_detections(rgb: np.ndarray, detections: list[dict[str, Any]]) -> Image.Image:
    image = Image.fromarray(rgb).convert('RGB')
    draw = ImageDraw.Draw(image)
    label_font = _font(13)
    for det in sorted(detections, key=lambda item: (str(item.get('room_id')), -float(item.get('confidence') or 0.0))):
        bbox = det.get('bbox_xyxy')
        if not bbox:
            continue
        x1, y1, x2, y2 = [int(v) for v in bbox]
        conf = float(det.get('confidence') or 0.0)
        label = f'{det.get("room_id")} {conf:.2f}'
        color = (255, 64, 32) if conf >= 0.9 else (255, 180, 0)
        draw.rectangle((x1, y1, x2, y2), outline=color, width=3)
        text_w = int(draw.textlength(label, font=label_font))
        y_label = max(0, y1 - 20)
        draw.rectangle((x1, y_label, x1 + text_w + 8, y_label + 19), fill=color)
        draw.text((x1 + 4, y_label + 2), label, fill=(0, 0, 0), font=label_font)
    return image


def _collect_ocr(data: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    by_video: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for det in data.get('detections') or []:
        if det.get('method') == 'ocr':
            by_video[str(det['video_stem'])].append(det)
    return by_video


def _video_summaries(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for item in data.get('videos') or []:
        out[str(item.get('video_stem'))] = item
    return out


def _write_count_summary(
    out_dir: Path,
    data: dict[str, Any],
    by_video: dict[str, list[dict[str, Any]]],
) -> None:
    summaries = _video_summaries(data)
    rows = []
    stems = sorted(set(EXPECTED_COUNTS) | set(by_video))
    for stem in stems:
        raw_ids = sorted({d['room_id'] for d in by_video.get(stem, [])})
        physical = ((summaries.get(stem) or {}).get('ocr') or {}).get('physical_track_best') or {}
        track_ids = physical.get('room_ids') or []
        rows.append((stem, EXPECTED_COUNTS.get(stem), len(raw_ids), len(track_ids)))

    font_title = _font(20)
    font_small = _font(13)
    row_h = 48
    width = 1120
    height = 86 + row_h * len(rows)
    canvas = Image.new('RGB', (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 18), 'OCR IDs vs GT (raw OCR and RGB-track-best, no VLM union)', fill=(17, 24, 39), font=font_title)
    max_count = max([1] + [max(v for v in row[1:] if v is not None) for row in rows])
    bar_x = 420
    bar_w = 560
    for i, (stem, gt, raw_count, track_count) in enumerate(rows):
        y = 66 + i * row_h
        draw.text((20, y + 13), stem, fill=(17, 24, 39), font=font_small)
        draw.text((278, y + 13), f'GT {gt or "-"} / raw {raw_count} / track {track_count}', fill=(17, 24, 39), font=font_small)
        for j, (value, color) in enumerate(((gt or 0, (59, 130, 246)), (raw_count, (239, 68, 68)), (track_count, (16, 185, 129)))):
            x0 = bar_x
            y0 = y + j * 14
            draw.rectangle((x0, y0, x0 + bar_w, y0 + 10), outline=(209, 213, 219), fill=(245, 247, 250))
            draw.rectangle((x0, y0, x0 + int(bar_w * value / max_count), y0 + 10), fill=color)
    canvas.save(out_dir / 'ocr_count_summary.png')


def _best_by_room_id(detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
    best: dict[str, dict[str, Any]] = {}
    for det in detections:
        old = best.get(str(det.get('room_id')))
        if old is None or float(det.get('confidence') or 0.0) > float(old.get('confidence') or 0.0):
            best[str(det.get('room_id'))] = det
    return sorted(best.values(), key=lambda d: (int(d.get('frame_index') or 0), str(d.get('room_id'))))


def _render_crop_gallery(
    out_path: Path,
    config: dict[str, Any],
    detections: list[dict[str, Any]],
    title_prefix: str,
) -> None:
    if not detections:
        return
    font_small = _font(13)
    cells: list[Image.Image] = []
    for det in detections:
        bbox = det.get('bbox_xyxy')
        if not bbox:
            continue
        stem = str(det['video_stem'])
        rgb = _read_frame(Path(det['video']), stem, int(det['frame_index']), config)
        if rgb is None:
            continue
        h, w = rgb.shape[:2]
        x1, y1, x2, y2 = [int(v) for v in bbox]
        pad = max(18, int(max(x2 - x1, y2 - y1) * 1.5))
        crop_box = [max(0, x1 - pad), max(0, y1 - pad), min(w - 1, x2 + pad), min(h - 1, y2 + pad)]
        cx1, cy1, cx2, cy2 = crop_box
        crop_rgb = rgb[cy1:cy2 + 1, cx1:cx2 + 1]
        adjusted = {**det, 'bbox_xyxy': [x1 - cx1, y1 - cy1, x2 - cx1, y2 - cy1]}
        crop = _draw_detections(crop_rgb, [adjusted])
        cell_w = 230
        image_h = 150
        scale = min(cell_w / crop.width, image_h / crop.height)
        resized = crop.resize((max(1, int(crop.width * scale)), max(1, int(crop.height * scale))), Image.Resampling.LANCZOS)
        cell = Image.new('RGB', (cell_w, image_h + 52), (255, 255, 255))
        cell.paste(resized, ((cell_w - resized.width) // 2, 4))
        draw = ImageDraw.Draw(cell)
        draw.text((6, image_h + 7), f'{title_prefix}{det.get("room_id")} conf={float(det.get("confidence") or 0.0):.2f}', fill=(17, 24, 39), font=font_small)
        draw.text((6, image_h + 27), f'f={det.get("frame_index")}', fill=(75, 85, 99), font=font_small)
        draw.rectangle((0, 0, cell_w - 1, image_h + 51), outline=(209, 213, 219))
        cells.append(cell)
    if not cells:
        return
    cols = 5
    rows = math.ceil(len(cells) / cols)
    cw, ch = cells[0].size
    gallery = Image.new('RGB', (cols * cw, rows * ch), (255, 255, 255))
    for i, cell in enumerate(cells):
        gallery.paste(cell, ((i % cols) * cw, (i // cols) * ch))
    gallery.save(out_path, quality=92)


def _render_montages(out_dir: Path, config: dict[str, Any], stem: str, detections: list[dict[str, Any]]) -> None:
    frame_map: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for det in detections:
        frame_map[int(det['frame_index'])].append(det)
    frames = sorted(frame_map)
    if not frames:
        return
    font_small = _font(13)
    per_page = 30
    cols = 5
    cell_w = 360
    label_h = 28
    video = Path(detections[0]['video'])
    for page, start in enumerate(range(0, len(frames), per_page), 1):
        thumbs = []
        for frame_index in frames[start:start + per_page]:
            rgb = _read_frame(video, stem, frame_index, config)
            if rgb is None:
                continue
            annotated = _draw_detections(rgb, frame_map[frame_index])
            scale = cell_w / annotated.width
            thumb = annotated.resize((cell_w, max(1, int(round(annotated.height * scale)))), Image.Resampling.LANCZOS)
            thumbs.append((frame_index, thumb, frame_map[frame_index]))
        if not thumbs:
            continue
        rows = math.ceil(len(thumbs) / cols)
        cell_h = max(t.height for _, t, _ in thumbs) + label_h
        montage = Image.new('RGB', (cols * cell_w, rows * cell_h), (255, 255, 255))
        draw = ImageDraw.Draw(montage)
        for i, (frame_index, thumb, frame_dets) in enumerate(thumbs):
            x = (i % cols) * cell_w
            y = (i // cols) * cell_h
            montage.paste(thumb, (x, y + label_h))
            ids = ', '.join(sorted({str(d['room_id']) for d in frame_dets}))[:40]
            draw.rectangle((x, y, x + cell_w - 1, y + label_h - 1), fill=(245, 247, 250), outline=(209, 213, 219))
            draw.text((x + 6, y + 6), f'f={frame_index}  {ids}', fill=(17, 24, 39), font=font_small)
        montage.save(out_dir / f'ocr_detected_montage_{stem}_part{page:02d}.jpg', quality=92)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('result_json', type=Path)
    parser.add_argument('--out-dir', type=Path)
    args = parser.parse_args()

    data = json.loads(args.result_json.read_text(encoding='utf-8'))
    out_dir = args.out_dir or (args.result_json.parent / 'figures')
    out_dir.mkdir(parents=True, exist_ok=True)
    config = data.get('config') or {}
    by_video = _collect_ocr(data)
    summaries = _video_summaries(data)

    _write_count_summary(out_dir, data, by_video)
    for stem, detections in sorted(by_video.items()):
        _render_montages(out_dir, config, stem, detections)
        _render_crop_gallery(
            out_dir / f'ocr_raw_candidate_gallery_{stem}.jpg',
            config,
            _best_by_room_id(detections),
            '',
        )
        physical = ((summaries.get(stem) or {}).get('ocr') or {}).get('physical_track_best') or {}
        track_dets = []
        for track in physical.get('tracks') or []:
            selected_frame = track.get('selected_frame')
            selected_id = track.get('selected_room_id')
            matches = [
                det for det in detections
                if det.get('room_id') == selected_id and det.get('frame_index') == selected_frame
            ]
            if matches:
                track_dets.append(max(matches, key=lambda d: float(d.get('confidence') or 0.0)))
        _render_crop_gallery(
            out_dir / f'ocr_track_best_gallery_{stem}.jpg',
            config,
            track_dets,
            'T ',
        )

    print(out_dir)
    for path in sorted(out_dir.iterdir()):
        print(path)


if __name__ == '__main__':
    main()
