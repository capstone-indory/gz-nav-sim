#!/usr/bin/env python3
"""Semantic VLM inspection from the robot RGB camera stream.

The node samples one RGB frame every N camera frames, calls one VLM, validates
the strict JSON response, and publishes candidate/confirmed semantic map
annotations. It intentionally does not use a separate OCR engine.
"""

from __future__ import annotations

import copy
import json
import math
import re
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Optional

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


TEXT_OBJECT_SYSTEM_PROMPT = """You are a robot visual inspection module.

Input: one camera frame.

Task:
Detect visible signs, warning labels, door plates, package labels, delivery labels, or other text-bearing objects.
For each object, return its approximate pixel location and only the text that is clearly readable.

Rules:
- Return only valid JSON.
- Do not explain outside JSON.
- Do not infer missing text.
- Do not complete partially visible words.
- Do not guess based on context.
- If text is blurry, too small, occluded, angled, overexposed, or unreadable, set text=null.
- If bbox is uncertain, set bbox_xyxy=null.
- Prefer false negatives over hallucinated text.
- Use original image pixel coordinates.
- The Korean control summary must mention only detected evidence.
- If confidence is low or text is null, say that human confirmation is needed.

Output schema:
{
  "has_text_object": boolean,
  "objects": [
    {
      "type": "sign|package_label|warning_label|doorplate|delivery_label|other",
      "bbox_xyxy": [x1, y1, x2, y2] | null,
      "text": "clearly readable text only" | null,
      "confidence": "low|medium|high",
      "failure_reason": "none|blur|small_text|occlusion|glare|low_resolution|motion|angle|unknown"
    }
  ],
  "control_summary_ko": "관제에 보낼 한국어 한 문장",
  "need_human_check": boolean
}"""

TEXT_OBJECT_USER_PROMPT = "Return only the JSON object for this frame."

SCENE_DESCRIPTION_SYSTEM_PROMPT = """You are a robot scene description module.

Input: one robot camera frame.

Task:
Describe only what is visibly present in this frame for remote monitoring.
For the main visible objects you mention, also return where they are in image pixel coordinates.

Rules:
- Return only valid JSON.
- Do not explain outside JSON.
- Do not guess unseen areas.
- Do not infer hidden objects.
- If visibility is poor, state that clearly.
- Prefer concise factual descriptions.
- Use pixel coordinates from the input image.
- If an object's image location is unclear, set bbox_xyxy=null.
- The Korean control summary must mention only visible evidence.

Output schema:
{
  "scene_description_ko": "현재 프레임에 대한 짧고 사실적인 한국어 설명",
  "objects": [
    {
      "label": "visible object name",
      "bbox_xyxy": [x1, y1, x2, y2] | null,
      "visible_evidence_ko": "이 물체가 왜 그렇게 보이는지에 대한 짧은 근거",
      "confidence": "low|medium|high"
    }
  ],
  "control_summary_ko": "관제에 보낼 한국어 한 문장",
  "need_human_check": boolean
}"""

SCENE_DESCRIPTION_USER_PROMPT = "Return only the JSON object describing this frame."

OBJECT_TYPES = {
    'sign',
    'package_label',
    'warning_label',
    'doorplate',
    'delivery_label',
    'other',
}
CONFIDENCES = {'low', 'medium', 'high'}
FAILURE_REASONS = {
    'none',
    'blur',
    'small_text',
    'occlusion',
    'glare',
    'low_resolution',
    'motion',
    'angle',
    'unknown',
}


@dataclass
class VlmJob:
    rgb: np.ndarray
    stamp: object
    frame_id: str
    depth_msg: Optional[Image]
    camera_info: Optional[CameraInfo]
    seq: int


@dataclass
class SemanticCandidate:
    id: str
    object_type: str
    text: Optional[str]
    bbox_xyxy: Optional[list[int]]
    confidence: str
    failure_reason: str
    frame_id: str
    world_xyz: Optional[tuple[float, float, float]]
    first_seen_sec: float
    last_seen_sec: float
    observations: int = 1
    confirmed: bool = False
    need_human_check: bool = False
    control_summary_ko: str = ''


class SemanticVlmNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_vlm_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/d456/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d456/depth/camera_info')
        self.declare_parameter('detections_topic', '/semantic_vlm/detections')
        self.declare_parameter('markers_topic', '/semantic_vlm/markers')
        self.declare_parameter('image_annotations_topic', '/semantic_vlm/image_annotations')
        self.declare_parameter('task_mode', 'scene_description')
        self.declare_parameter('model_name', 'Qwen/Qwen2.5-VL-3B-Instruct')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('torch_dtype', 'auto')
        self.declare_parameter('frame_interval', 20)
        self.declare_parameter('max_new_tokens', 256)
        self.declare_parameter('crop_requery', False)
        self.declare_parameter('vram_budget_mb', 12288.0)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('fallback_target_frame', 'odom')
        self.declare_parameter('max_depth_m', 12.0)
        self.declare_parameter('depth_window_px', 7)
        self.declare_parameter('candidate_ttl_s', 180.0)
        self.declare_parameter('confirm_min_observations', 3)
        self.declare_parameter('confirm_window_s', 120.0)
        self.declare_parameter('match_radius_m', 1.0)
        self.declare_parameter('publish_raw_vlm_output', False)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_depth: Optional[Image] = None
        self._latest_info: Optional[CameraInfo] = None
        self._frame_seq = 0
        self._model = None
        self._processor = None
        self._torch = None
        self._device = 'cpu'
        self._load_lock = threading.Lock()
        self._job_cv = threading.Condition()
        self._pending_job: Optional[VlmJob] = None
        self._worker_busy = False
        self._stop_worker = False
        self._candidates: list[SemanticCandidate] = []

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

        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            sensor_qos,
        )

        self._pub_json = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), out_qos)
        self._pub_markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('markers_topic').value), out_qos)
        self._pub_image_annotations = self.create_publisher(
            ImageAnnotations,
            str(self.get_parameter('image_annotations_topic').value),
            out_qos,
        )

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'semantic VLM ready. image=%s depth=%s interval=%s model=%s' % (
                self.get_parameter('image_topic').value,
                self.get_parameter('depth_topic').value,
                self.get_parameter('frame_interval').value,
                self.get_parameter('model_name').value,
            )
        )

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_info = msg

    def _on_image(self, msg: Image) -> None:
        self._frame_seq += 1
        interval = max(1, int(self.get_parameter('frame_interval').value))
        if self._frame_seq % interval != 0:
            return

        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn(f'failed to convert RGB frame for VLM: {exc}')
            return

        job = VlmJob(
            rgb=np.asarray(rgb).copy(),
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
            depth_msg=self._latest_depth,
            camera_info=self._latest_info,
            seq=self._frame_seq,
        )
        with self._job_cv:
            if self._pending_job is not None or self._worker_busy:
                self.get_logger().debug('dropping VLM sample because previous inference is still running')
                return
            self._pending_job = job
            self._job_cv.notify()

    def _worker_loop(self) -> None:
        while True:
            with self._job_cv:
                while self._pending_job is None and not self._stop_worker:
                    self._job_cv.wait(timeout=0.2)
                if self._stop_worker and self._pending_job is None:
                    return
                job = self._pending_job
                self._pending_job = None
                self._worker_busy = True
            try:
                if job is not None:
                    self._run_job(job)
            finally:
                with self._job_cv:
                    self._worker_busy = False
                    self._job_cv.notify_all()

    def _run_job(self, job: VlmJob) -> None:
        start = time.monotonic()
        try:
            raw = self._infer_vlm(job.rgb)
        except Exception as exc:
            self.get_logger().error(f'VLM inference failed: {exc}')
            return

        parsed = self._parse_json(raw)
        if parsed is None:
            self.get_logger().warn(f'VLM JSON parse failed. raw_output={raw!r}')
            return

        height, width = job.rgb.shape[:2]
        task_mode = str(self.get_parameter('task_mode').value).strip().lower()
        if task_mode == 'scene_description':
            observation = self._validate_scene_description(parsed, width, height)
        else:
            observation = self._validate_observation(parsed, width, height)
        if observation is None:
            self.get_logger().warn(f'VLM schema validation failed. raw_output={raw!r}')
            return

        enriched_objects = []
        now_sec = self._stamp_sec(job.stamp)
        if task_mode != 'scene_description':
            for obj in observation['objects']:
                world_xyz, frame_id = self._position_object(obj, job)
                candidate = self._update_candidate(obj, world_xyz, frame_id, now_sec, observation)
                enriched = copy.deepcopy(obj)
                enriched.update({
                    'candidate_id': candidate.id,
                    'annotation_status': 'confirmed' if candidate.confirmed else 'candidate',
                    'observations': candidate.observations,
                    'world_xyz': list(candidate.world_xyz) if candidate.world_xyz is not None else None,
                    'frame_id': candidate.frame_id,
                })
                enriched_objects.append(enriched)
            self._prune_candidates(now_sec)
        else:
            enriched_objects = copy.deepcopy(observation['objects'])

        latency_s = time.monotonic() - start
        vram = self._vram_snapshot()
        self._warn_if_over_vram_budget(vram)
        payload = {
            'task_mode': task_mode,
            'has_text_object': observation.get('has_text_object', False),
            'objects': enriched_objects,
            'control_summary_ko': observation['control_summary_ko'],
            'need_human_check': observation['need_human_check'],
            'annotations': [] if task_mode == 'scene_description' else [self._candidate_to_dict(c) for c in self._candidates],
            'metadata': {
                'source_stamp': self._stamp_dict(job.stamp),
                'source_frame': job.frame_id,
                'image_width': width,
                'image_height': height,
                'frame_seq': job.seq,
                'model': str(self.get_parameter('model_name').value),
                'latency_s': round(latency_s, 3),
                'vram': vram,
            },
        }
        if task_mode == 'scene_description':
            payload['scene_description_ko'] = observation['scene_description_ko']
        if bool(self.get_parameter('publish_raw_vlm_output').value):
            payload['raw_vlm_output'] = raw

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._pub_json.publish(msg)
        self._pub_markers.publish(self._make_markers(job.stamp))
        self._pub_image_annotations.publish(
            self._make_image_annotations(job.stamp, enriched_objects)
        )
        self.get_logger().info(
            f'VLM mode={task_mode} frame={job.seq} objects={len(enriched_objects)} '
            f'candidates={len(self._candidates)} latency={latency_s:.2f}s')

    def _ensure_model(self) -> None:
        if self._model is not None and self._processor is not None:
            return
        with self._load_lock:
            if self._model is not None and self._processor is not None:
                return

            import torch
            from transformers import AutoProcessor

            model_name = str(self.get_parameter('model_name').value)
            requested_device = str(self.get_parameter('device').value).lower()
            dtype_param = str(self.get_parameter('torch_dtype').value).lower()
            if dtype_param == 'auto':
                torch_dtype = 'auto'
            elif dtype_param in ('float16', 'fp16'):
                torch_dtype = torch.float16
            elif dtype_param in ('bfloat16', 'bf16'):
                torch_dtype = torch.bfloat16
            else:
                torch_dtype = torch.float32

            if requested_device == 'auto':
                self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
            else:
                self._device = requested_device

            model_cls = None
            for module_name, class_name in (
                ('transformers', 'Qwen2_5_VLForConditionalGeneration'),
                ('transformers', 'AutoModelForImageTextToText'),
                ('transformers', 'AutoModelForVision2Seq'),
            ):
                try:
                    module = __import__(module_name, fromlist=[class_name])
                    model_cls = getattr(module, class_name)
                    break
                except Exception:
                    continue
            if model_cls is None:
                from transformers import AutoModelForCausalLM
                model_cls = AutoModelForCausalLM

            self.get_logger().info(f'loading VLM {model_name} on {self._device}')
            self._processor = AutoProcessor.from_pretrained(
                model_name,
                trust_remote_code=True,
            )
            self._model = model_cls.from_pretrained(
                model_name,
                torch_dtype=torch_dtype,
                trust_remote_code=True,
            ).eval()
            if self._device != 'cpu':
                self._model.to(self._device)
            self._torch = torch
            self.get_logger().info(f'loaded VLM {model_name}')

    def _infer_vlm(self, rgb: np.ndarray) -> str:
        self._ensure_model()
        from PIL import Image as PILImage

        assert self._processor is not None
        assert self._model is not None
        assert self._torch is not None

        pil_image = PILImage.fromarray(rgb)
        messages = [
            {'role': 'system', 'content': self._system_prompt()},
            {
                'role': 'user',
                'content': [
                    {'type': 'image', 'image': pil_image},
                    {'type': 'text', 'text': self._user_prompt()},
                ],
            },
        ]
        text = self._processor.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        inputs = self._processor(
            text=[text],
            images=[pil_image],
            padding=True,
            return_tensors='pt',
        )
        inputs = {
            key: value.to(self._device) if hasattr(value, 'to') else value
            for key, value in inputs.items()
        }
        input_len = int(inputs['input_ids'].shape[-1])
        max_new_tokens = max(1, int(self.get_parameter('max_new_tokens').value))
        with self._torch.no_grad():
            generated = self._model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                do_sample=False,
            )
        generated = generated[:, input_len:]
        return self._processor.batch_decode(
            generated,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0].strip()

    def _parse_json(self, raw: str) -> Optional[dict]:
        text = raw.strip()
        if text.startswith('```'):
            text = re.sub(r'^```(?:json)?\s*', '', text)
            text = re.sub(r'\s*```$', '', text)
        try:
            obj = json.loads(text)
            return obj if isinstance(obj, dict) else None
        except Exception:
            pass
        start = text.find('{')
        end = text.rfind('}')
        if start < 0 or end <= start:
            return None
        try:
            obj = json.loads(text[start:end + 1])
            return obj if isinstance(obj, dict) else None
        except Exception:
            return None

    def _validate_observation(self, obj: dict, width: int, height: int) -> Optional[dict]:
        objects_raw = obj.get('objects', [])
        if objects_raw is None:
            objects_raw = []
        if not isinstance(objects_raw, list):
            return None

        valid_objects = []
        force_human = False
        for item in objects_raw:
            if not isinstance(item, dict):
                continue
            object_type = str(item.get('type', 'other')).strip()
            if object_type not in OBJECT_TYPES:
                object_type = 'other'

            bbox = self._validate_bbox(item.get('bbox_xyxy'), width, height)
            text = item.get('text')
            if not isinstance(text, str) or not text.strip():
                text = None
            else:
                text = text.strip()

            confidence = str(item.get('confidence', 'low')).strip().lower()
            if confidence not in CONFIDENCES:
                confidence = 'low'
            failure_reason = str(item.get('failure_reason', 'unknown')).strip().lower()
            if failure_reason not in FAILURE_REASONS:
                failure_reason = 'unknown'
            if confidence == 'low' or text is None:
                force_human = True

            valid_objects.append({
                'type': object_type,
                'bbox_xyxy': bbox,
                'text': text,
                'confidence': confidence,
                'failure_reason': failure_reason,
            })

        has_text_object = bool(obj.get('has_text_object', bool(valid_objects))) and bool(valid_objects)
        summary = obj.get('control_summary_ko')
        if not isinstance(summary, str) or not summary.strip():
            summary = (
                '카메라 프레임에서 확인 가능한 텍스트 객체가 없습니다.'
                if not valid_objects else
                '텍스트 객체가 감지되었으며 일부 항목은 사람 확인이 필요합니다.'
            )
        need_human = bool(obj.get('need_human_check', False)) or force_human

        return {
            'has_text_object': has_text_object,
            'objects': valid_objects,
            'control_summary_ko': summary.strip(),
            'need_human_check': need_human,
        }

    def _validate_bbox(self, bbox, width: int, height: int) -> Optional[list[int]]:
        if not isinstance(bbox, list) or len(bbox) != 4:
            return None
        try:
            values = [float(v) for v in bbox]
        except Exception:
            return None
        if not all(math.isfinite(v) for v in values):
            return None
        x1, y1, x2, y2 = values
        x1 = int(round(min(max(x1, 0.0), float(max(width - 1, 0)))))
        x2 = int(round(min(max(x2, 0.0), float(max(width - 1, 0)))))
        y1 = int(round(min(max(y1, 0.0), float(max(height - 1, 0)))))
        y2 = int(round(min(max(y2, 0.0), float(max(height - 1, 0)))))
        if x2 <= x1 or y2 <= y1:
            return None
        return [x1, y1, x2, y2]

    def _validate_scene_description(self, obj: dict, width: int, height: int) -> Optional[dict]:
        scene_description = obj.get('scene_description_ko')
        if not isinstance(scene_description, str) or not scene_description.strip():
            return None
        objects_raw = obj.get('objects', [])
        if objects_raw is None:
            objects_raw = []
        if not isinstance(objects_raw, list):
            return None

        valid_objects = []
        for item in objects_raw:
            if not isinstance(item, dict):
                continue
            label = item.get('label')
            if not isinstance(label, str) or not label.strip():
                continue
            bbox = self._validate_bbox(item.get('bbox_xyxy'), width, height)
            confidence = str(item.get('confidence', 'low')).strip().lower()
            if confidence not in CONFIDENCES:
                confidence = 'low'
            evidence = item.get('visible_evidence_ko')
            if not isinstance(evidence, str) or not evidence.strip():
                evidence = label.strip()
            valid_objects.append({
                'label': label.strip(),
                'bbox_xyxy': bbox,
                'visible_evidence_ko': evidence.strip(),
                'confidence': confidence,
                'pixel_location_ko': self._pixel_location_ko(bbox, width, height),
            })

        summary = obj.get('control_summary_ko')
        if not isinstance(summary, str) or not summary.strip():
            summary = scene_description
        return {
            'scene_description_ko': scene_description.strip(),
            'objects': valid_objects,
            'control_summary_ko': summary.strip(),
            'need_human_check': bool(obj.get('need_human_check', False)) or any(
                o.get('confidence') == 'low' or o.get('bbox_xyxy') is None for o in valid_objects
            ),
        }

    @staticmethod
    def _pixel_location_ko(bbox: Optional[list[int]], width: int, height: int) -> Optional[str]:
        if bbox is None or width <= 0 or height <= 0:
            return None
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        hx = cx / float(width)
        hy = cy / float(height)
        if hx < 1.0 / 3.0:
            x_desc = '좌측'
        elif hx < 2.0 / 3.0:
            x_desc = '중앙'
        else:
            x_desc = '우측'
        if hy < 1.0 / 3.0:
            y_desc = '상단'
        elif hy < 2.0 / 3.0:
            y_desc = '중단'
        else:
            y_desc = '하단'
        return f'{x_desc} {y_desc} ({x1},{y1})-({x2},{y2})'

    def _system_prompt(self) -> str:
        task_mode = str(self.get_parameter('task_mode').value).strip().lower()
        if task_mode == 'scene_description':
            return SCENE_DESCRIPTION_SYSTEM_PROMPT
        return TEXT_OBJECT_SYSTEM_PROMPT

    def _user_prompt(self) -> str:
        task_mode = str(self.get_parameter('task_mode').value).strip().lower()
        if task_mode == 'scene_description':
            return SCENE_DESCRIPTION_USER_PROMPT
        return TEXT_OBJECT_USER_PROMPT

    def _position_object(
        self,
        obj: dict,
        job: VlmJob,
    ) -> tuple[Optional[tuple[float, float, float]], str]:
        bbox = obj.get('bbox_xyxy')
        if bbox is None or job.depth_msg is None or job.camera_info is None:
            return None, ''
        z = self._sample_depth(bbox, job.depth_msg)
        if z is None:
            return None, ''
        k = job.camera_info.k
        fx, fy = float(k[0]), float(k[4])
        cx, cy = float(k[2]), float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None, ''
        x1, y1, x2, y2 = bbox
        u = (float(x1) + float(x2)) * 0.5
        v = (float(y1) + float(y2)) * 0.5
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        source_frame = job.camera_info.header.frame_id or job.frame_id
        for target in (
            str(self.get_parameter('target_frame').value),
            str(self.get_parameter('fallback_target_frame').value),
        ):
            if not target:
                continue
            world = self._transform_point((x, y, z), source_frame, target, job.stamp)
            if world is not None:
                return world, target
        return (float(x), float(y), float(z)), source_frame

    def _sample_depth(self, bbox: list[int], depth_msg: Image) -> Optional[float]:
        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'failed to convert depth image: {exc}')
            return None
        x1, y1, x2, y2 = bbox
        u = int(round((x1 + x2) * 0.5))
        v = int(round((y1 + y2) * 0.5))
        radius = max(1, int(self.get_parameter('depth_window_px').value))
        y0, y3 = max(0, v - radius), min(depth.shape[0], v + radius + 1)
        x0, x3 = max(0, u - radius), min(depth.shape[1], u + radius + 1)
        patch = np.asarray(depth[y0:y3, x0:x3])
        if patch.size == 0:
            return None
        values = patch.astype(np.float32).reshape(-1)
        encoding = (depth_msg.encoding or '').lower()
        if '16u' in encoding or 'mono16' in encoding:
            values *= 0.001
        max_depth = max(0.1, float(self.get_parameter('max_depth_m').value))
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

    def _update_candidate(
        self,
        obj: dict,
        world_xyz: Optional[tuple[float, float, float]],
        frame_id: str,
        now_sec: float,
        observation: dict,
    ) -> SemanticCandidate:
        best = None
        best_score = float('inf')
        text_norm = self._normalize_text(obj.get('text'))
        radius = max(0.0, float(self.get_parameter('match_radius_m').value))
        window = max(0.0, float(self.get_parameter('confirm_window_s').value))
        for candidate in self._candidates:
            if candidate.object_type != obj['type']:
                continue
            if window > 0.0 and now_sec - candidate.last_seen_sec > window:
                continue
            if not self._text_compatible(text_norm, self._normalize_text(candidate.text)):
                continue
            score = 0.0
            if world_xyz is not None and candidate.world_xyz is not None:
                score = float(np.linalg.norm(
                    np.asarray(world_xyz) - np.asarray(candidate.world_xyz)))
                if radius > 0.0 and score > radius:
                    continue
            elif text_norm:
                score = radius * 0.5
            else:
                continue
            if score < best_score:
                best = candidate
                best_score = score

        need_human = obj['confidence'] == 'low' or obj.get('text') is None
        if best is None:
            best = SemanticCandidate(
                id=uuid.uuid4().hex[:12],
                object_type=obj['type'],
                text=obj.get('text'),
                bbox_xyxy=copy.deepcopy(obj.get('bbox_xyxy')),
                confidence=obj['confidence'],
                failure_reason=obj['failure_reason'],
                frame_id=frame_id,
                world_xyz=world_xyz,
                first_seen_sec=now_sec,
                last_seen_sec=now_sec,
                need_human_check=need_human,
                control_summary_ko=observation['control_summary_ko'],
            )
            self._candidates.append(best)
        else:
            best.observations += 1
            best.last_seen_sec = now_sec
            best.bbox_xyxy = copy.deepcopy(obj.get('bbox_xyxy'))
            best.confidence = self._max_confidence(best.confidence, obj['confidence'])
            best.failure_reason = obj['failure_reason']
            best.need_human_check = best.need_human_check or need_human
            best.control_summary_ko = observation['control_summary_ko']
            if obj.get('text') is not None:
                best.text = obj.get('text')
            if world_xyz is not None:
                if best.world_xyz is None:
                    best.world_xyz = world_xyz
                else:
                    old = np.asarray(best.world_xyz, dtype=np.float64)
                    new = np.asarray(world_xyz, dtype=np.float64)
                    best.world_xyz = tuple(float(v) for v in (0.7 * old + 0.3 * new))
                best.frame_id = frame_id

        min_obs = max(1, int(self.get_parameter('confirm_min_observations').value))
        if best.observations >= min_obs:
            best.confirmed = True
        return best

    def _prune_candidates(self, now_sec: float) -> None:
        ttl = max(1.0, float(self.get_parameter('candidate_ttl_s').value))
        self._candidates = [
            c for c in self._candidates
            if c.confirmed or now_sec - c.last_seen_sec <= ttl
        ]

    def _make_markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        delete = Marker()
        delete.header.frame_id = str(self.get_parameter('target_frame').value)
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        marker_id = 1
        for candidate in self._candidates:
            if candidate.world_xyz is None:
                continue
            x, y, z = candidate.world_xyz
            color = (0.1, 0.9, 0.2, 0.9) if candidate.confirmed else (1.0, 0.75, 0.1, 0.75)

            sphere = Marker()
            sphere.header.frame_id = candidate.frame_id or str(self.get_parameter('target_frame').value)
            sphere.header.stamp = stamp
            sphere.ns = 'semantic_vlm_points'
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
            label.ns = 'semantic_vlm_labels'
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
            status = 'confirmed' if candidate.confirmed else 'candidate'
            text = candidate.text if candidate.text is not None else 'unreadable'
            label.text = f'{status}: {candidate.object_type} {text}'
            markers.markers.append(label)
        return markers

    def _make_image_annotations(self, stamp, objects: list[dict]) -> ImageAnnotations:
        msg = ImageAnnotations()
        msg.timestamp = stamp

        for index, obj in enumerate(objects):
            bbox = obj.get('bbox_xyxy')
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = [float(v) for v in bbox]
            outline = self._annotation_color(str(obj.get('confidence') or 'low'))
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
                KeyValuePair(key='type', value=str(obj.get('type') or 'other')),
                KeyValuePair(key='confidence', value=str(obj.get('confidence') or 'low')),
                KeyValuePair(key='failure_reason', value=str(obj.get('failure_reason') or 'unknown')),
            ]
            msg.points.append(box)

            label_text = (
                obj.get('text')
                or obj.get('label')
                or f"{obj.get('type', 'text')} ({obj.get('confidence', 'low')})"
            )
            label = TextAnnotation()
            label.timestamp = stamp
            label.position = Point2(x=x1, y=max(0.0, y1 - 6.0))
            label.text = str(label_text)
            label.font_size = 14.0
            label.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
            label.background_color = Color(r=0.0, g=0.0, b=0.0, a=0.65)
            label.metadata = [KeyValuePair(key='index', value=str(index))]
            msg.texts.append(label)

        return msg

    @staticmethod
    def _annotation_color(confidence: str) -> Color:
        if confidence == 'high':
            return Color(r=0.10, g=0.90, b=0.20, a=1.0)
        if confidence == 'medium':
            return Color(r=1.00, g=0.75, b=0.10, a=1.0)
        return Color(r=0.95, g=0.25, b=0.25, a=1.0)

    def _candidate_to_dict(self, candidate: SemanticCandidate) -> dict:
        return {
            'id': candidate.id,
            'type': candidate.object_type,
            'text': candidate.text,
            'bbox_xyxy': candidate.bbox_xyxy,
            'confidence': candidate.confidence,
            'failure_reason': candidate.failure_reason,
            'annotation_status': 'confirmed' if candidate.confirmed else 'candidate',
            'observations': candidate.observations,
            'world_xyz': list(candidate.world_xyz) if candidate.world_xyz is not None else None,
            'frame_id': candidate.frame_id,
            'need_human_check': candidate.need_human_check,
            'control_summary_ko': candidate.control_summary_ko,
        }

    def _vram_snapshot(self) -> Optional[dict]:
        torch = self._torch
        if torch is None or not hasattr(torch, 'cuda') or not torch.cuda.is_available():
            return None
        try:
            return {
                'allocated_mb': round(torch.cuda.memory_allocated() / 1048576.0, 1),
                'reserved_mb': round(torch.cuda.memory_reserved() / 1048576.0, 1),
                'max_allocated_mb': round(torch.cuda.max_memory_allocated() / 1048576.0, 1),
            }
        except Exception:
            return None

    def _warn_if_over_vram_budget(self, vram: Optional[dict]) -> None:
        if vram is None:
            return
        budget = max(0.0, float(self.get_parameter('vram_budget_mb').value))
        if budget <= 0.0:
            return
        used = float(vram.get('max_allocated_mb') or 0.0)
        if used > budget:
            self.get_logger().warn(
                f'VLM max allocated VRAM {used:.1f} MB exceeds budget {budget:.1f} MB',
                throttle_duration_sec=30.0,
            )

    @staticmethod
    def _normalize_text(text: Optional[str]) -> str:
        if not isinstance(text, str):
            return ''
        return re.sub(r'\s+', '', text).lower()

    @staticmethod
    def _text_compatible(new: str, old: str) -> bool:
        if not new or not old:
            return True
        return new == old or new in old or old in new

    @staticmethod
    def _max_confidence(a: str, b: str) -> str:
        order = {'low': 0, 'medium': 1, 'high': 2}
        return a if order.get(a, 0) >= order.get(b, 0) else b

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _stamp_dict(stamp) -> dict:
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

    def destroy_node(self) -> None:
        with self._job_cv:
            self._stop_worker = True
            self._job_cv.notify_all()
        if hasattr(self, '_worker') and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SemanticVlmNode()
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
