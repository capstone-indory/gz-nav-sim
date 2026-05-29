#!/usr/bin/env python3
"""Republish RTSP camera previews as ROS 2 CompressedImage topics.

This is for ROS tooling such as Foxglove. Browser video should still use the
RTSP/WebRTC media path directly.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage


@dataclass
class CameraSpec:
    name: str
    url: str
    image_topic: str
    info_topic: str
    frame_id: str


class RtspCameraWorker:
    def __init__(
        self,
        node: Node,
        spec: CameraSpec,
        rate_hz: float,
        jpeg_quality: int,
        retry_sec: float,
    ) -> None:
        self.node = node
        self.spec = spec
        self.period = 1.0 / max(0.1, rate_hz)
        self.jpeg_quality = max(1, min(100, jpeg_quality))
        self.retry_sec = max(0.1, retry_sec)
        qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        info_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self.image_pub = node.create_publisher(CompressedImage, spec.image_topic, qos)
        self.info_pub = node.create_publisher(CameraInfo, spec.info_topic, info_qos)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name=f"rtsp-camera-{spec.name}",
            daemon=True,
        )
        self.frames = 0
        self.last_report_frames = 0
        self.last_report_time = time.monotonic()

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def report(self) -> None:
        now = time.monotonic()
        dt = max(0.001, now - self.last_report_time)
        fps = (self.frames - self.last_report_frames) / dt
        self.node.get_logger().info(
            f"{self.spec.name}: {fps:.1f} Hz -> {self.spec.image_topic}")
        self.last_report_time = now
        self.last_report_frames = self.frames

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self.spec.url, cv2.CAP_FFMPEG)
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _run(self) -> None:
        next_publish = 0.0
        cap: cv2.VideoCapture | None = None
        while rclpy.ok() and not self.stop_event.is_set():
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                self.node.get_logger().info(f"{self.spec.name}: opening {self.spec.url}")
                cap = self._open()
                if not cap.isOpened():
                    self.node.get_logger().warn(
                        f"{self.spec.name}: RTSP open failed; retrying")
                    time.sleep(self.retry_sec)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                self.node.get_logger().warn(
                    f"{self.spec.name}: RTSP read failed; reconnecting")
                cap.release()
                cap = None
                time.sleep(self.retry_sec)
                continue
            now = time.monotonic()
            if now < next_publish:
                continue
            next_publish = now + self.period
            self._publish(frame)
        if cap is not None:
            cap.release()

    def _publish(self, frame) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        stamp = self.node.get_clock().now().to_msg()
        header = CompressedImage().header
        header.stamp = stamp
        header.frame_id = self.spec.frame_id

        image_msg = CompressedImage()
        image_msg.header = header
        image_msg.format = "jpeg"
        image_msg.data = encoded.tobytes()
        self.image_pub.publish(image_msg)

        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.height = int(frame.shape[0])
        info_msg.width = int(frame.shape[1])
        info_msg.distortion_model = "plumb_bob"
        self.info_pub.publish(info_msg)
        self.frames += 1


class RtspCompressedBridge(Node):
    def __init__(self) -> None:
        super().__init__("rtsp_compressed_bridge")
        self.declare_parameter("camera_names", "base,wrist_left,wrist_right")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("retry_sec", 1.0)

        names = [
            part.strip()
            for part in str(self.get_parameter("camera_names").value).split(",")
            if part.strip()
        ]
        self.workers: list[RtspCameraWorker] = []
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        retry_sec = float(self.get_parameter("retry_sec").value)
        for name in names:
            spec = self._declare_spec(name)
            if not spec.url:
                self.get_logger().info(f"{name}: disabled because URL is empty")
                continue
            worker = RtspCameraWorker(self, spec, rate_hz, jpeg_quality, retry_sec)
            self.workers.append(worker)
            worker.start()
        if self.workers:
            self.create_timer(5.0, self._report)
        self.get_logger().info(f"RTSP compressed bridge active: {len(self.workers)} cameras")

    def _declare_spec(self, name: str) -> CameraSpec:
        default_path = {
            "base": "xlerobot_base",
            "wrist_left": "xlerobot_wrist_left",
            "wrist_right": "xlerobot_wrist_right",
        }.get(name, name)
        default_topic_prefix = {
            "base": "/xlerobot/base_camera",
            "wrist_left": "/xlerobot/wrist_left_camera",
            "wrist_right": "/xlerobot/wrist_right_camera",
        }.get(name, f"/xlerobot/{name}_camera")
        self.declare_parameter(f"{name}.url", f"rtsp://127.0.0.1:8554/{default_path}")
        self.declare_parameter(f"{name}.image_topic", f"{default_topic_prefix}/image/compressed")
        self.declare_parameter(f"{name}.info_topic", f"{default_topic_prefix}/camera_info")
        self.declare_parameter(f"{name}.frame_id", f"{name}_camera_optical_frame")
        return CameraSpec(
            name=name,
            url=str(self.get_parameter(f"{name}.url").value).strip(),
            image_topic=str(self.get_parameter(f"{name}.image_topic").value).strip(),
            info_topic=str(self.get_parameter(f"{name}.info_topic").value).strip(),
            frame_id=str(self.get_parameter(f"{name}.frame_id").value).strip(),
        )

    def _report(self) -> None:
        for worker in self.workers:
            worker.report()

    def destroy_node(self) -> bool:
        for worker in self.workers:
            worker.stop()
        return super().destroy_node()


def main() -> None:
    os.environ.setdefault(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS",
        "rtsp_transport;tcp|max_delay;500000",
    )
    rclpy.init()
    node = RtspCompressedBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
