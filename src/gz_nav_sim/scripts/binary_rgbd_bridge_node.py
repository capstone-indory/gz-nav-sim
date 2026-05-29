#!/usr/bin/env python3
"""Receive Pi RGB-D binary frames and publish standard ROS 2 camera topics."""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def _read_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("peer closed")
        data.extend(chunk)
    return bytes(data)


def _stamp(value) -> Time:
    msg = Time()
    if isinstance(value, dict):
        msg.sec = int(value.get("sec", 0))
        msg.nanosec = int(value.get("nanosec", value.get("nsec", 0)))
    return msg


def _camera_info(data: dict, fallback_frame: str, stamp: Time) -> CameraInfo:
    msg = CameraInfo()
    header = data.get("header", {}) if isinstance(data, dict) else {}
    msg.header.stamp = stamp
    msg.header.frame_id = str(header.get("frame_id") or fallback_frame)
    msg.height = int(data.get("height", 0))
    msg.width = int(data.get("width", 0))
    msg.distortion_model = str(data.get("distortion_model", "plumb_bob"))
    msg.d = [float(x) for x in data.get("d", [])]
    msg.k = [float(x) for x in data.get("k", [0.0] * 9)]
    msg.r = [float(x) for x in data.get("r", [0.0] * 9)]
    msg.p = [float(x) for x in data.get("p", [0.0] * 12)]
    return msg


class BinaryRgbdBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("binary_rgbd_bridge")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 9102)
        self.declare_parameter("color_image_topic", "/camera/image_raw")
        self.declare_parameter("color_compressed_topic", "/camera/image_raw/compressed")
        self.declare_parameter("color_info_topic", "/camera/camera_info")
        self.declare_parameter("depth_image_topic", "/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/depth/camera_info")
        self.declare_parameter("color_frame_id", "camera_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_optical_frame")
        self.declare_parameter("publish_color_raw", True)
        self.declare_parameter("publish_color_compressed", True)
        self.declare_parameter("max_header_bytes", 1048576)
        self.declare_parameter("max_payload_bytes", 52428800)

        g = lambda name: self.get_parameter(name).value
        self._listen_host = str(g("listen_host"))
        self._listen_port = int(g("listen_port"))
        self._publish_color_raw = bool(g("publish_color_raw"))
        self._publish_color_compressed = bool(g("publish_color_compressed"))
        self._color_frame_id = str(g("color_frame_id")).strip()
        self._depth_frame_id = str(g("depth_frame_id")).strip()
        self._max_header_bytes = int(g("max_header_bytes"))
        self._max_payload_bytes = int(g("max_payload_bytes"))

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        info_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub_color = (
            self.create_publisher(Image, str(g("color_image_topic")), sensor_qos)
            if self._publish_color_raw else None
        )
        self._pub_color_compressed = (
            self.create_publisher(CompressedImage, str(g("color_compressed_topic")), sensor_qos)
            if self._publish_color_compressed else None
        )
        self._pub_color_info = self.create_publisher(
            CameraInfo, str(g("color_info_topic")), info_qos)
        self._pub_depth = self.create_publisher(
            Image, str(g("depth_image_topic")), sensor_qos)
        self._pub_depth_info = self.create_publisher(
            CameraInfo, str(g("depth_info_topic")), info_qos)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, name="binary-rgbd-tcp", daemon=True)
        self._frames = 0
        self._bytes = 0
        self._last_frame_time = 0.0
        self._last_report_frames = 0
        self._last_report_time = time.monotonic()
        self.create_timer(5.0, self._report)
        self._thread.start()
        self.get_logger().info(
            f"binary RGB-D bridge listening on {self._listen_host}:{self._listen_port}")

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    server.bind((self._listen_host, self._listen_port))
                    server.listen(1)
                    server.settimeout(1.0)
                    while not self._stop.is_set():
                        try:
                            conn, addr = server.accept()
                        except socket.timeout:
                            continue
                        with conn:
                            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                            self.get_logger().info(f"binary RGB-D client connected: {addr}")
                            self._handle_client(conn)
            except Exception as exc:
                if not self._stop.is_set():
                    self.get_logger().warn(f"binary RGB-D server error: {exc}")
                    time.sleep(1.0)

    def _handle_client(self, sock: socket.socket) -> None:
        while not self._stop.is_set():
            raw_len = _read_exact(sock, 4)
            header_len = struct.unpack("!I", raw_len)[0]
            if header_len <= 0 or header_len > self._max_header_bytes:
                raise ValueError(f"invalid header length: {header_len}")
            header = json.loads(_read_exact(sock, header_len).decode("utf-8"))
            if header.get("type") != "rgbd":
                continue
            color_len = int(header.get("color_len", 0))
            depth_len = int(header.get("depth_len", 0))
            payload_len = color_len + depth_len
            if color_len <= 0 or depth_len <= 0 or payload_len > self._max_payload_bytes:
                raise ValueError(f"invalid payload length: {payload_len}")
            payload = _read_exact(sock, payload_len)
            self._publish_rgbd(header, payload[:color_len], payload[color_len:])
            self._frames += 1
            self._bytes += 4 + header_len + payload_len
            self._last_frame_time = time.monotonic()

    def _publish_rgbd(self, header: dict, color_bytes: bytes, depth_bytes: bytes) -> None:
        stamp = _stamp(header.get("stamp"))
        color_frame = self._color_frame_id or str(header.get("color_frame_id", "camera_optical_frame"))
        depth_frame = self._depth_frame_id or str(header.get("depth_frame_id", color_frame))

        color_msg = CompressedImage()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = color_frame
        color_msg.format = str(header.get("color_format", "jpeg"))
        color_msg.data = color_bytes
        if self._pub_color_compressed is not None:
            self._pub_color_compressed.publish(color_msg)

        if self._pub_color is not None:
            color_encoding = str(header.get("color_encoding", "bgr8")).lower()
            decode_flag = cv2.IMREAD_GRAYSCALE if color_encoding == "mono8" else cv2.IMREAD_COLOR
            color = cv2.imdecode(np.frombuffer(color_bytes, dtype=np.uint8), decode_flag)
            if color is not None:
                raw_encoding = "mono8"
                if color.ndim == 3:
                    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                    raw_encoding = "rgb8"
                raw_color = Image()
                raw_color.header = color_msg.header
                raw_color.height = int(color.shape[0])
                raw_color.width = int(color.shape[1])
                raw_color.encoding = raw_encoding
                raw_color.is_bigendian = 0
                raw_color.step = int(color.shape[1] if color.ndim == 2 else color.shape[1] * color.shape[2])
                raw_color.data = np.ascontiguousarray(color).tobytes()
                self._pub_color.publish(raw_color)

        depth_format = str(header.get("depth_format", "raw16uc1-le")).lower()
        if depth_format.startswith("png"):
            depth = cv2.imdecode(np.frombuffer(depth_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if depth is None:
                return
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            depth = np.ascontiguousarray(depth.astype(np.uint16, copy=False))
        else:
            width = int(header.get("depth_width", 0))
            height = int(header.get("depth_height", 0))
            if width <= 0 or height <= 0:
                return
            depth = np.frombuffer(depth_bytes, dtype="<u2", count=width * height)
            depth = np.ascontiguousarray(depth.reshape((height, width)))

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = depth_frame
        depth_msg.height = int(depth.shape[0])
        depth_msg.width = int(depth.shape[1])
        depth_msg.encoding = str(header.get("depth_encoding", "16UC1"))
        depth_msg.is_bigendian = 0
        depth_msg.step = int(header.get("depth_step", depth.shape[1] * 2))
        depth_msg.data = depth.tobytes()
        self._pub_depth.publish(depth_msg)

        color_info = _camera_info(header.get("color_camera_info", {}), color_frame, stamp)
        depth_info = _camera_info(header.get("depth_camera_info", {}), depth_frame, stamp)
        color_info.header.frame_id = color_frame
        depth_info.header.frame_id = depth_frame
        self._pub_color_info.publish(color_info)
        self._pub_depth_info.publish(depth_info)

    def _report(self) -> None:
        now = time.monotonic()
        dt = max(now - self._last_report_time, 1e-6)
        frames = self._frames - self._last_report_frames
        age = now - self._last_frame_time if self._last_frame_time else -1.0
        self.get_logger().info(
            f"binary RGB-D rate={frames / dt:.1f} Hz total={self._frames} "
            f"bytes={self._bytes / 1_000_000.0:.1f} MB last_age={age:.2f}s")
        self._last_report_frames = self._frames
        self._last_report_time = now


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BinaryRgbdBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
