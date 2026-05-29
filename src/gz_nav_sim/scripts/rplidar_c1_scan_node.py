#!/usr/bin/env python3
"""RPLIDAR C-series serial reader publishing sensor_msgs/LaserScan.

This intentionally keeps only the serial connection/protocol path from the
standalone web viewer. It sends the standard RPLIDAR scan command, groups raw
5-byte samples by rotation, bins them into a fixed LaserScan array, and
publishes /scan for SLAM Toolbox/Nav2.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime in __init__
    serial = None


CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52


def make_command(cmd: int, payload: bytes = b'') -> bytes:
    if not payload:
        return bytes([0xA5, cmd])
    packet = bytearray([0xA5, cmd, len(payload)])
    packet.extend(payload)
    checksum = 0
    for byte in packet:
        checksum ^= byte
    packet.append(checksum)
    return bytes(packet)


def parse_scan_point(data: bytes) -> Optional[tuple[int, float, float, int]]:
    if len(data) != 5:
        return None
    b0, b1, b2, b3, b4 = data
    start = b0 & 1
    inverted_start = (b0 >> 1) & 1
    if start == inverted_start:
        return None
    if (b1 & 1) != 1:
        return None

    quality = b0 >> 2
    angle_deg = (((b1 >> 1) | (b2 << 7)) / 64.0) % 360.0
    distance_mm = (b3 | (b4 << 8)) / 4.0
    return start, angle_deg, distance_mm, quality


class RPLidarC1ScanNode(Node):
    def __init__(self) -> None:
        super().__init__('rplidar_c1_scan_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('samples_per_scan', 720)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('invert', False)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('min_quality', 0)
        self.declare_parameter('min_points_per_rotation', 30)
        self.declare_parameter('serial_timeout_s', 0.02)
        self.declare_parameter('reconnect_delay_s', 1.0)

        g = lambda name: self.get_parameter(name).value
        self._serial_port = str(g('serial_port'))
        self._baud = int(g('baud'))
        self._frame_id = str(g('frame_id'))
        self._samples_per_scan = max(90, int(g('samples_per_scan')))
        self._angle_min = float(g('angle_min'))
        self._angle_max = float(g('angle_max'))
        self._angle_offset = math.radians(float(g('angle_offset_deg')))
        self._invert = bool(g('invert'))
        self._range_min = float(g('range_min'))
        self._range_max = float(g('range_max'))
        self._min_quality = int(g('min_quality'))
        self._min_points = int(g('min_points_per_rotation'))
        self._serial_timeout_s = float(g('serial_timeout_s'))
        self._reconnect_delay_s = float(g('reconnect_delay_s'))

        self._pub = self.create_publisher(LaserScan, str(g('scan_topic')), 10)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ser = None
        self._last_error = ''

        if serial is None:
            self.get_logger().error(
                'python serial module is missing. Install python3-serial or pyserial.')
            return

        self._thread = threading.Thread(target=self._run_reader, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'RPLIDAR C-series reader: {self._serial_port} @ {self._baud}, '
            f'frame={self._frame_id}, topic={g("scan_topic")}')

    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            if self._ser is not None:
                self._send(CMD_STOP)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _send(self, cmd: int, payload: bytes = b'') -> None:
        if self._ser is None:
            return
        self._ser.write(make_command(cmd, payload))
        self._ser.flush()

    def _read_descriptor(self, timeout_s: float = 1.0) -> Optional[dict]:
        deadline = time.monotonic() + timeout_s
        window = bytearray()
        while time.monotonic() < deadline and not self._stop_event.is_set():
            byte = self._ser.read(1)
            if not byte:
                continue
            window += byte
            while len(window) >= 2 and not (window[0] == 0xA5 and window[1] == 0x5A):
                del window[0]
            if len(window) == 7:
                raw_len = int.from_bytes(window[2:6], 'little')
                return {
                    'size': raw_len & 0x3FFFFFFF,
                    'mode': (raw_len >> 30) & 0x03,
                    'type': window[6],
                    'raw': bytes(window),
                }
        return None

    def _read_exact(self, size: int, timeout_s: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while len(data) < size and time.monotonic() < deadline and not self._stop_event.is_set():
            chunk = self._ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def _command_response(self, cmd: int, timeout_s: float = 1.0) -> tuple[Optional[dict], bytes]:
        self._ser.reset_input_buffer()
        self._send(cmd)
        descriptor = self._read_descriptor(timeout_s)
        if not descriptor:
            return None, b''
        return descriptor, self._read_exact(int(descriptor['size']), timeout_s)

    def _read_info(self) -> None:
        descriptor, data = self._command_response(CMD_GET_INFO, 1.0)
        if not descriptor or len(data) != 20:
            return
        model = data[0]
        firmware = f'{data[2]}.{data[1]}'
        hardware = data[3]
        serial_number = data[4:].hex()
        self.get_logger().info(
            f'RPLIDAR info: model=0x{model:02x}, firmware={firmware}, '
            f'hardware=0x{hardware:02x}, serial={serial_number}')

    def _read_health(self) -> None:
        descriptor, data = self._command_response(CMD_GET_HEALTH, 1.0)
        if not descriptor or len(data) != 3:
            return
        labels = {0: 'good', 1: 'warning', 2: 'error'}
        error_code = data[1] | (data[2] << 8)
        self.get_logger().info(
            f'RPLIDAR health: {labels.get(data[0], str(data[0]))}, error_code={error_code}')

    def _start_scan(self) -> None:
        self._ser.reset_input_buffer()
        self._send(CMD_SCAN)
        descriptor = self._read_descriptor(1.0)
        if not descriptor:
            raise RuntimeError('scan descriptor timeout')
        if descriptor['size'] != 5 or descriptor['type'] != 0x81:
            raise RuntimeError(
                f'unexpected scan descriptor: size={descriptor["size"]} '
                f'type=0x{descriptor["type"]:02x}')

    def _run_reader(self) -> None:
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                self.get_logger().info(
                    f'connecting to RPLIDAR: {self._serial_port} @ {self._baud}')
                with serial.Serial(
                    self._serial_port,
                    self._baud,
                    timeout=self._serial_timeout_s,
                    write_timeout=0.5,
                ) as ser:
                    self._ser = ser
                    ser.dtr = False
                    ser.rts = False
                    time.sleep(0.1)

                    self._send(CMD_STOP)
                    time.sleep(0.1)
                    self._read_info()
                    self._read_health()
                    self._start_scan()
                    self._last_error = ''

                    current: list[tuple[float, float, int]] = []
                    buffer = bytearray()
                    rotation_started_at: Optional[float] = None

                    while not self._stop_event.is_set() and rclpy.ok():
                        chunk = ser.read(4096)
                        if not chunk:
                            continue
                        buffer.extend(chunk)
                        while len(buffer) >= 5:
                            raw = bytes(buffer[:5])
                            del buffer[:5]
                            parsed = parse_scan_point(raw)
                            if parsed is None:
                                continue
                            start, angle_deg, distance_mm, quality = parsed
                            now = time.monotonic()
                            if start:
                                if len(current) >= self._min_points:
                                    scan_time = 0.0
                                    if rotation_started_at is not None:
                                        scan_time = max(now - rotation_started_at, 1e-6)
                                    self._publish_rotation(current, scan_time)
                                current = []
                                rotation_started_at = now
                            if distance_mm > 0:
                                current.append((angle_deg, distance_mm, quality))
            except Exception as exc:
                msg = str(exc)
                if msg != self._last_error:
                    self.get_logger().error(f'RPLIDAR reader error: {msg}')
                    self._last_error = msg
                time.sleep(self._reconnect_delay_s)
            finally:
                try:
                    self._send(CMD_STOP)
                except Exception:
                    pass
                self._ser = None

    def _publish_rotation(self, points: list[tuple[float, float, int]], scan_time: float) -> None:
        if self._angle_max <= self._angle_min:
            self.get_logger().error('angle_max must be greater than angle_min')
            return

        count = self._samples_per_scan
        span = self._angle_max - self._angle_min
        angle_increment = span / float(count)
        ranges = [math.inf] * count
        intensities = [0.0] * count

        for angle_deg, distance_mm, quality in points:
            if quality < self._min_quality:
                continue
            distance_m = distance_mm / 1000.0
            if distance_m < self._range_min or distance_m > self._range_max:
                continue

            angle = math.radians(angle_deg) + self._angle_offset
            if self._invert:
                angle = -angle
            while angle < self._angle_min:
                angle += math.tau
            while angle >= self._angle_max:
                angle -= math.tau

            index = int((angle - self._angle_min) / angle_increment)
            if index < 0 or index >= count:
                continue
            if not math.isfinite(ranges[index]) or distance_m < ranges[index]:
                ranges[index] = distance_m
                intensities[index] = float(quality)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = self._angle_min
        msg.angle_increment = angle_increment
        msg.angle_max = self._angle_min + angle_increment * float(count - 1)
        msg.time_increment = scan_time / float(count) if scan_time > 0.0 else 0.0
        msg.scan_time = scan_time
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self._pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RPLidarC1ScanNode()
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


if __name__ == '__main__':
    main()
