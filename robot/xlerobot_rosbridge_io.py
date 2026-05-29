#!/usr/bin/env python3
"""Lightweight XLeRobot hardware I/O over rosbridge.

This script is meant for the Raspberry Pi / robot computer. It does not import
ROS 2 or start DDS. The compute PC runs ROS, SLAM, Nav2, Foxglove, and
rosbridge_server; this process talks to that rosbridge websocket with roslibpy.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Any, Optional
from urllib.parse import urlparse

import numpy as np

try:
    import roslibpy
except Exception as exc:  # pragma: no cover - startup path
    roslibpy = None
    ROSLIBPY_IMPORT_ERROR = exc
else:
    ROSLIBPY_IMPORT_ERROR = None

try:
    import serial
except Exception as exc:  # pragma: no cover - runtime optional
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

try:
    import pyrealsense2 as rs
except Exception as exc:  # pragma: no cover - runtime optional
    rs = None
    DEPTH_SENSOR_IMPORT_ERROR = exc
else:
    DEPTH_SENSOR_IMPORT_ERROR = None


CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def env_int_list(name: str, default: tuple[int, ...]) -> list[int]:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return list(default)
    values: list[int] = []
    for item in raw.replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        try:
            values.append(int(item))
        except ValueError:
            return list(default)
    return values or list(default)


def env_first(names: tuple[str, ...], default: str) -> str:
    for name in names:
        value = os.environ.get(name)
        if value is not None:
            return value
    return default


def stamp() -> dict[str, int]:
    now = time.time()
    sec = int(now)
    return {"sec": sec, "nanosec": int((now - sec) * 1_000_000_000)}


def yaw_quat(yaw: float) -> dict[str, float]:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw * 0.5),
        "w": math.cos(yaw * 0.5),
    }


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def vector3(msg: dict[str, Any], key: str, axis: str) -> float:
    try:
        return float(msg.get(key, {}).get(axis, 0.0))
    except (TypeError, ValueError):
        return 0.0


def extend_pythonpath_from_env() -> None:
    for key in ("LEROBOT_ROOT", "XLE_LEROBOT_ROOT"):
        root = os.environ.get(key, "").strip()
        if not root:
            continue
        src = os.path.join(os.path.expanduser(root), "src")
        if os.path.isdir(src) and src not in sys.path:
            sys.path.insert(0, src)


def import_lerobot_feetech():
    extend_pythonpath_from_env()
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    return Motor, MotorNormMode, FeetechMotorsBus, OperatingMode


def make_command(cmd: int, payload: bytes = b"") -> bytes:
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


class TopicPublisher:
    def __init__(self, client, name: str, msg_type: str):
        self.topic = roslibpy.Topic(client, name, msg_type)
        self.topic.advertise()

    def publish(self, payload: dict[str, Any]) -> None:
        self.topic.publish(roslibpy.Message(payload))

    def close(self) -> None:
        try:
            self.topic.unadvertise()
        except Exception:
            pass


class StatusPublisher:
    def __init__(self, client, topic_name: str):
        self._pub = TopicPublisher(client, topic_name, "std_msgs/String")
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {}

    def update(self, **kwargs: Any) -> None:
        with self._lock:
            self._state.update(kwargs)
            payload = dict(self._state)
        payload["stamp"] = time.time()
        try:
            self._pub.publish({"data": json.dumps(payload, separators=(",", ":"))})
        except Exception:
            pass

    def close(self) -> None:
        self._pub.close()


class RosbridgeBase:
    JOINT_NAMES = (
        [f"left_hand_{i}" for i in range(1, 7)]
        + [f"right_hand_{i}" for i in range(1, 7)]
        + ["head_pan", "head_tilt"]
    )
    BASE_WHEEL_NAMES = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
    JOINT_STATE_NAMES = tuple(JOINT_NAMES) + BASE_WHEEL_NAMES

    def __init__(self, client, stop_event: threading.Event, status: StatusPublisher, dry_run: bool):
        self.client = client
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run

        self.cmd_topic = os.environ.get("CMD_TOPIC", "/xlerobot/cmd_vel")
        self.joint_target_topic = os.environ.get(
            "JOINT_TARGET_TOPIC", "/xlerobot/teleop/joint_targets")
        self.joint_states_topic = os.environ.get("JOINT_STATES_TOPIC", "/xlerobot/joint_states")
        self.odom_topic = os.environ.get("ODOM_TOPIC", "/xlerobot/odom")

        self.left_base_port = env_first(
            ("LEFT_BASE_PORT", "BASE_PORT", "LEFT_HAND_PORT"),
            "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D046415-if00")
        self.right_head_port = env_first(
            ("RIGHT_HEAD_PORT", "RIGHT_HAND_PORT", "HEAD_PORT"),
            "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14032190-if00")
        self.left_hand_ids = self._six_ids(
            env_int_list("LEFT_HAND_MOTOR_IDS", (1, 2, 3, 4, 5, 6)))
        self.right_hand_ids = self._six_ids(
            env_int_list("RIGHT_HAND_MOTOR_IDS", (1, 2, 3, 4, 5, 6)))
        self.head_ids = self._two_ids(
            env_int_list("HEAD_MOTOR_IDS", (env_int("HEAD_PAN_ID", 7), env_int("HEAD_TILT_ID", 8))))
        self.base_wheel_ids = {
            "base_left_wheel": env_int("BASE_LEFT_WHEEL_ID", 7),
            "base_back_wheel": env_int("BASE_BACK_WHEEL_ID", 8),
            "base_right_wheel": env_int("BASE_RIGHT_WHEEL_ID", 9),
        }

        self.wheel_radius = env_float("BASE_WHEEL_RADIUS_M", 0.05)
        self.base_radius = env_float("BASE_RADIUS_M", 0.125)
        self.left_sign = env_float("BASE_LEFT_SIGN", 1.0)
        self.back_sign = env_float("BASE_BACK_SIGN", 1.0)
        self.right_sign = env_float("BASE_RIGHT_SIGN", 1.0)
        self._wheel_signs = {
            "base_left_wheel": self.left_sign,
            "base_back_wheel": self.back_sign,
            "base_right_wheel": self.right_sign,
        }
        self.max_raw = env_int("BASE_MAX_RAW_COMMAND", 3000)
        self.command_rate_hz = max(1.0, env_float("BASE_COMMAND_RATE_HZ", 200.0))
        self.joint_command_rate_hz = max(1.0, env_float("JOINT_COMMAND_RATE_HZ", 50.0))
        self.joint_state_rate_hz = max(1.0, env_float("JOINT_STATE_RATE_HZ", 20.0))
        self.odom_rate_hz = max(1.0, env_float("BASE_FEEDBACK_RATE_HZ", 20.0))
        self.watchdog_timeout = max(0.05, env_float("BASE_WATCHDOG_TIMEOUT_S", 0.3))
        self.reconnect_delay_s = max(0.2, env_float("BASE_RECONNECT_DELAY_S", 2.0))
        self.joint_target_min = env_int("JOINT_TARGET_MIN", 0)
        self.joint_target_max = env_int("JOINT_TARGET_MAX", 4095)
        self.odom_frame = os.environ.get("ODOM_FRAME", "odom")
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")
        self.publish_odom = env_bool("BASE_PUBLISH_ODOM", False)
        self.use_feedback_odom = env_bool("BASE_USE_FEEDBACK_ODOM", True)
        self.disable_torque_on_shutdown = env_bool("BASE_DISABLE_TORQUE_ON_SHUTDOWN", False)
        self.motor_model = os.environ.get("FEETECH_MOTOR_MODEL", "sts3215")

        self._cmd_lock = threading.Lock()
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._last_cmd_at = 0.0
        self._last_base_written: Optional[tuple[int, int, int]] = None
        self._last_status_at = 0.0
        self._last_joint_target_at = 0.0
        self._last_connect_attempt = 0.0
        self._connected = False
        self._cmd_event = threading.Event()
        self._left_base_bus = None
        self._right_head_bus = None
        self._bus_lock = threading.RLock()
        self._left_base_available: set[str] = set()
        self._right_head_available: set[str] = set()
        self._left_base_detected_ids: list[int] = []
        self._right_head_detected_ids: list[int] = []
        self._left_base_missing_ids: list[int] = []
        self._right_head_missing_ids: list[int] = []
        self._base_ready = False
        self._lerobot = None
        self._joint_lock = threading.Lock()
        self._joint_targets: list[Optional[int]] = [None] * len(self.JOINT_NAMES)
        self._joint_written: dict[str, int] = {}
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_at = time.monotonic()
        self._threads: list[threading.Thread] = []

        angles = np.radians(np.array([240.0, 0.0, 120.0]) - 90.0)
        self._omni_matrix = np.array(
            [[math.cos(angle), math.sin(angle), self.base_radius] for angle in angles],
            dtype=float,
        )
        self._omni_inverse = np.linalg.pinv(self._omni_matrix)

        self._cmd_sub = roslibpy.Topic(client, self.cmd_topic, "geometry_msgs/Twist")
        self._joint_sub = roslibpy.Topic(
            client, self.joint_target_topic, "std_msgs/Float64MultiArray")
        self._joint_state_pub = TopicPublisher(
            client, self.joint_states_topic, "sensor_msgs/JointState")
        self._odom_pub = (
            TopicPublisher(client, self.odom_topic, "nav_msgs/Odometry")
            if self.publish_odom else None
        )

    @staticmethod
    def _six_ids(values: list[int]) -> list[int]:
        default = [1, 2, 3, 4, 5, 6]
        values = list(values[:6])
        return values + default[len(values):]

    @staticmethod
    def _two_ids(values: list[int]) -> list[int]:
        default = [7, 8]
        values = list(values[:2])
        return values + default[len(values):]

    @property
    def _left_base_motor_specs(self) -> dict[str, int]:
        specs = {f"left_hand_{i}": motor_id for i, motor_id in enumerate(self.left_hand_ids, 1)}
        specs.update(self.base_wheel_ids)
        return specs

    @property
    def _right_head_motor_specs(self) -> dict[str, int]:
        specs = {f"right_hand_{i}": motor_id for i, motor_id in enumerate(self.right_hand_ids, 1)}
        specs["head_pan"] = self.head_ids[0]
        specs["head_tilt"] = self.head_ids[1]
        return specs

    def start(self) -> None:
        self._cmd_sub.subscribe(self._on_cmd_vel)
        self._joint_sub.subscribe(self._on_joint_targets)
        self._threads = [
            threading.Thread(target=self._command_loop, name="motor-command", daemon=True),
            threading.Thread(target=self._joint_state_loop, name="motor-joint-state", daemon=True),
        ]
        if self.publish_odom:
            self._threads.append(threading.Thread(target=self._odom_loop, name="base-odom", daemon=True))
        for thread in self._threads:
            thread.start()
        mode = "dry-run" if self.dry_run else "hardware"
        odom_desc = self.odom_topic if self.publish_odom else "off"
        print(
            f"[motors] started ({mode}), cmd={self.cmd_topic}, "
            f"joint_targets={self.joint_target_topic}, joint_states={self.joint_states_topic}, "
            f"odom={odom_desc}",
            flush=True,
        )
        self.status.update(
            motors="started",
            base="started",
            motor_mode=mode,
            motor_left_base_port=self.left_base_port,
            motor_right_head_port=self.right_head_port,
            cmd_topic=self.cmd_topic,
            joint_target_topic=self.joint_target_topic,
            joint_states_topic=self.joint_states_topic,
            left_base_expected_ids=sorted(set(self._left_base_motor_specs.values())),
            right_head_expected_ids=sorted(set(self._right_head_motor_specs.values())),
        )

    def stop(self) -> None:
        self._safe_stop()
        try:
            self._cmd_sub.unsubscribe()
        except Exception:
            pass
        try:
            self._joint_sub.unsubscribe()
        except Exception:
            pass
        self._disconnect_buses()
        self._joint_state_pub.close()
        if self._odom_pub is not None:
            self._odom_pub.close()

    def _on_cmd_vel(self, msg: dict[str, Any]) -> None:
        linear_x = vector3(msg, "linear", "x")
        linear_y = vector3(msg, "linear", "y")
        angular_z = vector3(msg, "angular", "z")
        with self._cmd_lock:
            self._linear_x = linear_x
            self._linear_y = linear_y
            self._angular_z = angular_z
            self._last_cmd_at = time.monotonic()
        self._cmd_event.set()

    def _on_joint_targets(self, msg: dict[str, Any]) -> None:
        data = msg.get("data", [])
        if not isinstance(data, list):
            return
        changed = False
        with self._joint_lock:
            for index, raw_value in enumerate(data[:len(self.JOINT_NAMES)]):
                try:
                    value = float(raw_value)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(value):
                    continue
                target = int(round(clamp(value, self.joint_target_min, self.joint_target_max)))
                if self._joint_targets[index] != target:
                    self._joint_targets[index] = target
                    changed = True
            if changed:
                self._last_joint_target_at = time.monotonic()
        if changed:
            self._cmd_event.set()

    def _connect_buses(self) -> None:
        self._last_connect_attempt = time.monotonic()
        with self._bus_lock:
            self._disconnect_buses()
            self._last_base_written = None
            self._joint_written = {}

            if self.dry_run:
                self._left_base_available = set(self._left_base_motor_specs)
                self._right_head_available = set(self._right_head_motor_specs)
                self._left_base_detected_ids = sorted(set(self._left_base_motor_specs.values()))
                self._right_head_detected_ids = sorted(set(self._right_head_motor_specs.values()))
                self._left_base_missing_ids = []
                self._right_head_missing_ids = []
                self._base_ready = True
                self._connected = True
                self.status.update(motors="connected", base="connected", base_ready=True)
                return

            if self._lerobot is None:
                try:
                    self._lerobot = import_lerobot_feetech()
                except Exception as exc:
                    raise RuntimeError(f"LeRobot Feetech import failed: {exc}") from exc

            self._left_base_bus = self._open_detected_bus(
                self.left_base_port,
                self._left_base_motor_specs,
                base_wheel_names=self.BASE_WHEEL_NAMES,
            )
            self._right_head_bus = self._open_detected_bus(
                self.right_head_port,
                self._right_head_motor_specs,
                base_wheel_names=(),
            )
            self._left_base_available = set(self._left_base_bus.motors) if self._left_base_bus else set()
            self._right_head_available = set(self._right_head_bus.motors) if self._right_head_bus else set()
            self._base_ready = all(name in self._left_base_available for name in self.BASE_WHEEL_NAMES)
            self._connected = self._left_base_bus is not None or self._right_head_bus is not None
            if not self._connected:
                raise RuntimeError("no Feetech motors detected on either configured bus")

            status = "connected" if self._base_ready else "error"
            base_error = "" if self._base_ready else "base wheel ID missing; base velocity writes disabled"
            self.status.update(
                motors="connected",
                base=status,
                base_ready=self._base_ready,
                base_error=base_error,
                left_base_detected_ids=self._left_base_detected_ids,
                left_base_missing_ids=self._left_base_missing_ids,
                right_head_detected_ids=self._right_head_detected_ids,
                right_head_missing_ids=self._right_head_missing_ids,
            )
            print(
                f"[motors] connected left/base {self.left_base_port} ids={self._left_base_detected_ids} "
                f"missing={self._left_base_missing_ids}; right/head {self.right_head_port} "
                f"ids={self._right_head_detected_ids} missing={self._right_head_missing_ids}",
                flush=True,
            )

    def _open_detected_bus(self, port: str, specs: dict[str, int], base_wheel_names: tuple[str, ...]):
        Motor, MotorNormMode, FeetechMotorsBus, OperatingMode = self._lerobot
        motors = {
            name: Motor(motor_id, self.motor_model, MotorNormMode.RANGE_M100_100)
            for name, motor_id in specs.items()
        }
        probe = FeetechMotorsBus(port=port, motors=motors)
        detected_by_id: set[int] = set()
        try:
            probe.connect(handshake=False)
            for motor_id in sorted(set(specs.values())):
                if probe.ping(motor_id, num_retry=2) is not None:
                    detected_by_id.add(motor_id)
        finally:
            try:
                probe.disconnect(False)
            except Exception:
                pass

        missing = sorted(set(specs.values()) - detected_by_id)
        if port == self.left_base_port:
            self._left_base_detected_ids = sorted(detected_by_id)
            self._left_base_missing_ids = missing
        else:
            self._right_head_detected_ids = sorted(detected_by_id)
            self._right_head_missing_ids = missing

        available_motors = {
            name: motor for name, motor in motors.items()
            if motor.id in detected_by_id
        }
        if not available_motors:
            return None

        bus = FeetechMotorsBus(port=port, motors=available_motors)
        bus.connect(handshake=False)
        bus.configure_motors()
        for name in bus.motors:
            mode = OperatingMode.VELOCITY.value if name in base_wheel_names else OperatingMode.POSITION.value
            bus.write("Operating_Mode", name, mode, normalize=False)
            if name not in base_wheel_names:
                try:
                    bus.write("P_Coefficient", name, 16, normalize=False)
                    bus.write("I_Coefficient", name, 0, normalize=False)
                    bus.write("D_Coefficient", name, 43, normalize=False)
                except Exception:
                    pass
        bus.enable_torque()
        return bus

    def _disconnect_buses(self) -> None:
        with self._bus_lock:
            for attr in ("_left_base_bus", "_right_head_bus"):
                bus = getattr(self, attr)
                if bus is None:
                    continue
                try:
                    bus.disconnect(self.disable_torque_on_shutdown)
                except Exception:
                    pass
                setattr(self, attr, None)
            self._left_base_available = set()
            self._right_head_available = set()
            self._base_ready = False
            self._connected = False

    def _command_loop(self) -> None:
        delay = min(1.0 / self.command_rate_hz, 1.0 / self.joint_command_rate_hz)
        last_error = ""
        while not self.stop_event.is_set():
            if not self._connected:
                if time.monotonic() - self._last_connect_attempt >= self.reconnect_delay_s:
                    try:
                        self._connect_buses()
                        last_error = ""
                    except Exception as exc:
                        msg = str(exc)
                        if msg != last_error:
                            print(f"[motors] connect failed: {msg}", flush=True)
                            self.status.update(motors="error", base="error", motor_error=msg)
                            last_error = msg
                time.sleep(0.05)
                continue

            try:
                self._write_base_velocity()
                self._write_joint_targets()
                self._publish_motor_status()
            except Exception as exc:
                print(f"[motors] command loop failed: {exc}", flush=True)
                self.status.update(motors="error", motor_error=str(exc), base="error")
                self._disconnect_buses()
                self._last_base_written = None
            self._cmd_event.wait(delay)
            self._cmd_event.clear()

    def _write_base_velocity(self) -> None:
        linear_x, linear_y, angular_z = self._commanded_twist()
        left_raw, back_raw, right_raw = self._twist_to_raw(linear_x, linear_y, angular_z)
        triple = (left_raw, back_raw, right_raw)
        if triple == self._last_base_written:
            return
        if self.dry_run:
            self._last_base_written = triple
            return
        with self._bus_lock:
            if not self._base_ready or self._left_base_bus is None:
                return
            self._left_base_bus.sync_write(
                "Goal_Velocity",
                {
                    "base_left_wheel": left_raw,
                    "base_back_wheel": back_raw,
                    "base_right_wheel": right_raw,
                },
                normalize=False,
            )
            self._last_base_written = triple

    def _write_joint_targets(self) -> None:
        with self._joint_lock:
            targets = list(self._joint_targets)
        left_values: dict[str, int] = {}
        right_values: dict[str, int] = {}
        for name, target in zip(self.JOINT_NAMES, targets):
            if target is None or self._joint_written.get(name) == target:
                continue
            if name.startswith("left_hand_") and name in self._left_base_available:
                left_values[name] = target
            elif (name.startswith("right_hand_") or name.startswith("head_")) and name in self._right_head_available:
                right_values[name] = target

        if self.dry_run:
            for name, target in {**left_values, **right_values}.items():
                self._joint_written[name] = target
            return
        with self._bus_lock:
            if left_values and self._left_base_bus is not None:
                self._left_base_bus.sync_write("Goal_Position", left_values, normalize=False)
                self._joint_written.update(left_values)
            if right_values and self._right_head_bus is not None:
                self._right_head_bus.sync_write("Goal_Position", right_values, normalize=False)
                self._joint_written.update(right_values)

    def _publish_motor_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_at < 1.0:
            return
        self._last_status_at = now
        with self._cmd_lock:
            cmd_age = now - self._last_cmd_at if self._last_cmd_at else math.inf
        with self._joint_lock:
            joint_age = now - self._last_joint_target_at if self._last_joint_target_at else math.inf
        self.status.update(
            motors="connected",
            base="connected" if self._base_ready else "error",
            base_ready=self._base_ready,
            left_base_detected_ids=self._left_base_detected_ids,
            left_base_missing_ids=self._left_base_missing_ids,
            right_head_detected_ids=self._right_head_detected_ids,
            right_head_missing_ids=self._right_head_missing_ids,
            last_cmd_age_s=round(cmd_age, 3) if math.isfinite(cmd_age) else None,
            last_joint_target_age_s=round(joint_age, 3) if math.isfinite(joint_age) else None,
        )

    def _odom_loop(self) -> None:
        delay = 1.0 / self.odom_rate_hz
        while not self.stop_event.is_set():
            now = time.monotonic()
            dt = max(0.0, min(now - self._last_odom_at, 0.2))
            self._last_odom_at = now

            if self._connected and self.use_feedback_odom and not self.dry_run and self._base_ready:
                try:
                    with self._bus_lock:
                        assert self._left_base_bus is not None
                        raw = self._left_base_bus.sync_read(
                            "Present_Velocity",
                            list(self.BASE_WHEEL_NAMES),
                            normalize=False,
                        )
                    linear_x, linear_y, angular_z = self._raw_to_twist(
                        int(raw["base_left_wheel"]),
                        int(raw["base_back_wheel"]),
                        int(raw["base_right_wheel"]),
                    )
                except Exception:
                    linear_x, linear_y, angular_z = self._commanded_twist()
            else:
                linear_x, linear_y, angular_z = self._commanded_twist()

            if dt > 0.0:
                yaw_for_xy = self._yaw
                self._yaw = math.atan2(
                    math.sin(self._yaw + angular_z * dt),
                    math.cos(self._yaw + angular_z * dt),
                )
                self._x += (linear_x * math.cos(yaw_for_xy) - linear_y * math.sin(yaw_for_xy)) * dt
                self._y += (linear_x * math.sin(yaw_for_xy) + linear_y * math.cos(yaw_for_xy)) * dt
            self._publish_odom(linear_x, linear_y, angular_z)
            time.sleep(delay)

    def _joint_state_loop(self) -> None:
        delay = 1.0 / self.joint_state_rate_hz
        while not self.stop_event.is_set():
            self._publish_joint_states()
            time.sleep(delay)

    def _commanded_twist(self) -> tuple[float, float, float]:
        with self._cmd_lock:
            age = time.monotonic() - self._last_cmd_at if self._last_cmd_at else math.inf
            linear_x = self._linear_x
            linear_y = self._linear_y
            angular_z = self._angular_z
        if age > self.watchdog_timeout:
            return 0.0, 0.0, 0.0
        return linear_x, linear_y, angular_z

    def _publish_joint_states(self) -> None:
        positions: list[float] = []
        velocities: list[float] = []
        with self._bus_lock:
            for name in self.JOINT_STATE_NAMES:
                position = 0.0
                velocity = 0.0
                try:
                    if name in self._left_base_available and self._left_base_bus is not None:
                        if name in self.BASE_WHEEL_NAMES:
                            raw_velocity = self._left_base_bus.read(
                                "Present_Velocity", name, normalize=False, num_retry=1)
                            velocity = self._raw_to_radps(int(raw_velocity)) * self._wheel_signs.get(name, 1.0)
                        else:
                            position = float(self._left_base_bus.read(
                                "Present_Position", name, normalize=False, num_retry=1))
                    elif name in self._right_head_available and self._right_head_bus is not None:
                        position = float(self._right_head_bus.read(
                            "Present_Position", name, normalize=False, num_retry=1))
                except Exception:
                    pass
                positions.append(position)
                velocities.append(velocity)
        self._joint_state_pub.publish({
            "header": {"stamp": stamp(), "frame_id": self.base_frame},
            "name": list(self.JOINT_STATE_NAMES),
            "position": positions,
            "velocity": velocities,
            "effort": [0.0] * len(self.JOINT_STATE_NAMES),
        })

    def _publish_odom(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        if self._odom_pub is None:
            return
        quat = yaw_quat(self._yaw)
        self._odom_pub.publish({
            "header": {"stamp": stamp(), "frame_id": self.odom_frame},
            "child_frame_id": self.base_frame,
            "pose": {
                "pose": {
                    "position": {"x": self._x, "y": self._y, "z": 0.0},
                    "orientation": quat,
                },
                "covariance": [
                    0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.10,
                ],
            },
            "twist": {
                "twist": {
                    "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
                },
                "covariance": [
                    0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
                ],
            },
        })

    def _twist_to_raw(self, linear_x: float, linear_y: float, angular_z: float) -> tuple[int, int, int]:
        velocity_vector = np.array([linear_x, linear_y, angular_z], dtype=float)
        wheel_linear_speeds = self._omni_matrix.dot(velocity_vector)
        wheel_radps = wheel_linear_speeds / self.wheel_radius
        raw_values = np.array([self._radps_to_raw_raw(radps) for radps in wheel_radps], dtype=float)
        max_abs = float(np.max(np.abs(raw_values))) if raw_values.size else 0.0
        if max_abs > self.max_raw:
            raw_values *= float(self.max_raw) / max_abs
        signed = [
            int(round(raw_values[0] * self.left_sign)),
            int(round(raw_values[1] * self.back_sign)),
            int(round(raw_values[2] * self.right_sign)),
        ]
        return tuple(int(clamp(value, -self.max_raw, self.max_raw)) for value in signed)

    def _raw_to_twist(self, left_raw: int, back_raw: int, right_raw: int) -> tuple[float, float, float]:
        raw_values = np.array([
            float(left_raw) * self.left_sign,
            float(back_raw) * self.back_sign,
            float(right_raw) * self.right_sign,
        ])
        wheel_radps = np.array([self._raw_to_radps(int(raw)) for raw in raw_values])
        wheel_linear_speeds = wheel_radps * self.wheel_radius
        body = self._omni_inverse.dot(wheel_linear_speeds)
        return float(body[0]), float(body[1]), float(body[2])

    @staticmethod
    def _radps_to_raw_raw(radps: float) -> int:
        degps = radps * 180.0 / math.pi
        return int(round(degps * 4096.0 / 360.0))

    @staticmethod
    def _raw_to_radps(raw: int) -> float:
        degps = float(raw) / (4096.0 / 360.0)
        return degps * math.pi / 180.0

    def _safe_stop(self) -> None:
        if self.dry_run or self._left_base_bus is None:
            return
        available = {
            name: 0
            for name in self.BASE_WHEEL_NAMES
            if name in self._left_base_available
        }
        if not available:
            return
        try:
            self._left_base_bus.sync_write(
                "Goal_Velocity",
                available,
                normalize=False,
                num_retry=5,
            )
        except Exception:
            pass


class RPLidarPublisher:
    def __init__(self, client, stop_event: threading.Event, status: StatusPublisher, dry_run: bool):
        self.client = client
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self.topic = os.environ.get("SCAN_TOPIC", "/xlerobot/scan")
        self.serial_port = os.environ.get(
            "LIDAR_SERIAL",
            "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_12703f59806eef11ba3ee8c2c169b110-if00-port0",
        )
        self.baud = env_int("LIDAR_BAUD", 460800)
        self.frame = os.environ.get("LIDAR_FRAME", "base_link")
        self.samples = max(90, env_int("LIDAR_SAMPLES", 240))
        self.publish_rate_hz = max(0.2, env_float("LIDAR_PUBLISH_RATE_HZ", 8.0))
        self.angle_min = env_float("LIDAR_ANGLE_MIN", -math.pi)
        self.angle_max = env_float("LIDAR_ANGLE_MAX", math.pi)
        self.angle_offset = math.radians(env_float("LIDAR_ANGLE_OFFSET_DEG", 0.0))
        self.invert = env_bool("LIDAR_INVERT", False)
        self.range_min = env_float("LIDAR_RANGE_MIN", 0.12)
        self.range_max = env_float("LIDAR_RANGE_MAX", 12.0)
        self.min_quality = env_int("LIDAR_MIN_QUALITY", 0)
        self.min_points = env_int("LIDAR_MIN_POINTS_PER_ROTATION", 30)
        self.serial_timeout_s = env_float("LIDAR_SERIAL_TIMEOUT_S", 0.02)
        self.reconnect_delay_s = env_float("LIDAR_RECONNECT_DELAY_S", 1.0)
        self.fake_hz = max(0.2, env_float("LIDAR_FAKE_HZ", 5.0))

        self._pub = TopicPublisher(client, self.topic, "sensor_msgs/LaserScan")
        self._thread = threading.Thread(target=self._run, name="rplidar", daemon=True)
        self._ser = None
        self._last_scan_publish_at = 0.0

    def start(self) -> None:
        self._thread.start()
        mode = "dry-run" if self.dry_run else "serial"
        print(f"[lidar] started ({mode}), topic={self.topic}", flush=True)
        self.status.update(lidar="started", lidar_mode=mode, lidar_port=self.serial_port)

    def stop(self) -> None:
        try:
            self._send(CMD_STOP)
        except Exception:
            pass
        self._pub.close()

    def _run(self) -> None:
        if self.dry_run:
            self._run_fake()
            return
        if serial is None:
            msg = f"pyserial import failed: {SERIAL_IMPORT_ERROR}"
            print(f"[lidar] {msg}", flush=True)
            self.status.update(lidar="error", lidar_error=msg)
            return

        last_error = ""
        while not self.stop_event.is_set():
            try:
                print(f"[lidar] connecting {self.serial_port} @ {self.baud}", flush=True)
                with serial.Serial(
                    self.serial_port,
                    self.baud,
                    timeout=self.serial_timeout_s,
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
                    last_error = ""
                    self.status.update(lidar="connected")

                    current: list[tuple[float, float, int]] = []
                    buffer = bytearray()
                    rotation_started_at: Optional[float] = None

                    while not self.stop_event.is_set():
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
                                if len(current) >= self.min_points:
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
                if msg != last_error:
                    print(f"[lidar] error: {msg}", flush=True)
                    self.status.update(lidar="error", lidar_error=msg)
                    last_error = msg
                time.sleep(self.reconnect_delay_s)
            finally:
                try:
                    self._send(CMD_STOP)
                except Exception:
                    pass
                self._ser = None

    def _run_fake(self) -> None:
        delay = 1.0 / self.fake_hz
        scan_time = delay
        while not self.stop_event.is_set():
            ranges = [4.0] * self.samples
            for i in range(self.samples):
                angle = self.angle_min + (self.angle_max - self.angle_min) * i / self.samples
                if abs(angle) < 0.35:
                    ranges[i] = 6.0
                elif abs(abs(angle) - math.pi * 0.5) < 0.3:
                    ranges[i] = 1.2
            self._publish_scan(ranges, [20.0] * self.samples, scan_time)
            time.sleep(delay)

    def _send(self, cmd: int, payload: bytes = b"") -> None:
        if self._ser is None:
            return
        self._ser.write(make_command(cmd, payload))
        self._ser.flush()

    def _read_descriptor(self, timeout_s: float = 1.0) -> Optional[dict[str, Any]]:
        deadline = time.monotonic() + timeout_s
        window = bytearray()
        while time.monotonic() < deadline and not self.stop_event.is_set():
            byte = self._ser.read(1)
            if not byte:
                continue
            window += byte
            while len(window) >= 2 and not (window[0] == 0xA5 and window[1] == 0x5A):
                del window[0]
            if len(window) == 7:
                raw_len = int.from_bytes(window[2:6], "little")
                return {
                    "size": raw_len & 0x3FFFFFFF,
                    "mode": (raw_len >> 30) & 0x03,
                    "type": window[6],
                    "raw": bytes(window),
                }
        return None

    def _read_exact(self, size: int, timeout_s: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while len(data) < size and time.monotonic() < deadline and not self.stop_event.is_set():
            chunk = self._ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def _command_response(self, cmd: int, timeout_s: float = 1.0) -> tuple[Optional[dict[str, Any]], bytes]:
        self._ser.reset_input_buffer()
        self._send(cmd)
        descriptor = self._read_descriptor(timeout_s)
        if not descriptor:
            return None, b""
        return descriptor, self._read_exact(int(descriptor["size"]), timeout_s)

    def _read_info(self) -> None:
        descriptor, data = self._command_response(CMD_GET_INFO, 1.0)
        if not descriptor or len(data) != 20:
            return
        print(
            "[lidar] info: "
            f"model=0x{data[0]:02x}, firmware={data[2]}.{data[1]}, "
            f"hardware=0x{data[3]:02x}, serial={data[4:].hex()}",
            flush=True,
        )

    def _read_health(self) -> None:
        descriptor, data = self._command_response(CMD_GET_HEALTH, 1.0)
        if not descriptor or len(data) != 3:
            return
        labels = {0: "good", 1: "warning", 2: "error"}
        error_code = data[1] | (data[2] << 8)
        print(
            f"[lidar] health: {labels.get(data[0], str(data[0]))}, error_code={error_code}",
            flush=True,
        )

    def _start_scan(self) -> None:
        self._ser.reset_input_buffer()
        self._send(CMD_SCAN)
        descriptor = self._read_descriptor(1.0)
        if not descriptor:
            raise RuntimeError("scan descriptor timeout")
        if descriptor["size"] != 5 or descriptor["type"] != 0x81:
            raise RuntimeError(
                f"unexpected scan descriptor: size={descriptor['size']} "
                f"type=0x{descriptor['type']:02x}")

    def _publish_rotation(self, points: list[tuple[float, float, int]], scan_time: float) -> None:
        if self.angle_max <= self.angle_min:
            return
        now = time.monotonic()
        if now - self._last_scan_publish_at < 1.0 / self.publish_rate_hz:
            return
        self._last_scan_publish_at = now
        count = self.samples
        span = self.angle_max - self.angle_min
        angle_increment = span / float(count)
        ranges = [math.inf] * count
        intensities = [0.0] * count
        for angle_deg, distance_mm, quality in points:
            if quality < self.min_quality:
                continue
            distance_m = distance_mm / 1000.0
            if distance_m < self.range_min or distance_m > self.range_max:
                continue
            angle = math.radians(angle_deg) + self.angle_offset
            if self.invert:
                angle = -angle
            while angle < self.angle_min:
                angle += math.tau
            while angle >= self.angle_max:
                angle -= math.tau
            index = int((angle - self.angle_min) / angle_increment)
            if index < 0 or index >= count:
                continue
            if not math.isfinite(ranges[index]) or distance_m < ranges[index]:
                ranges[index] = distance_m
                intensities[index] = float(quality)
        self._publish_scan(ranges, intensities, scan_time)

    def _publish_scan(self, ranges: list[float], intensities: list[float], scan_time: float) -> None:
        count = len(ranges)
        angle_increment = (self.angle_max - self.angle_min) / float(count)
        self._pub.publish({
            "header": {"stamp": stamp(), "frame_id": self.frame},
            "angle_min": self.angle_min,
            "angle_max": self.angle_min + angle_increment * float(count - 1),
            "angle_increment": angle_increment,
            "time_increment": scan_time / float(count) if scan_time > 0.0 else 0.0,
            "scan_time": scan_time,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "ranges": ranges,
            "intensities": intensities,
        })


class CompressedCameraPublisher:
    def __init__(self, client, stop_event: threading.Event, status: StatusPublisher):
        self.client = client
        self.stop_event = stop_event
        self.status = status
        self.device = os.environ.get("CAMERA_DEVICE", "/dev/video0")
        self.image_topic = os.environ.get("CAMERA_TOPIC", "/xlerobot/base_camera/image/compressed")
        self.info_topic = os.environ.get("CAMERA_INFO_TOPIC", "/xlerobot/base_camera/camera_info")
        self.frame = os.environ.get("CAMERA_FRAME", "base_camera_optical_frame")
        self.width = env_int("CAMERA_WIDTH", 320)
        self.height = env_int("CAMERA_HEIGHT", 240)
        self.fps = max(0.2, env_float("CAMERA_RATE_HZ", env_float("CAMERA_FPS", 8.0)))
        self.quality = int(clamp(env_int("CAMERA_JPEG_QUALITY", 60), 10, 95))
        self.hfov_deg = env_float("CAMERA_HFOV_DEG", 70.0)
        self._image_pub = TopicPublisher(client, self.image_topic, "sensor_msgs/CompressedImage")
        self._info_pub = TopicPublisher(client, self.info_topic, "sensor_msgs/CameraInfo")
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)

    def start(self) -> None:
        self._thread.start()
        print(
            f"[camera] started, topic={self.image_topic}, "
            f"{self.width}x{self.height}@{self.fps}fps jpeg={self.quality}",
            flush=True,
        )
        self.status.update(
            camera="started",
            camera_device=self.device,
            camera_topic=self.image_topic,
            camera_info_topic=self.info_topic,
            camera_rate_hz=self.fps,
            camera_jpeg_quality=self.quality,
        )

    def stop(self) -> None:
        self._image_pub.close()
        self._info_pub.close()

    def _run(self) -> None:
        try:
            import cv2
        except Exception as exc:
            print(f"[camera] cv2 import failed: {exc}", flush=True)
            self.status.update(camera="error", camera_error=str(exc))
            return

        cap = cv2.VideoCapture(self.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not cap.isOpened():
            msg = f"camera open failed: {self.device}"
            print(f"[camera] {msg}", flush=True)
            self.status.update(camera="error", camera_error=msg)
            return

        delay = 1.0 / self.fps
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while not self.stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(delay)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, params)
            if not ok:
                time.sleep(delay)
                continue
            st = stamp()
            self._image_pub.publish({
                "header": {"stamp": st, "frame_id": self.frame},
                "format": "jpeg",
                "data": encoded.reshape(-1).tolist(),
            })
            self._info_pub.publish(self._camera_info(st))
            time.sleep(delay)
        cap.release()

    def _camera_info(self, st: dict[str, int]) -> dict[str, Any]:
        fx = self.width / (2.0 * math.tan(math.radians(self.hfov_deg) * 0.5))
        fy = fx
        cx = (self.width - 1.0) * 0.5
        cy = (self.height - 1.0) * 0.5
        return {
            "header": {"stamp": st, "frame_id": self.frame},
            "height": self.height,
            "width": self.width,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }


class RtspH264Publisher:
    def __init__(self, stop_event: threading.Event, status: StatusPublisher):
        self.stop_event = stop_event
        self.status = status
        self.url = os.environ.get("DEPTH_SENSOR_RTSP_URL", "").strip()
        self.fps = max(1.0, env_float(
            "DEPTH_SENSOR_RTSP_FPS", env_float("DEPTH_SENSOR_COLOR_FPS", 15.0)))
        self.bitrate_kbps = max(250, env_int("DEPTH_SENSOR_RTSP_BITRATE_KBPS", 3000))
        self.preset = os.environ.get("DEPTH_SENSOR_RTSP_X264_PRESET", "ultrafast")
        self.profile = os.environ.get("DEPTH_SENSOR_RTSP_H264_PROFILE", "baseline")
        self.transport = os.environ.get("DEPTH_SENSOR_RTSP_TRANSPORT", "tcp")
        self._queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._run, name="depth_sensor-rtsp", daemon=True)
        self._process: Optional[subprocess.Popen] = None
        self._started = False
        self._last_frame_at = 0.0
        self._last_error_log_at = 0.0
        self._size: Optional[tuple[int, int]] = None

    @property
    def enabled(self) -> bool:
        return bool(self.url)

    def start(self) -> None:
        if not self.enabled:
            return
        if shutil.which("ffmpeg") is None:
            msg = "ffmpeg not found; depth sensor RTSP/H.264 video disabled"
            print(f"[depth_sensor-rtsp] {msg}", flush=True)
            self.status.update(depth_sensor_rtsp="error", depth_sensor_rtsp_error=msg)
            return
        self._started = True
        self._thread.start()
        self.status.update(
            depth_sensor_rtsp="starting",
            depth_sensor_rtsp_url=self.url,
            depth_sensor_rtsp_fps=self.fps,
            depth_sensor_rtsp_bitrate_kbps=self.bitrate_kbps,
        )
        print(
            f"[depth_sensor-rtsp] publishing H.264 to {self.url} "
            f"@{self.fps:g}fps {self.bitrate_kbps}kbps",
            flush=True,
        )

    def stop(self) -> None:
        if self._process is not None:
            try:
                if self._process.stdin:
                    self._process.stdin.close()
            except Exception:
                pass
            try:
                self._process.terminate()
            except Exception:
                pass

    def submit_bgr(self, image: np.ndarray) -> None:
        if not self._started or not self.enabled:
            return
        now = time.monotonic()
        if now - self._last_frame_at < 1.0 / self.fps:
            return
        self._last_frame_at = now
        frame = np.ascontiguousarray(image).copy()
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                frame = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            h, w = frame.shape[:2]
            size = (w, h)
            if self._process is None or self._process.poll() is not None or self._size != size:
                self._restart_ffmpeg(size)
            if self._process is None or self._process.stdin is None:
                continue
            try:
                self._process.stdin.write(frame.tobytes())
            except (BrokenPipeError, OSError) as exc:
                self._log_error(f"ffmpeg pipe failed: {exc}")
                self._stop_process()
                time.sleep(0.5)

    def _restart_ffmpeg(self, size: tuple[int, int]) -> None:
        self._stop_process()
        width, height = size
        gop = max(1, int(round(self.fps)))
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            f"{self.fps:g}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            self.preset,
            "-tune",
            "zerolatency",
            "-profile:v",
            self.profile,
            "-pix_fmt",
            "yuv420p",
            "-g",
            str(gop),
            "-bf",
            "0",
            "-b:v",
            f"{self.bitrate_kbps}k",
            "-maxrate",
            f"{self.bitrate_kbps}k",
            "-bufsize",
            f"{max(self.bitrate_kbps // 2, 250)}k",
            "-f",
            "rtsp",
            "-rtsp_transport",
            self.transport,
            self.url,
        ]
        try:
            self._process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            self._size = size
            self.status.update(depth_sensor_rtsp="publishing", depth_sensor_rtsp_size=f"{width}x{height}")
            print(f"[depth_sensor-rtsp] ffmpeg started for {width}x{height}", flush=True)
        except Exception as exc:
            self._process = None
            self._size = None
            self._log_error(f"ffmpeg start failed: {exc}")

    def _stop_process(self) -> None:
        proc = self._process
        self._process = None
        self._size = None
        if proc is None:
            return
        try:
            if proc.stdin:
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.terminate()
        except Exception:
            pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_at > 2.0:
            print(f"[depth_sensor-rtsp] {msg}", flush=True)
            self.status.update(depth_sensor_rtsp="error", depth_sensor_rtsp_error=msg)
            self._last_error_log_at = now


class UsbCameraRtspProcess:
    def __init__(self, name: str, stop_event: threading.Event, status: StatusPublisher):
        self.name = name
        self.stop_event = stop_event
        self.status = status
        prefix = name.upper()
        self.enabled = env_bool(
            f"{prefix}_CAMERA_RTSP_ENABLE",
            env_bool("USB_CAMERA_RTSP_ENABLE", env_bool("ENABLE_USB_CAMERA_RTSP", False)),
        )
        self.device = os.environ.get(f"{prefix}_CAMERA_DEVICE", "").strip()
        self.url = os.environ.get(f"{prefix}_CAMERA_RTSP_URL", "").strip()
        self.width = env_int(f"{prefix}_CAMERA_WIDTH", env_int("USB_CAMERA_WIDTH", 640))
        self.height = env_int(f"{prefix}_CAMERA_HEIGHT", env_int("USB_CAMERA_HEIGHT", 480))
        self.fps = max(1.0, env_float(f"{prefix}_CAMERA_FPS", env_float("USB_CAMERA_FPS", 10.0)))
        self.input_format = os.environ.get(
            f"{prefix}_CAMERA_INPUT_FORMAT",
            os.environ.get("USB_CAMERA_INPUT_FORMAT", "mjpeg"),
        ).strip()
        self.bitrate_kbps = max(
            250,
            env_int(f"{prefix}_CAMERA_RTSP_BITRATE_KBPS", env_int("USB_CAMERA_RTSP_BITRATE_KBPS", 1500)),
        )
        self.transport = os.environ.get(f"{prefix}_CAMERA_RTSP_TRANSPORT", "tcp")
        self.preset = os.environ.get(f"{prefix}_CAMERA_RTSP_X264_PRESET", "ultrafast")
        self.profile = os.environ.get(f"{prefix}_CAMERA_RTSP_H264_PROFILE", "baseline")
        self.rotate_deg = env_int(f"{prefix}_CAMERA_ROTATE_DEG", env_int("USB_CAMERA_ROTATE_DEG", 0)) % 360
        self.video_filter = os.environ.get(f"{prefix}_CAMERA_FFMPEG_FILTER", "").strip()
        if not self.video_filter and self.rotate_deg == 180:
            self.video_filter = "hflip,vflip"
        self._thread = threading.Thread(target=self._run, name=f"{name}-camera-rtsp", daemon=True)
        self._process: Optional[subprocess.Popen] = None
        self._last_error_log_at = 0.0

    def start(self) -> None:
        if not self.enabled:
            return
        if not self.device or not self.url:
            self._log_error("disabled; missing device or RTSP URL")
            return
        if shutil.which("ffmpeg") is None:
            self._log_error("ffmpeg not found")
            return
        self._thread.start()
        self.status.update(**{
            f"{self.name}_camera_rtsp": "starting",
            f"{self.name}_camera_device": self.device,
            f"{self.name}_camera_rtsp_url": self.url,
        })
        print(
            f"[{self.name}_camera-rtsp] publishing {self.device} to {self.url} "
            f"@{self.width}x{self.height}/{self.fps:g}fps {self.bitrate_kbps}kbps "
            f"input={self.input_format or 'default'} rotate={self.rotate_deg}",
            flush=True,
        )

    def stop(self) -> None:
        self._stop_process()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            if not os.path.exists(self.device):
                self._log_error(f"device not found: {self.device}")
                time.sleep(1.0)
                continue
            self._start_ffmpeg()
            proc = self._process
            if proc is None:
                time.sleep(1.0)
                continue
            while not self.stop_event.is_set() and proc.poll() is None:
                time.sleep(0.5)
            if not self.stop_event.is_set():
                self._log_error("ffmpeg exited; restarting")
                self._stop_process()
                time.sleep(1.0)

    def _start_ffmpeg(self) -> None:
        self._stop_process()
        gop = max(1, int(round(self.fps)))
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "v4l2",
        ]
        if self.input_format:
            cmd.extend(["-input_format", self.input_format])
        cmd.extend([
            "-framerate",
            f"{self.fps:g}",
            "-video_size",
            f"{self.width}x{self.height}",
            "-i",
            self.device,
            "-an",
        ])
        if self.video_filter:
            cmd.extend(["-vf", self.video_filter])
        cmd.extend([
            "-c:v",
            "libx264",
            "-preset",
            self.preset,
            "-tune",
            "zerolatency",
            "-profile:v",
            self.profile,
            "-pix_fmt",
            "yuv420p",
            "-g",
            str(gop),
            "-bf",
            "0",
            "-b:v",
            f"{self.bitrate_kbps}k",
            "-maxrate",
            f"{self.bitrate_kbps}k",
            "-bufsize",
            f"{max(self.bitrate_kbps // 2, 250)}k",
            "-f",
            "rtsp",
            "-rtsp_transport",
            self.transport,
            self.url,
        ])
        try:
            self._process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL)
            self.status.update(**{f"{self.name}_camera_rtsp": "publishing"})
        except Exception as exc:
            self._process = None
            self._log_error(f"ffmpeg start failed: {exc}")

    def _stop_process(self) -> None:
        proc = self._process
        self._process = None
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_at > 2.0:
            print(f"[{self.name}_camera-rtsp] {msg}", flush=True)
            self.status.update(**{
                f"{self.name}_camera_rtsp": "error",
                f"{self.name}_camera_rtsp_error": msg,
            })
            self._last_error_log_at = now


class BinaryRgbdTcpPublisher:
    """Length-framed RGB-D side channel for ROS 2 consumers on the compute PC."""

    def __init__(self, stop_event: threading.Event, status: StatusPublisher):
        self.stop_event = stop_event
        self.status = status
        self.host = env_first(("DEPTH_SENSOR_BINARY_HOST", "COMPUTE_PC_HOST"), "").strip()
        self.port = env_int("DEPTH_SENSOR_BINARY_PORT", 9102)
        self.max_fps = max(0.0, env_float("DEPTH_SENSOR_BINARY_FPS", 0.0))
        self._queue: queue.Queue[tuple[dict[str, Any], bytes]] = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._run, name="depth_sensor-binary-rgbd", daemon=True)
        self._sock: Optional[socket.socket] = None
        self._started = False
        self._last_submit_at = 0.0
        self._last_error_log_at = 0.0
        self._frames_sent = 0
        self._bytes_sent = 0

    @property
    def enabled(self) -> bool:
        return bool(self.host) and self.port > 0

    def start(self) -> None:
        if not self.enabled:
            print("[depth_sensor-binary] disabled; missing DEPTH_SENSOR_BINARY_HOST", flush=True)
            self.status.update(depth_sensor_binary="disabled")
            return
        self._started = True
        self._thread.start()
        self.status.update(
            depth_sensor_binary="starting",
            depth_sensor_binary_target=f"{self.host}:{self.port}",
            depth_sensor_binary_max_fps=self.max_fps,
        )
        print(
            f"[depth_sensor-binary] publishing RGB-D to tcp://{self.host}:{self.port} "
            f"(max_fps={self.max_fps:g}, 0 means camera rate)",
            flush=True,
        )

    def stop(self) -> None:
        self._close_socket()

    def submit(self, header: dict[str, Any], payload: bytes) -> None:
        if not self._started or not self.enabled:
            return
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self._last_submit_at < 1.0 / self.max_fps:
            return
        self._last_submit_at = now
        item = (header, payload)
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                header, payload = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if self._sock is None and not self._connect():
                continue
            try:
                header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
                packet = struct.pack("!I", len(header_bytes)) + header_bytes + payload
                assert self._sock is not None
                self._sock.sendall(packet)
                self._frames_sent += 1
                self._bytes_sent += len(packet)
                if self._frames_sent % 150 == 0:
                    self.status.update(
                        depth_sensor_binary="publishing",
                        depth_sensor_binary_frames=self._frames_sent,
                        depth_sensor_binary_mb=round(self._bytes_sent / 1_000_000.0, 1),
                    )
            except Exception as exc:
                self._log_error(f"send failed: {exc}")
                self._close_socket()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)
            self._sock = sock
            self.status.update(depth_sensor_binary="connected")
            print(f"[depth_sensor-binary] connected to tcp://{self.host}:{self.port}", flush=True)
            return True
        except Exception as exc:
            self._log_error(f"connect failed: {exc}")
            self._close_socket()
            time.sleep(0.5)
            return False

    def _close_socket(self) -> None:
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_at > 2.0:
            print(f"[depth_sensor-binary] {msg}", flush=True)
            self.status.update(depth_sensor_binary="error", depth_sensor_binary_error=msg)
            self._last_error_log_at = now


class DepthSensorPublisher:
    def __init__(self, client, stop_event: threading.Event, status: StatusPublisher, dry_run: bool):
        self.client = client
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self.serial = os.environ.get("DEPTH_SENSOR_SERIAL", "").strip()
        self.enable_color = env_bool("DEPTH_SENSOR_ENABLE_COLOR", True)
        self.enable_imu = env_bool("DEPTH_SENSOR_ENABLE_IMU", True)
        self.align_depth_to_color = env_bool("DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR", True)
        self.depth_width = env_int("DEPTH_SENSOR_DEPTH_WIDTH", 640)
        self.depth_height = env_int("DEPTH_SENSOR_DEPTH_HEIGHT", 360)
        self.depth_fps = env_int("DEPTH_SENSOR_DEPTH_FPS", 15)
        self.color_width = env_int("DEPTH_SENSOR_COLOR_WIDTH", 640)
        self.color_height = env_int("DEPTH_SENSOR_COLOR_HEIGHT", 360)
        self.color_fps = env_int("DEPTH_SENSOR_COLOR_FPS", 15)
        self.require_usb3 = env_bool("DEPTH_SENSOR_REQUIRE_USB3", False)
        self.depth_publish_hz = max(
            0.0,
            env_float("DEPTH_SENSOR_DEPTH_PUBLISH_HZ", self.depth_fps),
        )
        self.color_publish_hz = max(
            0.0,
            env_float("DEPTH_SENSOR_COLOR_PUBLISH_HZ", self.color_fps),
        )
        self.imu_rate_hz = max(1.0, env_float("DEPTH_SENSOR_IMU_PUBLISH_HZ", 100.0))
        self.png_compress_level = int(clamp(env_int("DEPTH_SENSOR_PNG_COMPRESS_LEVEL", 1), 0, 9))
        self.jpeg_quality = int(clamp(env_int("DEPTH_SENSOR_JPEG_QUALITY", 60), 10, 95))
        self.binary_enabled = env_bool("DEPTH_SENSOR_BINARY_ENABLE", False)
        self.rosbridge_image_enable = env_bool(
            "DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE", not self.binary_enabled)
        self.binary_depth_format = os.environ.get(
            "DEPTH_SENSOR_BINARY_DEPTH_FORMAT", "raw16").strip().lower()
        if self.binary_depth_format not in ("raw16", "png16"):
            self.binary_depth_format = "raw16"
        self.binary_color_mode = os.environ.get(
            "DEPTH_SENSOR_BINARY_COLOR_MODE", "bgr8").strip().lower()
        if self.binary_color_mode in ("gray", "grey", "grayscale", "mono"):
            self.binary_color_mode = "mono8"
        if self.binary_color_mode not in ("mono8", "bgr8"):
            self.binary_color_mode = "mono8"
        self.depth_topic = os.environ.get("DEPTH_SENSOR_DEPTH_TOPIC", "/xlerobot/head_camera/depth/image")
        self.depth_info_topic = os.environ.get(
            "DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC",
            "/xlerobot/head_camera/depth/camera_info",
        )
        self.color_topic = os.environ.get("DEPTH_SENSOR_COLOR_TOPIC", "/xlerobot/head_camera/color/image")
        self.color_info_topic = os.environ.get(
            "DEPTH_SENSOR_COLOR_CAMERA_INFO_TOPIC",
            "/xlerobot/head_camera/color/camera_info",
        )
        self.imu_topic = os.environ.get("DEPTH_SENSOR_IMU_TOPIC", "/xlerobot/head_camera/imu")
        self.depth_frame = os.environ.get(
            "DEPTH_SENSOR_DEPTH_FRAME", "head_camera_depth_optical_frame")
        self.color_frame = os.environ.get(
            "DEPTH_SENSOR_COLOR_FRAME", "head_camera_color_optical_frame")
        self.imu_frame = os.environ.get("DEPTH_SENSOR_IMU_FRAME", "head_camera_imu_frame")
        self._align = None
        self._depth_publish_frame = self.depth_frame

        self._depth_pub = None
        self._depth_info_pub = None
        self._color_pub = None
        self._color_info_pub = None
        if self.rosbridge_image_enable:
            self._depth_pub = TopicPublisher(client, self.depth_topic, "sensor_msgs/CompressedImage")
            self._depth_info_pub = TopicPublisher(
                client, self.depth_info_topic, "sensor_msgs/CameraInfo")
        if self.rosbridge_image_enable and self.enable_color:
            self._color_pub = TopicPublisher(client, self.color_topic, "sensor_msgs/CompressedImage")
            self._color_info_pub = TopicPublisher(client, self.color_info_topic, "sensor_msgs/CameraInfo")
        self._imu_pub = None
        if self.enable_imu:
            self._imu_pub = TopicPublisher(client, self.imu_topic, "sensor_msgs/Imu")
        self._pipeline = None
        self._thread = threading.Thread(target=self._run, name="depth_sensor", daemon=True)
        self._latest_accel: Optional[tuple[float, float, float]] = None
        self._latest_gyro: Optional[tuple[float, float, float]] = None
        self._last_imu_publish_at = 0.0
        self._last_depth_publish_at = 0.0
        self._last_color_publish_at = 0.0
        self._last_frame_error_log_at = 0.0
        self._last_rs_frame_at = 0.0
        self._color_active = False
        self._imu_active = False
        self._rtsp = None
        rtsp_enabled = env_bool("DEPTH_SENSOR_RTSP_ENABLE", False)
        if rtsp_enabled:
            self._rtsp = RtspH264Publisher(stop_event, status)
        self._binary = None
        if self.binary_enabled:
            self._binary = BinaryRgbdTcpPublisher(stop_event, status)

    def start(self) -> None:
        self._thread.start()
        print(
            f"[depth_sensor] started, depth={self.depth_topic}, imu={self.imu_topic}, "
            f"color={self.color_topic if self.enable_color else 'off'}, "
            f"rosbridge_images={self.rosbridge_image_enable}, "
            f"binary={self._binary.enabled if self._binary is not None else False}",
            flush=True,
        )
        self.status.update(
            depth_sensor="started",
            depth_sensor_depth_topic=self.depth_topic,
            depth_sensor_depth_info_topic=self.depth_info_topic,
            depth_sensor_imu_topic=self.imu_topic if self.enable_imu else "",
            depth_sensor_color_topic=self.color_topic if self.enable_color else "",
            depth_sensor_depth_aligned_to_color=self.align_depth_to_color,
            depth_sensor_rtsp="enabled" if self._rtsp is not None else "disabled",
            depth_sensor_rosbridge_images=self.rosbridge_image_enable,
            depth_sensor_binary="enabled" if self._binary is not None else "disabled",
            depth_sensor_binary_color_mode=self.binary_color_mode,
        )
        if self._rtsp is not None:
            self._rtsp.start()
        if self._binary is not None:
            self._binary.start()

    def stop(self) -> None:
        if self._rtsp is not None:
            self._rtsp.stop()
        if self._binary is not None:
            self._binary.stop()
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        for pub in (
            self._depth_pub,
            self._depth_info_pub,
            self._color_pub,
            self._color_info_pub,
            self._imu_pub,
        ):
            if pub is not None:
                pub.close()

    def _run(self) -> None:
        if self.dry_run:
            self._run_fake()
            return
        if rs is None:
            msg = f"pyrealsense2 import failed: {DEPTH_SENSOR_IMPORT_ERROR}"
            print(f"[depth_sensor] {msg}", flush=True)
            self.status.update(depth_sensor="error", depth_sensor_error=msg)
            return

        last_error = ""
        reconnect_delay_s = max(0.5, env_float("DEPTH_SENSOR_RECONNECT_DELAY_S", 2.0))
        while not self.stop_event.is_set():
            try:
                self._start_pipeline()
                last_error = ""
                self.status.update(
                    depth_sensor="connected",
                    depth_sensor_color=self._color_active,
                    depth_sensor_imu=self._imu_active,
                    depth_sensor_depth_aligned_to_color=self._align is not None,
                )
                while not self.stop_event.is_set():
                    time.sleep(0.2)
                    if self._last_rs_frame_at and time.monotonic() - self._last_rs_frame_at > 5.0:
                        now = time.monotonic()
                        if now - self._last_frame_error_log_at > 2.0:
                            print("[depth_sensor] no frames for 5s; restarting pipeline", flush=True)
                            self.status.update(depth_sensor_frame_error="no frames for 5s")
                            self._last_frame_error_log_at = now
                        raise RuntimeError("depth sensor produced no frames for 5s")
            except Exception as exc:
                msg = str(exc)
                if msg != last_error:
                    print(f"[depth_sensor] error: {msg}", flush=True)
                    self.status.update(depth_sensor="error", depth_sensor_error=msg)
                    last_error = msg
            finally:
                if self._pipeline is not None:
                    try:
                        self._pipeline.stop()
                    except Exception:
                        pass
                    self._pipeline = None
            if not self.stop_event.is_set():
                time.sleep(reconnect_delay_s)

    def _start_pipeline(self) -> None:
        attempts = [
            (self.enable_color, self.enable_imu),
            (self.enable_color, False),
            (False, self.enable_imu),
            (False, False),
        ]
        errors = []
        for color_enabled, imu_enabled in attempts:
            pipeline = rs.pipeline()
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.depth_fps,
            )
            if color_enabled:
                config.enable_stream(
                    rs.stream.color,
                    self.color_width,
                    self.color_height,
                    rs.format.bgr8,
                    self.color_fps,
                )
            if imu_enabled:
                config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
                config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
            try:
                self._align = (
                    rs.align(rs.stream.color)
                    if color_enabled and self.align_depth_to_color
                    else None
                )
                self._depth_publish_frame = (
                    self.color_frame if self._align is not None else self.depth_frame
                )
                profile = pipeline.start(config, self._on_rs_frame)
            except Exception as exc:
                errors.append(f"color={color_enabled} imu={imu_enabled}: {exc}")
                try:
                    pipeline.stop()
                except Exception:
                    pass
                self._align = None
                self._depth_publish_frame = self.depth_frame
                continue
            usb_type = self._usb_type_from_profile(profile)
            if self.require_usb3 and usb_type and not usb_type.startswith("3"):
                try:
                    pipeline.stop()
                except Exception:
                    pass
                errors.append(
                    f"color={color_enabled} imu={imu_enabled}: "
                    f"USB3 required, but sensor negotiated USB {usb_type}"
                )
                self._align = None
                self._depth_publish_frame = self.depth_frame
                continue
            self._pipeline = pipeline
            self._last_rs_frame_at = time.monotonic()
            self._color_active = color_enabled
            self._imu_active = imu_enabled
            self.status.update(depth_sensor_usb_type=usb_type or "unknown")
            print(
                f"[depth_sensor] pipeline connected "
                f"(color={color_enabled}, imu={imu_enabled}, "
                f"align_depth_to_color={self._align is not None}, "
                f"usb={usb_type or 'unknown'}, require_usb3={self.require_usb3})",
                flush=True,
            )
            if self.enable_color and not color_enabled:
                print("[depth_sensor] color stream unavailable; continuing with depth/IMU", flush=True)
            if self.enable_imu and not imu_enabled:
                print("[depth_sensor] IMU stream unavailable; continuing with depth only", flush=True)
            return
        raise RuntimeError("could not start depth sensor pipeline; " + " | ".join(errors))

    def _usb_type_from_profile(self, profile) -> str:
        try:
            device = profile.get_device()
            return str(device.get_info(rs.camera_info.usb_type_descriptor))
        except Exception:
            return ""

    def _on_rs_frame(self, frame) -> None:
        try:
            self._last_rs_frame_at = time.monotonic()
            if frame.is_frameset():
                frameset = frame.as_frameset()
                if self._align is not None:
                    frameset = self._align.process(frameset)
                depth_frame = frameset.get_depth_frame()
                color_frame = frameset.get_color_frame()
                if color_frame and self._rtsp is not None:
                    self._submit_color_rtsp(color_frame)
                if depth_frame and color_frame and self._binary is not None:
                    self._publish_binary_rgbd(depth_frame, color_frame)
                if depth_frame and self.rosbridge_image_enable:
                    self._publish_depth(depth_frame)
                if color_frame and self._color_pub is not None:
                    self._publish_color(color_frame)
            elif frame.is_motion_frame():
                self._publish_motion(frame.as_motion_frame())
        except Exception as exc:
            print(f"[depth_sensor] frame handling failed: {exc}", flush=True)

    def _submit_color_rtsp(self, frame) -> None:
        if self._rtsp is None:
            return
        image = np.asanyarray(frame.get_data())
        self._rtsp.submit_bgr(image)

    def _publish_depth(self, frame) -> None:
        if self._depth_pub is None or self._depth_info_pub is None:
            return
        if not self._should_publish("depth"):
            return
        try:
            import cv2
        except Exception as exc:
            print(f"[depth_sensor] cv2 import failed for depth PNG: {exc}", flush=True)
            return
        depth = np.asanyarray(frame.get_data())
        ok, encoded = cv2.imencode(
            ".png",
            depth,
            [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_compress_level],
        )
        if not ok:
            return
        st = stamp()
        self._depth_pub.publish({
            "header": {"stamp": st, "frame_id": self._depth_publish_frame},
            "format": "png; 16UC1",
            "data": encoded.reshape(-1).tolist(),
        })
        self._depth_info_pub.publish(
            self._camera_info_from_video_frame(frame, st, self._depth_publish_frame))

    def _publish_color(self, frame) -> None:
        if self._color_pub is None or self._color_info_pub is None:
            return
        if not self._should_publish("color"):
            return
        try:
            import cv2
        except Exception as exc:
            print(f"[depth_sensor] cv2 import failed for color JPEG: {exc}", flush=True)
            return
        image = np.asanyarray(frame.get_data())
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        st = stamp()
        self._color_pub.publish({
            "header": {"stamp": st, "frame_id": self.color_frame},
            "format": "jpeg",
            "data": encoded.reshape(-1).tolist(),
        })
        self._color_info_pub.publish(
            self._camera_info_from_video_frame(frame, st, self.color_frame))

    def _publish_binary_rgbd(self, depth_frame, color_frame) -> None:
        if self._binary is None:
            return
        try:
            import cv2
        except Exception as exc:
            print(f"[depth_sensor] cv2 import failed for binary RGB-D: {exc}", flush=True)
            return

        st = stamp()
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        if self.binary_color_mode == "mono8":
            color_payload = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            color_encoding = "mono8"
        else:
            color_payload = color
            color_encoding = "bgr8"
        ok, color_encoded = cv2.imencode(
            ".jpg",
            color_payload,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return

        depth = np.ascontiguousarray(depth.astype(np.uint16, copy=False))
        if self.binary_depth_format == "png16":
            ok, depth_encoded = cv2.imencode(
                ".png",
                depth,
                [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_compress_level],
            )
            if not ok:
                return
            depth_bytes = depth_encoded.tobytes()
            depth_format = "png;16UC1"
        else:
            depth_bytes = depth.tobytes(order="C")
            depth_format = "raw16uc1-le"

        color_bytes = color_encoded.tobytes()
        depth_units = 0.001
        try:
            depth_units = float(depth_frame.get_units())
        except Exception:
            pass
        header = {
            "type": "rgbd",
            "stamp": st,
            "color_format": "jpeg",
            "color_len": len(color_bytes),
            "color_encoding": color_encoding,
            "color_frame_id": self.color_frame,
            "depth_format": depth_format,
            "depth_len": len(depth_bytes),
            "depth_encoding": "16UC1",
            "depth_frame_id": self._depth_publish_frame,
            "depth_width": int(depth.shape[1]),
            "depth_height": int(depth.shape[0]),
            "depth_step": int(depth.shape[1] * 2),
            "depth_units": depth_units,
            "color_camera_info": self._camera_info_from_video_frame(
                color_frame, st, self.color_frame),
            "depth_camera_info": self._camera_info_from_video_frame(
                depth_frame, st, self._depth_publish_frame),
        }
        self._binary.submit(header, color_bytes + depth_bytes)

    def _should_publish(self, stream: str) -> bool:
        if stream == "depth":
            hz = self.depth_publish_hz
            if hz <= 0:
                return False
            last = self._last_depth_publish_at
        else:
            hz = self.color_publish_hz
            if hz <= 0:
                return False
            last = self._last_color_publish_at

        now = time.monotonic()
        if last and now - last < 1.0 / hz:
            return False
        if stream == "depth":
            self._last_depth_publish_at = now
        else:
            self._last_color_publish_at = now
        return True

    def _publish_motion(self, frame) -> None:
        if self._imu_pub is None:
            return
        data = frame.get_motion_data()
        values = (float(data.x), float(data.y), float(data.z))
        stream_type = frame.get_profile().stream_type()
        if stream_type == rs.stream.accel:
            self._latest_accel = values
        elif stream_type == rs.stream.gyro:
            self._latest_gyro = values
        now = time.monotonic()
        if now - self._last_imu_publish_at < 1.0 / self.imu_rate_hz:
            return
        self._last_imu_publish_at = now
        accel = self._latest_accel or (0.0, 0.0, 0.0)
        gyro = self._latest_gyro or (0.0, 0.0, 0.0)
        self._imu_pub.publish({
            "header": {"stamp": stamp(), "frame_id": self.imu_frame},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "orientation_covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "angular_velocity": {"x": gyro[0], "y": gyro[1], "z": gyro[2]},
            "angular_velocity_covariance": [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ],
            "linear_acceleration": {"x": accel[0], "y": accel[1], "z": accel[2]},
            "linear_acceleration_covariance": [
                0.10, 0.0, 0.0,
                0.0, 0.10, 0.0,
                0.0, 0.0, 0.10,
            ],
        })

    def _camera_info_from_video_frame(self, frame, st: dict[str, int], frame_id: str) -> dict[str, Any]:
        intr = frame.profile.as_video_stream_profile().get_intrinsics()
        return {
            "header": {"stamp": st, "frame_id": frame_id},
            "height": int(intr.height),
            "width": int(intr.width),
            "distortion_model": "plumb_bob",
            "d": [float(x) for x in intr.coeffs[:5]],
            "k": [
                float(intr.fx), 0.0, float(intr.ppx),
                0.0, float(intr.fy), float(intr.ppy),
                0.0, 0.0, 1.0,
            ],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [
                float(intr.fx), 0.0, float(intr.ppx), 0.0,
                0.0, float(intr.fy), float(intr.ppy), 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        }

    def _run_fake(self) -> None:
        try:
            import cv2
        except Exception:
            cv2 = None
        delay = 1.0 / max(0.2, float(self.depth_fps))
        while not self.stop_event.is_set():
            st = stamp()
            depth = np.full((self.depth_height, self.depth_width), 1500, dtype=np.uint16)
            if cv2 is not None and self._depth_pub is not None:
                ok, encoded = cv2.imencode(".png", depth)
                if ok:
                    self._depth_pub.publish({
                        "header": {"stamp": st, "frame_id": self.depth_frame},
                        "format": "png; 16UC1",
                        "data": encoded.reshape(-1).tolist(),
                    })
            if self._depth_info_pub is not None:
                self._depth_info_pub.publish(self._fake_camera_info(st, self.depth_frame))
            if self._imu_pub is not None:
                self._imu_pub.publish({
                    "header": {"stamp": st, "frame_id": self.imu_frame},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "orientation_covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular_velocity_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
                    "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.80665},
                    "linear_acceleration_covariance": [0.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.10],
                })
            time.sleep(delay)

    def _fake_camera_info(self, st: dict[str, int], frame_id: str) -> dict[str, Any]:
        fx = self.depth_width / (2.0 * math.tan(math.radians(86.0) * 0.5))
        fy = fx
        cx = (self.depth_width - 1.0) * 0.5
        cy = (self.depth_height - 1.0) * 0.5
        return {
            "header": {"stamp": st, "frame_id": frame_id},
            "height": self.depth_height,
            "width": self.depth_width,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }


class DepthSensorProcess:
    """Run RealSense capture out-of-process so USB stalls cannot freeze motor I/O."""

    def __init__(
        self,
        host: str,
        port: int,
        stop_event: threading.Event,
        status: StatusPublisher,
        dry_run: bool,
    ):
        self.host = host
        self.port = port
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self._process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        env = os.environ.copy()
        env.update({
            "ENABLE_BASE": "0",
            "ENABLE_LIDAR": "0",
            "ENABLE_CAMERA": "0",
            "ENABLE_USB_CAMERA_RTSP": "0",
            "ENABLE_DEPTH_SENSOR": "1",
            "DEPTH_SENSOR_SEPARATE_PROCESS": "0",
            "DEPTH_SENSOR_CHILD": "1",
        })
        cmd = [
            sys.executable,
            os.path.abspath(__file__),
            "--host",
            self.host,
            "--port",
            str(self.port),
        ]
        if self.dry_run:
            cmd.append("--dry-run")
        self._process = subprocess.Popen(cmd, env=env)
        self.status.update(
            depth_sensor_process="started",
            depth_sensor_process_pid=self._process.pid,
        )
        print(f"[depth_sensor] child process pid={self._process.pid}", flush=True)

    def stop(self) -> None:
        proc = self._process
        self._process = None
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="XLeRobot hardware I/O over rosbridge")
    parser.add_argument("--rosbridge-uri", default=os.environ.get("ROSBRIDGE_URI", ""))
    parser.add_argument("--host", default=os.environ.get("ROSBRIDGE_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=env_int("ROSBRIDGE_PORT", 9090))
    parser.add_argument("--dry-run", action="store_true", default=env_bool("DRY_RUN", False))
    return parser.parse_args()


def rosbridge_target(args: argparse.Namespace) -> tuple[str, int]:
    if args.rosbridge_uri:
        parsed = urlparse(args.rosbridge_uri)
        if not parsed.hostname:
            raise SystemExit(f"invalid ROSBRIDGE_URI: {args.rosbridge_uri}")
        return parsed.hostname, parsed.port or 9090
    return args.host, args.port


def main() -> int:
    if roslibpy is None:
        print(f"[err] roslibpy import failed: {ROSLIBPY_IMPORT_ERROR}", file=sys.stderr)
        print("      install with: python -m pip install roslibpy", file=sys.stderr)
        return 1

    args = parse_args()
    host, port = rosbridge_target(args)
    stop_event = threading.Event()

    def handle_signal(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    enable_base = env_bool("ENABLE_BASE", True)
    enable_lidar = env_bool("ENABLE_LIDAR", True)
    enable_camera = env_bool("ENABLE_CAMERA", False)
    enable_usb_camera_rtsp = env_bool("ENABLE_USB_CAMERA_RTSP", False)
    enable_depth_sensor = env_bool("ENABLE_DEPTH_SENSOR", True)
    separate_depth_sensor = (
        enable_depth_sensor
        and env_bool("DEPTH_SENSOR_SEPARATE_PROCESS", True)
        and not env_bool("DEPTH_SENSOR_CHILD", False)
    )
    status_topic = os.environ.get("STATUS_TOPIC", "/xlerobot/io_status")
    reconnect_delay_s = max(0.5, env_float("ROSBRIDGE_RECONNECT_DELAY_S", 2.0))

    print("============================================================", flush=True)
    print("XLeRobot rosbridge hardware I/O", flush=True)
    print("============================================================", flush=True)
    print(f"rosbridge : ws://{host}:{port}", flush=True)
    print(f"dry run   : {args.dry_run}", flush=True)
    print(f"base      : {enable_base}", flush=True)
    print(f"lidar     : {enable_lidar}", flush=True)
    print(f"depth     : {enable_depth_sensor}", flush=True)
    print(f"camera    : {enable_camera}", flush=True)
    print(f"usb video : {enable_usb_camera_rtsp}", flush=True)
    print("============================================================", flush=True)

    while not stop_event.is_set():
        client = roslibpy.Ros(host=host, port=port)
        components: list[Any] = []
        status = None
        try:
            print(f"[net] connecting to rosbridge ws://{host}:{port}", flush=True)
            client.run()
            for _ in range(50):
                if stop_event.is_set() or client.is_connected:
                    break
                time.sleep(0.1)
            if not client.is_connected:
                raise RuntimeError("rosbridge connection timeout")
            print("[net] connected", flush=True)
            status = StatusPublisher(client, status_topic)
            status.update(state="connected")

            if enable_base:
                base = RosbridgeBase(client, stop_event, status, args.dry_run)
                base.start()
                components.append(base)
            if enable_lidar:
                lidar = RPLidarPublisher(client, stop_event, status, args.dry_run)
                lidar.start()
                components.append(lidar)
            if enable_depth_sensor:
                if separate_depth_sensor:
                    depth_sensor = DepthSensorProcess(host, port, stop_event, status, args.dry_run)
                else:
                    depth_sensor = DepthSensorPublisher(client, stop_event, status, args.dry_run)
                depth_sensor.start()
                components.append(depth_sensor)
            if enable_camera:
                camera = CompressedCameraPublisher(client, stop_event, status)
                camera.start()
                components.append(camera)
            if enable_usb_camera_rtsp:
                names = [
                    item.strip()
                    for item in os.environ.get(
                        "USB_CAMERA_RTSP_CAMERAS",
                        "base,wrist_left,wrist_right",
                    ).split(",")
                    if item.strip()
                ]
                for name in names:
                    usb_camera = UsbCameraRtspProcess(name, stop_event, status)
                    usb_camera.start()
                    components.append(usb_camera)

            while not stop_event.is_set() and client.is_connected:
                time.sleep(0.5)
        except Exception as exc:
            if not stop_event.is_set():
                print(f"[net] rosbridge loop error: {exc}", flush=True)
        finally:
            for component in reversed(components):
                try:
                    component.stop()
                except Exception:
                    pass
            if status is not None:
                status.update(state="disconnecting")
                status.close()
            try:
                client.terminate()
            except Exception:
                pass
        if not stop_event.is_set():
            time.sleep(reconnect_delay_s)

    print("[exit] stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
