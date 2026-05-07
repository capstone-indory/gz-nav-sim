from __future__ import annotations

import contextlib
import math
import time
from typing import Iterable, Iterator

import msgpack
import zmq


SCHEMA = "xlerobot_v1"
ARM_DOF = 14
BASE_DOF = 3


@contextlib.contextmanager
def sub_socket(
    host: str,
    port: int = 5555,
    topics: Iterable[str] | None = None,
) -> Iterator[zmq.Socket]:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{host}:{port}")
    sock.setsockopt(zmq.RCVHWM, 8)
    for topic in (topics if topics is not None else [""]):
        sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
    try:
        yield sock
    finally:
        sock.close(linger=0)


@contextlib.contextmanager
def push_socket(host: str, port: int = 5556) -> Iterator[zmq.Socket]:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.SNDTIMEO, 250)
    sock.setsockopt(zmq.IMMEDIATE, 1)
    sock.connect(f"tcp://{host}:{port}")
    sock.setsockopt(zmq.SNDHWM, 4)
    try:
        yield sock
    finally:
        sock.close(linger=0)


@contextlib.contextmanager
def req_socket(host: str, port: int = 5557, timeout_ms: int = 1000) -> Iterator[zmq.Socket]:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.connect(f"tcp://{host}:{port}")
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    try:
        yield sock
    finally:
        sock.close(linger=0)


def pack_command(arm_pos: list[float], base_vel: list[float]) -> bytes:
    if len(arm_pos) != ARM_DOF:
        raise ValueError(f"arm_pos must have {ARM_DOF} values, got {len(arm_pos)}")
    if len(base_vel) != BASE_DOF:
        raise ValueError(f"base_vel must have {BASE_DOF} values, got {len(base_vel)}")
    msg = {
        "schema": SCHEMA,
        "stamp_ns": time.time_ns(),
        "arm_joint_pos_target": [float(v) for v in arm_pos],
        "base_cmd_vel": [float(v) for v in base_vel],
    }
    return msgpack.packb(msg, use_bin_type=True)


def rpc(sock: zmq.Socket, op: str, **kwargs) -> dict:
    msg = {"schema": SCHEMA, "op": op}
    msg.update(kwargs)
    sock.send(msgpack.packb(msg, use_bin_type=True))
    return msgpack.unpackb(sock.recv(), raw=False)


def unpack_payload(payload: bytes) -> dict | None:
    try:
        msg = msgpack.unpackb(payload, raw=False)
    except Exception:
        return None
    if msg.get("schema") != SCHEMA:
        return None
    return msg


def yaw_from_xyzw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
