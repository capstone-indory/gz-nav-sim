from __future__ import annotations

import argparse
import math
import os
import signal
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import zmq

try:
    import zstandard
except ImportError:
    zstandard = None

from _client_common import (
    pack_command,
    push_socket,
    req_socket,
    rpc,
    sub_socket,
    unpack_payload,
    wrap_pi,
    yaw_from_xyzw,
)


STREAM_TOPIC_ARGS = (
    ("proprio", "proprio_rate_hz"),
    ("scan", "scan_rate_hz"),
    ("scan.mid", "scan_mid_rate_hz"),
    ("rgb.front", "rgb_rate_hz"),
    ("depth.front", "depth_rate_hz"),
)


@dataclass
class State:
    lock: threading.Lock = field(default_factory=threading.Lock)
    scan: Optional[dict] = None
    depth_front: Optional[dict] = None
    proprio: Optional[dict] = None
    topics: set[str] = field(default_factory=set)
    last_rx_ns: dict[str, int] = field(default_factory=dict)

    def put(self, topic: str, msg: dict) -> None:
        with self.lock:
            self.topics.add(topic)
            self.last_rx_ns[topic] = time.time_ns()
            if topic == "scan":
                self.scan = msg
            elif topic == "depth.front":
                self.depth_front = msg
            elif topic == "proprio":
                self.proprio = msg

    def snapshot(self) -> tuple[Optional[dict], Optional[dict], Optional[dict], set[str], dict[str, int]]:
        with self.lock:
            return self.scan, self.depth_front, self.proprio, set(self.topics), dict(self.last_rx_ns)


def sensor_reader(host: str, port: int, state: State, stop: threading.Event) -> None:
    with sub_socket(host, port, ["scan", "proprio", "rgb.", "depth."]) as sock:
        sock.setsockopt(zmq.RCVTIMEO, 500)
        while not stop.is_set():
            try:
                topic_b, payload = sock.recv_multipart()
            except zmq.Again:
                continue
            msg = unpack_payload(payload)
            if msg is None:
                continue
            state.put(topic_b.decode("utf-8", errors="replace"), msg)


def ranges_array(value) -> Optional[np.ndarray]:
    try:
        if isinstance(value, (bytes, bytearray, memoryview)):
            return np.frombuffer(bytes(value), dtype=np.float32).copy()
        return np.asarray(value, dtype=np.float32).reshape(-1).copy()
    except Exception:
        return None


def ranges_from_scan(msg: dict) -> tuple[np.ndarray, np.ndarray]:
    ranges = ranges_array(msg.get("ranges"))
    if ranges is None:
        return np.asarray([], dtype=np.float32), np.asarray([], dtype=np.float32)
    angles = np.linspace(
        float(msg["angle_min"]),
        float(msg["angle_max"]),
        ranges.size,
        endpoint=True,
        dtype=np.float32,
    )
    rmin = float(msg.get("range_min", 0.05))
    rmax = float(msg.get("range_max", 12.0))
    valid = np.isfinite(ranges) & (ranges > rmin) & (ranges < rmax)
    ranges[~valid] = np.inf
    return ranges, angles


def sector_min(ranges: np.ndarray, angles: np.ndarray, deg_min: float, deg_max: float) -> float:
    amin = math.radians(deg_min)
    amax = math.radians(deg_max)
    wrapped = (angles + math.pi) % (2.0 * math.pi) - math.pi
    if amin <= amax:
        mask = (wrapped >= amin) & (wrapped <= amax)
    else:
        mask = (wrapped >= amin) | (wrapped <= amax)
    if not mask.any():
        return math.inf
    vals = ranges[mask]
    finite = vals[np.isfinite(vals)]
    return float(finite.min()) if finite.size else math.inf


def sector_min_center(
    ranges: np.ndarray,
    angles: np.ndarray,
    center_rad: float,
    half_width_deg: float,
) -> float:
    rel = (angles - center_rad + math.pi) % (2.0 * math.pi) - math.pi
    mask = np.abs(rel) <= math.radians(half_width_deg)
    if not mask.any():
        return math.inf
    vals = ranges[mask]
    finite = vals[np.isfinite(vals)]
    return float(finite.min()) if finite.size else math.inf


def depth_u16_mm(msg: Optional[dict]) -> Optional[np.ndarray]:
    if msg is None or zstandard is None:
        return None
    if msg.get("encoding") != "u16_zstd":
        return None
    try:
        raw = zstandard.decompress(bytes(msg["data"]))
        width = int(msg["width"])
        height = int(msg["height"])
    except Exception:
        return None
    arr = np.frombuffer(raw, dtype=np.uint16)
    if arr.size != width * height:
        return None
    return arr.reshape(height, width)


def depth_roi_distance_m(
    msg: Optional[dict],
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    percentile: float = 12.0,
) -> float:
    arr = depth_u16_mm(msg)
    if arr is None or arr.size == 0:
        return math.inf
    h, w = arr.shape
    ix0 = max(0, min(w - 1, int(round(x0 * w))))
    ix1 = max(ix0 + 1, min(w, int(round(x1 * w))))
    iy0 = max(0, min(h - 1, int(round(y0 * h))))
    iy1 = max(iy0 + 1, min(h, int(round(y1 * h))))
    roi = arr[iy0:iy1, ix0:ix1]
    valid = roi[(roi > 50) & np.isfinite(roi)]
    if valid.size == 0:
        return math.inf
    return float(np.percentile(valid.astype(np.float32), percentile) * 0.001)


def depth_clearances(msg: Optional[dict]) -> tuple[float, float, float]:
    # Ignore the top/bottom of the image. The center band is the part that
    # most directly affects forward motion for a forward-facing head camera.
    left = depth_roi_distance_m(msg, 0.08, 0.38, 0.32, 0.78)
    front = depth_roi_distance_m(msg, 0.34, 0.66, 0.30, 0.80)
    right = depth_roi_distance_m(msg, 0.62, 0.92, 0.32, 0.78)
    return front, left, right


def clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def pose_xy_yaw(proprio: dict) -> tuple[float, float, float]:
    joint_pose = proprio.get("joint_vel_arm_sample")
    if isinstance(joint_pose, list) and len(joint_pose) >= 3:
        return float(joint_pose[0]), float(joint_pose[1]), float(joint_pose[2])
    pose = proprio.get("base_pose")
    if not isinstance(pose, list) or len(pose) < 7:
        raise ValueError("proprio.base_pose is missing or malformed")
    x, y = float(pose[0]), float(pose[1])
    yaw = yaw_from_xyzw(float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]))
    return x, y, yaw


def camera_to_root_cmd(
    camera_cmd: list[float],
    proprio: Optional[dict],
    camera_yaw_offset: float,
) -> list[float]:
    forward, left, wz = camera_cmd
    yaw = 0.0
    if proprio is not None:
        try:
            yaw = pose_xy_yaw(proprio)[2]
        except Exception:
            yaw = 0.0
    theta = yaw + camera_yaw_offset
    vx = forward * math.cos(theta) - left * math.sin(theta)
    vy = forward * math.sin(theta) + left * math.cos(theta)
    return [vx, vy, wz]


def root_to_camera_cmd(
    root_cmd: list[float],
    proprio: Optional[dict],
    camera_yaw_offset: float,
) -> list[float]:
    vx, vy, wz = root_cmd
    yaw = 0.0
    if proprio is not None:
        try:
            yaw = pose_xy_yaw(proprio)[2]
        except Exception:
            yaw = 0.0
    theta = yaw + camera_yaw_offset
    forward = vx * math.cos(theta) + vy * math.sin(theta)
    left = -vx * math.sin(theta) + vy * math.cos(theta)
    return [forward, left, wz]


def goal_command(
    proprio: dict,
    goal: tuple[float, float],
    max_v: float,
    max_w: float,
    pos_tol: float,
    camera_yaw_offset: float,
) -> tuple[list[float], bool, float]:
    x, y, yaw = pose_xy_yaw(proprio)
    dx = goal[0] - x
    dy = goal[1] - y
    dist = math.hypot(dx, dy)
    if dist <= pos_tol:
        return [0.0, 0.0, 0.0], True, dist

    heading_err = wrap_pi(math.atan2(dy, dx) - (yaw + camera_yaw_offset))
    vx = clip(0.45 * dx, max_v)
    vy = clip(0.45 * dy, max_v)
    wz = clip(1.2 * heading_err, max_w)
    return [vx, vy, wz], False, dist


def explore_command(
    scan: dict,
    depth_front: Optional[dict],
    cruise_v: float,
    max_w: float,
    depth_stop_distance: float,
    use_depth: bool,
    camera_yaw_offset: float,
) -> list[float]:
    ranges, angles = ranges_from_scan(scan)
    front = sector_min_center(ranges, angles, camera_yaw_offset, 28.0)
    left = sector_min_center(ranges, angles, camera_yaw_offset + math.pi * 0.5, 40.0)
    right = sector_min_center(ranges, angles, camera_yaw_offset - math.pi * 0.5, 40.0)
    if use_depth:
        dfront, dleft, dright = depth_clearances(depth_front)
        front = min(front, dfront)
        if math.isfinite(dleft):
            left = min(left, dleft)
        if math.isfinite(dright):
            right = min(right, dright)

    if front < depth_stop_distance:
        turn = max_w if left >= right else -max_w
        side = 0.16 if left >= right else -0.16
        return [-0.04, side, turn]

    balance = 0.0
    if math.isfinite(left) and math.isfinite(right):
        balance = clip(0.18 * (right - left), 0.25)
    return [cruise_v, balance, 0.25 * balance]


def depth_only_command(
    depth_front: Optional[dict],
    cruise_v: float,
    max_w: float,
    depth_stop_distance: float,
) -> list[float]:
    dfront, dleft, dright = depth_clearances(depth_front)
    if not math.isfinite(dfront):
        return [0.0, 0.0, 0.0]
    if dfront < depth_stop_distance:
        turn = max_w if dleft >= dright else -max_w
        side = 0.14 if dleft >= dright else -0.14
        return [-0.03, side, turn]
    balance = 0.0
    if math.isfinite(dleft) and math.isfinite(dright):
        balance = clip(0.16 * (dright - dleft), 0.22)
    return [cruise_v, balance, 0.20 * balance]


def apply_sensor_safety(
    cmd: list[float],
    scan: Optional[dict],
    depth_front: Optional[dict],
    stop_distance: float,
    depth_stop_distance: float,
    max_w: float,
    use_depth: bool,
    camera_yaw_offset: float,
) -> list[float]:
    if scan is None:
        if use_depth and depth_front is not None:
            dfront, dleft, dright = depth_clearances(depth_front)
            if not math.isfinite(dfront):
                return [0.0, 0.0, 0.0]
            if dfront >= depth_stop_distance:
                return cmd
            turn = max_w if dleft >= dright else -max_w
            side = 0.14 if dleft >= dright else -0.14
            return [min(cmd[0], 0.0), side, turn]
        return [0.0, 0.0, 0.0]
    ranges, angles = ranges_from_scan(scan)
    front = sector_min_center(ranges, angles, camera_yaw_offset, 25.0)
    left = sector_min_center(ranges, angles, camera_yaw_offset + math.pi * 0.5, 40.0)
    right = sector_min_center(ranges, angles, camera_yaw_offset - math.pi * 0.5, 40.0)
    stop = stop_distance
    if use_depth:
        dfront, dleft, dright = depth_clearances(depth_front)
        front = min(front, dfront)
        stop = max(stop_distance, depth_stop_distance)
        if math.isfinite(dleft):
            left = min(left, dleft)
        if math.isfinite(dright):
            right = min(right, dright)
    if front >= stop:
        return cmd
    turn = max_w if left >= right else -max_w
    side = 0.14 if left >= right else -0.14
    return [min(cmd[0], 0.0), side, turn]


def try_rpc_setup(args: argparse.Namespace) -> None:
    with req_socket(args.sim_host, args.rpc_port, args.rpc_timeout_ms) as sock:
        if args.reset:
            print("rpc reset:", rpc(sock, "reset"))
        if args.set_pose is not None:
            print("rpc set_pose:", rpc(sock, "set_pose", pose=args.set_pose))
        try:
            print("rpc topic_list:", rpc(sock, "topic_list"))
        except Exception as exc:
            print("rpc topic_list failed:", exc)


def enable_streams_once(args: argparse.Namespace) -> bool:
    ok = True
    for topic, attr in STREAM_TOPIC_ARGS:
        rate = float(getattr(args, attr))
        try:
            with req_socket(args.sim_host, args.rpc_port, args.rpc_timeout_ms) as sock:
                if rate <= 0.0:
                    resp = rpc(sock, "disable_stream", topic=topic)
                    print(f"rpc disable_stream {topic}:", resp)
                else:
                    resp = rpc(sock, "enable_stream", topic=topic, rate_hz=rate)
                    print(f"rpc enable_stream {topic}@{rate:g}Hz:", resp)
            if isinstance(resp, dict) and resp.get("ok") is False:
                ok = False
        except Exception as exc:
            print(f"rpc stream setup {topic}: failed: {exc}")
            ok = False
    try:
        with req_socket(args.sim_host, args.rpc_port, args.rpc_timeout_ms) as sock:
            print("rpc topic_list:", rpc(sock, "topic_list"))
    except Exception as exc:
        print("rpc topic_list failed:", exc)
    return ok


def stream_enable_loop(args: argparse.Namespace, stop: threading.Event) -> None:
    next_log = 0.0
    while not stop.is_set():
        try:
            if enable_streams_once(args):
                print("rpc streams enabled")
                return
        except Exception as exc:
            now = time.monotonic()
            if now >= next_log:
                print(f"rpc stream setup unavailable; retrying: {exc}")
                next_log = now + max(1.0, args.stream_retry_s)
        stop.wait(max(1.0, args.stream_retry_s))


def send_command(sock: zmq.Socket, arm: list[float], base: list[float]) -> bool:
    try:
        sock.send(pack_command(arm, base), flags=zmq.NOBLOCK)
        return True
    except zmq.Again:
        return False


def reset_local_viewer(url: str, timeout_s: float) -> None:
    try:
        body = urllib.request.urlopen(url, timeout=timeout_s).read().decode("utf-8", errors="replace")
        print(f"reset_local: {body}")
    except Exception as exc:
        print(f"reset_local skipped: {url} ({exc})")


def main() -> None:
    parser = argparse.ArgumentParser(description="Isaac Sim local navigation client over xlerobot_v1 ZMQ.")
    parser.add_argument("--sim-host", default="localhost")
    parser.add_argument("--pub-port", type=int, default=5555)
    parser.add_argument("--push-port", type=int, default=5556)
    parser.add_argument("--rpc-port", type=int, default=5557)
    parser.add_argument("--rpc-timeout-ms", type=int, default=1000)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--status-period-s", type=float, default=2.0)
    parser.add_argument("--mode", choices=("initialize", "explore", "goal"), default="initialize")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--pos-tol", type=float, default=0.25)
    parser.add_argument("--max-v", type=float, default=0.28)
    parser.add_argument("--max-w", type=float, default=0.75)
    parser.add_argument("--cruise-v", type=float, default=0.16)
    parser.add_argument("--initialize-cruise-v", type=float, default=0.13)
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=float(os.environ.get("NAV_SPEED_SCALE", os.environ.get("SPEED_SCALE", "1.0"))),
    )
    parser.add_argument("--stop-distance", type=float, default=0.62)
    parser.add_argument("--depth-stop-distance", type=float, default=0.70)
    parser.add_argument("--disable-depth-nav", action="store_true")
    parser.add_argument(
        "--camera-yaw-offset-rad",
        type=float,
        default=float(os.environ.get("CAMERA_YAW_OFFSET_RAD", "0.0")),
        help="Camera forward yaw relative to root +X. Default 0 means RGB-D faces root +X.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until Ctrl-C.")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--set-pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--enable-streams", action="store_true")
    parser.add_argument("--proprio-rate-hz", type=float, default=10.0)
    parser.add_argument("--rgb-rate-hz", type=float, default=5.0)
    parser.add_argument("--depth-rate-hz", type=float, default=3.0)
    parser.add_argument("--scan-rate-hz", type=float, default=5.0)
    parser.add_argument("--scan-mid-rate-hz", type=float, default=0.0)
    parser.add_argument(
        "--reset-local-url",
        default=os.environ.get("RESET_LOCAL_URL", f"http://127.0.0.1:{os.environ.get('HTTP_PORT', '18081')}/reset_local"),
    )
    parser.add_argument("--reset-local-timeout-s", type=float, default=0.8)
    parser.add_argument("--no-reset-local", action="store_true")
    parser.add_argument("--stream-retry-s", type=float, default=10.0)
    args = parser.parse_args()
    if args.speed_scale <= 0.0:
        parser.error("--speed-scale must be positive")
    args.max_v *= args.speed_scale
    args.max_w *= args.speed_scale
    args.cruise_v *= args.speed_scale
    args.initialize_cruise_v *= args.speed_scale

    if args.mode == "initialize":
        args.enable_streams = True

    if not args.no_reset_local:
        reset_local_viewer(args.reset_local_url, args.reset_local_timeout_s)

    if args.mode == "goal" and args.goal is None:
        parser.error("--mode goal requires --goal X Y")

    if args.reset or args.set_pose is not None:
        try:
            try_rpc_setup(args)
        except Exception as exc:
            print(f"rpc setup failed: {exc}")

    state = State()
    stop = threading.Event()
    thread = threading.Thread(target=sensor_reader, args=(args.sim_host, args.pub_port, state, stop), daemon=True)
    thread.start()

    def handle_signal(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    stream_thread = None
    if args.enable_streams:
        stream_thread = threading.Thread(target=stream_enable_loop, args=(args, stop), daemon=True)
        stream_thread.start()

    arm_home = [0.0] * 14
    period = 1.0 / max(args.rate_hz, 1e-3)
    start = time.monotonic()
    last_status = 0.0
    reached = False

    with push_socket(args.sim_host, args.push_port) as sock:
        while not stop.is_set():
            now = time.monotonic()
            if args.duration > 0.0 and now - start >= args.duration:
                break

            scan, depth_front, proprio, topics, last_rx = state.snapshot()
            cmd = [0.0, 0.0, 0.0]
            cmd_camera = [0.0, 0.0, 0.0]
            dist = math.nan
            if scan is not None:
                if args.mode == "goal" and proprio is not None:
                    cmd, reached, dist = goal_command(
                        proprio,
                        (float(args.goal[0]), float(args.goal[1])),
                        args.max_v,
                        args.max_w,
                        args.pos_tol,
                        args.camera_yaw_offset_rad,
                    )
                    cmd_camera = root_to_camera_cmd(cmd, proprio, args.camera_yaw_offset_rad)
                    cmd_camera = apply_sensor_safety(
                        cmd_camera,
                        scan,
                        depth_front,
                        args.stop_distance,
                        args.depth_stop_distance,
                        args.max_w,
                        not args.disable_depth_nav,
                        args.camera_yaw_offset_rad,
                    )
                    cmd = camera_to_root_cmd(cmd_camera, proprio, args.camera_yaw_offset_rad)
                elif args.mode in ("initialize", "explore"):
                    cruise_v = args.initialize_cruise_v if args.mode == "initialize" else args.cruise_v
                    cmd_camera = explore_command(
                        scan,
                        depth_front,
                        cruise_v,
                        args.max_w,
                        args.depth_stop_distance,
                        not args.disable_depth_nav,
                        args.camera_yaw_offset_rad,
                    )
                    cmd_camera = apply_sensor_safety(
                        cmd_camera,
                        scan,
                        depth_front,
                        args.stop_distance,
                        args.depth_stop_distance,
                        args.max_w,
                        not args.disable_depth_nav,
                        args.camera_yaw_offset_rad,
                    )
                    cmd = camera_to_root_cmd(cmd_camera, proprio, args.camera_yaw_offset_rad)
            elif depth_front is not None and not args.disable_depth_nav and args.mode in ("initialize", "explore"):
                cruise_v = args.initialize_cruise_v if args.mode == "initialize" else args.cruise_v
                cmd_camera = depth_only_command(depth_front, cruise_v, args.max_w, args.depth_stop_distance)
                cmd = camera_to_root_cmd(cmd_camera, proprio, args.camera_yaw_offset_rad)

            sent = send_command(sock, arm_home, cmd)

            if now - last_status >= max(0.2, args.status_period_s):
                scan_age = (time.time_ns() - last_rx.get("scan", 0)) / 1e9 if "scan" in last_rx else math.inf
                depth_age = (time.time_ns() - last_rx.get("depth.front", 0)) / 1e9 if "depth.front" in last_rx else math.inf
                prop_age = (time.time_ns() - last_rx.get("proprio", 0)) / 1e9 if "proprio" in last_rx else math.inf
                dfront, _, _ = depth_clearances(depth_front)
                print(
                    f"mode={args.mode} cmd=({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}) "
                    f"cam=({cmd_camera[0]:+.2f},{cmd_camera[1]:+.2f},{cmd_camera[2]:+.2f}) "
                    f"topics={','.join(sorted(topics)) or '-'} "
                    f"scan_age={scan_age:.2f}s depth_age={depth_age:.2f}s proprio_age={prop_age:.2f}s "
                    f"depth_front={dfront:.2f}m"
                    + (f" goal_dist={dist:.2f}m" if args.mode == "goal" and not math.isnan(dist) else "")
                    + (" sent=0" if not sent else "")
                )
                last_status = now
            if reached:
                break
            time.sleep(period)

        for _ in range(5):
            send_command(sock, arm_home, [0.0, 0.0, 0.0])
            time.sleep(period)

    stop.set()
    print("stopped")


if __name__ == "__main__":
    main()
