from __future__ import annotations

import argparse
import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty
import urllib.request

import zmq

from _client_common import pack_command, push_socket, req_socket, rpc, sub_socket, unpack_payload, yaw_from_xyzw


HELP = """\
WASD teleop over Isaac ZMQ

  w/s : camera-forward/back
  a/d : camera-left/right strafe
  q/e : rotate left/right
  space : stop
  r : reset pose command to zero velocity only
  x or Ctrl-C : quit

The last commanded velocity is re-sent continuously until changed.
"""


def send(sock: zmq.Socket, arm_home: list[float], base: list[float]) -> bool:
    try:
        sock.send(pack_command(arm_home, base), flags=zmq.NOBLOCK)
        return True
    except zmq.Again:
        return False


def reset_local_viewer(url: str, timeout_s: float) -> None:
    try:
        body = urllib.request.urlopen(url, timeout=timeout_s).read().decode("utf-8", errors="replace")
        print(f"reset_local: {body}", flush=True)
    except Exception as exc:
        print(f"reset_local skipped: {url} ({exc})", flush=True)


def try_enable_streams(args: argparse.Namespace) -> None:
    topics = (
        ("proprio", args.proprio_rate_hz),
        ("rgb.front", args.rgb_rate_hz),
        ("depth.front", args.depth_rate_hz),
        ("scan", args.scan_rate_hz),
        ("scan.mid", args.scan_mid_rate_hz),
    )
    for topic, rate_hz in topics:
        try:
            with req_socket(args.sim_host, args.rpc_port, args.rpc_timeout_ms) as sock:
                if rate_hz <= 0.0:
                    resp = rpc(sock, "disable_stream", topic=topic)
                    print(f"disable_stream {topic}: {resp}", flush=True)
                else:
                    resp = rpc(sock, "enable_stream", topic=topic, rate_hz=rate_hz)
                    print(f"enable_stream {topic}@{rate_hz:g}Hz: {resp}", flush=True)
        except Exception as exc:
            print(f"stream setup {topic}: failed: {exc}", flush=True)


class YawStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._yaw: float | None = None
        self._source = "none"

    def put(self, yaw: float, source: str) -> None:
        with self._lock:
            self._yaw = float(yaw)
            self._source = source

    def get(self) -> tuple[float | None, str]:
        with self._lock:
            return self._yaw, self._source


def pose_yaw_from_proprio(msg: dict) -> tuple[float, str] | None:
    joint_pose = msg.get("joint_vel_arm_sample")
    if isinstance(joint_pose, list) and len(joint_pose) >= 3:
        return float(joint_pose[2]), "joint_vel_arm_sample"
    pose = msg.get("base_pose")
    if isinstance(pose, list) and len(pose) >= 7:
        return yaw_from_xyzw(float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6])), "base_pose"
    return None


def proprio_loop(args: argparse.Namespace, yaw_store: YawStore, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            with sub_socket(args.sim_host, args.pub_port, ["proprio"]) as sock:
                sock.setsockopt(zmq.RCVTIMEO, 500)
                while not stop.is_set():
                    try:
                        _topic, payload = sock.recv_multipart()
                    except zmq.Again:
                        continue
                    msg = unpack_payload(payload)
                    if msg is None:
                        continue
                    yaw = pose_yaw_from_proprio(msg)
                    if yaw is not None:
                        yaw_store.put(*yaw)
        except Exception:
            stop.wait(0.5)


def key_to_camera_cmd(key: str, args: argparse.Namespace) -> list[float] | None:
    vx = args.vx * args.speed_scale
    vy = args.vy * args.speed_scale
    wz = args.wz * args.speed_scale
    if key == "w":
        return [vx, 0.0, 0.0]
    if key == "s":
        return [-vx, 0.0, 0.0]
    if key == "a":
        return [0.0, vy, 0.0]
    if key == "d":
        return [0.0, -vy, 0.0]
    if key == "q":
        return [0.0, 0.0, wz]
    if key == "e":
        return [0.0, 0.0, -wz]
    if key in (" ", "r"):
        return [0.0, 0.0, 0.0]
    return None


def camera_to_root_cmd(camera_cmd: list[float], robot_yaw: float | None, camera_yaw_offset: float) -> list[float]:
    forward, left, wz = camera_cmd
    theta = (robot_yaw or 0.0) + camera_yaw_offset
    vx = forward * math.cos(theta) - left * math.sin(theta)
    vy = forward * math.sin(theta) + left * math.cos(theta)
    return [vx, vy, wz]


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual WASD teleop for xlerobot_v1 over Isaac ZMQ.")
    parser.add_argument("--sim-host", default=os.environ.get("SIM_HOST", "100.80.87.68"))
    parser.add_argument("--pub-port", type=int, default=5555)
    parser.add_argument("--push-port", type=int, default=5556)
    parser.add_argument("--rpc-port", type=int, default=5557)
    parser.add_argument("--rpc-timeout-ms", type=int, default=700)
    parser.add_argument("--rate-hz", type=float, default=float(os.environ.get("TELEOP_CMD_RATE_HZ", os.environ.get("CMD_RATE_HZ", "8.0"))))
    parser.add_argument("--vx", type=float, default=0.12, help="Forward/back velocity in m/s.")
    parser.add_argument("--vy", type=float, default=0.12, help="Left/right strafe velocity in m/s.")
    parser.add_argument("--wz", type=float, default=0.55, help="Yaw velocity in rad/s.")
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=float(os.environ.get("TELEOP_SPEED_SCALE", os.environ.get("SPEED_SCALE", "4.0"))),
    )
    parser.add_argument(
        "--camera-yaw-offset-rad",
        type=float,
        default=float(os.environ.get("CAMERA_YAW_OFFSET_RAD", "0.0")),
        help="Camera forward yaw relative to root +X. Default 0 means RGB-D faces root +X.",
    )
    parser.add_argument("--enable-streams", action=argparse.BooleanOptionalAction, default=True)
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
    args = parser.parse_args()
    if args.speed_scale <= 0.0:
        parser.error("--speed-scale must be positive")

    if not sys.stdin.isatty():
        raise SystemExit("wasd_teleop.py must be run from an interactive terminal.")

    if not args.no_reset_local:
        reset_local_viewer(args.reset_local_url, args.reset_local_timeout_s)

    if args.enable_streams:
        try_enable_streams(args)

    old_term = termios.tcgetattr(sys.stdin)
    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    arm_home = [0.0] * 14
    camera_cmd = [0.0, 0.0, 0.0]
    base = [0.0, 0.0, 0.0]
    yaw_store = YawStore()
    proprio_stop = threading.Event()
    proprio_thread = threading.Thread(target=proprio_loop, args=(args, yaw_store, proprio_stop), daemon=True)
    proprio_thread.start()
    period = 1.0 / max(args.rate_hz, 1e-3)
    last_print = 0.0

    print(HELP, flush=True)
    print(
        f"sim tcp://{args.sim_host}:{args.push_port}  rate={args.rate_hz:.1f}Hz  "
        f"speed_scale={args.speed_scale:.2f}  camera_yaw_offset={args.camera_yaw_offset_rad:.3f}rad",
        flush=True,
    )

    try:
        tty.setcbreak(sys.stdin.fileno())
        with push_socket(args.sim_host, args.push_port) as sock:
            while not stop:
                readable, _, _ = select.select([sys.stdin], [], [], 0.0)
                if readable:
                    key = sys.stdin.read(1).lower()
                    if key in ("\x03", "x"):
                        stop = True
                        continue
                    cmd = key_to_camera_cmd(key, args)
                    if cmd is not None:
                        camera_cmd = cmd

                robot_yaw, yaw_source = yaw_store.get()
                base = camera_to_root_cmd(camera_cmd, robot_yaw, args.camera_yaw_offset_rad)
                sent = send(sock, arm_home, base)
                now = time.monotonic()
                if now - last_print > 0.5:
                    print(
                        f"\rbase_cmd_vel=({base[0]:+.2f},{base[1]:+.2f},{base[2]:+.2f}) "
                        f"camera_cmd=({camera_cmd[0]:+.2f},{camera_cmd[1]:+.2f},{camera_cmd[2]:+.2f}) "
                        f"yaw_src={yaw_source} {'sent' if sent else 'send-blocked'}   ",
                        end="",
                        flush=True,
                    )
                    last_print = now
                time.sleep(period)

            print("\nStopping robot...", flush=True)
            for _ in range(10):
                send(sock, arm_home, [0.0, 0.0, 0.0])
                time.sleep(period)
    finally:
        proprio_stop.set()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)


if __name__ == "__main__":
    main()
