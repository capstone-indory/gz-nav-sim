#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parent
LOG_PATH = Path(os.environ.get("LOG_FILE", "/tmp/isaac_zmq_stack.log"))


def pkill(pattern: str) -> None:
    subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)


def wait_process_cleanup() -> None:
    time.sleep(0.4)


def reset_local_viewer(http_port: int) -> None:
    try:
        urllib.request.urlopen(f"http://127.0.0.1:{http_port}/reset_local", timeout=0.8).read()
    except Exception:
        pass


def open_log():
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    return LOG_PATH.open("w", buffering=1)


def start(args: argparse.Namespace) -> tuple[subprocess.Popen, subprocess.Popen]:
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    log = open_log()

    viewer_cmd = [
        sys.executable,
        "examples/web_viewer.py",
        "--sim-host",
        args.sim_host,
        "--http-port",
        str(args.http_port),
        "--ocr-backend",
        args.ocr_backend,
        "--floor-hint",
        args.floor_hint,
        "--floor-prior-mode",
        args.floor_prior_mode,
        "--ocr-scales",
        args.ocr_scales,
        "--grid-resolution-m",
        str(args.grid_resolution_m),
        "--grid-size-m",
        str(args.grid_size_m),
        "--grid-view-m",
        str(args.grid_view_m),
        "--camera-x-m",
        str(args.camera_x_m),
        "--camera-yaw-offset-rad",
        str(args.camera_yaw_offset_rad),
    ]
    nav_cmd = [
        sys.executable,
        "examples/isaac_nav_client.py",
        "--sim-host",
        args.sim_host,
        "--mode",
        "initialize",
        "--enable-streams",
        "--rate-hz",
        str(args.cmd_rate_hz),
        "--speed-scale",
        str(args.speed_scale),
        "--proprio-rate-hz",
        str(args.proprio_rate_hz),
        "--rgb-rate-hz",
        str(args.rgb_rate_hz),
        "--depth-rate-hz",
        str(args.depth_rate_hz),
        "--scan-rate-hz",
        str(args.scan_rate_hz),
        "--scan-mid-rate-hz",
        str(args.scan_mid_rate_hz),
        "--camera-yaw-offset-rad",
        str(args.camera_yaw_offset_rad),
        "--reset-local-url",
        f"http://127.0.0.1:{args.http_port}/reset_local",
    ]
    if args.reset_sim:
        nav_cmd.append("--reset")

    viewer = subprocess.Popen(viewer_cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    nav = subprocess.Popen(nav_cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    return viewer, nav


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Restart Isaac ZMQ viewer + initialize navigation with a clean local grid/OCR state."
    )
    parser.add_argument("--sim-host", default=os.environ.get("SIM_HOST", "100.80.87.68"))
    parser.add_argument("--http-port", type=int, default=int(os.environ.get("HTTP_PORT", "18081")))
    parser.add_argument("--ocr-backend", choices=("gazebo", "paddle", "easyocr", "tesseract", "none"), default=os.environ.get("OCR_BACKEND", "gazebo"))
    parser.add_argument("--floor-hint", default=os.environ.get("FLOOR_HINT", "5"))
    parser.add_argument("--floor-prior-mode", choices=("reject", "complete"), default=os.environ.get("FLOOR_PRIOR_MODE", "complete"))
    parser.add_argument("--ocr-scales", default=os.environ.get("OCR_SCALES", "1.0,2.0,3.0,4.0,6.0"))
    parser.add_argument("--grid-resolution-m", type=float, default=float(os.environ.get("GRID_RESOLUTION_M", "0.10")))
    parser.add_argument("--grid-size-m", type=float, default=float(os.environ.get("GRID_SIZE_M", "80.0")))
    parser.add_argument("--grid-view-m", type=float, default=float(os.environ.get("GRID_VIEW_M", "18.0")))
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=float(os.environ.get("NAV_SPEED_SCALE", os.environ.get("SPEED_SCALE", "4.0"))),
    )
    parser.add_argument("--cmd-rate-hz", type=float, default=float(os.environ.get("NAV_CMD_RATE_HZ", os.environ.get("CMD_RATE_HZ", "8.0"))))
    parser.add_argument("--proprio-rate-hz", type=float, default=float(os.environ.get("PROPRIO_RATE_HZ", "10.0")))
    parser.add_argument("--rgb-rate-hz", type=float, default=float(os.environ.get("RGB_RATE_HZ", "5.0")))
    parser.add_argument("--depth-rate-hz", type=float, default=float(os.environ.get("DEPTH_RATE_HZ", "3.0")))
    parser.add_argument("--scan-rate-hz", type=float, default=float(os.environ.get("SCAN_RATE_HZ", "5.0")))
    parser.add_argument("--scan-mid-rate-hz", type=float, default=float(os.environ.get("SCAN_MID_RATE_HZ", "0.0")))
    parser.add_argument("--camera-x-m", type=float, default=float(os.environ.get("CAMERA_X_M", "-0.147")))
    parser.add_argument(
        "--camera-yaw-offset-rad",
        type=float,
        default=float(os.environ.get("CAMERA_YAW_OFFSET_RAD", "0.0")),
    )
    parser.add_argument("--reset-sim", action="store_true", help="Also request Isaac Sim reset over RPC.")
    parser.add_argument("--foreground", action="store_true", help="Keep this process attached and stop children on Ctrl-C.")
    args = parser.parse_args()

    reset_local_viewer(args.http_port)
    pkill(r"[p]ython3 examples/wasd_teleop.py")
    pkill(r"[p]ython3 examples/isaac_nav_client.py")
    pkill(r"[p]ython3 examples/web_viewer.py")
    pkill(r"[b]ash ./run_isaac_zmq_stack.sh")
    wait_process_cleanup()

    viewer, nav = start(args)
    print(f"init nav started: viewer_pid={viewer.pid} nav_pid={nav.pid}")
    print(f"http://127.0.0.1:{args.http_port}/  sim={args.sim_host}  log={LOG_PATH}")
    print("local grid map and OCR annotations were reset before startup")

    if not args.foreground:
        return

    def stop(_signum, _frame):
        for proc in (nav, viewer):
            if proc.poll() is None:
                proc.terminate()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while viewer.poll() is None and nav.poll() is None:
            time.sleep(0.5)
    finally:
        stop(None, None)


if __name__ == "__main__":
    main()
