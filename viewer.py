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
sys.path.insert(0, str(ROOT / "examples"))

from _client_common import req_socket, rpc  # noqa: E402


def pkill(pattern: str) -> None:
    subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)


def stream_setup(args: argparse.Namespace) -> None:
    if not args.enable_streams:
        return
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
                    print(f"disable_stream {topic}: {resp}")
                else:
                    resp = rpc(sock, "enable_stream", topic=topic, rate_hz=rate_hz)
                    print(f"enable_stream {topic}@{rate_hz:g}Hz: {resp}")
        except Exception as exc:
            print(f"stream setup {topic}: failed: {exc}")


def reset_local_viewer(http_port: int) -> None:
    try:
        body = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/reset_local", timeout=0.8).read()
        print(f"reset_local: {body.decode('utf-8', errors='replace')}")
    except Exception as exc:
        print(f"reset_local skipped: http://127.0.0.1:{http_port}/reset_local ({exc})")


def wait_http(http_port: int, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{http_port}/topics.json", timeout=0.5).read()
            return True
        except Exception:
            time.sleep(0.15)
    return False


def log_path(http_port: int) -> Path:
    return Path(os.environ.get("LOG_FILE", f"/tmp/isaac_web_viewer_{http_port}.log"))


def build_viewer_cmd(args: argparse.Namespace) -> list[str]:
    return [
        sys.executable,
        "examples/web_viewer.py",
        "--sim-host",
        args.sim_host,
        "--http-host",
        args.http_host,
        "--http-port",
        str(args.http_port),
        "--ocr-backend",
        args.ocr_backend,
        "--ocr-interval",
        str(args.ocr_interval),
        "--ocr-min-confidence",
        str(args.ocr_min_confidence),
        "--floor-hint",
        args.floor_hint,
        "--floor-prior-mode",
        args.floor_prior_mode,
        "--ocr-scales",
        args.ocr_scales,
        "--ocr-max-side",
        str(args.ocr_max_side),
        "--map-interval",
        str(args.map_interval),
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Start only the Isaac ZMQ web viewer. No nav or teleop process is started.")
    parser.add_argument("--sim-host", default=os.environ.get("SIM_HOST", "100.80.87.68"))
    parser.add_argument("--http-host", default=os.environ.get("HTTP_HOST", "0.0.0.0"))
    parser.add_argument("--http-port", type=int, default=int(os.environ.get("HTTP_PORT", "18081")))
    parser.add_argument("--rpc-port", type=int, default=5557)
    parser.add_argument("--rpc-timeout-ms", type=int, default=1000)
    parser.add_argument("--ocr-backend", choices=("gazebo", "paddle", "easyocr", "tesseract", "none"), default=os.environ.get("OCR_BACKEND", "gazebo"))
    parser.add_argument("--ocr-interval", type=float, default=float(os.environ.get("OCR_INTERVAL", "2.0")))
    parser.add_argument("--ocr-min-confidence", type=float, default=float(os.environ.get("OCR_MIN_CONFIDENCE", "0.25")))
    parser.add_argument("--floor-hint", default=os.environ.get("FLOOR_HINT", "5"))
    parser.add_argument("--floor-prior-mode", choices=("reject", "complete"), default=os.environ.get("FLOOR_PRIOR_MODE", "complete"))
    parser.add_argument("--ocr-scales", default=os.environ.get("OCR_SCALES", "1.0,2.0,3.0,4.0,6.0"))
    parser.add_argument("--ocr-max-side", type=int, default=int(os.environ.get("OCR_MAX_SIDE", "2400")))
    parser.add_argument("--map-interval", type=float, default=float(os.environ.get("MAP_INTERVAL", "0.20")))
    parser.add_argument("--grid-resolution-m", type=float, default=float(os.environ.get("GRID_RESOLUTION_M", "0.10")))
    parser.add_argument("--grid-size-m", type=float, default=float(os.environ.get("GRID_SIZE_M", "80.0")))
    parser.add_argument("--grid-view-m", type=float, default=float(os.environ.get("GRID_VIEW_M", "18.0")))
    parser.add_argument("--camera-x-m", type=float, default=float(os.environ.get("CAMERA_X_M", "-0.147")))
    parser.add_argument("--camera-yaw-offset-rad", type=float, default=float(os.environ.get("CAMERA_YAW_OFFSET_RAD", "0.0")))
    parser.add_argument("--proprio-rate-hz", type=float, default=float(os.environ.get("PROPRIO_RATE_HZ", "10.0")))
    parser.add_argument("--rgb-rate-hz", type=float, default=float(os.environ.get("RGB_RATE_HZ", "5.0")))
    parser.add_argument("--depth-rate-hz", type=float, default=float(os.environ.get("DEPTH_RATE_HZ", "3.0")))
    parser.add_argument("--scan-rate-hz", type=float, default=float(os.environ.get("SCAN_RATE_HZ", "5.0")))
    parser.add_argument("--scan-mid-rate-hz", type=float, default=float(os.environ.get("SCAN_MID_RATE_HZ", "0.0")))
    parser.add_argument("--enable-streams", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--restart", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-local", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--foreground", action="store_true")
    args = parser.parse_args()

    stream_setup(args)
    if args.restart:
        pkill(r"[p]ython3 examples/web_viewer.py")
        time.sleep(0.3)

    path = log_path(args.http_port)
    path.parent.mkdir(parents=True, exist_ok=True)
    log = path.open("w", buffering=1)
    proc = subprocess.Popen(
        build_viewer_cmd(args),
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=not args.foreground,
    )

    if wait_http(args.http_port):
        if args.reset_local:
            reset_local_viewer(args.http_port)
        print(f"viewer_pid={proc.pid} http://127.0.0.1:{args.http_port}/ log={path}")
    else:
        print(f"viewer_pid={proc.pid} started, but HTTP did not respond yet. log={path}")

    if not args.foreground:
        return

    def stop(_signum, _frame):
        if proc.poll() is None:
            proc.terminate()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        proc.wait()
    finally:
        stop(None, None)


if __name__ == "__main__":
    main()
