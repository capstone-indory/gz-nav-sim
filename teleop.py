#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def pkill(pattern: str) -> None:
    subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)


def reset_local_viewer(http_port: int) -> None:
    try:
        body = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/reset_local", timeout=0.8).read()
        print(f"reset_local: {body.decode('utf-8', errors='replace')}")
    except Exception as exc:
        print(f"reset_local skipped: http://127.0.0.1:{http_port}/reset_local ({exc})")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run WASD teleop with clean viewer grid/OCR state. Does not start the web viewer."
    )
    parser.add_argument("--sim-host", default=os.environ.get("SIM_HOST", "100.80.87.68"))
    parser.add_argument("--http-port", type=int, default=int(os.environ.get("HTTP_PORT", "18081")))
    parser.add_argument("--vx", type=float, default=0.12)
    parser.add_argument("--vy", type=float, default=0.12)
    parser.add_argument("--wz", type=float, default=0.55)
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=float(os.environ.get("TELEOP_SPEED_SCALE", os.environ.get("SPEED_SCALE", "4.0"))),
    )
    parser.add_argument("--cmd-rate-hz", type=float, default=float(os.environ.get("TELEOP_CMD_RATE_HZ", os.environ.get("CMD_RATE_HZ", "8.0"))))
    parser.add_argument("--proprio-rate-hz", type=float, default=float(os.environ.get("PROPRIO_RATE_HZ", "10.0")))
    parser.add_argument("--rgb-rate-hz", type=float, default=float(os.environ.get("RGB_RATE_HZ", "5.0")))
    parser.add_argument("--depth-rate-hz", type=float, default=float(os.environ.get("DEPTH_RATE_HZ", "3.0")))
    parser.add_argument("--scan-rate-hz", type=float, default=float(os.environ.get("SCAN_RATE_HZ", "5.0")))
    parser.add_argument("--scan-mid-rate-hz", type=float, default=float(os.environ.get("SCAN_MID_RATE_HZ", "0.0")))
    parser.add_argument(
        "--camera-yaw-offset-rad",
        type=float,
        default=float(os.environ.get("CAMERA_YAW_OFFSET_RAD", "0.0")),
    )
    parser.add_argument("--keep-nav", action="store_true", help="Do not stop an existing autonomous nav process.")
    parser.add_argument("--no-reset-local", action="store_true")
    args = parser.parse_args()

    if not args.keep_nav:
        pkill(r"[p]ython3 examples/isaac_nav_client.py")
    pkill(r"[p]ython3 examples/wasd_teleop.py")
    if not args.no_reset_local:
        reset_local_viewer(args.http_port)

    teleop_cmd = [
        sys.executable,
        "examples/wasd_teleop.py",
        "--sim-host",
        args.sim_host,
        "--enable-streams",
        "--rate-hz",
        str(args.cmd_rate_hz),
        "--vx",
        str(args.vx),
        "--vy",
        str(args.vy),
        "--wz",
        str(args.wz),
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
        "--no-reset-local",
    ]
    subprocess.run(teleop_cmd, cwd=ROOT, check=False)


if __name__ == "__main__":
    main()
