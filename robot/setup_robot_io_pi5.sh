#!/usr/bin/env bash
# Raspberry Pi 5 / onboard computer setup for XLeRobot rosbridge hardware I/O.
#
# This intentionally installs only:
#   - serial/depth sensor OS utilities
#   - a tiny Python venv with roslibpy, pyserial, pyrealsense2, and LeRobot Feetech support
#
# Do not install ROS 2/Nav2/SLAM/Foxglove on the Pi for this path.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${XLE_ROBOT_VENV:=$HOME/xlerobot-io-venv}"
: "${LEROBOT_ROOT:=$HOME/lerobot}"
: "${LEROBOT_INSTALL_MODE:=feetech}"

if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
else
  echo "[err] /etc/os-release not found"
  exit 1
fi

echo "============================================================"
echo "XLeRobot Raspberry Pi 5 hardware-I/O setup"
echo "============================================================"
echo "Repo             : $ROOT"
echo "OS               : ${PRETTY_NAME:-unknown}"
echo "Python venv      : $XLE_ROBOT_VENV"
echo "LeRobot checkout : $LEROBOT_ROOT"
echo "Install mode     : $LEROBOT_INSTALL_MODE"
echo "No ROS/Nav2/SLAM : yes"
echo "============================================================"

echo "[setup] installing light apt dependencies..."
sudo apt update
sudo apt install -y \
  curl \
  ffmpeg \
  git \
  libjpeg-dev \
  libusb-1.0-0 \
  libopenblas0 \
  python3-opencv \
  python3-pip \
  python3-venv \
  usbutils \
  v4l-utils

echo "[setup] granting serial/camera groups..."
sudo usermod -aG dialout,video "$USER"

echo "[setup] creating Python venv..."
python3 -m venv --system-site-packages "$XLE_ROBOT_VENV"
# shellcheck disable=SC1091
source "$XLE_ROBOT_VENV/bin/activate"
python -m pip install --upgrade pip setuptools wheel
PIP_NO_CACHE_DIR=1 python -m pip install numpy roslibpy pyserial pyrealsense2

if [[ ! -d "$LEROBOT_ROOT/.git" ]]; then
  echo "[setup] cloning LeRobot..."
  git clone --depth 1 https://github.com/huggingface/lerobot.git "$LEROBOT_ROOT"
fi

echo "[setup] installing LeRobot motor support..."
cd "$LEROBOT_ROOT"
case "$LEROBOT_INSTALL_MODE" in
  feetech)
    PIP_NO_CACHE_DIR=1 python -m pip install -e ".[feetech]"
    ;;
  minimal)
    PIP_NO_CACHE_DIR=1 python -m pip install -e . --no-deps
    PIP_NO_CACHE_DIR=1 python -m pip install pyserial
    echo "[warn] minimal mode may miss Feetech extra dependencies on newer LeRobot versions."
    ;;
  *)
    echo "[err] unknown LEROBOT_INSTALL_MODE: $LEROBOT_INSTALL_MODE"
    echo "      use feetech or minimal"
    exit 2
    ;;
esac

cd "$ROOT"
if [[ ! -f robot/xlerobot_robot_io.env ]]; then
  cp robot/xlerobot_robot_io.env.example robot/xlerobot_robot_io.env
fi

echo "[setup] running robot environment check..."
robot/check_robot_io_env.sh || true

cat <<EOF

Done.

Important:
  1. Reboot or log out/in once so dialout/video groups apply.
  2. Edit robot/xlerobot_robot_io.env for actual device paths:
       ROSBRIDGE_HOST, BASE_PORT, LIDAR_SERIAL, DEPTH_SENSOR_SERIAL
  3. Start only hardware I/O:
       ./run_xlerobot_rosbridge_io.sh

The compute PC should run:
  ./run_multisession_slam.sh hardware

EOF
