#!/usr/bin/env bash
# User-space setup for the compute PC using micromamba/Robostack.
#
# This is the no-sudo path for:
#   ./run_multisession_slam.sh hardware
#   ./run_xlerobot_compute_nav.sh
#
# It installs ROS 2 Humble, Nav2, SLAM Toolbox, twist_mux, Foxglove,
# rosbridge_server, OpenCV, and pyserial into a micromamba environment.
# It does not install DB/web/Isaac.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${XLE_COMPUTE_ENV:=gz-nav-humble}"
: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=1}"
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS

if ! command -v micromamba >/dev/null 2>&1; then
  echo "[err] micromamba not found. Install Miniforge/micromamba first."
  exit 1
fi

echo "============================================================"
echo "Compute PC hardware-mode conda setup"
echo "============================================================"
echo "Repo          : $ROOT"
echo "Conda env     : $XLE_COMPUTE_ENV"
echo "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "No DB/web     : yes"
echo "============================================================"

if ! micromamba env list | awk '{print $1}' | grep -qx "$XLE_COMPUTE_ENV"; then
  echo "[setup] creating micromamba env: $XLE_COMPUTE_ENV"
  micromamba create -y -n "$XLE_COMPUTE_ENV" \
    -c robostack-humble -c conda-forge --strict-channel-priority \
    python=3.12 \
    colcon-core \
    colcon-cmake \
    colcon-ros \
    colcon-package-selection \
    colcon-recursive-crawl \
    numpy \
    opencv \
    pyserial \
    pyyaml \
    ros-humble-cv-bridge \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-robot-state-publisher \
    ros-humble-ros-base \
    ros-humble-rosbridge-server \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-sensor-msgs-py \
    ros-humble-slam-toolbox \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-twist-mux
else
  echo "[setup] micromamba env already exists: $XLE_COMPUTE_ENV"
fi

echo "[setup] building gz_nav_sim inside $XLE_COMPUTE_ENV..."
micromamba run -n "$XLE_COMPUTE_ENV" bash -lc '
  set -eo pipefail
  set +u
  source "$CONDA_PREFIX/setup.bash"
  set -u
  colcon build --symlink-install --paths src/gz_nav_sim
'

echo "[setup] running environment check..."
micromamba run -n "$XLE_COMPUTE_ENV" bash -lc '
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}" \
    ./scripts/check_compute_pc_hardware_env.sh
'

cat <<EOF

Done.

Run compute-side hardware mode with rosbridge enabled:
  micromamba run -n $XLE_COMPUTE_ENV ./run_multisession_slam.sh hardware

Robot-side:
  edit robot/xlerobot_robot_io.env and set ROSBRIDGE_HOST to this compute PC IP
  ./run_xlerobot_rosbridge_io.sh

EOF
