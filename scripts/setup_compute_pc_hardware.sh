#!/usr/bin/env bash
# Setup this compute PC for remote XLeRobot hardware mode.
#
# Installs only the ROS 2/Nav2/SLAM/Foxglove stack used by:
#   ./run_multisession_slam.sh hardware
#   ./run_xlerobot_compute_nav.sh
#
# If you want the no-sudo Robostack path instead, use:
#   scripts/setup_compute_pc_hardware_conda.sh
#
# It deliberately does not install/start Postgres, Java, Node, web backend,
# frontend, Isaac Sim, VLM/OCR model stacks, or robot hardware drivers.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${ROS_DISTRO:=humble}"
: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=1}"
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
else
  echo "[err] /etc/os-release not found"
  exit 1
fi

if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "[warn] This setup is tuned for Ubuntu 22.04 + ROS 2 Humble."
  echo "[warn] Detected: ${PRETTY_NAME:-unknown}"
fi

if ! command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then
  echo "[err] sudo is required for apt-based ROS setup."
  exit 1
fi

echo "============================================================"
echo "Compute PC hardware-mode setup"
echo "============================================================"
echo "Repo           : $ROOT"
echo "ROS_DISTRO     : $ROS_DISTRO"
echo "ROS_DOMAIN_ID  : $ROS_DOMAIN_ID"
echo "No DB/web stack: yes"
echo "============================================================"

if [[ $EUID -ne 0 ]]; then
  echo "[setup] sudo password may be requested for apt installs."
  "${SUDO[@]}" -v
fi

echo "[setup] installing apt prerequisites..."
"${SUDO[@]}" apt update
"${SUDO[@]}" apt install -y \
  curl \
  gnupg \
  lsb-release \
  software-properties-common

echo "[setup] enabling Ubuntu universe repository..."
"${SUDO[@]}" add-apt-repository -y universe

echo "[setup] configuring ROS 2 apt repository..."
"${SUDO[@]}" install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | "${SUDO[@]}" tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null

UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-jammy}}"
ARCH="$(dpkg --print-architecture)"
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "[setup] installing ROS/Nav2/SLAM/Foxglove packages..."
"${SUDO[@]}" apt update
"${SUDO[@]}" apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-opencv \
  python3-pip \
  python3-rosdep \
  python3-serial \
  python3-vcstool \
  python3-yaml \
  ros-"${ROS_DISTRO}"-ament-cmake \
  ros-"${ROS_DISTRO}"-cv-bridge \
  ros-"${ROS_DISTRO}"-foxglove-bridge \
  ros-"${ROS_DISTRO}"-foxglove-msgs \
  ros-"${ROS_DISTRO}"-launch-ros \
  ros-"${ROS_DISTRO}"-nav2-bringup \
  ros-"${ROS_DISTRO}"-navigation2 \
  ros-"${ROS_DISTRO}"-rclcpp-components \
  ros-"${ROS_DISTRO}"-rclpy \
  ros-"${ROS_DISTRO}"-robot-state-publisher \
  ros-"${ROS_DISTRO}"-ros-base \
  ros-"${ROS_DISTRO}"-rosbridge-server \
  ros-"${ROS_DISTRO}"-rosbag2-storage-mcap \
  ros-"${ROS_DISTRO}"-sensor-msgs-py \
  ros-"${ROS_DISTRO}"-slam-toolbox \
  ros-"${ROS_DISTRO}"-tf2-geometry-msgs \
  ros-"${ROS_DISTRO}"-tf2-ros \
  ros-"${ROS_DISTRO}"-twist-mux

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "[setup] initializing rosdep..."
  "${SUDO[@]}" rosdep init || true
fi

echo "[setup] updating rosdep cache..."
rosdep update

echo "[setup] building gz_nav_sim workspace package..."
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"
colcon build --symlink-install --paths src/gz_nav_sim

echo "[setup] running environment check..."
ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY" \
FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS" \
  scripts/check_compute_pc_hardware_env.sh

cat <<EOF

Done.

Start the compute-side stack with rosbridge enabled:
  ROS_DOMAIN_ID=$ROS_DOMAIN_ID ./run_multisession_slam.sh hardware

Robot-side:
  edit robot/xlerobot_robot_io.env and set ROSBRIDGE_HOST to this compute PC IP
  ./run_xlerobot_rosbridge_io.sh

EOF
