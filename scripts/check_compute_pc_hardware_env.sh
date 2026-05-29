#!/usr/bin/env bash
# Check the compute-PC environment for remote XLeRobot hardware mode.
# This does not start the robot; it only verifies the local ROS/Nav stack.

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${ROS_DISTRO:=humble}"
: "${XLE_COMPUTE_ENV:=gz-nav-humble}"
if [[ -z "${ROS_SETUP:-}" && -n "${CONDA_PREFIX:-}" && -f "$CONDA_PREFIX/setup.bash" ]]; then
  ROS_SETUP="$CONDA_PREFIX/setup.bash"
elif [[ -z "${ROS_SETUP:-}" && -f "$HOME/micromamba/envs/$XLE_COMPUTE_ENV/setup.bash" ]]; then
  ROS_SETUP="$HOME/micromamba/envs/$XLE_COMPUTE_ENV/setup.bash"
else
  : "${ROS_SETUP:=/opt/ros/${ROS_DISTRO}/setup.bash}"
fi
: "${ROS_DOMAIN_ID:=42}"
ROS_LOCALHOST_ONLY=1
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
if [[ -z "${WORKSPACE_SETUP:-}" ]]; then
  if [[ -f install/setup.bash ]]; then
    WORKSPACE_SETUP="install/setup.bash"
  else
    WORKSPACE_SETUP="install/setup.sh"
  fi
fi

export ROS_DISTRO XLE_COMPUTE_ENV ROS_SETUP WORKSPACE_SETUP ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS

FAIL=0

ok() {
  printf '[ ok ] %s\n' "$*"
}

warn() {
  printf '[warn] %s\n' "$*"
}

bad() {
  printf '[err] %s\n' "$*"
  FAIL=1
}

need_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    ok "command: $1"
  else
    bad "missing command: $1"
  fi
}

need_pkg() {
  local pkg="$1"
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    ok "ROS package: $pkg"
  else
    bad "missing ROS package: $pkg"
  fi
}

echo "============================================================"
echo "Compute PC hardware-mode environment check"
echo "============================================================"
echo "ROOT=$ROOT"
echo "ROS_SETUP=$ROS_SETUP"
echo "WORKSPACE_SETUP=$WORKSPACE_SETUP"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS"
echo "============================================================"

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$ROS_SETUP"
  set -u
  ok "ROS setup sourced"
else
  bad "ROS setup not found: $ROS_SETUP"
fi

need_cmd ros2
need_cmd colcon
need_cmd python3

if command -v python3 >/dev/null 2>&1; then
  python3 - <<'PY'
import importlib.util
import sys

missing = [name for name in ("cv2", "serial", "yaml") if importlib.util.find_spec(name) is None]
if missing:
    print("[err] missing Python modules: " + ", ".join(missing))
    sys.exit(1)
print("[ ok ] Python modules: cv2, serial, yaml")
PY
  if [[ $? -ne 0 ]]; then
    FAIL=1
  fi
fi

if command -v ros2 >/dev/null 2>&1; then
  for pkg in \
    ament_cmake \
    launch_ros \
    rclpy \
    rosbridge_server \
    nav2_bringup \
    slam_toolbox \
    twist_mux \
    robot_state_publisher \
    tf2_ros \
    tf2_geometry_msgs \
    cv_bridge \
    sensor_msgs_py \
    foxglove_bridge \
    foxglove_msgs; do
    need_pkg "$pkg"
  done
fi

if command -v python3 >/dev/null 2>&1; then
  python3 - <<'PY'
import importlib.util
import sys

if importlib.util.find_spec("numpy") is None:
    print("[err] missing Python module: numpy")
    sys.exit(1)
print("[ ok ] Python module: numpy")
PY
  if [[ $? -ne 0 ]]; then
    FAIL=1
  fi
fi

if [[ -f "$WORKSPACE_SETUP" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$WORKSPACE_SETUP"
  set -u
  ok "workspace setup: $WORKSPACE_SETUP"
  if command -v ros2 >/dev/null 2>&1; then
    need_pkg gz_nav_sim
  fi
else
  bad "workspace is not built: $WORKSPACE_SETUP missing"
  warn "build with: colcon build --symlink-install --paths src/gz_nav_sim"
fi

if [[ $FAIL -eq 0 ]]; then
  echo "============================================================"
  ok "ready for compute-side hardware mode"
  echo "Run:"
  echo "  ./run_multisession_slam.sh hardware"
  echo "or:"
  echo "  ./run_xlerobot_compute_nav.sh"
  echo ""
  echo "Robot-side topics expected:"
  echo "  /xlerobot/scan"
  echo "  /xlerobot/cmd_vel"
  echo "  /xlerobot/odom is optional and ignored by default hardware SLAM"
  echo "Robot-side connection:"
  echo "  ws://<this-compute-pc-ip>:9090"
else
  echo "================================================------------"
  bad "environment is incomplete"
  echo "Install/build with:"
  echo "  scripts/setup_compute_pc_hardware.sh"
fi

exit "$FAIL"
