#!/usr/bin/env bash
# Compute-PC runtime: passive SLAM + Nav2 + map-only Foxglove.
#
# The robot computer should run ./run_xlerobot_rosbridge_io.sh and connect to
# this machine's rosbridge_server. This script does no DB/web/backend.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

: "${ROS_DISTRO:=humble}"
: "${ROS_DOMAIN_ID:=42}"
ROS_LOCALHOST_ONLY=1
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
: "${START_ROSBRIDGE:=1}"
: "${ROSBRIDGE_HOST:=0.0.0.0}"
: "${ROSBRIDGE_PORT:=9090}"
ROSBRIDGE_WIRE_FORMAT=json
: "${ROBOT_SSH_AUTOSTART:=1}"
: "${ROBOT_SSH_TARGET:=RasberryPi}"
: "${ROBOT_SSH_FALLBACK_TARGET:=pi@lekiwi}"
: "${ROBOT_SSH_IDENTITY:=~/.ssh/indory_RasberryPi_ed25519}"
: "${ROBOT_REMOTE_REPO:=~/gz-nav-sim}"
: "${ROBOT_IO_REMOTE_LOG:=~/xlerobot_rosbridge_io.log}"
: "${ROBOT_SSH_SYNC:=0}"
: "${ROBOT_SSH_REQUIRED:=0}"
: "${XLE_COMPUTE_ENV:=gz-nav-humble}"
if [[ -z "${ROS_SETUP:-}" && -n "${CONDA_PREFIX:-}" && -f "$CONDA_PREFIX/setup.bash" ]]; then
  ROS_SETUP="$CONDA_PREFIX/setup.bash"
elif [[ -z "${ROS_SETUP:-}" && -f "$HOME/micromamba/envs/$XLE_COMPUTE_ENV/setup.bash" ]]; then
  ROS_SETUP="$HOME/micromamba/envs/$XLE_COMPUTE_ENV/setup.bash"
else
  : "${ROS_SETUP:=/opt/ros/${ROS_DISTRO}/setup.bash}"
fi
if [[ -z "${WORKSPACE_SETUP:-}" ]]; then
  if [[ -f install/setup.bash ]]; then
    WORKSPACE_SETUP="install/setup.bash"
  else
    WORKSPACE_SETUP="install/setup.sh"
  fi
fi

export ROS_DISTRO XLE_COMPUTE_ENV ROS_SETUP WORKSPACE_SETUP ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
export ROSBRIDGE_PORT ROSBRIDGE_WIRE_FORMAT ROBOT_SSH_AUTOSTART ROBOT_SSH_TARGET ROBOT_SSH_FALLBACK_TARGET ROBOT_SSH_IDENTITY ROBOT_REMOTE_REPO ROBOT_IO_REMOTE_LOG ROBOT_SSH_SYNC ROBOT_SSH_REQUIRED

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[err] ROS setup not found: $ROS_SETUP"
  echo "      apt setup: scripts/setup_compute_pc_hardware.sh"
  echo "      conda/robostack: export ROS_SETUP=/path/to/env/setup.bash"
  exit 1
fi
set +u
source "$ROS_SETUP"
set -u

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "[err] workspace setup not found: $WORKSPACE_SETUP"
  echo "      Build first: colcon build --symlink-install --paths src/gz_nav_sim"
  exit 1
fi
set +u
source "$WORKSPACE_SETUP"
set -u

echo "================================================------------"
echo "XLeRobot compute-only Nav2/SLAM"
echo "================================================------------"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "Input topics from robot:"
echo "  /xlerobot/odom"
echo "  /xlerobot/scan"
echo "  /xlerobot/head_camera/color/image"
echo "  /xlerobot/head_camera/depth/image"
echo "  /xlerobot/head_camera/imu"
echo "Output command:"
echo "  /cmd_vel -> /cmd_vel_mux -> /xlerobot/cmd_vel"
echo "Robot connection:"
echo "  ws://<this-compute-pc-ip>:${ROSBRIDGE_PORT} (wire=json)"
if [[ "${ROBOT_SSH_AUTOSTART}" == "1" || "${ROBOT_SSH_AUTOSTART}" == "true" ]]; then
  echo "  SSH autostart: ${ROBOT_SSH_TARGET} (${ROBOT_REMOTE_REPO})"
else
  echo "  SSH autostart: disabled; start ./run_xlerobot_rosbridge_io.sh on the robot"
fi
echo "Foxglove:"
echo "  ws://localhost:8765"
echo "Topic checks:"
echo "  ros2 topic hz /xlerobot/odom"
echo "  ros2 topic hz /xlerobot/scan"
echo "Manual base test:"
echo "  ros2 topic pub --rate 10 /cmd_vel_teleop geometry_msgs/msg/Twist '{linear: {x: 0.10}, angular: {z: 0.0}}'"
echo "================================================------------"

ROSBRIDGE_PID=""
ROBOT_IO_SSH_STARTED=0
existing_rosbridge_is_bson() {
  ps -eo args= | awk '
    /rosbridge_(websocket|server)|rosbridge_server/ && !/awk/ {
      if ($0 ~ /bson_only_mode[:= ]+true|--bson_only_mode[ =]true/) found=1
    }
    END { exit found ? 0 : 1 }
  '
}
cleanup() {
  if [[ ${ROBOT_IO_SSH_STARTED:-0} == 1 && -x "$SCRIPT_DIR/scripts/robot_io_ssh.sh" ]]; then
    echo "[exit] stopping remote robot I/O on ${ROBOT_SSH_TARGET}"
    "$SCRIPT_DIR/scripts/robot_io_ssh.sh" stop || true
  fi
  if [[ -n ${ROSBRIDGE_PID:-} ]] && kill -0 "$ROSBRIDGE_PID" 2>/dev/null; then
    echo "[exit] stopping rosbridge_server pid=${ROSBRIDGE_PID}"
    kill "$ROSBRIDGE_PID" 2>/dev/null || true
    wait "$ROSBRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$START_ROSBRIDGE" == "1" ]]; then
  if (echo > "/dev/tcp/127.0.0.1/${ROSBRIDGE_PORT}") >/dev/null 2>&1; then
    if existing_rosbridge_is_bson; then
      echo "[err] rosbridge_server is already reachable, but it appears to be BSON-only."
      echo "      Stop the stale rosbridge_server and rerun; this stack uses JSON only."
      exit 1
    fi
    echo "[boot] rosbridge_server already reachable on :${ROSBRIDGE_PORT}; using existing"
  elif ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    echo "[boot] starting rosbridge_server on ${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT} (wire=json, bson_only_mode=false)"
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
      address:="${ROSBRIDGE_HOST}" port:="${ROSBRIDGE_PORT}" bson_only_mode:=false &
    ROSBRIDGE_PID=$!
    for _ in {1..15}; do
      (echo > "/dev/tcp/127.0.0.1/${ROSBRIDGE_PORT}") >/dev/null 2>&1 && break
      sleep 1
    done
    if ! (echo > "/dev/tcp/127.0.0.1/${ROSBRIDGE_PORT}") >/dev/null 2>&1; then
      echo "[err] rosbridge_server did not open :${ROSBRIDGE_PORT}"
      exit 1
    fi
  else
    echo "[err] rosbridge_server package missing."
    echo "      conda: micromamba install -n $XLE_COMPUTE_ENV -c robostack-humble -c conda-forge ros-humble-rosbridge-server"
    echo "      apt  : sudo apt install ros-${ROS_DISTRO}-rosbridge-server"
    exit 1
  fi
fi

if [[ "${ROBOT_SSH_AUTOSTART}" == "1" || "${ROBOT_SSH_AUTOSTART}" == "true" ]]; then
  if [[ ! -x "$SCRIPT_DIR/scripts/robot_io_ssh.sh" ]]; then
    echo "[err] missing robot SSH helper: $SCRIPT_DIR/scripts/robot_io_ssh.sh"
    exit 1
  fi
  echo "[boot] starting remote robot I/O over SSH"
  if "$SCRIPT_DIR/scripts/robot_io_ssh.sh" start; then
    ROBOT_IO_SSH_STARTED=1
  elif [[ "${ROBOT_SSH_REQUIRED}" == "1" || "${ROBOT_SSH_REQUIRED}" == "true" ]]; then
    echo "[err] failed to start remote robot I/O over SSH"
    exit 1
  else
    echo "[warn] failed to start remote robot I/O over SSH; continuing ROS stack"
  fi
fi

LAUNCH_ARGS=(
  isaac_transport:=xlerobot_ros
  ros_localhost_only:="${ROS_LOCALHOST_ONLY}"
  use_sim_time:=false
  use_hardware_lidar:=false
  use_foxglove:=true
  foxglove_profile:=map
  use_slam_toolbox:="${USE_SLAM_TOOLBOX:-false}"
  use_lidar_odom:="${USE_LIDAR_ODOM:-false}"
  use_rtabmap:="${USE_RTABMAP:-true}"
  rtabmap_odom_source:="${RTABMAP_ODOM_SOURCE:-rgbd}"
  use_imu:="${USE_IMU:-true}"
  use_slam_scan_filter:="${USE_SLAM_SCAN_FILTER:-true}"
  slam_scan_filter_min_range:="${SLAM_SCAN_FILTER_MIN_RANGE:-0.20}"
  slam_scan_filter_max_range:="${SLAM_SCAN_FILTER_MAX_RANGE:-0.0}"
  slam_scan_filter_remove_isolated_clusters:="${SLAM_SCAN_FILTER_REMOVE_ISOLATED_CLUSTERS:-true}"
  slam_scan_filter_min_cluster_points:="${SLAM_SCAN_FILTER_MIN_CLUSTER_POINTS:-3}"
  slam_scan_filter_cluster_jump_m:="${SLAM_SCAN_FILTER_CLUSTER_JUMP_M:-0.30}"
  slam_scan_filter_cluster_max_range:="${SLAM_SCAN_FILTER_CLUSTER_MAX_RANGE:-2.5}"
  lidar_odom_max_range:="${LIDAR_ODOM_MAX_RANGE:-8.0}"
  lidar_odom_max_points:="${LIDAR_ODOM_MAX_POINTS:-240}"
  lidar_odom_icp_iterations:="${LIDAR_ODOM_ICP_ITERATIONS:-8}"
  lidar_odom_max_correspondence_distance:="${LIDAR_ODOM_MAX_CORRESPONDENCE_DISTANCE:-0.35}"
  lidar_odom_min_pairs:="${LIDAR_ODOM_MIN_PAIRS:-35}"
  lidar_odom_max_translation_per_scan:="${LIDAR_ODOM_MAX_TRANSLATION_PER_SCAN:-0.35}"
  lidar_odom_max_rotation_per_scan:="${LIDAR_ODOM_MAX_ROTATION_PER_SCAN:-0.60}"
  lidar_odom_invert_delta:="${LIDAR_ODOM_INVERT_DELTA:-false}"
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=false
  use_semantic_ocr:=false
  use_semantic_vlm:=false
  use_explore:=false
  direct_depth:=true
)

if [[ "$#" -gt 0 ]]; then
  LAUNCH_ARGS+=("$@")
fi

ros2 launch gz_nav_sim sim_nav.launch.py "${LAUNCH_ARGS[@]}"
