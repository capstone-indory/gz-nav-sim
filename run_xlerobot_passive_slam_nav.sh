#!/usr/bin/env bash
# XLeRobot topics -> passive SLAM + Nav2 + map-only Foxglove.
#
# Expected Isaac-side topics, from xlerobot_hospital:
#   /xlerobot/odom
#   /xlerobot/scan
#   /xlerobot/head_camera/color/image
#   /xlerobot/head_camera/color/camera_info
#   /xlerobot/head_camera/depth/image
#   /xlerobot/head_camera/depth/camera_info
#   /xlerobot/head_camera/imu
#   /xlerobot/cmd_vel
#
# This script keeps heavy perception off. SLAM builds /map from LiDAR scan
# matching, Nav2 accepts goals, and twist_mux forwards Nav2 velocity to
# /xlerobot/cmd_vel.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

: "${ROS_DISTRO:=humble}"
: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=0}"
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
: "${ROSBRIDGE_PORT:=9090}"
: "${START_ROSBRIDGE:=auto}"   # auto | true | false
: "${USE_HARDWARE_LIDAR:=false}"
: "${USE_LIDAR_ODOM:=true}"
: "${HARDWARE_LIDAR_SERIAL:=/dev/ttyUSB0}"
: "${HARDWARE_LIDAR_BAUD:=460800}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
if [[ ! -f "${ROS_SETUP}" && -f "/home/indory/micromamba/envs/gz-nav-humble/setup.bash" ]]; then
  ROS_SETUP="/home/indory/micromamba/envs/gz-nav-humble/setup.bash"
fi
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[err] ROS setup not found: ${ROS_SETUP}"
  exit 1
fi
source "${ROS_SETUP}"
PROJECT_SETUP=""
if [[ -f install/setup.bash ]]; then
  PROJECT_SETUP="install/setup.bash"
elif [[ -f install/setup.sh ]]; then
  PROJECT_SETUP="install/setup.sh"
fi
if [[ -z "${PROJECT_SETUP}" ]]; then
  echo "[err] install setup not found. Build first: colcon build --symlink-install"
  exit 1
fi
source "${PROJECT_SETUP}"

port_listening() {
  local port="$1"
  ss -lnt 2>/dev/null | awk '{print $4}' | grep -Eq "(^|:)${port}$"
}

existing_rosbridge_is_bson() {
  ps -eo args= | awk '
    /rosbridge_(websocket|server)|rosbridge_server/ && !/awk/ {
      if ($0 ~ /bson_only_mode[:= ]+true|--bson_only_mode[ =]true/) found=1
    }
    END { exit found ? 0 : 1 }
  '
}

ROSBRIDGE_PID=""
cleanup() {
  if [[ -n "${ROSBRIDGE_PID}" ]] && kill -0 "${ROSBRIDGE_PID}" 2>/dev/null; then
    echo "[exit] stopping managed rosbridge_server pid=${ROSBRIDGE_PID}"
    kill "${ROSBRIDGE_PID}" 2>/dev/null || true
    wait "${ROSBRIDGE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

case "${START_ROSBRIDGE}" in
  auto|true)
    if port_listening "${ROSBRIDGE_PORT}"; then
      if existing_rosbridge_is_bson; then
        echo "[err] rosbridge_server is already reachable, but it appears to be BSON-only."
        echo "      Stop the stale rosbridge_server and rerun; this stack uses JSON only."
        exit 1
      fi
      echo "[boot] rosbridge_server already reachable on :${ROSBRIDGE_PORT}; using existing one"
    elif ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
      echo "[boot] starting rosbridge_server on 0.0.0.0:${ROSBRIDGE_PORT}"
      ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
        address:=0.0.0.0 port:="${ROSBRIDGE_PORT}" bson_only_mode:=false &
      ROSBRIDGE_PID="$!"
      sleep 2
    elif [[ "${START_ROSBRIDGE}" == "true" ]]; then
      echo "[err] rosbridge_server package missing. Install: sudo apt install ros-${ROS_DISTRO}-rosbridge-server"
      exit 1
    else
      echo "[warn] rosbridge_server package missing; continuing without starting it"
    fi
    ;;
  false)
    echo "[boot] START_ROSBRIDGE=false; assuming xlerobot topics are already present"
    ;;
  *)
    echo "[err] START_ROSBRIDGE must be auto, true, or false"
    exit 2
    ;;
esac

echo "============================================================"
echo "XLeRobot passive SLAM + Nav2 + map-only Foxglove"
echo "============================================================"
echo "Input topics:"
if [[ "${USE_HARDWARE_LIDAR}" == "true" ]]; then
  echo "  hardware RPLIDAR ${HARDWARE_LIDAR_SERIAL}"
else
  echo "  /xlerobot/scan"
fi
echo "  robot/wheel odom bridge: disabled"
echo "Command output:"
echo "  /cmd_vel -> /cmd_vel_mux -> /xlerobot/cmd_vel"
echo "Manual base test:"
echo "  ros2 topic pub --rate 10 /cmd_vel_teleop geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'"
echo "Nav2 goal test:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"
echo "Foxglove:"
echo "  ws://localhost:8765"
echo ""
echo "Isaac side should connect to rosbridge at this machine:${ROSBRIDGE_PORT}"
echo "For example: restart_streaming_terminal.sh --rosbridge-host <this-host> --rosbridge-port ${ROSBRIDGE_PORT} --rosbridge-wire-format json --no-keyboard"
echo "============================================================"

LAUNCH_ARGS=(
  isaac_transport:=xlerobot_ros
  ros_localhost_only:="${ROS_LOCALHOST_ONLY}"
  use_sim_time:=false
  use_hardware_lidar:="${USE_HARDWARE_LIDAR}"
  use_lidar_odom:="${USE_LIDAR_ODOM}"
  hardware_lidar_serial:="${HARDWARE_LIDAR_SERIAL}"
  hardware_lidar_baud:="${HARDWARE_LIDAR_BAUD}"
  use_foxglove:=true
  foxglove_profile:=map
  use_slam_toolbox:=true
  use_rtabmap:=false
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
