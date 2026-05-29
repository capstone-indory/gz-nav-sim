#!/usr/bin/env bash
# Manage the lightweight robot I/O agent on the Raspberry Pi over SSH.
#
# Defaults match the user's SSH target:
#   HostName lekiwi
#   User pi
#   IdentityFile ~/.ssh/indory_RasberryPi_ed25519
#
# The Pi does not run ROS 2. Its indoory_ros runtime connects back to this
# compute PC's rosbridge_server and exchanges /xlerobot/* messages with roslibpy.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ACTION="${1:-}"
if [[ -z "$ACTION" ]]; then
  echo "usage: $0 {start|stop|restart|status|sync}" >&2
  exit 2
fi
shift

: "${ROBOT_SSH_TARGET:=RasberryPi}"
: "${ROBOT_SSH_FALLBACK_TARGET:=pi@lekiwi}"
: "${ROBOT_SSH_IDENTITY:=~/.ssh/indory_RasberryPi_ed25519}"
: "${ROBOT_SSH_CONNECT_TIMEOUT:=5}"
: "${ROBOT_REMOTE_REPO:=~/indoory_ros}"
: "${ROBOT_IO_REMOTE_COMMAND:=run_xlerobot_rosbridge_io.sh}"
: "${ROBOT_IO_REMOTE_LOG:=~/indoory_ros/logs/pi_bridge_stack.log}"
: "${ROBOT_IO_PUBLISH_TF:=0}"
: "${ROBOT_LEFT_BASE_PORT:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D046415-if00}"
: "${ROBOT_RIGHT_HEAD_PORT:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14032190-if00}"
: "${ROBOT_LIDAR_SERIAL:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_12703f59806eef11ba3ee8c2c169b110-if00-port0}"
: "${ROBOT_LEFT_HAND_MOTOR_IDS:=1,2,3,4,5,6}"
: "${ROBOT_RIGHT_HAND_MOTOR_IDS:=1,2,3,4,5,6}"
: "${ROBOT_BASE_LEFT_WHEEL_ID:=7}"
: "${ROBOT_BASE_BACK_WHEEL_ID:=8}"
: "${ROBOT_BASE_RIGHT_WHEEL_ID:=9}"
: "${ROBOT_HEAD_PAN_ID:=7}"
: "${ROBOT_HEAD_TILT_ID:=8}"
: "${ROBOT_BASE_WHEEL_RADIUS_M:=0.05}"
: "${ROBOT_BASE_RADIUS_M:=0.125}"
: "${ROBOT_BASE_LEFT_SIGN:=1.0}"
: "${ROBOT_BASE_BACK_SIGN:=1.0}"
: "${ROBOT_BASE_RIGHT_SIGN:=1.0}"
: "${ROBOT_BASE_MAX_RAW_COMMAND:=3000}"
: "${ROBOT_JOINT_TARGET_TOPIC:=/xlerobot/teleop/joint_targets}"
: "${ROBOT_JOINT_STATES_TOPIC:=/xlerobot/joint_states}"
: "${ROBOT_JOINT_COMMAND_RATE_HZ:=50}"
: "${ROBOT_JOINT_STATE_RATE_HZ:=20}"
: "${ROBOT_BASE_WATCHDOG_TIMEOUT_S:=0.3}"
: "${ROBOT_IO_MAX_LINEAR_X:=0.30}"
: "${ROBOT_IO_MAX_LINEAR_Y:=0.30}"
: "${ROBOT_IO_MAX_ANGULAR_Z:=1.00}"
: "${ROBOT_IO_LOOP_HZ:=200}"
: "${ROBOT_IO_COMMAND_RATE_HZ:=50}"
: "${ROBOT_IO_FEEDBACK_RATE_HZ:=0}"
: "${ROBOT_IO_BASE_PUBLISH_ODOM:=0}"
: "${ROBOT_IO_LIDAR_RATE_HZ:=10}"
: "${ROBOT_IO_DEPTH_SENSOR_STREAM_FPS:=15}"
: "${ROBOT_IO_DEPTH_SENSOR_DEPTH_FPS:=15}"
: "${ROBOT_IO_DEPTH_SENSOR_COLOR_FPS:=15}"
: "${ROBOT_IO_DEPTH_SENSOR_DEPTH_PUBLISH_HZ:=15}"
: "${ROBOT_IO_DEPTH_SENSOR_COLOR_PUBLISH_HZ:=15}"
: "${ROBOT_IO_DEPTH_SENSOR_WIDTH:=640}"
: "${ROBOT_IO_DEPTH_SENSOR_HEIGHT:=480}"
: "${ROBOT_IO_DEPTH_SENSOR_JPEG_QUALITY:=80}"
: "${ROBOT_IO_DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR:=true}"
: "${ROBOT_IO_DEPTH_SENSOR_REQUIRE_USB3:=0}"
: "${ROBOT_RGBD_BINARY_ENABLE:=1}"
: "${ROBOT_RGBD_BINARY_PORT:=9102}"
: "${ROBOT_RGBD_BINARY_DEPTH_FORMAT:=raw16}"
: "${ROBOT_RGBD_BINARY_COLOR_MODE:=bgr8}"
: "${ROBOT_RGBD_BINARY_FPS:=0}"
: "${ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE:=0}"
: "${ROBOT_VIDEO_ENABLE:=1}"
: "${ROBOT_VIDEO_PATH:=xlerobot_head}"
: "${ROBOT_VIDEO_RTSP_PORT:=8554}"
: "${ROBOT_VIDEO_FPS:=$ROBOT_IO_DEPTH_SENSOR_STREAM_FPS}"
: "${ROBOT_VIDEO_BITRATE_KBPS:=3000}"
: "${ROBOT_VIDEO_TRANSPORT:=tcp}"
: "${ROBOT_USB_CAMERA_RTSP_ENABLE:=1}"
: "${ROBOT_USB_CAMERA_RTSP_CAMERAS:=base,wrist_left,wrist_right}"
: "${ROBOT_BASE_CAMERA_PATH:=xlerobot_base}"
: "${ROBOT_WRIST_LEFT_CAMERA_PATH:=xlerobot_wrist_left}"
: "${ROBOT_WRIST_RIGHT_CAMERA_PATH:=xlerobot_wrist_right}"
: "${ROBOT_BASE_CAMERA_DEVICE:=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0}"
: "${ROBOT_WRIST_LEFT_CAMERA_DEVICE:=/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0}"
: "${ROBOT_WRIST_RIGHT_CAMERA_DEVICE:=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.4:1.0-video-index0}"
: "${ROBOT_BASE_CAMERA_ROTATE_DEG:=0}"
: "${ROBOT_WRIST_LEFT_CAMERA_ROTATE_DEG:=180}"
: "${ROBOT_WRIST_RIGHT_CAMERA_ROTATE_DEG:=0}"
: "${ROBOT_USB_CAMERA_WIDTH:=640}"
: "${ROBOT_USB_CAMERA_HEIGHT:=480}"
: "${ROBOT_USB_CAMERA_FPS:=15}"
: "${ROBOT_USB_CAMERA_INPUT_FORMAT:=mjpeg}"
: "${ROBOT_USB_CAMERA_BITRATE_KBPS:=1500}"
: "${ROBOT_IO_BASE_STARTUP_S:=8}"
: "${ROBOT_IO_SENSOR_ONLY_ON_BASE_FAIL:=1}"
: "${ROBOT_STOP_PI_LOCAL_WEB:=1}"
: "${ROBOT_SSH_SYNC:=0}"
: "${ROBOT_SSH_RESTART_EXISTING:=1}"
: "${ROSBRIDGE_PORT:=9090}"
ROSBRIDGE_WIRE_FORMAT=json

expand_local_path() {
  local path=$1
  case "$path" in
    "~") printf '%s\n' "$HOME" ;;
    "~/"*) printf '%s/%s\n' "$HOME" "${path#\~/}" ;;
    *) printf '%s\n' "$path" ;;
  esac
}

shq() {
  printf '%q' "$1"
}

ssh_args() {
  local identity
  identity="$(expand_local_path "$ROBOT_SSH_IDENTITY")"
  printf '%s\0' \
    -o "BatchMode=yes" \
    -o "ConnectTimeout=${ROBOT_SSH_CONNECT_TIMEOUT}" \
    -o "ServerAliveInterval=5" \
    -o "ServerAliveCountMax=2" \
    -o "StrictHostKeyChecking=accept-new"
  if [[ -n "$ROBOT_SSH_IDENTITY" && -f "$identity" ]]; then
    printf '%s\0' -i "$identity"
  elif [[ -n "$ROBOT_SSH_IDENTITY" ]]; then
    echo "[warn] identity file not found, falling back to ssh defaults: $identity" >&2
  fi
}

effective_ssh_target() {
  local target=$ROBOT_SSH_TARGET
  local ssh_g hostname user

  if [[ "$target" == "RasberryPi" && -n "${ROBOT_SSH_FALLBACK_TARGET:-}" ]]; then
    ssh_g=$(ssh -G "$target" 2>/dev/null || true)
    hostname=$(awk '/^hostname / {print $2; exit}' <<<"$ssh_g")
    user=$(awk '/^user / {print $2; exit}' <<<"$ssh_g")
    if [[ "$hostname" == "rasberrypi" && "$user" != "pi" ]]; then
      target=$ROBOT_SSH_FALLBACK_TARGET
    fi
  fi

  printf '%s\n' "$target"
}

robot_hostname_for_route() {
  local target ssh_g hostname
  target="$(effective_ssh_target)"
  ssh_g=$(ssh -G "$target" 2>/dev/null || true)
  hostname=$(awk '/^hostname / {print $2; exit}' <<<"$ssh_g")
  if [[ -n "$hostname" ]]; then
    printf '%s\n' "$hostname"
    return 0
  fi
  printf '%s\n' "${target#*@}"
}

resolve_host_for_route() {
  local name=$1
  local first
  first=$(getent hosts "$name" 2>/dev/null | awk '{print $1; exit}' || true)
  if [[ -n "$first" ]]; then
    printf '%s\n' "$first"
  else
    printf '%s\n' "$name"
  fi
}

detect_compute_rosbridge_host() {
  if [[ -n "${COMPUTE_ROSBRIDGE_HOST:-}" ]]; then
    printf '%s\n' "$COMPUTE_ROSBRIDGE_HOST"
    return 0
  fi

  local robot_host route_host src
  robot_host="$(robot_hostname_for_route)"
  route_host="$(resolve_host_for_route "$robot_host")"
  src=$(ip route get "$route_host" 2>/dev/null | awk '
    {
      for (i = 1; i <= NF; i++) {
        if ($i == "src") {
          print $(i + 1)
          exit
        }
      }
    }' || true)
  if [[ -n "$src" ]]; then
    printf '%s\n' "$src"
    return 0
  fi

  src=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
  if [[ -n "$src" ]]; then
    printf '%s\n' "$src"
    return 0
  fi

  echo "[err] could not detect compute PC IP; set COMPUTE_ROSBRIDGE_HOST" >&2
  return 1
}

ssh_robot() {
  local -a args=()
  local item target
  target="$(effective_ssh_target)"
  while IFS= read -r -d '' item; do
    args+=("$item")
  done < <(ssh_args)
  ssh "${args[@]}" "$target" "$@"
}

sync_robot_io() {
  local -a args=()
  local item target
  target="$(effective_ssh_target)"
  while IFS= read -r -d '' item; do
    args+=("$item")
  done < <(ssh_args)

  echo "[ssh] syncing robot I/O files to ${target}:${ROBOT_REMOTE_REPO}"
  ssh "${args[@]}" "$target" \
    "mkdir -p $(shq "$ROBOT_REMOTE_REPO")"
  rsync -az --delete \
    --exclude 'xlerobot_robot_io.env' \
    -e "ssh ${args[*]@Q}" \
    robot \
    run_xlerobot_rosbridge_io.sh \
    run_xlerobot_robot_io.sh \
    "$target:$ROBOT_REMOTE_REPO/"
}

start_robot_io() {
  local compute_host target
  target="$(effective_ssh_target)"
  compute_host="$(detect_compute_rosbridge_host)"

  if [[ "$ROBOT_SSH_SYNC" == "1" || "$ROBOT_SSH_SYNC" == "true" ]]; then
    sync_robot_io
  fi

  echo "[ssh] target robot : $target"
  if [[ "$target" != "$ROBOT_SSH_TARGET" ]]; then
    echo "[ssh] requested    : $ROBOT_SSH_TARGET"
  fi
  echo "[ssh] remote repo  : $ROBOT_REMOTE_REPO"
  echo "[ssh] remote cmd   : $ROBOT_IO_REMOTE_COMMAND"
  echo "[ssh] rosbridge    : ws://${compute_host}:${ROSBRIDGE_PORT} (wire=json)"
  if [[ "$ROBOT_VIDEO_ENABLE" == "1" || "$ROBOT_VIDEO_ENABLE" == "true" ]]; then
    echo "[ssh] video       : rtsp://${compute_host}:${ROBOT_VIDEO_RTSP_PORT}/${ROBOT_VIDEO_PATH} -> MediaMTX WebRTC"
    if [[ "$ROBOT_USB_CAMERA_RTSP_ENABLE" == "1" || "$ROBOT_USB_CAMERA_RTSP_ENABLE" == "true" ]]; then
      echo "[ssh] usb cams    : selected=${ROBOT_USB_CAMERA_RTSP_CAMERAS} base=${ROBOT_BASE_CAMERA_DEVICE}->/${ROBOT_BASE_CAMERA_PATH}, left=${ROBOT_WRIST_LEFT_CAMERA_DEVICE}->/${ROBOT_WRIST_LEFT_CAMERA_PATH} rotate=${ROBOT_WRIST_LEFT_CAMERA_ROTATE_DEG}, right=${ROBOT_WRIST_RIGHT_CAMERA_DEVICE}->/${ROBOT_WRIST_RIGHT_CAMERA_PATH}"
    fi
  else
    echo "[ssh] video       : disabled"
  fi
  echo "[ssh] remote log   : $ROBOT_IO_REMOTE_LOG"
  echo "[ssh] motor map    : ${ROBOT_LEFT_BASE_PORT} left_hand=${ROBOT_LEFT_HAND_MOTOR_IDS} base=${ROBOT_BASE_LEFT_WHEEL_ID},${ROBOT_BASE_BACK_WHEEL_ID},${ROBOT_BASE_RIGHT_WHEEL_ID}"
  echo "[ssh] motor map    : ${ROBOT_RIGHT_HEAD_PORT} right_hand=${ROBOT_RIGHT_HAND_MOTOR_IDS} head=${ROBOT_HEAD_PAN_ID},${ROBOT_HEAD_TILT_ID}"
  echo "[ssh] lidar        : ${ROBOT_LIDAR_SERIAL}"
  echo "[ssh] joint topics : target=${ROBOT_JOINT_TARGET_TOPIC}, states=${ROBOT_JOINT_STATES_TOPIC}"
  echo "[ssh] cmd limits   : x=${ROBOT_IO_MAX_LINEAR_X} m/s, y=${ROBOT_IO_MAX_LINEAR_Y} m/s, yaw=${ROBOT_IO_MAX_ANGULAR_Z} rad/s"
  echo "[ssh] rates        : loop=${ROBOT_IO_LOOP_HZ}Hz, keepalive=${ROBOT_IO_COMMAND_RATE_HZ}Hz, base_odom=${ROBOT_IO_BASE_PUBLISH_ODOM}, lidar=${ROBOT_IO_LIDAR_RATE_HZ}Hz, depth_sensor=${ROBOT_IO_DEPTH_SENSOR_WIDTH}x${ROBOT_IO_DEPTH_SENSOR_HEIGHT}@${ROBOT_IO_DEPTH_SENSOR_COLOR_FPS}Hz depth=${ROBOT_IO_DEPTH_SENSOR_DEPTH_FPS}Hz binary_rgbd=${ROBOT_IO_DEPTH_SENSOR_COLOR_PUBLISH_HZ}/${ROBOT_IO_DEPTH_SENSOR_DEPTH_PUBLISH_HZ}Hz stream=${ROBOT_IO_DEPTH_SENSOR_STREAM_FPS}fps usb3_required=${ROBOT_IO_DEPTH_SENSOR_REQUIRE_USB3}"
  echo "[ssh] rgbd binary  : tcp://${compute_host}:${ROBOT_RGBD_BINARY_PORT} enable=${ROBOT_RGBD_BINARY_ENABLE}, image=${ROBOT_RGBD_BINARY_COLOR_MODE}, depth=${ROBOT_RGBD_BINARY_DEPTH_FORMAT}, fps=${ROBOT_RGBD_BINARY_FPS}, rosbridge_images=${ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE}"

  ssh_robot \
    "COMPUTE_ROSBRIDGE_HOST=$(shq "$compute_host") ROSBRIDGE_PORT=$(shq "$ROSBRIDGE_PORT") ROSBRIDGE_WIRE_FORMAT=json ROBOT_REMOTE_REPO=$(shq "$ROBOT_REMOTE_REPO") ROBOT_IO_REMOTE_COMMAND=$(shq "$ROBOT_IO_REMOTE_COMMAND") ROBOT_IO_REMOTE_LOG=$(shq "$ROBOT_IO_REMOTE_LOG") ROBOT_IO_PUBLISH_TF=$(shq "$ROBOT_IO_PUBLISH_TF") ROBOT_LEFT_BASE_PORT=$(shq "$ROBOT_LEFT_BASE_PORT") ROBOT_RIGHT_HEAD_PORT=$(shq "$ROBOT_RIGHT_HEAD_PORT") ROBOT_LIDAR_SERIAL=$(shq "$ROBOT_LIDAR_SERIAL") ROBOT_LEFT_HAND_MOTOR_IDS=$(shq "$ROBOT_LEFT_HAND_MOTOR_IDS") ROBOT_RIGHT_HAND_MOTOR_IDS=$(shq "$ROBOT_RIGHT_HAND_MOTOR_IDS") ROBOT_BASE_LEFT_WHEEL_ID=$(shq "$ROBOT_BASE_LEFT_WHEEL_ID") ROBOT_BASE_BACK_WHEEL_ID=$(shq "$ROBOT_BASE_BACK_WHEEL_ID") ROBOT_BASE_RIGHT_WHEEL_ID=$(shq "$ROBOT_BASE_RIGHT_WHEEL_ID") ROBOT_HEAD_PAN_ID=$(shq "$ROBOT_HEAD_PAN_ID") ROBOT_HEAD_TILT_ID=$(shq "$ROBOT_HEAD_TILT_ID") ROBOT_BASE_WHEEL_RADIUS_M=$(shq "$ROBOT_BASE_WHEEL_RADIUS_M") ROBOT_BASE_RADIUS_M=$(shq "$ROBOT_BASE_RADIUS_M") ROBOT_BASE_LEFT_SIGN=$(shq "$ROBOT_BASE_LEFT_SIGN") ROBOT_BASE_BACK_SIGN=$(shq "$ROBOT_BASE_BACK_SIGN") ROBOT_BASE_RIGHT_SIGN=$(shq "$ROBOT_BASE_RIGHT_SIGN") ROBOT_BASE_MAX_RAW_COMMAND=$(shq "$ROBOT_BASE_MAX_RAW_COMMAND") ROBOT_JOINT_TARGET_TOPIC=$(shq "$ROBOT_JOINT_TARGET_TOPIC") ROBOT_JOINT_STATES_TOPIC=$(shq "$ROBOT_JOINT_STATES_TOPIC") ROBOT_JOINT_COMMAND_RATE_HZ=$(shq "$ROBOT_JOINT_COMMAND_RATE_HZ") ROBOT_JOINT_STATE_RATE_HZ=$(shq "$ROBOT_JOINT_STATE_RATE_HZ") ROBOT_BASE_WATCHDOG_TIMEOUT_S=$(shq "$ROBOT_BASE_WATCHDOG_TIMEOUT_S") ROBOT_IO_MAX_LINEAR_X=$(shq "$ROBOT_IO_MAX_LINEAR_X") ROBOT_IO_MAX_LINEAR_Y=$(shq "$ROBOT_IO_MAX_LINEAR_Y") ROBOT_IO_MAX_ANGULAR_Z=$(shq "$ROBOT_IO_MAX_ANGULAR_Z") ROBOT_IO_LOOP_HZ=$(shq "$ROBOT_IO_LOOP_HZ") ROBOT_IO_COMMAND_RATE_HZ=$(shq "$ROBOT_IO_COMMAND_RATE_HZ") ROBOT_IO_FEEDBACK_RATE_HZ=$(shq "$ROBOT_IO_FEEDBACK_RATE_HZ") ROBOT_IO_BASE_PUBLISH_ODOM=$(shq "$ROBOT_IO_BASE_PUBLISH_ODOM") ROBOT_IO_LIDAR_RATE_HZ=$(shq "$ROBOT_IO_LIDAR_RATE_HZ") ROBOT_IO_DEPTH_SENSOR_STREAM_FPS=$(shq "$ROBOT_IO_DEPTH_SENSOR_STREAM_FPS") ROBOT_IO_DEPTH_SENSOR_DEPTH_FPS=$(shq "$ROBOT_IO_DEPTH_SENSOR_DEPTH_FPS") ROBOT_IO_DEPTH_SENSOR_COLOR_FPS=$(shq "$ROBOT_IO_DEPTH_SENSOR_COLOR_FPS") ROBOT_IO_DEPTH_SENSOR_DEPTH_PUBLISH_HZ=$(shq "$ROBOT_IO_DEPTH_SENSOR_DEPTH_PUBLISH_HZ") ROBOT_IO_DEPTH_SENSOR_COLOR_PUBLISH_HZ=$(shq "$ROBOT_IO_DEPTH_SENSOR_COLOR_PUBLISH_HZ") ROBOT_IO_DEPTH_SENSOR_WIDTH=$(shq "$ROBOT_IO_DEPTH_SENSOR_WIDTH") ROBOT_IO_DEPTH_SENSOR_HEIGHT=$(shq "$ROBOT_IO_DEPTH_SENSOR_HEIGHT") ROBOT_IO_DEPTH_SENSOR_JPEG_QUALITY=$(shq "$ROBOT_IO_DEPTH_SENSOR_JPEG_QUALITY") ROBOT_IO_DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR=$(shq "$ROBOT_IO_DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR") ROBOT_IO_DEPTH_SENSOR_REQUIRE_USB3=$(shq "$ROBOT_IO_DEPTH_SENSOR_REQUIRE_USB3") ROBOT_RGBD_BINARY_ENABLE=$(shq "$ROBOT_RGBD_BINARY_ENABLE") ROBOT_RGBD_BINARY_PORT=$(shq "$ROBOT_RGBD_BINARY_PORT") ROBOT_RGBD_BINARY_DEPTH_FORMAT=$(shq "$ROBOT_RGBD_BINARY_DEPTH_FORMAT") ROBOT_RGBD_BINARY_COLOR_MODE=$(shq "$ROBOT_RGBD_BINARY_COLOR_MODE") ROBOT_RGBD_BINARY_FPS=$(shq "$ROBOT_RGBD_BINARY_FPS") ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE=$(shq "$ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE") ROBOT_VIDEO_ENABLE=$(shq "$ROBOT_VIDEO_ENABLE") ROBOT_VIDEO_PATH=$(shq "$ROBOT_VIDEO_PATH") ROBOT_VIDEO_RTSP_PORT=$(shq "$ROBOT_VIDEO_RTSP_PORT") ROBOT_VIDEO_FPS=$(shq "$ROBOT_VIDEO_FPS") ROBOT_VIDEO_BITRATE_KBPS=$(shq "$ROBOT_VIDEO_BITRATE_KBPS") ROBOT_VIDEO_TRANSPORT=$(shq "$ROBOT_VIDEO_TRANSPORT") ROBOT_USB_CAMERA_RTSP_ENABLE=$(shq "$ROBOT_USB_CAMERA_RTSP_ENABLE") ROBOT_USB_CAMERA_RTSP_CAMERAS=$(shq "$ROBOT_USB_CAMERA_RTSP_CAMERAS") ROBOT_BASE_CAMERA_ROTATE_DEG=$(shq "$ROBOT_BASE_CAMERA_ROTATE_DEG") ROBOT_WRIST_LEFT_CAMERA_ROTATE_DEG=$(shq "$ROBOT_WRIST_LEFT_CAMERA_ROTATE_DEG") ROBOT_WRIST_RIGHT_CAMERA_ROTATE_DEG=$(shq "$ROBOT_WRIST_RIGHT_CAMERA_ROTATE_DEG") ROBOT_BASE_CAMERA_PATH=$(shq "$ROBOT_BASE_CAMERA_PATH") ROBOT_WRIST_LEFT_CAMERA_PATH=$(shq "$ROBOT_WRIST_LEFT_CAMERA_PATH") ROBOT_WRIST_RIGHT_CAMERA_PATH=$(shq "$ROBOT_WRIST_RIGHT_CAMERA_PATH") ROBOT_BASE_CAMERA_DEVICE=$(shq "$ROBOT_BASE_CAMERA_DEVICE") ROBOT_WRIST_LEFT_CAMERA_DEVICE=$(shq "$ROBOT_WRIST_LEFT_CAMERA_DEVICE") ROBOT_WRIST_RIGHT_CAMERA_DEVICE=$(shq "$ROBOT_WRIST_RIGHT_CAMERA_DEVICE") ROBOT_USB_CAMERA_WIDTH=$(shq "$ROBOT_USB_CAMERA_WIDTH") ROBOT_USB_CAMERA_HEIGHT=$(shq "$ROBOT_USB_CAMERA_HEIGHT") ROBOT_USB_CAMERA_FPS=$(shq "$ROBOT_USB_CAMERA_FPS") ROBOT_USB_CAMERA_INPUT_FORMAT=$(shq "$ROBOT_USB_CAMERA_INPUT_FORMAT") ROBOT_USB_CAMERA_BITRATE_KBPS=$(shq "$ROBOT_USB_CAMERA_BITRATE_KBPS") ROBOT_IO_BASE_STARTUP_S=$(shq "$ROBOT_IO_BASE_STARTUP_S") ROBOT_IO_SENSOR_ONLY_ON_BASE_FAIL=$(shq "$ROBOT_IO_SENSOR_ONLY_ON_BASE_FAIL") ROBOT_STOP_PI_LOCAL_WEB=$(shq "$ROBOT_STOP_PI_LOCAL_WEB") ROBOT_SSH_RESTART_EXISTING=$(shq "$ROBOT_SSH_RESTART_EXISTING") bash -s" <<'REMOTE'
set -euo pipefail
process_pattern='robot/xlerobot_rosbridge_io.py|scripts/pi_rosbridge_client.py|scripts/start_pi_rosbridge_client.sh|scripts/start_pi_rosbridge_client_once.sh|scripts/start_pi_bridge_stack.sh|scripts/xlerobot_base_host.py|ffmpeg .*rtsp://.*xlerobot_'
pi_local_web_pattern='[x]lerobot_mobile_web.py'

case "$ROBOT_REMOTE_REPO" in
  "~") ROBOT_REMOTE_REPO="$HOME" ;;
  "~/"*) ROBOT_REMOTE_REPO="$HOME/${ROBOT_REMOTE_REPO#\~/}" ;;
esac
case "$ROBOT_IO_REMOTE_LOG" in
  "~") ROBOT_IO_REMOTE_LOG="$HOME/xlerobot_rosbridge_io.log" ;;
  "~/"*) ROBOT_IO_REMOTE_LOG="$HOME/${ROBOT_IO_REMOTE_LOG#\~/}" ;;
esac

if [[ ! -d "$ROBOT_REMOTE_REPO" ]]; then
  echo "[remote err] repo not found: $ROBOT_REMOTE_REPO"
  echo "[remote err] run with ROBOT_SSH_SYNC=1, or copy this repo to the Pi first"
  exit 1
fi
cd "$ROBOT_REMOTE_REPO"

if [[ "$ROBOT_STOP_PI_LOCAL_WEB" == "1" || "$ROBOT_STOP_PI_LOCAL_WEB" == "true" ]]; then
  mapfile -t pi_local_web_pids < <(pgrep -f "$pi_local_web_pattern" 2>/dev/null || true)
  if (( ${#pi_local_web_pids[@]} > 0 )); then
    echo "[remote] stopping Pi-local teleoperation web; compute web owns robot UI"
    for pid in "${pi_local_web_pids[@]}"; do
      kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 2
    mapfile -t pi_local_web_pids < <(pgrep -f "$pi_local_web_pattern" 2>/dev/null || true)
    for pid in "${pi_local_web_pids[@]}"; do
      kill -KILL "$pid" 2>/dev/null || true
    done
  fi
fi

remote_command="$ROBOT_IO_REMOTE_COMMAND"
if [[ "$remote_command" != /* && "$remote_command" != ./* ]]; then
  remote_command="./$remote_command"
fi

if [[ ! -x "$remote_command" ]]; then
  echo "[remote err] missing executable: $remote_command"
  echo "[remote err] set ROBOT_IO_REMOTE_COMMAND to the Pi-side start script"
  exit 1
fi

if [[ "$ROBOT_SSH_RESTART_EXISTING" == "1" || "$ROBOT_SSH_RESTART_EXISTING" == "true" ]]; then
  pkill -TERM -f "$process_pattern" 2>/dev/null || true
  sleep 1
  if pgrep -f "$process_pattern" >/dev/null 2>&1; then
    pkill -KILL -f "$process_pattern" 2>/dev/null || true
    sleep 1
  fi
fi

if pgrep -f "$process_pattern" >/dev/null 2>&1; then
  echo "[remote] robot I/O already running:"
  pgrep -af "$process_pattern" || true
  exit 0
fi

mkdir -p "$(dirname "$ROBOT_IO_REMOTE_LOG")"
nohup env \
  COMPUTE_PC_HOST="$COMPUTE_ROSBRIDGE_HOST" \
  ROSBRIDGE_HOST="$COMPUTE_ROSBRIDGE_HOST" \
  ROSBRIDGE_PORT="$ROSBRIDGE_PORT" \
  ROSBRIDGE_WIRE_FORMAT="json" \
  ROSBRIDGE_URL="ws://${COMPUTE_ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}" \
  ALLOW_SENSOR_ONLY_ON_BASE_FAIL="$ROBOT_IO_SENSOR_ONLY_ON_BASE_FAIL" \
  BASE_STARTUP_S="$ROBOT_IO_BASE_STARTUP_S" \
  CMD_TOPIC="/xlerobot/cmd_vel" \
  JOINT_TARGET_TOPIC="$ROBOT_JOINT_TARGET_TOPIC" \
  JOINT_STATES_TOPIC="$ROBOT_JOINT_STATES_TOPIC" \
  LEFT_BASE_PORT="$ROBOT_LEFT_BASE_PORT" \
  BASE_PORT="$ROBOT_LEFT_BASE_PORT" \
  RIGHT_HEAD_PORT="$ROBOT_RIGHT_HEAD_PORT" \
  LEFT_HAND_MOTOR_IDS="$ROBOT_LEFT_HAND_MOTOR_IDS" \
  RIGHT_HAND_MOTOR_IDS="$ROBOT_RIGHT_HAND_MOTOR_IDS" \
  BASE_LEFT_WHEEL_ID="$ROBOT_BASE_LEFT_WHEEL_ID" \
  BASE_BACK_WHEEL_ID="$ROBOT_BASE_BACK_WHEEL_ID" \
  BASE_RIGHT_WHEEL_ID="$ROBOT_BASE_RIGHT_WHEEL_ID" \
  HEAD_PAN_ID="$ROBOT_HEAD_PAN_ID" \
  HEAD_TILT_ID="$ROBOT_HEAD_TILT_ID" \
  BASE_WHEEL_RADIUS_M="$ROBOT_BASE_WHEEL_RADIUS_M" \
  BASE_RADIUS_M="$ROBOT_BASE_RADIUS_M" \
  BASE_LEFT_SIGN="$ROBOT_BASE_LEFT_SIGN" \
  BASE_BACK_SIGN="$ROBOT_BASE_BACK_SIGN" \
  BASE_RIGHT_SIGN="$ROBOT_BASE_RIGHT_SIGN" \
  BASE_MAX_RAW_COMMAND="$ROBOT_BASE_MAX_RAW_COMMAND" \
  JOINT_COMMAND_RATE_HZ="$ROBOT_JOINT_COMMAND_RATE_HZ" \
  JOINT_STATE_RATE_HZ="$ROBOT_JOINT_STATE_RATE_HZ" \
  BASE_WATCHDOG_TIMEOUT_S="$ROBOT_BASE_WATCHDOG_TIMEOUT_S" \
  BASE_COMMAND_RATE_HZ="$ROBOT_IO_LOOP_HZ" \
  BASE_FEEDBACK_RATE_HZ="$ROBOT_IO_FEEDBACK_RATE_HZ" \
  BASE_PUBLISH_ODOM="$ROBOT_IO_BASE_PUBLISH_ODOM" \
  LOOP_HZ="$ROBOT_IO_LOOP_HZ" \
  COMMAND_RATE_HZ="$ROBOT_IO_COMMAND_RATE_HZ" \
  MAX_LOOP_HZ="$ROBOT_IO_LOOP_HZ" \
  BASE_OBS_RATE_HZ="$ROBOT_IO_FEEDBACK_RATE_HZ" \
  LIDAR_SERIAL="$ROBOT_LIDAR_SERIAL" \
  LIDAR_SAMPLES="360" \
  LIDAR_PUBLISH_RATE_HZ="$ROBOT_IO_LIDAR_RATE_HZ" \
  SCAN_RATE_HZ="$ROBOT_IO_LIDAR_RATE_HZ" \
  SCAN_BINS="360" \
  DEPTH_SENSOR_DEPTH_FPS="$ROBOT_IO_DEPTH_SENSOR_DEPTH_FPS" \
  DEPTH_SENSOR_COLOR_FPS="$ROBOT_IO_DEPTH_SENSOR_COLOR_FPS" \
  DEPTH_SENSOR_DEPTH_PUBLISH_HZ="$ROBOT_IO_DEPTH_SENSOR_DEPTH_PUBLISH_HZ" \
  DEPTH_SENSOR_COLOR_PUBLISH_HZ="$ROBOT_IO_DEPTH_SENSOR_COLOR_PUBLISH_HZ" \
  DEPTH_SENSOR_DEPTH_WIDTH="$ROBOT_IO_DEPTH_SENSOR_WIDTH" \
  DEPTH_SENSOR_DEPTH_HEIGHT="$ROBOT_IO_DEPTH_SENSOR_HEIGHT" \
  DEPTH_SENSOR_COLOR_WIDTH="$ROBOT_IO_DEPTH_SENSOR_WIDTH" \
  DEPTH_SENSOR_COLOR_HEIGHT="$ROBOT_IO_DEPTH_SENSOR_HEIGHT" \
  DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR="$ROBOT_IO_DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR" \
  DEPTH_SENSOR_REQUIRE_USB3="$ROBOT_IO_DEPTH_SENSOR_REQUIRE_USB3" \
  DEPTH_SENSOR_ENABLE_COLOR="1" \
  DEPTH_SENSOR_ENABLE_IMU="1" \
  DEPTH_SENSOR_JPEG_QUALITY="$ROBOT_IO_DEPTH_SENSOR_JPEG_QUALITY" \
  DEPTH_SENSOR_PNG_COMPRESS_LEVEL="1" \
  DEPTH_SENSOR_BINARY_ENABLE="$ROBOT_RGBD_BINARY_ENABLE" \
  DEPTH_SENSOR_BINARY_HOST="$COMPUTE_ROSBRIDGE_HOST" \
  DEPTH_SENSOR_BINARY_PORT="$ROBOT_RGBD_BINARY_PORT" \
  DEPTH_SENSOR_BINARY_DEPTH_FORMAT="$ROBOT_RGBD_BINARY_DEPTH_FORMAT" \
  DEPTH_SENSOR_BINARY_COLOR_MODE="$ROBOT_RGBD_BINARY_COLOR_MODE" \
  DEPTH_SENSOR_BINARY_FPS="$ROBOT_RGBD_BINARY_FPS" \
  DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE="$ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE" \
  ENABLE_CAMERA="0" \
  ENABLE_DEPTH_SENSOR="1" \
  DEPTH_SENSOR_RTSP_ENABLE="$ROBOT_VIDEO_ENABLE" \
  DEPTH_SENSOR_RTSP_URL="rtsp://${COMPUTE_ROSBRIDGE_HOST}:${ROBOT_VIDEO_RTSP_PORT}/${ROBOT_VIDEO_PATH}" \
  DEPTH_SENSOR_RTSP_FPS="$ROBOT_VIDEO_FPS" \
  DEPTH_SENSOR_RTSP_BITRATE_KBPS="$ROBOT_VIDEO_BITRATE_KBPS" \
  DEPTH_SENSOR_RTSP_TRANSPORT="$ROBOT_VIDEO_TRANSPORT" \
  ENABLE_USB_CAMERA_RTSP="$ROBOT_USB_CAMERA_RTSP_ENABLE" \
  USB_CAMERA_RTSP_ENABLE="$ROBOT_USB_CAMERA_RTSP_ENABLE" \
  USB_CAMERA_RTSP_CAMERAS="$ROBOT_USB_CAMERA_RTSP_CAMERAS" \
  USB_CAMERA_WIDTH="$ROBOT_USB_CAMERA_WIDTH" \
  USB_CAMERA_HEIGHT="$ROBOT_USB_CAMERA_HEIGHT" \
  USB_CAMERA_FPS="$ROBOT_USB_CAMERA_FPS" \
  USB_CAMERA_INPUT_FORMAT="$ROBOT_USB_CAMERA_INPUT_FORMAT" \
  USB_CAMERA_RTSP_BITRATE_KBPS="$ROBOT_USB_CAMERA_BITRATE_KBPS" \
  BASE_CAMERA_DEVICE="$ROBOT_BASE_CAMERA_DEVICE" \
  BASE_CAMERA_ROTATE_DEG="$ROBOT_BASE_CAMERA_ROTATE_DEG" \
  BASE_CAMERA_RTSP_URL="rtsp://${COMPUTE_ROSBRIDGE_HOST}:${ROBOT_VIDEO_RTSP_PORT}/${ROBOT_BASE_CAMERA_PATH}" \
  WRIST_LEFT_CAMERA_DEVICE="$ROBOT_WRIST_LEFT_CAMERA_DEVICE" \
  WRIST_LEFT_CAMERA_ROTATE_DEG="$ROBOT_WRIST_LEFT_CAMERA_ROTATE_DEG" \
  WRIST_LEFT_CAMERA_RTSP_URL="rtsp://${COMPUTE_ROSBRIDGE_HOST}:${ROBOT_VIDEO_RTSP_PORT}/${ROBOT_WRIST_LEFT_CAMERA_PATH}" \
  WRIST_RIGHT_CAMERA_DEVICE="$ROBOT_WRIST_RIGHT_CAMERA_DEVICE" \
  WRIST_RIGHT_CAMERA_ROTATE_DEG="$ROBOT_WRIST_RIGHT_CAMERA_ROTATE_DEG" \
  WRIST_RIGHT_CAMERA_RTSP_URL="rtsp://${COMPUTE_ROSBRIDGE_HOST}:${ROBOT_VIDEO_RTSP_PORT}/${ROBOT_WRIST_RIGHT_CAMERA_PATH}" \
  PUBLISH_TF="$ROBOT_IO_PUBLISH_TF" \
  MAX_LINEAR_X="$ROBOT_IO_MAX_LINEAR_X" \
  MAX_LINEAR_Y="$ROBOT_IO_MAX_LINEAR_Y" \
  MAX_ANGULAR_Z="$ROBOT_IO_MAX_ANGULAR_Z" \
  RESTART_ROSBRIDGE_CLIENT="$ROBOT_SSH_RESTART_EXISTING" \
  FORCE="$ROBOT_SSH_RESTART_EXISTING" \
  "$remote_command" \
  >"$ROBOT_IO_REMOTE_LOG" 2>&1 < /dev/null &
pid=$!
echo "$pid" > /tmp/xlerobot_rosbridge_io.pid
echo "[remote] started robot I/O pid=$pid"
echo "[remote] tail log: tail -f $ROBOT_IO_REMOTE_LOG"
REMOTE
}

stop_robot_io() {
  echo "[ssh] stopping robot I/O on $(effective_ssh_target)"
  ssh_robot "bash -s" <<'REMOTE'
set -euo pipefail
if [[ -f /tmp/xlerobot_rosbridge_io.pid ]]; then
  pid=$(cat /tmp/xlerobot_rosbridge_io.pid 2>/dev/null || true)
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    sleep 1
  fi
fi
pattern='robot/xlerobot_rosbridge_io.py|scripts/pi_rosbridge_client.py|scripts/start_pi_rosbridge_client.sh|scripts/start_pi_rosbridge_client_once.sh|scripts/start_pi_bridge_stack.sh|scripts/xlerobot_base_host.py'
pkill -TERM -f "$pattern" 2>/dev/null || true
sleep 1
if pgrep -f "$pattern" >/dev/null 2>&1; then
  pkill -KILL -f "$pattern" 2>/dev/null || true
fi
rm -f /tmp/xlerobot_rosbridge_io.pid
echo "[remote] robot I/O stopped"
REMOTE
}

status_robot_io() {
  ssh_robot "bash -s" <<'REMOTE'
set -euo pipefail
pattern='robot/xlerobot_rosbridge_io.py|scripts/pi_rosbridge_client.py|scripts/start_pi_rosbridge_client.sh|scripts/start_pi_rosbridge_client_once.sh|scripts/start_pi_bridge_stack.sh'
if pgrep -af "$pattern" >/dev/null 2>&1; then
  pgrep -af "$pattern"
else
  echo "[remote] robot I/O is not running"
fi
REMOTE
}

case "$ACTION" in
  start) start_robot_io "$@" ;;
  stop) stop_robot_io "$@" ;;
  restart)
    stop_robot_io "$@" || true
    start_robot_io "$@"
    ;;
  status) status_robot_io "$@" ;;
  sync) sync_robot_io "$@" ;;
  *)
    echo "usage: $0 {start|stop|restart|status|sync}" >&2
    exit 2
    ;;
esac
