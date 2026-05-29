#!/usr/bin/env bash
# 멀티세션 SLAM/Nav 스택을 한 번에 기동.
#
#   기본/Isaac 모드: ROS2 시뮬 + ros_adapter + Spring Boot + React + Postgres.
#   hardware 모드: 원격 로봇 I/O와 rosbridge 로 붙고, 웹 관제까지 기본 기동.
#
# 종료: Ctrl-C 한 번. 자식 프로세스 그룹 전체 SIGTERM.
#
# 사용법:
#   ./run_multisession_slam.sh                 # XLeRobot Hospital Isaac v2 의
#                                                /xlerobot ROS 토픽과 연결.
#   ./run_multisession_slam.sh hardware        # 실제 로봇 컴퓨터의 /xlerobot 토픽과 연동.
#                                                기본은 웹/DB까지 같이 기동.
#                                                기본 SSH 대상은 pi@lekiwi.
#                                                수동 Pi 실행은 ROBOT_SSH_AUTOSTART=0.
#   ./run_multisession_slam.sh hardware --no-web
#                                                SLAM/Nav/Foxglove만 가볍게 띄울 때.
#   ./run_multisession_slam.sh local-lidar     # 이 PC에 USB RPLIDAR를 꽂은 bench 모드.
#   ./run_multisession_slam.sh --no-frontend   # 프론트 없이 (이미 띄워둔 경우)
#   ./run_multisession_slam.sh --no-postgres   # postgres 외부에서 관리
#   ./run_multisession_slam.sh --no-backend    # 시뮬+adapter 만 (REST 직접 테스트)
#   SIM_DURATION=120 ./run_multisession_slam.sh   # 시뮬에 자동 종료 시간 (디버깅)
#
# Isaac v2 모드 예:
#   ./run_multisession_slam.sh isaac
#   (ROS PC에서 rosbridge_server 가 떠 있고 Isaac 앱이 그 rosbridge 에 붙어
#    /xlerobot 토픽을 publish/subscribe 해야 함)
#
# 로그: bench/runs/<ts>_multisession/{sim.log,adapter.log,backend.log,frontend.log}

set -euo pipefail

# DOMAIN_ID 도 명시 — default 0 은 충돌 위험. 환경변수로만 override.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}
: "${ROS_DISTRO:=humble}"
: "${XLE_COMPUTE_ENV:=gz-nav-humble}"
if [[ -z "${ROS_SETUP:-}" && -n "${CONDA_PREFIX:-}" && -f "$CONDA_PREFIX/setup.bash" ]]; then
    ROS_SETUP="$CONDA_PREFIX/setup.bash"
elif [[ -z "${ROS_SETUP:-}" && -f "$HOME/micromamba/envs/$XLE_COMPUTE_ENV/setup.bash" ]]; then
    ROS_SETUP="$HOME/micromamba/envs/$XLE_COMPUTE_ENV/setup.bash"
else
    : "${ROS_SETUP:=/opt/ros/${ROS_DISTRO}/setup.bash}"
fi
ROS_ENV_PREFIX=""
if [[ $ROS_SETUP == */setup.bash ]]; then
    ROS_ENV_PREFIX=${ROS_SETUP%/setup.bash}
    if [[ -d "$ROS_ENV_PREFIX/bin" ]]; then
        case ":$PATH:" in
            *":$ROS_ENV_PREFIX/bin:"*) ;;
            *) export PATH="$ROS_ENV_PREFIX/bin:$PATH" ;;
        esac
    fi
fi
export ROS_DISTRO XLE_COMPUTE_ENV ROS_SETUP ROS_ENV_PREFIX

cd "$(dirname "$0")"
ROOT=$PWD
WEB_ROOT="$ROOT/indoors-web"
: "${ISAAC_SIM_PROJECT:=/home/indory/isaacsim/user_projects/xlerobot_hospital}"
: "${ISAAC_SIM_LAUNCH:=$ISAAC_SIM_PROJECT/scripts/start_streaming.sh}"
: "${RESTART_EXISTING_ISAAC_AFTER_ROSBRIDGE:=0}"
if [[ -z "${WORKSPACE_SETUP:-}" ]]; then
    if [[ -f "$ROOT/install/setup.bash" ]]; then
        WORKSPACE_SETUP="install/setup.bash"
    else
        WORKSPACE_SETUP="install/setup.sh"
    fi
fi
export WORKSPACE_SETUP

# ── 옵션 파싱 ──────────────────────────────────────────────────────────
WANT_FRONTEND=1
WANT_BACKEND=1
WANT_POSTGRES=1
WANT_SIM=1
WANT_ADAPTER=1
WITH_WEB_EXPLICIT=0
NO_WEB_EXPLICIT=0
SIM_MODE="isaac"
SIM_DURATION="${SIM_DURATION:-}"     # 빈 값이면 무한정 (Ctrl-C 까지)

while [[ $# -gt 0 ]]; do
    case "$1" in
        isaac) SIM_MODE="isaac" ;;
        hardware) SIM_MODE="hardware" ;;
        local-lidar|local_hardware|bench-lidar) SIM_MODE="local_lidar" ;;
        --no-frontend) WANT_FRONTEND=0 ;;
        --no-backend)  WANT_BACKEND=0 ;;
        --no-postgres) WANT_POSTGRES=0 ;;
        --no-sim)      WANT_SIM=0 ;;
        --no-adapter)  WANT_ADAPTER=0 ;;
        --with-web)    WITH_WEB_EXPLICIT=1 ;;
        --no-web|--nav-only|--headless)
                       NO_WEB_EXPLICIT=1 ;;
        -h|--help)
            sed -n '2,26p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
    shift
done

case "$SIM_MODE" in
    isaac)
        SIM_PRESET="depth_sensor_isaac"
        : "${ROS_LOCALHOST_ONLY:=1}"
        : "${ISAAC_TRANSPORT:=rosbridge_v2}"
        : "${ROSBRIDGE_HOST:=0.0.0.0}"
        : "${ROSBRIDGE_PORT:=9090}"
        : "${ODOM_READY_TIMEOUT_SEC:=0}"
        : "${SCAN_READY_TIMEOUT_SEC:=0}"
        echo "[boot] sim backend: isaac-v2  (preset: $SIM_PRESET)"
        ;;
    hardware)
        SIM_PRESET="depth_sensor_hardware"
        : "${ROBOT_IO_LINK:=rosbridge}"
        case "$ROBOT_IO_LINK" in
            rosbridge)
                ROS_LOCALHOST_ONLY=1
                : "${START_ROSBRIDGE:=1}"
                : "${ROSBRIDGE_REQUIRED:=1}"
                : "${ROSBRIDGE_TOPICS_GLOB:=['/xlerobot/*']}"
                ;;
            dds)
                : "${ROS_LOCALHOST_ONLY:=0}"
                : "${START_ROSBRIDGE:=0}"
                : "${ROSBRIDGE_REQUIRED:=0}"
                ;;
            *)
                echo "[err] unknown ROBOT_IO_LINK: $ROBOT_IO_LINK (use rosbridge or dds)" >&2
                exit 2
                ;;
        esac
        : "${ISAAC_TRANSPORT:=xlerobot_ros}"
        : "${ROSBRIDGE_HOST:=0.0.0.0}"
        : "${ROSBRIDGE_PORT:=9090}"
        : "${ROBOT_SSH_AUTOSTART:=1}"
        : "${ROBOT_SSH_TARGET:=RasberryPi}"
        : "${ROBOT_SSH_FALLBACK_TARGET:=pi@lekiwi}"
        : "${ROBOT_SSH_IDENTITY:=~/.ssh/indory_RasberryPi_ed25519}"
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
        : "${CMD_MAX_LINEAR_X:=$ROBOT_IO_MAX_LINEAR_X}"
        : "${CMD_MAX_LINEAR_Y:=$ROBOT_IO_MAX_LINEAR_Y}"
        : "${CMD_MAX_ANGULAR_Z:=$ROBOT_IO_MAX_ANGULAR_Z}"
        : "${ROBOT_STOP_PI_LOCAL_WEB:=1}"
        : "${ROBOT_SSH_SYNC:=1}"
        : "${ROBOT_SSH_REQUIRED:=0}"
        : "${HARDWARE_WITH_WEB:=1}"
        ROBOT_VIDEO_ENABLE_EXPLICIT="${ROBOT_VIDEO_ENABLE+x}"
        : "${MEDIAMTX_ENABLE:=1}"
        : "${MEDIAMTX_RTSP_PORT:=8554}"
        : "${MEDIAMTX_WEBRTC_PORT:=8889}"
        : "${ROBOT_VIDEO_ENABLE:=1}"
        : "${ROBOT_VIDEO_PATH:=xlerobot_head}"
        : "${ROBOT_BASE_CAMERA_PATH:=xlerobot_base}"
        : "${ROBOT_WRIST_LEFT_CAMERA_PATH:=xlerobot_wrist_left}"
        : "${ROBOT_WRIST_RIGHT_CAMERA_PATH:=xlerobot_wrist_right}"
        : "${MEDIAMTX_PATHS:=${ROBOT_VIDEO_PATH},${ROBOT_BASE_CAMERA_PATH},${ROBOT_WRIST_LEFT_CAMERA_PATH},${ROBOT_WRIST_RIGHT_CAMERA_PATH}}"
        : "${ROBOT_VIDEO_RTSP_PORT:=$MEDIAMTX_RTSP_PORT}"
        : "${ROBOT_VIDEO_FPS:=15}"
        : "${ROBOT_VIDEO_BITRATE_KBPS:=3000}"
        : "${ROBOT_USB_CAMERA_RTSP_ENABLE:=1}"
        : "${ROBOT_BASE_CAMERA_DEVICE:=/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0}"
        : "${ROBOT_WRIST_LEFT_CAMERA_DEVICE:=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0}"
        : "${ROBOT_WRIST_RIGHT_CAMERA_DEVICE:=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.4:1.0-video-index0}"
        : "${ROBOT_USB_CAMERA_WIDTH:=640}"
        : "${ROBOT_USB_CAMERA_HEIGHT:=480}"
        : "${ROBOT_USB_CAMERA_FPS:=15}"
        : "${ROBOT_USB_CAMERA_INPUT_FORMAT:=mjpeg}"
        : "${ROBOT_USB_CAMERA_BITRATE_KBPS:=1500}"
        : "${ROBOT_RGBD_BINARY_ENABLE:=1}"
        : "${ROBOT_RGBD_BINARY_PORT:=9102}"
        : "${ROBOT_RGBD_BINARY_DEPTH_FORMAT:=raw16}"
        : "${ROBOT_RGBD_BINARY_COLOR_MODE:=bgr8}"
        : "${ROBOT_RGBD_BINARY_FPS:=0}"
        : "${ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE:=0}"
        : "${USE_BINARY_RGBD_BRIDGE:=true}"
        : "${BINARY_RGBD_HOST:=0.0.0.0}"
        : "${BINARY_RGBD_PORT:=$ROBOT_RGBD_BINARY_PORT}"
        : "${CAMERA_VIDEO_MODE:=rtsp-webrtc}"
        : "${CAMERA_VIDEO_HEAD_PATH:=$ROBOT_VIDEO_PATH}"
        : "${CAMERA_VIDEO_BASE_PATH:=$ROBOT_BASE_CAMERA_PATH}"
        : "${CAMERA_VIDEO_WRIST_LEFT_PATH:=$ROBOT_WRIST_LEFT_CAMERA_PATH}"
        : "${CAMERA_VIDEO_WRIST_RIGHT_PATH:=$ROBOT_WRIST_RIGHT_CAMERA_PATH}"
        : "${CAMERA_VIDEO_WEBRTC_PORT:=$MEDIAMTX_WEBRTC_PORT}"
        : "${CAMERA_VIDEO_WEBRTC_SCHEME:=http}"
        if [[ -z "${CAMERA_VIDEO_WEBRTC_HOST:-}" ]]; then
            CAMERA_VIDEO_WEBRTC_HOST="$(tailscale ip -4 2>/dev/null | head -n1 || true)"
        fi
        if [[ -z "${CAMERA_VIDEO_WEBRTC_HOST:-}" ]]; then
            CAMERA_VIDEO_WEBRTC_HOST="$(hostname -I 2>/dev/null | awk '{print $1}' || true)"
        fi
        if [[ $WITH_WEB_EXPLICIT == 1 ]]; then
            HARDWARE_WITH_WEB=1
        fi
        if [[ $NO_WEB_EXPLICIT == 1 ]]; then
            HARDWARE_WITH_WEB=0
            if [[ -z "$ROBOT_VIDEO_ENABLE_EXPLICIT" ]]; then
                ROBOT_VIDEO_ENABLE=0
                MEDIAMTX_ENABLE=0
            fi
        fi
        echo "[boot] hardware backend: remote robot I/O via ${ROBOT_IO_LINK}  (preset: $SIM_PRESET)"
        echo "[boot] expected robot topics: /xlerobot/scan, /xlerobot/io_status, /xlerobot/joint_states, optional /xlerobot/head_camera/imu"
        echo "[boot] command ingress: /cmd_vel, /cmd_vel_teleop, /cmd_vel_mux -> /xlerobot/cmd_vel"
        echo "[boot] joint target input: ${ROBOT_JOINT_TARGET_TOPIC}"
        echo "[boot] motor buses: ${ROBOT_LEFT_BASE_PORT} left_hand=${ROBOT_LEFT_HAND_MOTOR_IDS} base=${ROBOT_BASE_LEFT_WHEEL_ID},${ROBOT_BASE_BACK_WHEEL_ID},${ROBOT_BASE_RIGHT_WHEEL_ID}; ${ROBOT_RIGHT_HEAD_PORT} right_hand=${ROBOT_RIGHT_HAND_MOTOR_IDS} head=${ROBOT_HEAD_PAN_ID},${ROBOT_HEAD_TILT_ID}"
        echo "[boot] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
        if [[ $ROBOT_IO_LINK == rosbridge ]]; then
            echo "[boot] rosbridge: ws://<this-compute-pc-ip>:${ROSBRIDGE_PORT}"
            echo "[boot] rosbridge topic allowlist: ${ROSBRIDGE_TOPICS_GLOB:-<all>}"
            if [[ "${ROBOT_SSH_AUTOSTART:-0}" == "1" || "${ROBOT_SSH_AUTOSTART:-0}" == "true" ]]; then
                echo "[boot] robot computer autostart: ${ROBOT_SSH_TARGET} (${ROBOT_REMOTE_REPO}: ${ROBOT_IO_REMOTE_COMMAND})"
            else
                echo "[boot] robot computer should run: cd ${ROBOT_REMOTE_REPO} && ${ROBOT_IO_REMOTE_COMMAND}"
            fi
            echo "[boot] scan path: /xlerobot/scan -> /scan_raw -> filtered /scan for Nav2"
            echo "[boot] SLAM: RTAB fusion = RGB-D visual odom + /xlerobot/scan occupancy refinement"
            echo "[boot] video: Pi RTSP/H.264 -> MediaMTX :${MEDIAMTX_RTSP_PORT}/${ROBOT_VIDEO_PATH} -> WebRTC :${MEDIAMTX_WEBRTC_PORT}/${ROBOT_VIDEO_PATH}"
            echo "[boot] RGB-D: Pi TCP binary depth/RGB -> :${BINARY_RGBD_PORT} -> ROS 2 camera topics; rosbridge image JSON=${ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE}"
            echo "[boot] goals: Foxglove /goal_pose or /nav/destination, /nav/goal_pose2d"
            echo "[boot] command safety clamp: x<=${ROBOT_IO_MAX_LINEAR_X} m/s, y<=${ROBOT_IO_MAX_LINEAR_Y} m/s, yaw<=${ROBOT_IO_MAX_ANGULAR_Z} rad/s"
        else
            echo "[boot] robot computer should publish /xlerobot/* through ROS 2 DDS"
        fi
        case "${HARDWARE_WITH_WEB}" in
            1|true|TRUE|yes|YES|on|ON)
                echo "[boot] hardware mode: web stack enabled (adapter/backend/frontend/postgres)"
                echo "[boot] use './run_multisession_slam.sh hardware --no-web' for SLAM/Nav/Foxglove only"
                ;;
            0|false|FALSE|no|NO|off|OFF)
                WANT_ADAPTER=0
                WANT_BACKEND=0
                WANT_FRONTEND=0
                WANT_POSTGRES=0
                echo "[boot] hardware mode: web stack disabled"
                echo "[boot] use './run_multisession_slam.sh hardware' to include the web stack"
                ;;
            *)
                echo "[err] HARDWARE_WITH_WEB must be 1/0 or true/false (got: ${HARDWARE_WITH_WEB})" >&2
                exit 2
                ;;
        esac
        if [[ $WANT_ADAPTER == 0 && $WANT_BACKEND == 0 && $WANT_FRONTEND == 0 ]]; then
            WANT_ADAPTER=0
            WANT_BACKEND=0
            WANT_FRONTEND=0
            WANT_POSTGRES=0
        fi
        ;;
    local_lidar)
        SIM_PRESET="depth_sensor_local_lidar"
        : "${ROS_LOCALHOST_ONLY:=0}"
        : "${ISAAC_TRANSPORT:=xlerobot_ros}"
        : "${HARDWARE_LIDAR_SERIAL:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_12703f59806eef11ba3ee8c2c169b110-if00-port0}"
        : "${HARDWARE_LIDAR_BAUD:=460800}"
        echo "[boot] local USB lidar bench backend  (preset: $SIM_PRESET)"
        echo "[boot] local lidar: ${HARDWARE_LIDAR_SERIAL} @ ${HARDWARE_LIDAR_BAUD}"
        if [[ ! -e "$HARDWARE_LIDAR_SERIAL" ]]; then
            echo "[warn] local lidar device not found yet: $HARDWARE_LIDAR_SERIAL"
            echo "[warn] rplidar node will keep reconnecting until it appears"
        fi
        ;;
    *)
        echo "[err] unknown SIM_MODE: $SIM_MODE" >&2
        exit 2
        ;;
esac
: "${ROSBRIDGE_HOST:=0.0.0.0}"
: "${ROSBRIDGE_PORT:=9090}"
ROSBRIDGE_WIRE_FORMAT=json
ROS_CAMERA=1
ROS_PUBLISH_HZ=10
ROS_LIDAR_FPS=8
ROS_CAMERA_TRANSPORT=compressed
if [[ $SIM_MODE == isaac ]]; then
    : "${USE_DA3:=true}"
    : "${USE_NVBLOX:=true}"
    : "${USE_SEMANTIC_OCR:=true}"
    : "${USE_RTABMAP:=true}"
    : "${USE_SLAM_TOOLBOX:=false}"
    : "${RTABMAP_ODOM_SOURCE:=fusion}"
    : "${USE_IMU:=true}"
    : "${DIRECT_DEPTH:=true}"
elif [[ $SIM_MODE == hardware ]]; then
    : "${USE_DA3:=false}"
    : "${USE_NVBLOX:=false}"
    : "${USE_SEMANTIC_OCR:=false}"
    : "${USE_RTABMAP:=true}"
    : "${USE_SLAM_TOOLBOX:=false}"
    : "${RTABMAP_ODOM_SOURCE:=fusion}"
    : "${USE_IMU:=false}"
    : "${DIRECT_DEPTH:=true}"
else
    : "${USE_DA3:=false}"
    : "${USE_NVBLOX:=false}"
    : "${USE_SEMANTIC_OCR:=false}"
    : "${USE_RTABMAP:=false}"
    : "${USE_SLAM_TOOLBOX:=true}"
    : "${RTABMAP_ODOM_SOURCE:=external}"
    : "${USE_IMU:=false}"
    : "${DIRECT_DEPTH:=true}"
fi
: "${ISAAC_ROSBRIDGE_HOST:=127.0.0.1}"
: "${ISAAC_ROSBRIDGE_PORT:=$ROSBRIDGE_PORT}"
: "${ISAAC_ROS_STATE:=1}"
: "${ROSBRIDGE_CLIENT_TIMEOUT_SEC:=90}"
: "${ODOM_READY_TIMEOUT_SEC:=0}"
if [[ -z "${SCAN_READY_TIMEOUT_SEC:-}" ]]; then
    SCAN_READY_TIMEOUT_SEC="$ODOM_READY_TIMEOUT_SEC"
fi
export ROS_LOCALHOST_ONLY ISAAC_TRANSPORT ROBOT_IO_LINK START_ROSBRIDGE ROSBRIDGE_REQUIRED ROSBRIDGE_HOST ROSBRIDGE_PORT ROSBRIDGE_TOPICS_GLOB ROSBRIDGE_WIRE_FORMAT
export ROS_CAMERA ROS_PUBLISH_HZ ROS_LIDAR_FPS ROS_CAMERA_TRANSPORT
export USE_DA3 USE_NVBLOX USE_SEMANTIC_OCR USE_RTABMAP USE_SLAM_TOOLBOX RTABMAP_ODOM_SOURCE USE_IMU DIRECT_DEPTH
export ISAAC_ROSBRIDGE_HOST ISAAC_ROSBRIDGE_PORT ISAAC_ROS_STATE ROSBRIDGE_CLIENT_TIMEOUT_SEC ODOM_READY_TIMEOUT_SEC SCAN_READY_TIMEOUT_SEC
export ROBOT_SSH_AUTOSTART ROBOT_SSH_TARGET ROBOT_SSH_FALLBACK_TARGET ROBOT_SSH_IDENTITY ROBOT_REMOTE_REPO ROBOT_IO_REMOTE_COMMAND ROBOT_IO_REMOTE_LOG ROBOT_IO_PUBLISH_TF
export ROBOT_LEFT_BASE_PORT ROBOT_RIGHT_HEAD_PORT ROBOT_LIDAR_SERIAL ROBOT_LEFT_HAND_MOTOR_IDS ROBOT_RIGHT_HAND_MOTOR_IDS ROBOT_BASE_LEFT_WHEEL_ID ROBOT_BASE_BACK_WHEEL_ID ROBOT_BASE_RIGHT_WHEEL_ID ROBOT_HEAD_PAN_ID ROBOT_HEAD_TILT_ID
export ROBOT_BASE_WHEEL_RADIUS_M ROBOT_BASE_RADIUS_M ROBOT_BASE_LEFT_SIGN ROBOT_BASE_BACK_SIGN ROBOT_BASE_RIGHT_SIGN ROBOT_BASE_MAX_RAW_COMMAND ROBOT_JOINT_TARGET_TOPIC ROBOT_JOINT_STATES_TOPIC ROBOT_JOINT_COMMAND_RATE_HZ ROBOT_JOINT_STATE_RATE_HZ ROBOT_BASE_WATCHDOG_TIMEOUT_S
export ROBOT_IO_MAX_LINEAR_X ROBOT_IO_MAX_LINEAR_Y ROBOT_IO_MAX_ANGULAR_Z CMD_MAX_LINEAR_X CMD_MAX_LINEAR_Y CMD_MAX_ANGULAR_Z ROBOT_STOP_PI_LOCAL_WEB ROBOT_SSH_SYNC ROBOT_SSH_REQUIRED HARDWARE_WITH_WEB
export ROBOT_RGBD_BINARY_ENABLE ROBOT_RGBD_BINARY_PORT ROBOT_RGBD_BINARY_DEPTH_FORMAT ROBOT_RGBD_BINARY_COLOR_MODE ROBOT_RGBD_BINARY_FPS ROBOT_DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE USE_BINARY_RGBD_BRIDGE BINARY_RGBD_HOST BINARY_RGBD_PORT
export MEDIAMTX_ENABLE MEDIAMTX_RTSP_PORT MEDIAMTX_WEBRTC_PORT MEDIAMTX_PATHS
export ROBOT_VIDEO_ENABLE ROBOT_VIDEO_PATH ROBOT_VIDEO_RTSP_PORT ROBOT_VIDEO_FPS ROBOT_VIDEO_BITRATE_KBPS
export ROBOT_BASE_CAMERA_PATH ROBOT_WRIST_LEFT_CAMERA_PATH ROBOT_WRIST_RIGHT_CAMERA_PATH
export ROBOT_USB_CAMERA_RTSP_ENABLE ROBOT_BASE_CAMERA_DEVICE ROBOT_WRIST_LEFT_CAMERA_DEVICE ROBOT_WRIST_RIGHT_CAMERA_DEVICE
export ROBOT_USB_CAMERA_WIDTH ROBOT_USB_CAMERA_HEIGHT ROBOT_USB_CAMERA_FPS ROBOT_USB_CAMERA_INPUT_FORMAT ROBOT_USB_CAMERA_BITRATE_KBPS
export CAMERA_VIDEO_MODE CAMERA_VIDEO_HEAD_PATH CAMERA_VIDEO_BASE_PATH CAMERA_VIDEO_WRIST_LEFT_PATH CAMERA_VIDEO_WRIST_RIGHT_PATH CAMERA_VIDEO_WEBRTC_PORT CAMERA_VIDEO_WEBRTC_SCHEME CAMERA_VIDEO_WEBRTC_HOST

# ── 사전 점검 ──────────────────────────────────────────────────────────
need_cmd() {
    command -v "$1" >/dev/null 2>&1 || { echo "[err] missing: $1"; exit 1; }
}

source_ros_env() {
    # ROS setup.bash reads unset variables; relax nounset only while sourcing it.
    set +u
    source "$ROS_SETUP"
    source "$ROOT/$WORKSPACE_SETUP"
    set -u
}

preset_launch_arg_equals() {
    local key=$1
    local expected=$2
    local preset_file="$ROOT/bench/presets/${SIM_PRESET}.sh"
    local arg value
    local PRESET_NAME="" PRESET_DESC=""
    local -a LAUNCH_ARGS=() RECORD_TOPICS=()

    [[ -f $preset_file ]] || return 1
    # shellcheck disable=SC1090
    source "$preset_file"
    for arg in "${LAUNCH_ARGS[@]}"; do
        if [[ $arg == "$key:="* ]]; then
            value=${arg#"$key:="}
            [[ $value == "$expected" ]]
            return
        fi
    done
    return 1
}

require_ros_packages() {
    local missing=()
    local pkg

    source_ros_env
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "[err] ros2 command not found after sourcing: $ROS_SETUP"
        exit 1
    fi
    for pkg in "$@"; do
        ros2 pkg prefix "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
    done
    if (( ${#missing[@]} > 0 )); then
        echo "[err] ${SIM_PRESET} preset requires missing ROS packages: ${missing[*]}"
        echo "      current ROS_SETUP: $ROS_SETUP"
        echo "      RTAB-Map 멀티세션 DB를 쓰려면 이 ROS 환경에 RTAB-Map ROS 패키지를 설치해야 합니다."
    echo "      이 모드는 slam_toolbox fallback을 쓰지 않습니다. RTAB-Map 패키지를 설치/빌드한 뒤 다시 실행하세요."
        exit 1
    fi
}

ros_pkg_available() {
    local pkg=$1
    source_ros_env
    ros2 pkg prefix "$pkg" >/dev/null 2>&1
}

find_java17_home() {
    local candidate spec major
    for candidate in \
        "${JAVA17_HOME:-}" \
        "${CORRETTO17_HOME:-}" \
        "${JAVA_HOME:-}" \
        /opt/corretto17 \
        "$HOME/.local/opt/corretto17" \
        /usr/lib/jvm/java-17-openjdk-amd64 \
        /usr/lib/jvm/java-21-openjdk-amd64
    do
        [[ -n $candidate && -x "$candidate/bin/java" ]] || continue
        spec=$("$candidate/bin/java" -XshowSettings:properties -version 2>&1 \
            | awk -F= '/java.specification.version/ { gsub(/[[:space:]]/, "", $2); print $2; exit }')
        major=${spec#1.}
        if [[ $major =~ ^[0-9]+$ && $major -ge 17 ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

[[ -f $ROS_SETUP ]] || { echo "[err] ROS setup 없음: $ROS_SETUP"; exit 1; }
[[ -f $WORKSPACE_SETUP && -d $ROOT/install/gz_nav_sim ]] || { echo "[err] colcon build 가 안 됨 — 'colcon build --symlink-install --paths src/gz_nav_sim' 먼저"; exit 1; }
if [[ $WANT_SIM == 1 ]] && preset_launch_arg_equals use_rtabmap true; then
    require_ros_packages rtabmap_odom rtabmap_slam rtabmap_msgs
fi
EXPECT_RTABMAP="$USE_RTABMAP"
EXPECT_NVBLOX="$USE_NVBLOX"
if [[ "${USE_NVBLOX,,}" == "true" ]] && ! ros_pkg_available nvblox_ros; then
    EXPECT_NVBLOX=false
    echo "[boot] nvblox requested but nvblox_ros is missing; health will not require nvblox topics"
fi
export EXPECT_RTABMAP EXPECT_NVBLOX
if [[ $SIM_MODE == local_lidar ]] && ! python3 -c "import serial" 2>/dev/null; then
    echo "[err] python serial module 없음 — sudo apt install python3-serial"
    exit 1
fi
# Isaac 백엔드:
# - 기본 v2: Isaac 앱이 rosbridge_server 를 통해 /xlerobot ROS 토픽을 직접
#   만들기 때문에 추가 Python wire 의존성이 필요 없다.
# - legacy zmq_v1: 옛 sim_server 연결용 pyzmq/msgpack/zstandard 를 설치한다.
: "${ISAAC_HOST:=127.0.0.1}"
: "${ISAAC_ROBOT_ID:=1}"
if [[ $ISAAC_TRANSPORT == zmq_v1 ]]; then
    echo "[boot] ensuring legacy isaac ZMQ bridge deps (pyzmq, msgpack, zstandard)..."
    missing=()
    for mod in zmq msgpack zstandard; do
        python3 -c "import $mod" 2>/dev/null || missing+=("$mod")
    done
    if (( ${#missing[@]} > 0 )); then
        declare -A PKG=( [zmq]=pyzmq [msgpack]=msgpack [zstandard]=zstandard )
        pkgs=()
        for m in "${missing[@]}"; do pkgs+=("${PKG[$m]}"); done
        echo "[boot] pip3 install ${pkgs[*]}"
        pip3 install --break-system-packages "${pkgs[@]}" \
            >/tmp/isaac_bridge_pip.log 2>&1 \
            || pip3 install "${pkgs[@]}" >>/tmp/isaac_bridge_pip.log 2>&1 \
            || { echo "[err] pip 설치 실패 — /tmp/isaac_bridge_pip.log 확인"; exit 1; }
    fi
    echo "[boot] isaac legacy ZMQ 대상: tcp://${ISAAC_HOST}:5555/5556/5557  (robot_id=${ISAAC_ROBOT_ID})"
else
    echo "[boot] xlerobot 대상: ROS graph /xlerobot/* topics"
fi
if [[ ! -d $WEB_ROOT ]]; then
    if [[ $WANT_ADAPTER == 1 || $WANT_BACKEND == 1 || $WANT_FRONTEND == 1 ]]; then
        echo "[warn] $WEB_ROOT 없음 — 별도 indoors-web repo가 없으므로 adapter/backend/frontend는 skip"
    fi
    WANT_ADAPTER=0
    WANT_BACKEND=0
    WANT_FRONTEND=0
    WANT_POSTGRES=0
fi

if [[ $WANT_BACKEND == 1 ]]; then
    # gradlew/Spring Boot 3.5 는 JDK 17+ 요구. ROS 설치 부수효과로 시스템 default 가
    # OpenJDK 11 이 되는 경우가 많아 JAVA_HOME 명시 안 하면 빌드 단계에서 죽음.
    JAVA17_HOME=$(find_java17_home) || {
        echo "[err] JDK 17+ 없음 — /opt/corretto17 또는 ~/.local/opt/corretto17 설치 필요"
        exit 1
    }
    export JAVA17_HOME
    export JAVA_HOME="$JAVA17_HOME"
    export PATH="$JAVA_HOME/bin:$PATH"
fi

if [[ $SIM_MODE == isaac ]]; then
    CAMERA_VIDEO=0
    unset CAMERA_VIDEO_HEAD_URL CAMERA_VIDEO_HEAD_KIND CAMERA_VIDEO_URL CAMERA_VIDEO_KIND
    export CAMERA_VIDEO
    echo "[boot] camera video side-channel: disabled; Isaac publishes cameras only through ROS"
fi

# ── 출력 디렉터리 ─────────────────────────────────────────────────────
TS=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$ROOT/bench/runs/${TS}_multisession"
mkdir -p "$LOG_DIR"
: "${INDOORY_STATE_DIR:=$ROOT/.state/indoory}"
: "${INDOORY_MAP_STORAGE:=$INDOORY_STATE_DIR/maps}"
: "${FLOOR_DB_DIR:=$INDOORY_STATE_DIR/floor_dbs}"
mkdir -p "$INDOORY_MAP_STORAGE" "$FLOOR_DB_DIR"
if [[ $SIM_MODE == hardware && "${USE_RTABMAP:-true}" == "true" && -z "${RTABMAP_DB:-}" ]]; then
    RTABMAP_DB="$LOG_DIR/rtabmap_active.db"
fi
export INDOORY_STATE_DIR INDOORY_MAP_STORAGE FLOOR_DB_DIR RTABMAP_DB
echo "[boot] logs → $LOG_DIR"
if [[ -n "${RTABMAP_DB:-}" ]]; then
    echo "[boot] RTAB-Map DB → $RTABMAP_DB"
fi

declare -a PIDS=()
declare -A NAMES=()

start_isaac_sim() {
    if [[ $SIM_MODE != isaac || $WANT_SIM != 1 ]]; then
        return 0
    fi
    if [[ -z ${ISAAC_SIM_ROOT:-} && "${ISAAC_STREAMING_WAS_RUNNING:-0}" != "1" ]]; then
        return 0
    fi
    if [[ -z ${ISAAC_SIM_ROOT:-} && "${RESTART_EXISTING_ISAAC_AFTER_ROSBRIDGE:-0}" != "1" ]]; then
        return 0
    fi
    if [[ ! -x "$ISAAC_SIM_LAUNCH" ]]; then
        echo "[warn] Isaac launch script is missing or not executable: $ISAAC_SIM_LAUNCH"
        return 0
    fi
    echo "[boot] starting Isaac Sim app from $ISAAC_SIM_PROJECT"
    echo "[boot] Isaac rosbridge target: ws://${ISAAC_ROSBRIDGE_HOST}:${ISAAC_ROSBRIDGE_PORT} (wire=${ROSBRIDGE_WIRE_FORMAT}, ROS_STATE=${ISAAC_ROS_STATE})"
    echo "[boot] Isaac ROS sensors: depth sensor front RGB + depth compressed + IMU sources"
    setsid env \
        ROSBRIDGE_HOST="${ISAAC_ROSBRIDGE_HOST:-127.0.0.1}" \
        ROSBRIDGE_PORT="${ISAAC_ROSBRIDGE_PORT:-9090}" \
        ROSBRIDGE_WIRE_FORMAT="json" \
        ROS_STATE="${ISAAC_ROS_STATE:-1}" \
        ROS_CAMERA="1" \
        ROS_CAMERA_TRANSPORT="compressed" \
        ROS_PUBLISH_HZ="10" \
        ROS_LIDAR_FPS="8" \
        NO_KEYBOARD="${ISAAC_NO_KEYBOARD:-0}" \
        bash -c "exec '$ISAAC_SIM_LAUNCH'" \
        >"$LOG_DIR/isaac_sim.log" 2>&1 &
    ISAAC_SIM_PID=$!
    if [[ -n ${ISAAC_SIM_ROOT:-} || "${TRACK_ISAAC_SIM_CHILD:=0}" == "1" ]]; then
        PIDS+=("$ISAAC_SIM_PID")
        NAMES[$ISAAC_SIM_PID]="isaac_sim"
    fi
    echo "[boot] isaac sim pid=$ISAAC_SIM_PID (log: $LOG_DIR/isaac_sim.log)"
    sleep 2
}

check_existing_isaac_ros_state() {
    if [[ $SIM_MODE != isaac || $WANT_SIM != 1 ]]; then
        return 0
    fi
    # If this launcher did not start Isaac itself, an already-open streaming app
    # must publish robot state. Otherwise /xlerobot/odom never appears and the
    # ROS stack waits until timeout, then tears rosbridge down under Isaac.
    if [[ -n ${ISAAC_SIM_ROOT:-} ]]; then
        return 0
    fi
    local -a no_state
    mapfile -t no_state < <(
        ps -eo pid=,args= |
            awk '/launch_streaming\.py/ && /--no-ros-state/ && !/awk/ {print}'
    )
    if (( ${#no_state[@]} == 0 )); then
        return 0
    fi
    echo "[warn] existing Isaac streaming process was launched with --no-ros-state."
    echo "      That mode does not publish /xlerobot/odom, so /odom/TF/SLAM cannot become ready."
    printf '      %s\n' "${no_state[@]}"
    echo "      Web/adapter will still start; pose stays unavailable until Isaac publishes state."
    echo "      To enable pose, restart Isaac with ROS_STATE=1, e.g.:"
    echo "        ROS_STATE=1 ROSBRIDGE_HOST=${ISAAC_ROSBRIDGE_HOST} ROSBRIDGE_PORT=${ISAAC_ROSBRIDGE_PORT} \\"
    echo "          /home/indory/isaacsim/user_projects/xlerobot_hospital/scripts/restart_streaming_terminal.sh"
}

rosbridge_has_client() {
    ss -ntp state established 2>/dev/null \
        | awk -v p=":${ROSBRIDGE_PORT}" '$0 ~ p {found=1} END {exit !found}' \
        && return 0
    [[ -f "$LOG_DIR/rosbridge_server.log" ]] \
        && grep -q 'Client connected' "$LOG_DIR/rosbridge_server.log"
}

rosbridge_process_args_for_port() {
    local port=${1:-$ROSBRIDGE_PORT}
    ps -eo pid=,args= | awk -v port="$port" '
        /rosbridge_(websocket|server)|rosbridge_server/ && !/awk/ {
            if ($0 ~ ("port:=" port) || $0 ~ ("--port " port) || $0 ~ ("--port=" port)) {
                print
            }
        }'
}

existing_rosbridge_wire_format() {
    local port=${1:-$ROSBRIDGE_PORT}
    local args
    args=$(rosbridge_process_args_for_port "$port" || true)
    [[ -n "$args" ]] || return 1

    if grep -Eq 'bson_only_mode[:= ]+true|--bson_only_mode[ =]true' <<<"$args"; then
        echo "bson"
    else
        echo "json"
    fi
}

rosbridge_log_has_wire_errors() {
    [[ -f "$LOG_DIR/rosbridge_server.log" ]] \
        && grep -q 'Exception in deserialization of BSON' "$LOG_DIR/rosbridge_server.log"
}

wait_for_isaac_rosbridge_client() {
    if [[ $SIM_MODE != isaac || $WANT_SIM != 1 || "${ISAAC_TRANSPORT:-rosbridge_v2}" != "rosbridge_v2" ]]; then
        return 0
    fi
    if [[ "${ISAAC_ROS_STATE:-1}" == "0" ]]; then
        echo "[warn] Isaac ROS_STATE=0; rosbridge client/data topics will not connect"
        return 0
    fi

    local timeout=${ROSBRIDGE_CLIENT_TIMEOUT_SEC:-0}
    if (( timeout <= 0 )); then
        echo "[boot] not waiting for Isaac rosbridge client (ROSBRIDGE_CLIENT_TIMEOUT_SEC=0)"
        return 0
    fi

    echo -n "[boot] waiting for Isaac rosbridge client (timeout ${timeout}s)..."
    for (( _=1; _<=timeout; _++ )); do
        if rosbridge_has_client; then
            echo " connected"
            return 0
        fi
        echo -n "."
        sleep 1
    done
    echo ""
    echo "[warn] rosbridge_server is up, but Isaac has not connected within ${timeout}s"
    echo "      Web/adapter will still start; /xlerobot topics remain empty until Isaac connects."
}

monitor_isaac_rosbridge_client() {
    if [[ $SIM_MODE != isaac || $WANT_SIM != 1 || "${ISAAC_TRANSPORT:-rosbridge_v2}" != "rosbridge_v2" ]]; then
        return 0
    fi
    if [[ "${ISAAC_ROS_STATE:-1}" == "0" ]]; then
        echo "[warn] Isaac ROS_STATE=0; rosbridge client/data topics will not connect"
        return 0
    fi

    local timeout=${ROSBRIDGE_CLIENT_TIMEOUT_SEC:-0}
    if (( timeout <= 0 )); then
        echo "[boot] not watching Isaac rosbridge client (ROSBRIDGE_CLIENT_TIMEOUT_SEC=0)"
        return 0
    fi

    echo "[boot] watching Isaac rosbridge client in background (timeout ${timeout}s)"
    (
        for (( _=1; _<=timeout; _++ )); do
            if rosbridge_has_client; then
                echo "[boot] Isaac rosbridge client connected"
                exit 0
            fi
            sleep 1
        done
        echo "[warn] rosbridge_server is up, but Isaac has not connected within ${timeout}s"
        echo "      Web/adapter stay up; /xlerobot topics remain empty until Isaac connects."
    ) &
}

topic_has_samples() {
    local topic=$1
    local output
    output=$(timeout 5s ros2 topic hz --window 2 "$topic" 2>&1 || true)
    grep -q 'average rate' <<<"$output"
}

MEDIAMTX_PID=""
start_mediamtx_video_gateway() {
    if [[ $SIM_MODE != hardware ]]; then
        return 0
    fi
    if [[ "${ROBOT_VIDEO_ENABLE:-0}" != "1" && "${ROBOT_VIDEO_ENABLE:-0}" != "true" ]]; then
        echo "[boot] RTSP/WebRTC camera gateway disabled (ROBOT_VIDEO_ENABLE=${ROBOT_VIDEO_ENABLE:-0})"
        return 0
    fi
    if [[ "${MEDIAMTX_ENABLE:-1}" == "0" || "${MEDIAMTX_ENABLE:-1}" == "false" ]]; then
        echo "[boot] MediaMTX disabled (MEDIAMTX_ENABLE=${MEDIAMTX_ENABLE:-1})"
        return 0
    fi
    if [[ ! -x "$ROOT/scripts/start_mediamtx_video.sh" ]]; then
        echo "[err] missing MediaMTX helper: $ROOT/scripts/start_mediamtx_video.sh"
        exit 1
    fi
    if (echo > "/dev/tcp/127.0.0.1/${MEDIAMTX_RTSP_PORT}") >/dev/null 2>&1 \
       && (echo > "/dev/tcp/127.0.0.1/${MEDIAMTX_WEBRTC_PORT}") >/dev/null 2>&1; then
        echo "[boot] MediaMTX already reachable on :${MEDIAMTX_RTSP_PORT}/:${MEDIAMTX_WEBRTC_PORT}; using existing"
        return 0
    fi

    echo "[boot] starting MediaMTX RTSP/WebRTC camera gateway..."
    setsid env \
        MEDIAMTX_ENABLE="${MEDIAMTX_ENABLE}" \
        MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT}" \
        MEDIAMTX_WEBRTC_PORT="${MEDIAMTX_WEBRTC_PORT}" \
        MEDIAMTX_PATH="${ROBOT_VIDEO_PATH}" \
        bash -c "exec '$ROOT/scripts/start_mediamtx_video.sh'" \
        >"$LOG_DIR/mediamtx.log" 2>&1 &
    MEDIAMTX_PID=$!
    PIDS+=( "$MEDIAMTX_PID" ); NAMES[$MEDIAMTX_PID]="mediamtx"
    echo "[boot] mediamtx pid=$MEDIAMTX_PID (log: $LOG_DIR/mediamtx.log)"
    echo -n "[boot] waiting for MediaMTX :${MEDIAMTX_RTSP_PORT}/:${MEDIAMTX_WEBRTC_PORT}..."
    local ready=0
    for _ in {1..30}; do
        if (echo > "/dev/tcp/127.0.0.1/${MEDIAMTX_RTSP_PORT}") >/dev/null 2>&1 \
           && (echo > "/dev/tcp/127.0.0.1/${MEDIAMTX_WEBRTC_PORT}") >/dev/null 2>&1; then
            ready=1
            echo " ready"
            break
        fi
        echo -n "."
        sleep 1
    done
    if [[ $ready != 1 ]]; then
        echo ""
        echo "[err] MediaMTX did not open RTSP/WebRTC ports"
        echo "      log: $LOG_DIR/mediamtx.log"
        tail -30 "$LOG_DIR/mediamtx.log" 2>/dev/null || true
        exit 1
    fi
}

ROBOT_IO_SSH_STARTED=0
start_remote_robot_io_over_ssh() {
    if [[ $SIM_MODE != hardware || "${ROBOT_IO_LINK:-rosbridge}" != "rosbridge" ]]; then
        return 0
    fi
    if [[ "${ROBOT_SSH_AUTOSTART:-0}" != "1" && "${ROBOT_SSH_AUTOSTART:-0}" != "true" ]]; then
        echo "[boot] robot SSH autostart disabled (ROBOT_SSH_AUTOSTART=${ROBOT_SSH_AUTOSTART:-0})"
        return 0
    fi
    if [[ ! -x "$ROOT/scripts/robot_io_ssh.sh" ]]; then
        echo "[err] missing robot SSH helper: $ROOT/scripts/robot_io_ssh.sh"
        exit 1
    fi

    echo "[boot] starting remote robot I/O over SSH (log: $LOG_DIR/robot_io_ssh.log)"
    if "$ROOT/scripts/robot_io_ssh.sh" start 2>&1 | tee "$LOG_DIR/robot_io_ssh.log"; then
        ROBOT_IO_SSH_STARTED=1
    else
        if [[ "${ROBOT_SSH_REQUIRED:-0}" == "1" || "${ROBOT_SSH_REQUIRED:-0}" == "true" ]]; then
            echo "[err] failed to start remote robot I/O over SSH"
            echo "      log: $LOG_DIR/robot_io_ssh.log"
            echo "      disable autostart with ROBOT_SSH_AUTOSTART=0 and run ${ROBOT_IO_REMOTE_COMMAND} manually on the Pi if needed"
            exit 1
        fi
        echo "[warn] failed to start remote robot I/O over SSH; continuing ROS stack"
        echo "      log: $LOG_DIR/robot_io_ssh.log"
        echo "      fix SSH/key auth, or run ${ROBOT_IO_REMOTE_COMMAND} manually on the Pi"
    fi
}

# ── 자식 프로세스 추적 + 정리 트랩 ────────────────────────────────────
cleanup() {
    # 재진입 방지 — Ctrl-C 두 번 누르면 SIGINT 다시 들어와 cleanup 재호출됨.
    [[ -n ${CLEANUP_RAN:-} ]] && return
    CLEANUP_RAN=1
    set +e
    echo ""; echo "[exit] tearing down (force mode)..."

    if [[ ${ROBOT_IO_SSH_STARTED:-0} == 1 && -x "$ROOT/scripts/robot_io_ssh.sh" ]]; then
        echo "  stopping remote robot I/O (${ROBOT_SSH_TARGET:-pi@lekiwi})..."
        "$ROOT/scripts/robot_io_ssh.sh" stop >>"$LOG_DIR/robot_io_ssh.log" 2>&1 || true
    fi

    # 0단계: rtabmap_slam 의 2D occupancy grid 는 destructor 시점에만 DB 에 저장됨
    # (CoreWrapper::~CoreWrapper → save2DMap → close DB). 100–400MB DB 는
    # SIGTERM 이후 save 에 5–15초 걸리는데, 기존 sleep 1 + SIGKILL 로 매번
    # 중간에 잘려서 부팅 시 "2D occupancy grid map loaded" 안 찍히고 빈 맵으로
    # 보였음. rtabmap PID 직접 찾아 SIGINT 보내고 종료 완료를 최대 45초 기다림.
    # 패턴: /opt/ros/humble/lib/rtabmap_slam/rtabmap 실행 파일만. icp_odometry,
    # rtabmap_odom 등은 persistence 없으므로 제외.
    rtabmap_pids=$(pgrep -f '/rtabmap_slam/rtabmap( |$)' 2>/dev/null)
    if [[ -n $rtabmap_pids ]]; then
        for rp in $rtabmap_pids; do
            echo "  SIGINT rtabmap (pid=$rp) — waiting for 2D grid save (max 45s)..."
            kill -INT "$rp" 2>/dev/null || true
        done
        for i in {1..45}; do
            still_alive=0
            for rp in $rtabmap_pids; do
                kill -0 "$rp" 2>/dev/null && still_alive=1
            done
            (( still_alive == 0 )) && { echo "  rtabmap saved+exited cleanly after ${i}s"; break; }
            sleep 1
        done
    fi

    # 1단계: 자식 process group 전체에 SIGTERM (graceful 3초 — gradle/spring 도 마무리 시간).
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ')
            echo "  SIGTERM ${NAMES[$pid]} (pid=$pid, pgid=$pgid)"
            [[ -n $pgid ]] && kill -TERM -- -"$pgid" 2>/dev/null || true
        fi
    done
    sleep 3
    # 2단계: 살아있으면 SIGKILL — 사용자가 강제 종료 원함.
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ')
            echo "  SIGKILL ${NAMES[$pid]} (pid=$pid, pgid=$pgid)"
            [[ -n $pgid ]] && kill -KILL -- -"$pgid" 2>/dev/null || true
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    # 3단계: gradle daemon 명시적 정지 + purge_stale 풀 패스.
    if [[ -x "$WEB_ROOT/backend/gradlew" ]]; then
        ( cd "$WEB_ROOT/backend" && \
          JAVA_HOME="${JAVA_HOME:-${JAVA17_HOME:-/opt/corretto17}}" \
          PATH="${JAVA_HOME:-${JAVA17_HOME:-/opt/corretto17}}/bin:$PATH" \
          ./gradlew --stop >/dev/null 2>&1 ) &
    fi
    if [[ ${LOCAL_POSTGRES_STARTED:-0} == 1 && -n ${POSTGRES_DATA_DIR:-} ]] \
            && command -v pg_ctl >/dev/null 2>&1; then
        pg_ctl -D "$POSTGRES_DATA_DIR" -m fast stop >>"$LOG_DIR/postgres.log" 2>&1 || true
    fi
    sleep 1
    if declare -F purge_stale >/dev/null; then purge_stale; fi
    echo "[exit] done"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ── 완전 정리: 이전 세션의 모든 좀비 / 고아 프로세스 박멸 ──────────────
purge_stale() {
    # set -e 가 purge 안에서 죽지 않게 일시 해제.
    set +e
    # 1) 이름 기반 광범위 kill (Java/Gradle/Node 도 한꺼번에).
    pkill -9 -f 'rtabmap|foxglove_bridge|nav2|controller_server|planner_server|behavior_server|smoother_server|bt_navigator|waypoint_follower|velocity_smoother|component_container|java.*indoor|java.*IndooryApp|java.*spring|gradle.*bootRun|GradleDaemon|GradleWorker|gradlew|uvicorn|vite|node.*frontend|node.*vite|tf2_ros|static_transform_publisher|image_transport|republish|slam_toolbox|nvblox|da3_depth|explore_node|trajectory_path|nav_destination_node|rplidar_c1_scan_node|launch_ros|ros2-daemon|joint_state_publisher|xlerobot_v2_bridge' 2>/dev/null
    sleep 1
    # 2) /opt/ros/humble/lib 에서 spawn 된 PID catch-all.
    pgrep -f '/opt/ros/humble/lib/' 2>/dev/null | xargs -r kill -9 2>/dev/null
    # 3) PPID=1 로 떨어진 우리 관련 고아 프로세스 (ROS + Java/Node/Python 웹 스택).
    for pid in $(ps -e -o pid=,ppid= 2>/dev/null | awk '$2==1{print $1}'); do
        cmdline=$(tr -d '\0' < /proc/$pid/cmdline 2>/dev/null)
        if [[ -n $cmdline ]] && [[ $cmdline =~ (ros|rclpy|rtabmap|gz_nav_sim|gradle|IndooryApp|indoors-web|spring-boot|uvicorn|vite|corretto17) ]]; then
            kill -9 "$pid" 2>/dev/null
        fi
    done
    # 4) 포트 점유 프로세스 직접 KILL (요청 응답 안 해도 listener 만 있으면 잡음).
    for port in 8080 8000 5173 8765 9090 8554 8889 11345 5555 5556 5557; do
        pid=$(ss -lntp 2>/dev/null | awk -v p=":$port" '$0 ~ p {print}' | grep -oP 'pid=\K\d+' | head -1)
        [[ -n $pid ]] && kill -9 "$pid" 2>/dev/null
    done
    # 5) 좀비(<defunct>) reap 유도 — 부모에게 SIGCHLD 보내서 wait() 깨우기.
    # PPID=1 좀비는 init/PID1 이 reap 안 하면 누적되니, PID1 에도 SIGCHLD.
    # 좀비 자체는 자원 안 쓰지만 시각적 노이즈 + 일부 모니터링 도구 오작동 원인.
    zombie_ppids=$(ps -A -o pid=,ppid=,stat= 2>/dev/null | awk '$3 ~ /Z/ {print $2}' | sort -u)
    for ppid in $zombie_ppids; do
        [[ $ppid -gt 1 ]] && kill -CHLD "$ppid" 2>/dev/null
    done
    # PID1 컨테이너에서 reap 안 하는 경우 흔함. SIGCHLD 시도 (보통 무시되지만 비용 0).
    kill -CHLD 1 2>/dev/null
    # 6) FastDDS shared memory + lockfile + Gradle daemon 캐시 락 정리.
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    rm -f /tmp/launch_params_* 2>/dev/null
    rm -f /tmp/.X99-lock /tmp/.X11-unix/X99 2>/dev/null
    rm -f "$WEB_ROOT/backend/.gradle/8."*/file-system.probe 2>/dev/null
    sleep 1
    # 호출자에서 set -e 유지/해제 결정. 여기서 다시 켜지 않음.
    return 0
}

echo "[boot] purging stale ROS/Web stack from previous runs..."
set +e   # 검증 블록 동안 pipefail / pgrep 0건 회피
# 사전 진단: 잔재 + 좀비 카운트.
ISAAC_STREAMING_WAS_RUNNING=0
if [[ $SIM_MODE == isaac ]] && pgrep -f '/xlerobot_hospital/scripts/launch_streaming.py' >/dev/null 2>&1; then
    ISAAC_STREAMING_WAS_RUNNING=1
    if [[ "${RESTART_EXISTING_ISAAC_AFTER_ROSBRIDGE:-0}" == "1" ]]; then
        echo "[boot] existing Isaac streaming process detected; it will be restarted after rosbridge is ready"
    else
        echo "[boot] existing Isaac streaming process detected; leaving it running"
    fi
fi
before_alive=$(pgrep -f '/opt/ros/humble/lib/|rtabmap|uvicorn|vite|java|gradle|node.*frontend' 2>/dev/null | wc -l)
before_zombie=$(ps -A -o stat= 2>/dev/null | awk '$1 ~ /Z/' | wc -l)
if (( before_alive > 0 || before_zombie > 0 )); then
    echo "[boot] before purge: $before_alive live stale procs, $before_zombie zombies"
fi
purge_stale
# 잔존 검사: 살아있는 것만. 좀비는 PID1 reap 권한이라 우리가 못 죽이므로 alive 만 카운트.
remaining=$(pgrep -f '/opt/ros/humble/lib/|rtabmap|uvicorn|vite|java.*indoor|gradle.*bootRun|GradleDaemon|node.*frontend' 2>/dev/null | wc -l)
if [[ $remaining -gt 0 ]]; then
    echo "[warn] $remaining stale processes still alive after purge — running second pass"
    purge_stale
    remaining=$(pgrep -f '/opt/ros/humble/lib/|rtabmap|uvicorn|vite|java.*indoor|gradle.*bootRun|GradleDaemon|node.*frontend' 2>/dev/null | wc -l)
    if [[ $remaining -gt 0 ]]; then
        echo "[err] $remaining stubborn processes — listing for manual review:"
        pgrep -fa '/opt/ros/humble/lib/|rtabmap|uvicorn|vite|java.*indoor|gradle.*bootRun|GradleDaemon|node.*frontend' 2>/dev/null | head -10
    fi
fi
# 좀비는 PID1 이 reap 안 해주면 영구적으로 남음. 자원은 안 쓰지만 누적되면 시각적 노이즈.
after_zombie=$(ps -A -o stat= 2>/dev/null | awk '$1 ~ /Z/' | wc -l)
if (( after_zombie > 0 )); then
    echo "[info] $after_zombie zombie(s) remain — PID1 reap 권한 필요 (재부팅 시 사라짐, 동작에는 영향 없음)"
fi
set -e

# ── Postgres ─────────────────────────────────────────────────────────
# 도커 의존성 제거: 시스템 패키지가 있으면 service/pg_ctlcluster 를 쓰고,
# sudo 없는 conda 환경에서는 사용자 공간 .state/postgres 로 실행.
# DB/유저는 indoory:indoory@localhost:5432/indoory (compose 와 동일 자격증명).
: "${POSTGRES_HOST:=127.0.0.1}"
: "${POSTGRES_PORT:=5432}"
: "${POSTGRES_DATA_DIR:=$ROOT/.state/postgres}"

postgres_reachable() {
    (echo > "/dev/tcp/$POSTGRES_HOST/$POSTGRES_PORT") >/dev/null 2>&1
}

_psql_super() {
    # conda/local postgres 는 trust auth 로 tcp 접속, 시스템 postgres 는 OS 유저로 fallback.
    if command -v psql >/dev/null 2>&1 \
            && psql -h "$POSTGRES_HOST" -p "$POSTGRES_PORT" -U postgres "$@"; then
        return 0
    fi
    # postgres OS 유저로 psql 실행.
    # postgres 가 cd 못 하는 디렉터리 경고 방지 위해 /tmp 에서 실행.
    if command -v sudo >/dev/null 2>&1; then
        (cd /tmp && sudo -n -u postgres psql "$@")
    else
        (cd /tmp && su -s /bin/bash postgres -c "psql $(printf '%q ' "$@")")
    fi
}

start_user_postgres() {
    command -v initdb >/dev/null 2>&1 || return 1
    command -v pg_ctl >/dev/null 2>&1 || return 1
    command -v psql >/dev/null 2>&1 || return 1

    mkdir -p "$(dirname "$POSTGRES_DATA_DIR")"
    if [[ ! -f "$POSTGRES_DATA_DIR/PG_VERSION" ]]; then
        rm -rf "$POSTGRES_DATA_DIR"
        initdb -D "$POSTGRES_DATA_DIR" -U postgres -A trust --encoding=UTF8 --locale=C \
            >>"$LOG_DIR/postgres.log" 2>&1
        {
            printf "\nlisten_addresses = '%s'\n" "$POSTGRES_HOST"
            printf "port = %s\n" "$POSTGRES_PORT"
        } >>"$POSTGRES_DATA_DIR/postgresql.conf"
    fi

    pg_ctl -D "$POSTGRES_DATA_DIR" -l "$LOG_DIR/postgres.log" \
        -o "-h $POSTGRES_HOST -p $POSTGRES_PORT" start \
        >>"$LOG_DIR/postgres.log" 2>&1
    LOCAL_POSTGRES_STARTED=1
}

ensure_indoory_db() {
    # 'indoory' 유저/DB 가 없으면 생성. 멱등.
    if ! _psql_super -tAc "SELECT 1 FROM pg_roles WHERE rolname='indoory'" 2>/dev/null | grep -q 1; then
        _psql_super -c "CREATE USER indoory WITH PASSWORD 'indoory';" \
            >>"$LOG_DIR/postgres.log" 2>&1 || true
    fi
    if ! _psql_super -tAc "SELECT 1 FROM pg_database WHERE datname='indoory'" 2>/dev/null | grep -q 1; then
        _psql_super -c "CREATE DATABASE indoory OWNER indoory;" \
            >>"$LOG_DIR/postgres.log" 2>&1 || true
    fi
}

if [[ $WANT_POSTGRES == 1 ]]; then
    if postgres_reachable; then
        echo "[boot] postgres reachable on :$POSTGRES_PORT — using existing"
        ensure_indoory_db || true
    else
        echo "[boot] starting postgres..."
        if command -v pg_ctlcluster >/dev/null 2>&1 && command -v service >/dev/null 2>&1; then
            service postgresql start >>"$LOG_DIR/postgres.log" 2>&1 || true
        fi
        if ! postgres_reachable && command -v pg_ctlcluster >/dev/null 2>&1; then
            cluster=$(pg_lsclusters -h 2>/dev/null | awk 'NR==1 {print $1, $2}')
            if [[ -n $cluster ]]; then
                pg_ctlcluster $cluster start >>"$LOG_DIR/postgres.log" 2>&1 || true
            fi
        fi
        if ! postgres_reachable; then
            start_user_postgres \
                || { echo "[err] postgres 실행 도구 없음 — conda postgresql 또는 시스템 postgresql 필요"; exit 1; }
        fi
        for _ in {1..30}; do
            postgres_reachable && break
            sleep 1
        done
        if ! postgres_reachable; then
            echo "[err] postgres did not come up — $LOG_DIR/postgres.log 확인"; exit 1
        fi
        echo "[boot] postgres ready"
        ensure_indoory_db || true
    fi
fi

# ── rosbridge_server (Isaac v2 또는 hardware/rosbridge 모드) ──────────
if [[ $WANT_SIM == 1 && ( "${ISAAC_TRANSPORT:-rosbridge_v2}" == "rosbridge_v2" || "${START_ROSBRIDGE:-0}" == "1" ) ]]; then
    if ! bash -c "set +u; source '$ROS_SETUP'; ros2 pkg prefix rosbridge_server >/dev/null 2>&1"; then
        if [[ "${ROSBRIDGE_REQUIRED:-0}" == "1" ]]; then
            echo "[err] rosbridge_server package missing."
            echo "      conda: micromamba install -n $XLE_COMPUTE_ENV -c robostack-humble -c conda-forge ros-humble-rosbridge-server"
            echo "      apt  : sudo apt install ros-${ROS_DISTRO}-rosbridge-server"
            exit 1
        fi
        echo "[warn] rosbridge_server package missing; continuing without it"
    else
        ROSBRIDGE_BSON_ONLY=false
        echo "[boot] starting rosbridge_server on ${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT} (wire=${ROSBRIDGE_WIRE_FORMAT}, bson_only_mode=${ROSBRIDGE_BSON_ONLY})..."
        ROSBRIDGE_PID=""
        if (echo > "/dev/tcp/127.0.0.1/${ROSBRIDGE_PORT}") >/dev/null 2>&1; then
            echo "[boot] rosbridge_server already reachable on :${ROSBRIDGE_PORT}; using existing"
            EXISTING_ROSBRIDGE_WIRE_FORMAT=$(existing_rosbridge_wire_format "${ROSBRIDGE_PORT}" || true)
            if [[ -n "$EXISTING_ROSBRIDGE_WIRE_FORMAT" && "$EXISTING_ROSBRIDGE_WIRE_FORMAT" != "${ROSBRIDGE_WIRE_FORMAT:-json}" ]]; then
                echo "[err] existing rosbridge_server on :${ROSBRIDGE_PORT} appears to use ${EXISTING_ROSBRIDGE_WIRE_FORMAT},"
                echo "      but this run needs ROSBRIDGE_WIRE_FORMAT=${ROSBRIDGE_WIRE_FORMAT:-json}."
                echo "      Stop the stale rosbridge or rerun with the same ROSBRIDGE_WIRE_FORMAT."
                rosbridge_process_args_for_port "${ROSBRIDGE_PORT}" | sed 's/^/      /'
                exit 1
            fi
            echo "[boot] existing rosbridge_server wire format: ${EXISTING_ROSBRIDGE_WIRE_FORMAT:-unknown}"
        else
            if [[ -n "${ROSBRIDGE_TOPICS_GLOB:-}" ]]; then
                setsid env \
                    ROSBRIDGE_TOPICS_GLOB="${ROSBRIDGE_TOPICS_GLOB}" \
                    ROSBRIDGE_HOST="${ROSBRIDGE_HOST}" \
                    ROSBRIDGE_PORT="${ROSBRIDGE_PORT}" \
                    bash -c "set -eo pipefail; \
                    set +u; source '$ROS_SETUP'; set -u; \
                    exec ros2 run rosbridge_server rosbridge_websocket \
                      --address \"\$ROSBRIDGE_HOST\" \
                      --port \"\$ROSBRIDGE_PORT\" \
                      --max_message_size 10000000 \
                      --topics_glob \"\$ROSBRIDGE_TOPICS_GLOB\"" \
                    >"$LOG_DIR/rosbridge_server.log" 2>&1 &
            else
                setsid bash -c "set -eo pipefail; \
                    set +u; source '$ROS_SETUP'; set -u; \
                    exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
                      address:='${ROSBRIDGE_HOST}' port:='${ROSBRIDGE_PORT}' \
                      bson_only_mode:='${ROSBRIDGE_BSON_ONLY}'" \
                    >"$LOG_DIR/rosbridge_server.log" 2>&1 &
            fi
            ROSBRIDGE_PID=$!
            PIDS+=( "$ROSBRIDGE_PID" ); NAMES[$ROSBRIDGE_PID]="rosbridge_server"
            echo "[boot] rosbridge_server pid=$ROSBRIDGE_PID (log: $LOG_DIR/rosbridge_server.log)"
        fi
    
    echo -n "[boot] waiting for rosbridge_server :${ROSBRIDGE_PORT}..."
        ROSBRIDGE_READY=0
    for _ in {1..15}; do
        if (echo > "/dev/tcp/127.0.0.1/${ROSBRIDGE_PORT}") >/dev/null 2>&1; then
            echo " ready"; ROSBRIDGE_READY=1; break
        fi
        echo -n "."
        sleep 1
    done
        if [[ $ROSBRIDGE_READY != 1 ]]; then
            echo ""
            if [[ "${ROSBRIDGE_REQUIRED:-0}" == "1" ]]; then
                echo "[err] rosbridge_server did not open :${ROSBRIDGE_PORT}"
                echo "      log: $LOG_DIR/rosbridge_server.log"
                exit 1
            fi
            echo "[warn] rosbridge_server not reachable after 15s"
        fi
    fi
fi

start_isaac_sim
check_existing_isaac_ros_state
monitor_isaac_rosbridge_client
start_mediamtx_video_gateway
start_remote_robot_io_over_ssh

# ── 시뮬레이터 (Isaac v2 bridge + SLAM + Nav2) ────────────────────────
if [[ $WANT_SIM == 1 ]]; then
    echo "[boot] starting sim ($SIM_PRESET preset)..."
    SIM_ARGS=( "$SIM_PRESET" )
    if [[ -n $SIM_DURATION ]]; then
        SIM_ARGS+=( --duration "$SIM_DURATION" )
    fi
    # bench/run.sh 가 ROS_LOG_DIR 를 처리. setsid 로 새 PG 만들어 정리 가능하게.
    # ISAAC_HOST / ISAAC_ROBOT_ID 는 launch 까지 환경변수로 전달
    # — preset 안에서 ${...:-default} 로 참조.
    setsid env ISAAC_HOST="${ISAAC_HOST:-127.0.0.1}" \
              ISAAC_ROBOT_ID="${ISAAC_ROBOT_ID:-0}" \
              ISAAC_TRANSPORT="${ISAAC_TRANSPORT:-xlerobot_ros}" \
              USE_DA3="${USE_DA3}" \
              USE_NVBLOX="${USE_NVBLOX}" \
              USE_SEMANTIC_OCR="${USE_SEMANTIC_OCR}" \
              USE_RTABMAP="${USE_RTABMAP}" \
              USE_SLAM_TOOLBOX="${USE_SLAM_TOOLBOX}" \
              RTABMAP_ODOM_SOURCE="${RTABMAP_ODOM_SOURCE}" \
              RTABMAP_DB="${RTABMAP_DB:-}" \
              USE_IMU="${USE_IMU:-true}" \
              DIRECT_DEPTH="${DIRECT_DEPTH}" \
              HARDWARE_LIDAR_SERIAL="${HARDWARE_LIDAR_SERIAL:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_12703f59806eef11ba3ee8c2c169b110-if00-port0}" \
              HARDWARE_LIDAR_BAUD="${HARDWARE_LIDAR_BAUD:-460800}" \
              HARDWARE_LIDAR_FRAME="${HARDWARE_LIDAR_FRAME:-laser}" \
              HARDWARE_LIDAR_SAMPLES="${HARDWARE_LIDAR_SAMPLES:-720}" \
              HARDWARE_LIDAR_ANGLE_OFFSET_DEG="${HARDWARE_LIDAR_ANGLE_OFFSET_DEG:-0.0}" \
              HARDWARE_LIDAR_INVERT="${HARDWARE_LIDAR_INVERT:-false}" \
              HARDWARE_LIDAR_RANGE_MIN="${HARDWARE_LIDAR_RANGE_MIN:-0.12}" \
              HARDWARE_LIDAR_RANGE_MAX="${HARDWARE_LIDAR_RANGE_MAX:-12.0}" \
              HARDWARE_LIDAR_MIN_QUALITY="${HARDWARE_LIDAR_MIN_QUALITY:-0}" \
        bash -c "exec $ROOT/bench/run.sh ${SIM_ARGS[*]}" \
        >"$LOG_DIR/sim.log" 2>&1 &
    SIM_PID=$!
    PIDS+=( "$SIM_PID" ); NAMES[$SIM_PID]="sim"
    echo "[boot] sim pid=$SIM_PID (log: $LOG_DIR/sim.log)"

    # /odom 토픽 이름만 보이는 상태는 부족하다. Isaac state publish가 꺼져 있으면
    # xlerobot_v2_bridge publisher는 생겨도 실제 /odom 메시지는 흐르지 않는다.
    ODOM_READY=0
    if (( ODOM_READY_TIMEOUT_SEC > 0 )); then
        echo -n "[boot] waiting for /odom messages (timeout ${ODOM_READY_TIMEOUT_SEC}s)..."
        source_ros_env
        for (( _=1; _<=ODOM_READY_TIMEOUT_SEC; _++ )); do
            if ! kill -0 "$SIM_PID" 2>/dev/null; then
                echo ""
                echo "[warn] sim process exited before /odom became ready — web/adapter will still start"
                echo "      log: $LOG_DIR/sim.log"
                tail -40 "$LOG_DIR/sim.log" 2>/dev/null || true
                break
            fi
            if topic_has_samples /odom; then
                echo " ready"; ODOM_READY=1; break
            fi
            echo -n "."
            sleep 1
        done
        if [[ $ODOM_READY != 1 ]]; then
            echo ""
            if rosbridge_log_has_wire_errors; then
                echo "[err] rosbridge_server is rejecting Isaac messages as BSON."
                echo "      This means Isaac and rosbridge disagree on ROSBRIDGE_WIRE_FORMAT."
                echo "      This launcher now uses JSON only; stop the stale BSON rosbridge and rerun."
                echo "      log: $LOG_DIR/rosbridge_server.log"
                exit 1
            fi
            echo "[warn] /odom did not become ready within ${ODOM_READY_TIMEOUT_SEC}s"
            echo "      log: $LOG_DIR/sim.log"
            if [[ $SIM_MODE == isaac ]]; then
                echo "      pose will show unavailable until Isaac publishes /xlerobot/odom."
                echo "      Check that Isaac was started with --ros-state and is connected to ws://${ISAAC_ROSBRIDGE_HOST}:${ISAAC_ROSBRIDGE_PORT}."
            fi
            tail -40 "$LOG_DIR/sim.log" 2>/dev/null || true
        fi
    else
        if [[ $SIM_MODE == hardware ]]; then
            echo "[boot] not waiting for /odom (ODOM_READY_TIMEOUT_SEC=0); RTAB will publish it after selected sensor odometry is ready"
        else
            echo "[boot] not waiting for /odom (ODOM_READY_TIMEOUT_SEC=0); pose will appear when Isaac publishes it"
        fi
    fi

    # /map needs real LaserScan frames, not just a DDS endpoint. Native Isaac
    # LiDAR can leave /xlerobot/scan visible with no data if Isaac was started
    # before this ROS stack or the LiDAR graph stopped ticking.
    SCAN_READY=0
    if (( SCAN_READY_TIMEOUT_SEC > 0 )); then
        echo -n "[boot] waiting for /scan messages (timeout ${SCAN_READY_TIMEOUT_SEC}s)..."
        source_ros_env
        for (( _=1; _<=SCAN_READY_TIMEOUT_SEC; _++ )); do
            if ! kill -0 "$SIM_PID" 2>/dev/null; then
                echo ""
                echo "[warn] sim process exited before /scan became ready — web/adapter will still start"
                echo "      log: $LOG_DIR/sim.log"
                tail -40 "$LOG_DIR/sim.log" 2>/dev/null || true
                break
            fi
            if topic_has_samples /scan; then
                echo " ready"; SCAN_READY=1; break
            fi
            echo -n "."
            sleep 1
        done
        if [[ $SCAN_READY != 1 ]]; then
            echo ""
            echo "[warn] /scan did not become ready within ${SCAN_READY_TIMEOUT_SEC}s"
            echo "      log: $LOG_DIR/sim.log"
            if [[ $SIM_MODE == isaac ]]; then
                echo "      /map will show unavailable until Isaac publishes real /xlerobot/scan frames."
                echo "      Restart Isaac after this ROS stack is up, with --ros-lidar-transport physx/native."
            fi
            tail -40 "$LOG_DIR/sim.log" 2>/dev/null || true
        fi
    else
        echo "[boot] not waiting for /scan (SCAN_READY_TIMEOUT_SEC=0); map will appear when SLAM receives scans"
    fi
fi

# ── ros_adapter (FastAPI :8000) ────────────────────────────────────────
if [[ $WANT_ADAPTER == 1 ]]; then
    echo "[boot] starting ros_adapter..."
    setsid env GZ_NAV_SIM_ROOT="$ROOT" \
        INDOORY_STATE_DIR="$INDOORY_STATE_DIR" \
        INDOORY_MAP_STORAGE="$INDOORY_MAP_STORAGE" \
        FLOOR_DB_DIR="$FLOOR_DB_DIR" \
        RTABMAP_DB="${RTABMAP_DB:-}" \
        ROS_CAMERA="1" \
        EXPECT_ROS_CAMERA="1" \
        EXPECT_SEMANTIC_OCR="${USE_SEMANTIC_OCR}" \
        EXPECT_RTABMAP="${EXPECT_RTABMAP}" \
        EXPECT_NVBLOX="${EXPECT_NVBLOX}" \
        TELEOP_MAX_LINEAR_X="${ROBOT_IO_MAX_LINEAR_X:-0.30}" \
        TELEOP_MAX_LINEAR_Y="${ROBOT_IO_MAX_LINEAR_Y:-0.30}" \
        TELEOP_MAX_ANGULAR_Z="${ROBOT_IO_MAX_ANGULAR_Z:-1.00}" \
        CAMERA_VIDEO_MODE="${CAMERA_VIDEO_MODE:-}" \
        CAMERA_VIDEO_HEAD_PATH="${CAMERA_VIDEO_HEAD_PATH:-xlerobot_head}" \
        CAMERA_VIDEO_BASE_PATH="${CAMERA_VIDEO_BASE_PATH:-xlerobot_base}" \
        CAMERA_VIDEO_WRIST_LEFT_PATH="${CAMERA_VIDEO_WRIST_LEFT_PATH:-xlerobot_wrist_left}" \
        CAMERA_VIDEO_WRIST_RIGHT_PATH="${CAMERA_VIDEO_WRIST_RIGHT_PATH:-xlerobot_wrist_right}" \
        CAMERA_VIDEO_WEBRTC_PORT="${CAMERA_VIDEO_WEBRTC_PORT:-8889}" \
        CAMERA_VIDEO_WEBRTC_SCHEME="${CAMERA_VIDEO_WEBRTC_SCHEME:-http}" \
        CAMERA_VIDEO_WEBRTC_HOST="${CAMERA_VIDEO_WEBRTC_HOST:-}" \
        bash -c "exec '$WEB_ROOT/ros_adapter/run.sh'" \
        >"$LOG_DIR/adapter.log" 2>&1 &
    ADAPTER_PID=$!
    PIDS+=( "$ADAPTER_PID" ); NAMES[$ADAPTER_PID]="adapter"
    echo "[boot] adapter pid=$ADAPTER_PID (log: $LOG_DIR/adapter.log)"

    echo -n "[boot] waiting for adapter :8000..."
    ADAPTER_READY=0
    for _ in {1..60}; do
        if kill -0 "$ADAPTER_PID" 2>/dev/null \
                && curl -sf http://localhost:8000/health >/dev/null 2>&1; then
            echo " ready"; ADAPTER_READY=1; break
        fi
        if ! kill -0 "$ADAPTER_PID" 2>/dev/null; then
            echo ""; echo "[warn] adapter process died — sim 은 계속 동작 (adapter.log 확인)"
            tail -10 "$LOG_DIR/adapter.log" 2>/dev/null
            ADAPTER_PID=""
            break
        fi
        echo -n "."
        sleep 1
    done
    if [[ -n ${ADAPTER_PID:-} && $ADAPTER_READY != 1 ]]; then
        echo ""; echo "[warn] adapter not responsive after 60s — sim 은 계속 동작"
    fi
fi

# ── Spring Boot 백엔드 (:8080) ────────────────────────────────────────
if [[ $WANT_BACKEND == 1 ]]; then
    echo "[boot] starting Spring Boot backend..."
    # 백엔드는 application.yml 이 datasource 안 잡아주므로 env 로 주입.
    # 1) indoory.bridge.* 어댑터 호출 활성화
    # 2) spring.datasource.* 로컬 postgres 연결
    # 3) jpa.hibernate.ddl-auto=update 로 신규 컬럼(rtabmap_db) 자동 반영
    BACKEND_JSON='{
      "indoory.bridge.enabled": true,
      "indoory.bridge.baseUrl": "http://localhost:8000",
      "spring.datasource.url": "jdbc:postgresql://localhost:5432/indoory",
      "spring.datasource.username": "indoory",
      "spring.datasource.password": "indoory",
      "spring.datasource.driver-class-name": "org.postgresql.Driver",
      "spring.jpa.hibernate.ddl-auto": "update",
      "spring.jpa.properties.hibernate.dialect": "org.hibernate.dialect.PostgreSQLDialect"
    }'
    setsid bash -c "cd '$WEB_ROOT/backend' && \
        JAVA_HOME='$JAVA_HOME' PATH='$JAVA_HOME/bin':\$PATH \
        SPRING_APPLICATION_JSON='$(echo "$BACKEND_JSON" | tr -d '\n')' \
        exec ./gradlew bootRun --console=plain" \
        >"$LOG_DIR/backend.log" 2>&1 &
    BACKEND_PID=$!
    PIDS+=( "$BACKEND_PID" ); NAMES[$BACKEND_PID]="backend"
    echo "[boot] backend pid=$BACKEND_PID (log: $LOG_DIR/backend.log)"

    echo -n "[boot] waiting for backend :8080 (gradle bootRun, 첫 빌드 1~2분)..."
    BACKEND_READY=0
    for _ in {1..240}; do
        # 우리가 띄운 자식 PID 가 살아있고, 8080 이 응답해야 진짜 ready.
        # 자식이 죽었는데 8080 이 응답하면 그건 stale process.
        if kill -0 "$BACKEND_PID" 2>/dev/null && \
                (curl -sf http://localhost:8080/actuator/health >/dev/null 2>&1 \
                 || curl -s -o /dev/null -w "%{http_code}" \
                        http://localhost:8080/api/maps 2>/dev/null | grep -qE '200|401'); then
            echo " ready"; BACKEND_READY=1; break
        fi
        if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
            echo ""; echo "[warn] backend process died — sim/adapter 는 계속 동작 (backend.log 확인)"
            tail -10 "$LOG_DIR/backend.log" 2>/dev/null
            BACKEND_PID=""  # 추적 해제
            break
        fi
        echo -n "."
        sleep 1
    done
    if [[ -n ${BACKEND_PID:-} && $BACKEND_READY != 1 ]]; then
        echo ""; echo "[warn] backend not ready after 240s — sim/adapter 는 계속 동작"
        tail -10 "$LOG_DIR/backend.log" 2>/dev/null
    fi
fi

# ── 프론트엔드 dev (:5173) ─────────────────────────────────────────────
if [[ $WANT_FRONTEND == 1 ]]; then
    # Vite 8+ 가 Node 20.19+ / 22.12+ 요구. 우분투 22.04 apt 기본은 12.x.
    # 20 미만이면 NodeSource 22 LTS 로 교체.
    NODE_OK=0
    if command -v node >/dev/null 2>&1; then
        NODE_MAJOR=$(node -v 2>/dev/null | sed -E 's/^v([0-9]+).*/\1/')
        if [[ -n $NODE_MAJOR ]] && (( NODE_MAJOR >= 20 )); then
            NODE_OK=1
        fi
    fi
    if [[ $NODE_OK == 0 ]]; then
        echo "[boot] node 20+ 필요 — NodeSource 22.x 로 설치..."
        apt-get remove -y nodejs npm libnode72 >>"$LOG_DIR/frontend.log" 2>&1 || true
        apt-get install -y curl ca-certificates gnupg >>"$LOG_DIR/frontend.log" 2>&1
        mkdir -p /etc/apt/keyrings
        curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
            | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg 2>>"$LOG_DIR/frontend.log"
        echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_22.x nodistro main" \
            > /etc/apt/sources.list.d/nodesource.list
        apt-get update >>"$LOG_DIR/frontend.log" 2>&1
        apt-get install -y nodejs >>"$LOG_DIR/frontend.log" 2>&1 \
            || { echo "[warn] nodejs 22 설치 실패 — 프론트엔드 skip"; WANT_FRONTEND=0; }
    fi
fi
if [[ $WANT_FRONTEND == 1 ]]; then
    echo "[boot] starting frontend dev..."
    if [[ ! -d $WEB_ROOT/frontend/node_modules ]]; then
        (cd "$WEB_ROOT/frontend" && npm install) >>"$LOG_DIR/frontend.log" 2>&1
    fi
    setsid bash -c "cd '$WEB_ROOT/frontend' && exec npm run dev -- --host 0.0.0.0" \
        >>"$LOG_DIR/frontend.log" 2>&1 &
    FRONT_PID=$!
    PIDS+=( "$FRONT_PID" ); NAMES[$FRONT_PID]="frontend"
    echo "[boot] frontend pid=$FRONT_PID (log: $LOG_DIR/frontend.log)"
fi

# ── 살아있는 동안 안내 + 자식 모니터 ──────────────────────────────────
FRONTEND_STATUS="http://localhost:5173"
BACKEND_STATUS="http://localhost:8080  (swagger /swagger-ui.html)"
ADAPTER_STATUS="http://localhost:8000/health"
VIDEO_STATUS="disabled"
if [[ $SIM_MODE == hardware && "${ROBOT_VIDEO_ENABLE:-0}" != "0" && "${ROBOT_VIDEO_ENABLE:-0}" != "false" ]]; then
    VIDEO_STATUS="rtsp://localhost:${MEDIAMTX_RTSP_PORT}/${ROBOT_VIDEO_PATH} -> http://localhost:${MEDIAMTX_WEBRTC_PORT}/${ROBOT_VIDEO_PATH}"
fi
if [[ $WANT_FRONTEND != 1 ]]; then FRONTEND_STATUS="skipped"; fi
if [[ $WANT_BACKEND != 1 ]]; then BACKEND_STATUS="skipped"; fi
if [[ $WANT_ADAPTER != 1 ]]; then ADAPTER_STATUS="skipped"; fi
if [[ $WANT_BACKEND == 1 && ${BACKEND_READY:-0} != 1 ]]; then
    BACKEND_STATUS="unavailable (see backend.log)"
fi
if [[ $WANT_ADAPTER == 1 && ${ADAPTER_READY:-0} != 1 ]]; then
    ADAPTER_STATUS="unavailable (see adapter.log)"
fi

cat <<EOF

╔══════════════════════════════════════════════════════════════╗
║ 멀티세션 SLAM/Nav 스택 기동 완료                               ║
╠══════════════════════════════════════════════════════════════╣
║ Frontend:   $FRONTEND_STATUS
║ Backend :   $BACKEND_STATUS
║ Adapter :   $ADAPTER_STATUS
║ Camera  :   $VIDEO_STATUS
║ Foxglove:   ws://localhost:8765                                ║
║                                                                ║
║ Logs    :   $LOG_DIR
║                                                                ║
║ 로그 라이브 보기:                                              ║
║   tail -F $LOG_DIR/sim.log                                     ║
║                                                                ║
║ 종료: Ctrl-C 한 번                                            ║
╚══════════════════════════════════════════════════════════════╝
EOF

# 관제 웹과 Isaac/ROS 데이터 파이프라인은 분리한다. sim 이 죽거나 /odom, /scan 이
# 비어도 adapter/backend/frontend 는 계속 살아 있어야 한다. 사용자가 Ctrl-C 로 이
# launcher 자체를 종료할 때만 cleanup trap 이 자식들을 정리한다.
if [[ -z ${SIM_PID:-} ]]; then
    echo "[boot] no sim — web stack stays up until Ctrl-C"
else
    SIM_DEATH_REPORTED=0
    while true; do
        if [[ $SIM_DEATH_REPORTED == 0 ]] && ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "[warn] sim died — keeping rosbridge/adapter/backend/frontend alive"
            echo "       log: $LOG_DIR/sim.log"
            SIM_DEATH_REPORTED=1
        fi
        sleep 5
    done
fi
