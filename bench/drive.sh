#!/bin/bash
# Robot 주행 helper. 두 가지 모드:
#   1. teleop: cmd_vel 직접 publish (--mode teleop)
#   2. nav2: /goal_pose에 PoseStamped publish (Nav2 goal, --mode nav2)
#   3. square: nav2 goal로 사각형 주행 (--mode square)
#
# 사용법:
#   ./bench/drive.sh teleop                      # 30s 직진+회전
#   ./bench/drive.sh teleop --duration 60 --vx 0.4 --wz 0.2
#   ./bench/drive.sh nav2 --x 5 --y 0 --yaw 0    # 한 번 goal 보냄
#   ./bench/drive.sh square --side 3             # 3m 사각형
#
# Pre-req: ROS2 Humble + install/setup.bash 소스됨.
# bench/run.sh가 launch 띄운 상태에서 별도 터미널로 실행.

set -e

source /opt/ros/humble/setup.bash
source /root/gz-nav-sim/install/setup.bash 2>/dev/null || true

MODE="${1:-teleop}"; shift || true

DURATION=30
VX=0.3
WZ=0.15
GX=0; GY=0; GYAW=0
SIDE=3

while [ $# -gt 0 ]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --vx) VX="$2"; shift 2 ;;
        --wz) WZ="$2"; shift 2 ;;
        --x) GX="$2"; shift 2 ;;
        --y) GY="$2"; shift 2 ;;
        --yaw) GYAW="$2"; shift 2 ;;
        --side) SIDE="$2"; shift 2 ;;
        *) echo "[warn] unknown arg: $1"; shift ;;
    esac
done

# yaw → quaternion (z, w 만 — 평지 가정)
yaw_to_qzqw() {
    local yaw="$1"
    python3 -c "import math; y=float('$yaw'); print(f'{math.sin(y/2):.6f} {math.cos(y/2):.6f}')"
}

send_goal() {
    local x="$1" y="$2" yaw="$3"
    read qz qw <<< "$(yaw_to_qzqw "$yaw")"
    echo "[drive] nav2 goal: x=$x y=$y yaw=$yaw"
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, \
          orientation: {z: $qz, w: $qw}}}"
}

case "$MODE" in
    teleop)
        echo "[drive] teleop ${DURATION}s vx=$VX wz=$WZ → /cmd_vel_teleop"
        # velocity_smoother가 cmd_vel_teleop을 받아 cmd_vel로 출력하는 구조 가정
        # 안 되면 /cmd_vel 직접 publish로 fallback (아래 두 줄 주석 해제)
        timeout "$DURATION" ros2 topic pub --rate 20 /cmd_vel_teleop geometry_msgs/msg/Twist \
            "{linear: {x: $VX}, angular: {z: $WZ}}" || true
        # timeout "$DURATION" ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
        #     "{linear: {x: $VX}, angular: {z: $WZ}}" || true
        echo "[drive] teleop done"
        ;;
    nav2)
        send_goal "$GX" "$GY" "$GYAW"
        echo "[drive] goal sent. /navigate_to_pose 액션이 처리할 때까지 대기 안 함."
        ;;
    square)
        echo "[drive] square ${SIDE}m × ${SIDE}m, 4 goals"
        send_goal "$SIDE" 0 0; sleep 8
        send_goal "$SIDE" "$SIDE" 1.5708; sleep 8
        send_goal 0 "$SIDE" 3.1416; sleep 8
        send_goal 0 0 -1.5708; sleep 8
        echo "[drive] square done"
        ;;
    *)
        echo "[err] unknown mode: $MODE (teleop|nav2|square)"
        exit 1
        ;;
esac
