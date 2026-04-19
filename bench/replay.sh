#!/bin/bash
# Bag replay — 옛 run의 입력 토픽 재생 + 새 mapper만 spawn.
#
# 사용법:
#   ./bench/replay.sh <run-dir> --mapper <preset>
#
# 예:
#   ./bench/replay.sh runs/20260419_140000_da3_nvblox --mapper vggt_only
#
# Gazebo 안 켬. ros2 bag play로 입력 토픽 재생 + 지정 preset의 mapper만 launch.
# 결과는 새 run dir에 저장됨.

set -e

REPO_ROOT="/root/gz-nav-sim"
BENCH_DIR="${REPO_ROOT}/bench"
PRESETS_DIR="${BENCH_DIR}/presets"
RUNS_DIR="${BENCH_DIR}/runs"

if [ -z "${1:-}" ]; then
    echo "사용법: $0 <run-dir> --mapper <preset>"
    exit 1
fi

SOURCE_RUN="$1"; shift
[ -d "${SOURCE_RUN}" ] || SOURCE_RUN="${RUNS_DIR}/${SOURCE_RUN}"
if [ ! -d "${SOURCE_RUN}/topics.bag" ]; then
    echo "[err] no bag in: ${SOURCE_RUN}/topics.bag"
    exit 1
fi

MAPPER=""
while [ $# -gt 0 ]; do
    case "$1" in
        --mapper) MAPPER="$2"; shift 2 ;;
        *) shift ;;
    esac
done

if [ -z "${MAPPER}" ]; then
    echo "[err] --mapper required"
    exit 1
fi

PRESET_FILE="${PRESETS_DIR}/${MAPPER}.sh"
[ -f "${PRESET_FILE}" ] || { echo "[err] preset not found: ${PRESET_FILE}"; exit 1; }
# shellcheck disable=SC1090
source "${PRESET_FILE}"

# Gazebo 끄기 위해 launch_args 수정 — headless로 강제 + simulator 없는 launch 별도 필요
# 현재 sim_nav.launch.py가 simulator + mapper 통합돼 있어서 단독 mapper-only launch가 없음.
# 임시 우회: launch_args에 추가 flag로 mapper만 띄우는 별도 launch가 필요.
echo "[replay] WARNING: 현재 launch는 mapper-only 모드 미지원."
echo "[replay] 우회 방법: 새 터미널에서 노드 직접 실행 + bag play 동시:"
echo ""
echo "  # 터미널 1 — bag play"
echo "  ros2 bag play ${SOURCE_RUN}/topics.bag --clock"
echo ""
echo "  # 터미널 2 — mapper만 (preset에 따라 직접 노드 spawn)"
case "${MAPPER}" in
    da3_nvblox)
        echo "  ros2 run gz_nav_sim da3_depth_node.py"
        echo "  # + nvblox_node 별도 spawn"
        ;;
    vggt_only|vggt_nvblox)
        echo "  ros2 run gz_nav_sim vggt_slam_bridge.py"
        ;;
esac
echo ""
echo "[replay] mapper-only launch 분리는 후속 작업. 현재는 매뉴얼 지침 출력."
exit 0
