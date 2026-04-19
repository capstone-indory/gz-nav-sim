#!/bin/bash
# Bench runner — preset 기반 launch + 자동 기록.
#
# 사용법:
#   ./bench/run.sh <preset> [--record] [--note "..."] [--duration <sec>]
#
# 예:
#   ./bench/run.sh da3_nvblox --record --note "baseline 1차"
#   ./bench/run.sh vggt_only --duration 180
#
# preset = bench/presets/<name>.sh 파일명에서 .sh 제외
#
# 출력:
#   bench/runs/<timestamp>_<preset>/
#     ├ config.sh           # 사용된 preset
#     ├ launch_args.txt     # 실제 launch args
#     ├ git_commit.txt      # 코드 commit hash
#     ├ stdout.log          # 전체 launch 출력
#     ├ topics.bag/         # ros2 bag (--record 시)
#     ├ metrics.json        # extract_metrics.py 결과
#     └ notes.md            # --note 내용

set -e

REPO_ROOT="/root/gz-nav-sim"
BENCH_DIR="${REPO_ROOT}/bench"
PRESETS_DIR="${BENCH_DIR}/presets"
RUNS_DIR="${BENCH_DIR}/runs"

# ── 인자 파싱 ─────────────────────────────────────────────────────────
if [ -z "${1:-}" ]; then
    echo "사용법: $0 <preset> [--record] [--note \"...\"] [--duration <sec>]"
    echo "preset 목록:"
    for f in "${PRESETS_DIR}"/*.sh; do
        name=$(basename "$f" .sh)
        # shellcheck disable=SC1090
        (source "$f"; echo "  $name — ${PRESET_DESC:-(no desc)}")
    done
    exit 1
fi

PRESET="$1"; shift
PRESET_FILE="${PRESETS_DIR}/${PRESET}.sh"
if [ ! -f "${PRESET_FILE}" ]; then
    echo "[err] preset not found: ${PRESET_FILE}"
    exit 1
fi

RECORD=false
NOTE=""
DURATION=0
while [ $# -gt 0 ]; do
    case "$1" in
        --record) RECORD=true; shift ;;
        --note) NOTE="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        *) echo "[warn] unknown arg: $1"; shift ;;
    esac
done

# ── preset 로드 ──────────────────────────────────────────────────────
# shellcheck disable=SC1090
source "${PRESET_FILE}"

# ── run 디렉토리 ─────────────────────────────────────────────────────
TS=$(date +%Y%m%d_%H%M%S)
RUN_DIR="${RUNS_DIR}/${TS}_${PRESET_NAME}"
mkdir -p "${RUN_DIR}"
echo "[bench] run dir: ${RUN_DIR}"

# 메타 저장
cp "${PRESET_FILE}" "${RUN_DIR}/config.sh"
printf '%s\n' "${LAUNCH_ARGS[@]}" > "${RUN_DIR}/launch_args.txt"
git -C "${REPO_ROOT}" rev-parse HEAD > "${RUN_DIR}/git_commit.txt" 2>/dev/null || \
    echo "unknown" > "${RUN_DIR}/git_commit.txt"
git -C "${REPO_ROOT}" status --short > "${RUN_DIR}/git_dirty.txt" 2>/dev/null || true
[ -n "${NOTE}" ] && echo "${NOTE}" > "${RUN_DIR}/notes.md"
echo "preset: ${PRESET_NAME}" >> "${RUN_DIR}/notes.md"
echo "started: ${TS}" >> "${RUN_DIR}/notes.md"

# ── 환경 ────────────────────────────────────────────────────────────
cd "${REPO_ROOT}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Stale X 정리 + Xvfb
pkill -f 'Xvfb.*:99' 2>/dev/null || true
rm -f /tmp/.X11-unix/X99 /tmp/.X99-lock 2>/dev/null || true
sleep 1
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!
sleep 2

# ── ros2 bag record (background) ─────────────────────────────────────
BAG_PID=""
if [ "${RECORD}" = "true" ] && [ ${#RECORD_TOPICS[@]} -gt 0 ]; then
    echo "[bench] recording topics: ${RECORD_TOPICS[*]}"
    ros2 bag record -o "${RUN_DIR}/topics.bag" \
        --storage mcap \
        "${RECORD_TOPICS[@]}" > "${RUN_DIR}/bag.log" 2>&1 &
    BAG_PID=$!
    sleep 1
fi

# ── 종료 핸들러 ──────────────────────────────────────────────────────
LAUNCH_PID=""
cleanup() {
    echo ""
    echo "[bench] shutting down..."
    [ -n "${BAG_PID}" ] && kill -INT "${BAG_PID}" 2>/dev/null && wait "${BAG_PID}" 2>/dev/null || true
    [ -n "${LAUNCH_PID}" ] && kill -INT "${LAUNCH_PID}" 2>/dev/null && wait "${LAUNCH_PID}" 2>/dev/null || true
    pkill -f 'ros2 launch gz_nav_sim' 2>/dev/null || true
    pkill -f 'da3_depth_node' 2>/dev/null || true
    pkill -f 'nvblox_node' 2>/dev/null || true
    pkill -f 'vggt_slam_server' 2>/dev/null || true
    kill "${XVFB_PID}" 2>/dev/null || true
    pkill -f 'Xvfb.*:99' 2>/dev/null || true

    # metrics 추출
    if [ -f "${RUN_DIR}/stdout.log" ]; then
        echo "[bench] extracting metrics..."
        python3 "${BENCH_DIR}/extract_metrics.py" \
            "${RUN_DIR}/stdout.log" > "${RUN_DIR}/metrics.json" 2>&1 || \
            echo "{\"error\": \"extract failed\"}" > "${RUN_DIR}/metrics.json"
    fi
    echo "finished: $(date +%Y%m%d_%H%M%S)" >> "${RUN_DIR}/notes.md"
    echo "[bench] done. results: ${RUN_DIR}"
}
trap cleanup EXIT INT TERM

# ── launch 실행 ──────────────────────────────────────────────────────
echo "[bench] launching: vglrun -d egl0 ros2 launch gz_nav_sim sim_nav.launch.py ${LAUNCH_ARGS[*]}"
vglrun -d egl0 ros2 launch gz_nav_sim sim_nav.launch.py "${LAUNCH_ARGS[@]}" \
    > "${RUN_DIR}/stdout.log" 2>&1 &
LAUNCH_PID=$!

# duration 지정 시 자동 종료
if [ "${DURATION}" -gt 0 ]; then
    echo "[bench] auto-stop after ${DURATION}s"
    sleep "${DURATION}"
    cleanup
    exit 0
fi

echo "[bench] launch PID=${LAUNCH_PID}, log=${RUN_DIR}/stdout.log"
echo "[bench] Ctrl+C to stop"
wait "${LAUNCH_PID}"
