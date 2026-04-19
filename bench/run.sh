#!/bin/bash
# Bench runner — preset 기반 launch + 자동 기록.
#
# 사용법:
#   ./bench/run.sh <preset> [--record] [--note "..."] [--duration <sec>] [--explore]
#
# 예:
#   ./bench/run.sh da3_nvblox --record --note "baseline 1차"
#   ./bench/run.sh vggt_only --duration 180 --explore
#
# preset = bench/presets/<name>.sh 파일명에서 .sh 제외
#
# 출력:
#   bench/runs/<timestamp>_<preset>/
#     ├ config.sh           # 사용된 preset
#     ├ launch_args.txt     # 실제 launch args
#     ├ git_commit.txt      # 코드 commit hash
#     ├ log/                # ROS2 launch가 기록하는 디렉토리 (ROS_LOG_DIR)
#     │   └ <ts>-<host>-<pid>/
#     │       └ launch.log  # 노드별 stdout/stderr가 prefix와 함께 합쳐진 로그
#     ├ topics.bag/         # ros2 bag (--record 시)
#     ├ metrics.json        # extract_metrics.py 결과 (launch.log 파싱)
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
EXPLORE=false
while [ $# -gt 0 ]; do
    case "$1" in
        --record) RECORD=true; shift ;;
        --note) NOTE="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --explore) EXPLORE=true; shift ;;
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

# ROS2 launch의 모든 노드 stdout/stderr가 launch.log에 timestamp + 노드명 prefix로
# 합쳐져 들어감. 별도 redirect 안 해도 됨.
export ROS_LOG_DIR="${RUN_DIR}/log"
mkdir -p "${ROS_LOG_DIR}"

# Xvfb on :99 — 항상 같은 display 사용 (다른 도구·문서와 호환)
# 컨테이너 PID 1=sleep이라 reap 안 됨 → 우리 trap에서 wait로 직접 reap.
# 이전 run의 zombie Xvfb (PPID=1)는 X 점유 안 하므로 무시 가능.
# 단 lock/socket 파일이 stale로 남아 있으면 정리.
export DISPLAY=:99
# 진짜 살아있는 Xvfb on :99 있나? (zombie 제외)
LIVE_XVFB=$(pgrep -f 'Xvfb.*:99' | while read pid; do
    [ -d "/proc/$pid" ] && [ "$(awk '/^State:/{print $2}' /proc/$pid/status)" != "Z" ] && echo "$pid"
done | head -1)
if [ -n "${LIVE_XVFB}" ]; then
    echo "[bench] live Xvfb on :99 already (PID=${LIVE_XVFB}). 종료 후 재시작."
    kill -TERM "${LIVE_XVFB}" 2>/dev/null && wait "${LIVE_XVFB}" 2>/dev/null
fi
# Stale lock/socket 정리 (live Xvfb 없을 때만 — 실수 방지 위해 위 블록 후)
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99 2>/dev/null
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!
sleep 2
if ! ps -p $XVFB_PID > /dev/null; then
    echo "[bench] Xvfb 시작 실패"; exit 1
fi
echo "[bench] Xvfb :99 PID=${XVFB_PID}"

# ── ros2 bag record (background) ─────────────────────────────────────
BAG_PID=""
if [ "${RECORD}" = "true" ] && [ ${#RECORD_TOPICS[@]} -gt 0 ]; then
    echo "[bench] recording topics: ${RECORD_TOPICS[*]}"
    # ros2 bag record default storage = sqlite3 (humble 기본).
    # mcap은 별도 패키지 필요해서 안 씀.
    ros2 bag record -o "${RUN_DIR}/topics.bag" \
        "${RECORD_TOPICS[@]}" > "${RUN_DIR}/bag.log" 2>&1 &
    BAG_PID=$!
    sleep 1
fi

# ── 종료 핸들러 ──────────────────────────────────────────────────────
LAUNCH_PID=""
cleanup() {
    echo ""
    echo "[bench] shutting down..."
    # 자식 프로세스를 SIGINT 후 wait — 컨테이너 PID 1=sleep라 우리가 reap 안 하면
    # zombie 누적. wait가 자식 reap.
    [ -n "${BAG_PID}" ] && kill -INT "${BAG_PID}" 2>/dev/null && wait "${BAG_PID}" 2>/dev/null || true
    [ -n "${LAUNCH_PID}" ] && kill -INT "${LAUNCH_PID}" 2>/dev/null && wait "${LAUNCH_PID}" 2>/dev/null || true
    # Launch가 spawn한 자식들 — pkill 후 init(=sleep)에 orphan으로 가지만
    # 우리가 wait 못 함. zombie는 system 재시작 전까진 남음 (불가피).
    pkill -f 'ros2 launch gz_nav_sim' 2>/dev/null || true
    pkill -f 'ros2 launch explore_lite' 2>/dev/null || true
    pkill -f 'explore_lite/explore' 2>/dev/null || true
    pkill -f 'da3_depth_node' 2>/dev/null || true
    pkill -f 'nvblox_node' 2>/dev/null || true
    pkill -f 'vggt_slam_bridge' 2>/dev/null || true
    pkill -f 'vggt_slam_server' 2>/dev/null || true
    pkill -f 'foxglove_bridge' 2>/dev/null || true
    pkill -f 'gzserver|gzclient' 2>/dev/null || true
    [ -n "${EXPLORE_PID}" ] && kill "${EXPLORE_PID}" 2>/dev/null && wait "${EXPLORE_PID}" 2>/dev/null || true
    # Xvfb는 우리 직속 자식 → kill + wait로 깔끔하게 reap
    if [ -n "${XVFB_PID}" ]; then
        kill -TERM "${XVFB_PID}" 2>/dev/null && wait "${XVFB_PID}" 2>/dev/null || true
    fi
    rm -f /tmp/.X99-lock /tmp/.X11-unix/X99 2>/dev/null

    # metrics 추출 — combined.log (노드 stdout/stderr)이 더 풍부.
    # 없으면 launch.log fallback (ROS launch 시스템 이벤트만)
    METRICS_SRC=""
    if [ -f "${RUN_DIR}/combined.log" ] && [ -s "${RUN_DIR}/combined.log" ]; then
        METRICS_SRC="${RUN_DIR}/combined.log"
    else
        METRICS_SRC=$(find "${RUN_DIR}/log" -name 'launch.log' 2>/dev/null | head -1)
    fi
    if [ -n "${METRICS_SRC}" ] && [ -f "${METRICS_SRC}" ]; then
        echo "[bench] extracting metrics from ${METRICS_SRC}"
        python3 "${BENCH_DIR}/extract_metrics.py" \
            "${METRICS_SRC}" > "${RUN_DIR}/metrics.json" 2>&1 || \
            echo "{\"error\": \"extract failed\"}" > "${RUN_DIR}/metrics.json"
    else
        echo "[bench] no log to parse — skip metrics"
        echo '{"error": "no log"}' > "${RUN_DIR}/metrics.json"
    fi
    echo "finished: $(date +%Y%m%d_%H%M%S)" >> "${RUN_DIR}/notes.md"
    echo "[bench] done. results: ${RUN_DIR}"
}
trap cleanup EXIT INT TERM

# ── launch 실행 ──────────────────────────────────────────────────────
# vglrun이 SSH client IP로 DISPLAY 자동 override하는 거 차단 — VGL_DISPLAY로 강제
export VGL_DISPLAY=":99"
echo "[bench] launching: vglrun -d egl0 ros2 launch gz_nav_sim sim_nav.launch.py ${LAUNCH_ARGS[*]}"
echo "[bench] ROS_LOG_DIR=${ROS_LOG_DIR} DISPLAY=${DISPLAY} VGL_DISPLAY=${VGL_DISPLAY}"
# launch 파일 모든 노드가 output='screen' → ros2 launch stdout으로 갬.
# /dev/null로 묻으면 노드 출력 다 잃음. RUN_DIR/combined.log에 캡처.
# (launch.log는 ROS launch 시스템 이벤트, combined.log은 노드 stdout/stderr)
vglrun -d egl0 ros2 launch gz_nav_sim sim_nav.launch.py "${LAUNCH_ARGS[@]}" \
    < /dev/null > "${RUN_DIR}/combined.log" 2>&1 &
LAUNCH_PID=$!

# --explore: Nav2 활성화 대기 후 explore_lite 자동 시작 (frontier-based 자율 탐사)
EXPLORE_PID=""
if [ "${EXPLORE}" = "true" ]; then
    (
        # Nav2의 /navigate_to_pose action 등장까지 대기 (max 60s)
        echo "[bench] waiting for Nav2 to be ready..."
        for i in $(seq 1 30); do
            if ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
                echo "[bench] Nav2 ready after ${i} × 2s"
                break
            fi
            sleep 2
        done
        echo "[bench] starting explore_lite..."
        ros2 launch explore_lite explore.launch.py use_sim_time:=true \
            >> "${RUN_DIR}/combined.log" 2>&1
    ) &
    EXPLORE_PID=$!
fi

# duration 지정 시 자동 종료
if [ "${DURATION}" -gt 0 ]; then
    echo "[bench] auto-stop after ${DURATION}s"
    sleep "${DURATION}"
    cleanup
    exit 0
fi

echo "[bench] launch PID=${LAUNCH_PID}"
echo "[bench] live tail: tail -F ${ROS_LOG_DIR}/*/launch.log"
echo "[bench] Ctrl+C to stop"
wait "${LAUNCH_PID}"
