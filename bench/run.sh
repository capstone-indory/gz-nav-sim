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

# X server는 xvfb-run으로 launch에 wrap.
# 직접 Xvfb 관리 (이전): X server :99 mid-run crash 두 번 발생 (XIO error 33).
# xvfb-run -a: 자유 display 자동 선택, command 종료 시 자동 cleanup.
# DISPLAY는 xvfb-run이 자식 프로세스에 자동 inject.
XVFB_PID=""  # 더 이상 직접 spawn 안 함 (cleanup 호환용 빈 변수)

# ── ros2 bag record (background) ─────────────────────────────────────
BAG_PID=""
if [ "${RECORD}" = "true" ] && [ ${#RECORD_TOPICS[@]} -gt 0 ]; then
    echo "[bench] recording topics: ${RECORD_TOPICS[*]}"
    # Storage: mcap (ros-humble-rosbag2-storage-mcap 필요) — sqlite3보다
    # write throughput 3-5배 빠름 + 압축 지원 → disk IO stall 적음.
    # max-cache-size: 500MB. burst load 시 디스크 flush 대기로 메시지 drop 방지.
    # qos-profile-overrides: subscriber queue depth 200으로 deeper buffering.
    QOS_FILE="${RUN_DIR}/bag_qos.yaml"
    : > "${QOS_FILE}"
    for topic in "${RECORD_TOPICS[@]}"; do
        # /tf_static 등 transient_local은 publisher 프로필 그대로 따라가도록 skip.
        if [ "${topic}" = "/tf_static" ] || [ "${topic}" = "/map" ]; then
            continue
        fi
        cat >> "${QOS_FILE}" <<EOF
${topic}:
  history: keep_last
  depth: 200
  reliability: reliable
  durability: volatile
EOF
    done
    ros2 bag record -o "${RUN_DIR}/topics.bag" \
        -s mcap \
        --max-cache-size 500000000 \
        --qos-profile-overrides-path "${QOS_FILE}" \
        "${RECORD_TOPICS[@]}" > "${RUN_DIR}/bag.log" 2>&1 &
    BAG_PID=$!
    sleep 1
fi

# ── 종료 핸들러 ──────────────────────────────────────────────────────
LAUNCH_PID=""
CLEANUP_DONE=0
cleanup() {
    # INT/TERM → cleanup 후 EXIT trap이 또 호출되는 거 방지
    [ "${CLEANUP_DONE}" = "1" ] && return 0
    CLEANUP_DONE=1
    echo ""
    echo "[bench] shutting down..."
    [ -n "${BAG_PID}" ] && kill -INT "${BAG_PID}" 2>/dev/null && wait "${BAG_PID}" 2>/dev/null || true
    # setsid로 spawn했으므로 LAUNCH_PID = 새 process group의 leader (PGID).
    # xvfb-run은 SIGINT를 자식에 forward 안 하므로 PGID에 직접 SIGTERM → 전 자식 동시 종료.
    # 15초 grace: DA3 inference cycle이 길면 (publish 6초+) 5초로는 cuda context
    # 정리 못 함 → SIGKILL되며 GPU 메모리 leak 발생 (nvidia-smi에 process 없는데 GB 잔여).
    if [ -n "${LAUNCH_PID}" ]; then
        kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
        for _ in $(seq 1 75); do
            kill -0 -- "-${LAUNCH_PID}" 2>/dev/null || break
            sleep 0.2
        done
        kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
    [ -n "${EXPLORE_PID}" ] && kill "${EXPLORE_PID}" 2>/dev/null && wait "${EXPLORE_PID}" 2>/dev/null || true

    # ── 유령 방어 (name-based SIGKILL) ────────────────────────────────
    # PGID kill을 빠져나가는 경우: Python 노드가 SIGTERM handler로 shutdown 중
    # block + SIGKILL 타이밍 놓침 → parent 죽으면 init(sleep infinity)이 adopt →
    # 3일 넘게 살아있으며 cmd_vel 등 topic에 계속 publish (옛날 데이터 유령).
    # 과거 elevator_teleport 5개, vggt_slam_bridge가 이렇게 남는 사고 있었음.
    # 현 run의 자식은 PGID kill에서 이미 정리됐으니 name match 남은 건 전부 유령.
    for pat in \
        'ros2 launch explore_lite' \
        'explore_lite/explore' \
        'vggt_slam_bridge' \
        'vggt_slam_server' \
        'da3_depth_node' \
        'nvblox_node' \
        'nvblox_mesh_to_gltf' \
        'elevator_teleport' \
        'foxglove_bridge' \
        'async_slam_toolbox_node' \
        'gzserver' \
        'gzclient' \
        'component_container_isolated.*nav2' \
        'xvfb-run' \
        'Xvfb'; do
        pkill -KILL -f "$pat" 2>/dev/null || true
    done
    # DDS shm + launch params 잔여 정리 (다음 run이 깨끗하게 시작되도록)
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    rm -f /tmp/launch_params_* 2>/dev/null || true

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
# xvfb-run -a: 자유 display 자동 + command 종료 시 자동 cleanup.
# vglrun -d egl0: GPU EGL 경로. xvfb-run이 inject한 DISPLAY를 vglrun이 사용.
echo "[bench] launching via xvfb-run: ros2 launch gz_nav_sim sim_nav.launch.py ${LAUNCH_ARGS[*]}"
echo "[bench] ROS_LOG_DIR=${ROS_LOG_DIR}"
# setsid: 새 session + process group 리더로 spawn. LAUNCH_PID == PGID.
# 종료 시 kill -- -$LAUNCH_PID로 xvfb-run+vglrun+ros2 launch+모든 노드 한번에 종료.
setsid xvfb-run -a -s "-screen 0 1280x1024x24" \
    vglrun -d egl0 \
    ros2 launch gz_nav_sim sim_nav.launch.py "${LAUNCH_ARGS[@]}" \
    < /dev/null > "${RUN_DIR}/combined.log" 2>&1 &
LAUNCH_PID=$!

# --explore: Nav2 lifecycle ACTIVE까지 대기 후 explore_lite 자동 시작
# (action 토픽만 보이면 lifecycle은 아직 inactive일 수 있어 첫 goal에서 SIGSEGV 위험)
EXPLORE_PID=""
if [ "${EXPLORE}" = "true" ]; then
    (
        echo "[bench] waiting for Nav2 bt_navigator lifecycle = active (max 90s)..."
        for i in $(seq 1 45); do
            STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
            if echo "$STATE" | grep -q "active"; then
                echo "[bench] bt_navigator ACTIVE after ${i} × 2s"
                # planner_server / controller_server 도 확인
                CTRL_STATE=$(ros2 lifecycle get /controller_server 2>/dev/null | head -1)
                PLAN_STATE=$(ros2 lifecycle get /planner_server 2>/dev/null | head -1)
                echo "[bench]   controller_server: $CTRL_STATE"
                echo "[bench]   planner_server:    $PLAN_STATE"
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
