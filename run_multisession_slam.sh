#!/usr/bin/env bash
# 멀티세션 SLAM 풀스택을 한 번에 기동.
#
#   ROS2 시뮬 + RTAB-Map + ros_adapter (FastAPI :8000)
#   + Spring Boot (:8080)  + React Vite dev (:5173)  + Postgres (docker)
#
# 종료: Ctrl-C 한 번. 자식 프로세스 그룹 전체 SIGTERM.
#
# 사용법:
#   ./run_multisession_slam.sh                # 모두 기동, foreground 로그 통합
#   ./run_multisession_slam.sh --no-frontend  # 프론트 없이 (이미 띄워둔 경우)
#   ./run_multisession_slam.sh --no-postgres  # postgres 외부에서 관리
#   ./run_multisession_slam.sh --no-backend   # 시뮬+adapter 만 (REST 직접 테스트)
#   SIM_DURATION=120 ./run_multisession_slam.sh   # 시뮬에 자동 종료 시간 (디버깅)
#
# 로그: bench/runs/<ts>_multisession/{sim.log,adapter.log,backend.log,frontend.log}

set -euo pipefail

cd "$(dirname "$0")"
ROOT=$PWD

# ── 옵션 파싱 ──────────────────────────────────────────────────────────
WANT_FRONTEND=1
WANT_BACKEND=1
WANT_POSTGRES=1
WANT_SIM=1
WANT_ADAPTER=1
SIM_DURATION="${SIM_DURATION:-}"     # 빈 값이면 무한정 (Ctrl-C 까지)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-frontend) WANT_FRONTEND=0 ;;
        --no-backend)  WANT_BACKEND=0 ;;
        --no-postgres) WANT_POSTGRES=0 ;;
        --no-sim)      WANT_SIM=0 ;;
        --no-adapter)  WANT_ADAPTER=0 ;;
        -h|--help)
            sed -n '2,16p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
    shift
done

# ── 사전 점검 ──────────────────────────────────────────────────────────
need_cmd() {
    command -v "$1" >/dev/null 2>&1 || { echo "[err] missing: $1"; exit 1; }
}

[[ -d /opt/ros/humble ]] || { echo "[err] /opt/ros/humble 없음"; exit 1; }
[[ -d $ROOT/install/gz_nav_sim ]] || { echo "[err] colcon build 가 안 됨 — 'colcon build --symlink-install' 먼저"; exit 1; }
need_cmd xvfb-run

if [[ $WANT_BACKEND == 1 ]]; then
    [[ -d /opt/corretto17 ]] || { echo "[err] /opt/corretto17 없음 — README §사전준비 4 참고"; exit 1; }
fi

# ── 출력 디렉터리 ─────────────────────────────────────────────────────
TS=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$ROOT/bench/runs/${TS}_multisession"
mkdir -p "$LOG_DIR"
echo "[boot] logs → $LOG_DIR"

# ── 자식 프로세스 추적 + 정리 트랩 ────────────────────────────────────
declare -a PIDS=()
declare -A NAMES=()

cleanup() {
    echo ""; echo "[exit] tearing down..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "  kill ${NAMES[$pid]} (pid=$pid, pgid=$(ps -o pgid= "$pid" | tr -d ' '))"
            kill -- -"$(ps -o pgid= "$pid" | tr -d ' ')" 2>/dev/null || true
        fi
    done
    sleep 2
    # 같은 purge_stale 로 ROS 잔존·orphan·shm·tmp 모두 정리 — 다음 실행이 깨끗.
    if declare -F purge_stale >/dev/null; then purge_stale; fi
    echo "[exit] done"
}
trap cleanup EXIT INT TERM

# ── 완전 정리: 이전 세션의 모든 좀비 / 고아 프로세스 박멸 ──────────────
purge_stale() {
    # 1) 이름 기반 광범위 kill (1차).
    pkill -9 -f 'gzserver|gzclient|rtabmap|foxglove_bridge|nav2|controller_server|planner_server|behavior_server|smoother_server|bt_navigator|waypoint_follower|velocity_smoother|component_container|xvfb-run|Xvfb|java.*indoor|java.*IndooryApp|gradle.*bootRun|GradleDaemon|uvicorn|vite|node.*frontend|tf2_ros|static_transform_publisher|image_transport|republish|elevator_teleport|slam_toolbox|nvblox|da3_depth|explore_node|trajectory_path|launch_ros|ros2-daemon|joint_state_publisher|gazebo_ros' 2>/dev/null || true
    sleep 1
    # 2) /opt/ros/humble/lib 에서 spawn 된 모든 프로세스 (1차에서 못 잡은 것들).
    #    cmdline 에 /opt/ros 포함하는 PID 모두 SIGKILL.
    for pid in $(pgrep -f '/opt/ros/humble/lib/' 2>/dev/null); do
        kill -9 "$pid" 2>/dev/null || true
    done
    # 3) PPID=1 로 떨어진 ROS 관련 고아 프로세스 (init 입양된 것들).
    while read -r pid; do
        if [[ -n $pid ]] && grep -qE 'ros|rclpy|gazebo|rtabmap|gz_nav_sim' \
                /proc/$pid/cmdline 2>/dev/null | head -1; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done < <(ps -e -o pid=,ppid= | awk '$2==1{print $1}')
    # 4) 포트 점유 프로세스 직접 KILL (8080/8000/5173/8765/11345).
    for port in 8080 8000 5173 8765 11345; do
        pid=$(ss -lntp 2>/dev/null | awk -v p=":$port" '$0 ~ p {print}' | grep -oP 'pid=\K\d+' | head -1 || true)
        if [[ -n $pid ]]; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    # 5) FastDDS shared memory + lockfile 정리 — 안 그러면 좀비 토픽 discovery 잔존.
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    # 6) ros2 launch 가 만든 임시 param 파일.
    rm -f /tmp/launch_params_* 2>/dev/null || true
    # 7) Xvfb lockfile (이전 비정상 종료 시 남음).
    rm -f /tmp/.X99-lock /tmp/.X11-unix/X99 2>/dev/null || true
    sleep 1
}

echo "[boot] killing stale ROS/Gazebo if any..."
purge_stale
# 검증: 남은 ros 관련 프로세스 카운트.
remaining=$(pgrep -f '/opt/ros/humble/lib/|gzserver|rtabmap|uvicorn|vite|java.*indoor' 2>/dev/null | wc -l)
if [[ $remaining -gt 0 ]]; then
    echo "[warn] $remaining stale processes still alive after purge — running second pass"
    purge_stale
    remaining=$(pgrep -f '/opt/ros/humble/lib/|gzserver|rtabmap|uvicorn|vite|java.*indoor' 2>/dev/null | wc -l)
    if [[ $remaining -gt 0 ]]; then
        echo "[err] $remaining stubborn processes — listing for manual review:"
        pgrep -fa '/opt/ros/humble/lib/|gzserver|rtabmap|uvicorn|vite|java.*indoor' 2>/dev/null | head -10
    fi
fi

# ── Postgres (네이티브) ───────────────────────────────────────────────
# 도커 의존성 제거: apt 패키지 + service postgresql 로 실행.
# DB/유저는 indoory:indoory@localhost:5432/indoory (compose 와 동일 자격증명).
postgres_reachable() {
    (echo > /dev/tcp/127.0.0.1/5432) >/dev/null 2>&1
}

_psql_super() {
    # postgres OS 유저로 psql 실행 (sudo 없는 환경 호환).
    # postgres 가 cd 못 하는 디렉터리 경고 방지 위해 /tmp 에서 실행.
    if command -v sudo >/dev/null 2>&1; then
        (cd /tmp && sudo -u postgres psql "$@")
    else
        (cd /tmp && su -s /bin/bash postgres -c "psql $(printf '%q ' "$@")")
    fi
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
        echo "[boot] postgres reachable on :5432 — using existing"
        ensure_indoory_db || true
    else
        # 1) 패키지 설치 (없으면)
        if ! command -v pg_ctlcluster >/dev/null 2>&1 && ! command -v pg_isready >/dev/null 2>&1; then
            echo "[boot] installing postgresql (apt)..."
            apt-get install -y postgresql >>"$LOG_DIR/postgres.log" 2>&1 \
                || { echo "[err] apt install postgresql 실패 — $LOG_DIR/postgres.log"; exit 1; }
        fi
        # 2) 서비스 기동 — sysvinit (service) / pg_ctlcluster 모두 시도
        echo "[boot] starting postgres (native)..."
        if command -v service >/dev/null 2>&1; then
            service postgresql start >>"$LOG_DIR/postgres.log" 2>&1 || true
        fi
        if ! postgres_reachable && command -v pg_ctlcluster >/dev/null 2>&1; then
            # cluster 자동 시작
            cluster=$(pg_lsclusters -h 2>/dev/null | awk 'NR==1 {print $1, $2}')
            if [[ -n $cluster ]]; then
                pg_ctlcluster $cluster start >>"$LOG_DIR/postgres.log" 2>&1 || true
            fi
        fi
        # 3) readiness wait
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

# ── 시뮬레이터 (RTAB-Map + Gazebo + Nav2) ─────────────────────────────
if [[ $WANT_SIM == 1 ]]; then
    echo "[boot] starting sim (d456_rtabmap preset)..."
    SIM_ARGS=( d456_rtabmap )
    if [[ -n $SIM_DURATION ]]; then
        SIM_ARGS+=( --duration "$SIM_DURATION" )
    fi
    # bench/run.sh 가 ROS_LOG_DIR + xvfb-run 처리. setsid 로 새 PG 만들어 정리 가능하게.
    setsid bash -c "exec $ROOT/bench/run.sh ${SIM_ARGS[*]}" \
        >"$LOG_DIR/sim.log" 2>&1 &
    SIM_PID=$!
    PIDS+=( "$SIM_PID" ); NAMES[$SIM_PID]="sim"
    echo "[boot] sim pid=$SIM_PID (log: $LOG_DIR/sim.log)"

    # /odom 토픽 올라올 때까지 대기 (최대 60초)
    echo -n "[boot] waiting for /odom..."
    # ROS2 setup.bash 가 unset 변수 참조 → set -u 와 충돌. 이 블록만 해제.
    set +u
    source /opt/ros/humble/setup.bash
    source "$ROOT/install/setup.bash"
    set -u
    for _ in {1..60}; do
        if ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
            echo " ready"; break
        fi
        echo -n "."
        sleep 1
    done
fi

# ── ros_adapter (FastAPI :8000) ────────────────────────────────────────
if [[ $WANT_ADAPTER == 1 ]]; then
    echo "[boot] starting ros_adapter..."
    setsid bash -c "exec $ROOT/indoors-web/ros_adapter/run.sh" \
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
            echo ""; echo "[err] adapter process died — see $LOG_DIR/adapter.log"
            tail -20 "$LOG_DIR/adapter.log" 2>/dev/null
            exit 1
        fi
        echo -n "."
        sleep 1
    done
    if [[ $ADAPTER_READY != 1 ]]; then
        echo ""; echo "[err] adapter not responsive after 60s"; exit 1
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
    setsid bash -c "cd $ROOT/indoors-web/backend && \
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
            echo ""; echo "[err] backend process died — see $LOG_DIR/backend.log"
            tail -40 "$LOG_DIR/backend.log" 2>/dev/null
            exit 1
        fi
        echo -n "."
        sleep 1
    done
    if [[ $BACKEND_READY != 1 ]]; then
        echo ""; echo "[err] backend not ready after 240s"
        tail -40 "$LOG_DIR/backend.log" 2>/dev/null
        exit 1
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
    if [[ ! -d $ROOT/indoors-web/frontend/node_modules ]]; then
        (cd "$ROOT/indoors-web/frontend" && npm install) >>"$LOG_DIR/frontend.log" 2>&1
    fi
    setsid bash -c "cd $ROOT/indoors-web/frontend && exec npm run dev -- --host 0.0.0.0" \
        >>"$LOG_DIR/frontend.log" 2>&1 &
    FRONT_PID=$!
    PIDS+=( "$FRONT_PID" ); NAMES[$FRONT_PID]="frontend"
    echo "[boot] frontend pid=$FRONT_PID (log: $LOG_DIR/frontend.log)"
fi

# ── 살아있는 동안 안내 + 자식 모니터 ──────────────────────────────────
cat <<EOF

╔══════════════════════════════════════════════════════════════╗
║ 멀티세션 SLAM 풀스택 기동 완료                                  ║
╠══════════════════════════════════════════════════════════════╣
║ Frontend:   http://localhost:5173                              ║
║ Backend :   http://localhost:8080  (swagger /swagger-ui.html)  ║
║ Adapter :   http://localhost:8000/health                       ║
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

# 자식 중 누가 죽으면 전체 종료. wait -n 으로 첫 사망 대기.
# 자식이 없는 경우 (모든 --no-* 옵션) 는 그냥 sleep infinity — Ctrl-C 까지 대기.
if [[ ${#PIDS[@]} -eq 0 ]]; then
    echo "[boot] no managed children — sleeping until Ctrl-C"
    sleep infinity
else
    wait -n "${PIDS[@]}" 2>/dev/null || true
    echo "[exit] one of the children died — tearing down others"
fi
