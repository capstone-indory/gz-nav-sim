#!/usr/bin/env bash
# 멀티세션 SLAM 풀스택을 한 번에 기동.
#
#   ROS2 시뮬 + RTAB-Map + ros_adapter (FastAPI :8000)
#   + Spring Boot (:8080)  + React Vite dev (:5173)  + Postgres (docker)
#
# 종료: Ctrl-C 한 번. 자식 프로세스 그룹 전체 SIGTERM.
#
# 사용법:
#   ./run_multisession_slam.sh                 # XLeRobot Hospital Isaac v2 의
#                                                /xlerobot ROS 토픽과 연결.
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

# DDS 격리: 같은 DOMAIN_ID 의 외부 ROS 노드가 토픽을 광고 중이면
# ros2 topic list 에 leak 되어 보임. ROS_LOCALHOST_ONLY=1 로 차단.
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
# DOMAIN_ID 도 명시 — default 0 은 충돌 위험. 환경변수로만 override.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

cd "$(dirname "$0")"
ROOT=$PWD
WEB_ROOT="$ROOT/indoors-web"

# ── 옵션 파싱 ──────────────────────────────────────────────────────────
WANT_FRONTEND=1
WANT_BACKEND=1
WANT_POSTGRES=1
WANT_SIM=1
WANT_ADAPTER=1
SIM_DURATION="${SIM_DURATION:-}"     # 빈 값이면 무한정 (Ctrl-C 까지)

while [[ $# -gt 0 ]]; do
    case "$1" in
        isaac) ;;
        --no-frontend) WANT_FRONTEND=0 ;;
        --no-backend)  WANT_BACKEND=0 ;;
        --no-postgres) WANT_POSTGRES=0 ;;
        --no-sim)      WANT_SIM=0 ;;
        --no-adapter)  WANT_ADAPTER=0 ;;
        -h|--help)
            sed -n '2,22p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
    shift
done

SIM_PRESET="d456_isaac"
echo "[boot] sim backend: isaac-v2  (preset: $SIM_PRESET)"

# ── 사전 점검 ──────────────────────────────────────────────────────────
need_cmd() {
    command -v "$1" >/dev/null 2>&1 || { echo "[err] missing: $1"; exit 1; }
}

[[ -d /opt/ros/humble ]] || { echo "[err] /opt/ros/humble 없음"; exit 1; }
[[ -d $ROOT/install/gz_nav_sim ]] || { echo "[err] colcon build 가 안 됨 — 'colcon build --symlink-install' 먼저"; exit 1; }
# Isaac 백엔드:
# - 기본 v2: Isaac 앱이 rosbridge_server 를 통해 /xlerobot ROS 토픽을 직접
#   만들기 때문에 추가 Python wire 의존성이 필요 없다.
# - legacy zmq_v1: 옛 sim_server 연결용 pyzmq/msgpack/zstandard 를 설치한다.
: "${ISAAC_TRANSPORT:=rosbridge_v2}"
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
    echo "[boot] isaac v2 대상: ROS graph /xlerobot/* topics via rosbridge_server"
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
    [[ -d /opt/corretto17 ]] || { echo "[err] /opt/corretto17 없음 — README §사전준비 4 참고"; exit 1; }
    # gradlew/Spring Boot 3.5 는 JDK 17+ 요구. ROS 설치 부수효과로 시스템 default 가
    # OpenJDK 11 이 되는 경우가 많아 JAVA_HOME 명시 안 하면 빌드 단계에서 죽음.
    export JAVA_HOME=/opt/corretto17
    export PATH=/opt/corretto17/bin:$PATH
fi

# ── 출력 디렉터리 ─────────────────────────────────────────────────────
TS=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$ROOT/bench/runs/${TS}_multisession"
mkdir -p "$LOG_DIR"
echo "[boot] logs → $LOG_DIR"

start_isaac_sim() {
    if [[ -z ${ISAAC_SIM_ROOT:-} ]]; then
        return 0
    fi
    if [[ ! -x "$ISAAC_SIM_LAUNCH" ]]; then
        echo "[warn] ISAAC_SIM_ROOT is set but Isaac launch script is missing or not executable: $ISAAC_SIM_LAUNCH"
        return 0
    fi
    echo "[boot] starting Isaac Sim app from $ISAAC_SIM_PROJECT"
    setsid env \
        ROSBRIDGE_HOST="${ROSBRIDGE_HOST:-}" \
        ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}" \
        NO_KEYBOARD=0 \
        bash -c "exec '$ISAAC_SIM_LAUNCH'" \
        >"$LOG_DIR/isaac_sim.log" 2>&1 &
    ISAAC_SIM_PID=$!
    PIDS+=("$ISAAC_SIM_PID")
    NAMES[$ISAAC_SIM_PID]="isaac_sim"
    echo "[boot] isaac sim pid=$ISAAC_SIM_PID (log: $LOG_DIR/isaac_sim.log)"
    sleep 2
}

start_isaac_sim

# ── 자식 프로세스 추적 + 정리 트랩 ────────────────────────────────────
declare -a PIDS=()
declare -A NAMES=()

cleanup() {
    # 재진입 방지 — Ctrl-C 두 번 누르면 SIGINT 다시 들어와 cleanup 재호출됨.
    [[ -n ${CLEANUP_RAN:-} ]] && return
    CLEANUP_RAN=1
    echo ""; echo "[exit] tearing down (force mode)..."

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
          JAVA_HOME=/opt/corretto17 PATH=/opt/corretto17/bin:$PATH \
          ./gradlew --stop >/dev/null 2>&1 ) &
    fi
    sleep 1
    if declare -F purge_stale >/dev/null; then purge_stale; fi
    echo "[exit] done"
}
trap cleanup EXIT INT TERM

# ── 완전 정리: 이전 세션의 모든 좀비 / 고아 프로세스 박멸 ──────────────
purge_stale() {
    # set -e 가 purge 안에서 죽지 않게 일시 해제.
    set +e
    # 1) 이름 기반 광범위 kill (Java/Gradle/Node 도 한꺼번에).
    pkill -9 -f 'rtabmap|foxglove_bridge|nav2|controller_server|planner_server|behavior_server|smoother_server|bt_navigator|waypoint_follower|velocity_smoother|component_container|java.*indoor|java.*IndooryApp|java.*spring|gradle.*bootRun|GradleDaemon|GradleWorker|gradlew|uvicorn|vite|node.*frontend|node.*vite|tf2_ros|static_transform_publisher|image_transport|republish|slam_toolbox|nvblox|da3_depth|explore_node|trajectory_path|launch_ros|ros2-daemon|joint_state_publisher|xlerobot_v2_bridge' 2>/dev/null
    sleep 1
    # 2) /opt/ros/humble/lib 에서 spawn 된 PID catch-all.
    pgrep -f '/opt/ros/humble/lib/' 2>/dev/null | xargs -r kill -9 2>/dev/null
    # 3) PPID=1 로 떨어진 우리 관련 고아 프로세스 (ROS + Java/Node/Python 웹 스택).
    for pid in $(ps -e -o pid=,ppid= 2>/dev/null | awk '$2==1{print $1}'); do
        cmdline=$(tr -d '\0' < /proc/$pid/cmdline 2>/dev/null)
        if [[ -n $cmdline ]] && [[ $cmdline =~ (ros|rclpy|rtabmap|gz_nav_sim|gradle|IndooryApp|indoors-web|spring-boot|uvicorn|vite|/opt/corretto17) ]]; then
            kill -9 "$pid" 2>/dev/null
        fi
    done
    # 4) 포트 점유 프로세스 직접 KILL (요청 응답 안 해도 listener 만 있으면 잡음).
    for port in 8080 8000 5173 8765 11345 5555 5556 5557; do
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

# ── rosbridge_server (Isaac v2 모드일 때) ────────────────────────────
if [[ $WANT_SIM == 1 && "${ISAAC_TRANSPORT:-rosbridge_v2}" == "rosbridge_v2" ]]; then
    echo "[boot] starting rosbridge_server on port 9090..."
    setsid bash -c "set -eo pipefail; \
        set +u; source /opt/ros/humble/setup.bash; set -u; \
        exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090" \
        >"$LOG_DIR/rosbridge_server.log" 2>&1 &
    ROSBRIDGE_PID=$!
    PIDS+=( "$ROSBRIDGE_PID" ); NAMES[$ROSBRIDGE_PID]="rosbridge_server"
    echo "[boot] rosbridge_server pid=$ROSBRIDGE_PID (log: $LOG_DIR/rosbridge_server.log)"
    
    echo -n "[boot] waiting for rosbridge_server :9090..."
    for _ in {1..15}; do
        if (echo > /dev/tcp/127.0.0.1/9090) >/dev/null 2>&1; then
            echo " ready"; break
        fi
        echo -n "."
        sleep 1
    done
fi

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
              ISAAC_TRANSPORT="${ISAAC_TRANSPORT:-rosbridge_v2}" \
        bash -c "exec $ROOT/bench/run.sh ${SIM_ARGS[*]}" \
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
    setsid env GZ_NAV_SIM_ROOT="$ROOT" \
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
        JAVA_HOME=/opt/corretto17 PATH=/opt/corretto17/bin:\$PATH \
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
if [[ $WANT_FRONTEND != 1 ]]; then FRONTEND_STATUS="skipped"; fi
if [[ $WANT_BACKEND != 1 ]]; then BACKEND_STATUS="skipped"; fi
if [[ $WANT_ADAPTER != 1 ]]; then ADAPTER_STATUS="skipped"; fi

cat <<EOF

╔══════════════════════════════════════════════════════════════╗
║ 멀티세션 SLAM 풀스택 기동 완료                                  ║
╠══════════════════════════════════════════════════════════════╣
║ Frontend:   $FRONTEND_STATUS
║ Backend :   $BACKEND_STATUS
║ Adapter :   $ADAPTER_STATUS
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

# sim 만 죽으면 전체 종료. 어댑터/백엔드/프론트는 자유롭게 재시작 가능 — 그 중 하나
# 죽었다고 sim 까지 같이 죽이면 사용자 데이터 손실 + 매핑 진행 잃음. SIM_PID 만 모니터.
# (이전 wait -n 은 어댑터 --reload 로 인한 일시 disappear 도 sim kill 트리거 → 위험)
if [[ -z ${SIM_PID:-} ]]; then
    echo "[boot] no sim — sleeping until Ctrl-C (Ctrl-C 로 모든 자식 cleanup)"
    sleep infinity
else
    while kill -0 "$SIM_PID" 2>/dev/null; do sleep 5; done
    echo "[exit] sim died — tearing down others"
fi
