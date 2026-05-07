#!/usr/bin/env bash
# Reset only the local Isaac ZMQ clients: HTTP viewer state, local grid map,
# OCR annotations, and the navigation client process. The Isaac Sim server is
# not signalled or restarted by this script.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

HTTP_PORT="${HTTP_PORT:-18081}"
export SIM_HOST="${SIM_HOST:-100.80.87.68}"
export HTTP_PORT
LOG_FILE="${LOG_FILE:-/tmp/isaac_zmq_stack.log}"

curl -fsS "http://127.0.0.1:${HTTP_PORT}/reset_local" >/dev/null 2>&1 || true

pkill -TERM -f '[b]ash ./run_isaac_zmq_stack.sh' 2>/dev/null || true
pkill -TERM -f '[p]ython3 examples/web_viewer.py' 2>/dev/null || true
pkill -TERM -f '[p]ython3 examples/isaac_nav_client.py' 2>/dev/null || true
sleep 0.5

setsid ./run_isaac_zmq_stack.sh >"$LOG_FILE" 2>&1 </dev/null &
echo "local Isaac ZMQ stack restarted; log=${LOG_FILE}"
