#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SIM_HOST="${SIM_HOST:-100.80.87.68}"
MODE="${MODE:-initialize}"
ENABLE_STREAMS="${ENABLE_STREAMS:-true}"

is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

echo "[info] Isaac Sim ZMQ host: ${SIM_HOST}"
echo "[info] Sensor viewer: python3 viewer.py"
echo "[info] Navigation mode: ${MODE}"

if [ "$MODE" = "goal" ]; then
  if [ -z "${GOAL_X:-}" ] || [ -z "${GOAL_Y:-}" ]; then
    echo "[err] MODE=goal requires GOAL_X and GOAL_Y"
    exit 2
  fi
  args=(python3 examples/isaac_nav_client.py \
    --sim-host "$SIM_HOST" \
    --mode goal \
    --goal "$GOAL_X" "$GOAL_Y")
  if is_true "${RESET:-false}"; then
    args+=(--reset)
  fi
  if is_true "$ENABLE_STREAMS"; then
    args+=(--enable-streams)
  fi
  exec "${args[@]}"
fi

args=(python3 examples/isaac_nav_client.py \
  --sim-host "$SIM_HOST" \
  --mode "$MODE")
if is_true "${RESET:-false}"; then
  args+=(--reset)
fi
if is_true "$ENABLE_STREAMS"; then
  args+=(--enable-streams)
fi
exec "${args[@]}"
