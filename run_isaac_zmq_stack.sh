#!/usr/bin/env bash
# Pure Isaac Sim ZMQ stack: HTTP monitor + lidar/depth reactive navigation.
# No ROS, Gazebo, Nav2, or Foxglove processes are started here.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PYTHONUNBUFFERED=1

SIM_HOST="${SIM_HOST:-100.80.87.68}"
HTTP_PORT="${HTTP_PORT:-18081}"
OCR_BACKEND="${OCR_BACKEND:-gazebo}"
FLOOR_HINT="${FLOOR_HINT:-5}"
FLOOR_PRIOR_MODE="${FLOOR_PRIOR_MODE:-complete}"
OCR_SCALES="${OCR_SCALES:-1.0,2.0,3.0,4.0,6.0}"
OCR_MAX_SIDE="${OCR_MAX_SIDE:-2400}"
RUN_NAV="${RUN_NAV:-true}"
MODE="${MODE:-initialize}"
ENABLE_STREAMS="${ENABLE_STREAMS:-true}"
RESTART_VIEWER="${RESTART_VIEWER:-true}"
GRID_RESOLUTION_M="${GRID_RESOLUTION_M:-0.10}"
GRID_SIZE_M="${GRID_SIZE_M:-80.0}"
GRID_VIEW_M="${GRID_VIEW_M:-18.0}"

echo "[info] Isaac Sim ZMQ host: ${SIM_HOST}"
echo "[info] HTTP monitor: http://0.0.0.0:${HTTP_PORT}"
echo "[info] OCR backend: ${OCR_BACKEND}"
echo "[info] Navigation: ${RUN_NAV} (${MODE})"
echo

is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

if is_true "$RESTART_VIEWER"; then
  pkill -f '[p]ython3 examples/web_viewer.py' 2>/dev/null || true
fi

python3 examples/web_viewer.py \
  --sim-host "$SIM_HOST" \
  --http-port "$HTTP_PORT" \
  --ocr-backend "$OCR_BACKEND" \
  --floor-hint "$FLOOR_HINT" \
  --floor-prior-mode "$FLOOR_PRIOR_MODE" \
  --ocr-scales "$OCR_SCALES" \
  --ocr-max-side "$OCR_MAX_SIDE" \
  --grid-resolution-m "$GRID_RESOLUTION_M" \
  --grid-size-m "$GRID_SIZE_M" \
  --grid-view-m "$GRID_VIEW_M" &
VIEWER_PID=$!

cleanup() {
  kill "$VIEWER_PID" "${NAV_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

if ! is_true "$RUN_NAV"; then
  wait "$VIEWER_PID"
  exit $?
fi

nav_args=(python3 examples/isaac_nav_client.py
  --sim-host "$SIM_HOST"
  --mode "$MODE")

if is_true "$ENABLE_STREAMS"; then
  nav_args+=(--enable-streams)
fi
if is_true "${RESET:-false}"; then
  nav_args+=(--reset)
fi
if is_true "${DISABLE_DEPTH_NAV:-false}"; then
  nav_args+=(--disable-depth-nav)
fi

if [ "$MODE" = "goal" ]; then
  if [ -z "${GOAL_X:-}" ] || [ -z "${GOAL_Y:-}" ]; then
    echo "[err] MODE=goal requires GOAL_X and GOAL_Y"
    exit 2
  fi
  nav_args+=(--goal "$GOAL_X" "$GOAL_Y")
fi

"${nav_args[@]}" &
NAV_PID=$!

wait -n "$VIEWER_PID" "$NAV_PID"
