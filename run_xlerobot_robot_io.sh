#!/usr/bin/env bash
# Compatibility wrapper. The Pi-side runtime is now rosbridge-only and does not
# require ROS 2 on the robot computer.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "[compat] run_xlerobot_robot_io.sh now delegates to run_xlerobot_rosbridge_io.sh"
exec "$SCRIPT_DIR/run_xlerobot_rosbridge_io.sh" "$@"
