#!/usr/bin/env bash
set -euo pipefail

packages=(
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly
  gstreamer1.0-libav
)

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[err] apt-get not found; install GStreamer H.264 plugins with your OS package manager." >&2
  exit 1
fi

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  if ! sudo -n true >/dev/null 2>&1; then
    echo "[err] sudo needs a password. Run this script from your terminal so sudo can prompt:" >&2
    echo "      sudo $0" >&2
    echo "      or: sudo apt install -y ${packages[*]}" >&2
    exit 1
  fi
  SUDO=(sudo)
else
  SUDO=()
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y "${packages[@]}"

missing=()
for plugin in rawvideoparse videoconvert x264enc h264parse mpegtsmux hlssink; do
  if ! gst-inspect-1.0 "$plugin" >/dev/null 2>&1; then
    missing+=("$plugin")
  fi
done

if (( ${#missing[@]} )); then
  echo "[err] missing GStreamer plugins after install: ${missing[*]}" >&2
  exit 1
fi

echo "[ok] H.264 camera video plugins are installed."
echo "[ok] H.264 plugins are available for a future ROS/web adapter encoder path."
