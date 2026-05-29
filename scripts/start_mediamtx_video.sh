#!/usr/bin/env bash
# Start a local MediaMTX gateway for robot camera video:
#   Pi publishes RTSP/H.264 to :8554/<path>
#   Browser reads WebRTC from :8889/<path>

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${MEDIAMTX_ENABLE:=1}"
: "${MEDIAMTX_VERSION:=v1.18.2}"
: "${MEDIAMTX_STATE_DIR:=$ROOT/.state/mediamtx}"
: "${MEDIAMTX_RTSP_PORT:=8554}"
: "${MEDIAMTX_WEBRTC_PORT:=8889}"
: "${MEDIAMTX_PATH:=xlerobot_head}"
: "${MEDIAMTX_PATHS:=${MEDIAMTX_PATH},xlerobot_base,xlerobot_wrist_left,xlerobot_wrist_right}"
: "${MEDIAMTX_BIN:=}"

if [[ "$MEDIAMTX_ENABLE" == "0" || "$MEDIAMTX_ENABLE" == "false" ]]; then
  echo "[mediamtx] disabled"
  exit 0
fi

mkdir -p "$MEDIAMTX_STATE_DIR/bin" "$MEDIAMTX_STATE_DIR/logs"

detect_asset() {
  local machine
  machine="$(uname -m)"
  case "$machine" in
    x86_64|amd64) echo "linux_amd64" ;;
    aarch64|arm64) echo "linux_arm64" ;;
    armv7l|armhf) echo "linux_armv7" ;;
    *) echo "[err] unsupported machine architecture for MediaMTX: $machine" >&2; return 1 ;;
  esac
}

ensure_mediamtx() {
  if [[ -n "$MEDIAMTX_BIN" ]]; then
    if [[ ! -x "$MEDIAMTX_BIN" ]]; then
      echo "[err] MEDIAMTX_BIN is not executable: $MEDIAMTX_BIN" >&2
      return 1
    fi
    printf '%s\n' "$MEDIAMTX_BIN"
    return 0
  fi
  if command -v mediamtx >/dev/null 2>&1; then
    command -v mediamtx
    return 0
  fi

  local asset archive url bin
  asset="$(detect_asset)"
  bin="$MEDIAMTX_STATE_DIR/bin/mediamtx-${MEDIAMTX_VERSION}"
  if [[ -x "$bin" ]]; then
    printf '%s\n' "$bin"
    return 0
  fi

  archive="$MEDIAMTX_STATE_DIR/mediamtx_${MEDIAMTX_VERSION}_${asset}.tar.gz"
  url="https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/mediamtx_${MEDIAMTX_VERSION}_${asset}.tar.gz"
  echo "[mediamtx] downloading $url" >&2
  curl -fsSL "$url" -o "$archive" || return 1
  tar -xzf "$archive" -C "$MEDIAMTX_STATE_DIR/bin" mediamtx || return 1
  mv "$MEDIAMTX_STATE_DIR/bin/mediamtx" "$bin"
  chmod +x "$bin"
  printf '%s\n' "$bin"
}

write_config() {
  local config=$1
  python3 - "$config" <<'PY'
from pathlib import Path
import os
import sys

config = Path(sys.argv[1])
paths = [
    item.strip().strip("/")
    for item in os.environ.get(
        "MEDIAMTX_PATHS",
        os.environ.get("MEDIAMTX_PATH", "xlerobot_head"),
    ).split(",")
    if item.strip()
]
if not paths:
    paths = ["xlerobot_head"]
rtsp_port = os.environ.get("MEDIAMTX_RTSP_PORT", "8554")
webrtc_port = os.environ.get("MEDIAMTX_WEBRTC_PORT", "8889")
path_config = "".join(
    f"  {path}:\n    source: publisher\n    overridePublisher: yes\n"
    for path in paths
)
config.write_text(f"""logLevel: info

rtsp: yes
rtspAddress: :{rtsp_port}
rtspTransports: [tcp]

rtmp: no
hls: no
srt: no

webrtc: yes
webrtcAddress: :{webrtc_port}
webrtcIPsFromInterfaces: yes

api: no
metrics: no
pprof: no

paths:
{path_config}
""")
PY
}

if (echo > "/dev/tcp/127.0.0.1/${MEDIAMTX_RTSP_PORT}") >/dev/null 2>&1 \
   && (echo > "/dev/tcp/127.0.0.1/${MEDIAMTX_WEBRTC_PORT}") >/dev/null 2>&1; then
  echo "[mediamtx] existing listener detected on :${MEDIAMTX_RTSP_PORT}/:${MEDIAMTX_WEBRTC_PORT}; using existing"
  exit 0
fi

bin="$(ensure_mediamtx)"
config="$MEDIAMTX_STATE_DIR/mediamtx.yml"
write_config "$config"

echo "[mediamtx] paths       : ${MEDIAMTX_PATHS}"
echo "[mediamtx] RTSP ingest : rtsp://0.0.0.0:${MEDIAMTX_RTSP_PORT}/<path>"
echo "[mediamtx] WebRTC out  : http://0.0.0.0:${MEDIAMTX_WEBRTC_PORT}/<path>"
exec "$bin" "$config"
