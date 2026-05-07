# Isaac ZMQ Viewer, Navigation, and Teleop Usage

This stack is for the Isaac Sim ZMQ server only. It does not start ROS 2,
Gazebo, Nav2, Foxglove, or the old Gazebo map pipeline.

## Network Contract

Isaac Sim must already be running and exposing the `xlerobot_v1` ZMQ protocol.

| Port | Pattern | Direction | Purpose |
| --- | --- | --- | --- |
| `5555` | PUB/SUB | sim to client | sensor topics |
| `5556` | PUSH/PULL | client to sim | 14 arm targets + 3 base velocity commands |
| `5557` | REQ/REP | client to sim | RPC such as `enable_stream`, `disable_stream`, reset |

Default host used by the wrappers in this repo:

```bash
SIM_HOST=100.80.87.68
HTTP_PORT=18081
```

The HTTP viewer binds inside this environment. If viewing from a laptop through
SSH, forward a free laptop port to the container HTTP port:

```bash
ssh -fN -4 -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
  -L 127.0.0.1:28081:172.17.0.2:18081 root@ubuntu
```

Then open:

```text
http://127.0.0.1:28081/
```

Use a different laptop-side port if `28081` is already occupied.

## Viewer Only

Start or restart only the browser viewer:

```bash
python3 viewer.py
```

Equivalent alias:

```bash
python3 init_viewer.py
```

What it does:

- enables Isaac streams through RPC:
  - `proprio` at 10 Hz
  - `rgb.front` at 5 Hz
  - `depth.front` at 3 Hz
  - `scan` at 5 Hz
  - `scan.mid` disabled by default
- restarts `examples/web_viewer.py`
- resets local viewer state through `/reset_local`
- leaves navigation and teleop untouched

Useful checks:

```bash
curl http://127.0.0.1:18081/topics.json
curl http://127.0.0.1:18081/analysis.json | python3 -m json.tool
tail -f /tmp/isaac_web_viewer_18081.log
```

Viewer endpoints:

| Endpoint | Meaning |
| --- | --- |
| `/` | responsive HTML dashboard |
| `/topics.json` | currently received topics |
| `/analysis.json` | OCR, projection, grid, and pose status |
| `/rgb.front.ocr.mjpg` | front RGB with OCR overlay |
| `/depth.front.mjpg` | front depth colormap |
| `/local_map.mjpg` | lidar/depth-derived local grid map with trajectory and OCR annotations |
| `/reset_local` | clears viewer frames, local grid, trajectory, and OCR annotations |

## Initialize Navigation

Start the viewer and autonomous initialize-mode navigation from a clean local
map/OCR state:

```bash
python3 init_nav.py
```

What it does:

- resets the viewer local state
- stops stale teleop, nav, viewer, and old stack processes
- starts `examples/web_viewer.py`
- starts `examples/isaac_nav_client.py --mode initialize`
- enables streams at moderate rates
- sends velocity commands at 8 Hz
- uses `SPEED_SCALE=4.0` by default

Common overrides:

```bash
SIM_HOST=100.80.87.68 HTTP_PORT=18081 python3 init_nav.py
SPEED_SCALE=6.0 python3 init_nav.py
GRID_VIEW_M=28.0 python3 init_nav.py
```

The nav client uses lidar and front depth for reactive motion. The RGB-D camera
is treated as looking along robot forward when `CAMERA_YAW_OFFSET_RAD=0.0`.

## Stop Navigation

Stop only autonomous navigation and force a zero base command. The viewer can
stay running.

```bash
pkill -TERM -f '[p]ython3 examples/isaac_nav_client.py' || true
python3 - <<'PY'
import sys, time
sys.path.insert(0, "examples")
from _client_common import pack_command, push_socket

with push_socket("100.80.87.68", 5556) as sock:
    payload = pack_command([0.0] * 14, [0.0, 0.0, 0.0])
    for _ in range(20):
        sock.send(payload)
        time.sleep(0.05)
PY
```

This matters because Isaac keeps applying the last received action if a client
disconnects without sending zero velocity.

## Manual WASD Teleop

If the viewer is already running, start manual control:

```bash
python3 teleop.py
```

What it does:

- stops autonomous nav unless `--keep-nav` is passed
- stops stale teleop
- resets local viewer grid/OCR unless `--no-reset-local` is passed
- runs `examples/wasd_teleop.py` interactively

Keys:

| Key | Command |
| --- | --- |
| `w` / `s` | camera-forward / camera-back |
| `a` / `d` | camera-left / camera-right strafe |
| `q` / `e` | rotate left / rotate right |
| `space` or `r` | zero velocity |
| `x` or `Ctrl-C` | quit after sending zero velocity |

Speed defaults:

```text
vx=0.12 m/s, vy=0.12 m/s, wz=0.55 rad/s, speed_scale=4.0
```

Override examples:

```bash
python3 teleop.py --speed-scale 6
python3 teleop.py --vx 0.20 --vy 0.20 --wz 0.80
python3 teleop.py --keep-nav
```

To restart the viewer and then enter teleop in one command:

```bash
python3 init_manual.py
```

## One-Script Legacy Wrapper

`run_isaac_zmq_stack.sh` starts the viewer and optionally starts nav.

```bash
SIM_HOST=100.80.87.68 HTTP_PORT=18081 ./run_isaac_zmq_stack.sh
```

Viewer only:

```bash
RUN_NAV=false SIM_HOST=100.80.87.68 HTTP_PORT=18081 ./run_isaac_zmq_stack.sh
```

Goal mode:

```bash
MODE=goal GOAL_X=1.0 GOAL_Y=2.0 SIM_HOST=100.80.87.68 ./run_isaac_zmq_stack.sh
```

For day-to-day use, prefer `viewer.py`, `init_nav.py`, and `teleop.py` because
their defaults match the current Isaac setup.

## OCR and Grid Map Policy

The viewer creates a local 2D grid map and overlays OCR results on that map.

Pipeline:

1. subscribe to `rgb.front`, `depth.front`, `scan`, and `proprio`
2. run OCR asynchronously on `rgb.front`
3. use the OCR bounding-box center pixel
4. sample `depth.front` with a median valid 9x9 patch
5. project pixel + depth through a pinhole HFOV model into robot base XY
6. transform base XY into world XY using `joint_vel_arm_sample` or `base_pose`
7. add or update a map annotation on `/local_map.mjpg`

Default OCR backend:

```text
OCR_BACKEND=gazebo
```

The `gazebo` backend uses:

- PaddleOCR when available
- multi-scale Tesseract fallback
- dark sign ROI crops for tiny distant signs
- room-ID postprocessing with `FLOOR_HINT=5`
- `FLOOR_PRIOR_MODE=complete`, applied after OCR candidate generation

The floor prior is not injected into the OCR engine. It is applied only after
OCR returns candidate text. For the current 5th-floor map this means compatible
room IDs are kept or completed as 5th-floor IDs when the visible candidate is a
partial room number.

Important OCR defaults:

```bash
OCR_BACKEND=gazebo
FLOOR_HINT=5
FLOOR_PRIOR_MODE=complete
OCR_SCALES=1.0,2.0,3.0,4.0,6.0
OCR_MAX_SIDE=2400
OCR_MIN_CONFIDENCE=0.25
OCR_INTERVAL=2.0
```

Map annotation merge policy:

- annotations are merged by projected map distance, not by text equality
- if a new OCR observation is within `OCR_MERGE_RADIUS_M` of an existing map
  annotation, it updates the same physical sign
- the map position is confidence-weighted over repeated observations
- the displayed text, source, bbox, and depth are replaced only when the new
  observation has higher confidence

Default merge radius:

```text
OCR_MERGE_RADIUS_M=0.75
```

## Grid Map Display

The local map is a fixed-world grid centered at the initial observed pose. The
robot moves inside the map; the view does not chase the robot. The rendered
view auto-zooms out when the trajectory or annotations approach the viewport
edge.

Defaults:

```bash
GRID_RESOLUTION_M=0.10
GRID_SIZE_M=80.0
GRID_VIEW_M=18.0
```

The map image shows:

- occupied/free grid evidence from lidar scan
- current robot pose arrow
- trajectory since local reset
- OCR annotation points and labels
- current status text with pose source and annotation count

## Stream Rate Defaults

The wrappers intentionally avoid very high stream and command rates.

| Setting | Default |
| --- | --- |
| `PROPRIO_RATE_HZ` | `10.0` |
| `RGB_RATE_HZ` | `5.0` |
| `DEPTH_RATE_HZ` | `3.0` |
| `SCAN_RATE_HZ` | `5.0` |
| `SCAN_MID_RATE_HZ` | `0.0` |
| `CMD_RATE_HZ` / `NAV_CMD_RATE_HZ` / `TELEOP_CMD_RATE_HZ` | `8.0` |

## Dependency Notes

Hard dependencies for the viewer:

```text
pyzmq, msgpack
```

For full RGB-D/grid/OCR behavior:

```text
numpy, opencv-python, zstandard, pillow, paddleocr, pytesseract
```

If OpenCV or zstandard is missing, RGB can still display, but depth/grid/OCR
features degrade.
