# `examples/` 분석 — Isaac Sim 서버와 통신하는 방법

`indoory_isaac_sim`은 Isaac Sim을 **항상 켜져 있는 서버 프로세스**로 띄워놓고, 로봇 측 클라이언트가 ZMQ로 붙었다 떨어지는 구조다. `examples/` 안의 클라이언트들이 그 "붙는 쪽"의 최소 레퍼런스 구현이다. 이 문서는 그 클라이언트들을 분해해서, **명령을 어떻게 보내고 / 센서 데이터를 어떻게 받는지** 정리한다.

기본 와이어 포맷은 [src/indoory_isaac_sim/wire/schema.py](../src/indoory_isaac_sim/wire/schema.py)에 정의된 `xlerobot_v1` 스키마이며, 모든 메시지는 msgpack으로 직렬화된다.

---

## 1. 통신 토폴로지 (3개의 ZMQ 채널)

| 포트 | 패턴 | 방향 | 용도 |
|------|------|-----|------|
| 5555 | PUB → SUB | sim → robot | 센서 토픽 (multipart `[topic, msgpack]`); 토픽은 `.<robot_id>` 접미사 |
| 5556 | PULL ← PUSH | robot → sim | 액션 명령 프레임 (관절 위치 + 베이스 속도 + `robot_id`) |
| 5557 | REP ↔ REQ | robot ↔ sim | RPC (reset, set_pose, enable_stream, fleet_info …) |

세 채널 모두 [src/indoory_isaac_sim/transport/zmq_bus.py](../src/indoory_isaac_sim/transport/zmq_bus.py)에서 sim 측이 `bind`, 클라이언트는 [examples/_client_common.py](../examples/_client_common.py)에서 `connect` 한다. **sim은 절대 ZMQ에서 블로킹하지 않는다** — PUB은 `NOBLOCK + HWM 16`이라 느린 구독자가 있으면 그쪽 프레임만 drop, PULL은 매 틱 [`recv_all()`](../src/indoory_isaac_sim/transport/zmq_bus.py)로 큐를 통째로 비우고 sim이 `robot_id`별로 마지막 메시지만 살림(per-id last-writer-wins), REP는 `poll(timeout=0)`으로 비동기 폴링한다.

### 1.1 멀티 로봇 fleet — 같은 hospital, N대 독립 제어

Sim_server는 `--num-robots N` 인자로 부팅 시 fleet 크기를 결정한다(기본 3, 프로토콜 상한 16). N대가 같은 hospital 안에 `/World/envs/env_0/Robot_0`, `Robot_1`, … 으로 spawn되며, **각 로봇은 자기만의 토픽 묶음과 명령 슬롯을 갖는다**:

| 자원 | 분기 방식 |
|------|----------|
| 센서 토픽 | `<base>.<robot_id>` (예: `proprio.0`, `scan.1`, `rgb.front.2`) |
| 명령 PULL | 단일 포트 5556을 모든 운전자가 공유. 메시지 안의 `robot_id`로 라우팅 |
| 행동 텐서 | 51-D `(num_envs=1, N*17)`. 로봇 i 슬롯은 `[i*17 : (i+1)*17]` |
| RPC `set_pose` | `robot_id` 인자로 어떤 로봇을 텔레포트할지 지정 |

운전자 3명이 서로 다른 `--robot-id` 값으로 동시에 PUSH해도 충돌 없음 — sim은 `robot_id`별 큐를 따로 관리한다. **운전자 같은 id를 두 명이 잡으면** 그 id 안에서만 last-writer-wins이 일어나서 둘이 싸운다(이건 클라이언트 약속 영역).

### 공용 헬퍼 — [examples/_client_common.py](../examples/_client_common.py)

세 가지 컨텍스트 매니저가 클라이언트 코드의 진입점이다.

| 헬퍼 | 소켓 타입 | 어디에 쓰는지 |
|------|----------|---------------|
| `sub_socket(host, port, topics)` | `zmq.SUB` + `RCVHWM=8` | 센서 구독 (5555) |
| `push_socket(host, port)` | `zmq.PUSH` + `SNDHWM=4` | 명령 송신 (5556) |
| `req_socket(host, port, timeout_ms)` | `zmq.REQ` + RCV/SND timeout | RPC (5557) |

핵심 직렬화 헬퍼:

- `pack_command(arm_pos, base_vel, *, robot_id=0)` — 14-D 관절 위치 + 3-D 베이스 속도를 positional로 받아 검증해서 msgpack으로 묶는다. 길이가 다르면 `ValueError`. `robot_id`를 빼면 0으로 들어가므로 단일 로봇 시절 코드도 그대로 동작(다 robot 0으로 향함). **`frame` 파라미터는 노출하지 않음** — 이 헬퍼로 보낸 명령은 메시지에 `frame` 키가 없고, sim 측 `validate_command`가 디폴트인 `"body"`로 처리한다. `frame="world"`를 쓰고 싶으면 [wire/schema.py의 `pack_command`](../src/indoory_isaac_sim/wire/schema.py#L112-L160)를 직접 import하거나, msgpack dict를 손으로 만들어 PUSH해야 한다(아래 4번 항목 참조).
- `rpc(sock, op, **kwargs)` — `{"schema": "xlerobot_v1", "op": op, ...}`를 보내고 응답을 unpack 해서 dict로 돌려준다.

```python
# 모든 명령 프레임의 본체 (per-robot) — 와이어 레벨 정의
{
    "schema": "xlerobot_v1",
    "stamp_ns": <int>,
    "robot_id": <int>,                        # 0..num_robots-1 (생략 시 0)
    "frame": "world" | "body",                # 생략 시 "body" (디폴트). _client_common 헬퍼는 이 키를 안 채움
    "arm_joint_pos_target": [f0, ..., f13],   # 길이 14
    "base_cmd_vel":         [vx, vy, wz],     # 길이 3
}
```

---

## 2. 명령(Action) 보내기 — PUSH → PULL :5556

### 2.1 액션 벡터 레이아웃

`xlerobot_v1`의 **단일 명령 프레임**은 한 로봇 분량 17차원이다. 와이어상에서는 두 키로 나뉘지만 sim 쪽 [wire/command_decode.py](../src/indoory_isaac_sim/wire/command_decode.py)의 `apply_command_to_action_tensor`가 `[arm_joint_pos_target(14) ‖ base_cmd_vel(3)]`로 concat한 뒤, **fleet 행동 텐서의 `robot_id*17` 오프셋 슬롯에만 in-place로 기입**한다. 다른 로봇 슬롯은 보존되므로, 운전자 N명이 매 틱 자기 17-D만 보내도 sim은 51-D(또는 N×17) 행동 벡터를 정확히 유지한다.

검증 두 단계:

1. **Wire(스키마) 검증** — `0 ≤ robot_id < MAX_NUM_ROBOTS=16`. 16 이상은 `pack_command`/`validate_command`에서 즉시 거부.
2. **런타임 fleet 검증** — `apply_command_to_action_tensor`가 행동 텐서의 마지막 축에서 fleet 크기를 읽어 `robot_id ≥ fleet`이면 `SchemaError`. 즉 `--num-robots 3`으로 띄운 sim에 `robot_id=5` 메시지를 보내면 wire는 통과하지만 적용 단계에서 드롭하고 경고 로그가 찍힘.

14-D 관절 위치 순서는 [schema.JOINT_POS_ORDER](../src/indoory_isaac_sim/wire/schema.py#L39-L45):

| idx | 0–4 | 5 | 6–10 | 11 | 12–13 |
|-----|-----|---|------|----|-------|
| 의미 | 오른팔 5축 (`Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`) | `Jaw` | 왼팔 5축 (`*_2`) | `Jaw_2` | `head_pan_joint`, `head_tilt_joint` |

3-D 베이스 속도는 `(root_x_axis_joint, root_y_axis_joint, root_z_rotation_joint)` 순서. 즉 `[vx, vy, wz]` (선속도 m/s, 각속도 rad/s).

이 순서가 어긋나면 다른 관절이 움직인다 — `scripted_arm_client`의 코멘트가 강조하는 바.

#### `base_cmd_vel`의 좌표계 — `frame` 필드

URDF의 mock mobile-base는 `root → X축 prismatic → Y축 prismatic → Z축 회전 → base_link` 순으로 직렬 연결돼 있어, 두 prismatic 조인트의 축은 물리적으로는 **월드 X/Y**다. 그렇지만 와이어 디폴트는 `frame="body"` — sim이 명령 적용 직전에 해당 로봇의 현재 root quaternion(`base_link`의 world orientation)을 읽어 `[vx, vy]`만 yaw로 회전해 텐서에 쓴다. 즉 **field를 생략하거나 `frame="body"`를 보내면** `base_cmd_vel = [1, 0, 0]`이 항상 "**현재 로봇 정면**으로 1 m/s"가 된다. yaw rate `wz`는 두 프레임에서 동일(Z축 공유).

| `frame` | `base_cmd_vel`의 의미 | 언제 |
|--------|----------------------|------|
| `"body"` (default, 생략 시) | `[forward(+X 바디) 속도, left(+Y 바디) 속도, yaw rate]` | 텔레옵/RL/단순 행동 정책 — yaw 변환을 클라가 안 해도 됨 |
| `"world"` | `[월드 X 속도, 월드 Y 속도, yaw rate]` | 클라가 이미 월드 프레임 odometry/플래너를 갖고 있을 때, sim의 회전을 우회하고 싶은 경우 |

```python
# 디폴트: body — 그냥 "정면으로 0.5 m/s"
# _client_common.pack_command(arm_pos, base_vel, *, robot_id=0)
push.send(pack_command([0.0]*14, [0.5, 0.0, 0.0], robot_id=2))

# 명시적 world: 월드 +X로 0.5 m/s (로봇이 어떤 yaw든 무관)
# _client_common 헬퍼는 frame 키를 채우지 않으므로,
# 와이어 dict를 직접 만들어 보내거나 schema.pack_command를 쓴다.
import time, msgpack
push.send(msgpack.packb({
    "schema": "xlerobot_v1",
    "stamp_ns": time.monotonic_ns(),
    "robot_id": 2,
    "frame": "world",
    "arm_joint_pos_target": [0.0]*14,
    "base_cmd_vel": [0.5, 0.0, 0.0],
}, use_bin_type=True))
```

검증/적용 위치: [wire/schema.py](../src/indoory_isaac_sim/wire/schema.py) `pack_command`/`validate_command`가 `frame ∈ {"world","body"}`만 허용 (디폴트 `"body"`), 회전은 [wire/command_decode.py](../src/indoory_isaac_sim/wire/command_decode.py)의 `apply_command_to_action_tensor`에서 `env.scene[f"robot_{rid}"].data.root_state_w` quat을 읽어 수행. publisher의 `proprio.<i>`가 발행하는 `base_pose`/`base_twist`는 frame 옵션과 무관하게 **항상 월드 프레임**이다(센서 페이로드 정의는 불변).

### 2.2 패턴 A — 키보드 텔레옵 ([keyboard_client.py](../examples/keyboard_client.py))

**무엇을 보여주나**: 쌍방향 계약(센서 구독 + 명령 송신)이 동시에 동작하는 가장 작은 데모. `--robot-id N`으로 어느 로봇을 운전할지 고른다(기본 0).

흐름:
1. 별도 데몬 스레드에서 [`_proprio_loop`](../examples/keyboard_client.py#L44-L73)가 자기 로봇의 `proprio.<robot_id>` 토픽만 SUB 한다 (1초에 한 번씩 베이스 위치/yaw를 stdout에 출력만 함).
2. 메인 스레드는 `tty.setcbreak`로 raw 키 입력을 받아 `KEY_VEL` 매핑(`w/s/a/d` → 선형, `q/e` → 회전, `space` → halt)으로 `base = [vx, vy, wz]`를 갱신한다.
3. **매 틱(20 Hz)** `pack_command(arm=[0]*14, base=base, robot_id=args.robot_id)`을 PUSH한다. 키가 안 눌려도 보낸다 — sim이 *해당 robot_id 슬롯의* `last_action`을 신선하게 유지해야 발산이 없다.

설계 포인트:
- 상수 14차원의 `arm` 벡터는 0으로 고정. 즉 팔/머리 자세는 home 자세로 implicit PD가 잡고, 베이스만 명령으로 움직인다.
- 명령 송신 주기는 키 입력 주기와 별개로 `--rate-hz`(default 20 Hz)에서 결정. 이건 **명령 stale을 방지**하는 가장 단순한 방법.
- 두 명이 동시에 운전하려면 다른 터미널에서 `--robot-id 1`로 띄우면 된다 — 토픽 prefix 매칭과 `robot_id`별 큐 덕분에 자연 격리.

### 2.3 패턴 B — 캔드 모션(스크립트 궤적) ([scripted_arm_client.py](../examples/scripted_arm_client.py))

**무엇을 보여주나**: 와이어 인덱스 ↔ 관절 이름 일치성을 눈으로 검증하는 스모크 테스트. 역시 `--robot-id N`로 대상 로봇 지정.

흐름:
1. `JOINT_POS_ORDER`를 클라이언트가 직접 들고 있다 ([scripted_arm_client.py:20-26](../examples/scripted_arm_client.py#L20-L26)). schema와 동일하게 정의해서 `PITCH_IDX`를 룩업.
2. 30 Hz 루프에서 `arm[PITCH_IDX] = amp · sin(2π f t)`만 갱신, 나머지는 모두 0. base도 0. 매번 `pack_command(..., robot_id=args.robot_id)`로 보냄.
3. 종료 시 `[0]*14`로 한 번 더 보내서 정지 자세를 보장.

이 클라이언트가 통과한다는 것 = **sim 쪽 해당 로봇의 액션 텀 순서와 와이어 순서가 일치**한다는 강력한 증거(잘못 매핑되어 있으면 다른 관절이 흔들리거나, 다른 로봇이 흔들림).

### 2.4 sim 쪽에서 어떻게 받아 적용하나

[sim_server.py](../src/indoory_isaac_sim/sim_server.py) 메인 루프 한 틱:

```
1. bus.poll_request()                 ← 5557 RPC 비동기 폴
2. raws = bus.recv_all()              ← 5556 큐 통째로 비움 (per-id last 살리려면 전부 봐야)
3. for raw in raws:                   ← robot_id별로 latest_by_robot[rid] = raw
4. for rid, raw in latest_by_robot:   ← 각 id의 가장 마지막 메시지만
        apply_command_to_action_tensor(unpack_command(raw),
                                       last_action, env)  # last_action[0, rid*17:(rid+1)*17] 만 덮어씀
5. env.step(last_action)              ← physics + RTX 카메라 자동 렌더 (last_action shape: (1, N*17))
6. publisher.publish_after_step()     ← 모든 토픽 (각각 .<id> 접미사) publish
7. heartbeat / rate.sleep()
```

명령이 한 틱 동안 한 로봇에서 안 오면 그 로봇의 슬롯이 **이전 명령**으로 유지된다(zero가 아님). 즉 클라이언트가 잠깐 끊겨도 그 로봇의 베이스가 마지막 cmd_vel로 계속 가는 점에 주의 — 다른 로봇은 영향 없음.

---

## 3. 센서 받기 — PUB → SUB :5555

### 3.1 멀티파트 토픽 컨벤션

모든 센서 메시지는 ZMQ multipart 두 프레임이다:
```
[ topic_bytes, msgpack_payload_bytes ]
```
SUB 측은 `setsockopt(SUBSCRIBE, prefix)`로 prefix 매칭한다.

**Fleet 토픽 분기**: 모든 토픽 이름은 `<base>.<robot_id>` 형태로 끝난다(예: `proprio.0`, `scan.1`, `rgb.front.2`). prefix 매칭 규칙 덕에:
- `""` → 모든 로봇의 모든 토픽
- `"depth"` → 모든 로봇의 depth 전부 (`depth.front.0`, `depth.front.1`, …, `depth.wrist.2`)
- `"depth.front"` → 모든 로봇의 front depth만
- `"proprio.1"` → 로봇 1의 proprio만 — 한 로봇 운전자가 자기 토픽만 받고 싶을 때

현재 활성 토픽은 [config/sensors.yaml](../config/sensors.yaml) 1회 선언 + 빌드 시 [config/multi_robot.replicate_decls_per_robot](../src/indoory_isaac_sim/config/multi_robot.py)가 N배 복제하여 결정된다. 디폴트 프로파일 + `--num-robots 3` 기준:

| 토픽 (각 i ∈ {0,1,2}) | 페이로드 | 기본 rate | 비고 |
|------------------------|---------|----------|-----|
| `proprio.<i>` | 관절 + 베이스 pose/twist + forward + `robot_id` (msgpack dict) | 30 Hz | 항상 켜짐, 가장 가벼움 |
| `tf.links.<i>` | 주요 링크의 base_link 기준 SE3 묶음 | 30 Hz | 양손 gripper / wrist / 머리 카메라 |
| `rgb.front.<i>` | JPEG 인코딩 RGB 1280×720 | 10 Hz | 헤드 카메라 (`jpeg_quality=75`) |
| `depth.front.<i>` | zstd 압축 depth uint16 1280×720 | 5 Hz | mm 단위 (`depth_scale_m=0.001`, `zstd_level=3`) |
| `rgb.wrist.<i>` | JPEG 320×240 | 10 Hz | 그리퍼 측 (`jpeg_quality=70`) |
| `depth.wrist.<i>` | — | (off) | 디폴트는 비활성 |
| `scan.<i>` | 2D 라이다 ranges (PhysX LiDAR, RPLIDAR C1 spec) | 10 Hz | base_link 기준 z=0.10 |
| `scan.mid.<i>` | 2D 라이다 ranges (두 번째 LiDAR) | 10 Hz | base_link 기준 z=0.40 |

총 토픽 수 = (활성 토픽/로봇) × `num_robots`. 활성 목록은 RPC `topic_list`로 언제든 조회 가능, fleet 크기는 `fleet_info`로 확인.

### 3.2 페이로드 헤더(공통)

[wire/schema.py:make_sensor_header](../src/indoory_isaac_sim/wire/schema.py#L72-L74)가 모든 페이로드 dict의 공통 머리:
```python
{ "schema": "xlerobot_v1", "stamp_ns": <int>, "frame_id": <int>, ... }
```

### 3.3 패턴 A — 토폴로지/대역폭 검사 ([noop_client.py](../examples/noop_client.py))

**무엇을 보여주나**: 명령은 안 보내고 SUB만 한다. **클라이언트는 자유롭게 붙었다 떨어져도 sim에 영향 없음**을 증명하는 데모.

핵심 줄:
```python
with sub_socket(args.host, args.pub_port, args.topics) as sock:
    while True:
        topic_b, payload = sock.recv_multipart()
        counters[topic_b.decode()] += 1
        bytes_seen[topic_b.decode()] += len(payload) + len(topic_b)
        # 주기적으로 토픽별 Hz / KB/s 출력
        sample = msgpack.unpackb(payload, raw=False)  # 디코드도 시연
```

`--topics ""` 가 디폴트라 모든 토픽을 받는다. 토픽별 페이로드의 키 셋(`"data"` 제외 sorted) 도 함께 찍어서 스키마가 맞는지 운영 시 빠르게 확인 가능.

### 3.4 패턴 B — 특정 로봇 / 특정 토픽만 SUB ([keyboard_client.py:_proprio_loop](../examples/keyboard_client.py#L44-L73))

`sub_socket(host, port, [f"proprio.{robot_id}"])` — 자기 로봇의 `proprio.<id>`만 구독해서 다른 로봇 / 카메라 / 라이다는 안 받는다. 데몬 스레드에서 `recv_multipart()`로 받아 `bp = msg["base_pose"]`, `bvel = msg["base_joint_vel"]` 등을 출력. 메시지 안에 `robot_id` 필드가 박혀 있어 prefix 없이 받았을 때도 어느 로봇 건지 식별 가능.

`proprio.<i>` payload 구조 ([sensors/builtins/proprio.py](../src/indoory_isaac_sim/sensors/builtins/proprio.py)):
```python
{
    "schema": "xlerobot_v1",
    "stamp_ns": <ns>, "topic": f"proprio.{i}", "frame": f"proprio_{i}",
    "robot_id": <i>,                      # 어느 로봇인지 (토픽 suffix와 동일)
    "joint_names_pos": [...14...],
    "joint_pos":       [...14...],
    "joint_vel_arm_sample": [...3...],     # 베이스 mock joint pos (x,y,θ)
    "joint_names_base": ["root_x_axis_joint","root_y_axis_joint","root_z_rotation_joint"],
    "base_joint_vel":  [...3...],
    "base_pose":  [x, y, z, qx, qy, qz, qw],  # 와이어는 xyzw (IsaacLab 내부의 wxyz와 다름)
    "base_twist": [vx, vy, vz, wx, wy, wz],
    "base_forward_w": [fx, fy, fz],          # base_link의 +X 축을 월드 좌표계로 (단위 벡터)
}

`tf.links.<i>` 페이로드 구조 ([sensors/builtins/frame_transform.py](../src/indoory_isaac_sim/sensors/builtins/frame_transform.py)) — `source`와 각 target의 prim 경로가 robot_i 별로 자동 재작성된다(`Robot_<i>` 접미):
```python
{
    "schema": "xlerobot_v1",
    "stamp_ns": <ns>, "topic": f"tf.links.{i}", "frame": f"tf_links_{i}",
    "source": f"{{ENV_REGEX_NS}}/Robot_{i}/base_link",
    "targets": [
        {"name": "gripper_right", "pose": [x,y,z, qx,qy,qz,qw]},   # 오른팔 Fixed_Jaw
        {"name": "gripper_left",  "pose": [...]},                  # 왼팔 Fixed_Jaw_2
        {"name": "jaw_right",     "pose": [...]},                  # Moving_Jaw
        {"name": "jaw_left",      "pose": [...]},                  # Moving_Jaw_2
        {"name": "wrist_right",   "pose": [...]},                  # Wrist_Pitch_Roll
        {"name": "wrist_left",    "pose": [...]},                  # Wrist_Pitch_Roll_2
        {"name": "head_pan",      "pose": [...]},
        {"name": "head_tilt",     "pose": [...]},
    ],
}
```

### 3.5 sim 쪽에서 누가 publish 하나

[transport/publisher.py:StreamPublisher](../src/indoory_isaac_sim/transport/publisher.py)가 `(센서, 토픽)` 별로 row를 들고 있다가, `sim_step_hz / target_rate_hz`로 **modulo**를 만들어 매 틱 그 modulo에 맞는 row만 `binding.encode(...)` → `bus.publish(topic, payload)`. 즉 timing이 **시뮬 시간 기준**으로 락스텝이라, SLAM/데이터셋 레코더 같은 물리-인지 컨슈머가 안전하게 사용할 수 있다.

---

## 4. RPC — REQ ↔ REP :5557

### 4.1 [rpc_client.py](../examples/rpc_client.py) CLI

`req_socket` 컨텍스트 안에서 `rpc(sock, op, **kwargs)`를 한 번 호출하면 끝. 응답은 `{"ok": bool, "error": str|None, ...}`.

지원되는 op (sim 측 핸들러 [sim_server.py](../src/indoory_isaac_sim/sim_server.py)):

| op | 인자 | 효과 |
|----|------|-----|
| `reset` | — | `env.reset()` 호출, 모든 로봇 슬롯의 `last_action`을 0으로 |
| `joint_names` | — | `joint_pos_order` (14), `joint_vel_order` (3) 리스트 반환 (per-robot order) |
| `topic_list` | — | 현재 활성 publish 토픽 목록 (`.<id>` 접미 포함) |
| `fleet_info` | — | `{num_robots: int}` — sim_server `--num-robots` 값 |
| `enable_stream` | `topic`, `rate_hz?` | 해당 토픽 활성화 (rate도 함께 변경 가능). 토픽 이름은 반드시 `.<id>` 접미 |
| `disable_stream` | `topic` | 토픽 비활성 |
| `set_stream_rate` | `topic`, `rate_hz` | 런타임 rate 변경 (modulo 재계산) |
| `set_stream_param` | `topic`, `key=value` 다수 | 인코더 파라미터(예: `jpeg_quality=85`) 조정 |
| `set_pose` | `pose=[x,y,z,qx,qy,qz,qw]`, `robot_id` | 지정한 로봇을 월드 좌표로 텔레포트. 와이어는 xyzw, sim 내부에서 wxyz로 재배열 |
| `set_mode` | `mode` | 프로파일 변경은 **sim 재시작 필요** — 에러 응답으로 알려줌 |
| `shutdown` | — | sim 종료 |

CLI 예시:
```bash
python examples/rpc_client.py topic_list
python examples/rpc_client.py fleet_info
python examples/rpc_client.py reset
python examples/rpc_client.py disable_stream depth.front.0     # 로봇 0의 front depth만 끔
python examples/rpc_client.py enable_stream  depth.wrist.1 5   # 로봇 1의 wrist depth만 켬
python examples/rpc_client.py set_stream_param rgb.front.2 jpeg_quality=85
python examples/rpc_client.py set_pose 0 0 0.05 0 0 0 1        # 항상 robot 0으로 텔레포트
```

> ⚠️ 현재 [`rpc_client.py`](../examples/rpc_client.py)의 `set_pose` 분기는 정확히 7개의 float 인자만 받고 `robot_id=N` kwargs를 파싱하지 않는다 — 즉 CLI에서는 항상 robot 0이 텔레포트된다. 다른 로봇을 옮기려면 와이어 프로토콜 자체는 `robot_id`를 지원하므로([sim_server.py의 RPC 핸들러](../src/indoory_isaac_sim/sim_server.py#L272-L283)) `req_socket` + `rpc(sock, "set_pose", pose=[...], robot_id=N)`을 직접 호출하는 짧은 스크립트를 쓰면 된다.

### 4.2 RPC와 PULL의 차이

- **PULL :5556** — 매 틱마다 보내는 **연속 액션** 채널. 액션이 안 와도 sim은 멈추지 않고 마지막 액션을 계속 적용.
- **REP :5557** — **이산적인 사이드 이펙트**(reset / 텔레포트 / 스트림 토글)용. 동기 응답이 필요하므로 REQ-REP. `rep_socket`은 timeout이 짧게 걸려 있어, sim이 죽었으면 클라이언트가 `zmq.Again`으로 알 수 있다.

---

## 5. 한눈에 보는 클라이언트별 역할

| 파일 | PUB 구독 | PULL 송신 | REP 호출 | 한 줄 요약 |
|------|---------|----------|---------|------------|
| [_client_common.py](../examples/_client_common.py) | (헬퍼) | (헬퍼) | (헬퍼) | 소켓 컨텍스트 + `pack_command(robot_id=…)` / `rpc` |
| [noop_client.py](../examples/noop_client.py) | ✅ 모든 토픽 (또는 prefix) | — | — | 토픽 Hz/대역폭 측정. 명령 안 보냄 |
| [keyboard_client.py](../examples/keyboard_client.py) | `proprio.<id>` | ✅ 매 틱 (자기 로봇) | — | WASD 텔레옵, `--robot-id N`으로 대상 선택 |
| [scripted_arm_client.py](../examples/scripted_arm_client.py) | — | ✅ 30 Hz sin (자기 로봇) | — | `Pitch`만 흔들어 액션 매핑 검증, `--robot-id N` |
| [rpc_client.py](../examples/rpc_client.py) | — | — | ✅ 단발 | RPC CLI 래퍼 (`fleet_info`, `set_pose ... robot_id=2` 등) |
| [web_viewer.py](../examples/web_viewer.py) | ✅ 모든 토픽 | — | — | 브라우저용 HTTP 브리지 (MJPEG/JSON), `--num-robots N`으로 그리드 행 수 결정 |

---

## 6. 새 클라이언트를 만들 때 체크리스트

1. **PUSH(:5556)로 보낼 액션은 항상 17차원으로 전부 채워야 한다** — `arm` 14개와 `base` 3개. 사용 안 하는 관절은 0(혹은 home pose). [`pack_command`](../examples/_client_common.py#L59-L78)가 길이를 강제 검증한다.
2. **`robot_id`를 명시하라** — `pack_command(arm, base, robot_id=N)` 또는 와이어에 `"robot_id": N`. 빠지면 0으로 들어가서 의도치 않게 robot 0을 운전한다. fleet 크기 모르면 RPC `fleet_info`로 먼저 확인.
3. **명령은 매 틱 보내라** — 명령이 끊기면 그 로봇의 `last_action` 슬롯이 stale 상태로 계속 적용된다 (특히 `cmd_vel`이 0이 아닐 때 위험). 다른 로봇 슬롯은 영향 없음. 정지하려면 명시적으로 `[0, 0, 0]`을 보낼 것.
4. **인덱스로 관절을 가리키지 말고 이름으로 lookup** — [`schema.JOINT_POS_ORDER`](../src/indoory_isaac_sim/wire/schema.py)에서 `index("Pitch")`처럼. 와이어 순서가 v2에서 바뀔 수 있다.
5. **SUB는 prefix 매칭**이라 `""`로 전체, `"proprio.1"`로 로봇 1 proprio만, `"depth"`로 모든 로봇 depth 등 자유롭게 자른다. 받자마자 항상 `recv_multipart()` 두 프레임을 같이 처리.
6. **schema 필드는 반드시 검사** — 아직 v1만 정의됐지만, `xlerobot_v1`이 아닌 메시지가 오면 **버려라**. sim 쪽 [validate_command](../src/indoory_isaac_sim/wire/schema.py)도 그렇게 동작한다. `robot_id` 검증은 두 단계(wire `MAX_NUM_ROBOTS=16`, runtime fleet bound) — `MAX_NUM_ROBOTS` 이상은 wire에서, fleet 크기 이상은 적용 단계에서 거부된다.
7. **base_pose의 quat는 xyzw**(와이어). IsaacLab 내부는 wxyz라 sim 쪽 `_set_robot_pose`가 변환 처리. 클라이언트는 xyzw로 통일.
8. **`frame` 필드를 의식하라** — 디폴트(생략 또는 `"body"`)는 `base_cmd_vel`이 **로봇 바디 프레임**의 [forward, left, yaw_rate]다. 즉 같은 명령을 stale하게 매 틱 보내고 있어도 sim은 매번 **현재 yaw**로 다시 회전해서 적용한다 (회전 중이면 궤적이 휜다는 뜻). 월드 프레임 odometry/플래너 위에서 명령을 쏘려면 `frame="world"`로 명시. RPC `set_pose`는 **항상 월드 프레임** (body-frame 텔레포트는 지원 안 함).
9. RPC `set_mode`처럼 **resolution / 토포로지가 바뀌는 변경은 sim 재시작 필수** — `--profile`/`--num-robots`는 부팅 시 결정된다.

---

## 7. 의존성

클라이언트 측은 의도적으로 가볍게 짜여 있다:

- 필수: `pyzmq`, `msgpack`
- IsaacLab / torch / leisaac 같은 sim-side 의존성은 **전혀 없다** (이게 설계 목표 — 로봇 서버는 sim-agnostic해야 한다).

따라서 어떤 conda env에서도, 별도 컨테이너에서도 클라이언트를 돌릴 수 있다. sim 서버만 `leisaac` env에서 [run_sim.sh](../run_sim.sh) 또는 `python -m indoory_isaac_sim.sim_server [—num-robots N]`로 띄우면 된다.
