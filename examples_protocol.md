# `examples/` 분석 — Isaac Sim 서버와 통신하는 방법

`indoory_isaac_sim`은 Isaac Sim을 **항상 켜져 있는 서버 프로세스**로 띄워놓고, 로봇 측 클라이언트가 ZMQ로 붙었다 떨어지는 구조다. `examples/` 안의 클라이언트들이 그 "붙는 쪽"의 최소 레퍼런스 구현이다. 이 문서는 그 클라이언트들을 분해해서, **명령을 어떻게 보내고 / 센서 데이터를 어떻게 받는지** 정리한다.

기본 와이어 포맷은 [src/indoory_isaac_sim/wire/schema.py](../src/indoory_isaac_sim/wire/schema.py)에 정의된 `xlerobot_v1` 스키마이며, 모든 메시지는 msgpack으로 직렬화된다.

---

## 1. 통신 토폴로지 (3개의 ZMQ 채널)

| 포트 | 패턴 | 방향 | 용도 |
|------|------|-----|------|
| 5555 | PUB → SUB | sim → robot | 센서 토픽 (multipart `[topic, msgpack]`) |
| 5556 | PULL ← PUSH | robot → sim | 액션 명령 프레임 (관절 위치 + 베이스 속도) |
| 5557 | REP ↔ REQ | robot ↔ sim | RPC (reset, set_pose, enable_stream …) |

세 채널 모두 [src/indoory_isaac_sim/transport/zmq_bus.py](../src/indoory_isaac_sim/transport/zmq_bus.py)에서 sim 측이 `bind`, 클라이언트는 [examples/_client_common.py](../examples/_client_common.py)에서 `connect` 한다. **sim은 절대 ZMQ에서 블로킹하지 않는다** — PUB은 `NOBLOCK + HWM 16`이라 느린 구독자가 있으면 그쪽 프레임만 drop, PULL은 `recv_latest()`로 큐를 비우고 가장 최신 메시지만 채택, REP는 `poll(timeout=0)`으로 비동기 폴링한다.

### 공용 헬퍼 — [examples/_client_common.py](../examples/_client_common.py)

세 가지 컨텍스트 매니저가 클라이언트 코드의 진입점이다.

| 헬퍼 | 소켓 타입 | 어디에 쓰는지 |
|------|----------|---------------|
| `sub_socket(host, port, topics)` | `zmq.SUB` + `RCVHWM=8` | 센서 구독 (5555) |
| `push_socket(host, port)` | `zmq.PUSH` + `SNDHWM=4` | 명령 송신 (5556) |
| `req_socket(host, port, timeout_ms)` | `zmq.REQ` + RCV/SND timeout | RPC (5557) |

핵심 직렬화 헬퍼:

- `pack_command(arm_pos, base_vel)` — 14-D 관절 위치 + 3-D 베이스 속도를 검증해서 msgpack으로 묶는다. 길이가 다르면 `ValueError`.
- `rpc(sock, op, **kwargs)` — `{"schema": "xlerobot_v1", "op": op, ...}`를 보내고 응답을 unpack 해서 dict로 돌려준다.

```python
# 모든 명령 프레임의 본체
{
    "schema": "xlerobot_v1",
    "stamp_ns": <int>,
    "arm_joint_pos_target": [f0, ..., f13],   # 길이 14
    "base_cmd_vel":         [vx, vy, wz],     # 길이 3
}
```

---

## 2. 명령(Action) 보내기 — PUSH → PULL :5556

### 2.1 액션 벡터 레이아웃

`xlerobot_v1`의 **단일 명령 프레임**은 17차원이다. 와이어상에서는 두 키로 나뉘지만 sim 쪽은 [wire/command_decode.py](../src/indoory_isaac_sim/wire/command_decode.py)에서 `[arm_joint_pos_target(14) ‖ base_cmd_vel(3)]`로 concat 해서 IsaacLab `env.step()`에 그대로 넘긴다.

14-D 관절 위치 순서는 [schema.JOINT_POS_ORDER](../src/indoory_isaac_sim/wire/schema.py#L39-L45):

| idx | 0–4 | 5 | 6–10 | 11 | 12–13 |
|-----|-----|---|------|----|-------|
| 의미 | 오른팔 5축 (`Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`) | `Jaw` | 왼팔 5축 (`*_2`) | `Jaw_2` | `head_pan_joint`, `head_tilt_joint` |

3-D 베이스 속도는 `(root_x_axis_joint, root_y_axis_joint, root_z_rotation_joint)` 순서. 즉 `[vx, vy, wz]` (선속도 m/s, 각속도 rad/s).

이 순서가 어긋나면 다른 관절이 움직인다 — `scripted_arm_client`의 코멘트가 강조하는 바.

### 2.2 패턴 A — 키보드 텔레옵 ([keyboard_client.py](../examples/keyboard_client.py))

**무엇을 보여주나**: 쌍방향 계약(센서 구독 + 명령 송신)이 동시에 동작하는 가장 작은 데모.

흐름:
1. 별도 데몬 스레드에서 [`_proprio_loop`](../examples/keyboard_client.py#L44-L73)가 `proprio` 토픽을 SUB 한다 (1초에 한 번씩 베이스 위치/yaw를 stdout에 출력만 함).
2. 메인 스레드는 `tty.setcbreak`로 raw 키 입력을 받아 `KEY_VEL` 매핑(`w/s/a/d` → 선형, `q/e` → 회전, `space` → halt)으로 `base = [vx, vy, wz]`를 갱신한다.
3. **매 틱(20 Hz)** `pack_command(arm=[0]*14, base=base)`을 PUSH한다. 키가 안 눌려도 보낸다 — sim이 `last_action`을 신선하게 유지해야 발산이 없다.

설계 포인트:
- 상수 14차원의 `arm` 벡터는 0으로 고정. 즉 팔/머리 자세는 home 자세로 implicit PD가 잡고, 베이스만 명령으로 움직인다.
- 명령 송신 주기는 키 입력 주기와 별개로 `--rate-hz`(default 20 Hz)에서 결정. 이건 **명령 stale을 방지**하는 가장 단순한 방법.

### 2.3 패턴 B — 캔드 모션(스크립트 궤적) ([scripted_arm_client.py](../examples/scripted_arm_client.py))

**무엇을 보여주나**: 와이어 인덱스 ↔ 관절 이름 일치성을 눈으로 검증하는 스모크 테스트.

흐름:
1. `JOINT_POS_ORDER`를 클라이언트가 직접 들고 있다 ([scripted_arm_client.py:20-26](../examples/scripted_arm_client.py#L20-L26)). schema와 동일하게 정의해서 `PITCH_IDX`를 룩업.
2. 30 Hz 루프에서 `arm[PITCH_IDX] = amp · sin(2π f t)`만 갱신, 나머지는 모두 0. base도 0.
3. 종료 시 `[0]*14`로 한 번 더 보내서 정지 자세를 보장.

이 클라이언트가 통과한다는 것 = **sim 쪽 액션 텀 순서와 와이어 순서가 일치**한다는 강력한 증거(잘못 매핑되어 있으면 다른 관절이 흔들림).

### 2.4 sim 쪽에서 어떻게 받아 적용하나

[sim_server.py:300-343](../src/indoory_isaac_sim/sim_server.py#L300-L343) 메인 루프 한 틱:

```
1. bus.poll_request()      ← 5557 RPC 비동기 폴
2. bus.recv_latest()       ← 5556에서 큐 비우고 최신 명령만 잡음
3. cmd → command_to_action_tensor() → last_action (Tensor[1, 17])
4. env.step(last_action)   ← physics + RTX 카메라 자동 렌더
5. publisher.publish_after_step()
6. heartbeat / rate.sleep()
```

명령이 한 틱 동안 안 오면 `last_action`이 그대로 유지된다 (zero가 아니라 **이전 명령**이 유지됨). 즉 클라이언트가 잠깐 끊겨도 베이스가 마지막 cmd_vel로 계속 가는 점에 주의.

---

## 3. 센서 받기 — PUB → SUB :5555

### 3.1 멀티파트 토픽 컨벤션

모든 센서 메시지는 ZMQ multipart 두 프레임이다:
```
[ topic_bytes, msgpack_payload_bytes ]
```
SUB 측은 `setsockopt(SUBSCRIBE, prefix)`로 prefix 매칭한다 — `""`은 모든 토픽 구독, `"depth"`는 `depth.front` / `depth.wrist` 모두 매칭.

현재 활성 토픽은 [config/sensors.yaml](../config/sensors.yaml) + 선택된 [config/streams.yaml](../config/streams.yaml) 프로파일에서 결정된다. 디폴트 프로파일에서:

| 토픽 | 페이로드 | 기본 rate | 비고 |
|------|---------|----------|-----|
| `proprio` | 관절 + 베이스 pose/twist (msgpack dict) | 30 Hz | 항상 켜짐, 가장 가벼움 |
| `rgb.front` | JPEG 인코딩 RGB 424×240 | 10 Hz | 헤드 카메라 |
| `depth.front` | zstd 압축 depth uint16 320×240 | 5 Hz | mm 단위 (`depth_scale_m=0.001`) |
| `rgb.wrist` | JPEG 320×240 | 10 Hz | 그리퍼 측 |
| `depth.wrist` | — | (off) | 디폴트는 비활성 |
| `scan`, `scan.mid` | 2D 라이다 ranges 360개 | 10 Hz | base_link 기준 z=0.10/0.40 |

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

### 3.4 패턴 B — 특정 토픽만 SUB ([keyboard_client.py:_proprio_loop](../examples/keyboard_client.py#L44-L73))

`sub_socket(host, port, ["proprio"])` — `proprio` prefix만 구독해서 카메라/라이다는 받지 않는다. 데몬 스레드에서 `recv_multipart()`로 받아 `bp = msg["base_pose"]`, `bvel = msg["base_joint_vel"]` 등을 출력.

`proprio` payload 구조는 [sensors/builtins/proprio.py:78-95](../src/indoory_isaac_sim/sensors/builtins/proprio.py#L78-L95)에서 만들어진다:
```python
{
    "schema": "xlerobot_v1",
    "stamp_ns": <ns>, "topic": "proprio", "frame": "proprio",
    "joint_names_pos": [...14...],
    "joint_pos":       [...14...],
    "joint_vel_arm_sample": [...3...],     # 베이스 mock joint pos (x,y,θ)
    "joint_names_base": ["root_x_axis_joint","root_y_axis_joint","root_z_rotation_joint"],
    "base_joint_vel":  [...3...],
    "base_pose":  [x, y, z, qx, qy, qz, qw],  # 와이어는 xyzw (IsaacLab 내부의 wxyz와 다름)
    "base_twist": [vx, vy, vz, wx, wy, wz],
}
```

### 3.5 sim 쪽에서 누가 publish 하나

[transport/publisher.py:StreamPublisher](../src/indoory_isaac_sim/transport/publisher.py)가 `(센서, 토픽)` 별로 row를 들고 있다가, `sim_step_hz / target_rate_hz`로 **modulo**를 만들어 매 틱 그 modulo에 맞는 row만 `binding.encode(...)` → `bus.publish(topic, payload)`. 즉 timing이 **시뮬 시간 기준**으로 락스텝이라, SLAM/데이터셋 레코더 같은 물리-인지 컨슈머가 안전하게 사용할 수 있다.

---

## 4. RPC — REQ ↔ REP :5557

### 4.1 [rpc_client.py](../examples/rpc_client.py) CLI

`req_socket` 컨텍스트 안에서 `rpc(sock, op, **kwargs)`를 한 번 호출하면 끝. 응답은 `{"ok": bool, "error": str|None, ...}`.

지원되는 op (sim 측 핸들러 [sim_server.py:228-284](../src/indoory_isaac_sim/sim_server.py#L228-L284)):

| op | 인자 | 효과 |
|----|------|-----|
| `reset` | — | `env.reset()` 호출, `last_action` 0으로 |
| `joint_names` | — | `joint_pos_order` (14), `joint_vel_order` (3) 리스트 반환 |
| `topic_list` | — | 현재 활성 publish 토픽 목록 |
| `enable_stream` | `topic`, `rate_hz?` | 해당 토픽 활성화 (rate도 함께 변경 가능) |
| `disable_stream` | `topic` | 토픽 비활성 |
| `set_stream_rate` | `topic`, `rate_hz` | 런타임 rate 변경 (modulo 재계산) |
| `set_stream_param` | `topic`, `key=value` 다수 | 인코더 파라미터(예: `jpeg_quality=85`) 조정 |
| `set_pose` | `pose=[x,y,z,qx,qy,qz,qw]` | 로봇을 월드 좌표로 텔레포트. 와이어는 xyzw, sim 내부에서 wxyz로 재배열 |
| `set_mode` | `mode` | 프로파일 변경은 **sim 재시작 필요** — 에러 응답으로 알려줌 |
| `shutdown` | — | sim 종료 |

CLI 예시 (README + 파일 docstring에서):
```bash
python examples/rpc_client.py topic_list
python examples/rpc_client.py reset
python examples/rpc_client.py disable_stream depth.front
python examples/rpc_client.py enable_stream depth.wrist 5
python examples/rpc_client.py set_stream_param rgb.front jpeg_quality=85
python examples/rpc_client.py set_pose 0 0 0.05 0 0 0 1
```

### 4.2 RPC와 PULL의 차이

- **PULL :5556** — 매 틱마다 보내는 **연속 액션** 채널. 액션이 안 와도 sim은 멈추지 않고 마지막 액션을 계속 적용.
- **REP :5557** — **이산적인 사이드 이펙트**(reset / 텔레포트 / 스트림 토글)용. 동기 응답이 필요하므로 REQ-REP. `rep_socket`은 timeout이 짧게 걸려 있어, sim이 죽었으면 클라이언트가 `zmq.Again`으로 알 수 있다.

---

## 5. 한눈에 보는 클라이언트별 역할

| 파일 | PUB 구독 | PULL 송신 | REP 호출 | 한 줄 요약 |
|------|---------|----------|---------|------------|
| [_client_common.py](../examples/_client_common.py) | (헬퍼) | (헬퍼) | (헬퍼) | 소켓 컨텍스트 + `pack_command` / `rpc` |
| [noop_client.py](../examples/noop_client.py) | ✅ 모든 토픽 | — | — | 토픽 Hz/대역폭 측정. 명령 안 보냄 |
| [keyboard_client.py](../examples/keyboard_client.py) | `proprio` | ✅ 매 틱 | — | WASD 텔레옵, 양방향 통신 데모 |
| [scripted_arm_client.py](../examples/scripted_arm_client.py) | — | ✅ 30 Hz sin | — | `Pitch`만 흔들어서 액션 매핑 검증 |
| [rpc_client.py](../examples/rpc_client.py) | — | — | ✅ 단발 | RPC CLI 래퍼 |

---

## 6. 새 클라이언트를 만들 때 체크리스트

1. **PUSH(:5556)로 보낼 액션은 항상 17차원으로 전부 채워야 한다** — `arm` 14개와 `base` 3개. 사용 안 하는 관절은 0(혹은 home pose). [`pack_command`](../examples/_client_common.py#L59-L72)가 길이를 강제 검증한다.
2. **명령은 매 틱 보내라** — 명령이 끊기면 `last_action`이 stale 상태로 계속 적용된다 (특히 `cmd_vel`이 0이 아닐 때 위험).
3. **인덱스로 관절을 가리키지 말고 이름으로 lookup** — [`schema.JOINT_POS_ORDER`](../src/indoory_isaac_sim/wire/schema.py#L39-L45)에서 `index("Pitch")`처럼. 와이어 순서가 v2에서 바뀔 수 있다.
4. **SUB는 prefix 매칭**이라 `"depth"`로 모든 depth 토픽을 한번에 받을 수 있다. 받자마자 항상 `recv_multipart()` 두 프레임을 같이 처리.
5. **schema 필드는 반드시 검사** — 아직 v1만 정의됐지만, `xlerobot_v1`이 아닌 메시지가 오면 **버려라**. sim 쪽 [validate_command](../src/indoory_isaac_sim/wire/schema.py#L123-L141)도 그렇게 동작한다.
6. **base_pose의 quat는 xyzw**(와이어). IsaacLab 내부는 wxyz라 sim 쪽 [_set_robot_pose](../src/indoory_isaac_sim/sim_server.py#L286-L298)가 변환 처리. 클라이언트는 xyzw로 통일.
7. RPC `set_mode`처럼 **resolution / 토포로지가 바뀌는 변경은 sim 재시작 필수** — `--profile` 인자로 부팅 시 결정된다.

---

## 7. 의존성

클라이언트 측은 의도적으로 가볍게 짜여 있다:

- 필수: `pyzmq`, `msgpack`
- IsaacLab / torch / leisaac 같은 sim-side 의존성은 **전혀 없다** (이게 설계 목표 — 로봇 서버는 sim-agnostic해야 한다).

따라서 어떤 conda env에서도, 별도 컨테이너에서도 클라이언트를 돌릴 수 있다. sim 서버만 `leisaac` env에서 [run_sim.sh](../run_sim.sh) 또는 `python -m indoory_isaac_sim.sim_server`로 띄우면 된다.