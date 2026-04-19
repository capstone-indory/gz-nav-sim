# Bench — 디버깅 / 벤치마킹 / 재현 환경

여러 매핑 조합 (DA3, nvblox, VGGT-SLAM)을 preset 단위로 돌리고, 결과를 자동
기록·비교한다. SLAM 성능 비교의 표준 패턴인 **bag record → replay**도 지원.

## 디렉토리

```
bench/
├ presets/              # 명명된 실행 조합
│  ├ da3_nvblox.sh        DA3 mono depth → nvblox TSDF (default)
│  ├ vggt_only.sh         VGGT-SLAM 단독
│  ├ vggt_nvblox.sh       VGGT + DA3 + nvblox 동시 (비교용)
│  └ baseline_lidar.sh    SLAM Toolbox + Nav2만 (depth 매핑 OFF)
├ runs/<ts>_<preset>/   # 자동 생성, run 별 격리
│  ├ config.sh             # 사용된 preset 복사본
│  ├ launch_args.txt       # 실제 launch args
│  ├ git_commit.txt        # 코드 commit hash
│  ├ git_dirty.txt         # uncommitted changes (있을 시)
│  ├ stdout.log            # 전체 launch stdout
│  ├ topics.bag/           # ros2 bag (--record 시)
│  ├ metrics.json          # extract_metrics.py 결과
│  └ notes.md              # 사용자 메모
├ run.sh                # 메인 runner
├ drive.sh              # robot 주행 (teleop / nav2 goal / square)
├ replay.sh             # bag → mapper-only 재실행 (WIP)
├ extract_metrics.py    # stdout → metrics.json
└ compare.py            # 여러 run 표 비교
```

## 빠른 시작

```bash
# 1. 실행
./bench/run.sh da3_nvblox --record --note "1차 baseline"

# 2. 다른 터미널에서 robot 주행
./bench/drive.sh teleop --duration 60 --vx 0.3 --wz 0.2
# 또는 Nav2 goal
./bench/drive.sh nav2 --x 5 --y 0
# 또는 사각형
./bench/drive.sh square --side 3

# 3. Ctrl+C로 종료
# → bench/runs/<ts>_da3_nvblox/ 에 모든 결과 저장됨

# 4. 비교
python3 bench/compare.py
```

## 실행 옵션

`./bench/run.sh <preset> [opts]`

| 옵션 | 효과 |
|---|---|
| `--record` | input 토픽을 ros2 bag으로 기록 (재현용) |
| `--note "..."` | run 메모 저장 |
| `--duration <sec>` | N초 후 자동 종료 |

## 주행 옵션

`./bench/drive.sh <mode> [opts]`

| 모드 | 설명 |
|---|---|
| `teleop` | `/cmd_vel_teleop`에 직접 publish |
| `nav2` | `/goal_pose`에 PoseStamped 1회 publish (Nav2 핸들링) |
| `square` | nav2 goal로 사각형 4점 순회 |

teleop이 안 먹히면 (nav2 velocity_smoother가 차단) `drive.sh` 안의
`/cmd_vel` 직접 publish 줄 주석 해제.

## 메트릭

`extract_metrics.py`가 stdout.log에서 자동 추출:

- DA3: 매 batch의 (s, t, inliers), smoothed (s, t), timing (decode/tf/infer/lidar/publish/total)
- VGGT: submap 수, global lock scale, ready 시점
- nvblox: depth/color/mesh 통합 누계
- 에러 카운트 + 샘플

## 비교 표

```
python3 bench/compare.py                       # 최근 10개
python3 bench/compare.py runs/20260419_*       # 패턴 매칭
python3 bench/compare.py runs/run1 runs/run2   # 명시
```

출력 컬럼: model, da3_batches, da3_s_min/max/jitter, smoothed_ds_max,
infer_mean_s, vggt_submaps, vggt_lock_scale, nvblox_depth_n, errors.

## 알려진 한계

- `replay.sh`는 현재 wrapper만 출력 (mapper-only launch 분리 필요)
- `extract_metrics.py` 패턴은 알려진 로그 포맷에만 매칭
- Nav2 goal은 SLAM Toolbox map이 어느 정도 만들어진 뒤 보내야 함
