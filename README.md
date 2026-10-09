# PIPER 사과 선별 (SSORRY 로봇 쪽)

AgileX PIPER 6축 팔이 트레이의 사과를 카메라로 찾아 집고, 등급(상/중)에 따라 상자 두 칸에 나눠 넣는다.
굴림을 보면 다음 놓기 속도·높이를 낮춘다. 결과는 SSORRY 대시보드 서버로 보낸다.
MoveIt 없이 piper_sdk 직접 제어. 거리 m, 각도 rad.

| 파일 | 내용 |
|---|---|
| `config.yaml` | 좌표·높이·속도·그리퍼·카메라 설정. **현장에서는 이것만 고친다** (★ = 실측/확인 필요) |
| `piper_robot.py` | CAN 점검, `move_to` / `down_to` / `transit_to` / `grip` / `go_home` / `estop`, SIM 로봇 |
| `ik.py` | SDK 순기구학 + 수치 역기구학 (joint5 한계 안에서 기울기 최소) |
| `mission.py` | `pick` / `inspect` / `place`, 집게 방향 선택, 드라이런·점검 명령, 실행 진입점 |
| `vision.py` | 위 카메라(Sony) 사과 검출·등급, 회색 카드 색 보정, 손목 카메라 굴림 판정 |
| `calibrate.py` | 카메라↔로봇 캘리브레이션 (로봇이 사과를 격자점에 놓으며) → `calib.yaml` |
| `adaptive.py` | 굴림 → 하강 속도·놓는 높이 20% 하향 (하한, 3회 연속이면 중단, 연속 성공 시 복구) |
| `dashboard.py` | SSORRY 서버 `/ingest/judge`, `/ingest/motion` 전송 (백그라운드) |
| `calib.yaml`, `color.yaml` | 현장 캘리브레이션 결과 (카메라·트레이를 옮기면 다시 만든다) |

## 준비

```bash
# piper_sdk 는 PIPER Studio venv 에 있다 — 실행은 항상 이 파이썬으로
PY=~/.venvs/piper-daemons/bin/python
sudo ip link set can_arm1 type can bitrate 1000000 && sudo ip link set can_arm1 up
export SSORRY_TOKEN=<대시보드 서버 INGEST_TOKEN>
```

- USB-CAN 어댑터는 **노트북 본체 포트에 직접** (카메라와 같은 허브에 꽂으면 `gs_usb ... -EPROTO` 로 끊긴다)
- 로봇이 티칭(드래그) 모드면 동작 명령이 무시된다 → `$PY mission.py --exit-teach` 또는 팔 끝 버튼
- PIPER Studio 의 [연결]/[연결 해제] 는 토크를 끈다 (팔이 떨어진다). 실행 중에는 Studio 로 팔을 만지지 않는다

## 실행 순서

```bash
SIM=true $PY mission.py            # 로봇 없이 전체 시퀀스 (SIM 로봇은 순기구학으로 실기처럼 움직인다)
$PY mission.py --check             # CAN·로봇 연결 점검
$PY mission.py --read-pose         # 현재 좌표 (모터 enable 안 함) → config 에 실측값 넣을 때
$PY mission.py --dry-run           # 사과 없이 경로+그리퍼를 한 단계씩 (Enter)
$PY mission.py --jaw-test          # 집게 방향 설정(gripper.jaw_axis_tool) 확인 — 한 번만

$PY vision.py --graycard           # 회색 카드 색 보정 (트레이 위 카드, 팔 치우고)
$PY vision.py                      # 검출 화면 (빨강=상, 노랑=중)
$PY calibrate.py                   # 트레이 비우고, 그리퍼가 열리면 사과 1개를 손가락 판 가운데에 → Enter
$PY calibrate.py --verify          # 사과마다 위로 가서 멈춤 → 중심 확인 (어긋나면 pick.grasp_offset_m)
$PY calibrate.py --floor           # 바닥 높이만 (트레이 바닥 → 상자 칸 바닥에 손가락끝을 대고 Enter) → tray_z_m, floor_z_m
$PY calibrate.py --teach           # 티칭(드래그)으로 손가락끝을 대고 Enter: 트레이 모서리 4 (좌상·우상·좌하·우하) →
                                   #   상자 칸 꼭짓점 6 (상·중·하 각 좌상·우하) — 여기까지 xy 만 → 바닥 높이 2 (트레이 바닥, 상자 바닥)
                                   #   → config.yaml 갱신 (백업 config.yaml.bak-*), 원본 점 teach_points.yaml
$PY mission.py                     # 본 미션 (calib.yaml 있으면 카메라 모드)
$PY mission.py --apples 8          # 사과 개수 지정 (기본: 환경변수 MISSION_APPLE_COUNT → config mission.apple_count)
$PY mission.py --apples all        # 트레이에 사과가 없을 때까지 (0 / all). 대시보드에는 apple_count: null
$PY mission.py --signals manual    # 카메라 없이: 좌표·등급을 터미널로
```

실행 중 **Ctrl+C = 비상정지**. 해제: `$PY mission.py --resume` (팔을 받치고).

## 움직임 설계 (현장 실측으로 정한 것)

- **공구 길이 0.132 m**: 손가락 끝이 트레이 바닥에 닿을 때 플랜지 z = 0.132. 모든 높이는 손가락 끝 기준으로 계산
- **그리퍼 수직은 낮은 곳에서만**: 높이 올리면 joint5 가 한계(±70°)를 넘는다. 상자 벽(바닥 위 14 cm)을 넘는
  이동(`transit_to`, 손가락끝 0.19 m)은 `ik.py` 로 관절을 직접 구해 보낸다 — 그리퍼가 16~20° 기울어진다
- **내려가기(`down_to`)**: 매번 IK 를 풀어 보고, 완전히 수직이고 joint5 여유가 있으면 펌웨어 직선(MOVE L),
  아니면(베이스 가까이·멀리) 관절 명령으로 최소 기울기. 트레이 전 영역 파지 가능 (계산상 최대 기울기 8°)
- **홈**: 관절 0(접힌 자세)은 긴 그리퍼가 몸체를 친다 → 상자 위 높은 곳(`home_xy_m`)
- **명령은 처음 0.25 초만 보낸다**: 계속 보내면 PIPER 가 매번 궤적을 다시 시작해서 뚝뚝 끊긴다
- **집게 방향 자동**: 사과마다 0~180° 를 돌려 보고, 트레이 벽·이웃 사과에서 손가락 여유가 가장 큰 방향으로 손목을 돌린다.
  열림 폭은 사과 지름 + 2 cm
- **도달 불가(펌웨어 0x02/0x04)**는 비상정지가 아니라 그 자리 정지 → 그 사과만 건너뛰고 다음 사과

## 카메라

- 위: Sony A7C (UVC, PIPER 라벨 `top`). PIPER Studio camerad 스트림을 받고, 안 되면 장치를 직접 연다.
  UVC 모드엔 WB·노출 컨트롤이 없어서 회색 카드로 **소프트웨어 색 보정**
- 검출: HSV(빨강/노랑) + 거리변환 봉우리로 붙은 사과 분리. 등급: 빨강 비율 ≥ 0.5 이고 흠 비율 ≤ 0.1 → 상
- 손목: Global Shutter (라벨 `wrist`). **굴림 판정**: 놓기 직전 쥔 사과 위치 → 그리퍼 연 뒤 1초 정지 상태에서 60 px 넘게 움직이거나 사라지면 굴림
- 캘리브레이션: 파지 높이 평면의 호모그래피. 실측 9점 RMS 1.8 mm (19:26)

## 대시보드 (IAMSSORRY/Server)

| 항목 | 보내는 값 |
|---|---|
| 등급 | 상 / 중 |
| `v_value` / `threshold` | 빨강 비율 / 임계값 (판정 근거) |
| `confidence` | 임계값에서 떨어진 정도 0.5~1.0 |
| `bbox`, `cam=top` | Sony 화면 픽셀 |
| 모션 | `approach_speed` = 하강 속도 배율, `place_height` = 놓는 높이 m, `roll_detected` |

SIM 에서는 전송이 꺼진다 (`--dashboard on` 으로 강제).

## 원격 제어 (프론트에서 시작·비상정지·해제)

```bash
SSORRY_TOKEN=<토큰> ~/.venvs/piper-daemons/bin/python mission.py --serve     # 0.0.0.0:8765 (config control)
```

| 요청 | 동작 |
|---|---|
| `GET /status` | `{state, error, index, placed, results}` — state: idle / running / stopping(`/park` 정리 중) / estopped / error / done |
| `POST /start` `{"apples": 5}` | 미션 시작 (`"all"` = 트레이가 빌 때까지, 생략 = config) |
| `POST /estop` | 비상정지: **즉시 그 자리 정지 → 모터 정지** (~0.2초). 팔을 낮추지 않는다. `/park` 정리 중에도 바로 나간다 |
| `POST /park` | 정리 후 정지: 그 자리 정지 → 쥔 사과를 집은 자리에 되돌림 → 팔을 낮게 → 모터 정지 (끝난 뒤 응답, 서버 타임아웃 20초) |
| `POST /resume` | 비상정지(또는 오류 정지) 해제 → 모터 enable → **멈춘 사과부터 이어서** (칸별 개수·속도 조정 유지). 사과를 쥔 채 멈췄으면 트레이 가운데에 내려놓고 다시 찍는다 |
| `POST /stop` | 지금 사과까지만 하고 멈춤 |

- 헤더 `Authorization: Bearer <SSORRY_TOKEN>` (대시보드 ingest 토큰과 같은 값)
- PIPER Studio 비상정지·힘 이상 정지로 멈춰도 state 가 estopped / error 가 되고 `/resume` 으로 이어진다
- ⚠ 비상정지 해제(SDK `EmergencyStop(0x02)`) 순간 모터 전원이 잠깐 빠져 팔이 처질 수 있다 — 프론트에서 확인창을 띄울 것
- 상태 변화는 대시보드에 `mission` 이벤트 `{"event": "control", "state": ..., "error": ...}` 로도 간다
