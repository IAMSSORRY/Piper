# Piper — SSORRY 로봇 제어 · 비전

AgileX PIPER 6축 로봇팔이 트레이의 사과를 카메라로 찾아 집고, 색과 멍으로 등급(상 · 중 · 하)을 매겨 상자 칸에 나눠 넣습니다.
판정 결과와 근거 수치는 [SSORRY 서버](https://github.com/IAMSSORRY/Server)로 보냅니다.

- MoveIt · ROS2 없이 `piper_sdk` 로 직접 제어합니다
- 단위: 거리 m, 각도 rad

---

## 파일

| 파일 | 내용 |
|---|---|
| `config.yaml` | 좌표 · 높이 · 속도 · 그리퍼 · 카메라 · 판정 기준. **현장에서는 이 파일만 고칩니다** (★ = 실측 필요) |
| `mission.py` | 미션 실행 진입점. `pick` → `inspect` → `place`, 점검 명령, 원격 제어 서버(`--serve`) |
| `piper_robot.py` | CAN 점검, 이동 · 그리퍼 · 홈 · 비상정지, SIM 로봇 |
| `ik.py` | SDK 순기구학 + 수치 역기구학 |
| `vision.py` | 사과 검출 · 등급, 회색 카드 색 보정, 회전 멍 검사, 손목 카메라 굴림 판정 |
| `live_detect.py` | 위 카메라 프레임마다 사과를 검출해 대시보드에 실시간 박스 전송 |
| `adaptive.py` | 굴림이 나면 다음 놓기의 하강 속도 · 놓는 높이를 낮춤 |
| `calibrate.py` | 카메라 ↔ 로봇 캘리브레이션 → `calib.yaml` |
| `dashboard.py` | 서버 `/ingest/*` 로 판정 · 모션 · 미션 이벤트 전송 (백그라운드) |
| `calib.yaml`, `color.yaml` | 현장 캘리브레이션 결과. 카메라나 트레이를 옮기면 다시 만듭니다 |

## 미션 흐름

사과 하나를 다섯 단계로 처리합니다.

| 단계 | 동작 |
|---|---|
| 1. 파지 | 위 카메라 좌표 → IK. 트레이 벽 · 이웃 사과에서 여유가 가장 큰 방향으로 집게를 돌려 집음 |
| 2. 검사 | 사과를 들어 위 카메라에 비추고 손목을 −100° → +100° 돌리며 멍 비율 측정 |
| 3. 판정 | 색 + 멍으로 등급 결정, 근거 수치를 서버로 전송 |
| 4. 적재 | 상자 벽 위로 이동해 등급별 칸에 놓음 |
| 5. 확인 | 손목 카메라로 놓은 뒤 사과가 굴렀는지 확인 |

## 판정 기준

| 등급 | 조건 |
|---|---|
| 상 | 빨강 비율 ≥ 0.5, 흠 비율 ≤ 0.10, 회전 검사 멍 < 1.5% |
| 중 | 빨강이지만 회전 검사 멍 ≥ 1.5% |
| 하 | 빨강 비율 < 0.5 또는 흠 비율 > 0.10 — 회전 검사 생략 |

- 위 카메라(Sony A7C, UVC)는 화이트밸런스 · 노출을 제어할 수 없어서 회색 카드로 **소프트웨어 색 보정**을 합니다
- 신뢰도는 임계값에서 떨어진 정도를 0.5 ~ 1.0으로 환산한 값입니다 (확률 아님)
- 실측: 멍 사과 2.0 ~ 7.7%, 정상 사과 0.6% 이하

## 굴림 감지와 적응형 놓기

놓기 직전 쥔 사과 위치를 기억하고, 그리퍼를 연 뒤 1초 정지 상태에서 60 px 넘게 움직이거나 사라지면 굴림으로 봅니다.

| 상황 | 동작 |
|---|---|
| 굴림 발생 | 하강 속도 · 놓는 높이 20% 하향 (하한 있음) |
| 2회 연속 성공 | 5%씩 기준값까지 복구 |
| 3회 연속 감속 | 자동 조정 중단 |

## 준비

```bash
# piper_sdk 는 PIPER Studio venv 에 있습니다. 실행은 항상 이 파이썬으로
PY=~/.venvs/piper-daemons/bin/python
sudo ip link set can_arm1 type can bitrate 1000000 && sudo ip link set can_arm1 up
export SSORRY_TOKEN=<서버 INGEST_TOKEN>
```

- USB-CAN 어댑터는 **노트북 본체 포트에 직접** 꽂습니다 (카메라와 같은 허브에 꽂으면 `gs_usb ... -EPROTO` 로 끊깁니다)
- 티칭(드래그) 모드면 동작 명령이 무시됩니다 → `$PY mission.py --exit-teach` 또는 팔 끝 버튼
- PIPER Studio 의 [연결] / [연결 해제] 는 토크를 끕니다. 실행 중에는 Studio 로 팔을 만지지 않습니다

## 실행

```bash
SIM=true $PY mission.py            # 로봇 없이 전체 시퀀스
$PY mission.py --check             # CAN · 로봇 연결 점검
$PY mission.py --read-pose         # 현재 좌표 (config 에 실측값 넣을 때)
$PY mission.py --dry-run           # 사과 없이 경로 + 그리퍼를 한 단계씩 (Enter)
$PY mission.py --jaw-test          # 집게 방향 설정 확인 (한 번만)

$PY vision.py --graycard           # 회색 카드 색 보정
$PY vision.py                      # 검출 화면 확인
$PY calibrate.py                   # 카메라 ↔ 로봇 캘리브레이션
$PY calibrate.py --verify          # 사과마다 위로 가서 중심 확인

$PY mission.py                     # 본 미션
$PY mission.py --apples 8          # 사과 개수 지정
$PY mission.py --apples all        # 트레이가 빌 때까지
```

실행 중 **Ctrl+C = 비상정지**. 해제는 팔을 받친 뒤 `$PY mission.py --resume`.

## 원격 제어

대시보드에서 시작 · 정지 · 비상정지를 하려면 제어 서버로 띄웁니다. 서버의 `/control/*` 가 여기로 넘깁니다.

```bash
SSORRY_TOKEN=<토큰> $PY mission.py --serve      # 0.0.0.0:8765
```

| 요청 | 동작 |
|---|---|
| `GET /status` | `{state, error, index, placed, results}` |
| `POST /start` | 미션 시작 `{"apples": 5}` |
| `POST /estop` | **즉시 그 자리 정지 → 모터 정지** (약 0.2초). 다른 명령을 기다리지 않습니다 |
| `POST /park` | 쥔 사과를 집은 자리에 되돌리고 팔을 낮춘 뒤 정지 |
| `POST /resume` | 비상정지 해제 → 멈춘 사과부터 이어서 |
| `POST /stop` | 지금 사과까지만 하고 멈춤 |
| `POST /clear` | 칸 비움 `{"grade": "상"}` (생략 = 전체). 미션 중 사람이 칸을 비웠을 때 놓은 자리 기록을 지워 다시 놓게 한다 |

- 헤더 `Authorization: Bearer <SSORRY_TOKEN>`
- `/estop` 은 팔을 낮추지 않습니다. 위급하지 않을 때는 `/park` 를 씁니다
- ⚠ 비상정지 해제 순간 모터 전원이 잠깐 빠져 팔이 처질 수 있습니다

## 현장 실측으로 정한 것

- **공구 길이 0.132 m** — 모든 높이는 손가락 끝 기준
- **높은 이동은 관절 각도로 직접** — 그리퍼를 수직으로 높이 올리면 joint5 가 한계(±70°)를 넘습니다
- **명령은 처음 0.25초만 전송** — 계속 보내면 PIPER 가 궤적을 다시 시작해서 끊깁니다
- **도달 불가는 비상정지가 아님** — 그 자리 정지 후 그 사과만 건너뜁니다
- **캘리브레이션 오차** — 9점 실측 RMS 1.8 mm
