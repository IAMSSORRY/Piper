#!/usr/bin/env python3
"""카메라 ↔ 로봇 캘리브레이션 → calib.yaml (픽셀 → 로봇 XY 호모그래피, 파지 높이 평면).

  python3 calibrate.py              # 자동: 로봇이 사과를 격자점에 내려놓고 카메라가 찾는다 (약 2분)
  python3 calibrate.py --hand       # 수동: 사과를 손으로 놓고, 팔을 손으로 끌어 사과 위에 대고 Enter
  python3 calibrate.py --verify     # 검증: 검출된 사과 위로 차례로 가서 멈춘다 (집지 않음)
  python3 calibrate.py --floor      # 바닥 높이만: 트레이 바닥 · 상자 칸 바닥에 손가락끝을 대고 Enter
  python3 calibrate.py --teach      # 위치 재기: 팔을 끌어 손가락끝을 트레이 모서리 4개 · 상자 칸 꼭짓점 6개 (xy 만) ·
                                    #   마지막에 트레이 바닥 · 상자 바닥 (높이만) 에 대고 Enter → config.yaml 갱신
  SIM=true python3 calibrate.py     # 로봇·카메라 없이 순서만 확인

자동 모드 준비: 트레이를 비우고(다른 사과가 있으면 부딪힌다) 사과 1개만 손에 든다.
사과를 실제 집는 높이(pick.grasp_height_m)에 놓고 재므로, 카메라 시차 오차가 미션과 같은 조건에서 없어진다.
"""
import argparse
import logging
import math
import os
import random
import sys
import time

import cv2
import numpy as np
import yaml

from mission import _ask, load_config
from piper_robot import ForceStop, MotionTimeout, Robot, RobotError, RobotFault
from vision import Calib, apply_calib, calib_path, detect_apples, draw, open_camera

log = logging.getLogger("calib")
HERE = os.path.dirname(os.path.abspath(__file__))


def grid_points(cfg):
    c = cfg["calibration"]
    (x1, y1), (x2, y2) = c["corner1_xy_m"], c["corner2_xy_m"]
    m = c["margin_m"]
    xs = np.linspace(min(x1, x2) + m, max(x1, x2) - m, c["grid"][0])
    ys = np.linspace(min(y1, y2) + m, max(y1, y2) - m, c["grid"][1])
    # 지그재그 순서 (이동 거리 최소)
    return [(float(x), float(y)) for i, x in enumerate(xs) for y in (ys if i % 2 == 0 else ys[::-1])]


class Observer:
    """카메라로 트레이의 사과 1개 위치(픽셀)를 찾는다. 처음부터 있던 사과(baseline)는 무시."""

    def __init__(self, cfg, sim):
        self.cfg, self.sim = cfg, sim
        self.baseline = []
        self.last_img = None
        if sim:
            # 가짜 카메라: 대략적인 위→아래 시점 (1px ≈ 0.85mm) + 1px 노이즈
            s = 0.00085
            self.truth = Calib(np.array([[0, s, -0.017], [s, 0, -0.50], [0, 0, 1]]))
            self.rng = random.Random(0)
        else:
            self.cam = open_camera(cfg)

    def snap(self):
        img = self.cam.latest(after=time.monotonic() + self.cfg["vision"]["settle_s"])
        self.last_img = img
        return img, detect_apples(img, self.cfg)

    def set_baseline(self):
        if self.sim:
            return
        _, self.baseline = self.snap()
        if self.baseline:
            log.warning("트레이에 사과 %d개가 이미 있습니다 — 격자점과 겹치면 부딪힙니다. 치우는 것을 권장", len(self.baseline))

    def find(self, expect_xy=None):
        """새로 생긴 사과 1개의 픽셀 중심 (u, v). 없거나 여러 개면 None."""
        if self.sim:
            u, v = self.truth.xy2px(*expect_xy)
            return u + self.rng.gauss(0, 1), v + self.rng.gauss(0, 1)
        _, apples = self.snap()
        new = [a for a in apples if all(math.hypot(a.u - b.u, a.v - b.v) > 0.5 * b.r for b in self.baseline)]
        if len(new) != 1:
            log.warning("새 사과 %d개 검출 (1개여야 함)", len(new))
            return None
        return new[0].u, new[0].v

    def close(self):
        if not self.sim:
            self.cam.close()


def solve_and_save(cfg, pairs, obs, mode):
    if len(pairs) < 4:
        raise RobotError(f"대응점이 {len(pairs)}개뿐입니다 (최소 4개)")
    px = [p[0] for p in pairs]
    xy = [p[1] for p in pairs]
    calib, res = Calib.solve(px, xy)
    lim = cfg["calibration"]["max_residual_m"]
    # 오차 큰 점 하나씩 빼며 재계산 (4점 이상 남을 때까지)
    while max(res) > lim and len(px) > 4:
        k = int(np.argmax(res))
        log.warning("오차 큰 점 제외: xy=(%.3f, %.3f) 오차 %.1fmm", xy[k][0], xy[k][1], res[k] * 1000)
        del px[k], xy[k]
        calib, res = Calib.solve(px, xy)
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    log.info("캘리브레이션: %d점, RMS %.1fmm, 최대 %.1fmm", len(res), rms * 1000, max(res) * 1000)
    for (u, v), (x, y), r in zip(px, xy, res):
        log.info("  px (%4.0f, %4.0f) → xy (%.3f, %.3f)  오차 %.1fmm", u, v, x, y, r * 1000)
    if max(res) > lim:
        log.warning("최대 오차 %.1fmm > 허용 %.1fmm — 결과는 저장하지만 --verify 로 꼭 확인하세요",
                    max(res) * 1000, lim * 1000)
    path = calib_path(cfg)
    calib.save(path, created=time.strftime("%Y-%m-%d %H:%M:%S"), mode=mode, sim=obs.sim,
               plane_z_m=float(cfg["pick"]["tray_z_m"] + cfg["pick"]["grasp_height_m"]),
               rms_mm=round(float(rms) * 1000, 2), max_mm=round(float(max(res)) * 1000, 2),
               points=[{"px": [round(float(u), 1), round(float(v), 1)], "xy": [round(float(x), 4), round(float(y), 4)]}
                       for (u, v), (x, y) in zip(px, xy)])
    log.info("저장: %s", path)
    if obs.last_img is not None:
        out = obs.last_img.copy()
        for (u, v), (x, y) in zip(px, xy):
            pu, pv = calib.xy2px(x, y)
            cv2.circle(out, (int(u), int(v)), 6, (0, 255, 0), 2)
            cv2.drawMarker(out, (int(pu), int(pv)), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        p = os.path.join(HERE, "snapshots", time.strftime("%H%M%S_calib.jpg"))
        cv2.imwrite(p, out)
        log.info("확인 이미지: %s (초록 = 검출, 빨강 = 계산)", p)
    return calib


def run_auto(robot, cfg, obs):
    p, m = cfg["pick"], cfg["motion"]
    z_put = p["tray_z_m"] + p["grasp_height_m"]
    fast, slow = m["transit_speed_pct"], p["descend_speed_pct"]
    pts = []
    for x, y in grid_points(cfg):          # 출발 전에 전부 확인. 수직으로 못 닿는 점은 뺀다
        try:
            robot._check_ws(x, y, z_put)
            robot.lift_z(x, y)
            pts.append((x, y))
        except RobotError as e:
            log.warning("격자점 제외: %s", e)
    if len(pts) < 4:
        raise RobotError(f"닿는 격자점이 {len(pts)}개뿐 (최소 4) — calibration 모서리 확인")

    def up(x, y):
        return robot.lift_z(x, y)

    def app(x, y):
        return min(p["tray_z_m"] + p["approach_height_m"], robot.lift_z(x, y))

    robot.go_home()
    obs.set_baseline()
    robot.grip(True)
    if not obs.sim:
        s = _ask("그리퍼 사이에 사과 1개를 끼우고 Enter (q=취소). ★ 손가락 판의 앞뒤 방향으로도 정확히 가운데에", 120)
        if s is None or s == "q":
            raise RobotError("취소됨")
    w = robot.grip(False)
    if w < cfg["gripper"]["min_grasp_width_m"]:
        raise RobotError(f"사과를 못 쥐었습니다 (폭 {w * 1000:.1f}mm)")
    if not obs.sim and (_ask("사과가 손가락 판 한가운데(앞뒤로도 가운데)에 쥐어졌나? Enter=예, q=취소", 60) or "") == "q":
        raise RobotError("취소됨 — 사과를 가운데에 다시 끼우고 재실행")

    pairs = []
    last = pts[0]
    for i, (x, y) in enumerate(pts):
        log.info("---- 점 %d/%d (%.3f, %.3f)", i + 1, len(pts), x, y)
        robot.transit_to(x, y, fast)
        try:   # 내려가다 '도달 불가'면 이 점만 건너뛴다 (사과는 아직 쥐고 있다)
            robot.down_to(x, y, app(x, y), fast)
            robot.down_to(x, y, z_put, slow)
        except (RobotFault, MotionTimeout, ForceStop):
            raise
        except RobotError as e:
            log.warning("점 %d 건너뜀 (%s)", i + 1, e)
            robot.transit_to(*last, fast)
            continue
        last = (x, y)
        robot.grip(True)
        robot.wait(0.3)
        robot.down_to(x, y, up(x, y), p["lift_speed_pct"])
        robot.go_home()                       # 카메라 시야에서 팔 빼기
        uv = obs.find((x, y))
        if uv is None:
            log.warning("점 %d 검출 실패 → 건너뜀", i + 1)
        else:
            pairs.append((uv, (x, y)))
            log.info("검출 px (%.0f, %.0f)", *uv)
        # 놓은 자리에서 다시 집기 (로봇이 놓은 좌표 = 사과 위치). 한 번 실패하면 한 번 더
        robot.transit_to(x, y, fast)
        for attempt in range(2):
            robot.down_to(x, y, app(x, y), fast)
            robot.down_to(x, y, z_put, slow)
            w = robot.grip(False)
            if w >= cfg["gripper"]["min_grasp_width_m"]:
                break
            log.warning("다시 집기 실패 (폭 %.1fmm)%s", w * 1000, " → 재시도" if attempt == 0 else "")
            robot.grip(True)
            robot.down_to(x, y, app(x, y), fast)
        robot.down_to(x, y, up(x, y), p["lift_speed_pct"])
        if w < cfg["gripper"]["min_grasp_width_m"]:
            log.error("사과를 다시 못 집었습니다 (굴렀을 수 있음) — 지금까지 %d점으로 계산", len(pairs))
            break
    # 마지막 점에 사과 내려놓고 홈
    if robot.gripper_width() >= cfg["gripper"]["min_grasp_width_m"]:
        x, y = last
        robot.down_to(x, y, app(x, y), fast)
        robot.down_to(x, y, z_put, slow)
        robot.grip(True)
        robot.down_to(x, y, up(x, y), p["lift_speed_pct"])
    robot.go_home()
    return pairs


def run_hand(robot, cfg, obs):
    """모터 끈 채로: 사과를 놓고 → 팔을 끌어 그리퍼를 사과에 맞추고 Enter → 팔 치우고 Enter."""
    print("수동 캘리브레이션: 트레이 여러 곳(최소 4, 넓게 퍼지게)에서 반복. 끝내려면 q")
    obs.set_baseline()
    pairs = []
    while True:
        s = _ask(f"[{len(pairs) + 1}] 사과를 트레이에 놓고, 티칭 버튼으로 팔을 끌어 그리퍼 중심을 사과 위(파지 높이)에 맞춘 뒤 Enter", 300)
        if s is None or s == "q":
            break
        x, y, z = robot.current_pose()[:3]
        log.info("로봇 xyz (%.3f, %.3f, %.3f)", x, y, z)
        s = _ask("팔을 카메라 시야 밖으로 치우고 Enter", 120)
        if s is None or s == "q":
            break
        uv = obs.find((x, y))
        if uv is None:
            continue
        pairs.append((uv, (x, y)))
        log.info("기록 px (%.0f, %.0f) ↔ (%.3f, %.3f)", *uv, x, y)
        if obs.sim and len(pairs) >= 6:
            break
    return pairs


def run_verify(robot, cfg):
    """검출된 사과마다 접근 높이까지 내려가 멈춘다. 그리퍼 중심이 사과 중심 위에 오는지 눈으로 확인."""
    calib = Calib.load(calib_path(cfg))
    if calib.info.get("sim") and not robot.sim:
        raise RobotError("calib.yaml 이 SIM 으로 만든 가짜입니다. 먼저 실기에서 python3 calibrate.py")
    cam = open_camera(cfg)
    try:
        robot.go_home()
        img = cam.latest(after=time.monotonic() + cfg["vision"]["settle_s"])
        apples = apply_calib(detect_apples(img, cfg), calib)
        p = os.path.join(HERE, "snapshots", time.strftime("%H%M%S_verify.jpg"))
        cv2.imwrite(p, draw(img, apples, cfg, calib))
        log.info("사과 %d개 검출 (%s)", len(apples), p)
        for i, a in enumerate(apples):
            log.info("사과 %d: (%.3f, %.3f) 지름 %.0fmm %s", i, a.x, a.y, a.d_m * 1000, a.grade)
            try:
                ox, oy = cfg["pick"].get("grasp_offset_m", [0.0, 0.0])
                a.x, a.y = a.x + ox, a.y + oy
                z = min(cfg["pick"]["tray_z_m"] + cfg["pick"]["approach_height_m"], robot.lift_z(a.x, a.y))
                robot.transit_to(a.x, a.y, cfg["motion"]["transit_speed_pct"])
                robot.down_to(a.x, a.y, z, cfg["pick"]["descend_speed_pct"])
            except RobotError as e:
                if isinstance(e, (RobotFault, ForceStop)):
                    raise
                log.warning("건너뜀: %s", e)
                continue
            s = _ask("그리퍼가 사과 중심 위에 있나? Enter=다음, q=끝", 60)
            robot.down_to(a.x, a.y, robot.lift_z(a.x, a.y), cfg["motion"]["transit_speed_pct"])
            if s == "q":
                break
        robot.go_home()
    finally:
        cam.close()


TEACH_POINTS = (
    [("tray", k, f"트레이 바닥 {k} 모서리 (화면 기준)") for k in ("좌상", "우상", "좌하", "우하")]
    + [("box", (g, k), f"상자 '{g}' 칸 바닥 {k} 꼭짓점 (화면 기준)") for g in ("상", "중", "하") for k in ("좌상", "우하")]
    + [("height", "tray_floor", "트레이 바닥 (높이만 쓴다)"), ("height", "box_floor", "상자 칸 바닥 (높이만 쓴다)")]
)
# 모서리·꼭짓점은 xy 만 쓴다 (테두리 위를 찍어도 된다). 높이는 마지막에 바닥을 한 번씩 찍어서 딴다.


def _tip(robot):
    """손가락끝 로봇 좌표 (그리퍼가 기울어 있어도 정확히: 관절 → 순기구학)."""
    import ik
    if robot.sim:
        x, y, z = robot.current_pose()[:3]
        return x, y, z - robot.tool_len
    return tuple(float(v) for v in ik.fk_tip(robot.current_joints(), robot.tool_len)[0])


def _wait_enter(robot, prompt):
    """손가락끝 좌표를 계속 보여 주다가 Enter 면 그 값을 돌려준다. q = 중단."""
    import select
    print(f"\n▶ {prompt} — 손가락끝을 대고 Enter (q=중단)")
    while True:
        x, y, z = _tip(robot)
        sys.stdout.write(f"\r   손가락끝 x {x:+.3f}  y {y:+.3f}  z {z:+.3f} m   ")
        sys.stdout.flush()
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            s = sys.stdin.readline().strip().lower()
            print()
            if s == "q":
                raise KeyboardInterrupt
            return _tip(robot)


def _set_line(text, pattern, repl):
    import re
    new, n = re.subn(pattern, repl, text, count=1, flags=re.M)
    if n != 1:
        raise RuntimeError(f"config.yaml 에서 못 찾음: {pattern}")
    return new


def run_teach(robot, cfg, config_path):
    """팔 끝 티칭 버튼으로 드래그 모드에서: 점마다 손가락끝을 대고 Enter. 끝나면 config.yaml 을 고친다(백업 남김)."""
    print("위치 재기: 팔 끝의 티칭 버튼을 눌러 드래그 모드로 바꾸고, 안내대로 손가락끝을 대고 Enter.")
    pts = {}
    for kind, key, prompt in TEACH_POINTS:
        pts[(kind, key)] = _wait_enter(robot, prompt)
        log.info("기록 %s: (%.3f, %.3f, %.3f)", prompt, *pts[(kind, key)])
    tl = robot.tool_len
    tray = {k: pts[("tray", k)] for k in ("좌상", "우상", "좌하", "우하")}
    tray_floor = pts[("height", "tray_floor")][2]
    box_floor = pts[("height", "box_floor")][2]
    boxes = {}
    for g in ("상", "중", "하"):
        a, b = pts[("box", (g, "좌상"))], pts[("box", (g, "우하"))]
        boxes[g] = {"xy": ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), "size": (abs(b[0] - a[0]), abs(b[1] - a[1])),
                    "floor": box_floor}

    print("\n===== 결과 (손가락끝 기준, m) =====")
    for k, p in tray.items():
        print(f"트레이 {k}: ({p[0]:.3f}, {p[1]:.3f})")
    print(f"트레이 바닥 z {tray_floor:.3f} → pick.tray_z_m {tray_floor + tl:.3f} (지금 {cfg['pick']['tray_z_m']:.3f})")
    for g, b in boxes.items():
        print(f"'{g}' 칸 중심 ({b['xy'][0]:.3f}, {b['xy'][1]:.3f})  크기 {b['size'][0] * 1000:.0f}x{b['size'][1] * 1000:.0f}mm")
    print(f"상자 바닥 z {box_floor:.3f} → floor_z_m {box_floor + tl:.3f}")
    old_box = float(np.mean([b["floor_z_m"] for b in cfg["boxes"].values()]))
    if abs(tray_floor + tl - cfg["pick"]["tray_z_m"]) > 0.02 or abs(box_floor + tl - old_box) > 0.02:
        s = _ask(f"바닥 높이가 지금 값과 2cm 넘게 다릅니다 (트레이 {cfg['pick']['tray_z_m']:.3f}→{tray_floor + tl:.3f}, "
                 f"상자 {old_box:.3f}→{box_floor + tl:.3f}). 높이도 바꿀까? (y=바꿈, Enter=높이는 그대로)", 120)
        if (s or "").lower() != "y":
            tray_floor = cfg["pick"]["tray_z_m"] - tl
            for g, b in boxes.items():
                b["floor"] = cfg["boxes"][g]["floor_z_m"] - tl
            log.info("높이는 그대로 둔다 (xy 만 갱신)")

    # 원본 기록
    raw = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "tool_length_m": tl,
           "points_tip_m": {f"{kind}:{key if isinstance(key, str) else '/'.join(key)}": [round(v, 4) for v in p]
                            for (kind, key), p in pts.items()}}
    with open(os.path.join(HERE, "teach_points.yaml"), "w") as f:
        yaml.safe_dump(raw, f, allow_unicode=True, sort_keys=False)

    # config.yaml 갱신 (주석 유지: 해당 줄의 값만 바꾼다)
    with open(config_path) as f:
        text = f.read()
    bak = config_path + time.strftime(".bak-%H%M%S")
    with open(bak, "w") as f:
        f.write(text)
    when = time.strftime("%H:%M")
    ul, lr = tray["좌상"], tray["우하"]
    text = _set_line(text, r"^(  corner1_xy_m: )\[[^\]]*\](.*)$", rf"\g<1>[{ul[0]:.3f}, {ul[1]:.3f}]   # 트레이 왼쪽 위 (화면 기준). {when} --teach")
    text = _set_line(text, r"^(  corner2_xy_m: )\[[^\]]*\](.*)$", rf"\g<1>[{lr[0]:.3f}, {lr[1]:.3f}]   # 트레이 오른쪽 아래 (화면 기준). {when} --teach")
    text = _set_line(text, r"^(  tray_z_m: )[0-9.]+(.*)$", rf"\g<1>{tray_floor + tl:.3f}\g<2>")
    for g, b in boxes.items():
        text = _set_line(text, rf"^(  {g}: \{{xy_m: )\[[^\]]*\]", rf"\g<1>[{b['xy'][0]:.3f}, {b['xy'][1]:.3f}]")
        text = _set_line(text, rf"^(  {g}: \{{.*floor_z_m: )[0-9.]+", rf"\g<1>{b['floor'] + tl:.3f}")
    # 상자 위치 확인 기준 (미션 시작 때 사진과 비교): 칸 중심 픽셀 = 지금 캘리브레이션으로 투영, 상자 모서리 = 새 사진
    p = calib_path(cfg)
    if os.path.exists(p):
        calib = Calib.load(p)
        px = {g: [int(round(v)) for v in calib.xy2px(*b["xy"])] for g, b in boxes.items()}
        text = _set_line(text, r"^(    compartments_px: )\{[^}]*\}",
                         "\\g<1>{" + ", ".join(f"{g}: [{u}, {v}]" for g, (u, v) in px.items()) + "}")
        if _ask("팔을 카메라 시야 밖(트레이·상자 위가 아닌 곳)으로 치우고 Enter — 상자 기준 사진을 찍는다 (s=건너뜀)", 300) != "s":
            from vision import find_box
            cam = open_camera(cfg)
            try:
                img = cam.latest(after=time.monotonic() + cfg["vision"]["settle_s"])
            finally:
                cam.close()
            cv2.imwrite(os.path.join(HERE, "snapshots", time.strftime("%H%M%S_teach_box.jpg")), img)
            quad = find_box(img, cfg)
            if quad is None:
                log.warning("사진에서 상자를 못 찾음 → vision.box.ref_corners_px 는 그대로")
            else:
                text = _set_line(text, r"^(    ref_corners_px: )\[\[.*?\]\]",
                                 "\\g<1>[" + ", ".join(f"[{int(u)}, {int(v)}]" for u, v in quad) + "]")
    else:
        log.warning("calib.yaml 이 없어 상자 칸 픽셀(compartments_px)은 그대로 — 먼저 python3 calibrate.py")
    with open(config_path, "w") as f:
        f.write(text)
    log.info("config.yaml 갱신 (백업 %s), 원본 점 teach_points.yaml", os.path.basename(bak))


def run_floor(robot, cfg, config_path):
    """바닥 높이만: 트레이 바닥 · 상자 칸 바닥에 손가락끝을 대고 Enter → pick.tray_z_m, boxes.*.floor_z_m."""
    print("바닥 높이 재기: 팔 끝의 티칭 버튼으로 드래그 모드에서, 손가락끝을 바닥에 대고 Enter.")
    tl = robot.tool_len
    tray = _wait_enter(robot, "트레이 바닥")[2]
    box = _wait_enter(robot, "상자 칸 바닥")[2]
    print(f"\n트레이 바닥 z {tray:.3f} → pick.tray_z_m {tray + tl:.3f} (지금 {cfg['pick']['tray_z_m']:.3f})")
    old_box = {g: b["floor_z_m"] for g, b in cfg["boxes"].items()}
    print(f"상자 바닥 z {box:.3f} → floor_z_m {box + tl:.3f} (지금 " + ", ".join(f"{g} {v:.3f}" for g, v in old_box.items()) + ")")
    if (_ask("config.yaml 에 쓸까? (y=씀)", 120) or "").lower() != "y":
        log.info("안 씀")
        return
    with open(config_path) as f:
        text = f.read()
    bak = config_path + time.strftime(".bak-%H%M%S")
    with open(bak, "w") as f:
        f.write(text)
    text = _set_line(text, r"^(  tray_z_m: )[0-9.]+", rf"\g<1>{tray + tl:.3f}")
    for g in cfg["boxes"]:
        text = _set_line(text, rf"^(  {g}: \{{.*floor_z_m: )[0-9.]+", rf"\g<1>{box + tl:.3f}")
    with open(config_path, "w") as f:
        f.write(text)
    log.info("config.yaml 갱신 (백업 %s)", os.path.basename(bak))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hand", action="store_true", help="손으로 팔을 끌어서")
    g.add_argument("--verify", action="store_true", help="캘리브레이션 확인")
    g.add_argument("--floor", action="store_true", help="트레이·상자 바닥 높이만 재서 config.yaml 갱신")
    g.add_argument("--teach", action="store_true", help="트레이 모서리·상자 칸 꼭짓점·높이를 손가락끝으로 재서 config.yaml 갱신")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s.%(msecs)03d %(levelname).1s [%(name)s] %(message)s", datefmt="%H:%M:%S")
    sim = os.environ.get("SIM", "false").lower() in ("1", "true", "yes")
    cfg = load_config(args.config)
    os.makedirs(os.path.join(HERE, "snapshots"), exist_ok=True)
    robot = Robot(cfg, sim=sim)
    obs = None
    try:
        robot.connect(enable=not (args.hand or args.teach or args.floor))
        if args.floor:
            run_floor(robot, cfg, args.config)
            return 0
        if args.teach:
            run_teach(robot, cfg, args.config)
            return 0
        if args.verify:
            run_verify(robot, cfg)
            return 0
        obs = Observer(cfg, sim)
        pairs = run_hand(robot, cfg, obs) if args.hand else run_auto(robot, cfg, obs)
        solve_and_save(cfg, pairs, obs, "hand" if args.hand else "auto")
        if sim:
            log.warning("SIM 결과는 가짜 카메라 기준입니다 — 실기에서 다시 돌리세요")
        return 0
    except KeyboardInterrupt:
        log.error("Ctrl+C → 비상정지")
        robot.estop()
        return 130
    except RobotFault as e:
        log.error("로봇 고장: %s", e)
        robot.estop()
        return 2
    except (RobotError, IOError, FileNotFoundError) as e:
        log.error("%s", e)
        robot.hold()
        return 1
    finally:
        if obs:
            obs.close()
        robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
