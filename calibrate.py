#!/usr/bin/env python3
"""카메라 ↔ 로봇 캘리브레이션 → calib.yaml (픽셀 → 로봇 XY 호모그래피, 파지 높이 평면).

  python3 calibrate.py              # 자동: 로봇이 사과를 격자점에 내려놓고 카메라가 찾는다 (약 2분)
  python3 calibrate.py --hand       # 수동: 사과를 손으로 놓고, 팔을 손으로 끌어 사과 위에 대고 Enter
  python3 calibrate.py --verify     # 검증: 검출된 사과 위로 차례로 가서 멈춘다 (집지 않음)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hand", action="store_true", help="손으로 팔을 끌어서")
    g.add_argument("--verify", action="store_true", help="캘리브레이션 확인")
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
        robot.connect(enable=not args.hand)
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
