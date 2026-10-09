#!/usr/bin/env python3
"""사과 5개 픽앤플레이스 (PIPER 1대).

  SIM=true python3 mission.py          # 로봇 없이 전체 시퀀스
  python3 mission.py --check           # CAN·로봇 연결 점검만
  python3 mission.py --read-pose       # 현재 좌표 출력 (config.yaml 좌표 따기용, 모터 enable 안 함)
  python3 mission.py --home            # 홈 자세로
  python3 mission.py                   # 실기 미션 (신호는 터미널 수동 입력)
  python3 mission.py --resume          # 비상정지 해제

실행 중 Ctrl+C = 비상정지.
"""
import argparse
import logging
import math
import os
import random
import select
import sys
import time

import yaml

from adaptive import AdaptiveTuner
from dashboard import Dashboard
from piper_robot import MotionTimeout, Robot, RobotError, RobotFault

log = logging.getLogger("mission")
HERE = os.path.dirname(os.path.abspath(__file__))

REQUIRED = ["can_port", "tool_orientation_rad", "home_xy_m", "tool_length_m", "transit_tip_z_m", "j5_max_rad", "z_safe_m", "workspace", "motion",
            "gripper", "pick", "inspect", "boxes", "place", "adaptive", "mission", "sim"]


def load_config(path):
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    missing = [k for k in REQUIRED if k not in cfg]
    if missing:
        sys.exit(f"config 에 항목이 없습니다: {missing} ({path})")
    for g in ("상", "중"):
        if g not in cfg["boxes"]:
            sys.exit(f"config boxes 에 '{g}' 구역이 없습니다")
    return cfg


def resolve_boxes(cfg, sim):
    """boxes 의 놓는 위치를 로봇 좌표(xy_m)로 확정. xy_m > px(+calib.yaml) > xy_m_sim(SIM 만)."""
    from vision import Calib, calib_path
    p = calib_path(cfg)
    calib = Calib.load(p) if os.path.exists(p) else None
    if calib is not None and calib.info.get("sim") and not sim:
        calib = None
    for g, b in cfg["boxes"].items():
        if "xy_m" in b:
            src = "xy_m"
        elif "px" in b and calib is not None:
            b["xy_m"] = [round(float(v), 4) for v in calib.px2xy(*b["px"])]
            src = f"px {b['px']} → calib"
        elif sim and "xy_m_sim" in b:
            b["xy_m"] = list(b["xy_m_sim"])
            src = "xy_m_sim"
        else:
            raise RobotError(f"'{g}' 구역 좌표를 정할 수 없습니다 — calib.yaml 이 없으면 boxes.{g}.xy_m 을 직접 넣으세요\n"
                             "  (먼저 python3 calibrate.py)")
        log.info("놓는 곳 '%s': (%.3f, %.3f) [%s]", g, *b["xy_m"], src)


# ---------- 외부 신호 (사과 위치 / 등급 / 굴림) ----------
# 카메라 팀원 연동은 아래 세 메서드를 가진 클래스를 하나 더 만들면 된다.

class SimSignals:
    def __init__(self, cfg):
        self.rng = random.Random(cfg["sim"]["seed"] + 1)
        self.apples = [tuple(p) for p in cfg["sim"]["apples_xy_m"]]
        self.roll_prob = cfg["sim"]["roll_prob"]

    def apple_xy(self, i):
        return self.apples[i] if i < len(self.apples) else None

    def grade(self):
        g = self.rng.choice(["상", "중"])
        log.info("[SIM] 카메라 등급 판정: %s", g)
        return g

    def rolled(self, grade=None):
        r = self.rng.random() < self.roll_prob
        log.info("[SIM] 굴림 감지: %s", "있음" if r else "없음")
        return r


def _ask(prompt, timeout_s):
    """터미널 입력. timeout_s 안에 입력 없으면 None (프로그램이 매달리지 않게)."""
    print(f"{prompt} ({timeout_s:.0f}초) > ", end="", flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not ready:
        print("(시간 초과)")
        return None
    return sys.stdin.readline().strip()


class ManualSignals:
    """카메라 연동 전 현장 테스트용: 운영자가 터미널에 입력."""

    def __init__(self, cfg):
        self.defaults = [tuple(p) for p in cfg["sim"]["apples_xy_m"]]

    def apple_xy(self, i):
        s = _ask(f"사과 {i + 1} 좌표 'x y'[m] (Enter=기본값, q=종료)", 60)
        if s == "q":
            return None
        if s:
            try:
                x, y = map(float, s.replace(",", " ").split())
                return x, y
            except ValueError:
                log.warning("좌표 형식 오류 '%s' → 기본값 사용", s)
        return self.defaults[i] if i < len(self.defaults) else None

    def grade(self):
        s = _ask("등급 입력 (1=상, 2=중)", 20)
        return {"1": "상", "상": "상", "2": "중", "중": "중"}.get(s or "")

    def rolled(self, grade=None):
        s = _ask("굴림 있었나? (y/N)", 10)
        return (s or "").lower() in ("y", "yes", "ㅛ")


# ---------- 시퀀스 ----------

def best_jaw_yaw(x, y, apple_r, open_w, finger_w, obstacles, walls):
    """집게 두 손가락이 내려앉을 자리의 여유가 가장 큰 방향(rad)과 그 여유(m).
    obstacles: [(x, y, r)] 이웃 사과, walls: [(x1, y1, x2, y2)] 트레이 안쪽 벽 선분."""
    def seg_dist(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy or 1e-12)))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))
    best = None
    d = open_w / 2 + finger_w / 2            # 손가락 중심까지 거리
    for k in range(36):
        th = math.pi * k / 36
        ux, uy = math.cos(th), math.sin(th)
        clear = 1.0
        for sgn in (1, -1):
            fx, fy = x + sgn * d * ux, y + sgn * d * uy
            for ox, oy, orr in obstacles:
                clear = min(clear, math.hypot(fx - ox, fy - oy) - orr - finger_w / 2)
            for w in walls:
                clear = min(clear, seg_dist(fx, fy, *w) - finger_w / 2)
        if best is None or clear > best[1]:
            best = (th, clear)
    return best


class Mission:
    def __init__(self, robot, cfg, tuner, signals, dash=None, roll=None):
        self.r, self.cfg, self.tuner, self.sig = robot, cfg, tuner, signals
        self.dash = dash or Dashboard({})
        self.placed = {}      # 칸별 놓은 개수
        self.roll = roll      # 굴림 카메라 (vision.BoxRollWatcher). 없으면 signals.rolled()

    def pick(self, x, y):
        """사과 위 접근 → 하강 → 파지 → 상승. 쥐었으면 True."""
        p, c = self.cfg["pick"], self.cfg
        ox, oy = p.get("grasp_offset_m", [0.0, 0.0])   # 잡는 위치 미세 보정 (로봇 좌표, +x = Sony 화면 아래쪽)
        ax, ay = x, y                                   # 사과 실제 중심 (집게 여유 계산 기준)
        x, y = x + ox, y + oy
        ctx = getattr(self.sig, "grasp_context", lambda: None)()
        self.r.jaw_yaw = None
        g = c["gripper"]
        open_w = float(g["open_width_m"])
        if ctx:   # 사과 크기를 알면 필요한 만큼만 연다 (지름 + 여유)
            open_w = min(open_w, 2 * ctx["r"] + float(g.get("open_margin_m", 0.02)))
        if ctx and g.get("auto_jaw", True):
            th, clear = best_jaw_yaw(ax, ay, ctx["r"], open_w, g["finger_width_m"],
                                     ctx["others"], ctx["walls"])
            self.r.jaw_yaw = th
            (log.warning if clear < 0.005 else log.info)(
                "집게 방향 %.0f° (손가락 여유 %.0fmm%s)", math.degrees(th), clear * 1000,
                " — 벽/이웃 사과에 닿을 수 있음" if clear < 0 else "")
        z_lift = self.r.lift_z(x, y)                                  # 멀면 낮아진다
        z_app = min(p["tray_z_m"] + p["approach_height_m"], z_lift)
        z_grasp = p["tray_z_m"] + p["grasp_height_m"]
        log.info("== pick (%.3f, %.3f)", x, y)
        self.dash.mission("phase", phase="pick")
        self.r.transit_to(x, y, c["motion"]["transit_speed_pct"])
        self.r.grip(True, width=open_w)
        self.r.down_to(x, y, z_app, c["motion"]["transit_speed_pct"])
        self.r.down_to(x, y, z_grasp, self.tuner.speed(p["descend_speed_pct"]))
        w = self.r.grip(False)
        min_w = c["gripper"]["min_grasp_width_m"]
        ok = w >= min_w
        self.r.down_to(x, y, z_lift, p["lift_speed_pct"])
        if ok and self.r.gripper_width() < min_w:
            log.warning("상승 중 사과를 놓쳤습니다")
            ok = False
        log.info("파지 %s (폭 %.1fmm)", "성공" if ok else "실패", w * 1000)
        self.dash.mission("pick", ok=bool(ok), attempt=getattr(self, "_attempt", 0), width_mm=round(w * 1000, 1))
        return ok

    def inspect(self):
        """검사 위치로 이동 후 카메라 촬영 대기. 등급 반환."""
        i = self.cfg["inspect"]
        x, y = i["xy_m"]
        log.info("== inspect")
        self.dash.mission("phase", phase="inspect")
        self.r.transit_to(x, y, self.cfg["motion"]["transit_speed_pct"])
        self.r.wait(i["wait_s"])
        g = self.sig.grade()
        if g not in ("상", "중"):
            log.warning("등급 신호 없음/오류 (%r) → 기본 등급 %s", g, i["default_grade"])
            g = i["default_grade"]
        return g

    def place(self, grade):
        """등급 상자 위 → 하강 → 놓기 → 상승."""
        b, p = self.cfg["boxes"][grade], self.cfg["place"]
        # 칸 안 자리: 칸마다 놓은 개수만큼 다음 자리로 (같은 자리에 겹쳐 놓으면 먼저 놓은 사과를 밀어낸다)
        slots = p.get("slots", [{"dxy_m": [0.0, 0.0], "dz_m": 0.0}])
        k = self.placed.get(grade, 0)
        slot = slots[min(k, len(slots) - 1)]
        self.placed[grade] = k + 1
        bx, by = b["xy_m"][0] + slot["dxy_m"][0], b["xy_m"][1] + slot["dxy_m"][1]
        if p.get("place_jaw_yaw_deg") is not None:   # 칸 안에서는 손가락이 이웃 사과 쪽으로 안 가게
            self.r.jaw_yaw = math.radians(p["place_jaw_yaw_deg"])
        log.info("'%s' 칸 %d번째 자리 (%.3f, %.3f)%s", grade, k + 1, bx, by,
                 " — 위에 얹음 +%.0fmm" % (slot["dz_m"] * 1000) if slot["dz_m"] else "")
        z_lift = self.r.lift_z(bx, by)
        z_rel = b["floor_z_m"] + self.tuner.release_h + slot["dz_m"]
        spd = self.tuner.speed(p["descend_speed_pct"])
        log.info("== place '%s' 상자 (놓는 높이 %.3fm, 하강 %.0f%%)", grade, self.tuner.release_h, spd)
        self.dash.mission("phase", phase="place")
        self.r.transit_to(bx, by, self.cfg["motion"]["transit_speed_pct"])
        self.r.down_to(bx, by, z_rel, spd)
        if self.roll:
            self.roll.release()              # 쥔 사과 위치 기억 + 관찰 시작
        self.r.grip(True)
        self.r.wait(p["release_settle_s"])   # 팔 정지 상태로 관찰
        if self.roll and self.cfg["roll_camera"].get("mode") == "wrist":
            self.roll.stop()                 # 팔이 올라가기 전에 관찰 끝
        self.r.down_to(bx, by, z_lift, p["lift_speed_pct"])

    def run(self):
        n = self.cfg["mission"]["apple_count"]
        retries = self.cfg["pick"]["retries"]
        results = []
        t0 = time.monotonic()
        self.dash.mission("start", apple_count=n, sim=bool(getattr(self.r, "sim", False)))
        self.r.go_home()
        clear = getattr(self.sig, "needs_clear_view", False)   # 카메라: 찍기 전에 팔을 시야 밖(홈)으로
        for i in range(n):
            if clear and i:
                self.r.go_home()
            xy = self.sig.apple_xy(i)
            if xy is None:
                log.info("사과 좌표 없음 → 종료")
                break
            log.info("######## 사과 %d/%d ########", i + 1, n)
            self.dash.mission("apple", index=i + 1, total=n)
            ok = False
            for attempt in range(1 + retries):
                self._attempt = attempt
                if attempt:
                    log.warning("재시도 %d/%d", attempt, retries)
                try:
                    ok = self.pick(*xy)
                except (RobotFault, MotionTimeout):
                    raise
                except RobotError as e:   # 관절 해 없음·작업영역 밖 → 이 사과만 건너뛴다
                    log.error("사과 %d 건너뜀: %s", i + 1, e)
                    self.dash.mission("skip", index=i + 1, reason=str(e))
                    if hasattr(self.sig, "mark_bad"):
                        self.sig.mark_bad()   # 다음 검출에서 이 사과는 고르지 않는다
                    self.r.go_home()
                    break
                if ok:
                    break
                self.r.grip(True)
                if clear and attempt < retries:   # 재시도는 다시 찍어서 (사과가 밀렸을 수 있다)
                    self.r.go_home()
                    xy = self.sig.apple_xy(i) or xy
            if not ok:
                self.r.jaw_yaw = None
                if hasattr(self.sig, "mark_bad"):
                    self.sig.mark_bad()
                results.append((i + 1, "파지 실패", "-"))
                self.dash.mission("skip", index=i + 1, reason="파지 실패")
                continue
            grade = self.inspect()
            self.dash.judge(grade, getattr(self.sig, "judge_info", lambda: None)())
            self.place(grade)
            if clear:
                self.r.go_home()
            self.r.jaw_yaw = None
            rolled = self.roll.verdict() if self.roll else self.sig.rolled(grade)
            # 이번 놓기에 실제로 쓴 값 (tuner 갱신 전)
            self.dash.motion(self.tuner.scale, self.tuner.release_h, rolled)
            if rolled:
                self.tuner.on_roll()
            else:
                self.tuner.on_success()
            self.dash.mission("adaptive", scale=round(self.tuner.scale, 3), release_h=round(self.tuner.release_h, 4),
                              frozen=self.tuner.frozen, down_streak=self.tuner.down_streak)
            self.dash.mission("phase", phase="home")
            results.append((i + 1, grade, "굴림" if rolled else "OK"))
        self.r.go_home()

        log.info("======== 결과 (%.1fs) ========", time.monotonic() - t0)
        self.dash.mission("end", duration_s=round(time.monotonic() - t0, 1),
                          results=[{"index": idx, "grade": g, "note": note} for idx, g, note in results])
        for idx, grade, note in results:
            log.info("  사과 %d: %s  %s", idx, grade, note)
        log.info("  최종 속도 배율 %.2f, 놓는 높이 %.3fm%s", self.tuner.scale, self.tuner.release_h,
                 " (자동 조정 중단됨)" if self.tuner.frozen else "")
        return results


def make_signals(kind, sim, cfg):
    if kind is None:
        from vision import calib_path
        kind = "sim" if sim else ("camera" if os.path.exists(calib_path(cfg)) else "manual")
        if kind == "manual":
            log.warning("calib.yaml 없음 → 수동 입력 모드. 카메라를 쓰려면 먼저 python3 calibrate.py")
    log.info("신호 소스: %s", kind)
    if kind == "camera":
        from vision import CameraSignals
        sig = CameraSignals(cfg)
        if sig.calib.info.get("sim") and not sim:
            sig.close()
            raise RobotError("calib.yaml 이 SIM 으로 만든 가짜입니다. 실기에서 python3 calibrate.py 를 다시 돌리세요")
        return sig
    return (SimSignals if kind == "sim" else ManualSignals)(cfg)


def dry_run(robot, cfg, speed_pct):
    """사과 없이 실제 경로를 천천히, 그리퍼까지: 홈 → (트레이에서 집는 척 → 칸에 놓는 척) × 상·중 → 홈.
    단계마다 Enter (q=중단)."""
    p, pl = cfg["pick"], cfg["place"]
    (x1, y1), (x2, y2) = cfg["calibration"]["corner1_xy_m"], cfg["calibration"]["corner2_xy_m"]
    tx, ty = (x1 + x2) / 2, (y1 + y2) / 2
    T = "transit"
    steps = [("홈 (상자 쪽 높은 곳)", "home", None)]
    G = "grip"
    for g in ("상", "중"):    # 칸마다 한 번씩 집는 척 → 놓는 척
        b = cfg["boxes"][g]
        bx, by = b["xy_m"]
        steps += [("트레이 가운데 위 (이동 높이, 약간 기울어짐)", T, (tx, ty)),
                  ("그리퍼 열기", G, True),
                  ("트레이 접근 높이 (수직)", "move", (tx, ty, min(p["tray_z_m"] + p["approach_height_m"], robot.lift_z(tx, ty)))),
                  ("트레이 파지 높이 (손가락 끝이 바닥 위 3cm)", "move", (tx, ty, p["tray_z_m"] + p["grasp_height_m"])),
                  ("그리퍼 닫기 (집는 척)", G, False),
                  ("수직 상승", "move", (tx, ty, robot.lift_z(tx, ty))),
                  (f"'{g}' 칸 위 (이동 높이) — 벽을 넘는다", T, (bx, by)),
                  (f"'{g}' 칸 놓는 높이 (수직)", "move", (bx, by, b["floor_z_m"] + pl["release_height_m"])),
                  ("그리퍼 열기 (놓는 척)", G, True),
                  (f"'{g}' 칸 수직 상승", "move", (bx, by, robot.lift_z(bx, by))),
                  ("그리퍼 닫기", G, False)]
    steps.append(("홈", "home", None))
    print(f"드라이런 {len(steps)}단계, 속도 {speed_pct}%. 비상정지 버튼에 손 올려두세요. Ctrl+C = 비상정지")
    for i, (name, kind, arg) in enumerate(steps, 1):
        where = ' ' + str(tuple(round(v, 3) for v in arg)) if isinstance(arg, tuple) else ''
        s = _ask(f"[{i}/{len(steps)}] {name}{where} — Enter=실행, q=중단", 120)
        if s is None or s == "q":
            log.info("드라이런 중단")
            return
        if kind == G:
            w = robot.grip(arg)
            log.info("  그리퍼 폭 %.1fmm", w * 1000)
            continue
        if kind == "home":
            robot.go_home(speed_pct)
        elif kind == T:
            robot.transit_to(*arg, speed_pct)
        else:
            robot.down_to(*arg, speed_pct)
        x, y, z, rx, ry, rz = robot.current_pose()
        log.info("  도착 플랜지 (%.3f, %.3f, %.3f)", x, y, z)
    log.info("드라이런 완료")


def jaw_test(robot, cfg, speed_pct):
    """집게 방향 설정(gripper.jaw_axis_tool) 확인: 트레이 가운데 위에서 '로봇 x축(= Sony 화면 위아래)' 방향으로 벌린다."""
    (x1, y1), (x2, y2) = cfg["calibration"]["corner1_xy_m"], cfg["calibration"]["corner2_xy_m"]
    tx, ty = (x1 + x2) / 2, (y1 + y2) / 2
    p = cfg["pick"]
    robot.transit_to(tx, ty, speed_pct)
    robot.jaw_yaw = 0.0
    robot.down_to(tx, ty, p["tray_z_m"] + p["approach_height_m"], speed_pct)
    robot.grip(True)
    s = _ask("Sony 화면 기준으로 집게 두 손가락이 어디에 있나?  1 = 위·아래   2 = 왼쪽·오른쪽", 120)
    robot.grip(False)
    robot.jaw_yaw = None
    robot.go_home(speed_pct)
    cur = cfg["gripper"].get("jaw_axis_tool", "y")
    if s == "1":
        log.info("설정 맞음 (jaw_axis_tool: %s)", cur)
    elif s == "2":
        log.warning("설정 반대 → config.yaml gripper.jaw_axis_tool 을 %s 로 바꾸세요", "x" if cur == "y" else "y")
    else:
        log.warning("응답 없음 — 확인 못 함")


def read_pose_loop(robot):
    print("현재 좌표 (Ctrl+C 종료). 이 값을 config.yaml 에 복사하세요.")
    while True:
        x, y, z, rx, ry, rz = robot.current_pose()
        j = robot.current_joints()
        print(f"xyz_m: [{x:.3f}, {y:.3f}, {z:.3f}]  rpy_rad: [{rx:.4f}, {ry:.4f}, {rz:.4f}]  "
              f"joints_rad: [{', '.join(f'{v:.3f}' for v in j)}]  grip: {robot.gripper_width() * 1000:.1f}mm")
        time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.environ.get("CONFIG", os.path.join(HERE, "config.yaml")))
    ap.add_argument("--check", action="store_true", help="연결 점검만")
    ap.add_argument("--read-pose", action="store_true", help="현재 좌표 출력")
    ap.add_argument("--home", action="store_true", help="홈 자세로 이동만")
    ap.add_argument("--resume", action="store_true", help="비상정지 해제")
    ap.add_argument("--exit-teach", action="store_true", help="티칭(드래그) 모드 해제 — 팔이 그 자리에 선다")
    ap.add_argument("--dry-run", action="store_true", help="사과 없이 경로를 천천히 한 단계씩")
    ap.add_argument("--jaw-test", action="store_true", help="집게 방향 설정 확인 (트레이 위에서 한 번 벌려 본다)")
    ap.add_argument("--speed", type=float, default=15, help="--dry-run 속도 %% (기본 15)")
    ap.add_argument("--signals", choices=["sim", "manual", "camera"],
                    help="신호 소스 (기본: SIM이면 sim, 아니면 calib.yaml 있으면 camera, 없으면 manual)")
    ap.add_argument("--dashboard", choices=["on", "off"],
                    help="대시보드 전송 (기본: config dashboard.enabled, 단 SIM 이면 off — 실제 통계 오염 방지)")
    ap.add_argument("-v", "--verbose", action="store_true", help="SDK 호출까지 로그")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s.%(msecs)03d %(levelname).1s [%(name)s] %(message)s",
                        datefmt="%H:%M:%S")
    sim = os.environ.get("SIM", "false").lower() in ("1", "true", "yes")
    cfg = load_config(args.config)
    robot = Robot(cfg, sim=sim)

    try:
        if not (args.resume or args.check or args.read_pose or args.home):
            resolve_boxes(cfg, sim)   # --dry-run 도 상자 좌표가 필요
        if args.resume:
            _resume(robot, sim)
            return 0
        if args.exit_teach:
            robot.connect(enable=False)
            robot.exit_teach()
            return 0
        robot.connect(enable=not args.read_pose)
        if args.check:
            log.info("점검 완료: 현재 xyz %s", tuple(round(v, 3) for v in robot.current_pose()[:3]))
            return 0
        if args.read_pose:
            read_pose_loop(robot)
            return 0
        if args.home:
            robot.go_home()
            return 0
        if args.dry_run:
            dry_run(robot, cfg, args.speed)
            return 0
        if args.jaw_test:
            jaw_test(robot, cfg, args.speed)
            return 0
        signals = make_signals(args.signals, sim, cfg)
        on = args.dashboard == "on" if args.dashboard else (bool(cfg.get("dashboard", {}).get("enabled")) and not sim)
        dash = Dashboard({**cfg, "dashboard": {**cfg.get("dashboard", {}), "enabled": on}})
        roll = None
        if not sim:
            from vision import open_roll_watcher
            roll = open_roll_watcher(cfg)
        try:
            Mission(robot, cfg, AdaptiveTuner(cfg), signals, dash, roll).run()
        except KeyboardInterrupt:
            dash.mission("estop", reason="Ctrl+C 비상정지")
            raise
        except RobotFault as e:
            dash.mission("estop", reason=f"로봇 고장: {e}")
            raise
        except (RobotError, IOError) as e:
            dash.mission("estop", reason=f"정지: {e}")
            raise
        finally:
            dash.flush()
            if roll:
                roll.close()
            if hasattr(signals, "close"):
                signals.close()
        return 0
    except KeyboardInterrupt:
        if args.read_pose:
            return 0
        log.error("Ctrl+C → 비상정지")
        robot.estop()
        return 130
    except RobotFault as e:
        log.error("로봇 고장: %s", e)
        robot.estop()
        return 2
    except (RobotError, IOError) as e:   # IOError: 카메라 프레임 없음, 캘리브레이션 파일 없음
        log.error("%s", e)
        robot.hold()
        return 1
    finally:
        robot.disconnect()


def _resume(robot, sim):
    """비상정지 상태에서는 connect() 가 거부하므로 피드백 확인 없이 바로 해제 명령을 보낸다."""
    if not sim:
        print("주의: 해제 순간 모터 전원이 잠깐 빠져 팔이 처질 수 있습니다. 팔을 손으로 받치세요.")
        if (_ask("계속? (y/N)", 30) or "").lower() != "y":
            print("취소")
            return
    try:
        robot.connect(enable=False)
    except RobotFault:
        pass   # 비상정지 상태라 거부된 것 — 연결 자체는 됐다
    robot.resume()
    robot.connect(enable=True)
    log.info("해제 완료. 상태 정상")


if __name__ == "__main__":
    sys.exit(main())
