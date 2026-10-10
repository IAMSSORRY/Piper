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
import itertools
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
from piper_robot import ForceStop, MotionTimeout, Robot, RobotError, RobotFault, SoftStop

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
    for g in grade_labels(cfg):
        if g not in cfg["boxes"]:
            sys.exit(f"config boxes 에 '{g}' 구역이 없습니다")
    return cfg


def resolve_apple_count(cfg, arg=None):
    """미션 사과 개수. --apples > 환경변수 MISSION_APPLE_COUNT > config mission.apple_count.

    0 / all / auto (또는 config 의 null) 이면 None = 트레이에 사과가 없을 때까지 계속.
    """
    for src, raw in (("--apples", arg), ("MISSION_APPLE_COUNT", os.environ.get("MISSION_APPLE_COUNT")),
                     ("config mission.apple_count", cfg.get("mission", {}).get("apple_count"))):
        if raw is None or str(raw).strip() == "":
            continue
        text = str(raw).strip().lower()
        if text in ("0", "all", "auto", "none", "null"):
            log.info("사과 개수: 없을 때까지 [%s]", src)
            return None
        try:
            n = int(text)
        except ValueError:
            raise SystemExit(f"사과 개수가 숫자가 아닙니다: {src}={raw!r} (숫자, 또는 0/all = 없을 때까지)")
        if n < 0:
            raise SystemExit(f"사과 개수는 0 이상이어야 합니다: {src}={raw!r}")
        log.info("사과 개수: %d [%s]", n, src)
        return n
    return None


def grade_labels(cfg):
    """[높은 등급, 낮은 등급] — 빨강 → 앞, 노랑 → 뒤. 지금은 상 / 하 (상자 상·중·하 중 두 칸만 쓴다)."""
    return list(cfg["vision"]["grade"].get("labels", ["상", "중"]))


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
        self.labels = grade_labels(cfg)
        self.rng = random.Random(cfg["sim"]["seed"] + 1)
        self.apples = [tuple(p) for p in cfg["sim"]["apples_xy_m"]]
        self.roll_prob = cfg["sim"]["roll_prob"]

    def apple_xy(self, i):
        return self.apples[i] if i < len(self.apples) else None

    def grade(self):
        g = self.rng.choice(self.labels)
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
        self.labels = grade_labels(cfg)
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
        hi, lo = self.labels
        s = _ask(f"등급 입력 (1={hi}, 2={lo})", 20)
        return {"1": hi, hi: hi, "2": lo, lo: lo}.get(s or "")

    def rolled(self, grade=None):
        s = _ask("굴림 있었나? (y/N)", 10)
        return (s or "").lower() in ("y", "yes", "ㅛ")


# ---------- 시퀀스 ----------

class CompartmentFull(RobotError):
    """놓을 칸에 빈 자리가 없다 — 사과를 집은 자리에 되돌리고 다음 사과로."""


def _seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy or 1e-12)))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def jaw_clearance(th, x, y, open_w, finger_w, obstacles, walls):
    """집게가 th(rad) 방향으로 닫힐 때 두 손가락이 내려앉을 자리의 여유(m). 음수 = 벽/이웃 사과와 겹침."""
    d = open_w / 2 + finger_w / 2            # 손가락 중심까지 거리
    ux, uy = math.cos(th), math.sin(th)
    clear = 1.0
    for sgn in (1, -1):
        fx, fy = x + sgn * d * ux, y + sgn * d * uy
        for ox, oy, orr in obstacles:
            clear = min(clear, math.hypot(fx - ox, fy - oy) - orr - finger_w / 2)
        for w in walls:
            clear = min(clear, _seg_dist(fx, fy, *w) - finger_w / 2)
    return clear


def best_jaw_yaw(x, y, apple_r, open_w, finger_w, obstacles, walls):
    """집게 두 손가락이 내려앉을 자리의 여유가 가장 큰 방향(rad)과 그 여유(m).
    obstacles: [(x, y, r)] 이웃 사과, walls: [(x1, y1, x2, y2)] 트레이 안쪽 벽 선분."""
    best = None
    for k in range(36):
        th = math.pi * k / 36
        clear = jaw_clearance(th, x, y, open_w, finger_w, obstacles, walls)
        if best is None or clear > best[1]:
            best = (th, clear)
    return best


def wall_jaw_yaw(x, y, apple_r, open_w, finger_w, obstacles, walls, near_m, min_clear, corner_m=None):
    """벽에 붙은 사과의 집게 방향 — 벽을 보고 정한다. (방향 rad, 여유 m, 설명) 또는 None(가까운 벽 없음 / 안 됨).

    - 벽 하나에 붙음(가로벽 / 세로벽): 집게가 벽과 나란히 닫히게 → 손가락이 사과 양옆(벽을 따라)으로 내려앉는다.
      가로벽이면 손가락이 세로로 서고, 세로벽이면 가로로 선다. 벽과 사과 사이에 손가락을 넣지 않는다
    - 모서리(두 벽)거나, 벽과 나란히는 이웃 사과에 걸리면: 대각선(벽에서 ±45°) 중 자리가 있는 쪽
    - 어느 것도 min_clear 이상 여유가 없으면 None (여유가 가장 큰 방향으로 잡는다)"""
    def clr(th):
        return jaw_clearance(th, x, y, open_w, finger_w, obstacles, walls)

    # 모서리 먼저: 트레이 꼭짓점이 가까우면(사과가 두 벽에 다 붙지 않아 보여도) 꼭짓점을 가리키는 대각선으로 잡는다.
    # 벽 선은 사진에서 테두리 안쪽으로 짐작한 것이라 몇 cm 틀릴 수 있다 — 벽까지 거리보다 꼭짓점까지 거리가 확실하다
    # 좌상·우상·좌하·우하 모서리: 집게가 트레이의 대각선(그 꼭짓점 ↔ 맞은편 꼭짓점) 방향으로 벌어지게 잡는다
    if corner_m is not None and walls:
        verts = [(w[0], w[1]) for w in walls]
        k = min(range(len(verts)), key=lambda i: math.hypot(verts[i][0] - x, verts[i][1] - y))
        vx, vy = verts[k]
        if math.hypot(vx - x, vy - y) < apple_r * math.sqrt(2) + corner_m:
            ox, oy = verts[(k + len(verts) // 2) % len(verts)]   # 맞은편 꼭짓점
            th = math.atan2(oy - vy, ox - vx) % math.pi
            return th, clr(th), "대각선 (모서리)"
    near = [w for w in walls if _seg_dist(x, y, *w) < apple_r + near_m]
    if not near:
        return None
    dirs = []                                # 가까운 벽들의 방향 (0~180°, 거의 나란하면 하나로)
    for x1, y1, x2, y2 in sorted(near, key=lambda w: _seg_dist(x, y, *w)):
        a = math.atan2(y2 - y1, x2 - x1) % math.pi
        if all(abs((a - b + math.pi / 2) % math.pi - math.pi / 2) > math.radians(20) for b in dirs):
            dirs.append(a)
    base = dirs[0]                           # 가장 가까운 벽 방향

    if len(dirs) > 1:
        # 모서리: 집게를 모서리를 가리키는 대각선으로 — 한 손가락은 모서리 쪽 삼각형 틈에, 다른 손가락은 트레이 안쪽에.
        # 둥근 사과와 모서리 사이 틈은 손가락 원 모델보다 넓다 (판 모양 손가락이 모서리 삼각형에 들어간다) → 여유와 상관없이 이 방향
        vx = vy = 0.0
        for x1, y1, x2, y2 in near:              # 사과 → 각 벽의 가장 가까운 점 (= 모서리 쪽)
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy or 1e-12)))
            px, py = x1 + t * dx - x, y1 + t * dy - y
            n = math.hypot(px, py) or 1.0
            vx, vy = vx + px / n, vy + py / n
        th = math.atan2(vy, vx) % math.pi
        return th, clr(th), "대각선 (모서리)"
    c = clr(base)
    if c >= min_clear:
        return base, c, "벽과 나란히"
    diag = max((base + math.pi / 4, base - math.pi / 4), key=clr)
    c = clr(diag)
    if c >= min_clear:
        return diag, c, "대각선"
    return None


class Mission:
    def __init__(self, robot, cfg, tuner, signals, dash=None, roll=None):
        self.r, self.cfg, self.tuner, self.sig = robot, cfg, tuner, signals
        self.dash = dash or Dashboard({})
        self.placed = {}      # 칸별 놓은 개수
        self.used_slots = {}  # 칸별 이번 미션에 직접 놓은 자리 번호 (사진에 안 보여도 그 위에는 안 놓는다)
        self.results, self.next_i, self.stop_requested, self.t0 = [], 0, False, time.monotonic()
        self.last_pick = None
        self.last_drop = None  # 운반 중 낙하 표시
        self.roll = roll      # 굴림 카메라 (vision.BoxRollWatcher). 없으면 signals.rolled()

    def pick(self, x, y):
        """사과 위 접근 → 하강 → 파지 → 상승. 쥐었으면 True."""
        p, c = self.cfg["pick"], self.cfg
        ox, oy = p.get("grasp_offset_m", [0.0, 0.0])   # 잡는 위치 미세 보정 (로봇 좌표, +x = Sony 화면 아래쪽)
        ax, ay = x, y                                   # 사과 실제 중심 (집게 여유 계산 기준)
        x, y = x + ox, y + oy
        ctx = getattr(self.sig, "grasp_context", lambda: None)()
        self.r.jaw_yaw = None
        corner = False
        g = c["gripper"]
        open_w = float(g["open_width_m"])
        if ctx:   # 사과 크기를 알면 필요한 만큼만 연다 (지름 + 여유)
            open_w = min(open_w, 2 * ctx["r"] + float(g.get("open_margin_m", 0.02)))
        if ctx and g.get("auto_jaw", True):
            # 벽에 붙었으면 벽을 보고 돌려 잡는다: 벽과 나란히(가로벽 → 세로, 세로벽 → 가로), 모서리·애매하면 대각선
            wj = wall_jaw_yaw(ax, ay, ctx["r"], open_w, g["finger_width_m"], ctx["others"], ctx["walls"],
                              float(g.get("wall_near_m", 0.015)), float(g.get("wall_min_clear_m", 0.003)),
                              float(g.get("corner_near_m", 0.04)))
            if wj and "모서리" in wj[2]:
                # 모서리: 사과 지름 + 조금만 벌려 모서리 쪽 손가락이 틈에 들어가게. 내려가다 닿으면 거기서 멈추고 잡는다
                open_w = min(open_w, 2 * ctx["r"] + float(g.get("corner_open_margin_m", 0.006)))
                corner = True
                th, clear, how = wj
                log.warning("모서리 사과 — 집게를 대각선 %.0f° 로, %.0fmm 만 벌려 잡는다", math.degrees(th), open_w * 1000)
            elif wj:
                th, clear, how = wj
                log.info("벽에 붙은 사과 — 집게를 %s으로 돌려 잡는다", how)
            else:
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
        self.r.down_to(x, y, z_app, c["motion"]["approach_speed_pct"])
        if corner:   # 모서리: 손가락이 벽 위·사과 위에 걸리면 누르지 말고 거기서 멈춰 잡는다
            how_d, _ = self.r.guarded_down(x, y, z_grasp, self.tuner.speed(p["descend_speed_pct"]), p.get("contact_nm", 0.5))
            if how_d == "contact":
                log.warning("모서리 잡기 하강 중 닿음 — 그 높이에서 잡는다")
        else:
            self.r.down_to(x, y, z_grasp, self.tuner.speed(p["descend_speed_pct"]))   # 잡기 직전만 느리게
        w = self.r.grip(False)
        min_w = c["gripper"]["min_grasp_width_m"]
        ok = w >= min_w
        self.r.down_to(x, y, z_lift, p["lift_speed_pct"])
        if ok and self.r.gripper_width() < min_w:
            log.warning("상승 중 사과를 놓쳤습니다")
            ok = False
        self.last_pick = (x, y, self.r.jaw_yaw) if ok else None   # 비상정지 때 이 자리에 되돌려 놓는다
        log.info("파지 %s (폭 %.1fmm)", "성공" if ok else "실패", w * 1000)
        self.dash.mission("pick", ok=bool(ok), attempt=getattr(self, "_attempt", 0), width_mm=round(w * 1000, 1))
        return ok

    def inspect(self):
        """검사: 'rotate' 면 사과를 들어 위 카메라에 비추고 손목(joint6)을 돌려 가며 멍을 본다. 등급 반환."""
        i = self.cfg["inspect"]
        log.info("== inspect")
        self.dash.mission("phase", phase="inspect")
        low = grade_labels(self.cfg)[1]
        if i.get("mode") == "rotate" and hasattr(self.sig, "inspect_frame") and self.sig.grade() == low:
            log.info("노란 사과 → %s (회전 검사 생략)", low)   # 멍과 상관없이 낮은 등급
            g = self.sig.final_grade()
        elif i.get("mode") == "rotate" and hasattr(self.sig, "inspect_frame"):
            q = [float(v) for v in i["joints_rad"]]
            a, b = (float(v) for v in i["sweep_j6_rad"])
            cur6 = self.r.current_joints()[5]
            if abs(cur6 - b) < abs(cur6 - a):     # 지금 손목에서 가까운 끝부터
                a, b = b, a
            self.sig.bruises = []
            self.r.move_joints(q[:5] + [a], self.cfg["motion"]["transit_speed_pct"])
            last = [0.0]
            period = i.get("frame_period_s", 0.12)

            def snap():                           # 돌리는 동안 일정 간격으로 찍는다 (멈추지 않는다)
                now = time.monotonic()
                if now - last[0] >= period:
                    last[0] = now
                    self.sig.inspect_frame(f"{len(self.sig.bruises)}")

            snap()
            self.r.move_joints(q[:5] + [b], i.get("sweep_speed_pct", 40), tick=snap)
            g = self.sig.final_grade()
        else:
            x, y = i["xy_m"]
            self.r.transit_to(x, y, self.cfg["motion"]["transit_speed_pct"])
            self.r.wait(i["wait_s"])
            g = self.sig.grade()
        if g not in grade_labels(self.cfg) and g != self.cfg["inspect"].get("bruise", {}).get("label", "중"):
            log.warning("등급 신호 없음/오류 (%r) → 기본 등급 %s", g, i["default_grade"])
            g = i["default_grade"]
        return g

    def _free_slot(self, grade, slots):
        """칸 안 빈 자리 번호 (slots 순서대로 첫 빈 자리). 사과 위에 얹지 않는다.

        찬 자리 = (1) 위 카메라 사진에서 그 칸 사과가 가장 가까운 자리 (2) 이번 미션에 로봇이 직접 놓은 자리.
        둘을 합친다 — 홈 자세에서 팔이 상자를 가려 사진에 안 보여도 자기가 놓은 사과 위에는 놓지 않는다.
        빈 자리가 없으면 RobotError (칸을 비워야 한다)."""
        taken = set(self.used_slots.get(grade, set()))
        seen = None
        f = getattr(self.sig, "box_apples", None)
        if f is not None:
            try:
                seen = f(grade)
            except Exception as e:   # 사진 없음 등 — 직접 놓은 기록만 쓴다
                log.warning("상자 사진으로 빈 자리 찾기 실패 → 놓은 기록만 씀: %s", e)
        if seen:
            cents = {g: bb["xy_m"] for g, bb in self.cfg["boxes"].items()}
            bx, by = cents[grade]
            pts = [(bx + sl["dxy_m"][0], by + sl["dxy_m"][1]) for sl in slots]
            near = self.cfg["place"].get("slot_taken_m", 0.07)
            for x, y in seen:
                # 다른 칸 사과는 빼고(가장 가까운 칸 중심이 이 칸인 것만), 자리에서 사과 지름 안에 있으면 그 자리는 참
                # (가운데로 굴러간 사과는 양쪽 자리를 다 막는다 — 가장 가까운 자리 하나만 막으면 그 사과에 걸쳐 놓는다)
                if min(cents, key=lambda g: math.hypot(x - cents[g][0], y - cents[g][1])) != grade:
                    continue
                taken.update(i for i, (px, py) in enumerate(pts) if math.hypot(x - px, y - py) < near)
        free = [i for i in range(len(slots)) if i not in taken]
        log.info("'%s' 칸 자리: 찬 자리 %s (사진 %s개) → %s", grade, sorted(i + 1 for i in taken) or "-",
                 "?" if seen is None else len(seen), f"{free[0] + 1}번" if free else "빈 자리 없음")
        if not free:
            raise CompartmentFull(f"'{grade}' 칸에 빈 자리가 없습니다 — 칸을 비우세요 (위에 얹지 않는다)")
        return free[0]

    def _end_tilt(self, slot):
        """끝자리: 손가락끝은 자리에, 몸통(플랜지)은 칸 가운데 쪽으로 기울인다. 반환 공구 축 단위벡터 (필요 없으면 None)."""
        p = self.cfg["place"]
        dx, dy = slot["dxy_m"]
        d = math.hypot(dx, dy)
        room = max(0.0, (p.get("compartment_len_m", 0.195) - p.get("body_length_m", 0.20)) / 2 - p.get("body_margin_m", 0.005))
        if d <= room:
            return None
        sn = min((d - room) / self.r.tool_len, math.sin(math.radians(p.get("max_end_tilt_deg", 30))))
        c = math.sqrt(1 - sn * sn)
        log.info("끝자리: 손을 %.0f° 비스듬히 (몸통은 가운데 쪽으로 %.0fmm)", math.degrees(math.asin(sn)), sn * self.r.tool_len * 1000)
        return (sn * dx / d, sn * dy / d, -c)

    def place(self, grade):
        """등급 상자 위 → 하강 → 놓기 → 상승. 기울이기는 놓기 안에서만."""
        try:
            return self._place(grade)
        finally:
            self.r.tool_axis = None

    def _place(self, grade):
        """등급 상자 위 → 하강 → 놓기 → 상승."""
        b, p = self.cfg["boxes"][grade], self.cfg["place"]
        # 칸 안 자리: 칸마다 놓은 개수만큼 다음 자리로 (같은 자리에 겹쳐 놓으면 먼저 놓은 사과를 밀어낸다)
        slots = p.get("slots", [{"dxy_m": [0.0, 0.0], "dz_m": 0.0}])
        k = self.placed.get(grade, 0)
        j = self._free_slot(grade, slots)
        slot = slots[j]
        self.placed[grade] = k + 1
        self.used_slots.setdefault(grade, set()).add(j)
        bx, by = b["xy_m"][0] + slot["dxy_m"][0], b["xy_m"][1] + slot["dxy_m"][1]
        # 끝자리는 손을 비스듬히 넣어 그리퍼 몸통(~180mm)이 칸 끝벽(195mm 칸)에 안 닿게 한다
        self.r.tool_axis = self._end_tilt(slot)
        if p.get("place_jaw_yaw_deg") is not None:   # 칸 안에서는 손가락이 이웃 사과 쪽으로 안 가게
            self.r.jaw_yaw = math.radians(p["place_jaw_yaw_deg"])
        log.info("'%s' 칸 %d번 자리 (%.3f, %.3f)%s", grade, j + 1, bx, by,
                 " — 위에 얹음 +%.0fmm" % (slot["dz_m"] * 1000) if slot["dz_m"] else "")
        z_lift = self.r.lift_z(bx, by)
        z_floor = b["floor_z_m"] + self.tuner.release_h       # 바닥에 놓을 때 높이
        z_expect = z_floor + slot["dz_m"]                     # 개수로 짐작한 높이 (빠르게 내려갈 한계로만 쓴다)
        spd = self.tuner.speed(p["descend_speed_pct"])
        log.info("== place '%s' 상자 (놓는 높이 %.3fm, 하강 %.0f%%)", grade, self.tuner.release_h, spd)
        self.dash.mission("phase", phase="place")
        self.r.transit_to(bx, by, self.cfg["motion"]["transit_speed_pct"])
        slow = p.get("slow_zone_m", 0.04)
        z_fast = z_expect + slow + p.get("contact_margin_m", 0.03)   # 여기까지 빠르게
        if self.r.current_pose()[2] > z_fast + 0.005:
            self.r.down_to(bx, by, z_fast, self.cfg["motion"]["approach_speed_pct"])
        # 바닥까지 천천히 내려가다 닿으면(아래 사과든 바닥이든) 멈춘다 — 개수로 높이를 짐작하면 사과가 굴러 나갔을 때 공중에서 떨어뜨린다
        how, z_here = self.r.guarded_down(bx, by, z_floor, spd, p.get("contact_nm", 0.5))
        if how == "contact":
            self.r.down_to(bx, by, z_here + p.get("contact_backoff_m", 0.002), spd)   # 살짝 떼서 누르는 힘 빼기
        log.info("놓는 곳: %s (플랜지 z %.3f, 바닥 기준 +%.0fmm)", "닿은 곳" if how == "contact" else "바닥 높이",
                 z_here, (z_here - z_floor) * 1000)
        z_lift = max(z_lift, z_here + 0.05)                   # 쌓인 위에서는 그보다 위로 올라간다
        held = self.r.gripper_width()
        if held < self.cfg["gripper"]["min_grasp_width_m"]:   # 운반 중에 떨어뜨렸다
            log.error("낙하: 상자에 가는 동안 사과를 놓쳤습니다 (그리퍼 폭 %.0fmm)", held * 1000)
            self.dash.mission("drop", where="운반 중", width_mm=round(held * 1000, 1))
            self.last_drop = "운반 중"
            self.r.down_to(bx, by, z_lift, p["lift_speed_pct"])
            return
        if self.roll:
            self.roll.release()              # 쥔 사과 위치 기억 + 관찰 시작
        margin = p.get("release_open_margin_m")
        self.r.grip(True, width=held + margin if margin is not None else None)   # 조금만 연다
        self.last_pick = None                # 놓았다 — 여기서 멈춰도 되돌릴 사과가 없다
        self.r.wait(p["release_settle_s"])   # 팔 정지 상태로 관찰
        if self.roll and self.cfg["roll_camera"].get("mode") == "wrist":
            self.roll.stop()                 # 팔이 올라가기 전에 관찰 끝
        self.r.down_to(bx, by, z_lift, p["lift_speed_pct"])

    def return_held(self, spd, why):
        """쥔 사과를 집었던 자리(last_pick, 없으면 트레이 가운데)에 살살 내려놓고 올라온다."""
        p, c = self.cfg["pick"], self.cfg
        if self.last_pick:
            x, y, jaw = self.last_pick
        else:
            (x1, y1), (x2, y2) = c["calibration"]["corner1_xy_m"], c["calibration"]["corner2_xy_m"]
            x, y, jaw = (x1 + x2) / 2, (y1 + y2) / 2, None
        log.warning("%s: 쥐고 있던 사과를 원래 자리 (%.3f, %.3f) 에 되돌려 놓는다", why, x, y)
        self.r.tool_axis = None
        self.r.transit_to(x, y, spd)
        self.r.jaw_yaw = jaw
        how, z = self.r.guarded_down(x, y, p["tray_z_m"] + p["grasp_height_m"] + 0.005,
                                     p["descend_speed_pct"], c["place"].get("contact_nm", 0.5))
        self.r.grip(True, width=self.r.gripper_width() + 0.012)
        self.r.down_to(x, y, max(self.r.lift_z(x, y), z + 0.05), spd)
        self.r.jaw_yaw = None
        self.last_pick = None

    def safe_park(self):
        """정리 후 정지(/park): 사과를 쥐고 있으면 집었던 자리에 되돌려 놓고, 팔을 낮은 쉬는 자세로 내린다.
        사과를 되돌렸으면 True."""
        returned = False
        p, c = self.cfg["pick"], self.cfg
        spd = c.get("estop", {}).get("speed_pct", 30)
        if self.r.holding(c["gripper"]["min_grasp_width_m"]):
            self.dash.mission("phase", phase="estop_return")
            self.return_held(spd, "비상정지")
            returned = True
        # 팔 내리기: 쉬는 자세 (낮게 — 모터 전원이 빠져도 덜 떨어진다)
        rx, ry = c.get("estop", {}).get("rest_xy_m", c["home_xy_m"])
        rz = c.get("estop", {}).get("rest_tip_z_m", 0.17)
        log.info("비상정지: 팔을 쉬는 자세 (%.3f, %.3f) 손가락끝 %.2fm 로 내린다", rx, ry, rz)
        self.dash.mission("phase", phase="estop_rest")
        self.r.transit_to(rx, ry, spd)
        self.r.down_to(rx, ry, rz + self.r.tool_len, spd)
        return returned

    def recover_held(self):
        """이어하기 전: 사과를 쥔 채 멈췄으면 트레이 가운데에 살살 내려놓는다 (다시 찍어서 처음부터 집는다)."""
        if not self.r.holding(self.cfg["gripper"]["min_grasp_width_m"]):
            self.r.grip(True)
            return
        p = self.cfg["pick"]
        (x1, y1), (x2, y2) = self.cfg["calibration"]["corner1_xy_m"], self.cfg["calibration"]["corner2_xy_m"]
        tx, ty = (x1 + x2) / 2, (y1 + y2) / 2
        log.warning("사과를 쥔 채 멈췄었다 → 트레이 가운데 (%.3f, %.3f) 에 내려놓고 다시 시작", tx, ty)
        self.r.transit_to(tx, ty, self.cfg["motion"]["transit_speed_pct"])
        how, z = self.r.guarded_down(tx, ty, p["tray_z_m"] + p["grasp_height_m"] + 0.005,
                                     p["descend_speed_pct"], self.cfg["place"].get("contact_nm", 0.5))
        self.r.grip(True, width=self.r.gripper_width() + 0.012)
        self.r.down_to(tx, ty, max(self.r.lift_z(tx, ty), z + 0.05), p["lift_speed_pct"])
        self.r.jaw_yaw = None

    def run(self, resume=False):
        """미션. resume=True 면 멈춘 사과부터 이어서 (칸별 개수·속도 조정·결과 유지)."""
        n = self.cfg["mission"].get("apple_count")   # None = 사과가 없을 때까지 (resolve_apple_count)
        retries = self.cfg["pick"]["retries"]
        if resume:
            log.info("======== 이어서 시작: 사과 %d번째부터 ========", self.next_i + 1)
            self.dash.mission("resume", index=self.next_i + 1)
            self.recover_held()
            self.r.go_home()
            return self._loop(n, retries)
        self.results, self.next_i, self.stop_requested = [], 0, False
        self.t0 = time.monotonic()
        self.dash.mission("start", apple_count=n, sim=bool(getattr(self.r, "sim", False)))
        if hasattr(self.sig, "locate_boxes"):   # 상자 위치를 사진으로 확인 (팔을 트레이 위로 비켜서)
            vx, vy = self.cfg["vision"]["box"]["view_xy_m"]
            self.r.transit_to(vx, vy, self.cfg["motion"]["transit_speed_pct"])
            try:
                res, info = self.sig.locate_boxes()
            except ValueError as e:
                raise RobotError(f"상자 위치 확인 실패: {e}")
            for g, (x, y) in res.items():
                if g in self.cfg["boxes"]:
                    self.cfg["boxes"][g]["xy_m"] = [x, y]
            log.info("상자 위치 확인 (%s): %s", info,
                     ", ".join(f"{g} ({x:.3f}, {y:.3f})" for g, (x, y) in res.items()))
            self.dash.mission("box", info=info, **{g: [round(x, 3), round(y, 3)] for g, (x, y) in res.items()})
        self.r.go_home()
        return self._loop(n, retries)

    def _loop(self, n, retries):
        results, t0 = self.results, self.t0
        clear = getattr(self.sig, "needs_clear_view", False)   # 카메라: 찍기 전에 팔을 시야 밖(홈)으로
        for i in (range(self.next_i, n) if n is not None else itertools.count(self.next_i)):
            self.next_i = i                      # 여기서 멈추면 이 사과부터 다시
            if self.stop_requested:
                log.info("멈춤 요청 → 여기까지")
                break
            # 홈 복귀 없음: 놓고 올라온 팔은 상자 위라 트레이를 가리지 않는다 (사과 하나에 ~2초)
            xy = self.sig.apple_xy(i)
            if xy is None:
                log.info("사과 좌표 없음 → 종료")
                break
            log.info("######## 사과 %d/%s ########", i + 1, n if n is not None else "?")
            self.dash.mission("apple", index=i + 1, total=n)
            ok = False
            attempt = 0
            while attempt < 1 + retries:
                self._attempt = attempt
                if attempt:
                    log.warning("재시도 %d/%d", attempt, retries)
                try:
                    ok = self.pick(*xy)
                except (RobotFault, MotionTimeout, ForceStop, SoftStop):
                    raise   # 정지 요청·고장은 '이 사과 건너뜀'이 아니다 — 미션을 멈춘다
                except RobotError as e:   # 관절 해 없음·작업영역 밖 → 이 사과만 건너뛴다
                    log.error("사과 %d 건너뜀: %s", i + 1, e)
                    self.dash.mission("skip", index=i + 1, reason=str(e))
                    if hasattr(self.sig, "mark_bad"):
                        self.sig.mark_bad()   # 다음 검출에서 이 사과는 고르지 않는다
                    self.r.go_home()
                    break
                if ok:
                    break
                attempt += 1
                self.r.grip(True)
                if clear and attempt < 1 + retries:   # 재시도는 다시 찍어서 (사과가 밀렸을 수 있다)
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
            try:
                self.place(grade)
            except CompartmentFull as e:   # 칸이 찼다 — 위에 얹지 않고 사과를 되돌린 뒤 다음 사과로
                log.warning("사과 %d: %s → 집은 자리에 되돌리고 다음 사과", i + 1, e)
                self.dash.mission("skip", index=i + 1, reason=str(e))
                self.return_held(self.cfg["motion"]["transit_speed_pct"], "칸 가득")
                if hasattr(self.sig, "mark_bad"):
                    self.sig.mark_bad()          # 되돌린 사과를 다시 집지 않게
                results.append((i + 1, grade, "칸 가득(되돌림)"))
                self.next_i = i + 1
                self.r.jaw_yaw = None
                continue
            self.last_pick = None
            self.next_i = i + 1                  # 놓았으면 이 사과는 끝 (뒤에서 멈춰도 다시 하지 않는다)
            self.r.jaw_yaw = None
            rolled = self.roll.verdict() if self.roll else self.sig.rolled(grade)
            drop_m = getattr(self.roll, "last_drop_m", None) if self.roll else None
            dropped = self.last_drop is not None or (drop_m is not None and drop_m > self.cfg["place"].get("drop_max_m", 0.010))
            if drop_m is not None:
                self.dash.mission("drop_measure", index=i + 1, drop_mm=round(drop_m * 1000, 1), dropped=bool(dropped))
            if dropped:
                log.warning("낙하 감지 (%s)", self.last_drop or f"놓을 때 약 {drop_m * 1000:.0f}mm")
                rolled = True                  # 적응형 조정은 굴림과 같이 다룬다
            self.last_drop = None
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
    for g in grade_labels(cfg):    # 칸마다 한 번씩 집는 척 → 놓는 척
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
    ap.add_argument("--apples", metavar="N",
                    help="사과 개수 (기본: 환경변수 MISSION_APPLE_COUNT, 없으면 config). 0/all = 트레이가 빌 때까지")
    ap.add_argument("--dashboard", choices=["on", "off"],
                    help="대시보드 전송 (기본: config dashboard.enabled, 단 SIM 이면 off — 실제 통계 오염 방지)")
    ap.add_argument("--serve", action="store_true",
                    help="원격 제어 서버로 대기 (프론트에서 시작·비상정지·해제·멈춤) — control.py")
    ap.add_argument("--port", type=int, default=None, help="--serve 포트 (기본 config control.port)")
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
        if args.serve:   # 원격 제어: 비상정지 상태여도 서버는 뜬다 (/resume 으로 해제)
            try:
                robot.connect(enable=True)
            except RobotFault as e:
                log.warning("로봇이 비상정지/고장 상태로 시작 — /resume 으로 해제: %s", e)
                robot._estopped.set()
        else:
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
        cfg["mission"]["apple_count"] = resolve_apple_count(cfg, args.apples)
        if args.serve:
            from control import Controller, serve
            ctl = Controller(robot, cfg, lambda: Mission(robot, cfg, AdaptiveTuner(cfg), signals, dash, roll), dash)
            if robot._estopped.is_set():
                ctl.state, ctl.error = "estopped", "시작할 때 이미 비상정지 상태"
            cc = cfg.get("control", {})
            tok = os.environ.get(cfg.get("dashboard", {}).get("token_env", "SSORRY_TOKEN"), "")
            try:
                serve(ctl, cc.get("host", "0.0.0.0"), args.port or cc.get("port", 8765), tok)
            finally:
                dash.flush()
                if roll:
                    roll.close()
            return 0
        try:
            Mission(robot, cfg, AdaptiveTuner(cfg), signals, dash, roll).run()
        except KeyboardInterrupt:
            dash.mission("estop", reason="Ctrl+C 비상정지")
            raise
        except RobotFault as e:
            dash.mission("estop", reason=f"로봇 고장: {e}")
            raise
        except ForceStop as e:
            dash.mission("estop", reason=f"힘 이상 자동 정지: {e}")
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
