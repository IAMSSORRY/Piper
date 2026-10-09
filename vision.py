#!/usr/bin/env python3
"""카메라 — 사과 검출·등급·굴림 판정, 픽셀→로봇 좌표 변환.

  python3 vision.py                 # 실시간 검출 화면 (q 종료, s 저장)
  python3 vision.py --once out.jpg  # 한 장 찍어 검출 결과 저장 + 출력 (화면 없이)
  python3 vision.py --image a.jpg   # 저장된 사진으로 검출 테스트
  python3 vision.py --graycard      # 회색 카드로 색 보정 → color.yaml (조명이 바뀌면 다시)
  python3 vision.py --roll-test     # 상자 카메라 굴림 판정 시험: Enter 후 사과를 손으로 놓거나 굴린다

카메라는 PIPER Studio camerad 가 쥐고 있으므로 게이트웨이 MJPEG 스트림에서 프레임을 받는다.
스트림이 안 되면 장치를 직접 연다 (camerad 가 꺼져 있을 때).
좌표 변환은 calibrate.py 가 만든 calib.yaml (트레이 위 파지 높이 평면의 호모그래피).
"""
import logging
import math
import os
import threading
import time
import urllib.request
from dataclasses import dataclass

import cv2
import numpy as np
import yaml

log = logging.getLogger("vision")
HERE = os.path.dirname(os.path.abspath(__file__))


# ---------- 프레임 ----------

class FrameSource:
    """최신 프레임 하나만 유지하는 백그라운드 리더. latest(after=t) 로 t 이후 프레임을 받는다."""

    def __init__(self, url=None, device=None, width=1280, height=720, fourcc="MJPG"):
        self.url, self.device = url, device
        self.fourcc = fourcc
        self.size = (width, height)
        self._frame, self._t = None, 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._err = None
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _set(self, img):
        with self._lock:
            self._frame, self._t = img, time.monotonic()

    def _run(self):
        use_url = bool(self.url)
        while not self._stop.is_set():
            try:
                if use_url:
                    self._read_http()
                else:
                    self._read_device()
            except Exception as e:
                if self._err != str(e):
                    log.warning("카메라 읽기 실패: %s (재시도)", e)
                    self._err = str(e)
                # 스트림 ↔ 장치를 번갈아 시도 (camerad 가 재연결 중이면 장치는 잠겨 있고 스트림은 잠깐 빈다)
                if self.url and self.device:
                    use_url = not use_url
                    log.info("카메라 소스 전환 → %s", self.url if use_url else self.device)
                time.sleep(0.5)

    def _read_http(self):
        with urllib.request.urlopen(self.url, timeout=3) as r:
            buf = b""
            while not self._stop.is_set():
                chunk = r.read(65536)
                if not chunk:
                    raise IOError("스트림 종료")
                buf += chunk
                # 마지막 완전한 JPEG 만 디코드 (밀린 프레임은 버린다)
                e = buf.rfind(b"\xff\xd9")
                if e < 0:
                    continue
                s = buf.rfind(b"\xff\xd8", 0, e)
                if s >= 0:
                    img = cv2.imdecode(np.frombuffer(buf[s:e + 2], np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        self._set(img)
                        self._err = None
                buf = buf[e + 2:]

    def _read_device(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        if not cap.isOpened():
            raise IOError(f"{self.device} 열기 실패 (다른 프로그램이 쓰는 중? camerad)")
        try:
            while not self._stop.is_set():
                ok, img = cap.read()
                if not ok:
                    raise IOError(f"{self.device} 프레임 없음")
                self._set(img)
                self._err = None
        finally:
            cap.release()

    def latest(self, after=None, timeout=3.0):
        """after(monotonic) 이후에 들어온 프레임. timeout 안에 없으면 IOError."""
        after = time.monotonic() if after is None else after
        if self._frame is None:
            timeout += 5.0     # 첫 프레임: 스트림 실패 → 장치 직접 열기 전환 시간
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._lock:
                if self._frame is not None and self._t > after:
                    return self._frame.copy()
            time.sleep(0.01)
        raise IOError(f"카메라 프레임이 {timeout:.1f}s 동안 안 들어옵니다 ({self.url} / {self.device})"
                      + (f" — {self._err}" if self._err else ""))

    def close(self):
        self._stop.set()


def open_camera(cfg):
    v = cfg["vision"]
    return FrameSource(v.get("stream_url"), v.get("device"),
                       cfg["cameras"]["width"], cfg["cameras"]["height"])


# ---------- 색 보정 (회색 카드) ----------
# Sony A7C UVC 모드는 V4L2 컨트롤(WB·노출)이 하나도 없어 카메라 쪽 보정이 안 된다.
# 그래서 회색 카드를 찍어 채널별 이득을 구하고, 검출 전에 소프트웨어로 곱한다.

GRAY_TARGET = 118.0      # piper_cam.graycard.TARGET_LUMA 와 같은 목표 밝기
_color_cache = {}


def color_path(cfg):
    p = cfg["vision"]["color_file"]
    return p if os.path.isabs(p) else os.path.join(HERE, p)


def color_gains(cfg):
    """color.yaml 의 BGR 이득. 없으면 None (보정 안 함). 파일이 바뀌면 다시 읽는다."""
    p = color_path(cfg)
    if not os.path.exists(p):
        return None
    mt = os.path.getmtime(p)
    if _color_cache.get(p, (None,))[0] != mt:
        with open(p, encoding="utf-8") as f:
            _color_cache[p] = (mt, np.array(yaml.safe_load(f)["gains_bgr"], np.float32))
    return _color_cache[p][1]


def color_correct(bgr, cfg):
    g = color_gains(cfg)
    if g is None:
        return bgr
    return np.clip(bgr.astype(np.float32) * g, 0, 255).astype(np.uint8)


def measure_gray(bgr, roi):
    x, y, w, h = roi
    px = bgr[y:y + h, x:x + w].reshape(-1, 3).astype(np.float64)
    return px.mean(0), px.std(0)


def calibrate_gray_card(bgr, cfg):
    """회색 카드 영역 → 이득 계산·저장. (이득, 보정 전 평균, 보정 후 평균) 반환."""
    roi = cfg["vision"]["gray_card_roi_px"]
    mean, std = measure_gray(bgr, roi)
    if mean.min() < 20 or mean.max() > 240:
        raise ValueError(f"카드가 너무 어둡거나 밝습니다 (BGR {mean.round(1)}) — 조명/카메라 노출 확인")
    if std.max() > 20:
        raise ValueError(f"카드 영역이 고르지 않습니다 (표준편차 {std.round(1)}) — gray_card_roi_px 가 카드 안에 있는지 확인")
    gains = GRAY_TARGET / mean
    if gains.max() > 3.0:
        raise ValueError(f"필요한 이득 {gains.round(2)} 이 너무 큽니다 — 조명이 너무 어둡거나 카드가 아닙니다")
    with open(color_path(cfg), "w", encoding="utf-8") as f:
        yaml.safe_dump({"gains_bgr": [round(float(g), 4) for g in gains],
                        "card_bgr_before": [round(float(m), 1) for m in mean],
                        "roi": list(roi), "created": time.strftime("%Y-%m-%d %H:%M:%S")}, f, sort_keys=False)
    after, _ = measure_gray(np.clip(bgr.astype(np.float32) * gains.astype(np.float32), 0, 255), roi)
    return gains, mean, after


# ---------- 검출 ----------

@dataclass
class Apple:
    u: float          # 픽셀 중심
    v: float
    r: float          # 픽셀 반지름 (내접원)
    red_ratio: float  # 사과 영역 중 빨강 비율
    dark_ratio: float # 어두운 점(흠) 비율
    grade: str
    x: float = None   # 로봇 좌표 m (캘리브레이션 후)
    y: float = None
    d_m: float = None # 지름 m


def _masks(bgr, v):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, val = cv2.split(hsv)
    color = (s >= v["sat_min"]) & (val >= v["val_min"])
    r_lo, r_hi = v["red_hue"]           # 빨강은 0 근처를 감싼다: h <= r_lo or h >= r_hi
    y_lo, y_hi = v["yellow_hue"]
    red = color & ((h <= r_lo) | (h >= r_hi))
    yellow = color & (h > y_lo) & (h <= y_hi)
    dark = val < v["dark_val_max"]
    return red, yellow, dark


def _roi_mask(shape, poly):
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(m, [np.array(poly, np.int32)], 255)
    return m


def detect_apples(bgr, cfg, roi=None):
    """사과 목록 (픽셀). roi=None 이면 vision.tray_roi_px 사용. 붙어 있는 사과는 거리변환 봉우리로 나눈다."""
    v = cfg["vision"]
    bgr = color_correct(bgr, cfg)
    red, yellow, dark = _masks(bgr, v)
    mask = ((red | yellow).astype(np.uint8) * 255)
    roi = v["tray_roi_px"] if roi is None else roi
    mask &= _roi_mask(bgr.shape, roi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    rmin, rmax = v["min_radius_px"], v["max_radius_px"]
    # 꼭지·반사광 같은 작은 구멍만 메운다. 사과 여러 개 사이 빈틈(큰 구멍)까지 메우면 가짜 큰 사과가 생긴다
    cnts, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    filled = mask.copy()
    small = math.pi * (0.6 * rmin) ** 2
    for c, h in zip(cnts, hier[0] if hier is not None else []):
        if h[3] >= 0 and cv2.contourArea(c) < small:   # 구멍(부모 있음) 중 작은 것
            cv2.drawContours(filled, [c], -1, 255, cv2.FILLED)

    dist = cv2.distanceTransform(filled, cv2.DIST_L2, 5)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(rmin) + 1,) * 2)
    peaks = (dist >= cv2.dilate(dist, k) - 1e-3) & (dist >= rmin)
    n, _, _, cents = cv2.connectedComponentsWithStats(peaks.astype(np.uint8))
    cand = sorted(((float(dist[int(round(cy)), int(round(cx))]), cx, cy) for cx, cy in cents[1:]), reverse=True)

    apples = []
    for r, cx, cy in cand:
        if r > rmax:
            continue
        if any(math.hypot(cx - a.u, cy - a.v) < 0.9 * max(r, a.r) for a in apples):
            continue
        circ = np.zeros(filled.shape, np.uint8)
        cv2.circle(circ, (int(cx), int(cy)), int(r * 0.85), 1, -1)
        c = circ.astype(bool)
        n_red, n_yel = int((red & c).sum()), int((yellow & c).sum())
        red_ratio = n_red / max(1, n_red + n_yel)
        dark_ratio = float((dark & c).sum()) / max(1, int(c.sum()))
        apples.append(Apple(float(cx), float(cy), r, red_ratio, dark_ratio, grade_of(red_ratio, dark_ratio, v)))
    return apples


def grade_of(red_ratio, dark_ratio, v):
    g = v["grade"]
    hi, lo = g.get("labels", ["상", "중"])
    top = red_ratio >= g["red_ratio_min"] and dark_ratio <= g["dark_ratio_max"]
    return hi if top else lo


def draw(bgr, apples, cfg, calib=None, extra=None):
    out = color_correct(bgr, cfg)
    x, y, w, h = cfg["vision"]["gray_card_roi_px"]
    cv2.rectangle(out, (x, y), (x + w, y + h), (200, 200, 200), 1)
    cv2.polylines(out, [np.array(cfg["vision"]["tray_roi_px"], np.int32)], True, (255, 255, 0), 1)
    for i, a in enumerate(apples):
        col = (0, 0, 255) if a.grade == "상" else (0, 200, 255)   # 빨강 = 높은 등급, 노랑 = 낮은 등급
        cv2.circle(out, (int(a.u), int(a.v)), int(a.r), col, 2)
        cv2.drawMarker(out, (int(a.u), int(a.v)), col, cv2.MARKER_CROSS, 12, 2)
        t = f"{i} {'A' if a.grade == '상' else ('C' if a.grade == '하' else 'B')} r{a.red_ratio:.2f} d{a.dark_ratio:.2f}"
        if a.x is not None:
            t2 = f"({a.x:.3f},{a.y:.3f}) D{a.d_m * 1000:.0f}mm"
            cv2.putText(out, t2, (int(a.u - a.r), int(a.v + a.r + 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(out, t, (int(a.u - a.r), int(a.v + a.r + 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    for name, b in cfg["boxes"].items():
        if "xy_m" in b and calib is not None:
            u, v = calib.xy2px(*b["xy_m"])
        elif "px" in b:
            u, v = b["px"]
        else:
            continue
        cv2.drawMarker(out, (int(u), int(v)), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 20, 2)
        cv2.putText(out, "ZONE " + {"상": "A(top)", "중": "B(mid)", "하": "C(low)"}.get(name, name), (int(u) + 8, int(v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        if calib is not None:   # 굴림 판정 구역
            r_px = cfg["vision"]["box_radius_m"] / calib.m_per_px(u, v)
            cv2.circle(out, (int(u), int(v)), int(r_px), (255, 0, 255), 1)
    for txt_i, txt in enumerate(extra or []):
        cv2.putText(out, txt, (10, 24 + 22 * txt_i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return out


# ---------- 캘리브레이션 (호모그래피) ----------

class Calib:
    """픽셀 ↔ 로봇 XY (파지 높이 평면). calibrate.py 가 저장한 파일을 읽는다."""

    def __init__(self, H, info=None):
        self.H = np.asarray(H, np.float64)
        self.Hinv = np.linalg.inv(self.H)
        self.info = info or {}

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return cls(d["H_px_to_xy"], d)

    def save(self, path, **info):
        d = {"H_px_to_xy": self.H.tolist(), **info}
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)

    def px2xy(self, u, v):
        p = self.H @ [u, v, 1.0]
        return p[0] / p[2], p[1] / p[2]

    def xy2px(self, x, y):
        p = self.Hinv @ [x, y, 1.0]
        return p[0] / p[2], p[1] / p[2]

    def m_per_px(self, u, v):
        x0, y0 = self.px2xy(u, v)
        x1, y1 = self.px2xy(u + 1, v)
        return math.hypot(x1 - x0, y1 - y0)

    @staticmethod
    def solve(px, xy):
        """대응점 (≥4) → 호모그래피, 점별 잔차[m]."""
        px, xy = np.asarray(px, np.float64), np.asarray(xy, np.float64)
        H, _ = cv2.findHomography(px, xy, 0)
        if H is None:
            raise ValueError("호모그래피 계산 실패 (점이 한 줄로 늘어섰거나 너무 적음)")
        c = Calib(H)
        res = [math.dist(c.px2xy(*p), q) for p, q in zip(px, xy)]
        return c, res


def calib_path(cfg):
    p = cfg["vision"]["calib_file"]
    return p if os.path.isabs(p) else os.path.join(HERE, p)


def apply_calib(apples, calib):
    for a in apples:
        a.x, a.y = calib.px2xy(a.u, a.v)
        a.d_m = 2 * a.r * calib.m_per_px(a.u, a.v)
    return apples


# ---------- 상자 카메라 굴림 판정 ----------

def color_blobs(bgr, rc):
    """상자 카메라 화면의 사과(빨강·노랑) 덩어리 중심 [(u, v, 면적)]. 상자 청록·그리퍼 검정은 빠진다."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    col = (s >= rc["sat_min"]) & (v >= rc["val_min"])
    r_lo, r_hi = rc["red_hue"]
    y_lo, y_hi = rc["yellow_hue"]
    m = (col & ((h <= r_lo) | (h >= r_hi) | ((h > y_lo) & (h <= y_hi)))).astype(np.uint8) * 255
    if rc.get("roi_px"):
        m &= _roi_mask(bgr.shape, rc["roi_px"])
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    return [(float(cen[i][0]), float(cen[i][1]), int(st[i][4])) for i in range(1, n) if st[i][4] >= rc["min_area_px"]]


class BoxRollWatcher:
    """상자 안을 보는 카메라로 굴림 판정.

    release() — 그리퍼 열기 직전에 호출: 지금 상자 안 사과를 '기존'으로 기억하고 기록 시작
    verdict() — 놓고 올라온 뒤 호출: window_s 까지 기록을 마저 보고, 새 사과가 move_px 넘게 움직였으면 굴림
    """

    def __init__(self, rc):
        self.rc = rc
        self.cam = FrameSource(None, rc["device"], rc["width"], rc["height"], rc.get("fourcc"))
        self.before, self.track, self.t0 = [], [], None
        self._rec = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        last = 0.0
        while True:
            if not self._rec.is_set():
                time.sleep(0.02)
                continue
            try:
                img = self.cam.latest(after=last, timeout=1.0)
            except IOError:
                continue
            last = time.monotonic()
            self.track.append((last, color_blobs(img, self.rc), img))
            if len(self.track) > 200:
                self.track.pop(1)      # 첫 프레임(놓는 순간)은 남긴다

    def stop(self):
        """기록 멈춤 (손목 모드: 팔이 움직이기 전에 부른다 — 카메라가 움직이면 다 움직인 것처럼 보인다)."""
        self._rec.clear()

    def release(self):
        self.before_img, self.last_drop_m = None, None
        try:
            img = self.cam.latest(timeout=1.0)
            self.before_img = img
            self.before = color_blobs(img, self.rc)
        except IOError as e:
            log.warning("상자 카메라: %s", e)
            self.before = None
        self.track, self.t0 = [], time.monotonic()
        self._rec.set()

    def verdict(self):
        """True=굴림, False=안 구름 또는 판정 불가."""
        if self.t0 is None or (self.before is None and self.rc.get("mode") != "wrist"):
            return False
        if self._rec.is_set():
            rest = self.t0 + self.rc["window_s"] - time.monotonic()
            if rest > 0:
                time.sleep(rest)
            self._rec.clear()
        if self.rc.get("mode") == "wrist":
            return self._verdict_wrist()
        tol = self.rc["match_px"]

        def new(blobs):
            return [b for b in blobs if all(math.hypot(b[0] - o[0], b[1] - o[1]) > tol for o in self.before)]

        frames = [(t, new(b), img) for t, b, img in self.track]
        seen = [(t, nb, img) for t, nb, img in frames if nb]
        if not seen:
            log.warning("상자 카메라: 놓은 사과가 안 보임 → 굴림 판정 불가 (없음 처리). 카메라 각도/초점 확인")
            return False
        # 마지막에 보인 새 사과(가장 큰 것)를 기준으로, 처음 보인 위치에서 얼마나 움직였나
        end = max(seen[-1][1], key=lambda b: b[2])
        start = min(seen[0][1], key=lambda b: math.hypot(b[0] - end[0], b[1] - end[1]))
        path = max(math.hypot(b[0] - start[0], b[1] - start[1]) for _, nb, _ in seen for b in nb
                   if math.hypot(b[0] - end[0], b[1] - end[1]) < 4 * tol)
        moved = max(path, math.hypot(end[0] - start[0], end[1] - start[1]))
        out = seen[-1][2].copy()
        cv2.circle(out, (int(start[0]), int(start[1])), 8, (255, 255, 0), 2)
        cv2.circle(out, (int(end[0]), int(end[1])), 8, (0, 0, 255), 2)
        cv2.line(out, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), (0, 255, 255), 2)
        cv2.putText(out, f"moved {moved:.0f}px", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        os.makedirs(os.path.join(HERE, "snapshots"), exist_ok=True)
        cv2.imwrite(os.path.join(HERE, "snapshots", time.strftime("%H%M%S_roll.jpg")), out)
        rolled = moved > self.rc["move_px"]
        log.info("상자 카메라: 새 사과 이동 %.0fpx (기준 %dpx, %d프레임) → %s",
                 moved, self.rc["move_px"], len(seen), "굴림" if rolled else "안정")
        return rolled

    def _verdict_wrist(self):
        """손목 카메라: 그리퍼 연 뒤 사과가 떨어져 자리 잡은 시점(settle_from_s) 프레임과 마지막 프레임 사이의
        화면 전체 이동량(위상 상관)으로 판정. 색과 무관 — 바로 위에서 보면 노란 사과는 채도가 빠져 색으로 못 찾는다."""
        rc = self.rc
        t_ref = self.t0 + rc.get("settle_from_s", 0.5)
        frames = [(t, img) for t, _, img in self.track if t >= t_ref]
        if len(frames) < 3:
            log.warning("손목 카메라: 관찰 프레임 부족 (%d) → 판정 불가 (없음 처리)", len(frames))
            return False

        def gray(img):
            g = cv2.cvtColor(cv2.resize(img, (160, 120)), cv2.COLOR_BGR2GRAY).astype(np.float32)
            return cv2.GaussianBlur(g, (5, 5), 0)

        win = cv2.createHanningWindow((160, 120), cv2.CV_32F)
        g0 = gray(frames[0][1])
        moved, diff = 0.0, 0.0
        for _, img in frames[1:]:
            g = gray(img)
            (dx, dy), _ = cv2.phaseCorrelate(g0.copy(), g.copy(), win)   # ⚠ 창(win)을 주면 입력을 덮어쓴다 (OpenCV 4.5)
            moved = max(moved, math.hypot(dx, dy) * 4.0)          # 160 → 640 px 환산
            diff = max(diff, float(np.mean(np.abs(g - g0))))
        out = frames[-1][1].copy()
        cv2.putText(out, f"shift {moved:.0f}px diff {diff:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        os.makedirs(os.path.join(HERE, "snapshots"), exist_ok=True)
        cv2.imwrite(os.path.join(HERE, "snapshots", time.strftime("%H%M%S_roll.jpg")), out)
        self.last_drop_m = self._drop_from_scale(self.before_img, frames[-1][1]) if self.before_img is not None else None
        rolled = moved > rc["move_px"] or diff > rc.get("diff_max", 18.0)
        log.info("손목 카메라: 화면 이동 %.0fpx, 밝기 변화 %.1f (기준 %dpx / %.0f, %d프레임) → %s",
                 moved, diff, rc["move_px"], rc.get("diff_max", 18.0), len(frames), "굴림" if rolled else "안정")
        return rolled

    def _drop_from_scale(self, before, after):
        """놓기 직전(쥐고 있을 때)과 놓은 뒤 화면의 '가운데(사과) 크기 비'로 낙하 거리 추정.
        사과가 떨어지면 카메라에서 멀어져 작아 보인다: 거리 D → D+Δ 이면 크기비 s = D/(D+Δ), Δ = D(1/s - 1)."""
        D = float(self.rc.get("cam_to_apple_top_m", 0.07))

        def crop(img):
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            h, w = g.shape
            return g[int(h * 0.2):int(h * 0.8), int(w * 0.2):int(w * 0.8)]

        a, b = crop(before), crop(after)
        orb = cv2.ORB_create(1500)
        ka, da = orb.detectAndCompute(a, None)
        kb, db = orb.detectAndCompute(b, None)
        if da is None or db is None or len(ka) < 20 or len(kb) < 20:
            log.warning("손목 카메라: 낙하 측정 실패 (특징점 부족)")
            return None
        m = sorted(cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db), key=lambda x: x.distance)[:300]
        if len(m) < 15:
            log.warning("손목 카메라: 낙하 측정 실패 (매칭 %d개)", len(m))
            return None
        pa = np.float32([ka[x.queryIdx].pt for x in m])
        pb = np.float32([kb[x.trainIdx].pt for x in m])
        M, inl = cv2.estimateAffinePartial2D(pa, pb, ransacReprojThreshold=3)
        if M is None or inl is None or inl.sum() < 12:
            log.warning("손목 카메라: 낙하 측정 실패 (일관된 매칭 부족)")
            return None
        sc = math.hypot(M[0, 0], M[1, 0])
        drop = max(0.0, D * (1.0 / sc - 1.0))
        log.info("손목 카메라: 사과 크기비 %.3f → 낙하 약 %.0fmm (인라이어 %d)", sc, drop * 1000, int(inl.sum()))
        return drop

    def close(self):
        self.cam.close()


def open_roll_watcher(cfg):
    rc = cfg.get("roll_camera") or {}
    if not rc.get("enabled"):
        return None
    try:
        return BoxRollWatcher(rc)
    except Exception as e:
        log.warning("상자 카메라 열기 실패 (%s) → 위 카메라로 굴림 판정", e)
        return None


# ---------- 멍 검사 (사과를 들어 위 카메라에 비춰 돌려 가며) ----------

def bruise_score(bgr, cfg):
    """들고 있는 사과(화면에서 가장 큰 빨강/노랑 덩어리)에서 '평소보다 어두운 갈색' 비율.
    반환 (비율, 사과 픽셀 수, 표시 이미지). 사과가 안 보이면 (None, 0, img)."""
    v = cfg["vision"]
    b = cfg["inspect"]["bruise"]
    img = color_correct(bgr, cfg)
    red, yellow, _ = _masks(img, v)
    m = ((red | yellow).astype(np.uint8) * 255)
    m &= _roi_mask(img.shape, b["roi_px"])
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n <= 1:
        return None, 0, img
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < b["min_apple_px"]:
        return None, int(st[k, cv2.CC_STAT_AREA]), img
    apple = (lab == k).astype(np.uint8)
    # 구멍(아주 어두운 곳)은 채우지 않는다: 채우면 꽃받침(밑동의 둥근 검은 오목)까지 멍으로 센다 (21:40 실측).
    # 진한 멍은 가장자리의 덜 어두운 부분으로 잡힌다 (멍 사과 6.0% vs 멀쩡한 사과 0.2% 이하)
    # 가장자리(손가락 그림자·반사)는 빼고 안쪽만 본다: 가장자리에서 반지름의 edge_frac 이상 들어간 곳
    dist = cv2.distanceTransform(apple, cv2.DIST_L2, 5)
    apple = dist >= dist.max() * b.get("edge_frac", 0.25)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, val = cv2.split(hsv)
    med = float(np.median(val[apple]))
    med_s = float(np.median(s[apple]))
    # 멍 = 어둡고(밝기↓) + 칙칙하다(채도↓, 갈색). 그림자·꼭지 오목한 곳은 어두워도 채도가 그대로라 빠진다
    dark = apple & (val < med * b["dark_ratio_of_median"]) & (s < med_s * b["sat_ratio_of_median"])
    dark = cv2.morphologyEx(dark.astype(np.uint8), cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    ratio = float(dark.sum()) / max(1, int(apple.sum()))
    out = img.copy()
    out[dark.astype(bool)] = (255, 0, 255)
    cv2.putText(out, f"bruise {ratio * 100:.1f}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return ratio, int(apple.sum()), out


# ---------- 트레이 위치 (사진으로, 매번) ----------

def find_tray_floor(bgr, cfg, inset=None):
    """청록 트레이를 찾아 '바닥 가장자리' 사각형(픽셀 4점)을 돌려준다. 윗테두리에서 wall_inset_px 만큼 안쪽.
    (테두리는 바닥보다 높아 위에서 보면 바깥으로 보인다 — 벽 여유를 크게 잡는 실수를 막는다.) 못 찾으면 None."""
    t = cfg["vision"]["tray"]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    lo, hi = t["hue"]
    m = ((h >= lo) & (h <= hi) & (s >= t["sat_min"]) & (v >= t["val_min"])).astype(np.uint8) * 255
    m &= _roi_mask(bgr.shape, t["search_roi_px"])
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (41, 41)))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < t["min_area_px"]:
        return None
    (cx, cy), (w, hh), ang = cv2.minAreaRect(c)
    k = t["wall_inset_px"] if inset is None else inset
    return cv2.boxPoints(((cx, cy), (max(1.0, w - 2 * k), max(1.0, hh - 2 * k)), ang))


# ---------- 상자 위치 (사진으로) ----------

def find_box(bgr, cfg):
    """골판지 상자 바깥 사각형 4꼭짓점 (위왼·위오·아래오·아래왼, 픽셀). 못 찾으면 None."""
    b = cfg["vision"]["box"]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    lo, hi = b["hue"]
    m = ((h >= lo) & (h <= hi) & (s >= b["sat_min"]) & (v >= b["val_min"])).astype(np.uint8) * 255
    m &= _roi_mask(bgr.shape, b["search_roi_px"])
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31)))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < b["min_area_px"]:
        return None
    pts = cv2.boxPoints(cv2.minAreaRect(c))
    # 위왼·위오·아래오·아래왼 순서로
    sm, df = pts.sum(1), np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(sm)], pts[np.argmin(df)], pts[np.argmax(sm)], pts[np.argmax(df)]], np.float32)


def locate_boxes(bgr, cfg, calib):
    """사진에서 상자를 찾아 기준 사진 대비 이동·회전을 구하고, 칸 중심(로봇 좌표)을 돌려준다.
    반환 ({등급: (x, y)}, 정보 문자열, 그린 이미지). 못 찾거나 너무 많이 움직였으면 ValueError."""
    b = cfg["vision"]["box"]
    cur = find_box(bgr, cfg)
    if cur is None:
        raise ValueError("사진에서 상자를 못 찾았습니다 (팔이 가렸거나 상자가 화면 밖)")
    ref = np.array(b["ref_corners_px"], np.float32)
    M, _ = cv2.estimateAffinePartial2D(ref, cur)
    if M is None:
        raise ValueError("상자 위치 계산 실패")
    ang = math.degrees(math.atan2(M[1, 0], M[0, 0]))
    scale = math.hypot(M[0, 0], M[1, 0])
    out = bgr.copy()
    cv2.polylines(out, [cur.astype(int)], True, (0, 0, 255), 2)
    res, shift_m = {}, 0.0
    for g, (u, v) in b["compartments_px"].items():
        cu, cv_ = M @ np.array([u, v, 1.0])
        x, y = calib.px2xy(cu, cv_)
        x0, y0 = calib.px2xy(u, v)
        shift_m = max(shift_m, math.hypot(x - x0, y - y0))
        res[g] = (float(x), float(y))
        cv2.drawMarker(out, (int(cu), int(cv_)), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
        cv2.putText(out, f"{g} ({x:.3f},{y:.3f})", (int(cu) - 60, int(cv_) - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    info = f"이동 {shift_m * 1000:.0f}mm, 회전 {ang:.1f}°, 크기비 {scale:.3f}"
    if shift_m > b["max_shift_m"] or abs(ang) > b["max_rot_deg"] or abs(scale - 1) > 0.15:
        raise ValueError(f"상자가 기준에서 너무 많이 바뀌었습니다 ({info}) — 상자 위치 확인 또는 기준 사진 다시")
    return res, info, out


# ---------- 미션 연동 신호 ----------

class CameraSignals:
    """mission.py 신호 소스: apple_xy / grade / rolled.

    - apple_xy: 팔을 화면 밖(홈)으로 뺀 뒤 트레이를 찍어 가장 가까운(로봇 기준) 사과 좌표. 등급도 이때 매긴다
      (트레이 위에서는 사과 전체가 보여 판정이 가장 정확하다)
    - grade: 위에서 매긴 그 사과의 등급
    - rolled: 놓기 전 상자 사진과 비교해 새로 들어온 사과를 찾고, 놓은 지점에서의 거리·흔들림으로 판정
    """

    needs_clear_view = True   # mission 이 apple_xy 전에 팔을 홈으로 뺀다

    def __init__(self, cfg, cam=None):
        self.cfg, self.v = cfg, cfg["vision"]
        self.cam = cam or open_camera(cfg)
        p = calib_path(cfg)
        if not os.path.exists(p):
            raise FileNotFoundError(f"캘리브레이션 파일이 없습니다: {p}\n  먼저: python3 calibrate.py")
        self.calib = Calib.load(p)
        self.cur = None
        self.box_img = None        # 놓기 전 상자 상태 (apple_xy 때 같이 찍음)
        self.bad_xy = []           # 못 집은 사과 위치 — 다시 고르지 않는다
        self.bruises = []          # 회전 검사 멍 비율
        self.snap_dir = os.path.join(HERE, "snapshots")
        os.makedirs(self.snap_dir, exist_ok=True)

    def _save(self, tag, img):
        p = os.path.join(self.snap_dir, time.strftime("%H%M%S_") + tag + ".jpg")
        cv2.imwrite(p, img)
        return p

    def apple_xy(self, i):
        img = self.cam.latest(after=time.monotonic() + self.v["settle_s"])
        self.box_img = img
        floor = find_tray_floor(img, self.cfg)
        if floor is not None:   # 트레이가 밀려도 따라간다
            poly = [self.calib.px2xy(u, v) for u, v in floor]
            self.tray_walls = [(*poly[i], *poly[(i + 1) % 4]) for i in range(4)]
            self.tray_center = tuple(np.mean(poly, axis=0))
        else:
            log.warning("트레이를 사진에서 못 찾음 → 설정의 tray_roi_px 로 벽 계산")
            self.tray_walls, self.tray_center = None, None
        rim = find_tray_floor(img, self.cfg, inset=-10)   # 트레이를 따라 사과 찾는 영역 (조금 넉넉히)
        roi = rim.astype(int).tolist() if rim is not None else None
        apples = apply_calib(detect_apples(img, self.cfg, roi=roi), self.calib)
        ws = self.cfg["workspace"]
        ok = [a for a in apples if ws["x"][0] <= a.x <= ws["x"][1] and ws["y"][0] <= a.y <= ws["y"][1]
              and math.hypot(a.x, a.y) <= ws.get("max_reach_xy_m", 9.0)]
        if len(ok) < len(apples):
            log.warning("닿지 않는 사과 %d개 제외 (베이스에서 %.2fm 넘음 등)", len(apples) - len(ok), ws.get("max_reach_xy_m", 9.0))
        self._save(f"detect{i + 1}", draw(img, apples, self.cfg, self.calib, [f"apple {i + 1}: {len(ok)} found"]))
        if not ok:
            log.warning("사과 검출 0개 (전체 후보 %d개)", len(apples))
            return None
        cand = [a for a in ok if all(math.hypot(a.x - bx, a.y - by) > 0.03 for bx, by in self.bad_xy)]
        if not cand:
            log.warning("집을 수 있는 사과가 없음 (못 집는 사과 %d개 제외)", len(ok))
            return None
        # 로봇에 가까운 것부터 (팔이 다른 사과 위를 지나가지 않게)
        a = min(cand, key=lambda a: math.hypot(a.x, a.y))
        gw = self.cfg["gripper"]["open_width_m"]
        if a.d_m > gw:
            log.warning("사과 지름 %.0fmm > 그리퍼 열림 %.0fmm — 못 집을 수 있음", a.d_m * 1000, gw * 1000)
        self.cur = a
        self.cur_others = [(b.x, b.y, b.d_m / 2) for b in apples if b is not a]
        log.info("검출 %d개 → 사과 (%.3f, %.3f) 지름 %.0fmm 등급 %s (빨강 %.2f, 흠 %.2f)",
                 len(ok), a.x, a.y, a.d_m * 1000, a.grade, a.red_ratio, a.dark_ratio)
        return a.x, a.y

    def grade(self):
        return self.cur.grade if self.cur else None

    def inspect_frame(self, tag):
        """회전 검사 한 장: 멍 비율 기록."""
        img = self.cam.latest(after=time.monotonic() + 0.05)
        ratio, px, out = bruise_score(img, self.cfg)
        self._save(f"inspect_{tag}", out)
        self._save(f"inspect_{tag}_raw", img)      # 기준값 다시 맞출 때 쓰는 원본
        self.bruises.append(ratio)
        log.info("  검사 %s: 사과 %dpx, 멍 %s", tag, px, "안 보임" if ratio is None else f"{ratio * 100:.1f}%")
        return ratio

    def final_grade(self):
        """노랑 → 낮은 등급(하). 빨강: 회전 검사에서 멍이 기준 넘으면 중, 아니면 상."""
        if self.cur is None:
            return None
        hi, lo = self.v["grade"].get("labels", ["상", "하"])
        if self.cur.grade == lo:
            return lo
        seen = [b for b in self.bruises if b is not None]
        if not seen:
            log.warning("회전 검사에서 사과가 안 보였음 → 색으로만 판정 (%s)", self.cur.grade)
            return self.cur.grade
        worst = max(seen)
        self.cur.dark_ratio = worst
        thr = self.cfg["inspect"]["bruise"]["ratio_max"]
        g = self.cfg["inspect"]["bruise"].get("label", "중") if worst > thr else hi
        log.info("멍 최대 %.1f%% (기준 %.1f%%) → %s", worst * 100, thr * 100, g)
        return g

    def locate_boxes(self):
        """상자 칸 중심을 사진으로 다시 잡는다 (미션 시작 때, 팔이 상자를 안 가릴 때 부른다)."""
        img = self.cam.latest(after=time.monotonic() + self.v["settle_s"])
        res, info, out = locate_boxes(img, self.cfg, self.calib)
        self._save("box", out)
        return res, info

    def mark_bad(self):
        if self.cur is not None:
            self.bad_xy.append((self.cur.x, self.cur.y))

    def grasp_context(self):
        """집게 방향 고르기용: 이 사과 반지름, 이웃 사과 [(x, y, r)], 트레이 안쪽 벽 선분 (로봇 좌표)."""
        if self.cur is None:
            return None
        walls, center = getattr(self, "tray_walls", None), getattr(self, "tray_center", None)
        if walls is None:
            poly = [self.calib.px2xy(u, v) for u, v in self.v["tray_roi_px"]]
            walls = [(*poly[i], *poly[(i + 1) % len(poly)]) for i in range(len(poly))]
            center = tuple(np.mean(poly, axis=0))
        return {"r": self.cur.d_m / 2, "others": self.cur_others, "walls": walls, "center": center}

    def judge_info(self):
        """대시보드용 판정 근거: 빨강 비율 vs 임계값, 위 카메라 픽셀 bbox."""
        a = self.cur
        if a is None:
            return None
        return {"ratio": a.red_ratio, "threshold": self.v["grade"]["red_ratio_min"],
                "bbox": [a.u - a.r, a.v - a.r, 2 * a.r, 2 * a.r],
                "extra": {"dark_ratio": round(a.dark_ratio, 3), "dark_max": self.v["grade"]["dark_ratio_max"]}}

    def _box_roi(self, grade):
        bx, by = self.cfg["boxes"][grade]["xy_m"]
        r = self.v["box_radius_m"]
        pts = [self.calib.xy2px(bx + r * math.cos(t), by + r * math.sin(t))
               for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
        return [[int(u), int(v)] for u, v in pts]

    def _box_apples(self, grade, img):
        return apply_calib(detect_apples(img, self.cfg, roi=self._box_roi(grade)), self.calib)

    def rolled(self, grade=None):
        return self._rolled_top(grade)

    def _rolled_top(self, grade=None):
        """(상자 카메라가 없을 때) 놓은 뒤(팔이 시야 밖일 때) 상자 안 사과가 굴렀는지.
        놓기 전 상자 사진(apple_xy 때 찍은 것)에 없던 사과를 찾아, 놓은 지점에서의 거리와 관찰 중 움직임으로 판정."""
        if grade is None or self.box_img is None:
            return False
        rc = self.v["roll"]
        u, v = self.calib.xy2px(*self.cfg["boxes"][grade]["xy_m"])
        h, w = self.box_img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            log.info("'%s' 상자가 카메라 화면 밖 (px %.0f, %.0f) → 굴림 판정 불가, 없음으로 처리", grade, u, v)
            return False
        before = self._box_apples(grade, self.box_img)
        t0 = time.monotonic()
        first = self._box_apples(grade, self.cam.latest(after=t0))
        img = first_img = self.cam.latest(after=t0 + rc["window_s"])
        last = self._box_apples(grade, img)
        tol = rc["match_m"]

        def new_in(apples):
            return [a for a in apples if all(math.hypot(a.x - b.x, a.y - b.y) > tol for b in before)]

        first, last = new_in(first), new_in(last)
        self._save(f"place_{'A' if grade == '상' else 'B'}", draw(first_img, last, self.cfg, self.calib))
        if not last:
            log.warning("상자 안에서 놓은 사과를 못 찾음 → 굴림(이탈)으로 판단")
            return True
        px, py = self.cfg["boxes"][grade]["xy_m"]
        a = min(last, key=lambda a: math.hypot(a.x - px, a.y - py))
        off = math.hypot(a.x - px, a.y - py)
        moved = min((math.hypot(a.x - b.x, a.y - b.y) for b in first), default=0.0)
        log.info("놓은 사과: 목표에서 %.0fmm, 관찰 중 이동 %.0fmm", off * 1000, moved * 1000)
        return off > rc["max_offset_m"] or moved > rc["max_motion_m"]

    def close(self):
        self.cam.close()


# ---------- CLI ----------

def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    ap.add_argument("--once", metavar="OUT", help="한 장 찍어 결과 이미지를 OUT 에 저장")
    ap.add_argument("--image", help="카메라 대신 이미지 파일")
    ap.add_argument("--graycard", action="store_true", help="회색 카드 색 보정 (color.yaml 저장)")
    ap.add_argument("--roll-test", action="store_true", help="상자 카메라 굴림 판정 시험")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname).1s [%(name)s] %(message)s")
    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    calib = Calib.load(calib_path(cfg)) if os.path.exists(calib_path(cfg)) else None
    if calib is None:
        log.warning("calib.yaml 없음 — 픽셀 좌표만 표시")

    def report(img):
        apples = detect_apples(img, cfg)
        if calib:
            apply_calib(apples, calib)
        for i, a in enumerate(apples):
            xy = f"  xy=({a.x:.3f}, {a.y:.3f}) m  지름 {a.d_m * 1000:.0f}mm" if calib else ""
            print(f"  {i}: px=({a.u:.0f},{a.v:.0f}) r={a.r:.0f}  빨강 {a.red_ratio:.2f} 흠 {a.dark_ratio:.2f} → {a.grade}{xy}")
        return draw(img, apples, cfg, calib, [f"{len(apples)} apples"])

    if args.roll_test:
        w = BoxRollWatcher(cfg["roll_camera"])
        try:
            img = w.cam.latest(timeout=5)
            print(f"상자 카메라 {img.shape[1]}x{img.shape[0]}, 지금 보이는 사과 덩어리 {len(color_blobs(img, cfg['roll_camera']))}개")
            while True:
                if input("Enter = 관찰 시작 (그 뒤 사과를 놓거나 굴린다), q = 끝 > ").strip() == "q":
                    break
                w.release()
                print("결과:", "굴림" if w.verdict() else "안정/판정불가", "(snapshots/*_roll.jpg)")
        finally:
            w.close()
        return
    if args.graycard:
        if args.image:
            img = cv2.imread(args.image)
        else:
            cam = open_camera(cfg)
            img = cam.latest(timeout=5)
            cam.close()
        gains, before, after = calibrate_gray_card(img, cfg)
        print(f"회색 카드 BGR {before.round(1)} → 보정 후 {after.round(1)}  (이득 {gains.round(3)})")
        print("저장", color_path(cfg))
        if not (args.once or args.image):
            return
    if args.image or args.once:
        if args.image:
            img = cv2.imread(args.image)
        else:
            cam = open_camera(cfg)
            img = cam.latest(timeout=5)
            cam.close()
        out = report(img)
        if args.once:
            cv2.imwrite(args.once, out)
            print("저장", args.once)
        else:
            cv2.imshow("vision", out)
            cv2.waitKey(0)
        return
    cam = open_camera(cfg)
    while True:
        out = report(cam.latest(timeout=5))
        cv2.imshow("vision (s=save, q=quit)", out)
        k = cv2.waitKey(200) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("s"):
            p = os.path.join(HERE, "snapshots", time.strftime("%H%M%S_vision.jpg"))
            cv2.imwrite(p, out)
            print("저장", p)
    cam.close()


if __name__ == "__main__":
    main()
