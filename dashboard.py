"""SSORRY 대시보드 서버(https://github.com/IAMSSORRY/Server)로 판정·모션 결과 보내기.

  POST /ingest/judge  {"grade", "confidence", "v_value", "threshold", "bbox", "cam", "ts"}
  POST /ingest/motion {"approach_speed", "place_height", "roll_detected", "ts"}

전송은 백그라운드 스레드에서 한다 — 서버가 죽거나 느려도 로봇 동작은 절대 기다리지 않는다.
토큰은 서버의 INGEST_TOKEN 과 같은 값을 환경변수(dashboard.token_env, 기본 SSORRY_TOKEN)로 준다.
"""
import json
import logging
import os
import queue
import threading
import time
import urllib.error
import urllib.request

log = logging.getLogger("dashboard")


def confidence_of(ratio, threshold):
    """판정 근거(빨강 비율)가 임계값에서 얼마나 떨어졌나 → 0.5(경계) ~ 1.0(확실)."""
    span = max(threshold, 1 - threshold) or 1.0
    return round(min(1.0, 0.5 + abs(ratio - threshold) / (2 * span)), 3)


class Dashboard:
    def __init__(self, cfg):
        d = cfg.get("dashboard") or {}
        self.enabled = bool(d.get("enabled"))
        self.url = str(d.get("url", "http://localhost:8000")).rstrip("/")
        self.cam = d.get("cam", "top")
        self.timeout = float(d.get("timeout_s", 2.0))
        self.token = os.environ.get(d.get("token_env", "SSORRY_TOKEN"), "")
        self.sent = self.failed = 0
        self._q = queue.Queue(maxsize=100)
        if self.enabled:
            if not self.token:
                log.warning("대시보드 토큰 없음 (환경변수 %s) — 서버가 INGEST_TOKEN 을 쓰면 401 로 거부된다",
                            d.get("token_env", "SSORRY_TOKEN"))
            threading.Thread(target=self._worker, daemon=True).start()
            log.info("대시보드 전송: %s (cam=%s)", self.url, self.cam)

    def _post(self, path, body):
        req = urllib.request.Request(self.url + path, data=json.dumps(body, ensure_ascii=False).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        if self.token:
            req.add_header("Authorization", f"Bearer {self.token}")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return r.status

    def _worker(self):
        while True:
            path, body = self._q.get()
            try:
                self._post(path, body)
                self.sent += 1
            except urllib.error.HTTPError as e:
                self.failed += 1
                why = {401: "토큰 불일치", 422: "형식 오류"}.get(e.code, "")
                log.warning("대시보드 %s 실패: HTTP %d %s %s", path, e.code, why, e.read()[:200].decode(errors="replace"))
            except Exception as e:
                self.failed += 1
                log.warning("대시보드 %s 실패: %s", path, e)

    def _send(self, path, body):
        if not self.enabled:
            return
        try:
            self._q.put_nowait((path, body))
        except queue.Full:
            log.warning("대시보드 큐가 가득 참 — %s 버림", path)

    def judge(self, grade, info=None):
        """info: {"ratio", "threshold", "bbox"} (카메라 판정일 때). 없으면 근거 없이 등급만."""
        info = info or {}
        ratio, th = info.get("ratio"), info.get("threshold")
        self._send("/ingest/judge", {
            "grade": grade,
            "confidence": confidence_of(ratio, th) if ratio is not None else 0.0,
            "v_value": None if ratio is None else round(ratio, 3),
            "threshold": th,
            "bbox": [int(v) for v in info.get("bbox", [0, 0, 0, 0])],
            "cam": self.cam,
            "ts": time.time(),
        })

    def motion(self, approach_speed, place_height, rolled):
        self._send("/ingest/motion", {
            "approach_speed": round(float(approach_speed), 3),
            "place_height": round(float(place_height), 4),
            "roll_detected": bool(rolled),
            "ts": time.time(),
        })

    def flush(self, timeout=3.0):
        end = time.monotonic() + timeout
        while self.enabled and not self._q.empty() and time.monotonic() < end:
            time.sleep(0.05)
        if self.enabled:
            log.info("대시보드: 보냄 %d, 실패 %d", self.sent, self.failed)
