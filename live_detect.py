"""실시간 사과 박스: 위 카메라(top) 프레임마다 사과를 검출해 SSORRY 서버 POST /ingest/detections 로 보낸다.

대시보드가 top 영상 위에 박스를 계속 그린다 (판정 bbox 는 판정 때 한 번뿐이라 잠깐만 보였다).
- 미션 판정과 같은 검출(vision.detect_apples)·색 보정을 쓴다. bbox 는 camerad 스트림 원본 픽셀 =
  서버가 /ws/camera 로 내보내는 JPEG 와 같은 기준
- 초당 최대 dashboard.detections_hz 번 (기본 10). 사과가 없으면 boxes: [] (박스 지우기)
- 미션 신호 소스와 같은 FrameSource 를 함께 읽기만 한다 (최신 프레임 복사본) — 미션 동작을 기다리게 하지 않는다
- 전송은 짧은 타임아웃, 실패해도 루프는 계속 (로그는 10초에 한 번)
- 등급은 보내지 않는다: 트레이 위 색만 보고 매긴 값이라 회전 검사로 정하는 최종 등급과 다를 수 있다
"""
import json
import logging
import threading
import time
import urllib.error
import urllib.request

log = logging.getLogger("live_detect")


class DetectionStreamer:
    def __init__(self, cam, cfg, dash):
        d = cfg.get("dashboard") or {}
        self.cam, self.cfg = cam, cfg
        self.url = dash.url + "/ingest/detections"
        self.token = dash.token
        self.cam_name = dash.cam
        self.period = 1.0 / max(0.5, float(d.get("detections_hz", 10)))
        self.timeout = float(d.get("detections_timeout_s", 0.5))
        self._stop = threading.Event()
        self._last_err_log = 0.0
        self.sent = self.failed = 0
        self._th = threading.Thread(target=self._run, name="live-detect", daemon=True)

    def start(self):
        self._th.start()
        log.info("실시간 사과 박스 전송: %s (최대 %.0f회/초)", self.url, 1.0 / self.period)
        return self

    def stop(self):
        self._stop.set()
        self._th.join(timeout=2)
        log.info("실시간 사과 박스: 보냄 %d, 실패 %d", self.sent, self.failed)

    def _warn(self, msg, *args):
        now = time.monotonic()
        if now - self._last_err_log >= 10.0:
            self._last_err_log = now
            log.warning(msg, *args)

    def _post(self, body):
        req = urllib.request.Request(self.url, data=json.dumps(body, ensure_ascii=False).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        if self.token:
            req.add_header("Authorization", f"Bearer {self.token}")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return r.status

    def _run(self):
        from vision import detect_apples
        last_frame = 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                img = self.cam.latest(after=last_frame, timeout=1.0)   # 새 프레임만 (같은 프레임을 두 번 보내지 않는다)
                last_frame = time.monotonic()
                apples = detect_apples(img, self.cfg)
                boxes = [{"bbox": [round(a.u - a.r, 1), round(a.v - a.r, 1), round(2 * a.r, 1), round(2 * a.r, 1)]}
                         for a in apples]
                self._post({"cam": self.cam_name, "ts": time.time(), "boxes": boxes})
                self.sent += 1
            except IOError as e:          # 카메라 프레임 없음 / 서버 연결 실패 (URLError 는 OSError)
                self.failed += 1
                self._warn("실시간 사과 박스 실패 (계속): %s", e)
            except Exception as e:        # 검출 오류 등 — 루프는 멈추지 않는다
                self.failed += 1
                self._warn("실시간 사과 박스 오류 (계속): %r", e)
            rest = self.period - (time.monotonic() - t0)
            if rest > 0:
                self._stop.wait(rest)


def start_live_detections(signals, cfg, dash):
    """카메라 신호 소스이고 대시보드 전송이 켜져 있을 때만 시작. 아니면 None."""
    if not dash.enabled or not getattr(signals, "cam", None):
        return None
    if not (cfg.get("dashboard") or {}).get("live_detections", True):
        return None
    return DetectionStreamer(signals.cam, cfg, dash).start()
