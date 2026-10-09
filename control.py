"""원격 제어 — 프론트/대시보드 서버가 미션을 시작·비상정지·해제(이어하기)·멈춤 할 수 있게 하는 작은 HTTP 서버.

  python3 mission.py --serve            # 기본 0.0.0.0:8765

  GET  /status                  상태 {state, index, placed, results, error}
  POST /start   {"apples": 5}   미션 시작 (5개 / "all" = 트레이가 빌 때까지 / 생략 = config)
  POST /estop                   비상정지
  POST /resume                  비상정지(또는 오류 정지) 해제 → 멈춘 사과부터 이어서
  POST /stop                    지금 사과까지만 하고 멈춤

state: idle(대기) / running(실행 중) / estopped(비상정지) / error(오류로 그 자리 정지) / done(완료)
토큰: Authorization: Bearer <SSORRY_TOKEN> (대시보드 ingest 토큰과 같은 값). 토큰이 없으면 검사 안 함.
"""
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from piper_robot import ForceStop, RobotError, RobotFault

log = logging.getLogger("control")


class Controller:
    def __init__(self, robot, cfg, make_mission, dash):
        self.r, self.cfg, self.make_mission, self.dash = robot, cfg, make_mission, dash
        self.mission = None
        self.state, self.error = "idle", None
        self._th = None
        self._lock = threading.Lock()

    # ---- 상태 ----
    def status(self):
        m = self.mission
        return {"state": self.state, "error": self.error,
                "index": (m.next_i + 1) if m else None,
                "placed": dict(m.placed) if m else {},
                "results": [{"index": i, "grade": g, "note": n} for i, g, n in (m.results if m else [])],
                "ts": time.time()}

    def _set(self, state, error=None):
        self.state, self.error = state, error
        log.info("상태 → %s%s", state, f" ({error})" if error else "")
        self.dash.mission("control", state=state, error=error)

    # ---- 미션 실행 (백그라운드) ----
    def _work(self, resume):
        try:
            self.mission.run(resume=resume)
            self._set("done")
        except RobotFault as e:          # 비상정지 버튼·PIPER Studio 비상정지·로봇 고장
            if not self.r._estopped.is_set():
                self.r.estop()
            self._set("estopped", str(e))
        except ForceStop as e:           # 힘 이상 — 이미 그 자리에 섰다
            self._set("error", f"힘 이상 자동 정지: {e}")
        except (RobotError, IOError) as e:
            self.r.hold()
            self._set("error", str(e))
        except Exception as e:           # 예상 못 한 오류도 팔은 세운다
            log.exception("미션 오류")
            self.r.hold()
            self._set("error", repr(e))

    def _launch(self, resume):
        self._th = threading.Thread(target=self._work, args=(resume,), daemon=True)
        self._set("running")
        self._th.start()

    # ---- 명령 ----
    def start(self, apples=None):
        with self._lock:
            if self.state == "running":
                return False, "이미 실행 중"
            if apples is not None:
                self.cfg["mission"]["apple_count"] = None if str(apples).lower() in ("all", "0") else int(apples)
            self.mission = self.make_mission()
            self._launch(resume=False)
            return True, "시작"

    def estop(self):
        self.r.estop()                   # 실행 중이면 동작 루프가 RobotFault 로 빠져나와 estopped 가 된다
        if self.state != "running":
            self._set("estopped", "원격 비상정지")
        return True, "비상정지 전송"

    def resume(self):
        with self._lock:
            if self.state == "running":
                return False, "실행 중 — 해제할 것이 없음"
            if self.mission is None:
                return False, "이어 할 미션이 없음 (/start)"
            if self._th is not None:
                self._th.join(timeout=5)
            if self.state == "estopped" or self.r._estopped.is_set():
                log.warning("비상정지 해제 — 해제 순간 모터 전원이 잠깐 빠져 팔이 처질 수 있다")
                self.r.resume()          # SDK EmergencyStop(0x02)
                time.sleep(1.0)
            try:
                self.r.connect(enable=True)   # 상태 확인 + 모터 enable (비상정지가 안 풀렸으면 여기서 거부)
            except RobotError as e:
                self._set("estopped" if isinstance(e, RobotFault) else "error", f"해제 실패: {e}")
                return False, str(e)
            if self.mission.next_i is None or self.state == "done":
                return False, "미션이 이미 끝남 (/start)"
            self.mission.stop_requested = False
            self._launch(resume=True)
            return True, f"사과 {self.mission.next_i + 1}번째부터 이어서"

    def stop(self):
        if self.mission is not None:
            self.mission.stop_requested = True
        return True, "지금 사과까지만 하고 멈춤"


def serve(controller, host, port, token):
    class H(BaseHTTPRequestHandler):
        def _send(self, code, body):
            data = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _auth(self):
            if token and self.headers.get("Authorization", "") != f"Bearer {token}":
                self._send(401, {"ok": False, "error": "토큰 불일치"})
                return False
            return True

        def do_GET(self):
            if not self._auth():
                return
            if self.path.rstrip("/") in ("/status", ""):
                self._send(200, controller.status())
            else:
                self._send(404, {"ok": False, "error": "없는 경로"})

        def do_POST(self):
            if not self._auth():
                return
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
            except ValueError:
                return self._send(400, {"ok": False, "error": "JSON 형식 오류"})
            route = {"/start": lambda: controller.start(body.get("apples")),
                     "/estop": controller.estop, "/resume": controller.resume, "/stop": controller.stop}
            fn = route.get(self.path.rstrip("/"))
            if fn is None:
                return self._send(404, {"ok": False, "error": "없는 경로"})
            ok, msg = fn()
            self._send(200 if ok else 409, {"ok": ok, "message": msg, **controller.status()})

        def log_message(self, fmt, *args):
            log.debug("http %s", fmt % args)

    srv = ThreadingHTTPServer((host, port), H)
    log.info("원격 제어 대기: http://%s:%d  (GET /status, POST /start /estop /resume /stop)", host, port)
    srv.serve_forever()
