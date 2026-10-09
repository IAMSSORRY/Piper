"""원격 제어 — 프론트/대시보드 서버가 미션을 시작·비상정지·해제(이어하기)·멈춤 할 수 있게 하는 작은 HTTP 서버.

  python3 mission.py --serve            # 기본 0.0.0.0:8765

  GET  /status                  상태 {state, index, placed, results, error}
  POST /start   {"apples": 5}   미션 시작 (5개 / "all" = 트레이가 빌 때까지 / 생략 = config)
  POST /estop                   비상정지: 즉시 그 자리 정지 → 실제 비상정지. 언제든 바로 처리된다(다른 명령을 기다리지 않음)
  POST /park                    정리 후 정지: 그 자리 정지 → 쥔 사과를 집은 자리에 되돌림 → 팔을 낮게 → 실제 비상정지
                                (진행 중 /estop 을 누르면 정리를 버리고 즉시 비상정지)
  POST /resume                  비상정지(또는 오류 정지) 해제 → 멈춘 사과부터 이어서
  POST /stop                    지금 사과까지만 하고 멈춤

state: idle(대기) / running(실행 중) / stopping(/park 정리 중) / estopped(비상정지) / error(오류로 그 자리 정지) / done(완료)

⚠ 비상정지는 '즉시 그 자리'다. 예전에는 /estop 이 사과 되돌리기·팔 내리기를 먼저 했고(최대 ~20초),
  그 사이 미션 스레드가 정지 요청을 '사과 건너뜀'으로 삼켜 계속 돌면서 두 스레드가 팔을 같이 움직였다 (10-09 SIM 재현).
토큰: Authorization: Bearer <SSORRY_TOKEN> (대시보드 ingest 토큰과 같은 값). 토큰이 없으면 검사 안 함.
"""
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from piper_robot import ForceStop, RobotError, RobotFault, SoftStop  # noqa: F401 (SoftStop: _work)

log = logging.getLogger("control")


class Controller:
    def __init__(self, robot, cfg, make_mission, dash):
        self.r, self.cfg, self.make_mission, self.dash = robot, cfg, make_mission, dash
        self.mission = None
        self.state, self.error = "idle", None
        self._th = None
        self._lock = threading.Lock()       # start / resume / park 끼리만. /estop 은 이 잠금을 기다리지 않는다
        self._estop_now = threading.Event()  # /estop 이 들어옴 → /park 정리 중이면 버린다

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
        if state in ("estopped", "error"):
            # 웹(대시보드 서버)은 'estop' 이벤트로 비상정지 화면을 띄운다 — 어떤 이유로 멈췄든 보낸다
            self.dash.mission("estop", reason=error or "정지")
        elif state == "running" and self.mission is not None and self.mission.next_i:
            # 이어하기: 'apple' 이벤트가 와야 웹이 비상정지 → 진행 중으로 돌아간다
            self.dash.mission("apple", index=self.mission.next_i + 1,
                              total=self.cfg["mission"].get("apple_count"))

    # ---- 미션 실행 (백그라운드) ----
    def _work(self, resume):
        try:
            self.mission.run(resume=resume)
            self._set("done")
        except SoftStop:                 # 원격 정지 요청 — 그 자리에 섰다. 상태는 요청한 쪽(estop / park)이 정한다
            log.info("미션 스레드: 정지 요청으로 그 자리에서 멈춤")
        except RobotFault as e:          # 비상정지 버튼·PIPER Studio 비상정지·로봇 고장
            if not self.r._estopped.is_set():
                self.r.estop()
            if not self._estop_now.is_set():   # /estop 으로 멈춘 거면 상태는 estop() 이 정한다 (이벤트 중복 방지)
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
            if self.state in ("running", "stopping"):
                return False, "이미 실행 / 정리 중"
            if self.r._estopped.is_set():
                return False, "비상정지 상태 — /resume 으로 해제한 뒤 시작"
            if self._th is not None and self._th.is_alive():
                return False, "이전 미션이 아직 끝나지 않음"
            self.r.clear_soft_stop()
            self._estop_now.clear()
            if apples is not None:
                self.cfg["mission"]["apple_count"] = None if str(apples).lower() in ("all", "0") else int(apples)
            self.mission = self.make_mission()
            self._launch(resume=False)
            return True, "시작"

    def _stop_mission_thread(self, timeout):
        """미션 스레드에 정지 요청을 보내고 끝나기를 기다린다. 끝났으면 True."""
        th = self._th
        if th is None or not th.is_alive():
            return True
        self.r.soft_stop()
        th.join(timeout=timeout)
        return not th.is_alive()

    def estop(self):
        """비상정지: 즉시 그 자리 정지 → 실제 비상정지(SDK EmergencyStop). 잠금을 기다리지 않는다.

        미션 스레드는 다음 검사(10ms)에서 정지 요청(SoftStop)이나 비상정지(RobotFault)로 빠져나온다.
        /park 정리 중이었으면 정리 동작도 다음 검사에서 끊긴다.
        """
        self._estop_now.set()
        already = self.r._estopped.is_set()
        self.r.soft_stop()               # 미션 스레드가 하던 명령을 이어 보내지 않게
        if not already:
            self.r.hold()                # 펌웨어는 받은 목표로 계속 가므로 '여기서 멈춤'을 먼저 보낸다
        self.r.estop()                   # 실제 비상정지 (여러 번 눌러도 다시 보낸다)
        th = self._th
        if th is not None and th.is_alive():
            th.join(timeout=2.0)
            if th.is_alive():
                log.error("미션 스레드가 2초 안에 안 끝났다 — 비상정지는 이미 보냈다")
        note = "비상정지" + (" (다시 전송)" if already else "")
        self._set("estopped", note)
        return True, note

    def park(self):
        """정리 후 정지: 그 자리 정지 → 쥔 사과를 집은 자리에 되돌림 → 팔을 낮게 → 실제 비상정지.
        정리 중 /estop 이 오면 정리를 버리고 즉시 비상정지(estop 이 처리)."""
        if not self._lock.acquire(blocking=False):
            return False, "다른 명령 처리 중 — 바로 멈추려면 /estop"
        try:
            if self.r._estopped.is_set():
                return False, "이미 비상정지 상태 — 정리하려면 /resume 후 다시 /park"
            self._estop_now.clear()
            if not self._stop_mission_thread(timeout=3.0):
                # 미션 스레드가 안 멈췄다 — 두 스레드가 팔을 같이 움직이게 두지 않는다
                log.error("미션 스레드가 3초 안에 안 멈춤 → 정리 없이 즉시 비상정지")
                self.r.estop()
                self._set("estopped", "정리 후 정지 실패: 미션이 안 멈춰 즉시 비상정지")
                return True, "미션이 안 멈춰 즉시 비상정지"
            self._set("stopping", "정리 후 정지 중")
            self.r.clear_soft_stop()     # 미션 스레드가 끝났으니 지워도 된다 (정리 동작이 움직일 수 있게)
            note = "정리 후 정지"
            try:
                if self._estop_now.is_set():
                    raise RobotFault("정리 전에 비상정지 요청")
                if self.mission is not None:
                    returned = self.mission.safe_park()
                    note += " (사과 되돌림·팔 내림 후)" if returned else " (팔 내림 후)"
            except Exception as e:
                if self._estop_now.is_set() or self.r._estopped.is_set():
                    return True, "정리 중 비상정지 요청 — 즉시 정지"   # 상태는 estop() 이 이미 정했다
                log.error("정리 동작 실패 → 바로 비상정지: %s", e)
                note += f" (정리 실패: {e})"
            if self._estop_now.is_set():
                return True, "정리 중 비상정지 요청 — 즉시 정지"
            self.r.estop()
            self._set("estopped", note)
            return True, note
        finally:
            self._lock.release()

    def resume(self):
        with self._lock:
            if self.state in ("running", "stopping"):
                return False, "실행 / 정리 중 — 해제할 것이 없음"
            if self._th is not None:
                self._th.join(timeout=5)
                if self._th.is_alive():
                    return False, "이전 미션이 아직 끝나지 않음 — 잠시 뒤 다시"
            self.r.clear_soft_stop()
            self._estop_now.clear()
            if self.state == "estopped" or self.r._estopped.is_set():
                log.warning("비상정지 해제 — 해제 순간 모터 전원이 잠깐 빠져 팔이 처질 수 있다")
                # `mission.py --resume` 과 같은 순서: 새로 연결 → 해제 → 실제로 풀릴 때까지 기다림 → (아래) 모터 enable.
                # 예전에는 오래된 연결로 해제를 보내고 1초 뒤 바로 확인해, 늦게 풀리거나 연결이 끊겨 있으면
                # '비상정지 상태입니다 … --resume 실행' 으로 거부됐다
                try:
                    self.r.connect(enable=False)
                except RobotFault:
                    pass                 # 비상정지 상태라 거부된 것 — 연결 자체는 됐다
                except RobotError as e:
                    self._set("error", f"해제 실패: {e}")
                    return False, str(e)
                self.r.resume()          # SDK EmergencyStop(0x02) + 풀릴 때까지 대기
            try:
                self.r.connect(enable=True)   # 상태 확인 + 모터 enable (비상정지가 안 풀렸으면 여기서 거부)
            except RobotError as e:
                self._set("estopped" if isinstance(e, RobotFault) else "error", f"해제 실패: {e}")
                return False, str(e)
            if self.mission is None or self.state == "done":   # 이어 할 미션이 없으면 해제만 하고 대기
                self.r.go_home()
                self._set("idle")
                return True, "비상정지 해제 — 대기 (/start 로 시작)"
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
                     "/estop": controller.estop, "/park": controller.park,
                     "/resume": controller.resume, "/stop": controller.stop}
            fn = route.get(self.path.rstrip("/"))
            if fn is None:
                return self._send(404, {"ok": False, "error": "없는 경로"})
            ok, msg = fn()
            # 웹은 거부 이유를 error 로 읽는다 (Web src/lib/api.ts control())
            self._send(200 if ok else 409, {"ok": ok, "message": msg, **controller.status(),
                                            **({} if ok else {"error": msg})})

        def log_message(self, fmt, *args):
            log.debug("http %s", fmt % args)

    srv = ThreadingHTTPServer((host, port), H)
    log.info("원격 제어 대기: http://%s:%d  (GET /status, POST /start /estop /resume /stop)", host, port)
    srv.serve_forever()
