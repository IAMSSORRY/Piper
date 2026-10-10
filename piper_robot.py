"""PIPER 로봇 래퍼 — 실기(piper_sdk C_PiperInterface_V2)와 SIM 을 같은 코드 경로로 돌린다.

외부 API 단위는 m / rad. SDK 는 0.001mm / 0.001deg 정수 (piper_sdk 0.6.x 기준).
SIM 은 SDK 에서 쓰는 메서드만 같은 이름·같은 반환 구조로 흉내 내므로,
대기·타임아웃·고장 판정 로직이 실기와 똑같이 실행된다.
"""
import logging
import math
import re
import subprocess
import threading
import time
from types import SimpleNamespace as NS

log = logging.getLogger("robot")

# MotionCtrl_2 인자
CTRL_CAN = 0x01
MOVE_P, MOVE_J, MOVE_L = 0x00, 0x01, 0x02

# GetArmStatus().arm_status.arm_status 코드 (SDK ArmMsgFeedbackStatusEnum.ArmStatus)
ARM_STATUS_TEXT = {
    0x00: "정상", 0x01: "비상정지 상태", 0x02: "IK 해 없음 (도달 불가 좌표/자세)",
    0x03: "특이점", 0x04: "목표 각도 한계 초과", 0x05: "관절 통신 이상",
    0x06: "관절 브레이크 미해제", 0x07: "충돌 감지", 0x08: "티칭 중 과속",
    0x09: "관절 상태 이상", 0x0A: "기타 이상", 0x0B: "티칭 기록 중",
    0x0C: "티칭 실행 중", 0x0D: "티칭 일시정지", 0x0E: "메인보드 과열",
    0x0F: "방전저항 과열",
}
NON_FAULT_STATUS = {0x00, 0x0B, 0x0C, 0x0D}
# 도달 불가 목표 — 팔은 멀쩡하다. 비상정지(전원 차단) 대신 그 자리 정지 후 종료
UNREACHABLE_STATUS = {0x02, 0x03, 0x04}

# SDK JointCtrl 문서의 관절 한계 (rad)
JOINT_LIMITS = [(-2.6179, 2.6179), (0.0, 3.14), (-2.967, 0.0),
                (-1.745, 1.745), (-1.22, 1.22), (-2.09439, 2.09439)]

RESEND_PERIOD_S = 0.05   # 명령 전송 주기 (처음 SEND_BURST_S 동안만)
SEND_BURST_S = 0.25      # 명령은 처음에만 몇 번 보낸다 — 계속 보내면 궤적이 매번 재시작돼 뚝뚝 끊긴다
STALL_RESEND_S = 1.0     # 도착 전에 이만큼 안 움직이면 다시 보낸다
NEAR_HOLD_S = 0.4        # '로봇 도착 + 근처' 가 이만큼 유지되면 도착으로 본다
NEAR_POS_M = 0.015       # 근처 판정 거리
NEAR_JOINT_RAD = 0.05
MIN_MOVE_TIME_S = 0.15   # 명령 직후 motion_status 가 아직 이전 값일 수 있어 최소 대기


def m2sdk(v): return round(v * 1_000_000)
def sdk2m(v): return v / 1_000_000
def rad2sdk(v): return round(math.degrees(v) * 1000)
def sdk2rad(v): return math.radians(v / 1000)


class RobotError(Exception):
    pass


class CanError(RobotError):
    pass


class MotionTimeout(RobotError):
    pass


class RobotFault(RobotError):
    pass


class _Contact(Exception):
    """guarded_down 안에서만 쓰는 신호: 닿았다."""


class SoftStop(RobotError):
    """원격 정지 요청 — 하던 동작을 그 자리에서 멈춘다.

    ⚠ RobotError 의 하위라서 `except RobotError` 가 삼키기 쉽다. 미션에서 RobotError 를 잡는 곳은
    반드시 SoftStop 을 먼저 다시 올려야 한다 (예전에 '이 사과 건너뜀'으로 처리돼 미션이 계속 돌았다).
    요청은 clear_soft_stop() 전까지 유지되므로 한 번 삼켜져도 다음 검사에서 다시 걸린다."""
    pass


class ForceStop(RobotError):
    """관절 부하가 갑자기 튐 (어딘가에 닿았다) → 그 자리 정지."""
    pass


def _fix_cmd(port):
    return (f"sudo ip link set {port} down; "
            f"sudo ip link set {port} type can bitrate 1000000; "
            f"sudo ip link set {port} up")


def check_can(port, bitrate=1_000_000):
    """can 인터페이스 존재·UP·bitrate·버스 상태 점검. 문제면 원인과 해결 명령을 담아 CanError."""
    try:
        r = subprocess.run(["ip", "-details", "link", "show", port],
                           capture_output=True, text=True, timeout=3)
    except FileNotFoundError:
        raise CanError("ip 명령이 없습니다: sudo apt install iproute2")
    if r.returncode != 0:
        others = subprocess.run(["ip", "-br", "link", "show", "type", "can"],
                                capture_output=True, text=True, timeout=3).stdout.strip()
        raise CanError(f"{port} 인터페이스가 없습니다. USB-CAN 어댑터 연결을 확인하세요.\n"
                       f"  현재 CAN 인터페이스: {others or '없음'}")
    txt = r.stdout
    flags = re.search(r"<([^>]*)>", txt)
    up = flags is not None and "UP" in flags.group(1).split(",")
    br = re.search(r"bitrate (\d+)", txt)
    state = re.search(r"\bcan (?:<[^>]*> )?state ([A-Z-]+)", txt)
    if br is None or int(br.group(1)) != bitrate:
        raise CanError(f"{port} bitrate 가 {br.group(1) if br else '미설정'} 입니다 (필요: {bitrate}).\n"
                       f"  해결: {_fix_cmd(port)}")
    if not up:
        raise CanError(f"{port} 가 DOWN 입니다.\n  해결: {_fix_cmd(port)}")
    if state and state.group(1) == "BUS-OFF":
        raise CanError(f"{port} BUS-OFF — 배선(CAN H/L 뒤바뀜)·종단저항·bitrate 불일치 확인 후\n"
                       f"  재시작: {_fix_cmd(port)}")
    if state and state.group(1) == "ERROR-PASSIVE":
        log.warning("%s ERROR-PASSIVE — 받는 쪽이 없습니다. 로봇 전원/CAN 케이블 확인", port)
    log.info("CAN %s OK (UP, %d bps, %s)", port, bitrate, state.group(1) if state else "?")


def check_conflicts():
    """같은 CAN 을 잡는 PIPER Studio robotd 가 돌고 있으면 경고."""
    try:
        r = subprocess.run(["systemctl", "--user", "is-active", "piper-robotd"],
                           capture_output=True, text=True, timeout=3)
        if r.stdout.strip() == "active":
            log.warning("PIPER Studio robotd 가 실행 중입니다 (같은 CAN 을 읽는 것은 괜찮다).\n"
                        "  이 프로그램이 도는 동안 PIPER Studio 화면에서 팔을 조작하지 마세요 — 명령이 섞입니다.\n"
                        "  ⚠ Studio 에서 [연결 해제]/[연결(attach)] 을 누르면 토크가 꺼져 팔이 떨어집니다")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass


class Robot:
    def __init__(self, cfg, sim=False):
        self.cfg = cfg
        self.sim = sim
        m = cfg["motion"]
        self.timeout = float(m["move_timeout_s"])
        self.pos_tol = float(m["pos_tol_m"])
        self.joint_tol = float(m["joint_tol_rad"])
        self.tool_rpy = [float(v) for v in cfg["tool_orientation_rad"]]
        self.ws = cfg["workspace"]
        self.tool_len = float(cfg["tool_length_m"])
        self.transit_tip_z = float(cfg["transit_tip_z_m"])
        self.j5_max = float(cfg["j5_max_rad"])
        self.time_scale = float(cfg["sim"]["speedup"]) if sim else 1.0
        self.arm = None
        self._estopped = threading.Event()
        self._soft_stop = threading.Event()
        self._soft_held = False   # 정지 요청 뒤 hold() 를 이미 보냈는가 (검사마다 다시 보내지 않게)
        self.jaw_yaw = None   # 집게가 닫히는 방향(로봇 xy 평면 각도, rad). None = 기본 자세 그대로
        self.tool_axis = None  # down_to/guarded_down 에서 공구가 가리킬 방향 (단위벡터). None = 수직(기울기 최소)
        fm = cfg.get("force_monitor", {})
        self.fm_on = bool(fm.get("enabled", False)) and not sim
        self.fm_thr0 = [float(v) for v in fm.get("threshold_nm", [2.0] * 6)]
        self.fm_thr = list(self.fm_thr0)
        self.fm_slow = float(fm.get("slow_speed_max_pct", 30))     # 이 속도 이하 = 물체 근처 → 기준 그대로
        self.fm_fast_k = float(fm.get("fast_scale", 2.5))          # 빠른 이동(공중) → 기준 × 이 배
        self.fm_tau = float(fm.get("baseline_s", 0.5))
        self.fm_hold = float(fm.get("persist_s", 0.1))
        self.fm_base = None
        self.fm_t = None
        self.fm_over_since = None
        self.fm_peak = [0.0] * 6     # 이번 동작 최대 편차 (로그용, 기준값 정하기)
        self.fm_log = []             # (동작, 관절별 최대 편차) — 기준값 정할 때 본다

    # ---------- 연결 ----------
    def connect(self, enable=True):
        port = self.cfg["can_port"]
        if self.sim:
            self.arm = SimPiper(self.cfg["sim"])
            log.info("[SIM] 실제 CAN 대신 시뮬레이터 사용")
        else:
            check_can(port)
            check_conflicts()
            try:
                from piper_sdk import C_PiperInterface_V2
            except ImportError:
                raise RobotError("piper_sdk 미설치: pip3 install piper_sdk  (pip 없으면 sudo apt install python3-pip)")
            try:
                self.arm = C_PiperInterface_V2(port)
            except Exception as e:
                raise CanError(f"piper_sdk 가 {port} 를 열지 못했습니다: {e}")
        self.arm.ConnectPort()

        # 피드백이 실제로 들어오는지 확인 (CAN 은 UP 인데 로봇 전원이 꺼진 경우를 잡는다)
        deadline = time.monotonic() + 3.0
        while self.arm.GetArmStatus().time_stamp == 0:
            if time.monotonic() > deadline:
                raise CanError(f"{port} 는 UP 인데 로봇 응답이 3초간 없습니다.\n"
                               "  로봇 전원, CAN 케이블, 다른 프로그램이 CAN 을 쓰는지 확인하세요.")
            time.sleep(0.05)
        st = self._status()
        log.info("로봇 응답 OK (ctrl_mode=%s, arm_status=0x%02X %s)", int(st.ctrl_mode),
                 int(st.arm_status), ARM_STATUS_TEXT.get(int(st.arm_status), "?"))
        if int(st.arm_status) == 0x01:
            raise RobotFault("로봇이 비상정지 상태입니다. 팔을 받친 상태로 `python3 mission.py --resume` 실행")
        if int(st.ctrl_mode) == 0x02:
            if enable:   # 티칭 모드에서는 동작 명령이 무시된다 → 타임아웃까지 매달리지 말고 바로 알린다
                raise RobotError("로봇이 티칭(드래그) 모드입니다 — 동작 명령이 무시됩니다.\n"
                                 "  팔을 원하는 곳에 둔 뒤 티칭 버튼을 다시 눌러 해제하고 재실행하세요")
            log.warning("로봇이 티칭 모드입니다 (좌표 읽기만 함)")
        if enable:
            self._enable()

    def _enable(self):
        deadline = time.monotonic() + 5.0
        while not self.arm.EnablePiper():
            if time.monotonic() > deadline:
                raise RobotFault("모터 enable 실패 (5초). 전원/비상정지 버튼/관절 통신 확인")
            time.sleep(0.05)
        self.arm.GripperCtrl(0, 1000, 0x02, 0)   # 그리퍼 에러 클리어
        time.sleep(0.05)
        log.info("모터 enable 완료")

    def disconnect(self):
        if self.arm is not None and hasattr(self.arm, "DisconnectPort"):
            try:
                self.arm.DisconnectPort()
            except Exception as e:
                log.debug("DisconnectPort: %s", e)

    def exit_teach(self, timeout=3.0):
        """티칭(드래그) 모드 해제: SDK MotionCtrl_1(grag_teach_ctrl=0x02 '示教记录结束/退出拖动示教模式')."""
        if int(self._status().ctrl_mode) != 0x02:
            log.info("티칭 모드가 아닙니다 (ctrl_mode=%d)", int(self._status().ctrl_mode))
            return
        end = time.monotonic() + timeout
        while int(self._status().ctrl_mode) == 0x02:
            if time.monotonic() > end:
                raise RobotError("티칭 모드 해제 실패 — 팔 끝의 티칭 버튼을 눌러 해제하세요")
            self.arm.MotionCtrl_1(0x00, 0x00, 0x02)
            time.sleep(0.2)
        log.info("티칭 모드 해제 (ctrl_mode=%d)", int(self._status().ctrl_mode))

    # ---------- 정지 ----------
    def estop(self):
        """비상정지 (SDK EmergencyStop(0x01)). 해제는 resume() — 해제 시 팔이 힘이 빠질 수 있다."""
        self._estopped.set()
        if self.arm is None:
            return
        for _ in range(3):
            try:
                self.arm.EmergencyStop(0x01)
            except Exception as e:
                log.error("EmergencyStop 전송 실패: %s", e)
            time.sleep(0.01)
        log.error("!!! 비상정지 전송 완료 !!!")

    def resume(self):
        """비상정지 해제 (SDK EmergencyStop(0x02) == ResetPiper: 모터 전원이 잠깐 빠진다)."""
        self.arm.EmergencyStop(0x02)
        self._estopped.clear()
        time.sleep(0.5)
        log.info("비상정지 해제 전송")

    def hold(self):
        """현재 위치를 목표로 다시 보내 그 자리에 세운다 (전원 유지, 낙하 없음). 섰으면 True.

        정지 명령을 못 보냈으면(CAN 오류 등) '섰다'고 가정하지 않고 실제 비상정지로 넘어간다.
        """
        if self.arm is None or self._estopped.is_set():
            return False
        try:
            x, y, z, rx, ry, rz = self.current_pose()
            for _ in range(3):
                self.arm.MotionCtrl_2(CTRL_CAN, MOVE_P, 10, 0x00)
                self.arm.EndPoseCtrl(m2sdk(x), m2sdk(y), m2sdk(z), rad2sdk(rx), rad2sdk(ry), rad2sdk(rz))
                time.sleep(0.02)
            log.warning("현재 위치 정지 (%.3f, %.3f, %.3f)", x, y, z)
            return True
        except Exception as e:
            log.error("hold 실패 (%s) → 실제 비상정지", e)
            self.estop()
            return False

    # ---------- 상태 ----------
    def _status(self):
        return self.arm.GetArmStatus().arm_status

    def soft_stop(self):
        """원격 정지 요청. 미션 스레드가 다음 검사(_check_fault, 10ms 간격)에서 그 자리에 서고 SoftStop 을 올린다."""
        self._soft_held = False
        self._soft_stop.set()

    def clear_soft_stop(self):
        """미션 스레드가 끝난 뒤에만 부른다 — 그 전에 지우면 삼켜진 요청이 사라진다."""
        self._soft_stop.clear()
        self._soft_held = False

    @property
    def soft_stopped(self):
        return self._soft_stop.is_set()

    def _check_fault(self, unreach=True):
        if self._estopped.is_set():
            raise RobotFault("비상정지됨")
        if self._soft_stop.is_set():
            # 요청은 지우지 않는다(clear_soft_stop 전까지 유지) — 누가 SoftStop 을 삼켜도 다음 검사에서 다시 걸린다
            if not self._soft_held:
                self._soft_held = True
                self.hold()
            raise SoftStop("원격 정지 요청 — 그 자리 정지")
        st = self._status()
        code = int(st.arm_status)
        if code in UNREACHABLE_STATUS and not unreach:
            return   # 이전 명령의 '도달 불가' 표시 — 새 명령을 보낸 뒤 상태가 갱신될 때까지 본다
        if code in UNREACHABLE_STATUS:
            raise RobotError(f"도달 불가 0x{code:02X}: {ARM_STATUS_TEXT.get(code, '?')} — 좌표/높이/자세(config) 확인")
        if code not in NON_FAULT_STATUS:
            raise RobotFault(f"로봇 상태 이상 0x{code:02X}: {ARM_STATUS_TEXT.get(code, '알 수 없음')}\n{st.err_status}")

    def _efforts(self):
        h = self.arm.GetArmHighSpdInfoMsgs()
        return [getattr(h, f"motor_{i}").effort / 1000.0 for i in range(1, 7)]

    def _fm_speed(self, speed_pct):
        """속도별 힘 기준: 천천히(물체 근처) = 기본, 빠르게(공중) = 기본 × fast_scale (가속 때 부하가 커서)."""
        k = 1.0 if speed_pct is None or speed_pct <= self.fm_slow else self.fm_fast_k
        self.fm_thr = [v * k for v in self.fm_thr0]

    def _fm_reset(self):
        self.fm_base, self.fm_t, self.fm_over_since = None, None, None
        self.fm_peak = [0.0] * 6

    def _check_force(self):
        """관절 부하(N·m)가 최근 평균(baseline_s)에서 threshold 넘게 persist_s 이상 벗어나면 ForceStop."""
        if not self.fm_on:
            return
        e = self._efforts()
        now = time.monotonic()
        if self.fm_base is None:
            self.fm_base, self.fm_t = list(e), now
            return
        dt = now - self.fm_t
        self.fm_t = now
        a = min(1.0, dt / self.fm_tau)
        dev = [abs(v - b) for v, b in zip(e, self.fm_base)]
        self.fm_peak = [max(p, d) for p, d in zip(self.fm_peak, dev)]
        over = [i for i in range(6) if dev[i] > self.fm_thr[i]]
        if over:
            self.fm_over_since = self.fm_over_since or now
            if now - self.fm_over_since >= self.fm_hold:
                j = over[0]
                self.fm_on = False   # hold 동작 중 재발 방지
                try:
                    self.hold()      # 펌웨어는 받은 목표로 계속 가므로, 감지 즉시 '여기서 멈춤'을 보낸다
                finally:
                    self.fm_on = True
                raise ForceStop(f"관절{j + 1} 부하 이상: {e[j]:+.2f} N·m (평소 {self.fm_base[j]:+.2f}, 기준 ±{self.fm_thr[j]:.1f}) "
                                f"— 어딘가에 닿은 것 같습니다")
        else:
            self.fm_over_since = None
            self.fm_base = [b + a * (v - b) for v, b in zip(e, self.fm_base)]   # 정상일 때만 평균 갱신

    def current_pose(self):
        e = self.arm.GetArmEndPoseMsgs().end_pose
        return (sdk2m(e.X_axis), sdk2m(e.Y_axis), sdk2m(e.Z_axis),
                sdk2rad(e.RX_axis), sdk2rad(e.RY_axis), sdk2rad(e.RZ_axis))

    def current_joints(self):
        j = self.arm.GetArmJointMsgs().joint_state
        return [sdk2rad(v) for v in (j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6)]

    def holding(self, min_width):
        """사과를 쥐고 있는가: 마지막 그리퍼 명령이 '닫기'이고 닫힌 폭이 min_width 이상.
        폭만 보면 열린 그리퍼(사과보다 넓다)도 '쥐고 있음'이 된다 (예전 safe_park / recover_held 오판)."""
        return bool(getattr(self, "grip_closed", False)) and self.gripper_width() >= min_width

    def gripper_width(self):
        return sdk2m(self.arm.GetArmGripperMsgs().gripper_state.grippers_angle)

    def wait(self, seconds):
        """고장 감시하면서 대기 (SIM 은 배속)."""
        end = time.monotonic() + seconds / self.time_scale
        while time.monotonic() < end:
            self._check_fault()
            time.sleep(0.02)

    # ---------- 동작 ----------
    def _run_until(self, send, done, timeout, what, near=None, progress=None, tick=None):
        """명령은 처음 SEND_BURST_S 동안만 보낸다 — 계속 보내면 PIPER 가 매번 궤적을 새로 시작해서 뚝뚝 끊긴다.
        멈춘 채(progress 변화 없음) STALL_RESEND_S 가 지나면 한 번 더 보낸다.
        도착: done() (엄격) 또는 near() 가 NEAR_HOLD_S 동안 유지 (로봇이 '도착'이라는데 몇 mm 모자랄 때)."""
        t0 = time.monotonic()
        last_send = -1.0
        last_prog, last_prog_t = None, t0
        near_since = None
        resend_until = t0 + SEND_BURST_S
        self._fm_reset()
        try:
            return self._run_loop(send, done, timeout, what, near, progress, t0, last_send, last_prog, last_prog_t,
                                  near_since, resend_until, tick)
        finally:
            if self.fm_on:
                log.debug("%s 부하 최대 편차 %s", what, " ".join(f"{v:.2f}" for v in self.fm_peak))
                self.fm_log.append((what, list(self.fm_peak)))

    def _run_loop(self, send, done, timeout, what, near, progress, t0, last_send, last_prog, last_prog_t,
                  near_since, resend_until, tick=None):
        while True:
            now = time.monotonic()
            self._check_fault(unreach=now - t0 >= 0.3)
            self._check_force()
            if tick is not None:
                tick()
            if now < resend_until and now - last_send >= RESEND_PERIOD_S:
                send()
                last_send = now
            if now - t0 >= MIN_MOVE_TIME_S:
                if done():
                    return now - t0
                if near is not None:
                    if near():
                        near_since = near_since or now
                        if now - near_since >= NEAR_HOLD_S / self.time_scale:
                            log.warning("%s: 로봇은 도착 판정, 목표와 약간 차이 — 도착으로 처리", what)
                            return now - t0
                    else:
                        near_since = None
            if progress is not None:
                pv = progress()
                if last_prog is None or abs(pv - last_prog) > 0.001:
                    last_prog, last_prog_t = pv, now
                elif now - last_prog_t > STALL_RESEND_S / self.time_scale and now >= resend_until:
                    log.debug("%s: 멈춤 → 재전송", what)
                    resend_until = now + SEND_BURST_S
                    last_prog_t = now
            if now - t0 > timeout:
                raise MotionTimeout(f"{what} 타임아웃 {timeout:.1f}s")
            time.sleep(0.01)

    def _check_ws(self, x, y, z):
        w = self.ws
        for name, v, (lo, hi) in (("x", x, w["x"]), ("y", y, w["y"]), ("z", z, w["z"])):
            if not lo <= v <= hi:
                raise RobotError(f"{name}={v:.3f}m 가 작업영역 [{lo}, {hi}] 밖입니다 (config.yaml workspace)")
        if math.hypot(x, y) > w.get("max_reach_xy_m", 9.0):
            raise RobotError(f"({x:.3f},{y:.3f}) 가 베이스에서 너무 멉니다 (수평 {math.hypot(x, y):.3f} > {w['max_reach_xy_m']}m)")
        if math.hypot(x, y) < w.get("min_reach_m", 0.0):
            raise RobotError(f"({x:.3f},{y:.3f}) 가 베이스에 너무 가깝습니다 (수평 반경 < {w['min_reach_m']}m)")
        if math.sqrt(x * x + y * y + z * z) > w["max_reach_m"]:
            raise RobotError(f"({x:.3f},{y:.3f},{z:.3f}) 가 도달 거리 {w['max_reach_m']}m 밖입니다")

    def move_to(self, x, y, z, speed_pct, linear=False, timeout=None):
        """말단을 (x,y,z)[m] 로. linear=True 면 직선(MOVE L), 아니면 MOVE P. 도착까지 블로킹."""
        self._check_ws(x, y, z)
        spd = int(max(1, min(100, round(speed_pct))))
        self._fm_speed(spd)
        mode = MOVE_L if linear else MOVE_P
        rx, ry, rz = self.tool_rpy_for_jaw()
        cmd = (m2sdk(x), m2sdk(y), m2sdk(z), rad2sdk(rx), rad2sdk(ry), rad2sdk(rz))
        target = (x, y, z)
        log.info("move_to (%.3f, %.3f, %.3f) %s %d%%", x, y, z, "L" if linear else "P", spd)

        def send():
            self.arm.MotionCtrl_2(CTRL_CAN, mode, spd, 0x00)
            self.arm.EndPoseCtrl(*cmd)

        def done():
            p = self.current_pose()[:3]
            return math.dist(p, target) <= self.pos_tol and int(self._status().motion_status) == 0

        def near():
            return math.dist(self.current_pose()[:3], target) <= NEAR_POS_M and int(self._status().motion_status) == 0

        try:
            self._run_until(send, done, timeout or self.timeout, f"move_to({x:.3f},{y:.3f},{z:.3f})",
                            near=near, progress=lambda: math.dist(self.current_pose()[:3], target))
        except MotionTimeout as e:
            p = self.current_pose()
            hint = " — 로봇이 티칭 모드로 바뀌었습니다 (티칭 버튼 해제)" if int(self._status().ctrl_mode) == 0x02 else ""
            raise MotionTimeout(f"{e} — 현재 ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}), "
                                f"남은 거리 {math.dist(p[:3], target) * 1000:.1f}mm{hint}")

    def move_joints(self, joints, speed_pct, timeout=None, tick=None, tol=None):
        """관절 각도[rad] 6개로 이동 (MOVE J). 도착까지 블로킹.
        tol: 도착 판정 관절 오차(rad)를 더 엄격하게 (상자 위처럼 정확히 서야 할 때). None = joint_tol_rad"""
        tol_done = self.joint_tol if tol is None else tol
        tol_near = NEAR_JOINT_RAD if tol is None else 2 * tol
        if len(joints) != 6:
            raise RobotError("관절 각도는 6개여야 합니다")
        for i, (v, (lo, hi)) in enumerate(zip(joints, JOINT_LIMITS)):
            if not lo - 1e-6 <= v <= hi + 1e-6:
                raise RobotError(f"joint{i + 1}={v:.3f}rad 가 한계 [{lo}, {hi}] 밖입니다")
        spd = int(max(1, min(100, round(speed_pct))))
        self._fm_speed(spd)
        cmd = [rad2sdk(v) for v in joints]
        log.info("move_joints [%s] %d%%", ", ".join(f"{v:.2f}" for v in joints), spd)

        def send():
            self.arm.MotionCtrl_2(CTRL_CAN, MOVE_J, spd, 0x00)
            self.arm.JointCtrl(*cmd)

        def done():
            cur = self.current_joints()
            return (max(abs(a - b) for a, b in zip(cur, joints)) <= tol_done
                    and int(self._status().motion_status) == 0)

        def near():
            cur = self.current_joints()
            return (max(abs(a - b) for a, b in zip(cur, joints)) <= tol_near
                    and int(self._status().motion_status) == 0)

        self._run_until(send, done, timeout or self.timeout, "move_joints", near=near,
                        progress=lambda: max(abs(a - b) for a, b in zip(self.current_joints(), joints)), tick=tick)

    def grip(self, open_, timeout=None, width=None):
        """그리퍼 열기(True)/닫기(False). 목표 폭 도달 또는 멈춤(사과에 막힘)까지 블로킹. 최종 폭[m] 반환.
        width: 열 때 폭을 직접 (벽 옆 사과는 필요한 만큼만 연다)."""
        g = self.cfg["gripper"]
        if width is None:
            width = float(g["open_width_m"] if open_ else g["close_width_m"])
        effort = int(round(float(g["effort_nm"]) * 1000))
        timeout = timeout or float(g["timeout_s"])
        log.info("grip %s (목표 %.1fmm, 힘 %.1fN·m)", "open" if open_ else "close", width * 1000, effort / 1000)
        # 마지막으로 보낸 그리퍼 명령. 열린 그리퍼도 폭은 넓으므로 '쥐고 있다'는 이것과 폭을 함께 본다 (holding)
        self.grip_closed = not open_
        t0 = time.monotonic()
        last_send = -1.0
        last_w, still_since = None, None
        self._fm_speed(None)     # 그리퍼: 기본 기준
        self._fm_reset()
        while True:
            self._check_fault()
            self._check_force()
            now = time.monotonic()
            if now - last_send >= RESEND_PERIOD_S:
                self.arm.GripperCtrl(abs(m2sdk(width)), effort, 0x01, 0)
                last_send = now
            gs = self.arm.GetArmGripperMsgs().gripper_state
            if getattr(gs.foc_status, "driver_error_status", False):
                raise RobotFault(f"그리퍼 드라이버 에러: {gs.foc_status}")
            w = sdk2m(gs.grippers_angle)
            if now - t0 >= MIN_MOVE_TIME_S:
                if abs(w - width) <= 0.002:
                    return w
                # 0.3초 동안 0.5mm 이상 안 움직이면 멈춘 것 (사과를 쥠)
                if last_w is not None and abs(w - last_w) < 0.0005:
                    still_since = still_since or now
                    if now - still_since >= 0.3 / self.time_scale:
                        return w
                else:
                    still_since = None
            last_w = w
            if now - t0 > timeout:
                raise MotionTimeout(f"grip 타임아웃 {timeout:.1f}s (현재 폭 {w * 1000:.1f}mm)")
            time.sleep(0.01)

    # ---------- 집게 방향 (벽·이웃 사과 피하기) ----------
    def _jaw_axis_world(self, R):
        """공구 회전행렬 → 집게가 닫히는 방향의 로봇 xy 평면 각도."""
        ax = R[:, 1] if self.cfg["gripper"].get("jaw_axis_tool", "y") == "y" else R[:, 0]
        return math.atan2(ax[1], ax[0])

    def tool_rpy_for_jaw(self):
        if self.jaw_yaw is None:
            return self.tool_rpy
        import numpy as np
        import ik
        R0 = ik._rot(*self.tool_rpy)
        d = self.jaw_yaw - self._jaw_axis_world(R0)
        d = (d + math.pi / 2) % math.pi - math.pi / 2          # 집게는 180° 대칭 → 가장 작은 회전
        c, s_ = math.cos(d), math.sin(d)
        R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]]) @ R0   # 수직축으로 돌린다
        ry = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
        return rx, ry, rz

    def _q6_for_jaw(self, q, yaw, ref=None):
        """joint6 만 돌려서 집게 방향을 yaw 에 맞춘다. 같은 방향(180° 대칭) 중 ref(지금 손목)에 가장 가까운 값."""
        ref = q[5] if ref is None else ref
        import ik
        best = None
        lo, hi = -2.09, 2.09
        for k in range(0, 360, 2):
            q6 = lo + (hi - lo) * k / 359
            qq = list(q[:5]) + [q6]
            p = ik._FK().CalFK(qq)[-1]
            a = self._jaw_axis_world(ik._rot(*[math.radians(v) for v in p[3:]]))
            err = abs((a - yaw + math.pi / 2) % math.pi - math.pi / 2)
            if best is None or err < best[0] - 0.02 or (abs(err - best[0]) <= 0.02 and abs(q6 - ref) < abs(best[1] - ref)):
                best = (err, q6)
        return list(q[:5]) + [best[1]]

    # ---------- 수직 자세 높이 한계 ----------
    # 그리퍼 수직으로 닿는 손가락끝 최대 높이 vs 베이스 수평 거리 (ik.py 계산, joint5 ≤ j5_max)
    # 펌웨어 MOVE L(수직)을 써도 되는 범위. 실측 19:14: r=0.415 손가락끝 0.072 에서 '한계 초과' → 보수적으로.
    # 이 밖은 down_to 가 관절 계산(ik.py)으로 간다 — 닿지 않는 게 아니다.
    VERT_REACH = [(0.33, 0.10), (0.37, 0.08), (0.40, 0.05), (0.41, 0.03)]

    def max_vertical_tip_z(self, x, y):
        r = math.hypot(x, y)
        pts = self.VERT_REACH
        if r <= pts[0][0]:
            return pts[0][1]
        for (r0, z0), (r1, z1) in zip(pts, pts[1:]):
            if r <= r1:
                return z0 + (z1 - z0) * (r - r0) / (r1 - r0)
        return None

    def lift_z(self, x, y, want=None):
        """수직 상승 목표 플랜지 높이 (z_safe_m). 멀어서 수직이 안 되면 down_to 가 관절 계산으로 간다."""
        z = float(self.cfg["z_safe_m"])
        return z if want is None else min(z, want)

    def down_to(self, x, y, z_flange, speed_pct):
        """(x, y) 에서 높이만 바꾼다 (내려가기/올라가기).
        ik.py 로 먼저 풀어 보고, 그리퍼가 완전히 수직이고 joint5 여유가 있으면 펌웨어 직선(MOVE L),
        아니면(베이스 가까이·멀리) 구한 관절로 JointCtrl (최소 기울기)."""
        tip = z_flange - self.tool_len
        self._check_ws(x, y, z_flange)
        q, tilt = self._ik(x, y, tip, axis=self.tool_axis)
        # 집게 방향이나 기울기를 정했으면 항상 관절 명령 — 펌웨어 IK 는 손목(joint6)을 엉뚱한 쪽으로 크게 돌린다 (21:33 실측 170°)
        if self.jaw_yaw is None and self.tool_axis is None and tilt < 1.0 and abs(q[4]) <= float(self.cfg.get("j5_firmware_max_rad", 1.10)):
            return self.move_to(x, y, z_flange, speed_pct, linear=True)
        if self.jaw_yaw is not None:
            q = self._q6_for_jaw(q, self.jaw_yaw, ref=self.current_joints()[5])
        (log.warning if tilt > 15 else log.info)("down_to (%.3f, %.3f) 손가락끝 z %.3f — 관절 계산, 기울기 %.0f°",
                                                 x, y, tip, tilt)
        self.move_joints(q, speed_pct)

    def guarded_down(self, x, y, z_flange, speed_pct, thr_nm, skip_s=0.35, persist_s=0.06):
        """(x, y) 에서 z_flange 까지 천천히 내려가다가 '닿으면'(관절2·3 부하가 내려가는 동안의 평소 값에서
        thr_nm 넘게 바뀌면) 그 자리에 멈춘다. 반환 ('contact' | 'floor', 멈춘 플랜지 z).
        사과를 쌓을 때: 아래 사과(또는 바닥)에 닿은 곳에서 놓는다 — 개수로 높이를 짐작하지 않는다."""
        import ik
        q, _ = self._ik(x, y, z_flange - self.tool_len, axis=self.tool_axis)
        if self.jaw_yaw is not None:
            q = self._q6_for_jaw(q, self.jaw_yaw, ref=self.current_joints()[5])
        t0 = time.monotonic()
        base, since = None, None
        samples = []

        def tick():
            nonlocal base, since
            now = time.monotonic()
            e = self._efforts()
            if now - t0 < skip_s:          # 출발 가속 구간은 안 본다
                return
            if base is None:
                samples.append(e)
                if len(samples) >= 5:      # 등속 하강 중 평소 부하
                    base = [sorted(c)[len(c) // 2] for c in zip(*samples)]
                return
            dev = max(abs(e[j] - base[j]) for j in (1, 2))
            if dev > thr_nm:
                since = since or now
                if now - since >= persist_s:
                    raise _Contact()
            else:
                since = None

        fm = self.fm_on
        self.fm_on = False                 # 접촉 감지가 대신한다 (같은 신호로 비상정지하면 안 된다)
        try:
            self.move_joints(q, speed_pct, tick=tick)
            return "floor", self.current_pose()[2]
        except _Contact:
            self.hold()
            z = self.current_pose()[2]
            log.info("닿음: 플랜지 z %.3f (목표 %.3f 보다 %.0fmm 위)", z, z_flange, (z - z_flange) * 1000)
            return "contact", z
        finally:
            self.fm_on = fm

    # ---------- 높은 곳 이동 (관절 직접) ----------
    # 그리퍼를 수직으로 세운 채로는 손가락끝 ~0.10m 위로 못 올라간다 (joint5 한계).
    # 상자 벽(0.14m)을 넘는 이동은 ik.py 로 관절 각도를 직접 구해 JointCtrl 로 보낸다 (약간 기울어짐).

    def _ik(self, x, y, ztip, axis=None):
        import ik
        seeds = [self.current_joints()] + ik.SEEDS
        r = ik.solve((x, y, ztip), self.tool_len, self.j5_max, seeds=seeds, axis=axis)
        if r is None:
            raise RobotError(f"관절 해 없음: 손가락끝 ({x:.3f},{y:.3f},{ztip:.3f}) — 너무 멀거나 높음")
        return list(r[0]), r[1]

    def _path_pts(self, q):
        """관절 자세 → 팔 각 마디 + 손가락끝 위치들 (충돌 점검용)."""
        import ik
        pts = [(p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0) for p in ik._FK().CalFK(list(q))]
        tip = ik.fk_tip(q, self.tool_len)[0]
        return pts + [tuple(tip)]

    def _seg_check(self, qa, qb, n=14):
        """qa→qb 관절 보간을 점검: (손가락끝 최저 z, 금지 구역 침범 여부)."""
        import ik
        keep = self.cfg.get("keepout_xy_m") or []
        zmin, hit = 9.0, None
        for k in range(n + 1):
            q = [a + (b - a) * k / n for a, b in zip(qa, qb)]
            zmin = min(zmin, ik.fk_tip(q, self.tool_len)[0][2])
            for (px, py, pz) in self._path_pts(q):
                for kx, ky, kr in keep:
                    if math.hypot(px - kx, py - ky) < kr:
                        hit = (kx, ky)
        return zmin, hit

    def _tip_xy(self, q):
        import ik
        return ik.fk_tip(q, self.tool_len)[0][:2]

    def _plan(self, qa, qb, depth=0):
        """qa→qb 사이에 손가락끝이 이동 높이 밑으로 처지거나 금지 구역(삼각대 등)에 닿으면 가운데 경유점을 넣는다.
        금지 구역 때문이면 경유점을 베이스 쪽(transit_inner_r_m)으로 당긴다."""
        zmin, hit = self._seg_check(qa, qb)
        if zmin >= self.transit_tip_z - 0.01 and hit is None:
            return [qb]
        if depth >= 4:
            if hit is not None:
                raise RobotError(f"이동 경로가 금지 구역 {hit} 에 닿습니다 — 경로를 못 찾음 (keepout_xy_m 확인)")
            return [qb]
        (xa, ya), (xb, yb) = self._tip_xy(qa), self._tip_xy(qb)
        mx, my = (xa + xb) / 2, (ya + yb) / 2
        if hit is not None:
            r_in = float(self.cfg.get("transit_inner_r_m", 0.28))
            r = math.hypot(mx, my)
            if r > r_in:
                mx, my = mx * r_in / r, my * r_in / r
        qm, _ = self._ik(mx, my, self.transit_tip_z)
        return self._plan(qa, qm, depth + 1) + self._plan(qm, qb, depth + 1)

    def _inner(self, x, y):
        """먼 곳은 같은 방향 안쪽 경유점 (팔이 크게 휘돌지 않게)."""
        r = math.hypot(x, y)
        r_far, r_in = float(self.cfg.get("transit_far_r_m", 0.36)), float(self.cfg.get("transit_inner_r_m", 0.28))
        return (x * r_in / r, y * r_in / r) if r > r_far else None

    def transit_to(self, x, y, speed_pct, final_tol=None):
        """이동 높이(손가락끝 transit_tip_z_m)에서 (x, y) 위로. 제자리 상승 → (먼 곳이면 안쪽 경유) → 목표.
        구간마다 처짐·금지 구역을 순기구학으로 점검. 도착까지 블로킹."""
        r = math.hypot(x, y)
        r_min = float(self.cfg.get("transit_min_r_m", 0.16))
        if 1e-6 < r < r_min:   # 베이스 바로 위 높은 곳은 많이 기울어진다 → 바깥쪽에 멈추고 down_to 가 비스듬히 내려간다
            x, y = x * r_min / r, y * r_min / r
        self._check_ws(x, y, self.transit_tip_z)   # 도달 여부는 IK 가 판단 (3D 반경 점검은 손가락끝 기준)
        q0 = self.current_joints()
        x0, y0 = self._tip_xy(q0)
        r0 = math.hypot(x0, y0)
        if 1e-6 < r0 < r_min:   # 베이스 가까이서는 바깥쪽으로 비켜서 올라간다
            x0, y0 = x0 * r_min / r0, y0 * r_min / r0
        q_up, t_up = self._ik(x0, y0, self.transit_tip_z)
        stops = [p for p in (self._inner(x0, y0), self._inner(x, y)) if p is not None] + [(x, y)]
        path, q_prev = [q_up], q_up
        for sx, sy in stops:
            q_s, t_goal = self._ik(sx, sy, self.transit_tip_z)
            seg = self._plan(q_prev, q_s)
            path += seg
            q_prev = q_s
        log.info("transit_to (%.3f, %.3f) 손가락끝 z %.2f, 경유 %d점, 기울기 %.0f°→%.0f°",
                 x, y, self.transit_tip_z, len(path), t_up, t_goal)
        for k, q in enumerate(path):
            self.move_joints(q, speed_pct, tol=final_tol if k == len(path) - 1 else None)

    def go_home(self, speed_pct=None):
        """대기 자세 복귀: 낮게 있으면 먼저 수직 상승(z_safe_m) → 이동 높이로 home_xy 위.
        관절 0(접힌 자세)으로 가지 않는다 — 긴 그리퍼가 몸체를 친다."""
        spd = speed_pct or self.cfg["motion"]["transit_speed_pct"]
        x, y, z = self.current_pose()[:3]
        z_lift = self.lift_z(x, y)
        if z < z_lift - self.pos_tol:
            try:
                self._check_ws(x, y, z_lift)
            except RobotError as e:
                log.warning("상승 생략 (%s)", e)
            else:
                self.down_to(x, y, z_lift, spd)
        hx, hy = (float(v) for v in self.cfg["home_xy_m"])
        self.transit_to(hx, hy, spd)
        log.info("홈 복귀 완료")


class SimPiper:
    """C_PiperInterface_V2 에서 쓰는 메서드만 같은 이름·반환 구조로 흉내. CAN 대신 로그만 찍는다."""

    MAX_LIN_MPS = 0.5      # 속도 100% 기준
    MAX_JOINT_RPS = 1.5
    GRIP_MPS = 0.1

    def __init__(self, sim_cfg):
        import random
        self.c = sim_cfg
        self.rng = random.Random(sim_cfg["seed"])
        self.k = float(sim_cfg["speedup"])
        x, y, z = sim_cfg["start_pose_m"]
        self.pose = [m2sdk(x), m2sdk(y), m2sdk(z), 0, 85000, 0]
        self.joints = [0] * 6
        self.grip = 0
        self.t_pose = self.t_joints = None
        self.t_grip = 0
        self.grip_stop = None      # 닫을 때 사과에 막히는 폭
        self.mode, self.spd = MOVE_P, 50
        self.estop = False
        self.moving = False
        self.t_last = time.monotonic()
        self.t_start = None
        self.slog = logging.getLogger("sim")
        self.unreach = False
        self.tool_len = float(sim_cfg.get("tool_length_m", 0.132))
        try:   # piper_sdk 순기구학이 있으면 자세를 관절에서 계산 (실기와 같은 관계)
            import ik
            ik._FK()
            self.fk_ok = True
            self.joints = [rad2sdk(v) for v in ik.SEEDS[0]]
            self._pose_from_joints()
        except ImportError:
            self.fk_ok = False

    # --- SDK 흉내 ---
    def ConnectPort(self, *a, **k):
        self.t_start = time.monotonic()
        self.slog.info("[SIM] ConnectPort()")

    def DisconnectPort(self, *a, **k):
        self.slog.info("[SIM] DisconnectPort()")

    def EnablePiper(self):
        return True

    def MotionCtrl_1(self, emergency_stop=0, track_ctrl=0, grag_teach_ctrl=0):
        if emergency_stop == 0x01:
            self.estop = True
        elif emergency_stop == 0x02:
            self.estop = False
        self.slog.info("[SIM] MotionCtrl_1(emergency_stop=0x%02X)", emergency_stop)

    def EmergencyStop(self, emergency_stop=0):
        self.MotionCtrl_1(emergency_stop, 0, 0)

    def MotionCtrl_2(self, ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50, is_mit_mode=0x00, *a, **k):
        self._step()
        self.mode, self.spd = move_mode, move_spd_rate_ctrl

    def EndPoseCtrl(self, X, Y, Z, RX, RY, RZ):
        t = [X, Y, Z, RX, RY, RZ]
        if t != self.t_pose:
            self.slog.debug("[SIM] EndPoseCtrl%s mode=%d spd=%d", tuple(t), self.mode, self.spd)
            self.t_pose, self.t_joints = t, None
            if self.fk_ok and math.dist([v / 1e6 for v in t[:3]], [v / 1e6 for v in self.pose[:3]]) < 0.003:
                self.t_pose, self.unreach = None, False     # hold (지금 자리) — 기울어진 자세 그대로 선다
                return
            if self.fk_ok:   # 실기처럼 관절로 움직인다: 펌웨어 IK 대신 ik.solve (그리퍼 수직 가정)
                import ik
                r = ik.solve((sdk2m(X), sdk2m(Y), sdk2m(Z) - self.tool_len), self.tool_len, 1.22,
                             seeds=[[sdk2rad(v) for v in self.joints]] + ik.SEEDS)
                if r is None or r[1] > 3.0:
                    self.slog.warning("[SIM] 펌웨어라면 '도달 불가' — 수직 자세 해 없음 (%.3f, %.3f, %.3f)",
                                      sdk2m(X), sdk2m(Y), sdk2m(Z))
                    self.unreach = True
                else:
                    self.unreach = False
                    self.t_pose, self.t_joints = None, [rad2sdk(v) for v in r[0]]

    def JointCtrl(self, *j):
        j = list(j)
        if j != self.t_joints:
            self.slog.debug("[SIM] JointCtrl%s spd=%d", tuple(j), self.spd)
            self.t_joints, self.t_pose = j, None

    def GripperCtrl(self, gripper_angle=0, gripper_effort=0, gripper_code=0, set_zero=0):
        self._step()
        if gripper_angle != self.t_grip:
            self.slog.debug("[SIM] GripperCtrl(%d, %d, 0x%02X)", gripper_angle, gripper_effort, gripper_code)
            if gripper_angle < self.grip:   # 닫기 시작: 사과가 있는지 결정
                miss = self.rng.random() < self.c["miss_prob"]
                self.grip_stop = None if miss else m2sdk(self.c["apple_width_m"])
            self.t_grip = gripper_angle

    def _pose_from_joints(self):
        import ik
        _, _, fl = ik.fk_tip([sdk2rad(v) for v in self.joints], self.tool_len)
        p = ik._FK().CalFK([sdk2rad(v) for v in self.joints])[-1]
        self.pose = [m2sdk(fl[0]), m2sdk(fl[1]), m2sdk(fl[2])] + [round(v * 1000) for v in p[3:]]

    def GetArmStatus(self):
        self._step()
        code = 0x01 if self.estop else (0x02 if self.unreach else 0x00)
        return NS(time_stamp=time.time(), Hz=200.0, arm_status=NS(
            ctrl_mode=1, arm_status=code, mode_feed=self.mode,
            motion_status=1 if self.moving else 0, err_status="(sim)"))

    def GetArmEndPoseMsgs(self):
        self._step()
        p = self.pose
        return NS(time_stamp=time.time(), end_pose=NS(X_axis=p[0], Y_axis=p[1], Z_axis=p[2],
                                                      RX_axis=p[3], RY_axis=p[4], RZ_axis=p[5]))

    def GetArmJointMsgs(self):
        self._step()
        j = self.joints
        return NS(time_stamp=time.time(), joint_state=NS(joint_1=j[0], joint_2=j[1], joint_3=j[2],
                                                         joint_4=j[3], joint_5=j[4], joint_6=j[5]))

    def GetArmHighSpdInfoMsgs(self):
        self._step()
        m = NS(motor_speed=0, current=0, pos=0, effort=0.0)
        return NS(time_stamp=time.time(), Hz=100.0, **{f"motor_{i}": m for i in range(1, 7)})

    def GetArmGripperMsgs(self):
        self._step()
        return NS(time_stamp=time.time(), gripper_state=NS(
            grippers_angle=round(self.grip), grippers_effort=0, foc_status=NS(driver_error_status=False)))

    # --- 물리 흉내: 목표를 향해 일정 속도로 이동 ---
    def _step(self):
        now = time.monotonic()
        dt = now - self.t_last
        self.t_last = now
        if self.estop or self.c["stall"]:
            self.moving = self.t_pose is not None or self.t_joints is not None
            return
        frac = max(self.spd, 1) / 100.0
        self.moving = False
        if self.t_pose is not None:
            d = [self.t_pose[i] - self.pose[i] for i in range(3)]
            dist = math.sqrt(sum(v * v for v in d))
            step = m2sdk(self.MAX_LIN_MPS) * frac * self.k * dt
            if dist <= step:
                self.pose[:3] = self.t_pose[:3]
            else:
                self.pose[:3] = [self.pose[i] + d[i] * step / dist for i in range(3)]
                self.moving = True
            self.pose[3:] = self.t_pose[3:]
        if self.t_joints is not None:
            step = rad2sdk(self.MAX_JOINT_RPS) * frac * self.k * dt
            for i in range(6):
                d = self.t_joints[i] - self.joints[i]
                if abs(d) <= step:
                    self.joints[i] = self.t_joints[i]
                else:
                    self.joints[i] += math.copysign(step, d)
                    self.moving = True
            if self.fk_ok:
                self._pose_from_joints()
        target = self.t_grip
        if target < self.grip and self.grip_stop is not None:
            target = max(target, self.grip_stop)
        step = m2sdk(self.GRIP_MPS) * self.k * dt
        d = target - self.grip
        self.grip = target if abs(d) <= step else self.grip + math.copysign(step, d)
