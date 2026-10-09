"""관절 각도 직접 계산 (SDK 순기구학 C_PiperForwardKinematics + 수치 역기구학).

높은 곳에서 그리퍼를 수직으로 세우면 joint5 가 한계(±70°)를 넘는다 (실측: 플랜지 0.272m 에서 75°).
펌웨어 IK(EndPoseCtrl)에 맡기면 "목표 각도 한계 초과"로 멈추므로, 이동 높이의 경유점은
여기서 관절 각도를 직접 구해 JointCtrl 로 보낸다:
  - 손가락 끝 위치를 정확히 맞추고
  - joint5 는 한계 안쪽(j5_max)으로 제한하고
  - 그 안에서 그리퍼 기울기를 최소로 한다.
시작점(seed)은 실측한 관절 자세라서 펌웨어와 같은 자세 계열(팔꿈치 위)에 머문다.
"""
import math

import numpy as np

JOINT_LIMITS = np.array([(-2.6179, 2.6179), (0.0, 3.14), (-2.967, 0.0),
                         (-1.745, 1.745), (-1.22, 1.22), (-2.09439, 2.09439)])

# 실측 관절 자세 (모두 그리퍼 아래 방향, 팔꿈치 위) — 수치 해의 시작점
SEEDS = [
    [-0.221, 1.815, -0.927, -0.070, 0.837, 0.102],   # 트레이 바닥 (0.282, -0.068, 0.132)
    [0.888, 1.543, -0.577, -0.048, 0.641, 0.273],    # 상 칸 바닥 (0.136, 0.163, 0.136)
    [0.549, 1.902, -1.072, 0.000, 0.748, 0.273],     # 중 칸 바닥 (0.286, 0.175, 0.134)
    [0.250, 1.272, -0.933, 0.018, 1.150, -0.065],    # 상자 벽 위 (j5 를 한계 안으로 줄인 값)
]

_fk = None


def _FK():
    global _fk
    if _fk is None:
        from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
        _fk = C_PiperForwardKinematics()
    return _fk


def _rot(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


def fk_tip(q, tool_len):
    """관절[rad] → (손가락 끝 xyz[m], 공구 축(그리퍼 방향) 단위벡터, 플랜지 xyz[m])."""
    p = _FK().CalFK(list(q))[-1]
    a = _rot(*np.radians(p[3:]))[:, 2]
    flange = np.array(p[:3]) / 1000.0
    return flange + tool_len * a, a, flange


def tilt_deg(axis):
    return math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))


def solve(tip_xyz, tool_len, j5_max=1.20, seeds=None, iters=300):
    """손가락 끝을 tip_xyz 에 두는 관절 6개. joint5 ≤ j5_max, 그 안에서 기울기 최소.
    위치는 정확히 맞추고(주 과제), 기울기는 위치를 안 건드리는 방향(영공간)으로만 줄인다.
    반환 (q, tilt_deg) 또는 None (위치 오차 2mm 이내 해 없음)."""
    t = np.asarray(tip_xyz, float)
    lim = JOINT_LIMITS.copy()
    lim[4] = (-j5_max, j5_max)
    down = np.array([0.0, 0.0, -1.0])
    best = None
    for s in (seeds or SEEDS):
        q = np.clip(np.array(s, float), lim[:, 0], lim[:, 1])
        for _ in range(iters):
            p, a, _ = fk_tip(q, tool_len)
            Jp = np.zeros((3, 5))
            ga = np.zeros(5)                      # 기울기 비용 |a - down|² 의 기울기
            for k in range(5):                    # joint6 은 손목 회전 — 위치·기울기와 무관
                d = np.zeros(6)
                d[k] = 1e-6
                p2, a2, _ = fk_tip(q + d, tool_len)
                Jp[:, k] = (p2 - p) / 1e-6
                ga[k] = (np.sum((a2 - down) ** 2) - np.sum((a - down) ** 2)) / 1e-6
            # 한계에 붙은 관절은 그 방향으로 못 움직이게 뺀다
            free = np.array([not ((q[k] <= lim[k, 0] + 1e-4 and ga[k] > 0) or
                                  (q[k] >= lim[k, 1] - 1e-4 and ga[k] < 0)) for k in range(5)], float)
            Jf = Jp * free
            Jpinv = np.linalg.pinv(Jf, rcond=1e-4)
            dq = Jpinv @ (t - p)                                   # 위치
            dq += (np.eye(5) - Jpinv @ Jf) @ (-0.3 * ga * free)    # 위치 유지하며 기울기 줄이기
            n = np.linalg.norm(dq)
            if n > 0.1:
                dq *= 0.1 / n
            q[:5] = np.clip(q[:5] + dq, lim[:5, 0], lim[:5, 1])
            if n < 1e-6:
                break
        p, a, _ = fk_tip(q, tool_len)
        if np.linalg.norm(p - t) < 0.002:
            tl = tilt_deg(a)
            if best is None or tl < best[1]:
                best = (q.copy(), tl)
    return best
