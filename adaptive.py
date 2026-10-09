"""굴림 신호 기반 적응형 속도·놓는 높이 조정."""
import logging

log = logging.getLogger("adaptive")


class AdaptiveTuner:
    """굴림 → 하강 속도·놓는 높이 down_ratio 만큼 하향 (하한 있음).
    연속 감속 max_consecutive_down 회 → 자동 조정 중단 (그 값으로 고정).
    연속 성공 success_streak_for_up 회 → up_ratio 만큼 기준값 쪽으로 복구.
    """

    def __init__(self, cfg):
        a = cfg["adaptive"]
        self.enabled = bool(a["enabled"])
        self.down = float(a["down_ratio"])
        self.up = float(a["up_ratio"])
        self.streak_for_up = int(a["success_streak_for_up"])
        self.max_down = int(a["max_consecutive_down"])
        self.min_speed = float(a["min_speed_pct"])
        self.min_release = float(a["min_release_height_m"])
        self.base_release = float(cfg["place"]["release_height_m"])
        self.scale = 1.0                 # 하강 속도 배율 (기준값 대비)
        self.release_h = self.base_release
        self.down_streak = 0
        self.success_streak = 0
        self.frozen = False

    def speed(self, base_pct):
        """기준 속도에 현재 배율 적용, 하한 보장."""
        return max(self.min_speed, base_pct * self.scale)

    def on_roll(self):
        self.success_streak = 0
        if not self.enabled or self.frozen:
            log.warning("굴림 신호 — 자동 조정 %s, 현재값 유지 (배율 %.2f, 놓는 높이 %.3fm)",
                        "꺼짐" if not self.enabled else "중단 상태", self.scale, self.release_h)
            return
        self.scale *= (1 - self.down)
        self.release_h = max(self.min_release, self.release_h * (1 - self.down))
        self.down_streak += 1
        log.warning("굴림 신호 → 감속 %d회째: 속도 배율 %.2f, 놓는 높이 %.3fm",
                    self.down_streak, self.scale, self.release_h)
        if self.down_streak >= self.max_down:
            self.frozen = True
            log.error("연속 %d회 감속 → 자동 조정 중단. 그리퍼 힘/놓는 위치/상자 높이를 점검하고 config.yaml 을 조정하세요",
                      self.down_streak)

    def on_success(self):
        self.down_streak = 0
        self.success_streak += 1
        if not self.enabled or self.frozen:
            return
        if self.success_streak >= self.streak_for_up and (self.scale < 1.0 or self.release_h < self.base_release):
            self.scale = min(1.0, self.scale * (1 + self.up))
            self.release_h = min(self.base_release, self.release_h * (1 + self.up))
            self.success_streak = 0
            log.info("연속 성공 → 소폭 복구: 속도 배율 %.2f, 놓는 높이 %.3fm", self.scale, self.release_h)
