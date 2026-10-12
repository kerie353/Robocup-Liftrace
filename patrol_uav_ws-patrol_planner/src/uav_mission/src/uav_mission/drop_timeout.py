"""Per-ALIGN lock timeout using ROS time; no height-based interruption."""
import math


class DropTimeout:
    def __init__(self, lock_timeout=20.):
        if not math.isfinite(lock_timeout) or lock_timeout <= 0:
            raise ValueError("drop lock timeout must be finite and positive")
        self.lock_timeout = lock_timeout
        self.reset()

    def reset(self):
        self.key = None
        self.started = self.last_now = None
        self.reason = ""
        self.clock_invalid = False

    def bind(self, key, now):
        if key != self.key:
            self.reset()
            self.key, self.started, self.last_now = key, now, now

    def update(self, now):
        if self.key is None or self.clock_invalid:
            return ""
        if not math.isfinite(now) or now < self.last_now:
            self.clock_invalid = True
            self.reason = ""
            return ""
        self.last_now = now
        if not self.reason and now - self.started >= self.lock_timeout:
            self.reason = "forced_drop_lock_timeout"
        return self.reason
