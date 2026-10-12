"""Per-ALIGN timeout policy using ROS time and fresh FC AGL samples."""
from collections import deque
import math


class DropTimeout:
    def __init__(self, lock_timeout=20., rate_threshold=.15, stall_timeout=5., sample_gap=.5,
                 stall_start_height=.60):
        values = (lock_timeout, rate_threshold, stall_timeout, sample_gap, stall_start_height)
        if not all(math.isfinite(v) and v > 0 for v in values) or rate_threshold > 1:
            raise ValueError("drop timeout values must be finite and positive; rate <= 1")
        self.lock_timeout, self.rate_threshold = lock_timeout, rate_threshold
        self.stall_timeout, self.sample_gap = stall_timeout, sample_gap
        self.stall_start_height = stall_start_height
        self.reset()

    def reset(self):
        self.key = None
        self.started = self.last_now = None
        self.samples = deque()
        self.low_since = None
        self.reason = ""
        self.clock_invalid = False

    def bind(self, key, now):
        if key != self.key:
            self.reset()
            self.key, self.started, self.last_now = key, now, now

    def update(self, now, stamp=None, height=None):
        if self.key is None or self.clock_invalid:
            return ""
        if not math.isfinite(now) or now < self.last_now:
            self.clock_invalid = True
            self.reason = ""
            return ""
        self.last_now = now
        if self.reason:
            return self.reason
        if now - self.started >= self.lock_timeout:
            self.reason = "forced_drop_lock_timeout"
            return self.reason
        if (stamp is None or height is None or not math.isfinite(stamp) or
                not math.isfinite(height) or height <= .05 or
                not 0 <= now - stamp <= self.sample_gap or stamp < self.started):
            self.samples.clear()
            self.low_since = None
            return ""
        if self.samples and stamp <= self.samples[-1][0]:
            return ""  # Duplicate/out-of-order samples cannot advance stall time.
        if self.samples and stamp - self.samples[-1][0] > self.sample_gap:
            self.samples.clear()
            self.low_since = None
        self.samples.append((stamp, height))
        # Keep 1s history, but stall dwell only exists strictly below this AGL.
        if height >= self.stall_start_height:
            self.low_since = None
        cutoff = stamp - 1.
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        if self.samples[0][0] > cutoff or len(self.samples) < 2:
            return ""
        t0, h0 = self.samples[0]
        t1, h1 = self.samples[1]
        reference = h0 + (h1 - h0) * (cutoff - t0) / (t1 - t0)
        rate = abs(height - reference) / reference
        if height < self.stall_start_height and rate < self.rate_threshold and not math.isclose(
                rate, self.rate_threshold, rel_tol=0., abs_tol=1e-12):
            if self.low_since is None:
                self.low_since = stamp
            if stamp - self.low_since > self.stall_timeout:
                self.reason = "forced_drop_height_stall"
        else:
            self.low_since = None
        return self.reason
