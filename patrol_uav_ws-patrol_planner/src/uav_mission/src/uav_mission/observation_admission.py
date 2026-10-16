"""ROS-independent source-time and generation guards for ground map rebuild."""
import math


def ready_header(stamp, now, frame, expected_frame, max_age=.35):
    return (frame == expected_frame and math.isfinite(stamp) and
            math.isfinite(now) and stamp > 0 and 0 <= now-stamp <= max_age)


def rebuild_allowed(phase, has_active_action):
    return not has_active_action and phase in ('IDLE', 'COMPLETE', 'ABORTED')


def rebuild_completed(stamp, now, frame, expected_frame, generation,
                      previous_generation, started_at, max_age=.35):
    return (generation > previous_generation and stamp > started_at and
            ready_header(stamp, now, frame, expected_frame, max_age))
