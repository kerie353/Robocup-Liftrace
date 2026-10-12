"""Delivery-only descent settings; no H, actuator, route or PX4 parameter changes."""
import math


def drop_descent_parameters(settings):
    mode = settings.get('drop_descent_mode', 1)
    speed = settings.get('drop_descent_speed_mps', 0.20)
    if type(mode) is not int or mode not in (1, 2, 3):
        raise ValueError('drop_descent_mode must be 1, 2 or 3')
    if (isinstance(speed, bool) or not isinstance(speed, (int, float)) or
            not math.isfinite(speed) or not 0.0 < speed <= 1.0):
        raise ValueError('drop_descent_speed_mps must be in (0, 1]')
    return dict(mode=mode, speed_mps=float(speed), max_step_dt_sec=0.20,
                zero_velocity_setpoint_topic='/navigation/drop_setpoint_vertical')
