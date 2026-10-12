"""ROS-free tests of the production delivery mask, configuration and lane ordering."""
import ast
import copy
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from uav_mission.drop_descent_config import drop_descent_parameters


class PositionTarget:
    FRAME_LOCAL_NED = 1
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    FORCE, IGNORE_YAW, IGNORE_YAW_RATE = 512, 1024, 2048

    def __init__(self):
        self.position = NS(x=0., y=0., z=0.)
        self.velocity = NS(x=99., y=99., z=0.)


source = PACKAGE/'scripts/navigation_frame_adapter.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
target = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'drop_vertical_target')
adapter = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FrameAdapter')
callback = next(n for n in adapter.body if isinstance(n, ast.FunctionDef) and n.name == 'callback')
namespace = dict(copy=copy, np=np, PositionTarget=PositionTarget,
                 euler_from_quaternion=lambda q: (0., 0., 2*math.atan2(q[2], q[3])))
exec(compile(ast.Module(body=[target, callback], type_ignores=[]), str(source), 'exec'), namespace)


class DropDescentModes(unittest.TestCase):
    def test_default_and_three_mode_values(self):
        self.assertEqual(drop_descent_parameters({})['mode'], 1)
        for mode in (1, 2, 3):
            cfg = drop_descent_parameters(dict(drop_descent_mode=mode, drop_descent_speed_mps=.25))
            self.assertEqual(cfg['mode'], mode)
            self.assertEqual(cfg['speed_mps'], .25)
            self.assertEqual(cfg['zero_velocity_setpoint_topic'], '/navigation/drop_setpoint_vertical')

    def test_invalid_modes_and_rates_are_rejected(self):
        for mode in (True, 0, 4, '2', 2.0):
            with self.assertRaises(ValueError): drop_descent_parameters(dict(drop_descent_mode=mode))
        for speed in (True, 0, -.1, 1.01, float('nan'), float('inf'), '0.2'):
            with self.assertRaises(ValueError): drop_descent_parameters(dict(drop_descent_speed_mps=speed))

    def test_mode_three_ignores_xy_position_and_controls_zero_xy_velocity(self):
        pose = NS(position=NS(x=9., y=-8., z=.40),
                  orientation=NS(x=0., y=0., z=math.sin(.3), w=math.cos(.3)))
        header = NS(frame_id='camera_init', stamp=100.)
        out = namespace['drop_vertical_target'](header, pose, np.eye(3))
        self.assertEqual(out.type_mask, 2531)
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_PX)
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_PY)
        self.assertFalse(out.type_mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(out.type_mask & PositionTarget.IGNORE_VX)
        self.assertFalse(out.type_mask & PositionTarget.IGNORE_VY)
        self.assertEqual((out.velocity.x, out.velocity.y), (0., 0.))
        self.assertEqual(out.position.z, .40)
        self.assertAlmostEqual(out.yaw, .6)
        out.header.frame_id = 'map'
        self.assertEqual(header.frame_id, 'camera_init')

    def test_mode_three_requires_gravity_aligned_frame_conversion(self):
        pose = NS(position=NS(z=.40), orientation=NS(x=0., y=0., z=0., w=1.))
        yaw = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        namespace['drop_vertical_target'](NS(), pose, yaw)
        with self.assertRaises(ValueError):
            namespace['drop_vertical_target'](NS(), pose, np.diag([1., -1., -1.]))

    def test_late_position_drop_or_h_messages_cannot_replace_newer_command(self):
        node = NS(command_lock=threading.Lock(), last_command_stamp=0.,
                  convert_and_publish=Mock(return_value=True))
        for stamp, lane in [(10., 'setpoint'), (11., 'drop_vertical'), (10.5, 'setpoint'),
                            (10.7, 'landing_vertical'), (12., 'setpoint'), (11.5, 'drop_vertical')]:
            namespace['callback'](node, NS(header=NS(stamp=stamp)), ('mission', 'map', None, lane))
        self.assertEqual(node.convert_and_publish.call_count, 3)
        self.assertEqual(node.last_command_stamp, 12.)

    def test_rejected_drop_conversion_does_not_advance_command_watermark(self):
        node = NS(command_lock=threading.Lock(), last_command_stamp=10.,
                  convert_and_publish=Mock(return_value=False))
        namespace['callback'](node, NS(header=NS(stamp=11.)), ('mission', 'map', None, 'drop_vertical'))
        self.assertEqual(node.last_command_stamp, 10.)


if __name__ == '__main__':
    unittest.main()
