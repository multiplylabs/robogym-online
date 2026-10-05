import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from robogym_online.check_slope import RampCourse
from robogym_online.motionbricks_stream import MotionBricksStream
from robogym_online.scene import build_spec
from robogym_online.wasd_server import STYLE_SPEEDS_M_S, _apply_walk_speed


class SlopeTests(unittest.TestCase):
    def test_off_axis_speed_cap_persists_during_reversal(self):
        stream = MotionBricksStream.__new__(MotionBricksStream)
        stream._target_speed = 0.8
        stream._speed_cmd = 0.3
        stream._frame_dt = 1 / 30
        stream._heading = 0.0
        stream._command = (0.0, 0.45, 0.0)
        stream._move_angle = np.pi / 2
        stream._modes = ["idle", "slow_walk", "stealth"]
        stream._walk_mode = "stealth"
        stream._advance_target_speed()
        self.assertEqual(stream._speed_cmd, 0.3)
        stream._command = (0.8, 0.0, 0.0)
        stream._advance_target_speed()
        self.assertEqual(stream._speed_cmd, 0.3)
        self.assertEqual(stream.current_mode(), "slow_walk")
        stream._move_angle = 0.0
        stream._advance_target_speed()
        self.assertGreater(stream._speed_cmd, 0.3)
        self.assertEqual(stream.current_mode(), "stealth")

    def test_server_sets_each_explicit_gait_speed(self):
        class Stream:
            def set_target_speed(self, value):
                self.target_speed = value

        stream = Stream()
        for style, expected in STYLE_SPEEDS_M_S.items():
            _apply_walk_speed(stream, style)
            self.assertEqual(stream.target_speed, expected)

    def test_selected_styles_are_not_replaced_at_low_speed(self):
        stream = MotionBricksStream.__new__(MotionBricksStream)
        stream._command = (0.8, 0.0, 0.0)
        stream._modes = ["idle", "walk", "slow_walk", "stealth", "object_carrying"]
        stream._speed_cmd = 0.3
        stream._slow_gait = False
        stream._move_angle = 0.0
        stream._heading = 0.0
        for style in ("slow_walk", "stealth", "object_carrying", "careful"):
            stream._walk_mode = style
            self.assertEqual(stream.current_mode(), style)

    def test_terrain_height_matches_mujoco_collision_surface(self):
        model = build_spec(Path("assets/mjcf/g1_holo_compat.xml"), 0.001).compile()
        data = mujoco.MjData(model)
        data.qpos[:7] = [0, 0, 0.793, 1, 0, 0, 0]
        course = RampCourse(SimpleNamespace(model=model, data=data), 10)
        for along in (-0.1, 0.1, course.run / 2, course.run + 0.2,
                      course.run + course.plateau + 0.2, course.end - 0.1, course.end + 0.1):
            xy = course.toe + along * course.forward + 0.8 * course.side
            geom_id = np.zeros(1, dtype=np.int32)
            distance = mujoco.mj_ray(model, data, np.array([*xy, 3.0]), np.array([0.0, 0.0, -1.0]),
                                     None, 1, -1, geom_id)
            self.assertAlmostEqual(3.0 - distance, course.height(xy), places=6)


if __name__ == "__main__":
    unittest.main()
