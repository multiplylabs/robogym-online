"""Exertion transitions stay slow until the actual slewed travel aligns with facing."""
import math
import unittest
from robogym_online.motionbricks_stream import MotionBricksStream

class ExertionMovementTests(unittest.TestCase):
    def stream(self, guarded):
        s=MotionBricksStream.__new__(MotionBricksStream)
        s._browser_exertion_movement=guarded
        s._command=(.4,0,6)
        s._heading=0
        s._move_angle=math.radians(-60)
        s._frame_dt=.02
        s._target_speed=.3
        s._speed_cmd=None
        s._walk_mode='stealth'
        s._modes={'stealth','slow_walk','idle'}
        return s
    def test_force_motion_keeps_slow_gait_through_alignment_transition(self):
        s=self.stream(True)
        self.assertEqual(s.current_mode(),'slow_walk')
        s._advance_target_speed()
        self.assertAlmostEqual(s._speed_cmd,.18)
        previous=s._move_angle
        s._command_vectors()
        self.assertLessEqual(abs(s._move_angle-previous),math.radians(30)*.02+1e-9)
        s._move_angle=s._heading
        self.assertEqual(s.current_mode(),'stealth')
    def test_normal_walk_keeps_original_direction_rate_and_style(self):
        s=self.stream(False)
        self.assertEqual(s.current_mode(),'stealth')
        s._advance_target_speed()
        self.assertAlmostEqual(s._speed_cmd,.3)
        previous=s._move_angle
        s._command_vectors()
        self.assertAlmostEqual(abs(s._move_angle-previous),math.radians(60)*.02)

    def test_vertical_push_uses_forward_slow_walk_and_restores_selected_gait(self):
        s=self.stream(True)
        s._move_angle=s._heading
        s._browser_vertical_exertion=True
        s._command=(.4,0,10)
        s._target_speed=.5
        self.assertEqual(s.current_mode(),'slow_walk')
        s._advance_target_speed()
        self.assertAlmostEqual(s._speed_cmd,.5)
        s._command=(.8,0,0)
        self.assertEqual(s.current_mode(),'slow_walk')
        s._command=(0,0,0)
        self.assertEqual(s.current_mode(),'idle')
        s._browser_vertical_exertion=False
        s._command=(.4,0,20)
        self.assertEqual(s.current_mode(),'stealth')
