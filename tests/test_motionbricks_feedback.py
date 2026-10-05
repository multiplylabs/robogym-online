import unittest

import numpy as np

from robogym_online.motionbricks_onnx import CONTEXT_FRAMES, MotionBricksOnnxStream
from robogym_online.motionbricks_stream import CORRECTION_STEP_M


class FeedbackTests(unittest.TestCase):
    def stream(self):
        stream = MotionBricksOnnxStream.__new__(MotionBricksOnnxStream)
        stream._qpos = np.zeros((CONTEXT_FRAMES, 36))
        stream._qpos[:, 3] = 1.0
        stream._pending = np.zeros((24, 36))
        stream._robot_xy = np.array([-1.0, 1.0])
        stream._robot_frame = 0
        stream._reference = {"body_pos_w": np.zeros((2, 1, 3))}
        stream._correction = np.array([0.4, 0.4])
        return stream

    def test_feedback_is_bounded_per_emitted_frame(self):
        stream = self.stream()
        previous = stream._pending[0, :2].copy()
        for _ in range(24):
            stream._advance_correction()
            current = stream._pending[0, :2].copy()
            self.assertLessEqual(np.linalg.norm(current - previous), CORRECTION_STEP_M + 1e-12)
            previous = current
            stream._pending = stream._pending[1:]
        np.testing.assert_array_equal(stream._qpos[:, :2], 0.0)

    def test_context_does_not_reapply_accumulated_translation(self):
        stream = self.stream()
        stream._qpos[:, :2] = [2.0, 3.0]
        np.testing.assert_array_equal(stream._context()[0], stream._qpos)

    def test_feedback_uses_consumer_frame_not_lookahead_tail(self):
        stream = self.stream()
        stream._robot_xy = np.array([0.0, 0.0])
        stream._reference["body_pos_w"][1, 0, :2] = [10.0, 10.0]
        stream._advance_correction()
        np.testing.assert_array_equal(stream._pending[:, :2], 0.0)


if __name__ == "__main__":
    unittest.main()
