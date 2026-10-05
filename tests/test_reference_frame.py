"""The deployed policy must not interpret a common XY world origin as pose error."""
import unittest
from pathlib import Path

import numpy as np
import onnxruntime as ort


class ReferenceFrameTests(unittest.TestCase):
    def test_policy_is_invariant_to_shared_reference_translation(self):
        model = Path(__file__).resolve().parents[1] / 'assets/compiled_models_wrench/unified_pipeline.onnx'
        session = ort.InferenceSession(str(model), providers=['CPUExecutionProvider'])
        keys = ['mimic_future_pos', 'mimic_future_anchor_pos',
                'mimic_ref_state_rigid_body_pos', 'hand_force_x_priv_bodies']
        for seed in range(3):
            rng = np.random.default_rng(seed)
            feed = {}
            for item in session.get_inputs():
                shape = [n if isinstance(n, int) else 1 for n in item.shape]
                a = rng.normal(0, .05, shape).astype(np.float32)
                if 'rot' in item.name and shape[-1] == 4:
                    a[..., 3] += 1
                    a /= np.linalg.norm(a, axis=-1, keepdims=True)
                if item.name == 'initial_noise':
                    a.fill(0)
                feed[item.name] = a
            shifted = {k: v.copy() for k, v in feed.items()}
            for key in keys:
                shifted[key][..., :2] += np.array([12.5, -8.25], np.float32)
            for original, moved in zip(session.run(None, feed), session.run(None, shifted)):
                self.assertTrue(np.isfinite(original).all())
                np.testing.assert_allclose(original, moved, atol=2e-4, rtol=2e-4)
