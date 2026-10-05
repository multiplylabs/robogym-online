import unittest
from pathlib import Path

import mujoco
import numpy as np

from robogym_online.equipment import EQUIPMENT, ArmHold, CarryOverlay, set_payload
from robogym_online.scene import DEFAULT_MJCF, build_spec, load_contract


class EquipmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_contract(Path("assets/compiled_models_wrench"))

    def setUp(self):
        self.model = build_spec(DEFAULT_MJCF, 0.001).compile()
        self.data = mujoco.MjData(self.model)

    def test_mass_grips_and_state_survive_payload_changes(self):
        base = self.model.body_mass.sum()
        self.data.qpos[:3] = [0.5, -0.2, 0.82]
        self.data.qvel[:] = np.linspace(-0.1, 0.1, self.model.nv)
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        for name, item in EQUIPMENT.items():
            set_payload(self.model, self.data, name)
            self.assertAlmostEqual(self.model.body_mass.sum(), base + item["mass"])
            np.testing.assert_array_equal(self.data.qpos, qpos)
            np.testing.assert_array_equal(self.data.qvel, qvel)
            for shared in ["barbell", "kettlebell"]:
                self.assertEqual(
                    bool(self.data.eq_active[self.model.equality("gym_" + shared + "_grip").id]),
                    name == shared,
                )
        set_payload(self.model, self.data, "none")
        self.assertAlmostEqual(self.model.body_mass.sum(), base)

    def test_overlay_keeps_planner_and_committed_frames_unchanged(self):
        overlay = CarryOverlay(self.model, self.contract["joint_names"])
        q = np.tile(self.model.qpos0, (10, 1))
        overlay.select("barbell", q)
        future = np.tile(q[0], (80, 1))
        original = future.copy()
        composed = overlay.apply(future)
        np.testing.assert_array_equal(future, original)
        np.testing.assert_array_equal(composed[:10], original[:10])
        arms = overlay.poses["barbell"][0] + 7
        other = np.setdiff1d(np.arange(self.model.nq), arms)
        np.testing.assert_array_equal(composed[:, other], original[:, other])
        np.testing.assert_allclose(composed[-1, arms], overlay.poses["barbell"][1])
        overlay.reset()
        np.testing.assert_array_equal(overlay.apply(future), original)

    def test_servo_retains_effort_limits_and_restores_motor(self):
        hold = ArmHold(
            self.model,
            self.contract["joint_names"],
            self.contract["control"]["stiffness"],
            self.contract["control"]["damping"],
        )
        gain = self.model.actuator_gainprm.copy()
        bias = self.model.actuator_biasprm.copy()
        hold.select(self.data, "dumbbells")
        hold.update(1.5)
        a = hold.actuators
        np.testing.assert_array_equal(self.model.actuator_gainprm[a, 0], 0)
        np.testing.assert_array_equal(self.model.actuator_forcerange[a], self.model.actuator_ctrlrange[a])
        self.assertTrue(self.model.actuator_forcelimited[a].all())
        hold.reset()
        np.testing.assert_array_equal(self.model.actuator_gainprm, gain)
        np.testing.assert_array_equal(self.model.actuator_biasprm, bias)


if __name__ == "__main__":
    unittest.main()
