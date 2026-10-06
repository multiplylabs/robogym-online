"""Projectiles are physical detached bodies; policy inputs retain robot order."""
import unittest

import mujoco
import numpy as np

from robogym_online.scene import DEFAULT_MJCF, STABILITY_BODY_NAMES, build_spec


class StabilityTests(unittest.TestCase):
    def test_projectiles_preserve_robot_indices_and_transfer_momentum(self):
        original = build_spec(DEFAULT_MJCF, .001).compile()
        model = build_spec(DEFAULT_MJCF, .001, stability_objects=True).compile()
        self.assertEqual(model.nq, original.nq + 7 * len(STABILITY_BODY_NAMES))
        self.assertEqual(model.nu, original.nu)
        for i in range(original.nbody):
            self.assertEqual(model.body(i).name, original.body(i).name)
        for i in range(original.njnt):
            self.assertEqual(model.joint(i).name, original.joint(i).name)
        for name in STABILITY_BODY_NAMES:
            with self.subTest(ball=name):
                for parked in STABILITY_BODY_NAMES:
                    parked_geom = model.geom(parked).id
                    parked_body = model.geom_bodyid[parked_geom]
                    model.geom_contype[parked_geom] = model.geom_conaffinity[parked_geom] = 0
                    model.body_contype[parked_body] = model.body_conaffinity[parked_body] = 0
                data = mujoco.MjData(model)
                model.opt.gravity[:] = 0
                mujoco.mj_forward(model, data)
                joint = model.joint(f'{name}_free')
                qadr, vadr = joint.qposadr[0], joint.dofadr[0]
                geom = model.geom(name).id
                model.geom_contype[geom] = model.geom_conaffinity[geom] = 1
                body = model.geom_bodyid[geom]
                model.body_contype[body] = model.body_conaffinity[body] = 1
                self.assertAlmostEqual(model.body(name).mass[0], .75)
                torso = data.xpos[model.body('torso_link').id].copy()
                data.qpos[qadr:qadr+3] = torso + [.65, 0, .13]
                data.qvel[vadr] = -4
                hit = False
                for _ in range(240):
                    mujoco.mj_step(model, data)
                    for c in data.contact:
                        other = c.geom2 if c.geom1 == geom else c.geom1 if c.geom2 == geom else -1
                        if other >= 0 and 0 < model.geom_bodyid[other] < original.nbody:
                            hit = True
                self.assertTrue(hit, 'A real robot/projectile contact must occur')
                self.assertGreater(np.linalg.norm(data.qvel[:6]), .01)
                np.testing.assert_array_equal(data.xfrc_applied, 0)
