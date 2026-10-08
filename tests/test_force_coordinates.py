"""The exported brace keeps the dial local and rotates only world-frame outputs."""
import unittest
from pathlib import Path
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]

class ForceCoordinatesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        cls.session = ort.InferenceSession(str(ROOT/'assets/brace_wrench.onnx'), options,
                                           providers=['CPUExecutionProvider'])
        with np.load(ROOT/'assets/motion.npz') as clip:
            cls.pos = clip['body_pos_w'][:8][None].astype(np.float32)
            cls.quat = clip['body_quat_w'][:8][None].astype(np.float32)

    def evaluate(self, yaw, vector):
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]],np.float32)
        q = self.quat
        cw, sz = np.cos(yaw/2), np.sin(yaw/2)
        turned = np.stack([cw*q[...,0]-sz*q[...,3],cw*q[...,1]-sz*q[...,2],
                           cw*q[...,2]+sz*q[...,1],cw*q[...,3]+sz*q[...,0]],axis=-1)
        feed = {i.name:np.zeros(i.shape,np.float32) for i in self.session.get_inputs()}
        feed.update(ref_pos_window=self.pos@rotation.T+np.array([12,-7,0],np.float32),
                    ref_rot_window=turned.astype(np.float32))
        feed['dial'][0,0]=1
        feed['dial'][0,1:7]=np.tile(vector,2)
        feed['prev_mode'].fill(0)
        feed['prev_ramp'].fill(1)
        feed['prev_bal_ema'].fill(1)
        result = dict(zip([o.name.removeprefix('next_') for o in self.session.get_outputs()],
                          self.session.run(None,feed)))
        return result, rotation

    def test_all_signed_axes_at_multiple_headings(self):
        for vector in np.concatenate([8*np.eye(3),-8*np.eye(3)]):
            baseline, _ = self.evaluate(0,vector)
            for yaw in [0,np.pi/2,np.pi,-np.pi/2]:
                with self.subTest(vector=vector.tolist(),yaw=yaw):
                    got, rotation = self.evaluate(yaw,vector)
                    np.testing.assert_allclose(got['push_axis_local'][0],np.tile(vector/8,(2,1)),atol=2e-6)
                    np.testing.assert_allclose(got['push_axis_w'],baseline['push_axis_w']@rotation.T,atol=2e-6)
                    np.testing.assert_allclose(got['force_cmd_eff'],baseline['force_cmd_eff'],atol=2e-4,rtol=2e-4)
                    np.testing.assert_allclose(got['force_cmd_w'],got['force_cmd_eff']@rotation.T,atol=2e-5,rtol=2e-5)
