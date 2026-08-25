# SPDX-License-Identifier: Apache-2.0
"""Turn a stream of MuJoCo ``qpos`` into the reference fields a tracking policy observes.

Any generator that can produce a G1 pose per frame -- a motion model, a recorded clip, a planner --
delivers the same thing: root pose plus joint angles. What the policy observes is per-body world
frames and velocities at its own control rate. This module is that conversion, kept apart from any
particular generator because the generators are interchangeable and this part is not.

Forward kinematics runs on the *app's own* model, so the reference is expressed in exactly the
kinematics the robot will be simulated with; velocities are finite differences of the result, which
is what a recorded reference clip holds too.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .convert_motion import _nlerp


def resample_qpos(qpos: np.ndarray, src_fps: float, dst_fps: float) -> np.ndarray:
    """Resample ``[T, 7 + ndof]`` qpos, holding duration.

    The root quaternion blends with the sign-aligned nlerp the clip converter uses; everything else
    is linear. Resampling before forward kinematics rather than after keeps the body frames exactly
    consistent with the joint angles at every output frame.
    """
    if abs(src_fps - dst_fps) < 1e-6:
        return qpos
    n_src = qpos.shape[0]
    n_dst = int(round((n_src - 1) / src_fps * dst_fps)) + 1
    t = np.minimum(np.arange(n_dst) * (src_fps / dst_fps), n_src - 1)
    lo = np.floor(t).astype(np.int64)
    hi = np.minimum(lo + 1, n_src - 1)
    w = (t - lo)[:, None]

    out = np.empty((n_dst, qpos.shape[1]), dtype=np.float64)
    out[:, 0:3] = qpos[lo, 0:3] * (1.0 - w) + qpos[hi, 0:3] * w
    out[:, 3:7] = _nlerp(qpos[lo, 3:7], qpos[hi, 3:7], w)
    out[:, 7:] = qpos[lo, 7:] * (1.0 - w) + qpos[hi, 7:] * w
    return out

def central_diff(values: np.ndarray, dt: float) -> np.ndarray:
    """Central difference with one-sided ends, preserving length."""
    out = np.empty_like(values)
    out[1:-1] = (values[2:] - values[:-2]) / (2.0 * dt)
    out[0] = (values[1] - values[0]) / dt
    out[-1] = (values[-1] - values[-2]) / dt
    return out

def quat_ang_vel(quat_wxyz: np.ndarray, dt: float) -> np.ndarray:
    """World-frame angular velocity from a wxyz quaternion track.

    ``omega = 2 * (dq/dt) * conj(q)``, vector part. Neighbours are sign-aligned first: across the
    double cover a raw difference reads as a full-turn spike, which would enter the reference as a
    momentary huge angular velocity.
    """
    q = quat_wxyz.copy()
    flips = np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=-1) < 0.0, -1.0, 1.0))
    q[1:] *= flips[:, None]
    dq = central_diff(q, dt)
    w0, x0, y0, z0 = dq.T
    w1, x1, y1, z1 = q.T  # conjugate applied inline below
    return 2.0 * np.stack(
        [
            -w0 * x1 + x0 * w1 - y0 * z1 + z0 * y1,
            -w0 * y1 + x0 * z1 + y0 * w1 - z0 * x1,
            -w0 * z1 - x0 * y1 + y0 * x1 + z0 * w1,
        ],
        axis=-1,
    )


class ReferenceBuilder:
    """Forward-kinematics a qpos track into the mjswan ``body_world`` fields, at the control rate.

    Holds the compiled model and its address tables so a live stream can rebuild cheaply, every time
    a generator produces new frames.
    """

    def __init__(self, contract: dict, mjcf: Path) -> None:
        import mujoco

        from .scene import build_spec

        self._mj = mujoco
        self.control_dt = float(contract["timing"]["control_dt"])
        self.body_names = list(contract["body_names"])
        self.joint_names = list(contract["joint_names"])
        self._model = build_spec(mjcf, float(contract["timing"]["physics_dt"])).compile()
        self._data = mujoco.MjData(self._model)
        self._qpos_adr = np.array(
            [
                self._model.jnt_qposadr[i]
                for i in range(self._model.njnt)
                if self._model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
            ]
        )
        model_bodies = [
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, i)
            for i in range(1, self._model.nbody)
        ]
        if model_bodies[: len(self.body_names)] != self.body_names:
            raise ValueError("model body order does not match the contract's body_names")
        if len(self._qpos_adr) != len(self.joint_names):
            raise ValueError(
                f"model has {len(self._qpos_adr)} hinge joints, contract has {len(self.joint_names)}"
            )

    def build(self, qpos: np.ndarray, src_fps: float) -> dict[str, np.ndarray]:
        """``[T, 7 + ndof]`` qpos at ``src_fps`` -> reference fields at the control rate.

        Resampled before forward kinematics, so the body frames are exactly consistent with the
        joint angles at every output frame.
        """
        frames = resample_qpos(np.asarray(qpos, dtype=np.float64), src_fps, 1.0 / self.control_dt)
        n_bodies = len(self.body_names)
        body_pos = np.empty((frames.shape[0], n_bodies, 3))
        body_quat = np.empty((frames.shape[0], n_bodies, 4))  # wxyz, MuJoCo's order
        for i, frame in enumerate(frames):
            self._data.qpos[0:7] = frame[0:7]
            self._data.qpos[self._qpos_adr] = frame[7:]
            self._mj.mj_kinematics(self._model, self._data)
            body_pos[i] = self._data.xpos[1 : n_bodies + 1]
            body_quat[i] = self._data.xquat[1 : n_bodies + 1]

        joint_pos = frames[:, 7:]
        return {
            "joint_pos": joint_pos,
            "joint_vel": central_diff(joint_pos, self.control_dt),
            "body_pos_w": body_pos,
            "body_quat_w": body_quat,
            "body_lin_vel_w": central_diff(body_pos, self.control_dt),
            "body_ang_vel_w": np.stack(
                [quat_ang_vel(body_quat[:, b], self.control_dt) for b in range(n_bodies)], axis=1
            ),
        }
