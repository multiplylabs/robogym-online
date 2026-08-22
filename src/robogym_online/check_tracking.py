# SPDX-License-Identifier: Apache-2.0
"""Headless replica of the browser's control loop, as a smoke test of the contract wiring.

The browser is a poor first place to find out that a quaternion is in the wrong order. This
reproduces, in numpy + MuJoCo + onnxruntime, exactly what the browser does per control step --
the same 25 inputs from the same sources, the same external PD, the same decimation -- and reports
whether the robot actually tracks the clip.

It is deliberately a *replica*, not an import of ``terms.py``: those terms are torch functions
written against mjlab's entity API, and the point here is to check the contract reading they
encode, independently. When this tracks and the browser does not, the fault is in the browser's
slot plumbing; when neither tracks, it is in the contract reading -- which is the distinction
worth having.

Usage::

    python check_tracking.py                    # 5 s rollout, prints per-second tracking error
    python check_tracking.py --steps 500 --verbose
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_app import (  # noqa: E402
    DEFAULT_MJCF,
    DEFAULT_MOTION,
    DEFAULT_ONNX_DIR,
    build_spec,
    load_contract,
)
from convert_motion import convert  # noqa: E402

_PROPRIO_HISTORY = 8


def _to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    """wxyz -> xyzw on the last axis."""
    return np.concatenate([quat_wxyz[..., 1:4], quat_wxyz[..., 0:1]], axis=-1)


def _quat_apply_inv(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate ``vec`` by the inverse of ``quat_wxyz`` -- mjlab's ``quat_apply_inverse``."""
    w, x, y, z = quat_wxyz
    conj = np.array([w, -x, -y, -z])
    return _quat_apply(conj, vec)


def _quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


class ClipWindow:
    """The browser ``TrackingCommand``'s look-ahead window, over a ``body_world`` clip.

    Offsets clamp at the last frame rather than wrapping, as the TS does.
    """

    def __init__(self, npz: dict, time_steps: list[int]):
        self.body_pos = npz["body_pos_w"]  # [N, nbody, 3]
        self.body_quat = npz["body_quat_w"]  # [N, nbody, 4] wxyz
        self.joint_pos = npz["joint_pos"]
        self.joint_vel = npz["joint_vel"]
        self.body_lin_vel = npz["body_lin_vel_w"]
        self.body_ang_vel = npz["body_ang_vel_w"]
        self.time_steps = time_steps
        self.n = self.joint_pos.shape[0]
        self.idx = 0

    def _window(self, array: np.ndarray) -> np.ndarray:
        rows = [array[min(self.n - 1, max(0, self.idx + s))] for s in self.time_steps]
        return np.stack(rows, axis=0)

    def ref_body_pos_w(self) -> np.ndarray:
        return self._window(self.body_pos)

    def ref_body_quat_w(self) -> np.ndarray:
        return self._window(self.body_quat)

    def ref_joint_pos(self) -> np.ndarray:
        return self._window(self.joint_pos)

    def ref_joint_vel(self) -> np.ndarray:
        return self._window(self.joint_vel)

    def advance(self) -> None:
        self.idx = (self.idx + 1) % self.n


def run(
    onnx_dir: Path,
    motion_pt: Path,
    mjcf: Path,
    motion_index: int,
    steps: int,
    verbose: bool,
    hand_force: tuple[float, float, float] = (0.0, 0.0, 0.0),
    hand: str = "left_rubber_hand",
) -> int:
    contract = load_contract(onnx_dir)
    joint_names = list(contract["joint_names"])
    body_names = list(contract["body_names"])
    control_dt = float(contract["timing"]["control_dt"])
    physics_dt = float(contract["timing"]["physics_dt"])
    decimation = int(round(control_dt / physics_dt))
    time_steps = [int(s) for s in contract["motion"]["future_step_indices"]]
    anchor = int(contract["robot"]["anchor_body_index"])
    n_dofs = len(joint_names)
    kp = np.asarray(contract["control"]["stiffness"], dtype=np.float64)
    kd = np.asarray(contract["control"]["damping"], dtype=np.float64)

    model = build_spec(mjcf, physics_dt).compile()
    data = mujoco.MjData(model)

    # The disturbance the browser's Hand Force sliders apply: world-frame newtons at the hand
    # body origin, exactly as `tasks/force_comp/control.py` writes them.
    hand_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, hand)
    if hand_body < 0:
        raise ValueError(f"no body named {hand!r}")
    force_w = np.asarray(hand_force, dtype=np.float64)

    clip_bytes = convert(motion_pt, motion_index, 1.0 / control_dt, body_names)
    npz = dict(np.load(__import__("io").BytesIO(clip_bytes)))
    clip = ClipWindow(npz, time_steps)

    # mjlab's per-body / per-joint element order is the model's, free joint excluded, and the
    # build already asserted that this equals the contract's -- so plain index arithmetic.
    qpos_adr = np.array(
        [model.jnt_qposadr[i] for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
    )
    qvel_adr = np.array(
        [model.jnt_dofadr[i] for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
    )
    root_body = 1  # pelvis; body 0 is the worldbody

    # Reset onto the clip's first frame, which is what the browser's TrackingCommand does.
    data.qpos[0:3] = clip.body_pos[0, 0]
    data.qpos[3:7] = clip.body_quat[0, 0]
    data.qpos[qpos_adr] = clip.joint_pos[0]
    data.qvel[0:3] = clip.body_lin_vel[0, 0]
    data.qvel[3:6] = clip.body_ang_vel[0, 0]
    data.qvel[qvel_adr] = clip.joint_vel[0]
    mujoco.mj_forward(model, data)

    session = ort.InferenceSession(str(onnx_dir / "unified_pipeline.onnx"), providers=["CPUExecutionProvider"])
    in_names = [i.name for i in session.get_inputs()]
    out_names = [o.name for o in session.get_outputs()]

    # Newest-first buffers, seeded from the reset state / the clip's first pose -- the runner's
    # convention and what the browser's history priming reproduces.
    hist = {
        "dof_pos": np.tile(clip.joint_pos[0], (_PROPRIO_HISTORY, 1)),
        "dof_vel": np.tile(clip.joint_vel[0], (_PROPRIO_HISTORY, 1)),
        "anchor_rot": np.tile(_to_xyzw(clip.body_quat[0, anchor]), (_PROPRIO_HISTORY, 1)),
        "root_local_ang_vel": np.zeros((_PROPRIO_HISTORY, 3)),
        "actions": np.tile(clip.joint_pos[0], (_PROPRIO_HISTORY, 1)),
    }

    errors: list[float] = []
    for step in range(steps):
        dof_pos = data.qpos[qpos_adr].copy()
        dof_vel = data.qvel[qvel_adr].copy()
        anchor_quat_wxyz = data.xquat[anchor + 1].copy()  # +1: model bodies are offset by world
        anchor_rot = _to_xyzw(anchor_quat_wxyz)
        # mjlab's root_link_ang_vel_b: cvel's angular part, rotated into the root frame.
        ang_vel_w = data.cvel[root_body][0:3].copy()
        root_local_ang_vel = _quat_apply_inv(data.xquat[root_body], ang_vel_w)

        for key, value in (
            ("dof_pos", dof_pos),
            ("dof_vel", dof_vel),
            ("anchor_rot", anchor_rot),
            ("root_local_ang_vel", root_local_ang_vel),
        ):
            hist[key] = np.roll(hist[key], 1, axis=0)
            hist[key][0] = value

        ref_pos = clip.ref_body_pos_w()  # [steps, nbody, 3]
        ref_quat = _to_xyzw(clip.ref_body_quat_w())  # [steps, nbody, 4] xyzw
        feed = {
            "current_dof_pos": dof_pos[None],
            "current_dof_vel": dof_vel[None],
            "current_anchor_rot": anchor_rot[None],
            "current_root_local_ang_vel": root_local_ang_vel[None],
            "historical_dof_pos": hist["dof_pos"][None],
            "historical_dof_vel": hist["dof_vel"][None],
            "historical_anchor_rot": hist["anchor_rot"][None],
            "historical_root_local_ang_vel": hist["root_local_ang_vel"][None],
            "historical_processed_actions": hist["actions"][None],
            "mimic_future_dof_pos": clip.ref_joint_pos()[None],
            "mimic_future_dof_vel": clip.ref_joint_vel()[None],
            "mimic_future_pos": ref_pos[None],
            "mimic_future_rot": ref_quat[None],
            "mimic_future_anchor_pos": ref_pos[:, anchor][None],
            "mimic_future_anchor_rot": ref_quat[:, anchor][None],
            "mimic_ref_state_rigid_body_pos": ref_pos[0][None],
            "mimic_ref_state_rigid_body_rot": ref_quat[0][None],
            # Compensation mode: x_priv == x_ref, so the braced pose IS the reference pose.
            "hand_force_x_priv_bodies": ref_pos[0][None],
            "hand_force_x_priv_rot": ref_quat[0][None],
            "hand_force_dof_priv_delta": np.zeros((1, n_dofs)),
            "hand_force_xpriv_anchor_pos_delta": np.zeros((1, 3)),
            "hand_force_xpriv_anchor_rot_delta": np.array([[0.0, 0.0, 0.0, 1.0]]),
            "task_mode_mode_onehot": np.array([[0.0, 1.0]]),  # [exert, comp]
            "task_mode_force_cmd_eff": np.zeros((1, 2, 3)),
            "initial_noise": np.zeros((1, n_dofs)),
        }
        missing = set(in_names) - set(feed)
        if missing:
            raise KeyError(f"unfed ONNX inputs: {sorted(missing)}")
        feed = {k: np.ascontiguousarray(v, dtype=np.float32) for k, v in feed.items() if k in in_names}

        out = session.run(out_names, feed)
        targets = np.asarray(out[1], dtype=np.float64).reshape(-1)  # joint_pos_targets

        hist["actions"] = np.roll(hist["actions"], 1, axis=0)
        hist["actions"][0] = targets

        for _ in range(decimation):
            q = data.qpos[qpos_adr]
            v = data.qvel[qvel_adr]
            data.ctrl[:] = kp * (targets - q) - kd * v
            data.xfrc_applied[hand_body, 0:3] = force_w
            mujoco.mj_step(model, data)

        # Tracking error against the reference the policy was aiming at this step.
        ref_dof = clip.ref_joint_pos()[0]
        err = float(np.abs(data.qpos[qpos_adr] - ref_dof).mean())
        errors.append(err)
        clip.advance()

        if verbose and step % 10 == 0:
            print(
                f"step {step:4d} t={data.time:5.2f}s  pelvis_z={data.qpos[2]:.3f} "
                f"(ref {clip.body_pos[clip.idx, 0, 2]:.3f})  dof_err={err:.4f}"
            )

    per_second = int(round(1.0 / control_dt))
    print(f"\n{steps} control steps ({steps * control_dt:.1f} s), decimation {decimation}")
    print("mean |dof - ref| per second:")
    for i in range(0, len(errors), per_second):
        chunk = errors[i : i + per_second]
        print(f"  t={i * control_dt:4.1f}s  {np.mean(chunk):.4f} rad")
    if np.any(force_w):
        print(f"hand force on {hand}: {force_w.tolist()} N (world), |F| = {np.linalg.norm(force_w):.1f} N")
    print(f"final pelvis height {data.qpos[2]:.3f} m (clip {clip.body_pos[clip.idx, 0, 2]:.3f} m)")

    fell = data.qpos[2] < 0.4
    print("FELL" if fell else "upright")
    return 1 if fell else 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx-dir", type=Path, default=DEFAULT_ONNX_DIR)
    parser.add_argument("--motion-file", type=Path, default=DEFAULT_MOTION)
    parser.add_argument("--motion-index", type=int, default=0)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument(
        "--hand-force",
        default="0,0,0",
        help="world-frame force at the hand, 'fx,fy,fz' in newtons (the browser's sliders)",
    )
    parser.add_argument("--hand", default="left_rubber_hand")
    args = parser.parse_args()
    raise SystemExit(
        run(
            args.onnx_dir,
            args.motion_file,
            args.mjcf,
            args.motion_index,
            args.steps,
            args.verbose,
            tuple(float(v) for v in args.hand_force.split(",")),
            args.hand,
        )
    )


if __name__ == "__main__":
    main()
