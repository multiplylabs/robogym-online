# SPDX-License-Identifier: Apache-2.0
"""Drive the G1 around a MuJoCo scene with WASD, tracked by the force policy.

This is the whole pipeline closed on one machine: keys set a velocity command, ARDY generates the
reference to satisfy it, and the exported force-WBC policy tracks that reference while an external
force can be applied to a hand. It is the same policy, contract and kinematics the browser app runs,
so it is the honest local rehearsal of the browser build -- and where a convention error shows up as
a robot that falls over rather than as a subtly wrong picture.

    TEXT_ENCODER=null python -m robogym_online.live_wasd

Controls (in the viewer window):

    W / S     walk forward / back        Q / E   turn left / right
    A / D     strafe left / right        space   stop
    R / F     hand force +/- 5 N         X       zero the hand force

The reference is generated ahead of the robot, not with it: :class:`ReferenceStream` keeps roughly a
second in hand, so a keypress lands a stride or so later. That lag is inherent to steering a
*tracker* -- the policy follows a reference that must already exist to be followed.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from .check_tracking import _PROPRIO_HISTORY, _quat_apply_inv, _to_xyzw
from .scene import DEFAULT_MJCF, DEFAULT_MOTIONBRICKS_ROOT, DEFAULT_ONNX_DIR, build_spec, load_contract
from .wasd_stream import ReferenceStream

# What each key commands: (forward m/s, lateral m/s, turn deg/s), in the robot's own frame.
KEY_COMMANDS = {
    "w": (0.8, 0.0, 0.0),
    "s": (-0.5, 0.0, 0.0),
    "a": (0.0, 0.45, 0.0),
    "d": (0.0, -0.45, 0.0),
    "q": (0.4, 0.0, 20.0),
    "e": (0.4, 0.0, -20.0),
    " ": (0.0, 0.0, 0.0),
}
HAND_FORCE_STEP_N = 5.0
# Control steps between lag reports to the generator; 10 is 5 Hz at the contract's rate.
LAG_REPORT_EVERY = 10


class Runner:
    """The control loop: reference stream in, joint targets out, MuJoCo in between."""

    def __init__(
        self,
        onnx_dir: Path,
        mjcf: Path,
        hand: str,
        seed: int | None,
        remote: str | None = None,
        generator: str = "motionbricks",
        motionbricks_root: Path | None = None,
    ) -> None:
        import mujoco
        import onnxruntime as ort

        self._mj = mujoco
        contract = load_contract(onnx_dir)
        self.contract = contract
        self.control_dt = float(contract["timing"]["control_dt"])
        physics_dt = float(contract["timing"]["physics_dt"])
        self.decimation = int(round(self.control_dt / physics_dt))
        self.time_steps = [int(s) for s in contract["motion"]["future_step_indices"]]
        self.anchor = int(contract["robot"]["anchor_body_index"])
        self.n_dofs = len(contract["joint_names"])
        self.kp = np.asarray(contract["control"]["stiffness"], dtype=np.float64)
        self.kd = np.asarray(contract["control"]["damping"], dtype=np.float64)

        self.model = build_spec(mjcf, physics_dt).compile()
        self.data = mujoco.MjData(self.model)
        self.qpos_adr = np.array(
            [
                self.model.jnt_qposadr[i]
                for i in range(self.model.njnt)
                if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
            ]
        )
        self.qvel_adr = np.array(
            [
                self.model.jnt_dofadr[i]
                for i in range(self.model.njnt)
                if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
            ]
        )
        self.hand_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, hand)
        if self.hand_body < 0:
            raise ValueError(f"no body named {hand!r}")
        self.hand_force = np.zeros(3)

        self.session = ort.InferenceSession(
            str(onnx_dir / "unified_pipeline.onnx"), providers=["CPUExecutionProvider"]
        )
        self.in_names = [i.name for i in self.session.get_inputs()]
        self.out_names = [o.name for o in self.session.get_outputs()]

        # A remote stream is the same interface over a socket -- the path the browser takes, so
        # running the local loop against it exercises the wire format without a browser.
        if remote:
            from .wasd_server import RemoteReferenceStream

            self.stream = RemoteReferenceStream(remote)
        elif generator == "motionbricks":
            from .motionbricks_stream import MotionBricksStream

            self.stream = MotionBricksStream(
                contract, mjcf, Path(motionbricks_root or DEFAULT_MOTIONBRICKS_ROOT), seed=seed
            )
        elif generator == "ardy":
            self.stream = ReferenceStream(contract, mjcf, seed=seed)
        else:
            raise ValueError(f"unknown generator {generator!r}")
        self.idx = 0
        self.reset()

    def reset(self) -> None:
        """Put the robot on the reference's first frame, as the browser's tracker does."""
        first = self.stream.frames(0)
        self.data.qpos[0:3] = first["body_pos_w"][0, 0]
        self.data.qpos[3:7] = first["body_quat_w"][0, 0]
        self.data.qpos[self.qpos_adr] = first["joint_pos"][0]
        self.data.qvel[0:3] = first["body_lin_vel_w"][0, 0]
        self.data.qvel[3:6] = first["body_ang_vel_w"][0, 0]
        self.data.qvel[self.qvel_adr] = first["joint_vel"][0]
        self._mj.mj_forward(self.model, self.data)

        # Newest-first, seeded from the reference's first pose. Zeros here would inject a fictitious
        # whole-body step into the channel the policy reads load from.
        self.hist = {
            "dof_pos": np.tile(first["joint_pos"][0], (_PROPRIO_HISTORY, 1)),
            "dof_vel": np.tile(first["joint_vel"][0], (_PROPRIO_HISTORY, 1)),
            "anchor_rot": np.tile(_to_xyzw(first["body_quat_w"][0, self.anchor]), (_PROPRIO_HISTORY, 1)),
            "root_local_ang_vel": np.zeros((_PROPRIO_HISTORY, 3)),
            "actions": np.tile(first["joint_pos"][0], (_PROPRIO_HISTORY, 1)),
        }

    def step(self) -> float:
        """One control step. Returns the mean joint tracking error against the reference."""
        data = self.data
        dof_pos = data.qpos[self.qpos_adr].copy()
        dof_vel = data.qvel[self.qvel_adr].copy()
        anchor_rot = _to_xyzw(data.xquat[self.anchor + 1].copy())
        root_local_ang_vel = _quat_apply_inv(data.xquat[1], data.cvel[1][0:3].copy())

        for key, value in (
            ("dof_pos", dof_pos),
            ("dof_vel", dof_vel),
            ("anchor_rot", anchor_rot),
            ("root_local_ang_vel", root_local_ang_vel),
        ):
            self.hist[key] = np.roll(self.hist[key], 1, axis=0)
            self.hist[key][0] = value

        window = self.stream.frames([self.idx + s for s in self.time_steps])
        ref_pos = window["body_pos_w"]
        ref_quat = _to_xyzw(window["body_quat_w"])
        feed = {
            "current_dof_pos": dof_pos[None],
            "current_dof_vel": dof_vel[None],
            "current_anchor_rot": anchor_rot[None],
            "current_root_local_ang_vel": root_local_ang_vel[None],
            "historical_dof_pos": self.hist["dof_pos"][None],
            "historical_dof_vel": self.hist["dof_vel"][None],
            "historical_anchor_rot": self.hist["anchor_rot"][None],
            "historical_root_local_ang_vel": self.hist["root_local_ang_vel"][None],
            "historical_processed_actions": self.hist["actions"][None],
            "mimic_future_dof_pos": window["joint_pos"][None],
            "mimic_future_dof_vel": window["joint_vel"][None],
            "mimic_future_pos": ref_pos[None],
            "mimic_future_rot": ref_quat[None],
            "mimic_future_anchor_pos": ref_pos[:, self.anchor][None],
            "mimic_future_anchor_rot": ref_quat[:, self.anchor][None],
            "mimic_ref_state_rigid_body_pos": ref_pos[0][None],
            "mimic_ref_state_rigid_body_rot": ref_quat[0][None],
            # Compensation mode: the braced pose is the reference pose, deltas are null, and the
            # rotation delta is *identity* -- zeros there would annihilate the torso block.
            "hand_force_x_priv_bodies": ref_pos[0][None],
            "hand_force_x_priv_rot": ref_quat[0][None],
            "hand_force_dof_priv_delta": np.zeros((1, self.n_dofs)),
            "hand_force_xpriv_anchor_pos_delta": np.zeros((1, 3)),
            "hand_force_xpriv_anchor_rot_delta": np.array([[0.0, 0.0, 0.0, 1.0]]),
            "task_mode_mode_onehot": np.array([[0.0, 1.0]]),  # [exert, comp]
            "task_mode_force_cmd_eff": np.zeros((1, 2, 3)),
            "initial_noise": np.zeros((1, self.n_dofs)),
        }
        feed = {k: np.ascontiguousarray(v, dtype=np.float32) for k, v in feed.items() if k in self.in_names}
        targets = np.asarray(self.session.run(self.out_names, feed)[1], dtype=np.float64).reshape(-1)

        self.hist["actions"] = np.roll(self.hist["actions"], 1, axis=0)
        self.hist["actions"][0] = targets

        for _ in range(self.decimation):
            q = data.qpos[self.qpos_adr]
            v = data.qvel[self.qvel_adr]
            data.ctrl[:] = self.kp * (targets - q) - self.kd * v
            data.xfrc_applied[self.hand_body, 0:3] = self.hand_force
            self._mj.mj_step(self.model, data)

        # Tell the generator where the robot actually is; without it the reference drifts away for
        # as long as a key is held. A generator that continues from a pose takes the pose itself;
        # one that plans a path takes the error against the frame being tracked.
        if hasattr(self.stream, "set_context_qpos"):
            self.stream.set_context_qpos(data.qpos, self.idx)
        elif self.idx % LAG_REPORT_EVERY == 0:
            error = ref_pos[0, 0, :2] - data.qpos[0:2]
            self.stream.set_lag(float(error[0]), float(error[1]))
        self.idx += 1
        return float(np.abs(data.qpos[self.qpos_adr] - window["joint_pos"][0]).mean())

    @property
    def fallen(self) -> bool:
        return bool(self.data.qpos[2] < 0.4)


def _make_key_callback(runner: Runner, state: dict):
    def key_callback(keycode: int) -> None:
        key = chr(keycode).lower() if 0 < keycode < 0x110000 else ""
        if key in KEY_COMMANDS:
            runner.stream.set_command(*KEY_COMMANDS[key])
            state["command"] = KEY_COMMANDS[key]
        elif key == "r":
            runner.hand_force[0] += HAND_FORCE_STEP_N
        elif key == "f":
            runner.hand_force[0] -= HAND_FORCE_STEP_N
        elif key == "x":
            runner.hand_force[:] = 0.0

    return key_callback


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx-dir", type=Path, default=DEFAULT_ONNX_DIR)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--hand", default="left_rubber_hand")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--generator",
        default="motionbricks",
        choices=["motionbricks", "ardy"],
        help="which model invents the reference the policy tracks",
    )
    parser.add_argument("--motionbricks-root", type=Path, default=DEFAULT_MOTIONBRICKS_ROOT)
    parser.add_argument(
        "--remote",
        default=None,
        help="take the reference from a wasd_server instead of generating it here, e.g. ws://localhost:8765",
    )
    parser.add_argument(
        "--hand-force",
        default="0,0,0",
        help="world-frame force at the hand, 'fx,fy,fz' in newtons -- the disturbance to compensate",
    )
    parser.add_argument(
        "--script",
        nargs="+",
        default=None,
        help='run headless on a scripted command list instead of the viewer: "seconds:fwd,lat,turn"',
    )
    args = parser.parse_args()

    runner = Runner(
        args.onnx_dir, args.mjcf, args.hand, args.seed, args.remote,
        args.generator, args.motionbricks_root,
    )
    runner.hand_force[:] = [float(v) for v in args.hand_force.split(",")]

    if args.script:
        from .ardy_reference import parse_segment

        print(
            f"headless: {len(args.script)} segments, hand {args.hand}, "
            f"force {runner.hand_force.tolist()} N (|F| = {np.linalg.norm(runner.hand_force):.1f} N)"
        )
        for duration, forward, lateral, turn in (parse_segment(s) for s in args.script):
            runner.stream.set_command(forward, lateral, turn)
            steps = int(round(duration / runner.control_dt))
            errors, heights = [], []
            for _ in range(steps):
                errors.append(runner.step())
                heights.append(float(runner.data.qpos[2]))
            pelvis = runner.data.qpos[0:3]
            print(
                f"  cmd fwd={forward:5.2f} lat={lateral:5.2f} turn={turn:6.1f}  "
                f"{duration:4.1f}s  dof_err={np.mean(errors):.4f} rad  "
                f"pelvis=({pelvis[0]:6.2f},{pelvis[1]:6.2f})  "
                f"height mean {np.mean(heights):5.3f} min {np.min(heights):5.3f}"
                f"{'  FALLEN' if runner.fallen else ''}"
            )
        print("upright" if not runner.fallen else "FELL OVER")
        return

    import mujoco.viewer

    state = {"command": (0.0, 0.0, 0.0)}
    print(__doc__)
    with mujoco.viewer.launch_passive(
        runner.model, runner.data, key_callback=_make_key_callback(runner, state), show_left_ui=False
    ) as viewer:
        while viewer.is_running():
            wall = time.time()
            runner.step()
            # Camera follows the robot; without this it walks out of frame in a few seconds.
            viewer.cam.lookat[:] = runner.data.qpos[0:3]
            viewer.sync()
            lag = runner.control_dt - (time.time() - wall)
            if lag > 0:
                time.sleep(lag)


if __name__ == "__main__":
    main()
