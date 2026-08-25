# SPDX-License-Identifier: Apache-2.0
"""Turn a WASD velocity command into a reference clip the force policy can track.

The tracker follows a reference; it does not accept a velocity command. Driving it from a keyboard
therefore needs something upstream that *invents* the reference, and that is what ARDY (NVIDIA's
autoregressive motion diffusion model) does here: given root waypoints it generates a G1 motion
that passes through them. Waypoints come from integrating the operator's command, so the whole
chain is

    (forward, lateral, turn) -> root path + heading -> ARDY -> qpos -> FK -> reference clip

and the force policy downstream is untouched: it sees the same ``body_world`` fields a recorded
clip would give it. Compared to selecting from a fixed clip library this has the property that
matters for live control -- the motion is generated in one continuous world frame, so there is no
clip boundary to re-anchor at when the operator changes direction mid-stride.

Nothing here needs a text prompt. ARDY conditions on text and on constraints through separate
branches with separate CFG weights, and WASD is entirely the constraint branch, so the released
text encoder (a gated 8B model) can be replaced by ``TEXT_ENCODER=null``.

**The three conventions that have to be right.** All were measured against ARDY's own output, not
taken from its docstrings -- the interactive demo's own heading formula disagrees with its comment.

* **Axes.** ARDY is y-up with z forward; MuJoCo is z-up with x forward. Its exporter handles the
  full rotation, and the planar consequence is ``x_ardy -> y_mujoco``, ``z_ardy -> x_mujoco``.
  Constraints are given in ARDY's frame, so a MuJoCo-frame path swaps components on the way in.
* **Heading.** The constraint angle equals MuJoCo yaw (measured: commanded 90 deg -> pelvis faces
  86.6 deg). Facing the direction of travel therefore wants ``atan2(x_ardy, z_ardy)``; the demo
  uses ``atan2(z_ardy, x_ardy)``, whose 90-degree transposition makes the character strafe.
* **Heading is not optional.** Without it the model holds its initial facing and side-steps to hit
  the waypoints -- it satisfies the path either way, so this fails silently as a bad gait rather
  than as an error.

Joint order needs no conversion at all: ARDY's ``g1.xml`` hinge order is identical to the policy
contract's ``joint_names``, asserted below.

    TEXT_ENCODER=null python -m robogym_online.ardy_reference out.npz \
        --segments "3:0.8,0,0" "3:0.5,0,30" "2:0,0.4,0"
"""

from __future__ import annotations

import argparse
import io
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from .convert_motion import _nlerp

# ARDY generates at its own rate; the reference is resampled to the contract's control rate.
ARDY_FPS = 25.0
# Waypoint spacing, in ARDY frames. The demo's value: dense enough to pin the path, sparse enough
# to leave the model room to place strides between waypoints.
WAYPOINT_INTERVAL = 4


def parse_segment(text: str) -> tuple[float, float, float, float]:
    """``"seconds:forward,lateral,turn"`` -> ``(seconds, m/s, m/s, deg/s)``.

    Velocities are in the robot's own frame: forward is where it faces, lateral is to its left,
    turn is counter-clockwise. That is the frame a keyboard commands in.
    """
    duration, _, rates = text.partition(":")
    values = [float(v) for v in rates.split(",")]
    if len(values) != 3:
        raise ValueError(f"segment {text!r} needs three rates: forward,lateral,turn")
    return (float(duration), *values)


def plan_path(
    segments: list[tuple[float, float, float, float]], fps: float = ARDY_FPS
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate body-frame velocity commands into a world path.

    Returns MuJoCo-frame ``(x, y)`` per frame and the yaw the robot should hold there. Yaw comes
    from the integrated turn command rather than from the path tangent, so "walk forward while
    turning in place" and "strafe" stay distinguishable -- a tangent would erase the difference.
    """
    dt = 1.0 / fps
    x, y, yaw = 0.0, 0.0, 0.0
    path, headings = [], []
    for duration, forward, lateral, turn in segments:
        for _ in range(int(round(duration * fps))):
            yaw += math.radians(turn) * dt
            # Body-frame velocity rotated into the world by the *current* yaw.
            x += (forward * math.cos(yaw) - lateral * math.sin(yaw)) * dt
            y += (forward * math.sin(yaw) + lateral * math.cos(yaw)) * dt
            path.append((x, y))
            headings.append(yaw)
    return np.asarray(path, dtype=np.float64), np.asarray(headings, dtype=np.float64)


def build_constraints(path: np.ndarray, headings: np.ndarray) -> list[dict]:
    """A ``root2d`` constraint set in ARDY's frame, for the MuJoCo-frame path.

    Frame 0 is skipped: the model starts there by construction, and constraining it fights the
    generation's own initialization.
    """
    indices = list(range(WAYPOINT_INTERVAL, len(path), WAYPOINT_INTERVAL))
    return [
        {
            "type": "root2d",
            "frame_indices": indices,
            # (x_mujoco, y_mujoco) -> (x_ardy, z_ardy) = (y_mujoco, x_mujoco).
            "root_2d": [[float(path[i][1]), float(path[i][0])] for i in indices],
            "global_root_heading": [float(headings[i]) for i in indices],
        }
    ]


def generate_qpos(segments: list[tuple[float, float, float, float]], seed: int | None = None) -> np.ndarray:
    """Run ARDY on the commanded path and return MuJoCo ``qpos`` at :data:`ARDY_FPS`.

    Imported lazily: ARDY is a heavyweight optional dependency (a GPU checkpoint), and every other
    entry point in this package must keep working without it.
    """
    import torch
    from ardy.constraints import load_constraints_lst
    from ardy.exports.mujoco import MujocoQposConverter
    from ardy.model import load_model
    from ardy.motion_rep.tools import length_to_mask

    if seed is not None:
        torch.manual_seed(seed)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    model = load_model("g1", device=device)
    converter = MujocoQposConverter(model.skeleton)

    path, headings = plan_path(segments, ARDY_FPS)
    num_frames = len(path)
    constraints = load_constraints_lst(build_constraints(path, headings), model.skeleton)

    lengths = torch.tensor([num_frames], device=device)
    observed, mask = model.motion_rep.create_conditions_from_constraints_batched(
        constraints, lengths, to_normalize=True, device=device
    )
    with torch.no_grad():
        motion = model(
            [""],  # no prompt: WASD rides the constraint branch
            num_frames,
            num_denoising_steps=10,
            pad_mask=length_to_mask(lengths),
            first_heading_angle=torch.zeros(1, device=device),
            motion_mask=mask,
            observed_motion=observed,
            cfg_weight=[2.0, 2.0],
            crop_history_length=196,
        )
    output = model.motion_rep.inverse(motion, is_normalized=True)
    return np.asarray(converter.dict_to_qpos(output, device), dtype=np.float64)[0]


def _resample_qpos(qpos: np.ndarray, src_fps: float, dst_fps: float) -> np.ndarray:
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


def _central_diff(values: np.ndarray, dt: float) -> np.ndarray:
    """Central difference with one-sided ends, preserving length."""
    out = np.empty_like(values)
    out[1:-1] = (values[2:] - values[:-2]) / (2.0 * dt)
    out[0] = (values[1] - values[0]) / dt
    out[-1] = (values[-1] - values[-2]) / dt
    return out


def _quat_ang_vel(quat_wxyz: np.ndarray, dt: float) -> np.ndarray:
    """World-frame angular velocity from a wxyz quaternion track.

    ``omega = 2 * (dq/dt) * conj(q)``, vector part. Neighbours are sign-aligned first: across the
    double cover a raw difference reads as a full-turn spike, which would enter the reference as a
    momentary huge angular velocity.
    """
    q = quat_wxyz.copy()
    flips = np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=-1) < 0.0, -1.0, 1.0))
    q[1:] *= flips[:, None]
    dq = _central_diff(q, dt)
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


def qpos_to_reference(qpos: np.ndarray, mjcf: Path, contract: dict, src_fps: float = ARDY_FPS) -> bytes:
    """Forward-kinematics a qpos track into mjswan ``body_world`` npz bytes.

    ARDY gives root pose and joint angles; the reference the policy observes is per-body world
    frames and velocities. Those come from the *app's own* model, so the reference is expressed in
    exactly the kinematics the robot will be simulated with.

    Velocities are finite differences of the resulting kinematics, which is what a recorded
    reference clip holds too -- ARDY's own root velocity is not used, so the two stay consistent.
    """
    import mujoco

    from .scene import build_spec

    control_dt = float(contract["timing"]["control_dt"])
    physics_dt = float(contract["timing"]["physics_dt"])
    body_names = list(contract["body_names"])
    joint_names = list(contract["joint_names"])

    model = build_spec(mjcf, physics_dt).compile()
    data = mujoco.MjData(model)

    model_bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(1, model.nbody)]
    if model_bodies[: len(body_names)] != body_names:
        raise ValueError("model body order does not match the contract's body_names")
    qpos_adr = np.array(
        [model.jnt_qposadr[i] for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
    )
    if len(qpos_adr) != len(joint_names):
        raise ValueError(f"model has {len(qpos_adr)} hinge joints, contract has {len(joint_names)}")

    frames = _resample_qpos(qpos, src_fps, 1.0 / control_dt)
    n = frames.shape[0]
    n_bodies = len(body_names)
    body_pos = np.empty((n, n_bodies, 3))
    body_quat = np.empty((n, n_bodies, 4))  # wxyz, MuJoCo's order

    for i, frame in enumerate(frames):
        data.qpos[0:7] = frame[0:7]
        data.qpos[qpos_adr] = frame[7:]
        mujoco.mj_kinematics(model, data)
        body_pos[i] = data.xpos[1 : n_bodies + 1]
        body_quat[i] = data.xquat[1 : n_bodies + 1]

    joint_pos = frames[:, 7:]
    payload = io.BytesIO()
    np.savez(
        payload,
        fps=np.asarray(1.0 / control_dt, dtype=np.float32),
        joint_pos=joint_pos.astype(np.float32),
        joint_vel=_central_diff(joint_pos, control_dt).astype(np.float32),
        body_pos_w=body_pos.astype(np.float32),
        body_quat_w=body_quat.astype(np.float32),
        body_lin_vel_w=_central_diff(body_pos, control_dt).astype(np.float32),
        body_ang_vel_w=np.stack(
            [_quat_ang_vel(body_quat[:, b], control_dt) for b in range(n_bodies)], axis=1
        ).astype(np.float32),
        body_names=np.asarray(body_names),
    )
    return payload.getvalue()


def assert_joint_order(contract: dict) -> None:
    """ARDY's G1 hinge order must equal the contract's, since qpos is copied across positionally."""
    from ardy.assets import skeleton_asset_path

    xml = ET.parse(str(skeleton_asset_path("g1skel34", "xml", "g1.xml"))).getroot()
    ardy_joints = [j.get("name") for j in xml.find("worldbody").findall(".//joint")]
    if ardy_joints != list(contract["joint_names"]):
        raise ValueError("ARDY g1.xml joint order differs from the policy contract's joint_names")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="destination .npz")
    parser.add_argument(
        "--segments",
        nargs="+",
        default=["4:0.8,0,0"],
        help='WASD command segments, each "seconds:forward,lateral,turn" (m/s, m/s, deg/s)',
    )
    parser.add_argument("--contract", type=Path, default=Path("assets/compiled_models/unified_pipeline.yaml"))
    parser.add_argument("--mjcf", type=Path, default=Path("assets/mjcf/g1_holo_compat.xml"))
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    contract = yaml.safe_load(args.contract.read_text())
    assert_joint_order(contract)
    segments = [parse_segment(s) for s in args.segments]

    qpos = generate_qpos(segments, args.seed)
    payload = qpos_to_reference(qpos, args.mjcf, contract)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)

    with np.load(io.BytesIO(payload)) as npz:
        print(f"wrote {args.output}  {len(payload) / 1e6:.2f} MB")
        print(f"  frames={npz['joint_pos'].shape[0]} fps={float(npz['fps'])}")
        travel = npz["body_pos_w"][-1, 0, :2] - npz["body_pos_w"][0, 0, :2]
        print(f"  root travel={np.linalg.norm(travel):.2f} m  bodies={npz['body_pos_w'].shape[1]}")


if __name__ == "__main__":
    main()
