# SPDX-License-Identifier: Apache-2.0
"""Convert a ProtoMotions motion-library ``.pt`` clip to mjswan's ``body_world`` ``.npz``.

The tracker checkpoints in this family are driven from a recorded reference clip stored as a
ProtoMotions ``MotionLib`` state dict -- a concatenation of many motions with per-motion
``length_starts`` / ``motion_num_frames`` offsets, carrying precomputed *global* body frames:

    gts   [N, nbody, 3]   body translations (world)
    grs   [N, nbody, 4]   body rotations (world, xyzw -- ProtoMotions is w-last)
    gvs   [N, nbody, 3]   body linear velocities (world)
    gavs  [N, nbody, 3]   body angular velocities (world)
    dps   [N, ndof]       joint positions
    dvs   [N, ndof]       joint velocities

mjswan's browser motion player wants the same content per frame under its own names
(``body_pos_w`` / ``body_quat_w`` / ``body_lin_vel_w`` / ``body_ang_vel_w`` / ``joint_pos`` /
``joint_vel``), with quaternions in MuJoCo's **wxyz** order.

Two conversions matter beyond renaming:

* **Quaternion order.** ProtoMotions is xyzw, MuJoCo/mjlab is wxyz. The browser hands frames
  straight to MuJoCo, so the clip is stored wxyz; the observation terms flip back to xyzw on the
  way into the policy graph (see ``terms.py``).
* **Resampling to the control rate.** mjswan's ``TrackingCommand`` advances the clip one frame per
  control step, and the policy's look-ahead offsets (``future_step_indices``) are counted in
  *control steps*. Training sampled the reference by time (``future_dt_seconds``) with the motion
  library interpolating, so a clip recorded at 30 Hz must be resampled to 1/control_dt = 50 Hz for
  a frame offset to mean the same thing it meant in training. Positions and velocities interpolate
  linearly; rotations use sign-aligned nlerp, which is indistinguishable from slerp at these
  per-frame angles.
"""

from __future__ import annotations

import argparse
import io
from pathlib import Path

import numpy as np
import torch

# Body / joint order of the ONNX contract (``unified_pipeline.yaml``: ``body_names`` /
# ``joint_names``). Read from the YAML at build time; duplicated here only as the fallback the
# CLI uses when invoked standalone.
G1_BODY_NAMES = (
    "pelvis", "head",
    "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link",
    "left_ankle_pitch_link", "left_ankle_roll_link",
    "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link",
    "right_ankle_pitch_link", "right_ankle_roll_link",
    "waist_yaw_link", "waist_roll_link", "torso_link",
    "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
    "left_elbow_link", "left_wrist_roll_link", "left_wrist_pitch_link", "left_wrist_yaw_link",
    "left_rubber_hand",
    "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
    "right_elbow_link", "right_wrist_roll_link", "right_wrist_pitch_link",
    "right_wrist_yaw_link", "right_rubber_hand",
)


def _slice_motion(lib: dict, index: int) -> dict[str, np.ndarray]:
    """One motion out of the concatenated library, plus its native frame rate."""
    starts = lib["length_starts"].numpy()
    counts = lib["motion_num_frames"].numpy()
    if not 0 <= index < len(starts):
        raise IndexError(f"motion index {index} out of range (library holds {len(starts)})")
    lo = int(starts[index])
    hi = lo + int(counts[index])
    dt = float(lib["motion_dt"][index])
    out = {key: lib[key][lo:hi].numpy().astype(np.float64) for key in ("gts", "grs", "gvs", "gavs", "dps", "dvs")}
    out["fps"] = 1.0 / dt
    return out


def _nlerp(q0: np.ndarray, q1: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Sign-aligned normalized lerp between quaternion arrays, ``w`` broadcasting on the last axis.

    Flipping ``q1`` onto ``q0``'s hemisphere first is what makes this the *short-way* blend; without
    it a frame pair that straddles the double cover interpolates the long way round and the clip
    snaps.
    """
    flip = np.where(np.sum(q0 * q1, axis=-1, keepdims=True) < 0.0, -1.0, 1.0)
    out = q0 * (1.0 - w) + q1 * flip * w
    return out / np.clip(np.linalg.norm(out, axis=-1, keepdims=True), 1e-12, None)


def _resample(motion: dict[str, np.ndarray], target_fps: float) -> dict[str, np.ndarray]:
    """Resample a motion onto ``target_fps``, holding the clip's duration."""
    src_fps = motion["fps"]
    n_src = motion["dps"].shape[0]
    if abs(src_fps - target_fps) < 1e-6:
        return motion

    duration = (n_src - 1) / src_fps
    n_dst = int(round(duration * target_fps)) + 1
    # Source frame coordinate of each destination frame, clamped to the last source frame.
    t = np.minimum(np.arange(n_dst) * (src_fps / target_fps), n_src - 1)
    lo = np.floor(t).astype(np.int64)
    hi = np.minimum(lo + 1, n_src - 1)
    w = (t - lo).astype(np.float64)

    out: dict[str, np.ndarray] = {"fps": target_fps}
    for key in ("gts", "gvs", "gavs", "dps", "dvs"):
        a = motion[key]
        wt = w.reshape((-1,) + (1,) * (a.ndim - 1))
        out[key] = a[lo] * (1.0 - wt) + a[hi] * wt
    out["grs"] = _nlerp(motion["grs"][lo], motion["grs"][hi], w.reshape(-1, 1, 1))
    return out


def to_mjswan_npz(
    motion: dict[str, np.ndarray],
    body_names: tuple[str, ...] | list[str],
) -> bytes:
    """Serialize a resampled motion as mjswan ``body_world`` npz bytes.

    ``body_names`` is written into the archive so the browser maps bodies by name rather than by
    assuming the clip's body order equals the policy's.
    """
    n_bodies = motion["gts"].shape[1]
    if n_bodies != len(body_names):
        raise ValueError(f"clip has {n_bodies} bodies, body_names has {len(body_names)}")

    def f32(a: np.ndarray) -> np.ndarray:
        return np.ascontiguousarray(a, dtype=np.float32)

    payload = io.BytesIO()
    np.savez(
        payload,
        fps=np.asarray(motion["fps"], dtype=np.float32),
        joint_pos=f32(motion["dps"]),
        joint_vel=f32(motion["dvs"]),
        body_pos_w=f32(motion["gts"]),
        body_quat_w=f32(motion["grs"][:, :, [3, 0, 1, 2]]),  # xyzw -> wxyz
        body_lin_vel_w=f32(motion["gvs"]),
        body_ang_vel_w=f32(motion["gavs"]),
        body_names=np.asarray(list(body_names)),
    )
    return payload.getvalue()


def convert(
    source: Path,
    motion_index: int = 0,
    target_fps: float = 50.0,
    body_names: tuple[str, ...] | list[str] = G1_BODY_NAMES,
    max_frames: int | None = None,
) -> bytes:
    """Load ``source``, take one motion, resample it, and return mjswan npz bytes."""
    lib = torch.load(source, map_location="cpu", weights_only=True)
    motion = _resample(_slice_motion(lib, motion_index), target_fps)
    if max_frames is not None:
        for key in ("gts", "grs", "gvs", "gavs", "dps", "dvs"):
            motion[key] = motion[key][:max_frames]
    return to_mjswan_npz(motion, body_names)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="ProtoMotions motion-library .pt")
    parser.add_argument("output", type=Path, help="destination .npz")
    parser.add_argument("--motion-index", type=int, default=0)
    parser.add_argument("--fps", type=float, default=50.0, help="target rate (1/control_dt)")
    parser.add_argument("--max-frames", type=int, default=None)
    args = parser.parse_args()

    payload = convert(args.source, args.motion_index, args.fps, max_frames=args.max_frames)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)

    with np.load(io.BytesIO(payload)) as npz:
        print(f"wrote {args.output} ({len(payload) / 1e6:.2f} MB)")
        print(f"  frames={npz['joint_pos'].shape[0]} fps={float(npz['fps'])}")
        print(f"  bodies={npz['body_pos_w'].shape[1]} dofs={npz['joint_pos'].shape[1]}")


if __name__ == "__main__":
    main()
