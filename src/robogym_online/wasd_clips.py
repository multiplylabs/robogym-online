# SPDX-License-Identifier: Apache-2.0
"""Pick a WASD-spanning clip set out of a motion library, and measure what each clip commands.

The force policy is a mimic tracker: it follows a reference, not a velocity command. So driving it
with a keyboard means choosing, each moment, the clip whose own motion is closest to what the
operator asked for. That needs two things this produces:

* a handful of converted clips spanning the library's directional range, rather than all of it
  (601 clips is ~300 MB of page; a WASD set is ~10);
* the *measured* command of each clip -- forward, lateral and turn rate in its own start-heading
  frame -- so the browser can pick by nearest neighbour instead of by filename.

The measurement is over the clip's whole span, which is what makes it comparable to a held key: a
clip whose net displacement is forward-left at 0.5 m/s is the right answer for "W+A", regardless of
what its individual strides do.

Coverage is a property of the library, not of this script, and the two to hand differ sharply:
`g1_bones_locomotion.pt` (25,680 clips) spans -2.4 to +3.6 m/s forward, +/-2.8 lateral and +/-87
deg/s of turn, with idle at the centre -- everything WASD needs. `walking_data.pt` (601 clips) is
SE(2)-constrained *forward* walks only: no backward, no idle, and turn to about 13 deg/s, so `S` and
standing still have nothing to select there. A target with no clip near it is dropped rather than
approximated; see `pick_clips`.

    python -m robogym_online.wasd_clips assets/motion_library.pt assets/wasd --report
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
import yaml

from .convert_motion import _resample, _slice_motion, to_mjswan_npz

# What each key combination asks for: (forward m/s, lateral m/s, turn deg/s). Lateral is positive
# to the robot's left, turn positive counter-clockwise, matching the measurement below.
#
# The targets are deliberately inside the library's measured range rather than at its edges: the
# nearest clip to an unreachable target is whatever is least bad, which reads as the key doing
# nothing in particular.
WASD_TARGETS: dict[str, tuple[float, float, float]] = {
    # Neutral: no key held. A tracker always follows *something*, so "stand still" is a clip too.
    "idle": (0.0, 0.0, 0.0),
    # W / S
    "forward": (0.55, 0.0, 0.0),
    "backward": (-0.40, 0.0, 0.0),
    "run": (1.40, 0.0, 0.0),
    # A / D strafe
    "left": (0.30, 0.45, 0.0),
    "right": (0.30, -0.45, 0.0),
    # W with A / D
    "forward_left": (0.45, 0.30, 4.0),
    "forward_right": (0.45, -0.30, -4.0),
    "backward_left": (-0.30, 0.25, 0.0),
    "backward_right": (-0.30, -0.25, 0.0),
    # Q / E turn in place-ish
    "turn_left": (0.20, 0.0, 35.0),
    "turn_right": (0.20, 0.0, -35.0),
}


def clip_commands(lib: dict) -> np.ndarray:
    """Per-clip ``(forward, lateral, turn)`` in each clip's own start-heading frame.

    Net displacement over the clip divided by its duration, rotated into the heading the clip
    starts in -- so the numbers mean the same thing as a held key does, independent of where in the
    world the clip happens to have been recorded.
    """
    starts = lib["length_starts"].numpy()
    counts = lib["motion_num_frames"].numpy()
    dts = lib["motion_dt"].numpy()
    root_pos = lib["gts"][:, 0, :2]
    root_rot = lib["grs"][:, 0, :]

    out = np.zeros((len(starts), 3), dtype=np.float64)
    for i in range(len(starts)):
        lo = int(starts[i])
        hi = lo + int(counts[i])
        duration = (hi - lo) * float(dts[i])
        p = root_pos[lo:hi].numpy()
        q = root_rot[lo:hi].numpy()  # xyzw
        yaw = np.arctan2(
            2.0 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]),
            1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2),
        )
        disp = p[-1] - p[0]
        c, s = np.cos(-yaw[0]), np.sin(-yaw[0])
        out[i] = (
            (c * disp[0] - s * disp[1]) / duration,
            (s * disp[0] + c * disp[1]) / duration,
            np.degrees(np.unwrap(yaw)[-1] - np.unwrap(yaw)[0]) / duration,
        )
    return out


def pick_clips(
    commands: np.ndarray,
    counts: np.ndarray,
    targets: dict[str, tuple[float, float, float]],
    min_frames: int = 100,
    turn_weight: float = 0.03,
    tolerance: float = 0.22,
) -> tuple[dict[str, int], dict[str, float]]:
    """Nearest clip to each target, one clip per target, none reused.

    Turn rate is in deg/s and the speeds in m/s, so the distance is weighted -- otherwise a few
    degrees of yaw outvotes half a metre per second and every target picks the same clip.

    A target whose nearest clip is further than ``tolerance`` is **dropped**, not approximated. The
    nearest clip to something a library cannot do is just its most ordinary clip, and binding a key
    to that gives an operator a control that visibly does nothing -- worse than a missing key, which
    at least tells the truth. Returns the picks and the rejected targets with their best distance.
    """
    scale = np.array([1.0, 1.0, turn_weight])
    long_enough = counts >= min_frames
    chosen: dict[str, int] = {}
    rejected: dict[str, float] = {}
    used: set[int] = set()
    for name, target in targets.items():
        d = np.linalg.norm((commands - np.asarray(target)) * scale, axis=1)
        d[~long_enough] = np.inf
        for i in used:
            d[i] = np.inf
        best = int(np.argmin(d))
        if not np.isfinite(d[best]) or d[best] > tolerance:
            rejected[name] = float(d[best])
            continue
        chosen[name] = best
        used.add(best)
    return chosen, rejected


def build(
    library: Path,
    out_dir: Path,
    contract: Path,
    report: bool = False,
    max_frames: int = 400,
) -> list[dict]:
    """Write the WASD clip set and return its manifest.

    Clips are trimmed to ``max_frames`` (8 s at 50 Hz). These loop under a held key, so length past
    a few gait cycles buys nothing and costs page weight -- the library's idle clip alone runs two
    minutes, which was more than half the set.
    """
    spec = yaml.safe_load(contract.read_text())
    body_names = list(spec["body_names"])
    fps = 1.0 / float(spec["timing"]["control_dt"])

    # Memory-mapped: these libraries run to tens of gigabytes, and the survey only reads the root
    # rows of each clip. Slicing the chosen clips afterwards pages in just those frames.
    lib = torch.load(library, map_location="cpu", weights_only=True, mmap=True)
    commands = clip_commands(lib)
    counts = lib["motion_num_frames"].numpy()

    if report:
        print(f"{library.name}: {len(counts)} clips")
        for label, col in zip(("forward m/s", "lateral m/s", "turn deg/s"), commands.T, strict=True):
            print(
                f"  {label:12s} min {col.min():6.2f}  med {np.median(col):6.2f}  max {col.max():6.2f}"
            )
        missing = []
        if commands[:, 0].min() > -0.1:
            missing.append("backward (S)")
        if np.abs(commands[:, 0]).min() > 0.15:
            missing.append("idle / standing")
        if missing:
            print(f"  NOT COVERED by this library: {', '.join(missing)}")

    out_dir.mkdir(parents=True, exist_ok=True)
    manifest: list[dict] = []
    picks, rejected = pick_clips(commands, counts, WASD_TARGETS)
    if report and rejected:
        for name, distance in rejected.items():
            target = WASD_TARGETS[name]
            print(
                f"  DROPPED {name:14s} target fwd {target[0]:.2f} lat {target[1]:.2f} "
                f"turn {target[2]:.1f} -- nearest clip is {distance:.2f} away, out of range"
            )
    for name, index in picks.items():
        motion = _resample(_slice_motion(lib, index), fps)
        if max_frames and motion["dps"].shape[0] > max_frames:
            for key in ("gts", "grs", "gvs", "gavs", "dps", "dvs"):
                motion[key] = motion[key][:max_frames]
        path = out_dir / f"{name}.npz"
        path.write_bytes(to_mjswan_npz(motion, body_names))
        entry = {
            "name": name,
            "file": path.name,
            "source_index": index,
            # The measured command, which is what the browser matches a keypress against.
            "command": [round(float(v), 4) for v in commands[index]],
            "frames": int(motion["dps"].shape[0]),
        }
        manifest.append(entry)
        if report:
            fwd, lat, turn = entry["command"]
            print(
                f"  {name:14s} clip {index:3d}  fwd {fwd:5.2f}  lat {lat:5.2f}  "
                f"turn {turn:6.2f}  {entry['frames']:4d} frames  {path.stat().st_size / 1e6:.2f} MB"
            )

    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("library", type=Path, help="ProtoMotions motion library (.pt)")
    parser.add_argument("out_dir", type=Path, help="where to write the clips + manifest.json")
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path("assets/compiled_models/unified_pipeline.yaml"),
        help="the policy contract, for body order and the control rate",
    )
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--max-frames", type=int, default=400, help="trim each clip (0 = keep all)")
    args = parser.parse_args()
    build(args.library, args.out_dir, args.contract, args.report, args.max_frames)


if __name__ == "__main__":
    main()
