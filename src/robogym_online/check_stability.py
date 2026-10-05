"""Headless steering regression using the browser policy, physics and generator.

Example: python -m robogym_online.check_stability --planner /path/to/planner_sonic.onnx
The report fails on any fall during an episode, including one followed by recovery.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .live_wasd import Runner
from .scene import DEFAULT_MJCF, DEFAULT_PLANNER_ROOT
from .wasd_server import STYLE_SPEEDS_M_S, WALK_SPEED_M_S

# Idle, forward, strafe, reversal, backward, turn and stop, in seconds.
SEGMENTS = (
    (2, (0.0, 0.0, 0.0)),
    (8, (0.8, 0.0, 0.0)),
    (4, (0.0, 0.45, 0.0)),
    (4, (0.0, -0.45, 0.0)),
    (4, (-0.5, 0.0, 0.0)),
    (5, (0.4, 0.0, 20.0)),
    (3, (0.0, 0.0, 0.0)),
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx-dir", type=Path, default=Path("assets/compiled_models_wrench"))
    parser.add_argument("--planner", type=Path, default=DEFAULT_PLANNER_ROOT)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--style", default="slow_walk")
    parser.add_argument("--speeds", type=float, nargs="+")
    parser.add_argument("--remote", help="test a running websocket server instead of a local generator")
    parser.add_argument("--hand-force", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    args = parser.parse_args()
    if args.speeds is None:
        args.speeds = [STYLE_SPEEDS_M_S.get(args.style, WALK_SPEED_M_S)]
    failed = False
    for speed in args.speeds:
        for seed in args.seeds:
            runner = Runner(
                args.onnx_dir, args.mjcf, "left_rubber_hand", seed,
                remote=args.remote, generator="onnx", motionbricks_root=args.planner,
            )
            try:
                runner.stream.set_style(args.style)
                if not args.remote:
                    runner.stream.set_target_speed(speed)
                runner.hand_force[:] = args.hand_force
                errors, heights = [], []
                for duration, command in SEGMENTS:
                    runner.stream.set_command(*command)
                    for _ in range(round(duration / runner.control_dt)):
                        errors.append(runner.step())
                        heights.append(float(runner.data.qpos[2]))
                        if runner.fallen or not np.isfinite(runner.data.qpos).all():
                            failed = True
                            break
                    if runner.fallen or not np.isfinite(runner.data.qpos).all():
                        break
                report = {
                    "policy": str(args.onnx_dir), "style": args.style,
                    "seed": seed if not args.remote else "server controlled",
                    "speed": speed if not args.remote else "server default",
                    "steps": runner.idx, "min_pelvis_height": float(min(heights)),
                    "mean_joint_error": float(np.mean(errors)),
                    "fallen": runner.fallen,
                    "invalid_state": not bool(np.isfinite(runner.data.qpos).all()),
                }
                if not args.remote:
                    steps = np.diff(runner.stream._qpos[:, :2], axis=0)
                    report["max_reference_root_step"] = float(np.linalg.norm(steps, axis=1).max())
                    report["final_root_gap"] = runner.stream.tracking_error()
                print(json.dumps(report), flush=True)
            finally:
                if hasattr(runner.stream, "close"):
                    runner.stream.close()
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
