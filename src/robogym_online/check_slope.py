"""Measure complete ramp traversal, with the browser's exact three-slab placement.

Trials fail on terrain-relative low pelvis height, excessive tilt, leaving the
course, or timeout. Remaining upright before reaching the ramp is not success.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

from .live_wasd import Runner
from .scene import DEFAULT_MJCF, DEFAULT_PLANNER_ROOT, SLOPE_BODY_NAMES


class RampCourse:
    """Placement math matches mjswan's SlopeTerrain.place, dimensions from model."""

    def __init__(self, runner: Runner, angle: float, lead: float = 3.0):
        model, data = runner.model, runner.data
        yaw = math.atan2(2 * (data.qpos[3] * data.qpos[6] + data.qpos[4] * data.qpos[5]),
                         1 - 2 * (data.qpos[5] ** 2 + data.qpos[6] ** 2))
        self.forward = np.array([math.cos(yaw), math.sin(yaw)])
        self.side = np.array([-math.sin(yaw), math.cos(yaw)])
        self.toe = data.qpos[:2].copy() + lead * self.forward
        self.angle = math.radians(angle)
        geom_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in SLOPE_BODY_NAMES]
        a, p = (float(model.geom_size[g, 0]) for g in geom_ids[:2])
        t = float(model.geom_size[geom_ids[0], 2])
        self.width = float(model.geom_size[geom_ids[0], 1])
        sin, cos = math.sin(self.angle), math.cos(self.angle)
        self.run, self.rise, self.plateau = 2 * a * cos, 2 * a * sin, 2 * p
        self.end = 2 * self.run + self.plateau
        placements = (
            (self.run / 2 + t * sin, self.rise / 2 - t * cos, -self.angle),
            (self.run + p, self.rise - t, 0),
            (1.5 * self.run + 2 * p - t * sin, self.rise / 2 - t * cos, self.angle),
        )
        for name, (along, z, pitch) in zip(SLOPE_BODY_NAMES, placements, strict=True):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            m = model.body_mocapid[body]
            data.mocap_pos[m] = [*(self.toe + along * self.forward), z]
            cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
            data.mocap_quat[m] = [cy * cp, -sy * sp, cy * sp, sy * cp]
        mujoco.mj_forward(model, data)

    def coordinates(self, xy):
        delta = xy - self.toe
        return float(delta @ self.forward), float(delta @ self.side)

    def height(self, xy):
        along, side = self.coordinates(xy)
        if abs(side) > self.width or along < 0 or along > self.end:
            return 0.0
        if along < self.run:
            return along * math.tan(self.angle)
        if along < self.run + self.plateau:
            return self.rise
        return (self.end - along) * math.tan(self.angle)


def trial(runner, style, speed, angle, seed, lead=3.0, hand_force=(0, 0, 0), max_seconds=90,
          turn_pulse_deg_s=0.0, on_step=None):
    stream = runner.stream
    stream.reset()
    stream._seed = seed
    stream.set_style(style)
    stream.set_target_speed(speed)
    mujoco.mj_resetData(runner.model, runner.data)
    runner.idx = 0
    runner.reset()
    runner.hand_force[:] = hand_force
    course = RampCourse(runner, angle, lead)
    min_clearance, max_tilt, max_progress = 10.0, 0.0, -lead
    errors, modes = [], set()
    exit_step = None
    turn_start = None
    reason = "timeout"
    for step in range(round(max_seconds / runner.control_dt)):
        moving = step >= round(2 / runner.control_dt) and exit_step is None
        turn = 0.0
        if turn_start is not None and moving:
            elapsed = (step - turn_start) * runner.control_dt
            turn = turn_pulse_deg_s if elapsed < 1 else -turn_pulse_deg_s if elapsed < 2 else 0.0
        stream.set_command(0.8 if moving else 0.0, 0.0, turn)
        errors.append(runner.step())
        modes.add(stream.current_mode())
        data = runner.data
        along, side = course.coordinates(data.qpos[:2])
        max_progress = max(max_progress, along)
        if along > 0.75 and turn_start is None:
            turn_start = step
        clearance = float(data.qpos[2] - course.height(data.qpos[:2]))
        tilt = math.degrees(math.acos(np.clip(1 - 2 * (data.qpos[4] ** 2 + data.qpos[5] ** 2), -1, 1)))
        min_clearance, max_tilt = min(min_clearance, clearance), max(max_tilt, tilt)
        if on_step is not None:
            on_step(runner, course)
        if not np.isfinite(data.qpos).all() or clearance < 0.4 or tilt > 60:
            reason = "fall"
            break
        if abs(side) > course.width - 0.2:
            reason = "left_course"
            break
        if along > course.end + 0.5 and exit_step is None:
            exit_step = step
        if exit_step is not None and step - exit_step >= round(3 / runner.control_dt):
            reason = "complete"
            break
    return {
        "style": style, "target_speed": speed, "angle_deg": angle, "seed": seed,
        "success": reason == "complete", "result": reason, "seconds": runner.idx * runner.control_dt,
        "progress_m": max_progress, "course_end_m": course.end,
        "min_clearance_m": min_clearance, "max_tilt_deg": max_tilt,
        "mean_joint_error_rad": float(np.mean(errors)), "actual_modes": sorted(modes),
        "hand_force_n": list(hand_force), "turn_pulse_deg_s": turn_pulse_deg_s, "lead_m": lead,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx-dir", type=Path, default=Path("assets/compiled_models_wrench"))
    parser.add_argument("--planner", type=Path, default=DEFAULT_PLANNER_ROOT)
    parser.add_argument("--styles", nargs="+", default=["slow_walk", "stealth", "object_carrying"])
    parser.add_argument("--speeds", nargs="+", default=["native", "0.3", "0.5", "0.8"])
    parser.add_argument("--angles", type=float, nargs="+", default=[6.0, 8.0, 10.0])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0])
    parser.add_argument("--lead", type=float, default=3.0)
    parser.add_argument("--max-seconds", type=float, default=90)
    parser.add_argument("--hand-force", type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument("--turn-pulse", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=Path("slope_trials.jsonl"))
    args = parser.parse_args()
    runner = Runner(args.onnx_dir, DEFAULT_MJCF, "left_rubber_hand", 0,
                    generator="onnx", motionbricks_root=args.planner)
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 1
    runner.session = ort.InferenceSession(str(args.onnx_dir / "unified_pipeline.onnx"), opts,
                                         providers=["CPUExecutionProvider"])
    with args.output.open("w") as output:
        for style in args.styles:
            for speed in args.speeds:
                for angle in args.angles:
                    for seed in args.seeds:
                        result = trial(runner, style, None if speed == "native" else float(speed),
                                       angle, seed, args.lead, args.hand_force, args.max_seconds,
                                       args.turn_pulse)
                        line = json.dumps(result)
                        print(line, flush=True)
                        output.write(line + "\n")
                        output.flush()


if __name__ == "__main__":
    main()
