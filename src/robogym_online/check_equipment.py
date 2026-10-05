"""Loaded ramp audit using the same payload mass, grips, policy and arm references as the page."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import onnxruntime as ort

from .check_slope import trial
from .equipment import EQUIPMENT, ArmHold, set_payload
from .live_wasd import Runner
from .scene import DEFAULT_MJCF, DEFAULT_PLANNER_ROOT
from .wasd_server import STYLE_SPEEDS_M_S


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--planner", type=Path, default=DEFAULT_PLANNER_ROOT)
    p.add_argument("--equipment", nargs="+", default=["dumbbells", "kettlebell", "barbell"])
    p.add_argument("--styles", nargs="+", default=["stealth", "slow_walk", "object_carrying", "careful"])
    p.add_argument("--angles", type=float, nargs="+", default=[6, 10])
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    p.add_argument("--output", type=Path, default=Path("equipment_trials.jsonl"))
    args = p.parse_args()
    r = Runner(
        Path("assets/compiled_models_wrench"),
        DEFAULT_MJCF,
        "left_rubber_hand",
        0,
        generator="onnx",
        motionbricks_root=args.planner,
    )
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 1
    r.session = ort.InferenceSession(
        "assets/compiled_models_wrench/unified_pipeline.onnx", opts, providers=["CPUExecutionProvider"]
    )
    hold = ArmHold(r.model, r.contract["joint_names"], r.kp, r.kd)
    failed = False
    with args.output.open("w") as output:
        for name in args.equipment:
            if name not in EQUIPMENT or name == "none":
                p.error(f"Invalid equipment: {name}")
            for style in args.styles:
                for angle in args.angles:
                    for seed in args.seeds:
                        hold.reset()
                        set_payload(r.model, r.data, "none")

                        def on_step(runner, _course, name=name):
                            if runner.idx == 1:
                                runner.stream.set_equipment(name)
                                hold.select(runner.data, name)
                            hold.update(runner.control_dt)
                            if runner.idx == 180:
                                set_payload(runner.model, runner.data, name)

                        result = trial(r, style, STYLE_SPEEDS_M_S[style], angle, seed, on_step=on_step)
                        result.update(equipment=name, payload_mass_kg=EQUIPMENT[name]["mass"])
                        failed |= not result["success"]
                        line = json.dumps(result)
                        print(line, flush=True)
                        output.write(line + "\n")
                        output.flush()
    raise SystemExit(int(failed))


if __name__ == "__main__":
    main()
