# Reference input audit

The exported actor expects a flat MotionBricks reference. The green display remains ground-aligned and does not modify policy inputs.

## Local training and inference evidence

- `seahorse/projects/robust_wbc/experiments/terrain_mimic/mlp_bm_force_terrain_xprivgoal_2head_stockgains.py:30–83`: the force goal is expressed over the flat reference; terrain conformity belongs to privileged reward/decoder targets. O3 removes absolute target height (`include_height=False`, `include_xy_offset=False`). The actor has no terrain heightmap or slope normal.
- `seahorse/projects/robojudo/force_track_policy.py`: the current/brace reference uses the nearest future sample, with the same heading alignment as the future goal.
- `assets/compiled_models_wrench/unified_pipeline.yaml`: 0.02 s control interval, 29 joints, 33 bodies, torso anchor, and future offsets `[1,2,3,4,8,12,16,20]`.
- `assets/TrackingCommand.ts`: shared XY canonical origin, retained flat reference Z, matched joint position/velocity samples, normalized wxyz stream quaternions. The ONNX boundary converts to its declared xyzw order.

## Checks

`tests/reference_inputs.cjs` bundles the actual TrackingCommand implementation. It verifies future offsets, both body slots, joint position/velocity pairing, root/body consistency, quaternion values and normalization, shared XY origin, and last-frame holding at the live buffer frontier.

`tests/test_reference_frame.py` runs the deployed ONNX at three random inputs, then shifts every reference/force-goal position input together by `[12.5,-8.25,0]`, `[0,0,0.5]`, and `[0,0,-0.5]`. Every output remains equal within the existing 2e-4 tolerance; observed differences were below 3e-7.

A live unloaded Slow walk diagnostic on the 10° course sampled the policy inputs and brace output. `x_priv_bodies` matched the nearest future `ref_body_pos_w` exactly (maximum and mean absolute difference zero). This diagnostic intentionally stops before completing the course and is not counted as a successful crossing.

These checks found no input-frame mismatch to fix. They do not prove universal stability or complete training/deployment equivalence under every load. Loaded full-course tests separately reproduce actual locomotion failures. Reference inputs, force magnitudes, effective-force equations and contact physics are preserved.
