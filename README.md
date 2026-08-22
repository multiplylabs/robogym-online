# RoboGym Online

Whole-body humanoid force control, running in a browser tab. MuJoCo-WASM for physics,
onnxruntime-web for the policy, three.js for rendering — no server, no install, the page does all of
it.

The robot is a Unitree G1 (29 DoF) tracking a reference motion while an operator either **exerts** a
commanded hand force or **compensates** an external one. Three poses are drawn at once: the
reference clip in green, the force-adjusted goal in red, and the robot itself.

Built on [mjswan](https://github.com/ttktjmt/mjswan) (Apache-2.0).

> **The policy is not in this repo.** The checkpoint, the reference clip, the robot model and the
> pre-exported brace graph are fetched at build time. What lives here is the browser app and the
> bridge to the policy's contract.

## What is interesting here

A force-exertion policy of this kind is not deployable from its weights alone. It observes a
*force-braced reference* — the posture that produces the commanded force — which the deploy runner
has to compute every control step by solving a whole-body damped-least-squares IK. Running the
policy client-side therefore means running that solver client-side too.

- **The brace is an ONNX graph.** The deploy-side estimator is exported whole — the endpoint
  stiffness, the torque cone, the CoP-aware balance cap, the first-order contact lead, and 10
  iterations of whole-body IK — and stepped by the browser as a stateful command term. Its one
  unexportable operation, a damped-least-squares `solve`, becomes Jacobi-preconditioned conjugate
  gradient; measured against the original, that costs 0.19% of the published goal.
- **The contact is real.** Exertion is not a passive arm lead. A Kelvin-Voigt contact is anchored at
  the reference hand, the reaction loads the whole body, and the gauge reads the force that implies —
  the same quantity the training reward measured.
- **The observation assembly is already in the policy graph.** The exported pipeline traces its
  observation builders in, so its 25 inputs are raw context fields rather than a flattened vector.
  The browser never reproduces the policy's observation math; it serves raw state in the right frames
  and units. That is what makes a port of this tractable at all.

## Running it

```sh
pip install -e .
# Assets: see the ROBOGYM_* defaults in build_app.py, or pass the flags.
python -m robogym_online.build_app --exert --brace assets/brace.onnx --serve
```

`--serve` hosts it on localhost. The build is a plain static directory — it needs no COOP/COEP
headers, so any static host or `python -m http.server` will serve it.

| file | role |
|---|---|
| `build_app.py` | The mjswan build: scene, policy, observation groups, the brace command, the UI. Driven off the checkpoint's `unified_pipeline.yaml`, so a different checkpoint in the family is one flag. |
| `terms.py` | One observation term per ONNX input — each a shape-preserving read of MuJoCo state, the clip window, or the brace. |
| `convert_motion.py` | Reference clip → mjswan's `body_world` npz, resampled to the control rate. |
| `check_tracking.py` | Headless replica of the browser's control loop, for checking the contract without a browser. |

## The controls

**Hand Force** — six sliders applying an external force *to* each hand, in world axes, for
compensation. Arrows show what is applied.

**Exert** — a mode switch, then a force vector per hand in the **torso-yaw frame**: yaw-invariant, so
`+x` is the robot's own forward wherever it is facing. Same definition as the training override
`constant_force_xyz`. The mode switch cross-fades over 24 control steps, as training did, so the
policy never sees a force step at a mode change — only the flag change an operator actually commands.

**The gauge** — at each hand, the commanded force (blue) against the exerted force (orange), as
arrows and as a reading in newtons. Commanded is the *effective* post-cap value the policy observes,
not the raw dial, so the two numbers are directly comparable.

## Conventions that will bite

Each of these is silent when wrong — the robot almost tracks, or a feature quietly does nothing.

- **Quaternion order.** MuJoCo and mjlab are wxyz; the policy contract is xyzw. `terms._to_xyzw` is
  the only place that flips.
- **Body and joint order.** The model's compiled order (worldbody excluded) must equal the contract's
  `body_names` / `joint_names` index for index. Asserted at build time.
- **History direction.** `historical.*` is newest-first, primed from the current frame on reset.
- **The driven output.** The policy emits four tensors; the robot is driven by `joint_pos_targets`
  (absolute PD targets), not the raw `actions` residual. The runtime keys outputs positionally, so
  the io-keys JSON puts its `action` key in that slot.
- **Action history semantics.** `historical.processed_actions` holds those same absolute targets,
  seeded at the clip's first pose. That channel is the policy's implicit force observer — the load is
  only recoverable as the gap between commanded target and achieved position — so zeros inject a
  fictitious whole-pose step.
- **The brace command's wire contract.** `OnnxCommand` feeds `prev_<name>` and reads `next_<name>`.
  A bare output name matches nothing: the state never updates and every consumer reads the init value
  forever. The state fields are derived from the graph so this cannot drift.
- **`x_priv` at zero force.** The `hand_force.*` inputs are the *reference* pose, zero deltas, and an
  **identity** rotation delta — not zeros. The goal builder right-multiplies the reference anchor
  rotation by that delta, so a null quaternion annihilates the torso-orientation block.

## Known limits

- **The brace blocks the frame.** onnxruntime-web runs on the main thread (its proxy worker throws
  `document is not defined` as bundled), so a brace step stalls rendering rather than merely missing
  a control step. Off-thread inference needs the worker chunk emitted properly first.
- **Brace fidelity is a dial.** Outer IK iterations trade cost against goal error: 10 (the trained
  value) is 1.3e-3 rad at ~29 ms per solve single-threaded; 2 is 1.8e-2 rad at ~2.8 ms. Export the
  one that fits the target hardware.
- **ORT-Web rejects the optimized session.** Its constant-folding pass fails on the brace graph with
  a misleading `HasExternalDataInMemory` error; the engine retries unoptimized, which works.

## The mjswan fork

The app depends on additions to mjswan that are generic rather than task-specific — windowed
reference fields, structured policy inputs, a pose ghost, an operator wrench and the hand spring
contact, and the ORT-Web session fallback. They live on a fork, pinned in `pyproject.toml`, and each
is tracked for a pull request back upstream.
