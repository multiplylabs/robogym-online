# RoboGym Online

Whole-body humanoid force control, running in a browser tab. MuJoCo-WASM for physics,
onnxruntime-web for the policy, three.js for rendering — no server, no install, the page does all of
it.

The robot is a Unitree G1 (29 DoF) tracking a reference motion while an operator either **exerts** a
commanded hand force or **compensates** an external one. Three poses are drawn at once: the
reference clip in green, the force-adjusted goal in red, and the robot itself.

Built on [mjswan](https://github.com/ttktjmt/mjswan) (Apache-2.0).

> **Everything needed to build is here**, under `assets/`: the exported policy, the reference clip,
> the robot model and its meshes, and the pre-exported brace graph. Clone, install, build.
>
> The clip is a *converted* one (0.5 MB), not the ProtoMotions motion library it came from (170 MB
> for 601 clips, of which this plays one). The brace graphs are likewise pre-exported — one per
> exerting checkpoint, `brace.onnx` and `brace_wrench.onnx` — because exporting them needs
> ProtoMotions and the training-side whole-body IK, which live elsewhere.


## Gym equipment

The public BRACE demo starts in Stealth with empty hands. The equipment gallery adds paired
**1 kg dumbbells**, a shared **3.5 kg kettlebell**, or a shared **2 kg barbell**. Pick a tile,
release the movement keys while the grip settles, then walk. Put down or Reset removes the load.
Manual force presets remain available; switching between equipment and a force preset clears
the previous load so weight is not counted twice. The reference trajectory is shown by default. The visible Compensate and Exert force actions
switch between carrying weights and pushing with a preset or custom hand force.

These are physical payloads, not renamed force arrows: hand-body mass, centre of mass and inertia
change through MuJoCo's `mj_setConst`. A 1 kg dumbbell adds approximately 9.8 N of gravity plus
inertial effects when accelerating. Shared equipment has a fixed-size model attached to the left
palm and an equality weld to the right palm, so both hands carry one rigid object. This first version
assumes ideal grips; finger articulation, slipping, dropping and equipment collisions are not simulated.

Each equipment preset solves two joint-limited 7-DOF arm poses with torso-relative palm targets.
The arms blend into those poses over 1.5 seconds using affine PD motor biases, with 3× the contract
arm stiffness and 2× damping. Their combined torque remains capped at the original motor effort
limits (25 Nm for shoulder/elbow/roll and 5 Nm for wrist pitch/yaw). Legs and waist stay under the
walking policy. Selection is also sent on the existing generator context messages; decoded reference
poses and their body velocities include the held arms, while raw MotionBricks context stays intact.
The same holding pose works across Stealth, Slow walk, Object carrying and Careful.

The initial sweep, using the earlier heavier 2 kg-per-hand dumbbells and 3 kg barbell
(with the same 3.5 kg kettlebell), completed **72/72 loaded ramp crossings**: all three objects, all four
styles, 6° and 10° ramps, and seeds 0–2. Results are in `equipment_trials.jsonl` and
`equipment_audit.json`. This is evidence for those conditions, not a guarantee for arbitrary
loads, slopes or steering.

The current lighter presets also passed a Stealth crossing of the 10° ramp for each
object (seed 0), plus browser mass, grip, removal and reset checks. These current-preset
results are in `equipment_light_trials.jsonl` and `equipment_light_audit.json`.

Run the loaded ramp audit using the public policy and native counterparts of the browser payload
and arm servo. It detects falls and leaving the ramp, including failures followed by recovery:

```sh
python -m robogym_online.check_equipment --planner /path/to/planner_sonic.onnx \
  --seeds 0 1 2 --angles 6 10 --output equipment_trials.jsonl
python -m unittest discover -s tests -p 'test_*.py'
# NODE_PATH must point to mjswan/template/node_modules. Local generator + port 8080 required.
node tests/browser_equipment.cjs
```

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
python -m robogym_online.build_app --exert --brace assets/brace.onnx --serve
```

Paths default to `assets/` and can be overridden with `--onnx-dir`, `--motion-file`, `--mjcf` or the
matching `ROBOGYM_*` environment variables.

`--serve` hosts it on localhost. The build is a plain static directory — it needs no COOP/COEP
headers, so any static host or `python -m http.server` will serve it.

### The policy picker

One scene, one dropdown, a checkpoint per entry. Each brings its own controls, because what a
checkpoint was trained to take is what the panel should offer:

```sh
python -m robogym_online.build_app \
    --exert --brace assets/brace.onnx --wrench-brace assets/brace_wrench.onnx \
    --slope-onnx-dir  assets/compiled_models_slope \
    --wrench-onnx-dir assets/compiled_models_wrench \
    --half-weights --stream ws://127.0.0.1:8765 --serve
```

| entry | flag | controls |
|---|---|---|
| Dual force student (compensation) | `--onnx-dir` (the default) | hand force; exert with `--exert --brace` |
| Slope student (terrain) | `--slope-onnx-dir` | the ramp, on from the start |
| Wrench student (force + terrain) | `--wrench-onnx-dir` | hand force, exert **and** twist with `--wrench-brace`, and the ramp, parked until asked for |

The published page ships the wrench student alone, built with `--wrench-only`: one policy that
exerts and compensates force and moment and carries the ramp, so a picker would only offer
subsets of it. The three-entry build above is the local comparison page.

### Load time

The page is about 80 MB before it can move: the policy, the ONNX runtime, MuJoCo, the robot
meshes and the brace. Two things keep that from being slower than it must be:

- `--half-weights` stores the policies' large weights as float16, halving the policy download
  (31 MB to 16 MB for the wrench student). A `Cast` back to float32 in front of each tensor keeps
  the arithmetic unchanged; the runtime folds the casts at session creation. Checked headless,
  the tracking error on the same rollouts agrees with the float32 model to the third decimal.
- The brace graphs are folded at build time (`optimize_graph`). The exporter leaves the unrolled
  IK full of shape arithmetic, 55k nodes for the wrench brace, and the browser's runtime had to
  walk all of it, on the main thread, before the robot could appear: minutes. onnxruntime does
  the folding once, natively, at build time (13k nodes; page ready in about 14 s instead of
  never), and the build refuses the result unless its outputs are identical to the original's.
- `assets/isolation-worker.js` is copied beside every build and loaded first. WebAssembly threads
  need the page to be cross-origin isolated, which takes two response headers GitHub Pages cannot
  send; without them onnxruntime runs single-threaded and MuJoCo on its unthreaded build. The
  worker adds the headers from inside the page and reloads once. Where the server already sends
  them, as `serve_dist.py` does, it does nothing.

What is left is bandwidth and the one-off WebAssembly compile, so a second visit is quicker than
the first.

The wrench student is the force student's successor: the same braced-reference contract with a
per-hand moment beside the force (`task_mode.torque_cmd_eff`), trained on terrain as well as flat
ground — so it is the one entry that carries every control the other two have between them. Every
entry is steerable from the same generator, so WASD drives whichever is selected.

**A brace belongs to its checkpoint.** Exertion is not a dial the page owns; it is that
checkpoint's whole-body construction — its endpoint stiffnesses, its feasibility caps, its lead
constants — so each exerting policy gets its own graph, and `--wrench-brace` is a second one rather
than a reuse of the first. The wrench brace additionally carries the angular twins the force
student's has no notion of: the moment dial, the twist lead, and the hand-orientation target the IK
solves for. Wiring the wrong one is refused at build time rather than read as a field that never
updates.

A checkpoint outside the family is not a flag: the build reads each one's contract and fails on
any disagreement with the scene it would share — control rate, joint and body order, gains.

| file | role |
|---|---|
| `build_app.py` | The mjswan build: scene, policy, observation groups, the brace command, the UI. Driven off the checkpoint's `unified_pipeline.yaml`, so a different checkpoint in the family is one flag. |
| `terms.py` | One observation term per ONNX input — each a shape-preserving read of MuJoCo state, the clip window, or the brace. |
| `convert_motion.py` | Reference clip → mjswan's `body_world` npz, resampled to the control rate. |
| `check_tracking.py` | Headless replica of the browser's control loop, for checking the contract without a browser. `--brace` steps the brace in the loop, which is how the exertion wiring gets checked. |

## Driving it with a keyboard

The published page plays a recorded clip. Point it at a running generator and the same page becomes
steerable instead:

```sh
# on a machine with the model (no GPU needed -- see below)
python -m robogym_online.wasd_server --generator onnx
```

then open the page with `?stream=ws://localhost:8765` (or a `wss://` host, which is what an
HTTPS-served page requires -- a secure page may not open an insecure socket).

| key | |
|---|---|
| `W` / `S` | walk forward / backward |
| `A` / `D` | sidestep left / right |
| `Q` / `E` | turn left / right |
| `space` | stop |
| `1` – `9` | locomotion style, listed bottom-left |

The policy is a mimic tracker: it follows a reference and has no notion of a velocity command, so
something upstream has to *invent* the reference. That is MotionBricks, in one of two forms:

| `--generator` | needs | notes |
|---|---|---|
| `onnx` | CPU only, ~2.1 GB | the model frozen to ONNX, as SONIC's controller ships it. 22 ms per 2.1 s of motion; 12 styles |
| `motionbricks` | a CUDA GPU, ~4 GB + 1.5 GB VRAM | the PyTorch release. 9 styles |

Both talk the same protocol, so the browser cannot tell which is behind the socket.

The wrench page offers four explicit gaits, starting with `stealth`. The
generator preserves the selected forward gait instead of silently replacing
stealth or object carrying at a speed threshold. Sideways/backward commands use
the slow gait with a 0.3 m/s target cap, including the direction-slew transition
back to forward travel. Each forward style has its own planner target speed:

| Style | Target speed | Complete ramp crossings at 6/8/10 degrees |
|---|---|---|
| Slow walk | 0.5 m/s | 15/15 |
| Stealth (default) | 0.8 m/s | 15/15 |
| Object carrying | 0.8 m/s | 15/15 |
| Careful | 0.8 m/s | 15/15 |

These are ONNX planner targets, not measured robot speeds. Slow walk has the
largest terrain-relative pelvis clearance and lowest tracking error, so it is
available as the slower alternative. Stealth starts each new session by default.
Override with `WASD_SLOW_WALK_SPEED`, `WASD_STEALTH_SPEED` or
`WASD_OBJECT_CARRYING_SPEED`. The legacy `walk` style retains its automatic gait
selection and `WASD_WALK_SPEED` override for other clients.

The slope preset stays at 6–10 degrees, with the toe 3 m ahead of the robot, a
2.5 m ascent, 2 m plateau, matched descent and 4 m width. Validation used five
seeds at each angle; success requires crossing the whole course and stopping
upright for 3 seconds. Another 27 runs at 10 degrees passed with short 10/20
deg/s steering pulses or a 10 N hand disturbance, tested separately. Continuous
turning, exertion contact and an actual carried payload were not covered.
The object-carrying entry is a motion style, not a simulated loaded object.

Very low speed was not uniformly safer: object carrying at 0.3 m/s left the
course at 8 degrees, and native slow walk fell on the descent at 12 degrees.
12-degree results were screened with one seed, so the default upper limit stays
at 10 degrees. These are observed simulation results, not guaranteed success.

```sh
python -m robogym_online.check_slope --planner /path/to/planner_sonic.onnx \
  --styles slow_walk --speeds 0.5 --angles 6 8 10 --seeds 0 1 2 3 4
# Use stealth or object_carrying with --speeds 0.8 for their validated settings.
# At 10 degrees, optional stress cases: --turn-pulse 10 or --hand-force 10 0 0
```

Raw trials are in `slope_validation.jsonl`, `slope_steering_load.jsonl`,
`slope_screening.jsonl` and `slope_angle_screening.jsonl`; recordings in
`slope_previews/` show each selected gait crossing a 10-degree ramp.

The ONNX generator applies root feedback to uncommitted output frames, capped at
2 mm per emitted frame. Its next context already includes that translation.
Applying the accumulated offset to each new context instead produced backward
root jumps up to 7.3 cm in a 30-second walk/stop test. Height, orientation and joint
poses are left to MotionBricks; no extra ground-height or yaw correction is applied.
Travel direction now slews at 60 degrees per second (three seconds for a full
reversal). This also kept the strafe reversal upright with a 10 N hand load in
the three-seed check; a faster direction sweep still fell in that loaded case.

To repeat the steering regression (idle, walk, strafe, reverse, back, turn, stop):

```sh
python -m unittest discover -s tests -v
python -m robogym_online.check_stability --planner /path/to/planner_sonic.onnx
# Compare another checkpoint without replacing the page's policy or brace:
python -m robogym_online.check_stability --planner /path/to/planner_sonic.onnx \
  --onnx-dir /path/to/g1_wrench_v12/student/compiled_models
# Exercise the live websocket and its session defaults:
python -m robogym_online.check_stability --remote ws://127.0.0.1:8765 --seeds 0
```

`ROBOGYM_PLANNER_ONNX` may name either a GR00T checkout or the planner ONNX file.
The check exits unsuccessfully at the first fall and prints minimum pelvis height,
joint error and, for a local generator, maximum reference root step and final lag.
It checks compensation with optional `--hand-force fx fy fz`; exertion and ramp
behavior require separate checks with the checkpoint's matching brace.

## Driving it with a camera

The keyboard invents a velocity command; a camera can invent it instead -- and supply the upper body
while it does. The `camera_motionbricks` generator drives the same MotionBricks legs from a single
RGB camera: it takes the operator's **upper body** (waist lean + both arms) from the camera retarget
and their **root velocity** as the walk command, and leaves the legs to MotionBricks. This is the
same upper-from-operator / lower-generated split GR00T's 3-point VR teleop uses -- monocular legs are
the worst-estimated part of a camera capture, so they are the part MotionBricks replaces.

Three processes, because the vision stack and the generator live in different environments:

```sh
# 1. camera -> full-body G1 qpos on :28701/2   (in teleop_camera/, its own conda envs)
cd ~/Humanoid2/extras/motion_tracking/sim2real/teleop_camera
./run_camera_teleop.sh --soma            # RealSense -> GEM-X -> SOMA retarget

# 2. the browser half: build (once) + static host + the camera-driven generator
cd ~/Humanoid2/robogym-online
./run_camera_browser_teleop.sh           # serves dist/ and ws://localhost:8765
```

Then open `http://localhost:8080/?stream=ws://localhost:8765`. Stand 2-3 m back with feet in frame
(the retarget's framing rules apply -- see `teleop_camera/README.md`); step forward to walk, turn
your body to turn, raise your arms and the robot's arms follow, all tracked by the force policy.

| what the camera drives | what MotionBricks drives |
|---|---|
| waist roll/pitch (DoF 13,14), both arms (15-28) | legs (0-11), waist yaw (12) |
| root linear velocity -> `forward` / `lateral` | the gait itself (swing, stance, push-off) |
| root yaw rate -> `turn` | |

The root velocity is finite-differenced from the retarget and deadbanded, so a stationary operator
commands a stand rather than integrating estimator drift. A browser *movement* command is ignored in
this mode (the camera owns it); number-key style selection still works.

**No camera to hand?** `fake_camera_qpos` replays a scripted walk-and-wave on the same sockets, so
the whole browser path is testable with no gemx, no SOMA and no RealSense:

```sh
python -m robogym_online.fake_camera_qpos      # in the generator's env, one terminal
./run_camera_browser_teleop.sh                 # another
```

Setup notes: the generator runs in an env with torch+CUDA, `motionbricks` and `pyzmq` (the conda
`motionbricks` env here, with `pyzmq` added); the page is built by `build_app.py` in an env with
`mjswan` (the `.venv`), which the launcher does once if `dist/` is not already stream-capable.

## The controls

The interactive page is branded **BRACE** and links back to the research page. The primary
view keeps walking styles, reference visibility, the slope switch and reset visible. Open
**Forces & interaction** and choose **Compensate** (external hand load) or **Exert** (robot push).
Choose the hand and direction, then click a preset to apply it immediately, or enter a custom
value in newtons and click **Apply force**. The force stays active until **Clear force**;
changing mode, hand or direction clears the previous force. Compensation presets are 5/10/20 N,
and exertion presets are 2/5/8 N. Custom values respect the native control limits (25 N per hand
for compensation, 20 N each when both hands are selected to respect the 40 N total budget;
9 N per hand for the current exertion checkpoint). Compensation uses world axes; exertion follows
the robot's heading. **Individual axes** restores the original sliders.
**Display & model settings** restores
the model selectors and visualization switches. **Annotations** explains the scene colors
and suggests experiments. Direction keys also work as hold-to-move touch controls.
The floor uses white and light-blue checker tiles, with a stronger blue ramp and a light sky.
The presentation files are `assets/brace-ui.css`, `assets/brace-ui.js`, and `assets/brace-forces.js`; the builder installs
them beside both entry pages, so no installed mjswan source needs editing.

**Careful walking** is now included in the wrench page at a **0.8 m/s planner target**
(`WASD_CAREFUL_SPEED` to override). It is an existing MotionBricks mode, newly exposed in
this demo. Screening careful, happy and injured at native, 0.6 and 0.8 targets on a 10° ramp
found careful strongest: 3/3 completed; happy left the course once; injured fell twice.
Careful at 0.8 then completed 15/15 full-ramp trials across 6/8/10° and five seeds, 9/9
10° steering/load trials (10°/s or 20°/s brief turn pulses, or a constant 10 N hand load,
tested separately), and 3/3 flat-ground walk/strafe/reverse/stop trials. These are measured
successes for these conditions, not a guarantee of failure-free behavior. See
`careful_audit.json` and its referenced JSONL trial records. The carrying style still has
no physical payload attached.

**Hand Force** — six sliders applying an external force *to* each hand, in world axes, for
compensation. Arrows show what is applied.

**Exert** — a mode switch, then a force vector per hand in the **torso-yaw frame**: yaw-invariant, so
`+x` is the robot's own forward wherever it is facing. Same definition as the training override
`constant_force_xyz`. The mode switch cross-fades over 24 control steps, as training did, so the
policy never sees a force step at a mode change — only the flag change an operator actually commands.

**Twist** — on the wrench student only, a moment per hand beside the force, in the same frame and
behind the same fade. The two are not independent budgets: they run through the same arm actuators,
so the brace applies one combined effort cone that bounds both, and a hand already pushing hard has
less moment left to give. Watch `force_cmd_eff` and `torque_cmd_eff` fall together as either dial
climbs — that is the cone, not a bug.

**The gauge** — Exert shows a blue spring-loaded instrument at each active hand, with a moving
pad and piston. The panel repeats target (blue), measured reaction (amber), and signed error in
newtons. Target is the effective post-cap command; an 8 N request can be reduced by reach,
effort or balance limits. Measured force comes from the Kelvin-Voigt contact applied to the robot,
not the command or a display animation. The instrument is a visual representation of that virtual
contact, following the reference station; it is not an independently colliding world object.

Selecting Exert eases the upper-body reference into a fixed torso-relative IK pose over 1.5 seconds,
without altering raw planner context or leg references. Torque-limited arm servos hold the pose;
a slow Jacobian force-feedback correction adjusts their targets to achieve the effective force.
Offsets are bounded to 0.5 rad and slew to 0.5 rad/s, with original actuator torque and joint limits.
Feedback gain is 6/s, with a 0.07 s measurement filter and 0.002 Jacobian regularization.
The legs and torso remain controlled by the wrench policy. Interaction controls wait for the live
stream handover and initial reset before accepting actions, avoiding cleared startup commands. Clear releases the contact and returns
the hands toward their neutral pose; selecting weights or Reset clears exertion.

`assets/handSpringContact.ts` and `assets/externalWrench.ts` are bundled during builds. Both
accumulate into the force buffer, which the runtime clears once per control step. The old inactive
contact zeroed hand buffers after manual loads were applied, silently removing compensation forces.
Feedback now reads the spring sensor separately from other loads and mouse forces. It
matches training's signed smooth lead clamp and damps an EMA-filtered derivative of **raw** lead
(`damper_vel_ema_alpha=0.3`), independently primed for each hand. The previous browser instead
differentiated saturated lead without the training EMA. The build temporarily overlays the
mjswan contact source and restores the dependency in a `finally` block. `tests/contact_parity.cjs`
checks the training equations, signed forces, inactive-hand reaction and activation/reset spikes.
`tests/browser_exertion.cjs` checks actual force tracking, directions, both hands and cleanup.
`tests/browser_exertion_motion.cjs` checks the four walking styles and a complete ramp crossing.
See `force_audit.json` for measured results: ten idle cases averaged less than 0.10 N error
after settling; the public 5 N walking/slope trial averaged 0.71–0.93 N error, with no falls.
Release movement keys for a steady gauge reading. These are individual trials, not a guarantee
for every gait, force, or terrain configuration.
The graph does not publish the training estimator's cached brace-credit lead; the displayed
reaction uses arm spring displacement and damping only, with no synthetic brace-force credit.
Force only: no torsional contact is implemented for the advanced moment dial.

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
  one that fits the target hardware. The wrench brace costs more per iteration — it solves a hand
  *orientation* task as well as a position one — so it ships at 5 (1.9e-2 rad, ~8.9 ms), which buys
  the force brace's accuracy at three times its cost.
- **ORT-Web rejects the optimized session.** Its constant-folding pass fails on the brace graph with
  a misleading `HasExternalDataInMemory` error; the engine retries unoptimized, which works.

## The mjswan fork

The app depends on additions to mjswan that are generic rather than task-specific — windowed
reference fields, structured policy inputs, a pose ghost, an operator wrench and the hand spring
contact, and the ORT-Web session fallback. They live on a fork, pinned in `pyproject.toml`, and each
is tracked for a pull request back upstream.
