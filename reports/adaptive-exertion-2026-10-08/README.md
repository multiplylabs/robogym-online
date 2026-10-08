# Adaptive exertion steering — 2026-10-08

## Force preservation revision

The public 2 N side-step limit was too aggressive for the requested 8 N push. Normal
forward/lateral/vertical pushes now use a **6 N per-hand pre-brace movement budget** for
side steps, arcs, diagonals and slope travel. An 8 N request drops by 2 N at this stage;
a request at or below 6 N is unchanged. Aligned straight flat travel retains the original
BRACE budget. Requested dial values remain intact, and measured force still comes from
the physical virtual contact. BRACE reach/effort constraints and emergency tipping can
reduce the effective target further; 6 N is a budget, not a guaranteed physical minimum.

Movement carries the stability adjustment: flat forward arcs use Slow walk at 0.30 m/s
and at most 2°/s, giving an approximately 8.6 m planner radius. The forward diagonal
component is limited to lateral/forward ≤ 0.20. Pure side steps stay at 0.18 m/s.
Straight loaded ramp travel and non-diagonal ramp arcs retain Careful at 0.80 m/s;
ramp diagonals use Slow walk at 0.30 m/s with at most 1°/s yaw. Only directly opposing
translation is blocked. Routine torso lean up to 0.48 rad (27.5°) no longer collapses
the force budget. Above that, emergency contact unloading ramps down; the separate
predictive fall guard is unchanged.

The backward push/backward travel branch retains its earlier 1 N budget and 1 s step /
5 s settle cadence. A 6 N backward-push candidate failed after 9.94 seconds despite the
reference-buffer fixes (`preserve-backward`). It is not deployed. This limitation is
also stated in the scene annotations.

### Local force-preservation trials

Every fall or predictive respawn counts as failure. Trials run sequentially against the
real public generator tunnel, from the local page, with six simulation seconds after
actual key release. The original actor, brace graph and auxiliary arm controller remain
unchanged. Command packets and complete force measurements are retained.

| Case | Effective target while moving | Duration | Result |
| --- | --- | ---: | --- |
| Forward push, right hand, left/right side reversal | 6 N | 36.20 s | No fall |
| Forward push, both hands, left/right side reversal | 6 N per hand | 36.10 s | No fall |
| Full-force side reversal candidate, both hands | 8 N per hand | 36.06 s | No fall; measured medians 7.85 / 7.81 N |
| Forward push, ramp with turns/diagonals and reversal | 6 N per hand | 36.12 s | No fall; full course travel |
| Upward push, moving arc | 6 N per hand | 54.08 s | No fall; −240.7° rotation |
| Revised broad forward arc, diagonal reversal, release | 6 N per hand | 42.08 s | No fall; measured medians 6.00 / 5.99 N |
| Revised broad upward arc, diagonal reversal, release | 6 N per hand | 42.18 s | No fall |

The initial higher-force forward/turn/sideways candidate at 0.50 m/s and 5°/s failed at
19.58 seconds (`preserve-followup`). Slowing and widening its movement, while retaining
6 N, produced the selected 42-second passes (`preserve-tune`). The 8 N side-only candidate
is retained as evidence, but the default keeps the tested 6 N budget across combined
movement. These finite trials do not establish universal fall-free behavior.

The source contract checks cover requested-force preservation, the 6 N movement budget,
ordinary ramp lean without excess unloading, genuine emergency reduction, lower requests,
and compensation bypass. Browser checks verify the 6 N slope/arc target, raw 8 N dial,
only-opposite key restriction, restored commands after reconnect, and clear-force recovery.

## Earlier investigation and results

The settings and trial results below record the earlier low-force configuration and the
reference-stream fixes. They do not validate the new higher-force defaults. Current
higher-force validation is in `preserve6`, `preserve-followup` and `preserve-tune`.


This follow-up re-enables turns, sideways motion and diagonals under exertion. Only directly
opposing translation keys are blocked. Raw requested force remains unchanged. The automatic
movement profile and a pre-brace force budget produce the effective target used by the actor,
arm feedback, virtual contact and blue force arrow. The policy and brace ONNX weights are unchanged.

## Earlier conservative settings (superseded)

| Movement | Gait and planner target | Yaw cap | Pre-brace budget per hand |
| --- | --- | --- | --- |
| Forward aligned, flat | Selected gait | — | Existing BRACE limits |
| Forward arc, flat | Slow walk, 0.50 m/s | 5°/s | 3 N aligned +X; 2 N otherwise |
| Flat side or cross-axis forward without turning | Slow walk, 0.18 m/s | — | 2 N |
| Forward diagonal without turning | Slow walk, 0.30 m/s; lateral/forward ≤ 0.364 | — | 2 N flat; 1.5 N ramp |
| Forward diagonal with turning | Slow walk, 0.50 m/s flat; 0.30 m/s ramp | 5°/s flat; 1°/s ramp | 2 N flat; 1.5 N ramp |
| Backward step/arc | Slow walk, 0.18 m/s; 1 s stepping / 5 s settling | 2°/s flat; 1°/s ramp | 1 N |
| Forward aligned with ramp enabled | Careful, 0.80 m/s | — | 5 N |
| Forward ramp arc | Careful, 0.80 m/s | 1°/s | 1.5 N |
| Ramp pure side/back steps | Slow walk, 0.18 m/s | 1°/s | 1.5 N side; 1 N backward |
| Straight ramp travel with upward/lateral push | Careful, 0.80 m/s | — | 1.5 N |

These are planner targets, not guaranteed measured robot speeds. Backward arcs reverse their
travel component when forward translation directly opposes a backward push. Settling pauses stop
both travel and yaw. Reduction uses a 0.12 s time constant; recovery uses 1 s with a 1.6 s hold.
The backward 1 N budget also applies at rest, before the first step. Torso tilt above 0.30 rad further reduces the moving force budget. BRACE's reach, effort and balance
constraints can reduce the target further. Force mode, signs and requested dial remain intact;
compensation bypasses this envelope. Clear force restores the selected style and ordinary steering.

The budget is applied before BRACE constructs the pose and effective force, following its
[feasible-wrench then pose construction](https://arxiv.org/html/2610.07052v1) ordering. Scaling the
published target afterward would disagree with the pose passed to the actor. The existing wrench
policy remains active; no unsupported wrist torque or modified measured-force value is introduced.
Reference coordinates remain the verified training convention, with a flat reference and shared XY
origin. Gait changes happen in the generator, and the browser shows the automatic gait override.

## Selected configuration validation

Seven local movement/course trials passed with the movement presets: 66-second Slow backward
stepping, both 66-second backward turns, 96-second forward and upward-force full rotations, and
two complete 10° ramp crossings. However, the first public audit subsequently failed all three
extended turning cases. Its straight 10° ramp crossing passed. Those failures are retained in
`public`; the ramp success is in `public-downhill`.

The follow-up found a streaming defect: physics continued while the reference cursor stalled,
and the policy's future samples collapsed onto the last received walking pose. The diagnostic
`network` trial recorded 497 additional context reports with fewer than 20 future frames during
movement. That trial remained upright, so starvation alone does not establish the cause of every
fall. It does violate the reference contract and can destabilize tracking.

The final simulator now checks that all 20 future frames are available before each policy/physics
step. It requests more data and pauses the entire simulation clock while waiting. It does not
advance the motion cursor, integrate contact or fall timers, or simulate against a frozen walking
pose. Buffered playback resumes without skipping frames. Initial adoption also waits for a full
horizon. A small "Buffering motion…" label indicates these pauses. An unnecessary repeated gait
label rewrite in the UI observer was also removed. The final scheduler yields a browser task after
every frame, including frames whose inference exceeds the 20 ms budget. A chain of already resolved
Promises is not sufficient to deliver keyboard and WebSocket events. This improves callback delivery
but does not bound rendering or inference latency on every client.

Three repeat trials over the public generator tunnel passed with this protection and the original
arm controller. Raw requests stayed at 8 N. All sampled context reports had at least 20 future
frames; natural buffering pauses occurred during the trials.

| Final local tunnel trial | Duration | Actual rotation | Outcome |
| --- | ---: | ---: | --- |
| Forward + turn + side, reversal, release | 66.52 s | +103.81° cumulative | No fall |
| Upward push, full forward arc, release | 96.18 s | −454.36° | No fall; 34.66 m path |
| Backward push in both hands, backward arc, release | 66.18 s | +10.38° | No fall |

Each observes at least six simulation seconds after actual key release. The measured rotations
are cumulative and do not imply that every command is a continuous spin. Backward travel still
uses the tested step/settle cadence. Complete traces are in `stream`.

An experimental 35% auxiliary arm-servo blend also passed three trials (`policy-blend`). Its first
case ran without the buffer protection; the later cases used it. This mixed experiment does not
isolate an arm-blend benefit. It is **not deployed**: the repeats with the original controller
passed after correcting buffering.

The final build, 27 Python tests and eight simulator contract scripts passed. The new
`browser_reference_buffer.cjs` integration test deliberately withholds incoming frames while
exerting and turning: clock and root pose remain unchanged during the stall, the buffering label
appears, and motion resumes after delivery. Four public trials with the deployed guard passed (`public-stream`, `public-stream-downhill`):
forward/turn/side reversal for 66.12 s, upward-force rotation for 96.08 s / −452.78°, backward-push
turning for 66.04 s / +11.70°, and a full 10° downhill course with recovery. Requested force was
8 N throughout, and no trial fell or respawned.

A final preset correction prevents turning plus sideways motion on a ramp from using Careful's
0.80 m/s speed while the planner selects Slow walk. It uses 0.30 m/s, keeps both turn and side keys
enabled, and retains the 1°/s ramp yaw cap. The combined 36-second ramp/reversal/release trial
passed before and after the scheduler correction (`diagonal`, `final-loop`), with the final run
reaching 1.225 m root height and crossing back onto flat ground without a fall.

The overrun integration check injects 25 ms of work into each frame and verifies that browser
callbacks continue, simulation time/root pose freeze while incoming frames are withheld, exertion
remains active after delivery, and no automatic respawn occurs. An initial additional 1-second
callback-gap limit failed at 1.124 s under software rendering; that log is retained. The test checks
callback progress rather than assuming a universal latency limit. Final publication verification
of the combined ramp preset and scheduler is in `public-final-loop` when complete.

Straight ramp travel under an upward or lateral push also keeps Careful at 0.80 m/s rather than
compressing it to the flat cross-axis pace of 0.18 m/s. Its force budget remains 1.5 N per hand.
Two full-course tunnel trials passed (`final-cross`): upward 8 N in both hands, 23.10 s / 17.41°
maximum tilt; lateral 8 N in both hands, 22.48 s / 19.84° maximum tilt. Both include at least three
simulation seconds after actual release. These are force-axis changes, not hidden command reductions.
Requested force remains 8 N while the effective target adapts. Public repeats are in `public-final-cross`
when complete.

## Method and retained attempts

A fall or automatic respawn is a failure. Movement trials use fresh browser sessions, real ONNX
motion generation, the deployed wrench actor, MuJoCo and the real force contact. Final trials run
one browser at a time. Full-rotation trials require at least 350° of actual rotation and 20 m of
both robot and reference path travel. They continue after releasing the keys to test recovery.
The downhill audit requires ascent, plateau, descent, staying on course and three seconds after
releasing W. Command packets and requested-force preservation are recorded.

`summary.json` contains counts by phase. Each directory contains case settings, compact results
and compressed complete samples (`traces.json.gz`). No failed attempt is omitted:

- `screen`: initial candidate, 7 passes / 2 failures. Slow-walk ramp arcs and continuous backward
  walking failed; these settings were replaced.
- `parameters`: 7 passes / 2 failures. Flat rotations completed with 3, 5 and 8 N budgets; several
  side trials also passed higher budgets. The lower defaults were retained for a broader movement
  envelope. Merely widening a Slow-walk ramp arc or reducing continuous backward force to 1 N
  still failed.
- `recovery`: 2 failures with the eventual Careful ramp / settling backward settings while other
  browser audits were concurrently active.
- `quiet`: the same two cases passed when repeated sequentially. This does not establish that
  concurrency caused the failures; both outcomes are retained as a robustness limitation.
- `final`: longer sequential trials cover both full rotations, sideways/diagonal reversals,
  lateral-push transitions, signed vertical pushes, backward travel/arcs, both ramp turn directions,
  ramp side transitions, key release and the directly opposing key restriction.
- `backward`: the 8 N left-hand straight trial failed. A 0.2°/s backward yaw candidate kept
  both hands' trials upright, but the left-hand trial turned only 0.33°; this is insufficient
  turn response and was not selected merely for remaining upright.
- `backward-rest`: a 1 N backward budget applied while standing passed straight stepping,
  but its 1°/s backward arc still fell after 43.6 seconds.
- `backward-side-arc`: a sideways arc under a backward push failed for the left hand and
  passed for the right. This variant is not deployed.
- `combined`: forward+turn, then forward+turn+side combinations and reversals passed.
- `downhill`: full-course 10° crossings under an unchanged requested 8 N, with automatic Careful
  and an effective forward-travel budget of 5 N.
- `strict`: the generator applies the exertion speed preset on the first planned frame rather
  than inheriting a ramped speed from the previous gait. The right-hand backward arc passed
  with 12.6° of rotation, but left-hand backward trials still failed.
- `cadence`: tests shorter backward steps and longer settling, with the reduced backward
  budget already active at rest, using the corrected generator speed input.
- `idle`: adds zero speed while Idle and after release. Left-hand Slow backward stepping and
  its backward turn passed; the right-hand 1°/s turn remained upright but turned only 2.75°.
  Forward and upward-force full rotations passed. This 1°/s flat backward rate is superseded.
- `back-yaw2`: the selected 2°/s flat backward rate passed both hand cases for 66 seconds,
  with 18.88° left-hand and 16.02° right-hand actual turns. Actual release times are recorded;
  each trial observes at least six simulation seconds after release.
- `idle-downhill`: both preset-validation 10° full-course crossings passed, including three seconds
  after actual release, with requested 8 N unchanged.

Finite trials do not guarantee stability for every force, ramp position, abrupt transition or
client timing condition. The earlier concurrent failures remain relevant even if sequential final
checks pass. Ramp edges remain physical edges; steering off them is not made safe by these caps.

## Repeat

`stream/cases.json` repeats the three turning cases that failed the initial public audit. Use
`tests/browser_reference_buffer.cjs` for the deterministic network-stall check. The policy ONNX
weights, brace graph, force coordinate conventions and holding reference remain unchanged.


Use the built page on port 8080 and a dedicated generator running the same source:

```sh
export NODE_PATH="$PWD/.venv/lib/python3.12/site-packages/mjswan/template/node_modules"
BRACE_TEST_URL='http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765' \
BRACE_AUDIT_CASES_FILE=reports/adaptive-exertion-2026-10-08/final/cases.json \
BRACE_AUDIT_OUTPUT=/tmp/brace-adaptive-repeat BRACE_AUDIT_CONCURRENCY=1 \
node tests/audit_exertion_directions.cjs
```

`final-python.log` records the simulator/backend regression suite. `final-build.log` records the
successful full app build. Adaptive generation supplies the preset on the first frame, including zero while Idle, instead of inheriting the selected walking speed after key release.
The deployment workflow also runs contact parity, reference-input,
steering-lock and pre-brace budget checks. `browser_slope_profile.cjs` checks actual controls,
requested/effective targets, reconnects, tooltip reasons and clear-force restoration.
