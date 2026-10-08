# Adaptive exertion steering — 2026-10-08

This follow-up re-enables turns, sideways motion and diagonals under exertion. Only directly
opposing translation keys are blocked. Raw requested force remains unchanged. The automatic
movement profile and a pre-brace force budget produce the effective target used by the actor,
arm feedback, virtual contact and blue force arrow. The policy and brace ONNX weights are unchanged.

## Automatic settings

| Movement | Gait and planner target | Yaw cap | Pre-brace budget per hand |
| --- | --- | --- | --- |
| Forward aligned, flat | Selected gait | — | Existing BRACE limits |
| Forward arc, flat | Slow walk, 0.50 m/s | 5°/s | 3 N |
| Side or cross-axis forward | Slow walk, 0.18 m/s | 5°/s for arcs | 2 N |
| Forward diagonal | Slow walk, 0.30 m/s; lateral/forward ≤ 0.364 | 5°/s | 2 N |
| Backward step/arc | Slow walk, 0.18 m/s; 1 s stepping / 5 s settling | 2°/s flat; 1°/s ramp | 1 N |
| Forward aligned with ramp enabled | Careful, 0.80 m/s | — | 5 N |
| Forward ramp arc | Careful, 0.80 m/s | 1°/s | 1.5 N |
| Ramp side/cross steps | Slow walk, 0.18 m/s | 1°/s | 1.5 N |

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

Seven local movement/course trials passed with the final configuration: 66-second Slow backward
stepping, both 66-second backward turns, 96-second forward and upward-force full rotations, and
two complete 10° ramp crossings. The flat backward yaw rate is 2°/s; ramp backward yaw stays 1°/s.
The final build, 27 Python tests, eight browser simulator contract scripts and three actual control
checks passed. Earlier failed candidates and incomplete turns remain recorded below.

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
- `idle-downhill`: both final 10° full-course crossings passed, including three seconds
  after actual release, with requested 8 N unchanged.

Finite trials do not guarantee stability for every force, ramp position, abrupt transition or
client timing condition. The earlier concurrent failures remain relevant even if sequential final
checks pass. Ramp edges remain physical edges; steering off them is not made safe by these caps.

## Repeat

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
