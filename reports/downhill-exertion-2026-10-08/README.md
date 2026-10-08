# Loaded downhill audit — 2026-10-08

The reported Slow walk downhill instability was reproduced during forward force exertion. The reference window, quaternion ordering and coordinate origin were consistent with the exported policy. Its actor deliberately uses a flat reference without absolute target height. See [reference-audit.md](reference-audit.md).

## Movement change

When forward exertion is active and the ramp is enabled, permitted Slow walk travel temporarily uses **Careful at its native 0.8 m/s**. The selected style remains visible, with a small override caption. Clearing force or disabling the slope restores Slow walk at 0.5 m/s. This is a gait/command change; requested forces, effective-force equations, brace graph, contact physics and green reference visualization are unchanged.

Gait switching alone did not make the steep 8 N case reliable. Forward walking is therefore disabled for:

- A forward push above 5 N per hand on ramps above 8°.
- Two forward-pushing hands when either exceeds 5 N.
- Any forward push above 8 N per hand on a ramp.

W dims and explains the reason on hover. An already-held W stops when the restriction activates; keyboard and direct command paths obey it. Clearing force or disabling the slope enables W again. These are conservative movement permissions, not force caps. Other previously tested direction restrictions remain in place.

## Completed crossings within the final envelope

Fresh Chromium sessions ran the actual MuJoCo-WASM simulation, deployed wrench ONNX and ONNX MotionBricks generator. Each success required staying on the course through ascent, plateau and descent, then observing for three further simulation seconds after releasing W. Any fall or automatic respawn failed the attempt; leaving the course was also unsuccessful.

| Selected style | Forward force | Ramp | Completed without a fall |
| --- | --- | --- | --- |
| Slow walk → Careful | Right hand, 5 N | 10° | 3/3 |
| Slow walk → Careful | Left hand, 5 N | 10° | 2/2 |
| Slow walk → Careful | Both hands, 5 N each | 10° | 3/3 |
| Slow walk → Careful | Right hand, 8 N | 6° | 3/3 |
| Slow walk → Careful | Right hand, 8 N | 8° | 3/3 |
| Slow walk → Careful | Right hand, 2 N | 10° | 1/1 |

The last ten repeated crossings all passed, taking 21.5–24.1 simulation seconds including the observation after releasing W. Fresh sessions are repetitions with session timing variation, not a claim of exhaustive random-seed coverage. No test called a post-fall recovery a success.

Three blocked-input checks also passed 20 simulation seconds each (single-hand 8 N at 10° for Slow walk and Stealth, and two-hand 8 N each at 6°). Commands stayed zero, the force requests remained unchanged, and XY drift stayed below 4 cm. Blocked checks are kept separate from crossing successes.

## Unsuccessful attempts retained

- Slow walk at 0.5 m/s with right-hand 8 N fell on the 10° descent after 31.0 s. A 0.4 m/s trial fell on the plateau after 35.2 s; 0.35 m/s left the course on descent after 59.5 s.
- Careful reduced to 0.5/0.6 m/s fell on descent after 44.0/27.1 s.
- Native Careful at 0.8 m/s passed one right-hand 8 N / 10° trial but the actual selected-Slow-walk fallback failed another during ascent after 14.8 s. Two-hand 8 N failed on approach after 9.3 s. These combinations are blocked in the final movement envelope.
- Stealth with right-hand 8 N / 10° completed 2/3 trials; the unsuccessful attempt fell on approach after 9.1 s. This load/slope combination is now blocked for every selected style.
- Three earlier timed baseline attempts had two falls and one timed completion. Their checker did not enforce course confinement, so that timed completion is not included in the full-crossing totals.

[summary.json](summary.json) contains all summarized attempts. [raw/](raw/) retains every raw trace compressed as JSON, including failures and the separate reference diagnostic. Some early raw traces recorded negative duration after a respawn reset simulation time; the summary correctly uses the fall event timestamp minus the start timestamp.

## Checks and reproduction

All 23 Python simulator tests passed. Browser reference, steering-lock, contact-equation, robot-slot, sphere-burst, fall-guard and respawn-surface checks passed. The real browser control check verified gait labeling, stopping a held key, hover explanation, unchanged 8 N request, reconnect while blocked, and clear/slope-off restoration.

Run `tests/audit_downhill_exertion.cjs` with `BRACE_SLOPE_CASES_FILE=reports/downhill-exertion-2026-10-08/final-cases.json`, `BRACE_SLOPE_OUTPUT`, and a `BRACE_TEST_URL` pointing to a dedicated generator. Run sequentially to avoid competing for generator compute. ONNX sessions have independent state. `models.json` records the two ONNX hashes; `python-tests.log` and `browser-controls.log` retain contract-check results.

Finite trials do not guarantee stability for every terrain transition, custom force, timing, gait or impact. Higher loads were restricted by movement controls rather than reducing the force dial.


## Published-page verification

GitHub Pages deployment [37792517668](https://github.com/multiplylabs/robogym-online/actions/runs/37792517668) passed all build and simulator checks for commit `bac236a`. The published JavaScript was verified to contain the movement guard.

The actual [public page](https://multiplylabs.github.io/robogym-online/) and generator tunnel completed two additional full-course trials: right-hand 5 N at 10° (24.56 simulation seconds) and right-hand 8 N at 8° (25.18 s), including three seconds observed after releasing W. No falls or respawns occurred, and force requests remained unchanged. The observation includes deceleration/settling, rather than enforcing zero robot velocity.

Two further public blocked-input trials passed 20.1 seconds each: right-hand 8 N at 10°, and both hands at 8 N each on 6°. All movement packets stayed zero; force requests remained unchanged; XY drift stayed below 4 cm. The public browser control test also passed gait labeling, unsafe held-key stop, hover explanation, reconnect while blocked, and restoration after clearing force or disabling the slope.

`raw/public.json.gz`, `public-controls.log`, `deployment.json` and the `public-*.png` screenshots retain these checks. Public results are separate from the 15 local permitted crossings and three local blocked-input checks.
