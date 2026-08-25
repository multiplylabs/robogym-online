# SPDX-License-Identifier: Apache-2.0
"""A reference clip that is still being written -- ARDY generating ahead of a live WASD command.

:mod:`ardy_reference` plans a whole path up front, which is fine for a canned clip and useless for
a keyboard: an operator's command is not known in advance. This is the same chain run as a rolling
horizon instead. Each cycle plans the next stretch of path from *whatever the command is right now*,
generates it conditioned on the motion already committed, and appends the result:

    command -> path segment -> ARDY (continuing from history) -> qpos -> FK -> reference frames

The consumer (a MuJoCo runner, or a browser over a socket) reads frames by absolute index and never
sees a seam. That continuity is the reason to generate rather than to select from a clip library:
the motion stays in one world frame, so changing direction mid-stride needs no re-anchoring.

**Latency and lookahead.** ARDY generates a 52-frame horizon (2.1 s at 25 Hz) but only the first
:data:`COMMIT_FRAMES` are kept, and the rest is re-planned against the newer command -- the usual
receding-horizon trade: commit less, steer sooner. The policy observes 20 control steps (0.4 s) of
future reference, so the buffer must stay at least that far ahead of the robot; committing ~1 s at a
time keeps roughly double that in hand, and a cycle costs ~40 ms on an RTX 5090.

**Where world coordinates come from.** Each cycle decodes the window it just generated -- history
frames included -- and keeps the world poses of the new frames; the accumulated latent tensor is
only ever fed back as history, never decoded from frame zero. That ordering is not cosmetic. The
model canonicalizes the history it is given (translation and heading zeroed at the join), so each
window's latents mean "relative to that window", and integrating the concatenation from the start
mixes frames together: the error appears as the reference climbing steadily into the air, about a
quarter-metre per cycle, while still walking a plausible-looking path. ``overlap_error`` measures
the invariant that has to hold instead -- that a fresh decode reproduces the world poses already
handed out, over the frames the two windows share.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .ardy_reference import ARDY_FPS, WAYPOINT_INTERVAL, _central_diff, _nlerp, _quat_ang_vel

# Frames kept from each generated horizon, at :data:`ardy_reference.ARDY_FPS`. Smaller reacts to the
# keyboard sooner and generates more often; 24 frames is ~1 s, about double the policy's lookahead.
COMMIT_FRAMES = 24
# Longest history the model attends over, in frames -- ARDY's trained window minus its horizon.
HISTORY_MAX = 196
# Seconds for a command change to take full effect, as the interactive demo ramps it. A stepped
# command asks the reference to reverse within one frame, which reads as a stumble the policy then
# has to track.
COMMAND_RAMP_S = 2.0
# Fraction of the reported tracking error folded into each new plan.
#
# This is the gain of a loop whose dead time is however deeply the consumer buffers, so it has to
# suit the slowest consumer rather than the fastest. At 0.7 the local runner is stable and settles
# at 0.3 m, while the browser -- buffering two to three times as much -- oscillates instead, and the
# reference's alternating slow-down and catch-up is enough to topple the robot. Low and sluggish
# beats fast and marginal: the point is only to stop the reference drifting away for ever.
LAG_CORRECTION = 0.25
# How much the applied correction may change per cycle, in metres. A transient -- the hand-over from
# a bundled clip, a stumble -- reports several metres at once, and applying that in one step pulls
# the plan origin behind the frame the motion continues from: the reference stalls and a robot that
# was mid-stride goes down. Slewing reaches the same steady state without ever yanking the path. A
# fixed ceiling instead is the wrong shape -- small enough to be safe in a transient is too small to
# correct the steady drift, and the reference draws away again.
LAG_SLEW_M = 0.12
# Frames generated for the opening window, through the model's own call path. This has to span more
# than one generation horizon: at exactly one horizon the model runs a single window seeded from
# nothing, which drifts the same way a bare autoregressive step does (measured: 0.5 m of climb over
# 9 s). Two horizons' worth means the opening motion has itself been conditioned on real history.
# The cost is a fixed startup latency -- these frames are generated under whatever command is set
# when the stream starts, so the robot stands for about three seconds before the keyboard bites.
PRIME_FRAMES = 76


class RootHeightConstraint:
    """Pin the pelvis height at given frames -- ARDY's ``root_y_pos`` channel.

    Root height is the one part of the root the WASD command says nothing about, and leaving it
    unconstrained is what lets the reference wander vertically: asked to start walking from a
    standing history, the model answers with a root that sinks towards the floor (measured: 0.31 m)
    while the gait and the path still look correct. Pinning it removes that freedom.

    Applied at the same sparse indices as the path waypoints, deliberately: a value on every frame
    would flatten the few centimetres of pelvis bob a real gait has, whereas one every few frames
    leaves the model room to place the bob and only removes the slow drift. ARDY's own full-body
    constraint sets populate this same channel.
    """

    name = "root_y_pos"

    def __init__(self, frame_indices, height: float) -> None:
        import torch

        self.frame_indices = torch.as_tensor(frame_indices, dtype=torch.long)
        self.root_y_pos = torch.full((len(self.frame_indices),), float(height), dtype=torch.float32)

    def update_constraints(self, data_dict: dict, index_dict: dict) -> None:
        data_dict["root_y_pos"].append(self.root_y_pos)
        index_dict["root_y_pos"].append(self.frame_indices)


class ReferenceStream:
    """Reference frames at the control rate, generated on demand from a live velocity command."""

    def __init__(self, contract: dict, mjcf: Path, seed: int | None = None) -> None:
        import mujoco
        import torch
        from ardy.exports.mujoco import MujocoQposConverter
        from ardy.model import load_model

        from .scene import build_spec

        if seed is not None:
            torch.manual_seed(seed)
        self._torch = torch
        self._device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self._model = load_model("g1", device=self._device)
        self._converter = MujocoQposConverter(self._model.skeleton)
        self._horizon = int(self._model.gen_horizon_len)
        self._patch = int(self._model.num_frames_per_token)

        self.control_dt = float(contract["timing"]["control_dt"])
        self._body_names = list(contract["body_names"])
        self._mj = mujoco
        self._mj_model = build_spec(mjcf, float(contract["timing"]["physics_dt"])).compile()
        self._mj_data = mujoco.MjData(self._mj_model)
        self._qpos_adr = np.array(
            [
                self._mj_model.jnt_qposadr[i]
                for i in range(self._mj_model.njnt)
                if self._mj_model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
            ]
        )

        # The operator's command, in the robot's own frame: m/s, m/s, deg/s.
        self._command = (0.0, 0.0, 0.0)
        # Ramped velocity, carried across cycles so a command change eases in rather than steps.
        self._velocity = (0.0, 0.0, 0.0)  # forward, lateral, turn
        # Tracking error the consumer reports, in world metres: reference minus robot. See
        # `set_lag`.
        self._lag = (0.0, 0.0)
        self._applied_lag = (0.0, 0.0)

        # Pelvis height the reference is held at, measured from the opening generation so it is the
        # model's own idea of standing height rather than a number from the robot's spec sheet.
        self._height: float | None = None
        self._motion = None  # accumulated normalized motion tensor [1, T, D]
        self._qpos = np.zeros((0, 7 + len(contract["joint_names"])))  # decoded, at ARDY_FPS
        self._reference: dict[str, np.ndarray] | None = None  # decoded frames, at the control rate

    # -- command ----------------------------------------------------------------

    def set_command(self, forward: float, lateral: float, turn_deg: float) -> None:
        """Set the velocity command. Takes effect from the next generated horizon."""
        self._command = (float(forward), float(lateral), float(turn_deg))

    @property
    def command(self) -> tuple[float, float, float]:
        return self._command

    def set_lag(self, dx: float, dy: float) -> None:
        """Report the tracking error, reference minus robot, in world metres.

        The generator never sees the simulation, so without this it is open-loop with respect to the
        robot -- and a kinematic reference is not something a physical gait matches exactly. Foot
        slip alone loses about 0.09 m/s here, which is invisible over a few seconds and 3.5 m after
        forty, at which point the operator is steering a reference their robot is nowhere near.

        Feeding it back as a *speed limit* does not work, and the reason is worth recording: this is
        a mimic policy, so a stationary reference means a stationary robot. Throttling the command
        until the robot catches up instead parks both of them, permanently, a metre apart. The
        correction has to move the path, not the pace.

        How well this settles depends on how deeply the consumer buffers, because the buffer is
        dead time in this loop: measured at ~0.4 s of buffer the error holds around 0.45 m, and at
        ~1 s it holds around 0.8 m and wanders more.
        """
        self._lag = (float(dx), float(dy))

    # -- generation -------------------------------------------------------------

    def _anchor(self) -> tuple[float, float, float]:
        """Pose of the last committed reference frame: ``(x, y, yaw)`` in MuJoCo world.

        Every plan starts here rather than from where the previous plan ended. Integrating a plan
        onward from the *previous plan* instead makes the waypoints drift away from the motion: the
        generated gait need not exactly achieve the commanded speed, and nothing ever corrects the
        shortfall, so the targets creep further ahead each cycle. ARDY answers waypoints it cannot
        reach by leaving the ground -- the reference climbs a quarter-metre a second while still
        looking like a walk -- so this closes the loop that keeps the command reachable.
        """
        if self._qpos.shape[0] == 0:
            return (0.0, 0.0, 0.0)
        x, y = self._qpos[-1, 0:2]
        w, qx, qy, qz = self._qpos[-1, 3:7]
        yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return (float(x), float(y), float(yaw))

    def _plan(self, count: int, commit: int | None = None) -> tuple[np.ndarray, np.ndarray, tuple]:
        """Integrate ``count`` frames of path forward, ending up ahead of the *robot*.

        Returns the path, the headings, and the ramped velocity after ``commit`` emitted frames
        (default: all of them). Only that many frames are kept from a horizon, so the ramp has to
        resume from there next cycle -- the rest is re-planned against whatever the command is by
        then.

        The path starts at the last generated frame, pulled back by the robot's reported tracking
        error. It has to start there: the generated motion continues from the history, which *is*
        that frame, so a path laid out from the robot's own position instead asks the model to step
        backwards onto it -- which reads as a stumble and, measured over a minute of walking, falls
        over. Pulling the origin back leaves the trajectory continuous and merely asks for a shorter
        stride, which is a reference the robot can hold on to while it recovers the lost ground.
        """
        x, y, yaw = self._anchor()
        dx, dy = self._applied_lag
        x -= dx
        y -= dy
        vf, vl, vw = self._velocity
        target_f, target_l, target_w = self._command
        dt = 1.0 / ARDY_FPS
        blend = min(1.0, dt / COMMAND_RAMP_S)
        path, headings = np.empty((count, 2)), np.empty(count)
        # The first `skip` steps carry the walk across the buffer without being emitted; only the
        # tail lands on frames that have yet to be generated.
        emitted_commit = count if commit is None else min(commit, count)
        commit_velocity = (vf, vl, vw)
        for i in range(count):
            vf += (target_f - vf) * blend
            vl += (target_l - vl) * blend
            vw += (target_w - vw) * blend
            yaw += math.radians(vw) * dt
            x += (vf * math.cos(yaw) - vl * math.sin(yaw)) * dt
            y += (vf * math.sin(yaw) + vl * math.cos(yaw)) * dt
            path[i] = (x, y)
            headings[i] = yaw
            if i + 1 == emitted_commit:
                commit_velocity = (vf, vl, vw)
        return path, headings, commit_velocity

    def _constraints(self, path: np.ndarray, headings: np.ndarray, offset: int):
        """Waypoints for a planned path: a ``root2d`` set, plus the height pin, at ``offset + i``."""
        from ardy.constraints import load_constraints_lst

        indices = list(range(WAYPOINT_INTERVAL, len(path), WAYPOINT_INTERVAL))
        constraints = load_constraints_lst(
            [
                {
                    "type": "root2d",
                    "frame_indices": [offset + i for i in indices],
                    # MuJoCo (x, y) -> ARDY (x, z); see `ardy_reference`.
                    "root_2d": [[float(path[i][1]), float(path[i][0])] for i in indices],
                    "global_root_heading": [float(headings[i]) for i in indices],
                }
            ],
            self._model.skeleton,
        )
        if self._height is not None:
            constraints.append(RootHeightConstraint([offset + i for i in indices], self._height))
        return constraints

    def _conditions(self, constraints, num_frames: int, history: int):
        """``(observed_motion, motion_mask)`` for a window, with the history frames left free."""
        torch = self._torch
        lengths = torch.tensor([num_frames], device=self._device)
        observed, mask = self._model.motion_rep.create_conditions_from_constraints_batched(
            constraints, lengths, to_normalize=True, device=self._device
        )
        if history:
            # The history is supplied as motion, not as constraints; a mask left on those frames
            # would ask the model to re-satisfy waypoints it has already walked past.
            mask[:, :history] = 0.0
            observed[:, :history] = 0.0
        return observed, mask

    def _prime(self) -> None:
        """Start the stream with a one-shot generation rather than a bare autoregressive step.

        ``autoregressive_step`` needs a history to continue from, and seeding it with zeros instead
        produces a first chunk that *looks* fine but is a poor thing to condition on: every
        subsequent window inherits it, and the reference drifts vertically for as long as the
        command asks for motion -- climbing when walking fast, sinking when walking slowly. Going
        through the model's own call path for the first window costs one extra horizon of
        generation and leaves the stream conditioning on motion the model itself considers
        in-distribution.
        """
        torch = self._torch
        from ardy.motion_rep.tools import length_to_mask

        num_frames = PRIME_FRAMES
        path, headings, self._velocity = self._plan(num_frames)
        observed, mask = self._conditions(self._constraints(path, headings, 0), num_frames, 0)
        lengths = torch.tensor([num_frames], device=self._device)
        with torch.no_grad():
            samples = self._model(
                [""],
                num_frames,
                num_denoising_steps=10,
                pad_mask=length_to_mask(lengths),
                first_heading_angle=torch.zeros(1, device=self._device),
                motion_mask=mask,
                observed_motion=observed,
                cfg_weight=[2.0, 2.0],
                crop_history_length=HISTORY_MAX,
            )
        self._motion = samples
        decoded = self._model.motion_rep.inverse(samples, is_normalized=True)
        self._last_decode = (0, decoded)
        self._qpos = self._window_qpos(decoded, 0, num_frames)
        # ARDY's y is MuJoCo's z numerically, so the qpos height is the value to pin.
        self._height = float(np.median(self._qpos[:, 2]))
        self._rebuild_reference()

    def _slew_lag(self) -> None:
        """Move the applied correction a step towards the reported one. See :data:`LAG_SLEW_M`."""
        target = (LAG_CORRECTION * self._lag[0], LAG_CORRECTION * self._lag[1])
        step = (target[0] - self._applied_lag[0], target[1] - self._applied_lag[1])
        distance = math.hypot(*step)
        if distance > LAG_SLEW_M:
            step = (step[0] * LAG_SLEW_M / distance, step[1] * LAG_SLEW_M / distance)
        self._applied_lag = (self._applied_lag[0] + step[0], self._applied_lag[1] + step[1])

    def _generate_chunk(self) -> None:
        """Plan, generate one horizon, and commit its first :data:`COMMIT_FRAMES` frames."""
        torch = self._torch

        self._slew_lag()
        if self._motion is None:
            self._prime()
            return

        history = min(HISTORY_MAX, self._motion.shape[1])
        history -= history % self._patch
        num_frames = history + self._horizon

        path, headings, next_velocity = self._plan(self._horizon, COMMIT_FRAMES)
        observed, mask = self._conditions(
            self._constraints(path, headings, history), num_frames, history
        )
        text_feat, text_pad_mask = self._model._encode_text([""])
        with torch.no_grad():
            samples = self._model.autoregressive_step(
                num_frames=num_frames,
                num_denoising_steps=10,
                motion_mask=mask,
                observed_motion=observed,
                cfg_weight=(2.0, 2.0),
                text_feat=text_feat,
                text_pad_mask=text_pad_mask,
                init_history_sequence=self._motion[:, -history:],
                init_global_translation=None,
                init_first_heading_angle=None,
            )
        # Only the velocity carries over; the origin is re-derived from the robot next cycle.
        self._velocity = next_velocity
        fresh = samples[:, history : history + COMMIT_FRAMES]
        self._motion = torch.cat([self._motion, fresh], dim=1)

        # Decode this window, keep the new frames: world poses are only meaningful relative to the
        # history that anchored them, so they are read out here and never re-derived later.
        decoded = self._model.motion_rep.inverse(samples, is_normalized=True)
        self._last_decode = (history, decoded)
        window_qpos = self._window_qpos(decoded, history, history + COMMIT_FRAMES)
        self._qpos = np.concatenate([self._qpos, window_qpos], axis=0)
        self._rebuild_reference()

    def _window_qpos(self, decoded: dict, lo: int, hi: int) -> np.ndarray:
        """MuJoCo qpos for frames ``[lo, hi)`` of a decoded window."""
        sliced = {
            "local_rot_mats": decoded["local_rot_mats"][:, lo:hi],
            "root_positions": decoded["root_positions"][:, lo:hi],
        }
        return np.asarray(self._converter.dict_to_qpos(sliced, self._device), dtype=np.float64)[0]

    # -- reference frames -------------------------------------------------------

    def _rebuild_reference(self) -> None:
        """Resample the decoded qpos to the control rate and run forward kinematics.

        Rebuilt wholesale rather than appended to, so the interpolation and the finite-difference
        velocities at the join are computed from the same neighbours they would have had if the
        whole clip had existed from the start -- an incrementally extended buffer would leave a
        one-frame velocity artefact at every chunk boundary.
        """
        dst_fps = 1.0 / self.control_dt
        n_src = self._qpos.shape[0]
        n_dst = int(np.floor((n_src - 1) / ARDY_FPS * dst_fps)) + 1
        t = np.minimum(np.arange(n_dst) * (ARDY_FPS / dst_fps), n_src - 1)
        lo = np.floor(t).astype(np.int64)
        hi = np.minimum(lo + 1, n_src - 1)
        w = (t - lo)[:, None]

        frames = np.empty((n_dst, self._qpos.shape[1]))
        frames[:, 0:3] = self._qpos[lo, 0:3] * (1.0 - w) + self._qpos[hi, 0:3] * w
        frames[:, 3:7] = _nlerp(self._qpos[lo, 3:7], self._qpos[hi, 3:7], w)
        frames[:, 7:] = self._qpos[lo, 7:] * (1.0 - w) + self._qpos[hi, 7:] * w

        n_bodies = len(self._body_names)
        body_pos = np.empty((n_dst, n_bodies, 3))
        body_quat = np.empty((n_dst, n_bodies, 4))
        for i, frame in enumerate(frames):
            self._mj_data.qpos[0:7] = frame[0:7]
            self._mj_data.qpos[self._qpos_adr] = frame[7:]
            self._mj.mj_kinematics(self._mj_model, self._mj_data)
            body_pos[i] = self._mj_data.xpos[1 : n_bodies + 1]
            body_quat[i] = self._mj_data.xquat[1 : n_bodies + 1]

        joint_pos = frames[:, 7:]
        self._reference = {
            "joint_pos": joint_pos,
            "joint_vel": _central_diff(joint_pos, self.control_dt),
            "body_pos_w": body_pos,
            "body_quat_w": body_quat,
            "body_lin_vel_w": _central_diff(body_pos, self.control_dt),
            "body_ang_vel_w": np.stack(
                [_quat_ang_vel(body_quat[:, b], self.control_dt) for b in range(n_bodies)], axis=1
            ),
        }

    @property
    def available(self) -> int:
        """Number of reference frames decoded so far, at the control rate."""
        return 0 if self._reference is None else self._reference["joint_pos"].shape[0]

    def ensure(self, index: int) -> None:
        """Generate until frame ``index`` exists. Cheap when the buffer is already ahead."""
        while self.available <= index:
            self._generate_chunk()

    def frames(self, indices) -> dict[str, np.ndarray]:
        """Reference fields at ``indices`` (an int or a sequence), generating as needed."""
        idx = np.atleast_1d(np.asarray(indices, dtype=np.int64))
        self.ensure(int(idx.max()))
        return {key: value[idx] for key, value in self._reference.items()}

    def overlap_error(self) -> float:
        """Largest disagreement, in metres, between the last decode and the frames already emitted.

        The last cycle decoded a window whose leading ``history`` frames were already handed to the
        consumer. If the fresh decode places them somewhere else, then the new frames are in a
        different world frame than the old ones and the reference has a seam -- so this number
        staying near zero is what makes appending windows legitimate.
        """
        history, decoded = getattr(self, "_last_decode", (0, None))
        if not history or decoded is None:
            return 0.0
        emitted = self._qpos.shape[0] - COMMIT_FRAMES  # frames known before this cycle
        overlap = min(history, emitted)
        redecoded = self._window_qpos(decoded, history - overlap, history)
        return float(np.abs(redecoded[:, 0:3] - self._qpos[emitted - overlap : emitted, 0:3]).max())
