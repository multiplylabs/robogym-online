# SPDX-License-Identifier: Apache-2.0
"""Observation terms feeding the ``g1_force_v3`` family's ONNX in a browser.

The exported ``unified_pipeline.onnx`` already contains the observation *assembly* --
``export_tracker_onnx.py`` traces the ProtoMotions obs builders into the graph -- so its 25 inputs
are raw context fields (``current.dof_pos``, ``mimic.future_pos``, ...) rather than a flattened
normalized vector. Nothing here reproduces the policy's observation math; each term serves one
context field in the frame, order and units the graph expects. That is the whole reason a browser
port is tractable.

Two conventions cross a boundary in every term that touches a rotation:

* **Quaternion order.** MuJoCo and mjlab are wxyz; the ONNX contract is xyzw (ProtoMotions is
  w-last). :func:`_to_xyzw` is the only place that flips, and every rotation-bearing term goes
  through it.
* **Reference window.** mjswan's ``TrackingCommand`` samples the clip at the offsets in its
  ``time_steps``, which we set to the contract's ``motion.future_step_indices``
  (``[1, 2, 3, 4, 8, 12, 16, 20]`` control steps). The deploy runner treats the *nearest* future
  frame as the "current" reference -- ``mimic.ref_state.*`` is index 0 of the same window, not a
  separate read (``force_track_policy.py``: ``_ref_pos_now = _xpriv_body_pos[0]``) -- so this
  module slices both out of one window.

**Both modes.** The four ``hand_force.x_priv_*`` inputs carry the force-braced reference, which comes
from a per-step whole-body IK. Rather than special-casing compensation, they are read from the brace
graph in both modes: at ``F_cmd = 0`` the construction degenerates to ``x_priv == x_ref`` and the
graph's gate emits exactly what the runner does there -- zero joint/anchor deltas, an identity
rotation delta, and the *reference* body pose (not zeros) for the Cartesian keypoint channel
(``force_track_policy.py:1232-1235``).
"""

from __future__ import annotations

import torch
from mjlab.managers.scene_entity_config import SceneEntityCfg

# The policy entity, as the build names it.
ROBOT = SceneEntityCfg(name="robot")

# Mode one-hot, ``[exert, comp]`` (``tasks/force_dual/control.py``: MODE_EXERT = 0, MODE_COMP = 1).
MODE_EXERT = (1.0, 0.0)
MODE_COMP = (0.0, 1.0)


def _to_xyzw(quat: torch.Tensor) -> torch.Tensor:
    """wxyz (MuJoCo / mjlab) -> xyzw (ONNX contract), on the last axis."""
    return torch.cat([quat[..., 1:4], quat[..., 0:1]], dim=-1)


def _ref_body_pos(env, command_name: str) -> torch.Tensor:
    """Reference body positions over the window, ``[B, steps, bodies, 3]`` (world)."""
    return env.command_manager.get_term(command_name).ref_body_pos_w


def _ref_body_quat(env, command_name: str) -> torch.Tensor:
    """Reference body rotations over the window, ``[B, steps, bodies, 4]`` wxyz (world)."""
    return env.command_manager.get_term(command_name).ref_body_quat_w


# ---------------------------------------------------------------------------
# Proprioception. The same four terms serve `current.*` and, with the group's
# history_length, `historical.*` -- the runtime owns the ring buffer and stacks
# newest-first, which is the runner's convention (`np.roll(..., 1); [0] = now`).
# ---------------------------------------------------------------------------


def dof_pos(env, *, asset_cfg: SceneEntityCfg = ROBOT, **_):
    """Measured joint positions, in the contract's joint order."""
    return env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids]


def dof_vel(env, *, asset_cfg: SceneEntityCfg = ROBOT, **_):
    """Measured joint velocities, in the contract's joint order."""
    return env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids]


def anchor_rot(env, *, asset_cfg: SceneEntityCfg = ROBOT, **_):
    """World rotation of the anchor body (``torso_link``), xyzw.

    ``asset_cfg.body_ids`` is resolved at trace time, so the graph slices the anchor out of
    the entity's per-body field rather than the runtime serving a one-body slot.
    """
    quat = env.scene[asset_cfg.name].data.body_link_quat_w[:, asset_cfg.body_ids]
    return _to_xyzw(quat).flatten(1)


def root_local_ang_vel(env, *, asset_cfg: SceneEntityCfg = ROBOT, **_):
    """Root angular velocity in the root's own frame -- the runner's ``base_ang_vel``."""
    return env.scene[asset_cfg.name].data.root_link_ang_vel_b


# ---------------------------------------------------------------------------
# Mimic goal: the reference clip's look-ahead window.
# ---------------------------------------------------------------------------


def future_dof_pos(env, *, command_name: str = "motion", **_):
    """Reference joint positions at each future offset, ``[B, steps, dofs]``."""
    return env.command_manager.get_term(command_name).ref_joint_pos.flatten(1)


def future_dof_vel(env, *, command_name: str = "motion", **_):
    """Reference joint velocities at each future offset, ``[B, steps, dofs]``."""
    return env.command_manager.get_term(command_name).ref_joint_vel.flatten(1)


def future_pos(env, *, command_name: str = "motion", **_):
    """Reference body positions over the window, ``[B, steps, bodies, 3]``."""
    return _ref_body_pos(env, command_name).flatten(1)


def future_rot(env, *, command_name: str = "motion", **_):
    """Reference body rotations over the window, ``[B, steps, bodies, 4]`` xyzw."""
    return _to_xyzw(_ref_body_quat(env, command_name)).flatten(1)


def future_anchor_pos(env, *, command_name: str = "motion", anchor_index: int = 16, **_):
    """Reference anchor position at each future offset, ``[B, steps, 3]``.

    Only the height enters the policy's observation, but the contract takes the full vector.
    """
    return _ref_body_pos(env, command_name)[:, :, anchor_index].flatten(1)


def future_anchor_rot(env, *, command_name: str = "motion", anchor_index: int = 16, **_):
    """Reference anchor rotation at each future offset, ``[B, steps, 4]`` xyzw.

    The runner heading-aligns this one channel to the robot, because it is compared against the
    measured anchor rotation in the world frame. Here the reset places the robot *on* the
    reference frame, so that alignment is the identity and the clip's own rotation is correct --
    the same situation as a recorded-clip deploy, which aligns once at start and never again.
    """
    return _to_xyzw(_ref_body_quat(env, command_name)[:, :, anchor_index]).flatten(1)


def ref_state_body_pos(env, *, command_name: str = "motion", **_):
    """Current reference body positions, ``[B, bodies, 3]`` -- window index 0."""
    return _ref_body_pos(env, command_name)[:, 0].flatten(1)


def ref_state_body_rot(env, *, command_name: str = "motion", **_):
    """Current reference body rotations, ``[B, bodies, 4]`` xyzw -- window index 0."""
    return _to_xyzw(_ref_body_quat(env, command_name)[:, 0]).flatten(1)


# ---------------------------------------------------------------------------
# Force-braced goal (x_priv). Compensation-mode values: the degenerate case of the
# exertion construction, which is exact rather than an approximation.
# ---------------------------------------------------------------------------


def x_priv_bodies(env, *, command_name: str = "brace", **_):
    """Braced body positions, world.

    The keypoint observation computes ``local(x_priv) - local(x_ref)``, so the neutral value is
    the reference pose, not zeros -- feeding zeros would inject a whole-body displacement into
    every horizon step of the goal. The graph returns exactly that when the command is zero.
    """
    return _brace(env, command_name).x_priv_bodies


def x_priv_rot(env, *, command_name: str = "brace", **_):
    """Braced body rotations, xyzw."""
    return _brace(env, command_name).x_priv_rot


# ---------------------------------------------------------------------------
# Brace outputs. In EXERT the x_priv goal comes from the brace graph -- the deploy-side
# whole-body IK, exported by `brace_export.py` -- which the runtime steps as a command term
# and serves to these reads. In COMP the graph's own gate returns the reference pose and zero
# deltas, so the same wiring covers both modes and there is no second code path.
# ---------------------------------------------------------------------------


def _brace(env, command_name: str):
    return env.command_manager.get_term(command_name)


def dof_priv_delta(env, *, command_name: str = "brace", **_):
    """Joint-space brace lead, ONNX joint order. Carries the series-compliance 2x on arm rows."""
    return _brace(env, command_name).dof_priv_delta


def xpriv_anchor_pos_delta(env, *, command_name: str = "brace", **_):
    """Torso brace shift."""
    return _brace(env, command_name).xpriv_anchor_pos_delta


def xpriv_anchor_rot_delta(env, *, command_name: str = "brace", **_):
    """Torso brace bend, xyzw.

    Identity, not the null quaternion, when there is no brace: the observation builder
    right-multiplies the reference anchor rotation by this, so zeros would annihilate the whole
    torso-orientation goal instead of leaving it alone. The graph's gate emits the identity.
    """
    return _brace(env, command_name).xpriv_anchor_rot_delta


# ---------------------------------------------------------------------------
# Task mode + the flow prior's seed.
# ---------------------------------------------------------------------------


def mode_onehot(env, *, command_name: str = "brace", **_):
    """Which regime is active, one-hot ``[exert, comp]``.

    From the brace graph rather than the operator's checkbox directly: training cross-fades the
    command to zero before flipping the flag, so the flag the policy sees is the *post-fade* one
    and has to come from whatever owns the fade.
    """
    return _brace(env, command_name).mode_onehot


def force_cmd_eff(env, *, command_name: str = "brace", **_):
    """Effective (post-cap) per-hand force command, torso-yaw frame.

    The *effective* value -- after the reach, torque-cone and balance caps -- not the raw dial,
    which is what training put in this channel. Taking it from the brace means the goal and this
    observation cannot disagree about what force was actually asked for.
    """
    return _brace(env, command_name).force_cmd_eff


def initial_noise(env, *, num_dofs: int = 29, **_):
    """The flow prior's base-noise seed.

    Held fixed within an episode by construction. Zero is a legal, deterministic seed and the
    one a demo wants -- the same clip replays identically -- where training drew N(0, I) per
    episode.
    """
    del env
    return torch.zeros(1, num_dofs)


# ---------------------------------------------------------------------------
# Compensation-mode constants. What the brace graph emits at zero command, as plain values,
# for a build with exert mode off (`build_app.py --exert` turns the graph on).
# ---------------------------------------------------------------------------


def dof_priv_delta_const(env, *, num_dofs: int = 29, **_):
    """Joint-space brace lead: zero without a brace."""
    del env
    return torch.zeros(1, num_dofs)


def xpriv_anchor_pos_delta_const(env, **_):
    """Torso brace shift: zero without a brace."""
    del env
    return torch.zeros(1, 3)


def xpriv_anchor_rot_delta_const(env, **_):
    """Torso brace bend: the IDENTITY, not the null quaternion.

    The goal builder right-multiplies the reference anchor rotation by this, so zeros would
    annihilate the whole torso-orientation block instead of leaving it alone.
    """
    del env
    return torch.tensor([[0.0, 0.0, 0.0, 1.0]])


def mode_onehot_const(env, *, mode: tuple[float, float] = MODE_COMP, **_):
    """A pinned regime flag, one-hot ``[exert, comp]``."""
    del env
    return torch.tensor([list(mode)])


def force_cmd_eff_const(env, *, num_hands: int = 2, **_):
    """No exertion command."""
    del env
    return torch.zeros(1, num_hands * 3)
