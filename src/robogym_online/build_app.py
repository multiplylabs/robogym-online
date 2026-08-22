# SPDX-License-Identifier: Apache-2.0
"""Build the browser app for the ``g1_force_v3`` force-control policy family (mjswan).

Packages a G1 MuJoCo model, a reference clip, and one of the family's exported
``unified_pipeline.onnx`` checkpoints into a static site that runs the policy client-side
(MuJoCo-WASM + onnxruntime-web). This is the browser counterpart of the desktop run

    pixi run -e sim2sim robojudo-force-track --onnx-path <compiled_models> --motion-file <clip.pt>

and it is driven off the same contract file: everything shape- or order-bearing (joint and body
order, the future-step window, PD gains, control rate) is read from ``unified_pipeline.yaml``
rather than restated, so pointing ``--onnx-dir`` at a different checkpoint in the family is the
whole configuration.

**Compensation mode.** The mode flag is pinned to COMP and the force dial to zero, which makes the
four ``hand_force.x_priv_*`` inputs exactly the reference pose (see ``terms.py``). The load the
policy rejects is the viewer's own: mjswan writes mouse drags into ``xfrc_applied``. Exertion needs
the deploy-side whole-body-IK brace to run in the browser, which is not this build.

Usage::

    python build_app.py --onnx-dir /home/seahorse/Checkpoints/g1_force_v3/student/student_v2/compiled_models
    python build_app.py --serve          # build and serve on localhost
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnx
import yaml

import mjswan
from mjswan.envs.mdp.actions import JointPositionActionCfg
from mjswan.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjswan.trace_env import build_single_entity_trace_env

sys.path.insert(0, str(Path(__file__).resolve().parent))
import terms  # noqa: E402
from convert_motion import convert  # noqa: E402

HERE = Path(__file__).resolve().parent

# This repo carries no checkpoint, clip or robot model: they are large, and the policy is not ours
# to redistribute. Point these at wherever they were materialized (CI fetches them; see the
# workflow), or pass the flags.
DEFAULT_ONNX_DIR = Path(os.environ.get("ROBOGYM_ONNX_DIR", "assets/compiled_models"))
DEFAULT_MOTION = Path(os.environ.get("ROBOGYM_MOTION", "assets/motion.npz"))
DEFAULT_MJCF = Path(os.environ.get("ROBOGYM_MJCF", "assets/mjcf/g1_holo_compat.xml"))

# Proprioception history depth, from the contract's `historical.*` inputs.
_PROPRIO_HISTORY = 8

# Half-extent of the ground plane, in metres. See `_add_scene_visuals`.
_FLOOR_HALF_SIZE = 60.0


# ---------------------------------------------------------------------------
# Contract
# ---------------------------------------------------------------------------


def load_contract(onnx_dir: Path) -> dict:
    """The exported deployment contract (``unified_pipeline.yaml``)."""
    with (onnx_dir / "unified_pipeline.yaml").open() as f:
        return yaml.safe_load(f)


def onnx_input_shapes(model: onnx.ModelProto) -> dict[str, list[int]]:
    """Each graph input's declared shape, batch pinned to 1.

    The browser's observation groups produce flat buffers; these are what the runtime
    reinterprets them as. Read off the model so a re-export cannot silently disagree.
    """
    shapes: dict[str, list[int]] = {}
    for value in model.graph.input:
        dims = [d.dim_value if d.dim_value > 0 else 1 for d in value.type.tensor_type.shape.dim]
        shapes[value.name] = dims
    return shapes


def check_contract(model: onnx.ModelProto, contract: dict, mj_model: mujoco.MjModel) -> None:
    """Fail the build on any disagreement between the ONNX, the contract and the model.

    Every mismatch this catches is one that would otherwise produce a robot that *almost*
    tracks -- a reordered body, a joint permutation -- rather than an error.
    """
    graph_inputs = [v.name for v in model.graph.input]
    declared = list(contract["_runtime"]["onnx_in_names"])
    if graph_inputs != declared:
        raise ValueError(
            f"ONNX inputs disagree with the contract.\n  graph: {graph_inputs}\n  yaml:  {declared}"
        )

    body_names = list(contract["body_names"])
    joint_names = list(contract["joint_names"])
    model_bodies = [mj_model.body(i).name for i in range(1, mj_model.nbody)]
    model_joints = [
        mj_model.joint(i).name
        for i in range(mj_model.njnt)
        if mj_model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
    ]
    if model_bodies != body_names:
        raise ValueError(
            "MJCF body order does not match the contract's `body_names`.\n"
            f"  model: {model_bodies}\n  yaml:  {body_names}"
        )
    if model_joints != joint_names:
        raise ValueError(
            "MJCF joint order does not match the contract's `joint_names`.\n"
            f"  model: {model_joints}\n  yaml:  {joint_names}"
        )
    if mj_model.nu != len(joint_names):
        raise ValueError(f"model has {mj_model.nu} actuators for {len(joint_names)} joints")


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------


def build_spec(mjcf: Path, physics_dt: float) -> mujoco.MjSpec:
    """The robot MJCF plus the environment it needs to stand in and be looked at.

    The training MJCF is scene-less -- it declares foot/floor contact pairs but no ``floor``
    geom, which the harness injects -- so a bare compile fails. The floor, lighting and visual
    settings are added through ``MjSpec`` rather than a wrapper XML so the mesh directory keeps
    resolving from the original file's location.
    """
    spec = mujoco.MjSpec.from_file(str(mjcf))
    _add_scene_visuals(spec)
    # The contract's physics rate, not the MJCF's: `decimation = control_dt / physics_dt` has to
    # come out at the value the policy was trained with.
    spec.option.timestep = physics_dt
    return spec


def _add_scene_visuals(spec: mujoco.MjSpec) -> None:
    """Floor, lights and visual settings, following MuJoCo's own scene conventions.

    Two things carry most of the look: a *textured* floor, which gives the eye the ground plane
    and the sense of forward motion a flat colour cannot (the clip walks, and the camera tracks
    the torso, so without a texture nothing on screen moves); and floor ``reflectance``, which is
    what mjswan's reflection pass has to work with.
    """
    spec.add_texture(
        name="skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.24, 0.34, 0.44],
        rgb2=[0.04, 0.06, 0.09],
        width=512,
        height=3072,
    )
    spec.add_texture(
        name="groundplane",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        mark=mujoco.mjtMark.mjMARK_EDGE,
        rgb1=[0.34, 0.38, 0.42],
        rgb2=[0.28, 0.32, 0.36],
        markrgb=[0.62, 0.67, 0.72],
        width=300,
        height=300,
    )
    # `texrepeat` here is tiles across the WHOLE plane, not MuJoCo's per-unit-length reading:
    # the renderer applies it as a plain texture repeat on a sized plane and ignores
    # `texuniform`. At 2 checker squares per tile, this puts a square at roughly 0.75 m.
    ground = spec.add_material(
        name="groundplane",
        texrepeat=[_FLOOR_HALF_SIZE * 4.0 / 3.0] * 2,
        texuniform=True,
        reflectance=0.18,
        shininess=0.1,
        specular=0.2,
    )
    ground.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "groundplane"

    # A *sized* plane, not MuJoCo's usual `size="0 0 .05"` infinite one. The renderer draws an
    # infinite plane as a screen-space shader quad, which is not a world-space mesh and so can
    # neither receive the shadow map nor take part in the reflection pass -- a sized plane gets
    # both.
    #
    # The cost is a visible edge, and there is no haze to hide it: the renderer implements no fog,
    # so `visual.rgba.haze` is inert here and the infinite plane's soft horizon came from its own
    # shader. Hence a plane large enough that its edge sits at the horizon line rather than in
    # the scene.
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[_FLOOR_HALF_SIZE, _FLOOR_HALF_SIZE, 0.05],
        material="groundplane",
        condim=3,
    )
    # Key light: the one that casts the shadow seating the robot on the floor.
    #
    # `range` is load-bearing, not decoration. The renderer places a directional light `range`
    # units back along its own direction and gives it a fixed shadow frustum of near=1, far=10,
    # so a light aimed from far away puts the robot *behind* the far plane and it casts no shadow
    # at all. Aiming from a point near the robot, a few units out, keeps it inside that frustum.
    spec.worldbody.add_light(
        pos=[1.2, 1.2, 0.0],
        dir=[-0.35, -0.35, -1.0],
        range=5.0,
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.95, 0.94, 0.90],
        specular=[0.35, 0.35, 0.35],
        castshadow=True,
    )
    # Fill from the opposite side, so the shadowed half of the mesh reads as metal rather than
    # silhouette. No shadow of its own -- two shadows from one body looks like a bug.
    spec.worldbody.add_light(
        pos=[-2.0, -1.5, 0.0],
        dir=[0.5, 0.4, -1.0],
        range=6.0,
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.30, 0.33, 0.38],
        specular=[0.0, 0.0, 0.0],
        castshadow=False,
    )

    spec.visual.headlight.ambient = [0.40, 0.41, 0.44]
    spec.visual.headlight.diffuse = [0.30, 0.30, 0.32]
    spec.visual.headlight.specular = [0.15, 0.15, 0.15]
    spec.visual.quality.shadowsize = 4096
    spec.visual.quality.offsamples = 8
    # Scene scale: MuJoCo derives shadow and haze extents from this, and a humanoid-sized value
    # keeps the shadow map's resolution on the robot instead of spread over the whole plane.
    spec.stat.center = [0.0, 0.0, 0.8]
    spec.stat.extent = 1.6


def observation_groups(contract: dict, exert: bool) -> dict[str, ObservationGroupCfg]:
    """One group per ONNX input, keyed by input name.

    A group is mjswan's unit of ONNX-input plumbing, so a 25-input graph is 25 groups -- most
    holding a single shape-preserving term. Terms that read nothing off the env are baked to
    constants at build time and cost no graph at all.
    """
    anchor = int(contract["robot"]["anchor_body_index"])
    n_dofs = int(contract["robot"]["num_dofs"])
    anchor_cfg = terms.SceneEntityCfg(
        name="robot",
        body_names=(contract["robot"]["anchor_body_name"],),
    )

    def group(key: str, func, params=None, history: int | None = None) -> ObservationGroupCfg:
        # The term is named after the group: a group with per-term history does not fuse, and
        # that path names its traced graph after the TERM -- so a shared name silently has one
        # group's graph overwrite another's, leaving them all reading the last field traced.
        cfg = ObservationGroupCfg(
            terms={key: ObservationTermCfg(func=func, params=params or {})},
        )
        if history is not None:
            cfg.history_length = history
        return cfg

    proprio = {
        "dof_pos": (terms.dof_pos, {}),
        "dof_vel": (terms.dof_vel, {}),
        "anchor_rot": (terms.anchor_rot, {"asset_cfg": anchor_cfg}),
        "root_local_ang_vel": (terms.root_local_ang_vel, {}),
    }

    groups: dict[str, ObservationGroupCfg] = {}
    for name, (func, params) in proprio.items():
        groups[f"current_{name}"] = group(f"current_{name}", func, params)
        # The same term stacked by the runtime: newest-first, primed from the current frame
        # on reset -- which is how the runner seeds its own buffers.
        groups[f"historical_{name}"] = group(
            f"historical_{name}", func, params, history=_PROPRIO_HISTORY
        )

    # The policy's own output history. `prev_action` is the runtime's stored action vector, which
    # for this policy is the absolute joint target (see `out_keys` in build()).
    from mjlab.envs.mdp import observations as mjlab_obs

    groups["historical_processed_actions"] = group(
        "historical_processed_actions", mjlab_obs.last_action, history=_PROPRIO_HISTORY
    )

    mimic = {
        "mimic_future_dof_pos": (terms.future_dof_pos, {}),
        "mimic_future_dof_vel": (terms.future_dof_vel, {}),
        "mimic_future_pos": (terms.future_pos, {}),
        "mimic_future_rot": (terms.future_rot, {}),
        "mimic_future_anchor_pos": (terms.future_anchor_pos, {"anchor_index": anchor}),
        "mimic_future_anchor_rot": (terms.future_anchor_rot, {"anchor_index": anchor}),
        "mimic_ref_state_rigid_body_pos": (terms.ref_state_body_pos, {}),
        "mimic_ref_state_rigid_body_rot": (terms.ref_state_body_rot, {}),
    }
    for name, (func, params) in mimic.items():
        groups[name] = group(name, func, params)

    if exert:
        groups["hand_force_x_priv_bodies"] = group("hand_force_x_priv_bodies", terms.x_priv_bodies)
        groups["hand_force_x_priv_rot"] = group("hand_force_x_priv_rot", terms.x_priv_rot)
    else:
        groups["hand_force_x_priv_bodies"] = group(
            "hand_force_x_priv_bodies", terms.ref_state_body_pos
        )
        groups["hand_force_x_priv_rot"] = group("hand_force_x_priv_rot", terms.ref_state_body_rot)

    # The brace-fed block. With exert enabled these read the brace graph, which emits the
    # compensation values itself when the command is zero -- one path for both modes. With it
    # disabled they are the compensation constants, which is what the graph would have produced.
    if exert:
        for name, func in (
            ("hand_force_dof_priv_delta", terms.dof_priv_delta),
            ("hand_force_xpriv_anchor_pos_delta", terms.xpriv_anchor_pos_delta),
            ("hand_force_xpriv_anchor_rot_delta", terms.xpriv_anchor_rot_delta),
            ("task_mode_mode_onehot", terms.mode_onehot),
            ("task_mode_force_cmd_eff", terms.force_cmd_eff),
        ):
            groups[name] = group(name, func)
    else:
        groups["hand_force_dof_priv_delta"] = group(
            "hand_force_dof_priv_delta", terms.dof_priv_delta_const, {"num_dofs": n_dofs}
        )
        groups["hand_force_xpriv_anchor_pos_delta"] = group(
            "hand_force_xpriv_anchor_pos_delta", terms.xpriv_anchor_pos_delta_const
        )
        groups["hand_force_xpriv_anchor_rot_delta"] = group(
            "hand_force_xpriv_anchor_rot_delta", terms.xpriv_anchor_rot_delta_const
        )
        groups["task_mode_mode_onehot"] = group(
            "task_mode_mode_onehot", terms.mode_onehot_const, {"mode": terms.MODE_COMP}
        )
        groups["task_mode_force_cmd_eff"] = group(
            "task_mode_force_cmd_eff", terms.force_cmd_eff_const
        )

    groups["initial_noise"] = group("initial_noise", terms.initial_noise, {"num_dofs": n_dofs})
    return groups


# Per-hand cap the compensation teacher was trained against (`tasks/force_comp/control.py`:
# `abs_max_force`). The summed two-hand backstop there is 40 N, so both hands at full deflection
# is deliberately outside the trained envelope -- which is a useful thing to be able to try.
_HAND_FORCE_MAX_N = 25.0
_HAND_FORCE_TOTAL_MAX_N = 40.0

# Metres of arrow per newton. 25 N -> 0.5 m, about a forearm.
_FORCE_ARROW_SCALE = 0.02


def hand_force_command(contract: dict) -> tuple[mjswan.CommandTermConfig, dict]:
    """The disturbance interface: a slider per hand per axis, plus the arrows that show it.

    Returns the command and the ``external_wrench`` block that applies it.

    **Frame.** World, matching training: `tasks/force_comp/control.py` samples a direction on the
    sphere and writes the force straight into `xfrc_applied` in world coordinates, so a
    world-frame dial is exactly what the policy saw. It also means the arrow can be drawn
    directly from the slider values, with no frame conversion between the dial and the picture.

    **What is not reproduced.** Training applied the equivalent wrench `(F, r x F)` for a randomly
    offset application point, adding the wrist moment a real contact produces; this applies pure
    force at the hand origin, i.e. the zero-lever-arm case. The magnitude there also swept a
    triangle wave under a pose-feasible ceiling rather than being held constant, so a slider parked
    at 25 N is a harsher, more sustained load than any single training step.
    """
    hands = [
        ("left", contract["robot"]["body_names"].index("left_rubber_hand"), (0.95, 0.45, 0.15, 0.9)),
        ("right", contract["robot"]["body_names"].index("right_rubber_hand"), (0.25, 0.70, 0.85, 0.9)),
    ]

    inputs: list = []
    targets: list[dict] = []
    viz: list[dict] = []
    for side, body_index, color in hands:
        axes = [f"{side}_f{axis}" for axis in "xyz"]
        for axis, name in zip("XYZ", axes, strict=True):
            inputs.append(
                mjswan.SliderConfig(
                    name=name,
                    label=f"{side.capitalize()} hand {axis} (N)",
                    range=(-_HAND_FORCE_MAX_N, _HAND_FORCE_MAX_N),
                    default=0.0,
                    step=0.5,
                )
            )
        targets.append({"body": contract["robot"]["body_names"][body_index], "axes": axes})
        # The arrow starts at the hand and points along the applied force. `origin` picks that
        # body's three components out of the entity-wide position field; `vector` reads the same
        # slider values the wrench applier does, so the picture cannot disagree with the physics.
        # A zero force gives a zero-length arrow, which the renderer hides -- so no separate
        # on/off control is needed, and a visible arrow always means a real load.
        base = body_index * 3
        viz.append(
            {
                "shape": "arrow",
                "color": list(color),
                "width": 0.012,
                "origin": {
                    "entity": "robot",
                    "field": "body_link_pos_w",
                    "components": [base, base + 1, base + 2],
                },
                "vector": {
                    "state": "command",
                    "components": [len(viz) * 3, len(viz) * 3 + 1, len(viz) * 3 + 2],
                    "scale": _FORCE_ARROW_SCALE,
                },
            }
        )

    command = mjswan.CommandTermConfig(
        term_name="UiCommand",
        ui=mjswan.CommandUiConfig(inputs=inputs),
        viz=viz,
    )
    return command, {"command_name": "hand_force", "targets": targets}


# Exert ceiling from the checkpoint's own hand_force config (`max_force`), read at build time.
_EXERT_MAX_N = 9.0

# Where the brace graph lands inside the scene's asset directory. `graphRefs` discovers any `onnx`
# string under `commands`, so referencing it here is all the runtime needs to fetch it.
_BRACE_REF = "command/brace.onnx"


def exert_dial_command() -> mjswan.CommandTermConfig:
    """The operator's exertion controls: a mode switch and a force vector per hand.

    The vector is in the **torso-yaw frame**, which is how training defines the command
    (``constant_force_xyz``): yaw-invariant, so ``+x`` is the robot's own forward wherever it
    happens to be facing, and the dial keeps its meaning as the clip turns. The same three axes as
    the compensation sliders above, so the two halves of the panel read alike -- the difference is
    that these are a force the robot *produces*, not one applied to it.

    Declaration order IS the vector the brace graph reads (`UiCommand.getCommand` emits sliders and
    checkboxes in order), so this must stay in step with `brace_export.make_dial`:
    ``[exert, fx_L, fy_L, fz_L, fx_R, fy_R, fz_R]``.
    """
    inputs: list = [
        mjswan.CheckboxConfig(
            name="exert",
            label="Exert force (off = compensate)",
            default=False,
        )
    ]
    for side in ("left", "right"):
        for axis in ("x", "y", "z"):
            inputs.append(
                mjswan.SliderConfig(
                    name=f"{side}_f{axis}",
                    label=f"{side.capitalize()} hand exert {axis.upper()} (N)",
                    range=(-_EXERT_MAX_N, _EXERT_MAX_N),
                    default=0.0,
                    step=0.25,
                    enabled_when="exert",
                )
            )
    return mjswan.ui_command(inputs)


def _find_hand_force_cfg(node):
    """The ``hand_force`` control block anywhere in a resolved training config.

    Located by the flag only that block carries rather than by path, because the path differs
    between recipes. Same rule the deploy-side estimator uses.
    """
    if isinstance(node, dict):
        if "publish_full_xpriv_goal" in node:
            return node
        for value in node.values():
            found = _find_hand_force_cfg(value)
            if found is not None:
                return found
    elif isinstance(node, list):
        for value in node:
            found = _find_hand_force_cfg(value)
            if found is not None:
                return found
    return None


def load_hand_force_cfg(onnx_dir: Path) -> dict:
    """The checkpoint's ``hand_force`` block -- the contact constants the browser must match."""
    with (onnx_dir.parent / "resolved_configs_inference.yaml").open() as f:
        cfg = _find_hand_force_cfg(yaml.safe_load(f))
    if cfg is None:
        raise ValueError(f"no hand_force config beside {onnx_dir}")
    return cfg


def hand_spring_config(contract: dict, hf: dict, control_dt: float) -> dict:
    """The virtual contact the exertion policy pushes against.

    Exertion is not a passive arm lead: training anchors a Kelvin-Voigt contact at the *reference*
    hand, and the reaction is what loads the body and makes a force exist at all. Its constants come
    from the checkpoint rather than being restated here, so the browser's contact is the one the
    policy was trained against (`constant_force_via_spring` in the sim inference path).
    """

    def _b(key: str, default: bool = False) -> bool:
        return str(hf.get(key, default)).strip().lower() == "true"

    body_names = list(contract["body_names"])
    hands = ["left_rubber_hand", "right_rubber_hand"]
    return {
        "command_name": "brace",
        "anchor_body": contract["robot"]["anchor_body_name"],
        "targets": [{"body": name, "hand": i} for i, name in enumerate(hands)],
        "max_lead": float(hf.get("max_lead", 0.15)),
        "smooth_beta": float(hf.get("smooth_contact_beta", 120.0)) if _b("smooth_contact") else 0.0,
        "two_sided": _b("two_sided_spring"),
        "damping": _b("contact_damping"),
        "dt": control_dt,
        "gauge": True,
    }


def brace_command(contract: dict, brace_path: Path) -> mjswan.CommandTermConfig:
    """The x_priv brace graph, run as a stateful command term.

    Every graph output is a *state field*: the runtime holds each under its own name, serves it to
    the observation terms as a ``{command, field}`` slot, and feeds the matching ``prev_<name>`` back
    next step. So the cross-step state (the lead filter, the balance EMA, the IK warm start, the mode
    fade) and the published goal ride one mechanism.

    The fields are read **from the graph** rather than listed here. A field the graph emits but the
    config omits is not an error anywhere: the runtime simply never stores it, every consumer reads
    the init value forever, and the feature silently does nothing -- which is exactly how the force
    gauge came to draw nothing at all. Deriving them makes that impossible.

    Rate limiting is the runtime's: ``OnnxCommand.update`` skips while a run is in flight rather than
    queueing, so the brace runs as fast as it can and the last goal is held in between. That is what
    the quasi-static construction licenses -- the brace is broadcast across the whole lookahead
    horizon with reference velocities held.
    """
    n_dofs = len(contract["joint_names"])
    steps = len(contract["motion"]["future_step_indices"])
    n_bodies = len(contract["body_names"])

    # Non-zero starts. Everything else begins at zero, which is what "no brace yet" means.
    inits = {
        # Identity, not zeros: the goal builder right-multiplies the reference anchor rotation by
        # this, so a null quaternion would annihilate the whole block.
        "xpriv_anchor_rot_delta": [0.0, 0.0, 0.0, 1.0],
        "mode_onehot": [0.0, 1.0],  # start in COMP, which is what a zero dial means
        "mode": [1.0],
        "bal_ema": [1.0],
        # No NaN here (not valid JSON); `seeded` is the flag that selects the solver's sentinel.
        "seeded": [0.0],
    }

    graph = onnx.load(str(brace_path)).graph
    state_fields = []
    for value in graph.output:
        name = value.name.removeprefix("next_")
        shape = [d.dim_value if d.dim_value > 0 else 1 for d in value.type.tensor_type.shape.dim]
        spec = {"name": name, "shape": shape, "dtype": "float32"}
        if name in inits:
            spec["init"] = inits[name]
        state_fields.append(spec)

    return mjswan.CommandTermConfig(
        term_name="OnnxCommand",
        params={
            "onnx": _BRACE_REF,
            # The runtime wants one field nominated as "the command"; the effective force is the
            # only one that reads as a command value.
            "command_field": "force_cmd_eff",
            "rand_dim": 0,
            # The brace has nothing to resample -- it is driven entirely by the clip and the dial --
            # so the timer is pushed out of the way rather than left to fire.
            "resampling_time_range": [1.0e9, 1.0e9],
            "state_fields": state_fields,
            "input_slots": [
                {
                    "command": "motion",
                    "field": "ref_body_pos_w",
                    "input": "ref_pos_window",
                    "shape": [1, steps, n_bodies, 3],
                },
                {
                    "command": "motion",
                    "field": "ref_body_quat_w",
                    "input": "ref_rot_window",
                    "shape": [1, steps, n_bodies, 4],
                },
                {"command": "exert", "field": "command", "input": "dial", "shape": [1, 7]},
            ],
            "debug_vis": True,
            # The force-adjusted goal as a red copy of the robot, posed from the brace's own
            # `x_priv_bodies`/`x_priv_rot` -- so what is drawn is the pose the policy is actually
            # being asked for, not a redrawing of the dial. Beside mjswan's green reference ghost
            # and the robot itself the three read as: where the clip says, where the force says,
            # where the robot got to. At zero command the red sits exactly on the green, which is
            # the degeneracy `x_priv == x_ref` made visible.
            "ghost": {
                "pos_field": "x_priv_bodies",
                "quat_field": "x_priv_rot",
                "color": [0.85, 0.16, 0.16],
                "opacity": 0.45,
            },
        },
    )


class _BraceStandIn:
    """Trace-time stand-in for the brace command: shapes only, values neutral."""

    def __init__(self, n_bodies: int, n_dofs: int, n_hands: int = 2):
        import torch

        self.dof_priv_delta = torch.zeros(1, n_dofs)
        self.xpriv_anchor_pos_delta = torch.zeros(1, 3)
        self.xpriv_anchor_rot_delta = torch.tensor([[0.0, 0.0, 0.0, 1.0]])
        self.x_priv_bodies = torch.zeros(1, n_bodies, 3)
        self.x_priv_rot = torch.zeros(1, n_bodies, 4)
        self.x_priv_rot[..., 3] = 1.0
        self.force_cmd_eff = torch.zeros(1, n_hands, 3)
        self.mode_onehot = torch.tensor([[0.0, 1.0]])


def _write_io_keys(contract: dict, out_dir: Path) -> Path:
    """Write the policy JSON carrying the graph's io keys, and return its path.

    The runtime maps io keys onto the graph's inputs and outputs **positionally**, and drives the
    robot with the output it knows as ``action``. This graph's action-bearing output is its
    *second* -- ``joint_pos_targets``, the absolute PD target the RoboJuDo runner uses
    (``ort_out[1]``), not the raw residual in slot 0 -- so ``action`` is placed in that slot and
    the others keep their own names and go unread. Naming the inputs likewise keys them to the
    observation groups.
    """
    out_names = list(contract["_runtime"]["onnx_out_names"])
    if out_names[:2] != ["actions", "joint_pos_targets"]:
        raise ValueError(
            "expected the graph's first two outputs to be (actions, joint_pos_targets); "
            f"got {out_names}. The action slot below would drive the wrong tensor."
        )
    out_keys = ["residual_action", "action", *out_names[2:]]

    payload = {
        "in_keys": list(contract["_runtime"]["onnx_in_names"]),
        "out_keys": out_keys,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "policy_io_keys.json"
    path.write_text(json.dumps(payload, indent=2))
    return path


def build(
    onnx_dir: Path,
    motion_pt: Path,
    mjcf: Path,
    motion_index: int,
    output: Path,
    exert: bool,
    brace: Path,
) -> mjswan.Builder:
    contract = load_contract(onnx_dir)
    model = onnx.load(str(onnx_dir / "unified_pipeline.onnx"), load_external_data=True)

    joint_names = list(contract["joint_names"])
    body_names = list(contract["body_names"])
    control_dt = float(contract["timing"]["control_dt"])
    physics_dt = float(contract["timing"]["physics_dt"])
    future_steps = [int(s) for s in contract["motion"]["future_step_indices"]]

    spec = build_spec(mjcf, physics_dt)
    check_contract(model, contract, spec.compile())

    # The reference clip, resampled so one clip frame is one control step -- which is what makes
    # a `future_step_indices` offset mean the same thing it meant in training.
    #
    # An already-converted `.npz` is taken as-is. A ProtoMotions motion library is hundreds of
    # megabytes and holds hundreds of clips; converting the one we play is a thousandth of that, so
    # the repo carries the clip rather than the library.
    clip_path = output.parent / f"{motion_pt.stem}_{motion_index}.npz"
    clip_path.parent.mkdir(parents=True, exist_ok=True)
    if motion_pt.suffix == ".npz":
        clip_path = motion_pt
    else:
        clip_path.write_bytes(
            convert(
                motion_pt,
                motion_index=motion_index,
                target_fps=1.0 / control_dt,
                body_names=body_names,
            )
        )

    # The pose the robot holds before the first inference: the clip's first frame, so the
    # action history is seeded with a real reference rather than a zero pose.
    first_frame_dof = np.load(clip_path)["joint_pos"][0].astype(float).tolist()

    builder = mjswan.Builder()
    project = builder.add_project(name="G1 Force Control")
    scene = project.add_scene(name="Unitree G1", spec=spec, control_dt=control_dt)
    scene.set_trace_env(
        build_single_entity_trace_env(
            lambda: build_spec(mjcf, physics_dt),
            commands={
                "motion": _MotionWindowStandIn(len(future_steps), len(joint_names), len(body_names)),
                **(
                    {"brace": _BraceStandIn(len(body_names), len(joint_names))} if exert else {}
                ),
            },
        )
    )
    # The clip walks, so the camera tracks the torso rather than sitting at a world point.
    scene.set_viewer(
        mjswan.ViewerConfig(
            lookat=(0.0, 0.0, 0.15),
            distance=3.4,
            elevation=-14,
            azimuth=135,
            origin_type=mjswan.ViewerConfig.OriginType.ASSET_BODY,
            body_name=contract["robot"]["anchor_body_name"],
        )
    )

    force_command, external_wrench = hand_force_command(contract)
    hand_spring = hand_spring_config(contract, load_hand_force_cfg(onnx_dir), control_dt)

    policy = scene.add_policy(
        name="Dual force student (compensation)",
        policy=model,
        commands={
            "motion": mjswan.CommandTermConfig(
                term_name="TrackingCommand",
                # The contract's future window, in control steps. The runner reads the *nearest*
                # future frame as the current reference, so index 0 doubles as `ref_state`.
                params={"time_steps": future_steps},
            ),
            "hand_force": force_command,
            # Order matters at reset: the brace reads the motion window, so the clip has to be
            # positioned first. mjswan resets terms in config order for exactly this reason.
            **(
                {"exert": exert_dial_command(), "brace": brace_command(contract, brace)}
                if exert
                else {}
            ),
        },
        observations=observation_groups(contract, exert),
        actions={
            # The graph emits absolute PD targets, so the action IS the target: no default-pose
            # offset and no residual scale.
            "joint_pos": JointPositionActionCfg(
                actuator_names=(".*",),
                scale=1.0,
                use_default_offset=False,
                stiffness=dict(zip(joint_names, contract["control"]["stiffness"], strict=True)),
                damping=dict(zip(joint_names, contract["control"]["damping"], strict=True)),
            )
        },
        config_path=str(_write_io_keys(contract, output.parent)),
        policy_joint_names=joint_names,
        default_joint_pos=first_frame_dof,
        policy_input_shapes=onnx_input_shapes(model),
        external_wrench=external_wrench,
        hand_spring=hand_spring if exert else None,
        # Absolute targets, so the pre-inference stored action is a pose, not zeros.
        initial_action=first_frame_dof,
        default=True,
    )
    policy.add_motion(
        name=f"{motion_pt.stem} #{motion_index}",
        source=str(clip_path),
        fps=1.0 / control_dt,
        anchor_body_name=contract["robot"]["anchor_body_name"],
        body_names=tuple(body_names),
        dataset_joint_names=joint_names,
        default=True,
        # The default source advances the clip off the value the runtime passes the command
        # manager, which is `timestep * decimation` -- the control step. So it steps exactly one
        # clip frame per control step (the clip is resampled to 1/control_dt for that), it holds
        # while the sim is paused, and at the clip end it requests an episode reset so the robot
        # is re-placed on the reference. `time_source="sim"` instead derives the frame index from
        # absolute `mjData.time` and wraps it silently: no reset, so the robot keeps walking from
        # wherever it got to while the reference teleports back to the clip's origin, and the two
        # never re-align.
    )
    return builder


def install_brace_graph(dist: Path, brace: Path) -> None:
    """Copy the brace graph into each built scene, where its config's ``onnx`` ref points.

    The builder writes the graphs it traced itself; this one is exported separately (it needs
    protomotions and the checkpoint, which the app build does not), so it is placed afterwards
    beside them. The runtime discovers it structurally -- any ``onnx`` string under ``commands`` --
    so nothing else has to know about it.
    """
    if not brace.is_file():
        raise FileNotFoundError(
            f"brace graph not found: {brace}\n"
            "Export it first:  pixi run -e sim2sim python projects/force_web/brace_export.py"
        )
    targets = list(dist.glob(f"*/assets/*/{Path(_BRACE_REF).parent.name}")) or [
        d / Path(_BRACE_REF).parent.name for d in dist.glob("*/assets/*") if d.is_dir()
    ]
    if not targets:
        raise RuntimeError(f"no scene asset directory under {dist}")
    for target in targets:
        target.mkdir(parents=True, exist_ok=True)
        shutil.copy2(brace, target / Path(_BRACE_REF).name)
        print(f"  brace graph -> {target / Path(_BRACE_REF).name}")


class _MotionWindowStandIn:
    """Trace-time stand-in for the browser's ``TrackingCommand``.

    The clip lookup is data, not math, so the command stays native and these reads become graph
    inputs the runtime serves from ``getStateField``. Only the shapes matter -- the values are
    neutral (identity quaternions, ready).
    """

    def __init__(self, num_steps: int, num_joints: int, num_bodies: int):
        import torch

        self.ref_joint_pos = torch.zeros(1, num_steps, num_joints)
        self.ref_joint_vel = torch.zeros(1, num_steps, num_joints)
        self.ref_body_pos_w = torch.zeros(1, num_steps, num_bodies, 3)
        self.ref_body_quat_w = torch.zeros(1, num_steps, num_bodies, 4)
        self.ref_body_quat_w[..., 0] = 1.0
        self.is_ready = torch.ones(1, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx-dir", type=Path, default=DEFAULT_ONNX_DIR)
    parser.add_argument("--motion-file", type=Path, default=DEFAULT_MOTION)
    parser.add_argument("--motion-index", type=int, default=0)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--output", type=Path, default=HERE / "dist")
    parser.add_argument(
        "--brace",
        type=Path,
        default=HERE / "brace.onnx",
        help="x_priv brace graph, from brace_export.py",
    )
    parser.add_argument(
        "--exert",
        action="store_true",
        help="wire exert mode (needs --brace; blocked on onnxruntime-web, see the README)",
    )
    parser.add_argument("--serve", action="store_true", help="serve the build on localhost")
    args = parser.parse_args()
    # Absolute: a relative output path is not resolved against the cwd downstream, so the build
    # lands beside the module instead of where it was asked for.
    args.output = args.output.resolve()

    builder = build(
        args.onnx_dir, args.motion_file, args.mjcf, args.motion_index, args.output, args.exert,
        args.brace,
    )
    app = builder.build(output_dir=str(args.output))
    if args.exert:
        install_brace_graph(args.output, args.brace)
    print(f"built {args.output}")
    if args.serve or os.getenv("FORCE_WEB_SERVE") == "1":
        app.launch()


if __name__ == "__main__":
    main()
