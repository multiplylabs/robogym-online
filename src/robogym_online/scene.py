# SPDX-License-Identifier: Apache-2.0
"""The MuJoCo scene the robot stands in, and the asset paths everything else resolves against.

Separate from ``build_app`` because three entry points need the same model and only one of them
builds a browser app: the headless contract check and the ARDY reference builder want the compiled
kinematics without pulling in ``onnx`` and ``mjswan``. Same spec for all of them, so a reference
generated here is expressed in exactly the kinematics the browser will simulate.
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import yaml

# This repo carries no checkpoint, clip or robot model: they are large, and the policy is not ours
# to redistribute. Point these at wherever they were materialized (CI fetches them; see the
# workflow), or pass the flags.
DEFAULT_ONNX_DIR = Path(os.environ.get("ROBOGYM_ONNX_DIR", "assets/compiled_models"))
DEFAULT_MOTION = Path(os.environ.get("ROBOGYM_MOTION", "assets/motion.npz"))
DEFAULT_MJCF = Path(os.environ.get("ROBOGYM_MJCF", "assets/mjcf/g1_holo_compat.xml"))
# MotionBricks lives in its own tree (checkpoints, skeleton assets); it is not vendored here.
DEFAULT_MOTIONBRICKS_ROOT = Path(
    os.environ.get("ROBOGYM_MOTIONBRICKS", "../GR00T-WholeBodyControl/motionbricks")
)
# The same generator frozen to ONNX, which SONIC's controller loads and CLAW drives. Runs on CPU,
# so a machine hosting the browser demo needs no GPU.
DEFAULT_PLANNER_ROOT = Path(
    os.environ.get("ROBOGYM_PLANNER_ONNX", "../GR00T-WholeBodyControl")
)

# Half-extent of the ground plane, in metres. See `_add_scene_visuals`.
_FLOOR_HALF_SIZE = 60.0


def load_contract(onnx_dir: Path) -> dict:
    """The exported deployment contract (``unified_pipeline.yaml``)."""
    with (onnx_dir / "unified_pipeline.yaml").open() as f:
        return yaml.safe_load(f)


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

