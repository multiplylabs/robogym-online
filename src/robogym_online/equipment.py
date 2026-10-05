"""Rigid-grip gym equipment and torso-relative upper-body reference poses.

Payloads change MuJoCo mass/inertia; they are not constant world-frame forces.
Shared equipment is attached to the left palm and welded to the right palm.
The ideal grip has no finger articulation, slipping, dropping or object collisions.
"""

from __future__ import annotations

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

# Geometry stays the same when mass changes. All lengths are metres.
EQUIPMENT = {
    "none": {"label": "Empty hands", "mass": 0},
    "dumbbells": {"label": "Dumbbells", "mass": 2.0, "each": 1.0, "target": [0.21, 0.22, -0.07]},
    "kettlebell": {"label": "Kettlebell", "mass": 3.5, "target": [0.23, 0.10, -0.06]},
    "barbell": {"label": "Barbell", "mass": 2.0, "target": [0.23, 0.20, -0.06]},
}
PALM_OFFSET = np.array([-0.04, 0.0, 0.0])
HOLD_PITCH = 0.5


def add_equipment(spec):
    """Massless, hidden shapes preserve the robot's body and joint contract."""
    graphite = [0.10, 0.14, 0.20, 0]
    chrome = [0.68, 0.75, 0.83, 0]
    blue = [0.15, 0.36, 0.68, 0]

    def geom(body, name, kind, pos, size, color, quat=None):
        kw = {
            "name": name,
            "type": kind,
            "pos": pos,
            "size": size,
            "rgba": color,
            "density": 0,
            "contype": 0,
            "conaffinity": 0,
            "group": 0,
        }
        if quat is not None:
            kw["quat"] = quat
        body.add_geom(**kw)

    # Cylinders' native Z axis is rotated onto the handle's lateral Y axis.
    axis_y = [2**-0.5, 2**-0.5, 0, 0]
    for side in ("left", "right"):
        b = spec.body(f"{side}_rubber_hand")
        geom(
            b,
            f"gym_dumbbells_{side}_handle",
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            [-0.04, 0, 0],
            [0.013, 0.115, 0],
            chrome,
            axis_y,
        )
        for end in (-1, 1):
            geom(
                b,
                f"gym_dumbbells_{side}_plate_{end}",
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                [-0.04, end * 0.105, 0],
                [0.068, 0.026, 0],
                graphite,
                axis_y,
            )
            geom(
                b,
                f"gym_dumbbells_{side}_cap_{end}",
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                [-0.04, end * 0.135, 0],
                [0.035, 0.005, 0],
                blue,
                axis_y,
            )
    b = spec.body("left_rubber_hand")
    drop = np.array([np.sin(HOLD_PITCH), 0, -np.cos(HOLD_PITCH)])
    drop_quat = [np.cos((np.pi - HOLD_PITCH) / 2), 0, np.sin((np.pi - HOLD_PITCH) / 2), 0]
    for name, separation in [("barbell", 0.20), ("kettlebell", 0.10)]:
        if name == "barbell":
            geom(
                b,
                "gym_barbell_handle",
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                [-0.04, -separation, 0],
                [0.014, 0.48, 0],
                chrome,
                axis_y,
            )
            for end in (-1, 1):
                geom(
                    b,
                    f"gym_barbell_plate_{end}",
                    mujoco.mjtGeom.mjGEOM_CYLINDER,
                    [-0.04, -separation + end * 0.38, 0],
                    [0.11, 0.023, 0],
                    graphite,
                    axis_y,
                )
                geom(
                    b,
                    f"gym_barbell_cap_{end}",
                    mujoco.mjtGeom.mjGEOM_CYLINDER,
                    [-0.04, -separation + end * 0.41, 0],
                    [0.046, 0.006, 0],
                    blue,
                    axis_y,
                )
        else:
            geom(
                b,
                "gym_kettlebell_ball",
                mujoco.mjtGeom.mjGEOM_SPHERE,
                (PALM_OFFSET + 0.15 * drop + [0, -0.10, 0]).tolist(),
                [0.095, 0, 0],
                graphite,
            )
            geom(
                b,
                "gym_kettlebell_grip",
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                [-0.04, -0.10, 0],
                [0.015, 0.105, 0],
                chrome,
                axis_y,
            )
            for end in (-1, 1):
                geom(
                    b,
                    f"gym_kettlebell_handle_{end}",
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    (PALM_OFFSET + 0.055 * drop + [0, -0.10 + end * 0.085, 0]).tolist(),
                    [0.015, 0.055, 0],
                    blue,
                    drop_quat,
                )
        # body2 pose relative to body1; both palm axes point forward.
        spec.add_equality(
            name=f"gym_{name}_grip",
            type=mujoco.mjtEq.mjEQ_WELD,
            name1="left_rubber_hand",
            name2="right_rubber_hand",
            objtype=mujoco.mjtObj.mjOBJ_BODY,
            active=False,
            data=[0, 0, 0, 0, -2 * separation, 0, 1, 0, 0, 0, 0.1],
            solref=[0.03, 1],
        )


def solve_carry_pose(model, joint_names, name):
    """Solve each 7-DOF arm with limits; retain the waist and lower body."""
    if name not in EQUIPMENT or name == "none":
        raise ValueError(name)
    data = mujoco.MjData(model)
    data.qpos[:7] = [0, 0, 0.78, 1, 0, 0, 0]
    torso = model.body("torso_link").id
    target = np.asarray(EQUIPMENT[name]["target"])
    arm_indices, angles = [], []
    diagnostics = []
    for side, sign in [("left", 1), ("right", -1)]:
        names = [
            n
            for n in joint_names
            if n.startswith(side + "_") and any(x in n for x in ("shoulder", "elbow", "wrist"))
        ]
        joints = [model.joint(n).id for n in names]
        qadr = model.jnt_qposadr[joints]
        hand = model.body(f"{side}_rubber_hand").id
        limits = model.jnt_range[joints]
        desired = target.copy()
        desired[1] *= sign
        # Targets describe grip centres, not the terminal rubber-hand body origin.
        hold_rotation = Rotation.from_euler("y", HOLD_PITCH).as_matrix()
        desired -= hold_rotation @ PALM_OFFSET

        def residual(q, qadr=qadr, hand=hand, desired=desired, hold_rotation=hold_rotation):
            data.qpos[qadr] = q
            mujoco.mj_kinematics(model, data)
            R = data.xmat[torso].reshape(3, 3)
            position = R.T @ (data.xpos[hand] - data.xpos[torso])
            rotation = Rotation.from_matrix(hold_rotation.T @ R.T @ data.xmat[hand].reshape(3, 3)).as_rotvec()
            return np.r_[(position - desired) * 8, rotation, 0.015 * q]

        initial = np.zeros(7)
        initial[0] = 0.3
        initial[1] = sign * 0.2
        initial[3] = 1.0
        result = least_squares(
            residual,
            np.clip(initial, limits[:, 0] + 0.001, limits[:, 1] - 0.001),
            bounds=(limits[:, 0] + 0.0001, limits[:, 1] - 0.0001),
            max_nfev=400,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        errors = residual(result.x)
        pos_error = np.linalg.norm(errors[:3]) / 8
        rot_error = np.linalg.norm(errors[3:6])
        if pos_error > 0.015 or rot_error > 0.10:
            raise ValueError(
                f"{name} {side} unreachable: position={pos_error:.4f}, orientation={rot_error:.4f}"
            )
        arm_indices.extend(joint_names.index(n) for n in names)
        angles.extend(result.x)
        diagnostics.append(
            {"hand": side, "position_error_m": float(pos_error), "orientation_error_rad": float(rot_error)}
        )
    return np.asarray(arm_indices), np.asarray(angles), diagnostics


class CarryOverlay:
    """Apply arm targets only to decoded references, preserving raw planner context."""

    def __init__(self, model, joint_names):
        self.poses = {
            name: solve_carry_pose(model, joint_names, name) for name in EQUIPMENT if name != "none"
        }
        self.reset()

    def reset(self):
        self.name = "none"
        self.segments = []

    def select(self, name, qpos):
        if name not in EQUIPMENT:
            raise ValueError(f"Unknown gym equipment: {name}")
        if name == self.name:
            return
        start = len(qpos)
        indices = self.poses["dumbbells"][0]
        # Start from the already composed pose, including a partially completed transition.
        previous = self.apply(qpos)[-1, 7 + indices].copy() if start else None
        self.segments.append((start, name, previous))
        self.name = name

    def apply(self, qpos):
        if not self.segments:
            return qpos
        out = qpos.copy()
        indices = self.poses["dumbbells"][0]
        for start, name, previous in self.segments:
            if start >= len(out):
                continue
            t = np.clip((np.arange(start, len(out)) - start) / 45, 0, 1)
            w = (t * t * t * (10 - 15 * t + 6 * t * t))[:, None]  # 1.5 s at 30 Hz
            source = out[start:, 7 + indices] if previous is None else previous
            target = qpos[start:, 7 + indices] if name == "none" else self.poses[name][1]
            out[start:, 7 + indices] = source * (1 - w) + target * w
        return out


def payload_properties(name):
    """Mass, centre and diagonal inertia in the left/right hand frames."""
    if name == "dumbbells":
        return [(1.0, [-0.04, 0, 0], [0.0125, 0.002, 0.0125])] * 2
    if name == "barbell":
        return [(2.0, [-0.04, -0.20, 0], [0.31 * 2 / 3, 0.012, 0.31 * 2 / 3]), (0.0, [0, 0, 0], [0, 0, 0])]
    if name == "kettlebell":
        return [
            (
                3.5,
                (
                    PALM_OFFSET
                    + 0.13 * np.array([np.sin(HOLD_PITCH), 0, -np.cos(HOLD_PITCH)])
                    + [0, -0.10, 0]
                ).tolist(),
                [0.019, 0.019, 0.014],
            ),
            (0.0, [0, 0, 0], [0, 0, 0]),
        ]
    return [(0.0, [0.12, 0, 0], [0, 0, 0])] * 2


def set_payload(model, data, name):
    """Native counterpart of the browser plugin, used by loaded-policy audits."""
    if name not in EQUIPMENT:
        raise ValueError(name)
    for side, (mass, center, inertia) in zip(("left", "right"), payload_properties(name), strict=True):
        b = model.body(f"{side}_rubber_hand").id
        model.body_mass[b] = mass
        model.body_ipos[b] = center
        model.body_inertia[b] = inertia
        model.body_iquat[b] = [1, 0, 0, 0]
    for i in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if geom_name.startswith("gym_"):
            model.geom_rgba[i, 3] = float(geom_name.startswith(f"gym_{name}_"))
    for shared in ("barbell", "kettlebell"):
        eq = model.equality(f"gym_{shared}_grip").id
        data.eq_active[eq] = int(name == shared)
    # mj_setConst uses qpos0 to recompute model constants. Preserve live state.
    saved = data.qpos.copy()
    mujoco.mj_setConst(model, data)
    data.qpos[:] = saved
    mujoco.mj_forward(model, data)


class ArmHold:
    """Blend torque-limited MuJoCo arm servos with the policy's motor commands.

    A motor's affine bias implements PD in MuJoCo at every physics substep.
    Policy torque retains weight (1-w); the carrying servo contributes weight w.
    Force limits cap their combined torque at the original motor effort limit.
    """

    def __init__(self, model, joint_names, kp, kd):
        self.model = model
        self.poses = CarryOverlay(model, joint_names).poses
        indices = self.poses["dumbbells"][0]
        self.qadr = np.array([model.joint(joint_names[i]).qposadr[0] for i in indices])
        joints = [model.joint(joint_names[i]).id for i in indices]
        self.actuators = np.array([np.flatnonzero(model.actuator_trnid[:, 0] == j)[0] for j in joints])
        self.kp = np.asarray(kp)[indices] * 3
        self.kd = np.asarray(kd)[indices] * 2
        a = self.actuators
        self.original = {
            key: getattr(model, key)[a].copy()
            for key in [
                "actuator_gainprm",
                "actuator_biasprm",
                "actuator_biastype",
                "actuator_forcerange",
                "actuator_forcelimited",
            ]
        }
        self.reset()

    def reset(self):
        for key, value in self.original.items():
            getattr(self.model, key)[self.actuators] = value
        self.name = "none"
        self.elapsed = 0.0
        self.weight = 0.0
        self.start = None
        self.target = None

    def select(self, data, name):
        if name not in EQUIPMENT:
            raise ValueError(name)
        if name == self.name:
            return
        self.name = name
        self.elapsed = 0.0
        self.start_weight = self.weight
        self.start = data.qpos[self.qadr].copy()
        self.target = self.start if name == "none" else self.poses[name][1]
        a = self.actuators
        self.model.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_AFFINE
        self.model.actuator_forcelimited[a] = True
        self.model.actuator_forcerange[a] = self.model.actuator_ctrlrange[a]

    def update(self, dt):
        if self.target is None:
            return
        self.elapsed += dt
        t = np.clip(self.elapsed / 1.5, 0, 1)
        blend = t * t * t * (10 - 15 * t + 6 * t * t)
        goal = 0.0 if self.name == "none" else 1.0
        self.weight = float(self.start_weight * (1 - blend) + goal * blend)
        target = self.start * (1 - blend) + self.target * blend
        a = self.actuators
        w = self.weight
        self.model.actuator_gainprm[a, 0] = 1 - w
        self.model.actuator_biasprm[a, 0] = w * self.kp * target
        self.model.actuator_biasprm[a, 1] = -w * self.kp
        self.model.actuator_biasprm[a, 2] = -w * self.kd
        if self.name == "none" and t >= 1:
            self.reset()
