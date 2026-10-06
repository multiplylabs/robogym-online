/**
 * The virtual contact a force-exertion policy pushes against, and the gauge that reads it.
 *
 * A policy trained to *produce* a hand force is trained against a Kelvin-Voigt contact anchored at
 * the reference hand: the hand leads along a push axis, the contact resists, and the reaction loads
 * the whole body. Without that reaction the hand simply moves to its displaced target, nothing is
 * exerted, and there is nothing to measure — the arm lead is passive. This applies the reaction and
 * publishes the force it implies, which is the same quantity the training reward measured.
 *
 *     lead = n_robot . (x_hand - anchor)  -  n_ref . (x_ref - ref_anchor)
 *     p    = clamp(lead, p_max)                       (smooth, optionally two-sided)
 *     F    = K_v * p + C_v * dp/dt                    along the push axis
 *     robot receives -F * n_robot at the hand
 *
 * The lead is measured in each frame's *own* anchor — the robot's hand against the robot's torso,
 * the reference hand against the reference torso — so the force does not move with global tracking
 * drift. That is `local_frame_lead` in the training config, and it is why the robot-side push axis
 * has to be rotated by the robot's heading here rather than taken from the graph.
 *
 * Everything the reference side needs (`K_v`, `C_v`, the push axis, the reference hand and anchor)
 * comes from the brace graph's published state, so the browser never re-derives the training
 * constants.
 */

import * as THREE from 'three';

import { mjcToThreeCoordinate } from '../scene/coordinate';

type MjModel = import('mujoco').MjModel;
type MjData = import('mujoco').MjData;

/** Metres of arrow per newton. 9 N (the trained ceiling) reads as a forearm. */
const ARROW_M_PER_N = 0.05;
const COMMANDED_COLOR = 0x7894ad;
const EXERTED_COLOR = 0xf5a03f;

export interface HandSpringTarget {
  /** Body the force acts on. */
  body: string;
  /** Its index in the policy's body order, for slicing the graph's per-hand arrays. */
  hand: number;
}

export interface HandSpringConfig {
  /** Command term publishing the brace state. */
  command_name: string;
  /** Body whose heading defines the robot-side frame (the policy's anchor). */
  anchor_body: string;
  targets: HandSpringTarget[];
  /** Max lead the contact supports, metres (`max_lead`). */
  max_lead: number;
  /** Softplus knee width for the lead clamp, 1/m (`smooth_contact_beta`); 0 disables smoothing. */
  smooth_beta?: number;
  /** Symmetric lead, so the contact pulls as well as pushes (`two_sided_spring`). */
  two_sided?: boolean;
  /** Include the `C_v * dp/dt` damper term (`contact_damping`). */
  damping?: boolean;
  /** Control step, for the lead's finite difference. */
  dt: number;
  damper_vel_ema_alpha?: number;
  /** Draw the commanded and exerted force at each hand. */
  gauge?: boolean;
}

/** Shaft + head, unit length along +Y with its base at the origin. */
function makeArrow(color: number): THREE.Group {
  const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, depthWrite: false });
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 1, 10).translate(0, 0.5, 0), material);
  const head = new THREE.Mesh(new THREE.ConeGeometry(0.022, 0.06, 10).translate(0, 0.97, 0), material);
  const group = new THREE.Group();
  group.add(shaft, head);
  group.visible = false;
  return group;
}

/** Place `arrow` from `origin` along `vec` (MuJoCo world), hiding it when the force is ~0. */
function placeArrow(arrow: THREE.Group, origin: ArrayLike<number>, vec: readonly number[]): void {
  const magnitude = Math.hypot(vec[0], vec[1], vec[2]);
  if (magnitude < 1e-3) {
    arrow.visible = false;
    return;
  }
  // Both ends converted, not the direction: the MuJoCo-to-three mapping is an axis swizzle, so
  // transforming a vector on its own and a point are not the same operation.
  const start = mjcToThreeCoordinate([origin[0], origin[1], origin[2]]);
  const tip = mjcToThreeCoordinate([origin[0] + vec[0], origin[1] + vec[1], origin[2] + vec[2]]);
  const dir = tip.clone().sub(start);
  const length = dir.length();
  arrow.position.copy(start);
  arrow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  arrow.scale.set(1, length, 1);
  arrow.visible = true;
}

interface StateSource {
  getStateField?(field: string): Float32Array | null;
}

function decodeBodyNames(mjModel: MjModel): string[] {
  const bytes = new Uint8Array(mjModel.names);
  const decoder = new TextDecoder();
  const names: string[] = [];
  for (let b = 0; b < mjModel.nbody; b++) {
    const start = mjModel.name_bodyadr[b];
    let end = start;
    while (end < bytes.length && bytes[end] !== 0) end++;
    names.push(decoder.decode(bytes.subarray(start, end)));
  }
  return names;
}

function resolveBody(names: string[], name: string): number {
  const direct = names.indexOf(name);
  if (direct > 0) return direct;
  return names.findIndex((n) => n.endsWith(`/${name}`));
}

/** Rotate `v` by the yaw-only part of a wxyz quaternion — the heading frame. */
function applyHeading(quatWxyz: ArrayLike<number>, v: readonly number[]): [number, number, number] {
  const [w, x, y, z] = [quatWxyz[0], quatWxyz[1], quatWxyz[2], quatWxyz[3]];
  // Yaw of the quaternion, then a plain z-rotation: the push axis is defined against heading, not
  // against the full orientation, so pitch and roll must not turn it.
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
}

export class HandSpringContact {
  private readonly bodyIds: number[];
  private readonly anchorId: number;
  /** Previous clamped lead per hand, for the damper's finite difference. */
  private readonly prevLead: Float32Array;
  private readonly rate: Float32Array;
  private readonly primed: boolean[];
  /** Last measured along-axis force per hand, newtons — what the gauge reads. */
  readonly measured: Float32Array;
  private readonly commandedArrows: THREE.Group[] = [];
  private readonly exertedArrows: THREE.Group[] = [];

  constructor(
    private readonly config: HandSpringConfig,
    mjModel: MjModel,
    parent?: THREE.Object3D | null,
  ) {
    const names = decodeBodyNames(mjModel);
    this.bodyIds = config.targets.map((t) => resolveBody(names, t.body));
    this.anchorId = resolveBody(names, config.anchor_body);
    this.prevLead = new Float32Array(config.targets.length);
    this.measured = new Float32Array(config.targets.length);
    this.rate = new Float32Array(config.targets.length);
    this.primed = config.targets.map(() => false);
    if (config.gauge && parent) {
      for (let i = 0; i < config.targets.length; i++) {
        const commanded = makeArrow(COMMANDED_COLOR);
        const exerted = makeArrow(EXERTED_COLOR);
        commanded.name = `hand-force-commanded-${i}`;
        exerted.name = `hand-force-exerted-${i}`;
        parent.add(commanded, exerted);
        this.commandedArrows.push(commanded);
        this.exertedArrows.push(exerted);
      }
    }
  }

  /** One-sided or symmetric lead clamp, softened as `smooth_contact` does. */
  private clampLead(lead: number): number {
    const pMax = this.config.max_lead;
    if (this.config.two_sided) return (this.config.smooth_beta ?? 0) > 0
      ? pMax * Math.tanh(lead / Math.max(pMax, 1e-9)) : Math.max(-pMax, Math.min(pMax, lead));
    const beta = this.config.smooth_beta ?? 0;
    // Softplus knee at 0, then the same tanh saturation at p_max.
    const positive = beta > 0 ? Math.log1p(Math.exp(Math.min(beta * lead, 30))) / beta : Math.max(lead, 0);
    return beta > 0 ? pMax - Math.log1p(Math.exp(Math.min(beta * (pMax - positive), 30))) / beta : Math.min(pMax, positive);
  }

  /**
   * Apply this step's reaction and refresh {@link measured}.
   *
   * The runtime clears external forces before all writers. Add only the contact's reaction.
   */
  apply(mjData: MjData, brace: StateSource | undefined): void {
    const read = (field: string) => brace?.getStateField?.(field) ?? null;
    const kv = read('endpoint_kv');
    const cv = read('endpoint_cv');
    const axisRef = read('push_axis_w');
    const axisLocal = read('push_axis_local');
    const refHand = read('ref_hand_pos');
    const refAnchor = read('ref_anchor_pos');
    if (!kv || !axisRef || !axisLocal || !refHand || !refAnchor) {
      this.measured.fill(0);
      for (const a of [...this.commandedArrows, ...this.exertedArrows]) a.visible = false;
      return;
    }

    const anchor = [
      mjData.xpos[this.anchorId * 3],
      mjData.xpos[this.anchorId * 3 + 1],
      mjData.xpos[this.anchorId * 3 + 2],
    ];
    const anchorQuat = [
      mjData.xquat[this.anchorId * 4],
      mjData.xquat[this.anchorId * 4 + 1],
      mjData.xquat[this.anchorId * 4 + 2],
      mjData.xquat[this.anchorId * 4 + 3],
    ];

    for (let t = 0; t < this.config.targets.length; t++) {
      const bodyId = this.bodyIds[t];
      const h = this.config.targets[t].hand;
      const base = bodyId * 6;
      if (bodyId <= 0) continue;

      // The dial's direction carried into the robot's own heading.
      const nRobot = applyHeading(anchorQuat, [
        axisLocal[h * 3],
        axisLocal[h * 3 + 1],
        axisLocal[h * 3 + 2],
      ]);
      const active = Math.hypot(nRobot[0], nRobot[1], nRobot[2]);
      if (active < 1e-6) {
        this.measured[t] = 0;
        this.prevLead[t] = 0; this.rate[t] = 0; this.primed[t] = false;
        if (this.commandedArrows[t]) {
          this.commandedArrows[t].visible = false;
          this.exertedArrows[t].visible = false;
        }
        continue;
      }

      let robotLead = 0;
      let refLead = 0;
      for (let i = 0; i < 3; i++) {
        robotLead += nRobot[i] * (mjData.xpos[bodyId * 3 + i] - anchor[i]);
        refLead += axisRef[h * 3 + i] * (refHand[h * 3 + i] - refAnchor[i]);
      }
      const rawLead = robotLead - refLead;
      const p = this.clampLead(rawLead);
      // Training differentiates raw lead and filters its rate, independently per hand.
      const rawRate = this.primed[t] ? (rawLead - this.prevLead[t]) / this.config.dt : 0;
      const alpha = this.config.damper_vel_ema_alpha ?? 0.3;
      const pDot = this.rate[t] = alpha * rawRate + (1-alpha) * this.rate[t];
      this.prevLead[t] = rawLead; this.primed[t] = true;

      let force = kv[h] * p;
      if (this.config.damping && cv) force += cv[h] * pDot;
      this.measured[t] = force;

      // Reaction on the robot: the contact pushes back along -n.
      for (let i = 0; i < 3; i++) mjData.xfrc_applied[base + i] += -force * nRobot[i];

      // The gauge: both arrows start at the hand, so their lengths compare directly. Commanded is
      // what was asked for after the feasibility caps; exerted is what the contact actually reads.
      const hand = [
        mjData.xpos[bodyId * 3],
        mjData.xpos[bodyId * 3 + 1],
        mjData.xpos[bodyId * 3 + 2],
      ];
      if (this.commandedArrows[t]) {
        const cmdW = read('force_cmd_eff');
        const cmdMag = cmdW ? Math.hypot(cmdW[h * 3], cmdW[h * 3 + 1], cmdW[h * 3 + 2]) : 0;
        const cmd = nRobot.map(v => v * cmdMag * ARROW_M_PER_N);
        placeArrow(this.commandedArrows[t], hand, cmd);
        placeArrow(this.exertedArrows[t], hand, [
          force * nRobot[0] * ARROW_M_PER_N,
          force * nRobot[1] * ARROW_M_PER_N,
          force * nRobot[2] * ARROW_M_PER_N,
        ]);
      }
    }
    window.dispatchEvent(new CustomEvent('brace:force-reading', {detail:{
      measured:Array.from(this.measured), commanded:Array.from(read('force_cmd_eff') ?? []),
      time:Number(mjData.time)
    }}));
  }

  reset(): void {
    this.prevLead.fill(0);
    this.measured.fill(0);
    this.primed.fill(false); this.rate.fill(0);
  }
}
