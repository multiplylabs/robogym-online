/* MuJoCo command plugin: physical payloads and ideal two-hand grips. */
function names(model, count, addresses) {
  const bytes = new Uint8Array(model.names), decoder = new TextDecoder();
  return Array.from({length: count}, (_, i) => {
    let end = addresses[i]; while (bytes[end]) end++;
    return decoder.decode(bytes.subarray(addresses[i], end));
  });
}
class GymEquipment {
  constructor(_name, config, context) {
    this.context = context; this.catalog = config.catalog; this.applied = 'none';
    this.desired = 'none'; this.elapsed = 0; this.readyFor = 0; this.resolved = false;
  }
  getCommand() { return new Float32Array(0); }
  getUiConfig() { return null; }
  getStateField() { return null; }
  resolve() {
    const {mjModel: m, bodies} = this.context;
    if (!m || !bodies) return false;
    const bn = names(m, m.nbody, m.name_bodyadr);
    const gn = names(m, m.ngeom, m.name_geomadr);
    const en = names(m, m.neq, m.name_eqadr);
    this.hands = ['left_rubber_hand', 'right_rubber_hand'].map(n => bn.indexOf(n));
    this.torso = bn.indexOf('torso_link');
    this.equalities = Object.fromEntries(['kettlebell', 'barbell'].map(n => [n, en.indexOf(`gym_${n}_grip`)]));
    this.meshes = []; this.ghostIndices = {};
    for (const b of this.hands) {
      // The renderer creates one direct mesh per visible hand geom, in model order.
      const geoms = gn.map((n, i) => ({name:n, id:i})).filter(g => m.geom_bodyid[g.id] === b && m.geom_group[g.id] < 3);
      const meshes = bodies[b]?.children.filter(c => c.isMesh) ?? [];
      if (geoms.length !== meshes.length) throw new Error('Equipment mesh mapping does not match the scene.');
      this.ghostIndices[bn[b]] = geoms.flatMap((g,i) => g.name.startsWith('gym_') ? [i] : []);
      geoms.forEach((g, i) => { if (g.name.startsWith('gym_')) this.meshes.push({...g, mesh:meshes[i]}); });
    }
    const joints = names(m, m.njnt, m.name_jntadr);
    this.arms = this.catalog.dumbbells.arms.map(arm => {
      const joint = joints.indexOf(arm.joint);
      const actuator = Array.from({length:m.nu},(_,i)=>i).find(i => m.actuator_trnid[i*2] === joint);
      return {...arm,joint,actuator,qadr:m.jnt_qposadr[joint], original:{gain:Array.from(m.actuator_gainprm.subarray(actuator*10,actuator*10+10)),bias:Array.from(m.actuator_biasprm.subarray(actuator*10,actuator*10+10)),type:m.actuator_biastype[actuator],limited:m.actuator_forcelimited[actuator],range:Array.from(m.actuator_forcerange.subarray(actuator*2,actuator*2+2))}};
    });
    this.holdWeight = 0; this.holdElapsed = 0; this.holdName = 'none';
    this.resolved = true;
    window.BraceGym.physics = () => ({applied:this.applied, desired:this.desired, elapsed:this.elapsed, gripError:this.desired === 'none' ? 0 : this.gripError(this.desired), masses:this.hands.map(b => Number(m.body_mass[b])), welds:Object.fromEntries(Object.entries(this.equalities).map(([n,i]) => [n,Boolean(this.context.mjData.eq_active[i])])), root:Array.from(this.context.mjData.qpos.subarray(0,3)), meshes:this.meshes.filter(g => g.mesh.visible).length, hands:this.hands.map(b => Array.from(this.context.mjData.xpos.subarray(b*3,b*3+3))), torso:Array.from(this.context.mjData.xpos.subarray(this.torso*3,this.torso*3+3))});
    this.apply('none'); return true;
  }
  apply(name) {
    const {mujoco, mjModel: m, mjData: d} = this.context;
    const payloads = this.catalog[name].payloads;
    for (let h=0; h<2; h++) {
      const b = this.hands[h], p = payloads[h];
      m.body_mass[b] = p[0];
      m.body_ipos.set(p[1], b*3); m.body_inertia.set(p[2], b*3);
      m.body_iquat.set([1,0,0,0], b*4);
    }
    for (const [n, eq] of Object.entries(this.equalities)) d.eq_active[eq] = Number(n === name);
    const qpos = new Float64Array(d.qpos), qvel = new Float64Array(d.qvel);
    mujoco.mj_setConst(m, d);
    d.qpos.set(qpos); d.qvel.set(qvel);
    mujoco.mj_forward(m, d);
    for (const {id, name: geomName, mesh} of this.meshes) {
      const visible = geomName.startsWith(`gym_${name}_`);
      m.geom_rgba[id*4+3] = Number(visible); mesh.visible = visible;
      mesh.material.opacity = 1; mesh.material.transparent = false;
      mesh.material.depthWrite = true; mesh.material.metalness = geomName.includes('handle') || geomName.includes('grip') ? .65 : .2;
      mesh.material.roughness = .42; mesh.material.needsUpdate = true;
    }
    this.applied = name;
    window.BraceGym?.status(name === 'none' ? 'Empty hands. Choose a weight to begin.' : `Carrying ${this.catalog[name].label} · ${this.catalog[name].mass} kg total.`, name === 'none' ? 'empty' : 'carrying');
    window.dispatchEvent(new CustomEvent('brace:payload', {detail:{name, mass:this.catalog[name].mass}}));
  }
  startHold(name) {
    const {mjModel:m,mjData:d} = this.context;
    this.holdName = name; this.holdElapsed = 0;
    this.holdStartWeight = this.holdWeight;
    this.holdStart = this.arms.map(a => Number(d.qpos[a.qadr]));
    this.holdTarget = name === 'none' ? this.holdStart : this.catalog[name].arms.map(a => a.target);
    for (const a of this.arms) {
      m.actuator_biastype[a.actuator] = this.context.mujoco.mjtBias.mjBIAS_AFFINE.value;
      m.actuator_forcelimited[a.actuator] = 1;
      m.actuator_forcerange.set(m.actuator_ctrlrange.subarray(a.actuator*2,a.actuator*2+2),a.actuator*2);
    }
  }
  restoreArms() {
    const m = this.context.mjModel;
    for (const a of this.arms ?? []) {
      const o = a.original; m.actuator_gainprm.set(o.gain,a.actuator*10);m.actuator_biasprm.set(o.bias,a.actuator*10);m.actuator_biastype[a.actuator]=o.type;m.actuator_forcelimited[a.actuator]=o.limited;m.actuator_forcerange.set(o.range,a.actuator*2);
    }
    this.holdTarget = null; this.holdWeight = 0;
  }
  updateHold(dt) {
    if (!this.holdTarget) return;
    const m = this.context.mjModel; this.holdElapsed += dt;
    const t = Math.min(this.holdElapsed/1.5,1), s=t*t*t*(10-15*t+6*t*t);
    const goalWeight = this.holdName === 'none' ? 0 : 1;
    const w = this.holdWeight = this.holdStartWeight*(1-s)+goalWeight*s;
    this.arms.forEach((a,i) => {
      const target = this.holdStart[i]*(1-s)+this.holdTarget[i]*s, adr=a.actuator*10;
      m.actuator_gainprm[adr]=1-w; m.actuator_biasprm[adr]=w*a.kp*target;
      m.actuator_biasprm[adr+1]=-w*a.kp; m.actuator_biasprm[adr+2]=-w*a.kd;
    });
    if (goalWeight === 0 && t === 1) this.restoreArms();
  }
  gripError(name) {
    const d = this.context.mjData, t = this.torso;
    const torso = Array.from(d.xmat.subarray(t*9,t*9+9)), origin = d.xpos.subarray(t*3,t*3+3);
    let error = 0;
    for (let h=0; h<2; h++) {
      const hand = this.hands[h], R = d.xmat.subarray(hand*9, hand*9+9);
      const p = Array.from(d.xpos.subarray(hand*3,hand*3+3), (v,i) => v-origin[i]-.04*R[i*3]);
      const local = [0,1,2].map(j => torso[j]*p[0]+torso[3+j]*p[1]+torso[6+j]*p[2]);
      const target = this.catalog[name].target;
      error = Math.max(error, Math.hypot(local[0]-target[0], local[1]-target[1]*(h===0?1:-1), local[2]-target[2]));
    }
    return error;
  }
  update(dt) {
    if (!this.resolved && !this.resolve()) return;
    // Reference ghosts tint every shape, including initially transparent payload geoms.
    // Equipment belongs only to the physical robot, so hide those cloned shapes.
    this.context.scene.traverse(object => {
      const indices = this.ghostIndices[object.name];
      if (!indices || this.hands.some(b => this.context.bodies[b] === object)) return;
      const meshes = object.children.filter(c => c.isMesh);
      for (const i of indices) if (meshes[i]) meshes[i].visible = false;
    });
    const selected = window.BraceGym?.selected ?? 'none';
    if (selected !== this.desired) {
      const previousError = window.BraceGym?.lastError;
      this.apply('none'); if (previousError) window.BraceGym.status(previousError, 'error');
      this.startHold(selected); this.desired = selected; this.elapsed = 0; this.readyFor = 0;
      if (selected !== 'none') window.BraceGym?.status('Preparing grip… release the movement keys.', 'preparing');
    }
    this.updateHold(dt);
    if (this.desired === 'none' || this.applied === this.desired) return;
    this.elapsed += dt;
    if (this.elapsed > 2 && this.gripError(this.desired) < .075) this.readyFor += dt;
    else this.readyFor = 0;
    if (this.readyFor > .25) this.apply(this.desired);
    else if (this.elapsed > 12) {
      window.BraceGym?.remove();
      window.BraceGym?.status('Grip did not settle. Reset the robot and try again.', 'error');
    }
  }
  reset() {
    if (this.resolved) { this.apply('none'); this.restoreArms(); }
    this.desired = 'none'; this.elapsed = 0; this.readyFor = 0;
    window.BraceGym?.remove();
  }
  dispose() { if (this.resolved) { this.apply('none'); this.restoreArms(); } this.resolved = false; }
}
export const commands = {GymEquipment};
