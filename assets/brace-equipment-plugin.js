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
    window.BraceGym.force = () => {
      const read = field => Array.from(this.context.readOnnxSlot?.({command:'brace',field}) ?? []);
      const ascent=bn.indexOf('slope_ascent'), mocap=ascent < 0 ? -1 : m.body_mocapid[ascent];
      const rampQuat=mocap < 0 ? null : this.context.mjData.mocap_quat.subarray(mocap*4,mocap*4+4);
      return {rampAngleDeg:rampQuat ? Math.abs(2*Math.atan2(rampQuat[2],rampQuat[0])*180/Math.PI) : null, time:Number(this.context.mjData.time), raw:Array.from(this.context.readOnnxSlot?.({command:'exert',field:'command'}) ?? []), commanded:read('force_cmd_eff'), kv:read('endpoint_kv'), cv:read('endpoint_cv'), axis:read('push_axis_local'), refHand:read('ref_hand_pos'), refAnchor:read('ref_anchor_pos'), anchorDelta:read('xpriv_anchor_pos_delta'), reaction:this.hands.map(b => Array.from(this.context.mjData.xfrc_applied.subarray(b*6,b*6+3))), root:Array.from(this.context.mjData.qpos.subarray(0,3)), rootQuat:Array.from(this.context.mjData.qpos.subarray(3,7))};
    };
    const surface = this.context.bodies[this.torso]?.children.find(o => o.isMesh && o.material?.color);
    if (surface) this.context.scene.background = surface.material.color.clone().setHex(0xf0f6fc);
    const floorId = gn.indexOf('floor');
    const floor = this.context.bodies[m.geom_bodyid[floorId]]?.children.find(o => o.isMesh && o.material?.emissive);
    if (floor) {
      floor.material.toneMapped = false;
      // Keep the white floor bright without clipping its blue seams and shadows.
      floor.material.color.setRGB(.94,.965,.99);
      floor.material.emissive.setRGB(0,0,0);
      floor.material.needsUpdate = true;
    }
    this.setupStability(bn, gn, joints);
    this.apply('none'); return true;
  }
  setupStability(bn, gn, joints) {
    const m = this.context.mjModel;
    this.projectiles = bn.flatMap((name, body) => {
      if (!/^stability_ball_\d+$/.test(name)) return [];
      const joint=joints.indexOf(`${name}_free`), geom=gn.indexOf(name);
      return [{body,geom,qadr:m.jnt_qposadr[joint],vadr:m.jnt_dofadr[joint]}];
    });
    if (this.projectiles.length!==5) throw new Error('Stability scene requires five physical balls.');
    this.feet=['left_ankle_roll_link','right_ankle_roll_link'].map(n=>bn.indexOf(n));
    this.robotBodies = new Set(bn.flatMap((n,i) => i > 0 && !n.startsWith('stability_') && !n.startsWith('slope_') ? [i] : []));
    window.BraceStability = {launch:() => this.launchStability(), get state() { return this.read(); }, read:() => this.trial ? {...this.trial,hitIds:undefined} : {phase:'ready'}};
    this.parkStability();
  }
  parkStability() {
    const {mjData:d,mjModel:m} = this.context;
    for (const p of this.projectiles ?? []) {
      m.geom_contype[p.geom]=0; m.geom_conaffinity[p.geom]=0; m.body_contype[p.body]=0; m.body_conaffinity[p.body]=0;
      d.qpos.set([0,0,-30,1,0,0,0],p.qadr); d.qvel.fill(0,p.vadr,p.vadr+6);
      this.context.bodies[p.body].visible = false;
    }
  }
  launchStability() {
    if (this.trial?.phase === 'testing' || !window.BraceGym?.ready) return false;
    this.parkStability();
    const exert=Boolean(window.BraceExert?.enabled);
    const massRange=exert ? [.15,.40] : [.35,1.0], radiusRange=[.07,.14];
    const {mjModel:m,mjData:d,mujoco}=this.context;
    const shots=this.projectiles.map((p,i)=>{
      // Stratified ranges give every burst a mixture, rather than five similar random draws.
      const mass=massRange[0]+(massRange[1]-massRange[0])*(i+Math.random())/5;
      const radius=radiusRange[0]+(radiusRange[1]-radiusRange[0])*((i*3%5)+Math.random())/5;
      m.body_mass[p.body]=mass;
      for (let axis=0;axis<3;axis++) m.body_inertia[p.body*3+axis]=.4*mass*radius*radius;
      m.geom_size[p.geom*3]=radius; m.geom_rbound[p.geom]=radius;
      if (m.geom_aabb) for(let axis=3;axis<6;axis++) m.geom_aabb[p.geom*6+axis]=radius;
      this.context.bodies[p.body].children?.forEach(mesh=>mesh.scale.setScalar(radius/.12));
      return {mass,radius};
    });
    // Recompute MuJoCo constants without disturbing the robot's live pose or velocity.
    const qpos=new Float64Array(d.qpos),qvel=new Float64Array(d.qvel);
    mujoco.mj_setConst(m,d); d.qpos.set(qpos); d.qvel.set(qvel); mujoco.mj_forward(m,d);
    this.trial={phase:'testing',shape:'sphere',shots,massRange,radiusRange,speed:4,
      count:this.projectiles.length,launchDuration:1,interval:.2,observationSeconds:5,
      angle:Math.random()*2*Math.PI,start:Number(this.context.mjData.time),
      launched:0,hitCount:0,hit:false,fallen:false,hitIds:[],launchTimes:[]};
    this.fireStabilityBall(); return true;
  }
  fireStabilityBall() {
    const {mjData:d,mjModel:m,mujoco}=this.context, trial=this.trial;
    const p=this.projectiles[trial.launched];
    // One launcher direction per burst, with a small spread; every shot aims at the current torso.
    const angle=trial.angle+(Math.random()-.5)*.20;
    const distance=1.6, speed=trial.speed, flight=distance/speed;
    const target=Array.from(d.xpos.subarray(this.torso*3,this.torso*3+3)); target[2]+=.13;
    const direction=[Math.cos(angle),Math.sin(angle)];
    const position=[target[0]+distance*direction[0],target[1]+distance*direction[1],target[2]];
    const velocity=[-speed*direction[0]+d.qvel[0],-speed*direction[1]+d.qvel[1],-m.opt.gravity[2]*flight/2];
    m.geom_contype[p.geom]=1; m.geom_conaffinity[p.geom]=1; m.body_contype[p.body]=1; m.body_conaffinity[p.body]=1;
    d.qpos.set([...position,1,0,0,0],p.qadr); d.qvel.set([...velocity,0,0,0],p.vadr);
    mujoco.mj_forward(m,d); this.context.bodies[p.body].visible=true;
    trial.launched++; trial.launchTimes.push(Number(d.time)-trial.start); this.stabilityEvent();
  }
  stabilityEvent() {
    window.dispatchEvent(new CustomEvent('brace:stability',{detail:window.BraceStability.state}));
  }
  updateStability() {
    const trial=this.trial;
    if (!trial || trial.phase !== 'testing') { this.parkStability(); return; }
    const {mjModel:m,mjData:d}=this.context;
    const elapsed=Number(d.time)-trial.start;
    // Time comes from MuJoCo, not setTimeout: rendering stalls cannot lengthen the burst.
    while (trial.launched<trial.count && elapsed+1e-7>=trial.launched*trial.interval) this.fireStabilityBall();
    const previousHits=trial.hitCount;
    for (let i=0;i<d.ncon;i++) {
      const c=d.contact.get(i); if (!c || c.efc_address<0) continue;
      const g1=c.geom1,g2=c.geom2;
      const shot=this.projectiles.slice(0,trial.launched).findIndex(p=>
        (g1===p.geom && this.robotBodies.has(m.geom_bodyid[g2])) ||
        (g2===p.geom && this.robotBodies.has(m.geom_bodyid[g1])));
      if (shot>=0 && !trial.hitIds.includes(shot)) trial.hitIds.push(shot);
    }
    trial.hitCount=trial.hitIds.length; trial.hit=trial.hitCount>0;
    const q=d.qpos.subarray(3,7), tilt=1-2*(q[1]*q[1]+q[2]*q[2]);
    const ground=Math.min(...this.feet.map(b=>d.xpos[b*3+2]));
    if (d.qpos[2]-ground<.38 || tilt<.5) trial.fallen=true;
    if (elapsed>=trial.launchDuration+trial.observationSeconds) {
      trial.phase=trial.fallen ? 'failed' : trial.hitCount===trial.count ? 'recovered' : trial.hit ? 'partial' : 'missed';
      this.parkStability(); this.stabilityEvent();
    } else if (previousHits!==trial.hitCount) this.stabilityEvent();
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
    this.holdName = name; this.holdElapsed = 0; this.forceOffset = this.arms.map(() => 0); this.forceFiltered = [0,0];
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
    if (this.holdName === "exertion" && t === 1) this.trackForce(dt);
    this.arms.forEach((a,i) => {
      const target = this.holdStart[i]*(1-s)+this.holdTarget[i]*s + (this.forceOffset?.[i] ?? 0)*s, adr=a.actuator*10;
      m.actuator_gainprm[adr]=1-w; m.actuator_biasprm[adr]=w*a.kp*target;
      m.actuator_biasprm[adr+1]=-w*a.kp; m.actuator_biasprm[adr+2]=-w*a.kd;
    });
    if (goalWeight === 0 && t === 1) this.restoreArms();
  }
  // Integrate a task-space force error into bounded joint targets; contact forces stay physical.
  trackForce(dt) {
    const {mjModel:m,mjData:d} = this.context;
    const read = field => this.context.readOnnxSlot?.({command:'brace',field});
    const feedback=this.catalog.exertion.feedback;
    const command = read('force_cmd_eff'), axis = read('push_axis_local'), kv = read('endpoint_kv');
    if (!command || !axis || !kv) return;
    const q=d.xquat.subarray(this.torso*4,this.torso*4+4);
    const yaw=Math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    const c=Math.cos(yaw),sn=Math.sin(yaw);
    for (let h=0;h<2;h++) {
      const magnitude=Math.hypot(...command.subarray(h*3,h*3+3));
      if (magnitude < .01) {
        for (let k=0;k<7;k++) this.forceOffset[h*7+k] *= Math.exp(-dt/.6);
        this.forceFiltered[h]=0; continue;
      }
      const n=[c*axis[h*3]-sn*axis[h*3+1],sn*axis[h*3]+c*axis[h*3+1],axis[h*3+2]];
      const b=this.hands[h], p=d.xpos.subarray(b*3,b*3+3);
      const measured=window.BraceExert?.reading?.measured[h] ?? 0;
      const blend=1-Math.exp(-dt/feedback.filter_tau);
      this.forceFiltered[h]+=blend*(measured-this.forceFiltered[h]);
      const gradients=this.arms.slice(h*7,h*7+7).map(a=>{
        const axis=d.xaxis.subarray(a.joint*3,a.joint*3+3),anchor=d.xanchor.subarray(a.joint*3,a.joint*3+3);
        const r=Array.from(p,(v,k)=>v-anchor[k]);
        return n[0]*(axis[1]*r[2]-axis[2]*r[1])+n[1]*(axis[2]*r[0]-axis[0]*r[2])+n[2]*(axis[0]*r[1]-axis[1]*r[0]);
      });
      const norm=gradients.reduce((v,j)=>v+j*j,0)+feedback.regularization;
      const error=(magnitude-this.forceFiltered[h])/Math.max(40,kv[h]);
      gradients.forEach((j,k)=>{
        const i=h*7+k,a=this.arms[i],delta=Math.max(-feedback.joint_rate*dt,Math.min(feedback.joint_rate*dt,feedback.gain*dt*error*j/norm));
        const lo=Math.max(-feedback.joint_offset,m.jnt_range[a.joint*2]+.01-this.holdTarget[i]);
        const hi=Math.min(feedback.joint_offset,m.jnt_range[a.joint*2+1]-.01-this.holdTarget[i]);
        this.forceOffset[i]=Math.max(lo,Math.min(hi,this.forceOffset[i]+delta));
      });
    }
  }
  updateSteeringLock() {
    const raw=Array.from(this.context.readOnnxSlot?.({command:'exert',field:'command'}) ?? []);
    const force=raw.length===7 ? raw.slice(1) : raw;
    const blocked=[], reasons={};
    const block=(key,reason)=>{if(!blocked.includes(key)) blocked.push(key); reasons[key] ??= reason;};
    const active=Boolean(window.BraceExert?.enabled && force.some(v=>Math.abs(v)>.05));
    if(window.BraceExert?.enabled) {
      if([force[0],force[3]].some(v=>v>.05)) block('s','Backward movement opposes the forward hand push. This direction is not physically plausible for the current push in this demo.');
      if([force[0],force[3]].some(v=>v<-.05)) block('w','Forward movement opposes the backward hand push. This direction is not physically plausible for the current push in this demo.');
      if([force[1],force[4]].some(v=>v>.05)) block('d','Rightward movement opposes the leftward hand push. This direction is not physically plausible for the current push in this demo.');
      if([force[1],force[4]].some(v=>v<-.05)) block('a','Leftward movement opposes the rightward hand push. This direction is not physically plausible for the current push in this demo.');
      // Lateral pushes permit only slow aligned side steps; crossing travel and turns failed audits.
      if([force[1],force[4]].some(v=>Math.abs(v)>.05)) {
        for(const key of ['w','s','q','e']) block(key,'Crossing or turning while pushing sideways caused falls in testing. Only aligned side steps are supported while this push is active.');
      }
    }
    this.setSteeringLock(blocked, active ? {turnLimit:6,offAxisSpeed:.18,turnSpeed:.30,diagonalSpeed:.30,crossSpeed:.18,diagonalRatio:.364,forwardAligned:[force[0],force[3]].some(v=>v>.05) && [force[1],force[2],force[4],force[5]].every(v=>Math.abs(v ?? 0)<=.05)} : null, reasons);
  }
  setSteeringLock(blocked, motion=null, reasons={}) {
    if(JSON.stringify(window.BraceSteering?.blockedKeys ?? [])===JSON.stringify(blocked) && JSON.stringify(window.BraceSteering?.motion ?? null)===JSON.stringify(motion) && JSON.stringify(window.BraceSteering?.reasons ?? {})===JSON.stringify(reasons)) return;
    window.BraceSteering={blockedKeys:blocked,motion,reasons};
    window.dispatchEvent(new CustomEvent('brace:steering-lock'));
  }
  updateFallGuard() {
    const d=this.context.mjData, time=Number(d.time);
    if(this.respawnPending || !window.BraceGym?.ready || time<.75) return;
    const q=d.qpos.subarray(3,7), up=1-2*(q[1]*q[1]+q[2]*q[2]);
    const angle=Math.acos(Math.max(-1,Math.min(1,up)));
    const clearance=d.qpos[2]-Math.min(...this.feet.map(b=>d.xpos[b*3+2]));
    const previous=this.fallSample, dt=previous ? time-previous.time : 0;
    const tipping=dt>0 && dt<.1 ? (angle-previous.angle)/dt : 0;
    this.fallSample={time,angle};
    const hard=!Number.isFinite(clearance+angle) || clearance<.40 || up<.65;
    const predicted=(angle>.44 && tipping>.6 && angle+tipping*.35>.85) ||
      (clearance<.60 && d.qvel[2]<-.6 && clearance+d.qvel[2]*.30<.35);
    if(!hard && !predicted) {this.fallRiskSince=null;return;}
    this.fallRiskSince ??= time;
    if(!hard && time-this.fallRiskSince<.06) return;
    this.respawnPending=true;
    const reason=hard ? 'fall' : 'imminent fall';
    this.respawnReason=reason;
    if(this.trial?.phase==='testing') {
      this.trial.fallen=true;this.trial.phase='failed';this.trial.reason=reason;
      this.parkStability();this.stabilityEvent();
    }
    window.dispatchEvent(new CustomEvent('brace:auto-respawn',{detail:{reason,testFailed:this.trial?.phase==='failed'}}));
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
  referenceView() {
    const settings = {...window.BraceReference, mode:'body'};
    const actual = this.context.bodies[this.torso];
    if (!actual) return;
    const roots = (this.context.mujocoRoot ?? this.context.scene).children;
    const reference = roots.find(o => o.name === 'Tracking Ghost');
    const braced = roots.find(o => o.name === 'brace ghost');
    const anchorName = actual.name;
    const refAnchor = reference?.children.find(o => o.name === anchorName);
    if (!refAnchor) return;
    reference.position.set(0,0,0);
    // Follow horizontal travel only; the raw reference keeps its flat-ground height.
    reference.position.copy(actual.position).sub(refAnchor.position);
    reference.position.y = 0;
    if (braced) {
      braced.position.set(0,0,0);
      const anchor = this.context.readOnnxSlot?.({command:'brace',field:'ref_anchor_pos'});
      if (anchor) braced.position.set(actual.position.x-anchor[0],0,actual.position.z+anchor[1]);
    }
    const positions = this.context.readOnnxSlot?.({command:'motion',field:'ref_body_pos_w'});
    window.BraceReference = {...settings, sourceAnchor:refAnchor.position.toArray(), displayAnchor:refAnchor.position.clone().add(reference.position).toArray(), robotAnchor:actual.position.toArray(), policyAnchorXY:positions ? Array.from(positions.subarray((this.torso-1)*3,(this.torso-1)*3+2)) : []};
  }
  update(dt) {
    if (!this.resolved && !this.resolve()) return;
    this.updateSteeringLock();
    this.updateFallGuard();
    if(this.respawnPending) return;
    this.updateStability();
    this.referenceView();
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
      this.desired = selected; this.elapsed = 0; this.readyFor = 0;
      if (selected !== 'none') window.BraceGym?.status('Preparing grip… release the movement keys.', 'preparing');
    }
    const hold = selected !== 'none' ? selected : window.BraceExert?.enabled ? 'exertion' : 'none';
    if (hold !== this.holdName) this.startHold(hold);
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
    const autoReason=this.respawnPending ? this.respawnReason : null;
    const failed=this.respawnPending && this.trial?.phase==='failed' ? this.trial : null;
    if(failed) failed.respawned=true;
    this.trial = failed; this.parkStability?.();
    this.respawnPending=false;this.fallSample=null;this.fallRiskSince=null;
    this.setSteeringLock([]);
    if (this.resolved) { this.apply('none'); this.restoreArms(); }
    this.desired = 'none'; this.elapsed = 0; this.readyFor = 0;
    if (window.BraceGym) window.BraceGym.lastContext=undefined;
    window.BraceGym?.remove();
    if (window.BraceExert) window.BraceExert.enabled=false;
    window.dispatchEvent(new CustomEvent('brace:gym-reset'));
    if(autoReason) window.dispatchEvent(new CustomEvent('brace:respawned',{detail:{reason:autoReason}}));
    if(failed) this.stabilityEvent();
  }
  dispose() { if (this.resolved) { this.apply('none'); this.restoreArms(); } this.resolved = false; }
}
export const commands = {GymEquipment};
