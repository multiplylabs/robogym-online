// Run with NODE_PATH pointing at mjswan/template/node_modules.
const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module');
const source=fs.readFileSync('assets/handSpringContact.ts','utf8').replace("import { mjcToThreeCoordinate } from '../scene/coordinate';",'');
const code=require('esbuild').transformSync(source,{loader:'ts',format:'cjs'}).code;
const loaded=new Module(`${process.env.NODE_PATH}/contact-test.cjs`);loaded.filename=loaded.id;loaded.paths=Module._nodeModulePaths(process.env.NODE_PATH);loaded._compile(code,loaded.filename);
const {HandSpringContact}=loaded.exports;
global.window={dispatchEvent(){}};global.CustomEvent=class{constructor(type,options){this.detail=options.detail;}};
const bytes=Buffer.from('world\0torso\0left\0right\0');
const model={nbody:4,names:bytes,name_bodyadr:[0,6,12,17]};
const state={endpoint_kv:new Float32Array([100,100]),endpoint_cv:new Float32Array([2,2]),push_axis_w:new Float32Array([1,0,0,1,0,0]),push_axis_local:new Float32Array([1,0,0,1,0,0]),ref_hand_pos:new Float32Array(6),ref_anchor_pos:new Float32Array(3),force_cmd_eff:new Float32Array([5,0,0,5,0,0])};
const contact=new HandSpringContact({command_name:'brace',anchor_body:'torso',targets:[{body:'left',hand:0},{body:'right',hand:1}],max_lead:.15,smooth_beta:120,two_sided:true,damping:true,dt:.02,damper_vel_ema_alpha:.3,gauge:false},model);
const d={xpos:new Float64Array([0,0,0,0,0,0,.05,0,0,.05,0,0]),xquat:new Float64Array([1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0]),xfrc_applied:new Float64Array(24),time:0};
const brace={getStateField:name=>state[name]??null};
function apply(){d.xfrc_applied.fill(0);contact.apply(d,brace);}
const spring=x=>100*.15*Math.tanh(x/.15);
apply();assert(Math.abs(contact.measured[0]-spring(.05))<1e-5);
d.xpos[6]=.06;apply();assert(Math.abs(contact.measured[0]-(spring(.06)+2*.3*(.01/.02)))<1e-5);assert(Math.abs(d.xfrc_applied[12]+contact.measured[0])<1e-5);
console.log('PASS training parity: raw-lead damping, EMA and equal/opposite physical reaction');
state.push_axis_local.fill(0,0,3);apply();assert.equal(contact.measured[0],0);assert.equal(d.xfrc_applied[12],0);
d.xpos[6]=.10;state.push_axis_local[0]=1;apply();assert(Math.abs(contact.measured[0]-spring(.10))<1e-5);
console.log('PASS independently activated hands produce no derivative spike');
contact.reset();d.xpos[6]=-.04;apply();assert(Math.abs(contact.measured[0]-spring(-.04))<1e-5);
console.log('PASS reset and signed two-sided contact');

const externalSource=fs.readFileSync('assets/externalWrench.ts','utf8');
const externalModule=new Module(`${process.env.NODE_PATH}/external-test.cjs`);externalModule.filename=externalModule.id;externalModule.paths=loaded.paths;
externalModule._compile(require('esbuild').transformSync(externalSource,{loader:'ts',format:'cjs'}).code,externalModule.filename);
const external=new externalModule.exports.ExternalWrenchApplier({command_name:'load',targets:[{body:'left',axes:['x','y','z'],torque_axes:['tx','ty','tz']}]},model);
state.push_axis_local.fill(0);d.xfrc_applied.fill(0);d.xfrc_applied[12]=1;d.xfrc_applied[15]=2;
external.apply(d,{getUiValue:name=>({x:10,y:0,z:0,tx:3,ty:0,tz:0}[name])});
contact.apply(d,brace);assert.equal(d.xfrc_applied[12],11);assert.equal(d.xfrc_applied[15],5);assert.equal(contact.measured[0],0);
state.push_axis_local[0]=1;contact.apply(d,brace);assert(Math.abs(d.xfrc_applied[12]-(11-contact.measured[0]))<1e-5);assert.equal(d.xfrc_applied[15],5);
console.log('PASS manual loads, mouse forces and torques survive inactive/active spring contact');

// Different robot/reference headings and origins must not become apparent hand lead.
const rotate=(yaw,v)=>[Math.cos(yaw)*v[0]-Math.sin(yaw)*v[1],Math.sin(yaw)*v[0]+Math.cos(yaw)*v[1],v[2]];
function orientation(yaw,pitch,roll){const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2),cp=Math.cos(pitch/2),sp=Math.sin(pitch/2),cr=Math.cos(roll/2),sr=Math.sin(roll/2);return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]}
for(const yaw of [0,Math.PI/2,Math.PI,-.7])for(const axis of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]){
  const referenceYaw=-.9,anchor=[-12,5,.75],refAnchor=[1.2,-3,.8],handLocal=[.3,.1,-.2];
  state.push_axis_local.set([...axis,...axis]);state.push_axis_w.set([...rotate(referenceYaw,axis),...rotate(referenceYaw,axis)]);
  state.ref_anchor_pos.set(refAnchor);const r=rotate(referenceYaw,handLocal).map((v,i)=>v+refAnchor[i]);state.ref_hand_pos.set([...r,...r]);
  d.xpos.set(anchor,3);d.xquat.set(orientation(yaw,.35,.2),4);
  const hand=rotate(yaw,handLocal.map((v,i)=>v+.04*axis[i])).map((v,i)=>v+anchor[i]);d.xpos.set(hand,6);d.xpos.set(hand,9);
  contact.reset();apply();const measured=contact.measured[0];assert(Math.abs(measured-spring(.04))<1e-4);
  const n=rotate(yaw,axis);for(let i=0;i<3;i++)assert(Math.abs(d.xfrc_applied[12+i]+measured*n[i])<1e-6);
  // Move only the robot's origin, keeping its local pose: no contact/damping change.
  for(const b of [1,2,3])for(let i=0;i<3;i++)d.xpos[b*3+i]+=[9,-6,.4][i];
  apply();assert(Math.abs(contact.measured[0]-measured)<1e-4);
}
console.log('PASS all signed axes rotate with torso yaw; independent reference yaw/world drift and torso tilt preserve local contact');
