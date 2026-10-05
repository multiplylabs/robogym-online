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
