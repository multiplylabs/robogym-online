// Detached projectiles must never change the robot observation tensors.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),Module=require('node:module');
const core=path.resolve(process.env.NODE_PATH,'../src/core');
const code=require('esbuild').buildSync({stdin:{contents:fs.readFileSync('assets/slotReader.ts','utf8'),resolveDir:path.join(core,'onnx'),loader:'ts'},bundle:true,platform:'node',format:'cjs',write:false}).outputFiles[0].text;
const moduleUnderTest=new Module(path.join(process.env.NODE_PATH,'robot-slots.cjs'));moduleUnderTest.paths=Module._nodeModulePaths(process.env.NODE_PATH);moduleUnderTest._compile(code,moduleUnderTest.id);
const {createSlotReader}=moduleUnderTest.exports;
let bytes=[],offsets=[];for(const name of ['world','pelvis','torso','slope_ascent','stability_box','box_child','root','arm','box_free']){offsets.push(bytes.length);bytes.push(...Buffer.from(name),0);}
const model={names:new Uint8Array(bytes),nbody:6,njnt:3,nsite:0,name_bodyadr:offsets.slice(0,6),name_jntadr:offsets.slice(6),name_siteadr:[],body_parentid:[0,0,1,0,0,4],jnt_type:[0,3,0],jnt_bodyid:[1,2,4],jnt_qposadr:[0,7,8],jnt_dofadr:[0,6,7]};
const data={xpos:Float64Array.from({length:18},(_,i)=>i),xquat:Float64Array.from({length:24},(_,i)=>i),qpos:Float64Array.from({length:15},(_,i)=>i),qvel:Float64Array.from({length:13},(_,i)=>i)};
const read=createSlotReader(()=>({mjModel:model,mjData:data}));
assert.deepEqual(Array.from(read({entity:'robot',field:'body_link_pos_w'})),Array.from(data.xpos.subarray(3,12)));
assert.deepEqual(Array.from(read({entity:'robot',field:'body_link_quat_w'})),Array.from(data.xquat.subarray(4,16)));
assert.deepEqual(Array.from(read({entity:'robot',field:'joint_pos'})),[7]);
assert.equal(read({field:'body_link_pos_w'}).length,15);
console.log('PASS robot tensors retain robot and static scenery, excluding detached free bodies and their children');
