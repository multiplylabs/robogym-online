const fs=require('node:fs'),path=require('node:path'),Module=require('node:module'),assert=require('node:assert/strict'),esbuild=require('esbuild');
const template=path.dirname(process.env.NODE_PATH),command=path.join(template,'src/core/command');
const own=path.resolve('assets');
(async()=>{
const built=await esbuild.build({stdin:{contents:fs.readFileSync(path.join(own,'TrackingCommand.ts'),'utf8'),resolveDir:command,loader:'ts'},bundle:true,packages:'external',platform:'node',format:'cjs',write:false,plugins:[{name:'owned-reference-modules',setup(build){build.onResolve({filter:/^\.\/(referenceFrame|respawn|liveMotion)$/},args=>({path:path.join(own,`${args.path.slice(2)}.ts`)}))}}]});
const mod=new Module(path.join(process.env.NODE_PATH,'reference-inputs.cjs'));mod.filename=mod.id;mod.paths=Module._nodeModulePaths(process.env.NODE_PATH);mod._compile(built.outputFiles[0].text,mod.filename);
const tracking=Object.create(mod.exports.TrackingCommand.prototype),offsets=[1,2,3,4,8,12,16,20];
tracking.selectedMotion={body_names:['root','torso'],jointVel:[]};tracking.selectedAnchorBodyIndex=1;tracking.timeSteps=offsets;tracking.refIdx=4;tracking.refLen=32;tracking.nJoints=2;
tracking.refBodyPosW=[];tracking.refBodyQuatW=[];tracking.refRootPos=[];tracking.refRootQuat=[];tracking.refJointPos=[];tracking.refJointVel=[];
for(let f=0;f<32;f++) {
 const pos=new Float32Array([100+.02*f,-20,.8+.001*f,100+.02*f,-20,.9+.001*f]);
 const q=[2*Math.cos(f/200),0,0,2*Math.sin(f/200)];
 tracking.refBodyPosW.push(pos);tracking.refRootPos.push(pos.slice(0,3));tracking.refBodyQuatW.push(new Float32Array([...q,...q]));tracking.refRootQuat.push(new Float32Array(q));
 tracking.refJointPos.push(new Float32Array([f,100+f]));tracking.refJointVel.push(new Float32Array([2*f,200+f]));
}
const positions=tracking.getStateField('ref_body_pos_w'),rotations=tracking.getStateField('ref_body_quat_w'),joints=tracking.getStateField('ref_joint_pos'),velocities=tracking.getStateField('ref_joint_vel'),root=tracking.getStateField('ref_root_pos_w');
assert.equal(positions.length,8*2*3);assert.equal(rotations.length,8*2*4);
for(let i=0;i<8;i++) {
 const f=4+offsets[i];
 assert.deepEqual(Array.from(joints.slice(i*2,i*2+2)),[f,100+f]);
 assert.deepEqual(Array.from(velocities.slice(i*2,i*2+2)),[2*f,200+f]);
 assert(Math.abs(positions[i*6]-.02*(f-5))<1e-5);
 assert(Math.abs(positions[i*6+2]-(.8+.001*f))<1e-6);
 assert(Math.abs(positions[i*6+5]-(.9+.001*f))<1e-6);
 assert.deepEqual(Array.from(root.slice(i*3,i*3+3)),Array.from(positions.slice(i*6,i*6+3)));
 for(let b=0;b<2;b++) {
  const quat=rotations.slice(i*8+b*4,i*8+b*4+4);
  assert(Math.abs(Math.hypot(...quat)-1)<1e-6);
  assert(Math.abs(quat[0]-Math.cos(f/200))<1e-6);
  assert(Math.abs(quat[3]-Math.sin(f/200))<1e-6);
 }
}
// Nearest-future policy state must share the exact same pose as the brace window.
assert.deepEqual(Array.from(positions.slice(0,6)),[...tracking.refBodyPosW[5]].map((v,i)=>i%3===0?v-tracking.refBodyPosW[5][3]:i%3===1?v+20:v));
// At the live frontier, missing lookahead holds a valid last pose instead of zero filling.
tracking.refIdx=30;const held=tracking.getStateField('ref_joint_pos');
for(let i=0;i<8;i++)assert.deepEqual(Array.from(held.slice(i*2,i*2+2)),[31,131]);
console.log('PASS reference horizon offsets, body/joint order, matched position/velocity samples, quaternion normalization, shared XY origin and valid buffer-end holding');

})().catch(e=>{console.error(e);process.exitCode=1});
