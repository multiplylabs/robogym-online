const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module'),esbuild=require('esbuild');
const m=new Module(`${process.env.NODE_PATH}/envelope.cjs`);m._compile(esbuild.transformSync(fs.readFileSync('assets/brace-equipment-plugin.js','utf8'),{format:'cjs'}).code,m.filename||m.id);
global.window={BraceSteering:{command:[.4,0,5],motion:{forwardAligned:true}}};
const raw=new Float32Array([1,0,0,0,8,0,0]),original=raw.slice();let terrain=0;const data={time:0,qpos:new Float64Array([0,0,.8,1,0,0,0])};
const envelope=new m.exports.commands.ExertionEnvelope('test',{}, {mjData:data,readOnnxSlot:({command})=>command==='terrain'?new Float32Array([terrain]):raw});
function run(seconds){for(let i=0;i<seconds*50;i++){data.time+=.02;envelope.update(.02)}}
run(2);assert(Math.abs(envelope.getCommand()[4]-3)<1e-5);assert.deepEqual(raw,original);assert.equal(envelope.getCommand()[0],1);
window.BraceSteering.command=[0,.45,0];run(2);assert(Math.abs(envelope.getCommand()[4]-2)<1e-5);
window.BraceSteering.command=[.8,0,0];run(1);assert(envelope.getCommand()[4]<2.01);run(8);assert(envelope.getCommand()[4]>7.99);
terrain=1;run(3);assert(Math.abs(envelope.getCommand()[4]-5)<1e-5);window.BraceSteering.command=[.4,0,3];run(2);assert(Math.abs(envelope.getCommand()[4]-1.5)<1e-5);
raw[4]=-8;run(2);assert(Math.abs(envelope.getCommand()[4]+1)<1e-5);
window.BraceSteering.command=[0,0,0];run(2);assert(Math.abs(envelope.getCommand()[4]+1)<1e-5);
window.BraceSteering.command=[.4,0,3];
data.qpos.set([Math.cos(.24),Math.sin(.24),0,0],3);run(2);assert(Math.abs(envelope.getCommand()[4])<.5);
raw[0]=0;envelope.update(.02);assert.deepEqual(envelope.getCommand(),raw);assert.equal(envelope.scale,1);assert.equal(envelope.holdUntil,0);
const clone=envelope.getStateField('command');clone[4]=100;assert.equal(envelope.getCommand()[4],-8);
console.log('PASS pre-brace force budgets, direction/raw preservation, delayed recovery, terrain/tilt adaptation, compensation bypass and immutable slot snapshots');
