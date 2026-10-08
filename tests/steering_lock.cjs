const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module'),esbuild=require('esbuild');
function load(path,loader='js'){const m=new Module(`${process.env.NODE_PATH}/test-${loader}.cjs`);m.filename=m.id;m.paths=Module._nodeModulePaths(process.env.NODE_PATH);m._compile(esbuild.transformSync(fs.readFileSync(path,'utf8'),{loader,format:'cjs'}).code,m.filename);return m.exports;}
global.window=new EventTarget();window.BraceExert={enabled:true};global.CustomEvent=class extends Event{};
const source=Object.create(load('assets/liveMotion.ts','ts').LiveMotionSource.prototype);source.command=[0,0,0];source.pressed=new Set();const messages=[];source.socket={readyState:1,send:s=>messages.push(JSON.parse(s))};source.renderKeys=source.paintKeys=source.setFocused=()=>{};source.attachKeyboard(window);
const raw=new Float32Array(7);let terrain=0;const Gym=load('assets/brace-equipment-plugin.js').commands.GymEquipment;const gym=new Gym('test',{catalog:{}},{readOnnxSlot:({command})=>command==='terrain'?new Float32Array([terrain]):raw});
function key(type,key){const e=new Event(type);e.key=key;e.repeat=false;window.dispatchEvent(e)}
key('keydown','s');assert(source.getCommand()[0]<0);raw[4]=8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['s']);assert.equal(source.getCommand()[0],0);assert.match(window.BraceSteering.reasons.s,/directly opposes/);
key('keydown','q');assert.deepEqual(source.getCommand(),[.4,0,5]);assert.equal(messages.at(-1).speed_limit,.5);assert.equal(messages.at(-1).movement_profile,'exertion_adaptive');key('keyup','q');
key('keydown','a');assert(source.getCommand()[1]>0);assert.equal(messages.at(-1).speed_limit,.18);key('keyup','a');source.setCommand(.8,.45,20);assert.deepEqual(source.getCommand(),[.8,.8*.364,5]);assert.deepEqual(window.BraceSteering.command,source.getCommand());
terrain=1;gym.updateSteeringLock();source.setCommand(.8,0,20);assert.deepEqual(source.getCommand(),[.8,0,1]);assert.equal(messages.at(-1).movement_profile,'exertion_slope_adaptive');assert.deepEqual(window.BraceSteering.blockedKeys,['s']);
source.setCommand(.8,.45,20);assert.deepEqual(source.getCommand(),[.8,.8*.364,1]);assert.equal(messages.at(-1).speed_limit,.3);assert.equal(window.BraceSteering.blockedKeys.includes('q'),false);
raw.fill(0);raw[4]=-8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['w']);source.setCommand(.4,0,20);assert.deepEqual(source.getCommand(),[-.4,0,1]);assert.equal(messages.at(-1).speed_limit,.18);
terrain=0;gym.updateSteeringLock();source.setCommand(.4,0,20);assert.deepEqual(source.getCommand(),[-.4,0,2]);terrain=1;gym.updateSteeringLock();
raw.fill(0);raw[5]=8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['d']);source.setCommand(.8,0,20);assert(source.getCommand()[0]>0&&source.getCommand()[2]>0);
raw[5]=-8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['a']);
raw.fill(0);raw[6]=8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,[]);source.setCommand(.8,.2,20);assert(source.getCommand()[0]>0&&source.getCommand()[1]>0&&source.getCommand()[2]>0);
raw[6]=-8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,[]);
raw.fill(0);raw[1]=8;raw[4]=-8;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['s','w']);source.setCommand(.4,0,20);assert.deepEqual(source.getCommand(),[0,0,1]);
window.BraceExert.enabled=false;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,[]);source.setCommand(.8,.45,20);assert.deepEqual(source.getCommand(),[.8,.45,20]);assert.equal(messages.at(-1).movement_profile,null);assert.equal(messages.at(-1).speed_limit,null);source.detachKeys();
console.log('PASS only directly opposing keys blocked; turns, sides, vertical/ramp motion, backward arcs, held-key stop, direct commands and clear restoration');
