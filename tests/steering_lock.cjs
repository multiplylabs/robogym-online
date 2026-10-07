const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module'),esbuild=require('esbuild');
function load(path,loader='js'){const m=new Module(`${process.env.NODE_PATH}/test-${loader}.cjs`);m.filename=m.id;m.paths=Module._nodeModulePaths(process.env.NODE_PATH);m._compile(esbuild.transformSync(fs.readFileSync(path,'utf8'),{loader,format:'cjs'}).code,m.filename);return m.exports;}
global.window=new EventTarget();window.BraceExert={enabled:true};global.CustomEvent=class extends Event{};
const source=Object.create(load('assets/liveMotion.ts','ts').LiveMotionSource.prototype);source.command=[0,0,0];source.pressed=new Set();const messages=[];source.socket={readyState:1,send:s=>messages.push(JSON.parse(s))};source.renderKeys=source.paintKeys=source.setFocused=()=>{};source.attachKeyboard(window);
const raw=new Float32Array(7),Gym=load('assets/brace-equipment-plugin.js').commands.GymEquipment;const gym=new Gym('test',{catalog:{}},{readOnnxSlot:()=>raw});
function key(type,key){const e=new Event(type);e.key=key;e.repeat=false;window.dispatchEvent(e)}
key('keydown','s');assert(source.getCommand()[0]<0);raw[1]=5;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['s']);assert.equal(messages.at(-1).forward,0);key('keydown','s');assert.equal(source.getCommand()[0],0);
key('keydown','w');assert(source.getCommand()[0]>0);raw[1]=-5;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['w']);assert.equal(source.getCommand()[0],0);
key('keydown','q');assert.equal(source.getCommand()[0],0);assert.equal(source.getCommand()[2],20);key('keyup','q');key('keydown','s');assert(source.getCommand()[0]<0);key('keyup','s');
raw.fill(0);raw[2]=5;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['d']);key('keydown','d');assert.equal(source.getCommand()[1],0);key('keydown','a');assert(source.getCommand()[1]>0);
raw[2]=-5;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,['a']);assert.equal(source.getCommand()[1],0);
raw.fill(0);raw[3]=5;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,[]);
raw[1]=5;raw[4]=-5;gym.updateSteeringLock();assert(window.BraceSteering.blockedKeys.includes('w')&&window.BraceSteering.blockedKeys.includes('s'));source.setCommand(1,0,0);assert.equal(source.getCommand()[0],0);
window.BraceExert.enabled=false;gym.updateSteeringLock();assert.deepEqual(window.BraceSteering.blockedKeys,[]);key('keydown','s');assert(source.getCommand()[0]<0);source.detachKeys();
console.log('PASS opposing keyboard and direct commands blocked, held input stopped, Q/E turn retained, lateral/vertical/two-hand directions and clear restore');
