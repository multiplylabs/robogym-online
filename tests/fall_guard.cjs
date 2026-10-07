const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module');
const mod=new Module(`${process.env.NODE_PATH}/fall-test.cjs`);mod.filename=mod.id;mod.paths=Module._nodeModulePaths(process.env.NODE_PATH);mod._compile(require('esbuild').transformSync(fs.readFileSync('assets/brace-equipment-plugin.js','utf8'),{format:'cjs'}).code,mod.filename);
let events=[];global.CustomEvent=class{constructor(type,init={}){this.type=type;this.detail=init.detail}};global.window={BraceGym:{ready:true,remove(){}},dispatchEvent:e=>events.push(e)};
const d={time:2,qpos:new Float64Array([0,0,.8,1,0,0,0]),qvel:new Float64Array(6),xpos:new Float64Array(6)};
const gym=new mod.exports.commands.GymEquipment('test',{catalog:{}},{mjData:d});gym.feet=[0,1];gym.projectiles=[];gym.parkStability=()=>{};gym.stabilityEvent=()=>{};
const sample=(t,angle,height=.8,vz=0)=>{d.time=t;d.qpos[2]=height;d.qpos[3]=Math.cos(angle/2);d.qpos[4]=Math.sin(angle/2);d.qvel[2]=vz;gym.updateFallGuard()};
// Stable slope posture and low-but-static crouches are not falling.
for(let i=0;i<30;i++)sample(2+i*.02,.35,.45);assert.equal(events.length,0);
// A brief tipping impulse can recover without triggering a reset.
sample(3,.72);sample(3.02,.76);sample(3.04,.70);assert.equal(events.length,0);
// Sustained outward tipping predicts collapse while still above the hard-fall angle.
for(let i=0;i<20&&!gym.respawnPending;i++)sample(4+i*.02,.48+i*.04);
assert(gym.respawnPending);assert.equal(events.at(-1).detail.reason,'imminent fall');assert(gym.fallSample.angle<.75);const count=events.length;sample(5,1.5);assert.equal(events.length,count);
// A failed stability trial survives automatic reset, but manual reset clears it.
gym.trial={phase:'testing',fallen:false};gym.respawnPending=false;sample(6,1.3);assert.equal(gym.trial.phase,'failed');gym.reset();assert.equal(gym.trial.phase,'failed');gym.reset();assert.equal(gym.trial,null);
// Startup grace avoids reset loops during spawn initialization.
events=[];sample(.2,1.5,.2,-2);assert.equal(events.length,0);
// Low pelvis with rapid downward velocity is caught before ground impact.
sample(2,0,.46,-1.2);sample(2.14,0,.44,-1.2);assert(gym.respawnPending);assert.equal(events.at(-1).detail.reason,'imminent fall');
console.log('PASS early tipping/downward prediction, transient recovery, slope/crouch tolerance, grace, one-shot respawn and failed-test preservation');
