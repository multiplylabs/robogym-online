// Verify simulation-time scheduling, contact exposure and sticky fall outcomes.
const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module');
const mod=new Module(`${process.env.NODE_PATH}/burst-test.cjs`);mod.filename=mod.id;mod.paths=Module._nodeModulePaths(process.env.NODE_PATH);
mod._compile(require('esbuild').transformSync(fs.readFileSync('assets/brace-equipment-plugin.js','utf8'),{format:'cjs'}).code,mod.filename);
global.window={BraceGym:{ready:true,remove(){}},dispatchEvent(){}};global.CustomEvent=class{};
const d={time:0,qpos:new Float64Array(42),qvel:new Float64Array(36),xpos:new Float64Array(12),ncon:0,contact:{get:i=>contacts[i]}};
d.qpos[2]=.8;d.qpos[3]=1;d.xpos[5]=.8;
const m={body_mass:new Float64Array(9).fill(.75),body_inertia:new Float64Array(27).fill(.00432),geom_size:new Float64Array(18),geom_rbound:new Float64Array(6),geom_aabb:new Float64Array(36),geom_contype:new Uint8Array(6),geom_conaffinity:new Uint8Array(6),body_contype:new Uint8Array(9),body_conaffinity:new Uint8Array(9),geom_bodyid:[4,5,6,7,8,1],opt:{gravity:[0,0,-9.81]}};
const gym=new mod.exports.commands.GymEquipment('test',{catalog:{}},{mjData:d,mjModel:m,mujoco:{mj_forward(){},mj_setConst(){}},bodies:Array.from({length:9},()=>({visible:false}))});
gym.projectiles=Array.from({length:5},(_,i)=>({body:4+i,geom:i,qadr:7+i*7,vadr:6+i*6}));gym.torso=1;gym.feet=[2,3];gym.robotBodies=new Set([1,2,3]);
window.BraceStability={get state(){return {...gym.trial}}};let contacts=[];
assert(gym.launchStability());assert.deepEqual(gym.trial.massRange,[.35,1]);assert(!gym.launchStability());
for(const t of [.19,.2,.4,.6,.8]){d.time=t;gym.updateStability();}
assert.deepEqual(gym.trial.launchTimes,[0,.2,.4,.6,.8]);assert.equal(gym.trial.launched,5);
contacts=Array.from({length:5},(_,i)=>({geom1:i,geom2:5,efc_address:0}));d.ncon=5;d.time=1;gym.updateStability();assert.equal(gym.trial.hitCount,5);
d.time=6;gym.updateStability();assert.equal(gym.trial.phase,'recovered');assert(m.geom_contype.slice(0,5).every(v=>v===0));
d.time=10;gym.launchStability();d.qpos[2]=.2;gym.updateStability();d.qpos[2]=.8;d.time=16;gym.updateStability();assert.equal(gym.trial.phase,'failed');
d.time=20;contacts=[contacts[0]];d.ncon=1;gym.launchStability();d.time=26;gym.updateStability();assert.equal(gym.trial.phase,'partial');
d.time=28;window.BraceExert={enabled:true};gym.launchStability();assert.deepEqual(gym.trial.massRange,[.15,.4]);
for(const [i,p] of gym.projectiles.entries()){const shot=gym.trial.shots[i];assert(shot.mass>=.15&&shot.mass<=.4);assert(shot.radius>=.07&&shot.radius<=.14);assert.equal(m.body_mass[p.body],shot.mass);assert.equal(m.geom_size[p.geom*3],shot.radius);assert(Math.abs(m.body_inertia[p.body*3]-.4*shot.mass*shot.radius**2)<1e-9);}
assert(new Set(gym.trial.shots.map(s=>s.mass)).size===5);assert(new Set(gym.trial.shots.map(s=>s.radius)).size===5);
gym.resolved=false;gym.reset();d.time=30;gym.updateStability();assert.equal(gym.trial,null);assert(m.geom_contype.slice(0,5).every(v=>v===0));
window.BraceExert.enabled=false;gym.launchStability();assert.deepEqual(gym.trial.massRange,[.35,1]);for(const p of gym.projectiles)assert(m.body_mass[p.body]>=.35&&m.body_mass[p.body]<=1);
console.log('PASS varied physical radii/masses and lighter exertion range; five timed balls, hit counts, partial exposure, sticky falls and reset cancellation');
