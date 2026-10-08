const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs');
(async()=>{
 const b=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const results=[],out=process.env.BRACE_RESPAWN_OUTPUT||'/tmp/brace-respawn-location';fs.mkdirSync(out,{recursive:true});
 try {
  const c=await b.newContext({viewport:{width:1200,height:900},serviceWorkers:process.env.BRACE_TEST_URL?.startsWith('https:')?'allow':'block'});
  await c.route('**/brace-equipment-plugin.js*',async r=>{const res=await r.fetch();await r.fulfill({response:res,body:(await res.text()).replace("this.apply('none'); return true;","window.__BraceTest={m,d:this.context.mjData,gym:this,mujoco:this.context.mujoco}; this.apply('none'); return true;")})});
  const p=await c.newPage(),errors=[];p.on('pageerror',e=>{errors.push(e.message);console.error('Page error:',e.message)});
  p.on('console',m=>{if(m.type()==='error')console.error('Browser:',m.text())});
  await p.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
  await p.waitForFunction(()=>window.BraceGym?.ready&&window.BraceGym.force().time>2,null,{timeout:120000}).catch(async e=>{console.error(await p.evaluate(()=>({ready:window.BraceGym?.ready,force:window.BraceGym?.force?.(),hook:!!window.__BraceTest,body:document.body.innerText.slice(0,300)})));throw e});
  await p.evaluate(()=>{window.__recoveries=[];window.__falls=0;window.addEventListener('brace:auto-respawn',()=>window.__falls++);window.addEventListener('brace:respawned',()=>window.__recoveries.push({root:window.BraceGym.force().root,q:window.BraceGym.force().rootQuat,mocap:Array.from(window.__BraceTest.d.mocap_pos)}));});
  for(const surface of ['flat','slope_ascent','slope_plateau','slope_descent']) {
   const slope=p.getByRole('checkbox',{name:'Slope ahead',exact:true});
   if(surface==='flat')await slope.uncheck();else if(surface==='slope_ascent')await slope.check();
   await p.waitForFunction(()=>window.BraceGym.force().time>2);
   await p.waitForTimeout(300);
   const before=await p.evaluate(surface=>{
    const {m,d,gym,mujoco}=window.__BraceTest;
    let xy=[-4,3],top=0;
    if(surface!=='flat') {
     const bytes=new Uint8Array(m.names),decoder=new TextDecoder();let body=-1;
     for(let i=0;i<m.nbody;i++){let end=m.name_bodyadr[i];while(bytes[end])end++;if(decoder.decode(bytes.subarray(m.name_bodyadr[i],end))===surface)body=i;}
     const id=m.body_mocapid[body],pos=d.mocap_pos.subarray(id*3,id*3+3),q=d.mocap_quat.subarray(id*4,id*4+4),t=m.geom_size[m.body_geomadr[body]*3+2];
     const n=[2*(q[1]*q[3]+q[0]*q[2]),2*(q[2]*q[3]-q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])];
     xy=[pos[0]+n[0]*t,pos[1]+n[1]*t];top=pos[2]+n[2]*t;
    }
    d.qpos.set([...xy,top+.8],0);d.qpos.set([Math.cos(.65),Math.sin(.65),0,0],3);d.qvel.fill(0);
    mujoco.mj_forward(m,d);gym.fallSample=null;gym.respawnPending=false;gym.updateFallGuard();
    return {surface,xy,top,mocap:Array.from(d.mocap_pos),quat:Array.from(d.mocap_quat),count:window.__recoveries.length,falls:window.__falls};
   },surface);
   await p.waitForFunction(n=>window.__recoveries.length>n,before.count,{timeout:30000});
   const restored=await p.evaluate(()=>window.__recoveries.at(-1));
   assert(Math.hypot(restored.root[0]-before.xy[0],restored.root[1]-before.xy[1])<.001);
   assert(restored.root[2]>before.top+.65&&restored.root[2]<before.top+1.1);
   assert.deepEqual(restored.mocap,before.mocap);
   assert(1-2*(restored.q[1]**2+restored.q[2]**2)>.99);
   await p.waitForFunction(()=>window.BraceGym.force().time>5,null,{timeout:45000});
   assert.equal(await p.evaluate(()=>window.__falls),before.falls,'Recovery must not trigger another fall');
   const final=await p.evaluate(()=>window.BraceGym.force());
   results.push({before,restored,afterFiveSeconds:{root:final.root,quat:final.rootQuat},outcome:'success'});
   fs.writeFileSync(`${out}/results.json`,JSON.stringify(results,null,2));
   await p.screenshot({path:`${out}/${surface}.png`});
   console.log('PASS',surface,JSON.stringify(restored.root));
  }
  // An explicit Reset still starts a new episode and permits a fresh ramp placement.
  await p.getByRole('button',{name:'Reset',exact:true}).click();
  await p.waitForFunction(()=>window.BraceGym.force().time<1.5);
  assert.equal(await p.evaluate(()=>window.BraceRespawn),undefined);
  assert.deepEqual(errors,[]);
  console.log('PASS recovery stays at fall XY on all terrain pieces, no reset loop; manual Reset remains separate');
 } finally {await b.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
