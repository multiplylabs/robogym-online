// Run sequentially for comparable timing; ONNX browser sessions have independent state.
const {chromium}=require('playwright'),fs=require('node:fs');
const cases=JSON.parse(fs.readFileSync(process.env.BRACE_SLOPE_CASES_FILE,'utf8'));
const out=process.env.BRACE_SLOPE_OUTPUT||'/tmp/brace-downhill';fs.mkdirSync(out,{recursive:true});
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']}),results=[];
 try {
  for(const config of cases) {
   const c=await browser.newContext({viewport:{width:1100,height:900},serviceWorkers:process.env.BRACE_TEST_URL?.startsWith('https:')?'allow':'block'}),row={...config,outcome:'incomplete',samples:[],errors:[]};
   try {
    await c.addInitScript(cap=>{window.__falls=[];window.__commands=[];window.addEventListener('brace:auto-respawn',e=>window.__falls.push({...e.detail,force:window.BraceGym.force()}));const W=window.WebSocket;window.WebSocket=class extends W{send(s){try{const v=JSON.parse(s);if(v.type==='command'){if(cap&&v.forward>0)v.speed_limit=cap;window.__commands.push(v);s=JSON.stringify(v)}}catch{}super.send(s)}}},config.speedCap||null);
    await c.route('**/brace-equipment-plugin.js*',async r=>{const res=await r.fetch();await r.fulfill({response:res,body:(await res.text()).replace("this.apply('none'); return true;","window.__SlopeTest=this; this.apply('none'); return true;")})});
    const page=await c.newPage();page.on('pageerror',e=>row.errors.push(e.message));
    await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
    await page.waitForFunction(()=>window.BraceGym?.ready&&window.__SlopeTest,null,{timeout:120000});
    await page.locator(`.brace-style[aria-label="${config.style||'Slow walk'}"]`).click();
    await page.evaluate(angle=>{const rng=window.__SlopeTest.context.rng,original=rng.uniform.bind(rng);rng.uniform=(lo,hi)=>lo===6&&hi===10?angle:original(lo,hi)},config.angle);
    await page.getByRole('checkbox',{name:'Slope ahead',exact:true}).check();
    if(config.force) {
     await page.getByRole('button',{name:'Exert force',exact:true}).click();
     await page.getByRole('combobox',{name:'Force hand',exact:true}).selectOption(config.hand||'right');
     await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption(config.direction||'X:1');
     await page.getByRole('button',{name:`Apply ${config.force} newtons`,exact:true}).click();
    }
    let t=await page.evaluate(()=>window.BraceGym.force().time);
    await page.waitForFunction(t=>window.BraceGym.force().time>=t+2||window.__falls.length,t,{timeout:30000});
    const course=await page.evaluate(()=>{
     const {mjModel:m,mjData:d}=window.__SlopeTest.context,bytes=new Uint8Array(m.names),decoder=new TextDecoder(),pieces={};
     for(let b=0;b<m.nbody;b++){let end=m.name_bodyadr[b];while(bytes[end])end++;const name=decoder.decode(bytes.subarray(m.name_bodyadr[b],end));if(name.startsWith('slope_')){const id=m.body_mocapid[b],g=m.body_geomadr[b];pieces[name]={p:Array.from(d.mocap_pos.subarray(id*3,id*3+3)),q:Array.from(d.mocap_quat.subarray(id*4,id*4+4)),size:Array.from(m.geom_size.subarray(g*3,g*3+3))}}}
     const a=pieces.slope_ascent,[w,x,y,z]=a.q,R=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)];
     const norm=Math.hypot(R[0],R[1]),f=[R[0]/norm,R[1]/norm],top=[a.p[0]+R[2]*a.size[2],a.p[1]+R[3]*a.size[2]],toe=top.map((v,i)=>v-f[i]*a.size[0]*norm);
     return {pieces,forward:f,toe,run:2*a.size[0]*norm,plateau:2*pieces.slope_plateau.size[0],width:a.size[1]};
    });row.course=course;
    const initial=await page.evaluate(()=>window.BraceGym.force());t=initial.time;row.start=t;row.startRoot=initial.root;
    await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keydown',{key:'w'})));
    const wall=Date.now();let exit=null;
    while(Date.now()-wall<210000) {
     await page.waitForTimeout(200);
     const s=await page.evaluate(()=>{
      const g=window.__SlopeTest,read=field=>Array.from(g.context.readOnnxSlot({command:'motion',field})||[]),p=read('ref_body_pos_w'),q=read('ref_body_quat_w');
      const goal=Array.from(g.context.readOnnxSlot({command:'brace',field:'x_priv_bodies'})||[]);
      return {force:window.BraceGym.force(),reference:window.BraceReference,falls:window.__falls.length,blockedKeys:window.BraceSteering?.blockedKeys||[],policyRoot:p.slice(0,3),policyTorso:p.slice(16*3,17*3),policyFirstQuat:q.slice(0,4),referenceWindowLength:p.length,braceTorso:Array.from(g.context.readOnnxSlot({command:'brace',field:'ref_anchor_pos'})||[]),goalReferenceDifference:goal.length?Math.max(...goal.map((v,i)=>Math.abs(v-p[i]))):null};
     });
     const dx=s.force.root[0]-course.toe[0],dy=s.force.root[1]-course.toe[1];s.along=dx*course.forward[0]+dy*course.forward[1];s.side=-dx*course.forward[1]+dy*course.forward[0];
     const end=course.run*2+course.plateau,rise=2*course.pieces.slope_ascent.size[0]*Math.sqrt(1-(course.run/(2*course.pieces.slope_ascent.size[0]))**2);
     const h=Math.abs(s.side)>course.width||s.along<0||s.along>end?0:s.along<course.run?rise*s.along/course.run:s.along<course.run+course.plateau?rise:rise*(end-s.along)/course.run;
     s.clearance=s.force.root[2]-h;s.tilt=Math.acos(Math.max(-1,Math.min(1,1-2*(s.force.rootQuat[1]**2+s.force.rootQuat[2]**2))))*180/Math.PI;
     s.segment=s.along<0?'approach':s.along<course.run?'ascent':s.along<course.run+course.plateau?'plateau':s.along<end?'descent':'exit';row.samples.push(s);
     if(s.falls){row.outcome='failure';break;}
     if(config.expectBlocked&&s.force.time>=t+(config.seconds||20)){row.outcome=s.blockedKeys.includes('w')?'blocked':'failure';break;}
     if(Math.abs(s.side)>course.width-.15&&s.along>0&&s.along<end){row.outcome='left_course';break;}
     if(s.along>end+.5&&exit===null){exit=s.force.time;await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keyup',{key:'w'})));}
     if(exit!==null&&s.force.time>=exit+3){row.outcome='success';break;}
     if(s.force.time>=t+(config.seconds||65)){row.outcome='timeout';break;}
    }
    await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keyup',{key:'w'})));
    Object.assign(row,await page.evaluate(()=>({resets:window.__falls,commands:window.__commands})));
    row.duration=(row.resets[0]?.force.time??row.samples.at(-1)?.force.time)-t;row.minClearance=Math.min(...row.samples.map(s=>s.clearance));row.maxTilt=Math.max(...row.samples.map(s=>s.tilt));
    row.maxProgress=Math.max(...row.samples.map(s=>s.along));row.failureSegment=row.outcome==='failure'?row.samples.at(-1)?.segment:null;
    row.requestedForcePreserved=row.samples.every(s=>s.falls||s.force.raw.slice(1).every((v,i)=>Math.abs(v-initial.raw[i+1])<.01));
    row.maxDisplacement=Math.max(...row.samples.map(s=>Math.hypot(s.force.root[0]-initial.root[0],s.force.root[1]-initial.root[1])));
    if(config.expectBlocked&&row.outcome==='blocked'&&(row.commands.some(v=>v.forward||v.lateral||v.turn)||row.maxDisplacement>.35))row.outcome='failure';
    await page.screenshot({path:`${out}/${config.id}.png`});
   }catch(e){row.outcome='error';row.error=String(e)}finally{results.push(row);fs.writeFileSync(`${out}/results.json`,JSON.stringify(results,null,2));console.log(JSON.stringify({...row,samples:undefined}));await c.close()}
  }
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
