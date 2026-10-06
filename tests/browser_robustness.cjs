// Public-demo audit. Run serially: the MotionBricks server supports one client.
const {chromium}=require('playwright');
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const {execFileSync}=require('node:child_process');
const URL=process.env.BRACE_TEST_URL||'https://multiplylabs.github.io/robogym-online/';
const OUT=path.resolve(process.env.BRACE_AUDIT_DIR||'reports/robustness');
const REPEATS=Number(process.env.BRACE_REPEATS||3);
fs.mkdirSync(OUT,{recursive:true});for(const dir of ['videos','screenshots','traces'])fs.mkdirSync(path.join(OUT,dir),{recursive:true});
const cases=[];
for(const style of ['Stealth','Slow walk','Object carrying','Careful'])for(const terrain of ['flat','slope'])cases.push({id:`gait-${style.toLowerCase().replaceAll(' ','-')}-${terrain}`,category:'Gait',style,terrain});
for(const payload of ['dumbbells','barbell','kettlebell'])for(const terrain of ['flat','slope'])cases.push({id:`carry-${payload}-${terrain}`,category:'Carrying',style:'Stealth',terrain,payload});
for(const value of [2,5,8])for(const terrain of ['flat','slope'])cases.push({id:`exert-right-forward-${value}N-${terrain}`,category:'Exertion',style:'Stealth',terrain,force:{mode:'exert',hand:'right',direction:'X:1',value}});
for(const [hand,direction,label] of [['left','X:1','left-forward'],['both','X:1','both-forward'],['right','Y:1','right-leftward'],['right','Z:-1','right-downward']])cases.push({id:`exert-${label}-5N-flat`,category:'Exertion',style:'Stealth',terrain:'flat',force:{mode:'exert',hand,direction,value:5}});
for(const [hand,direction,value,label] of [['right','X:1',10,'right-forward-10N'],['both','Z:-1',20,'both-downward-20N']])cases.push({id:`compensate-${label}`,category:'Compensation',style:'Stealth',terrain:'flat',force:{mode:'compensate',hand,direction,value}});
for(const [motion,label] of [['backward','backward'],['left','left-strafe'],['turn','forward-turn']])cases.push({id:`steer-${label}`,category:'Steering',style:'Stealth',terrain:'flat',motion});
for(const transition of ['stop-start-style-switch','equip-while-walking','stop-then-push'])cases.push({id:`transition-${transition}`,category:'Transition',style:'Stealth',terrain:'flat',transition});
if(process.env.BRACE_FOLLOWUPS==='1'){
 cases.length=0;
 for(const [settleSeconds,rampForce] of [[3,false],[5,false],[3,true]])cases.push({id:`followup-settle-${settleSeconds}s-${rampForce?'2-to-5N':'5N'}-push`,category:'Transition follow-up',style:'Stealth',terrain:'flat',transition:'settled-push',settleSeconds,rampForce});
}
const selected=process.env.BRACE_CASES ? cases.filter(c=>process.env.BRACE_CASES.split(',').includes(c.id)) : cases;
let report={startedAt:new Date().toISOString(),url:URL,commit:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),repeats:REPEATS,caseDefinitions:selected,trials:[],resourceHashes:{},method:{instrumentation:'Read-only exposure of MuJoCo model/data in the published equipment plugin; no policy, generator, physics, or control parameters modified.',fall:'Pelvis clearance above local terrain <0.45 m, pelvis tilt >60 degrees, or torso/head/pelvis ground contact for >=0.10 simulated seconds. Non-finite physics is failure. Fall status is sticky and never cleared by recovery.',sampling:'Fall checks on every hand-spring control event; traces saved every 0.20 simulated seconds.',sessions:'Fresh browser page and generator session per trial. Generator RNG continues between sessions. Native ramp placement is redrawn 1, 2, or 3 times across repetitions, using the public 6–10 degree setting.',success:'No fall from ready-state configuration through scenario completion; intended commands/payload must be active. Slope runs must actually reach and descend the course. Survival is recorded separately from task completion.',durations:'Flat: 15 simulated seconds plus preparation. Slope: until crossing+2 seconds, at most 45 s (60 s for Slow walk). Transitions: 18 s plus preparation.',limitations:'Three observations per scenario are not a population reliability estimate. This tests the displayed wrench policy, default speeds, desktop Chromium/WASM, and native slope range; no unseen terrain, hardware, or higher loads.'}};
if(process.env.BRACE_RESUME==='1'){
 const prior=JSON.parse(fs.readFileSync(path.join(OUT,'results.json'),'utf8'));
 if(prior.commit!==report.commit||prior.url!==URL)throw Error('Cannot resume a different app version or URL');
 report=prior;report.resumptions??=[];report.resumptions.push({at:new Date().toISOString(),previousFatalError:report.fatalError||null});delete report.finishedAt;delete report.fatalError;
}
if(process.env.BRACE_FOLLOWUPS==='1')report.method.durations='Follow-ups: walk for 5 simulated seconds, stop, wait 3 or 5 seconds, then exert 5 N. The ramped-force variant starts at 2 N and increases to 5 N after 2 seconds. Observe for a total of 25 movement seconds plus setup; same frozen public app and control parameters.';
if(process.env.BRACE_GENERATOR_URL){
 report.transportResumptions??=[];report.transportResumptions.push({at:new Date().toISOString(),backend:process.env.BRACE_GENERATOR_URL});
 report.method.transport='Public browser app assets remain unchanged. Per-trial generatorURL/pageURL distinguish the shared public tunnel from an isolated loopback copy of the same ONNX generator, environment, model paths and defaults. The isolated copy is used to finish balance tests after repeated shared-client disconnections; it does not test tunnel/network reliability.';
}
function save(){fs.writeFileSync(path.join(OUT,'results.json'),JSON.stringify(report,null,2));}
// Installed before app initialization. Observes physics; never writes model or robot state.
function monitor(){
 window.__robust={active:false};
 // Observe outgoing steering context without changing any bytes sent to the generator.
 const send=window.WebSocket.prototype.send;
 window.WebSocket.prototype.send=function(data){if(typeof data==='string'&&window.__robust?.active){try{const message=JSON.parse(data);if(message.type==='context'){const {frame,...context}=message;window.__robust.latestContext=context;}if(message.type==='command')window.__robust.latestCommand=message;}catch{}}return send.call(this,data);};
 const terrainNames=new Set(['floor','slope_ascent','slope_plateau','slope_descent']);
 function ground(x,y,a){let height=0;const {m,d,gn}=a;for(let g=0;g<gn.length;g++){
  if(!terrainNames.has(gn[g])||gn[g]==='floor')continue;
  const R=d.geom_xmat.subarray(g*9,g*9+9),p=d.geom_xpos.subarray(g*3,g*3+3),size=m.geom_size.subarray(g*3,g*3+3);
  if(p[2]<-10||R[8]<.2)continue;
  const top=[p[0]+R[2]*size[2],p[1]+R[5]*size[2],p[2]+R[8]*size[2]];
  const z=top[2]-(R[2]*(x-top[0])+R[5]*(y-top[1]))/R[8],dx=x-top[0],dy=y-top[1],dz=z-top[2];
  const u=R[0]*dx+R[3]*dy+R[6]*dz,v=R[1]*dx+R[4]*dy+R[7]*dz;
  if(Math.abs(u)<=size[0]+.005&&Math.abs(v)<=size[1]+.005)height=Math.max(height,z);
 }return height;}
 window.__robustStart=()=>{const a=window.__BraceAudit,d=a.d;window.__robust={active:true,start:Number(d.time),last:Number(d.time),phase:'configuration',samples:[],minClearance:Infinity,minUpright:Infinity,maxGround:0,maxHeight:0,steps:0,lastTrace:-1,lastContacts:-1,badSince:null,badContacts:[],forceErrorSum:0,forceErrorCount:0,maxEffective:0,fall:null,initial:Array.from(d.qpos.subarray(0,3))};};
 window.__robustCourse=()=>{const {m,d,gn}=window.__BraceAudit;const g=gn.indexOf('slope_descent'),p=d.geom_xpos.subarray(g*3,g*3+3),R=d.geom_xmat.subarray(g*9,g*9+9),s=m.geom_size.subarray(g*3,g*3+3);return {end:[p[0]+R[0]*s[0]+R[2]*s[2],p[1]+R[3]*s[0]+R[5]*s[2]],heading:[R[0]/Math.hypot(R[0],R[3]),R[3]/Math.hypot(R[0],R[3])],plateauHeight:(()=>{const k=gn.indexOf('slope_plateau');return d.geom_xpos[k*3+2]+m.geom_size[k*3+2]})()};};
 window.addEventListener('brace:force-reading',event=>{
  const r=window.__robust,a=window.__BraceAudit;if(!r?.active||!a)return;const {m,d,bn,gn}=a;
  const time=Number(d.time),t=time-r.start,root=Array.from(d.qpos.subarray(0,3)),q=Array.from(d.qpos.subarray(3,7));
  const surface=ground(root[0],root[1],a),clearance=root[2]-surface,upright=1-2*(q[1]**2+q[2]**2);
  r.steps++;r.time=time;r.elapsed=t;r.lastRoot=root;r.minClearance=Math.min(r.minClearance,clearance);r.minUpright=Math.min(r.minUpright,upright);r.maxGround=Math.max(r.maxGround,surface);r.maxHeight=Math.max(r.maxHeight,root[2]);
  if(time-r.lastContacts>=.10){r.lastContacts=time;r.badContacts=[];for(let i=0;i<d.ncon;i++){const c=d.contact.get(i);if(!c||c.exclude||c.dist>.002)continue;const g1=c.geom1,g2=c.geom2;let body=null;if(terrainNames.has(gn[g1]))body=bn[m.geom_bodyid[g2]];if(terrainNames.has(gn[g2]))body=bn[m.geom_bodyid[g1]];if(body&&/pelvis|torso|head|waist/.test(body))r.badContacts.push(body);}}
  const reason=!Number.isFinite(clearance+upright) ? 'non_finite_physics' : clearance<.45 ? 'low_pelvis_clearance' : upright<.5 ? 'pelvis_tilt_over_60deg' : r.badContacts.length ? 'central_body_ground_contact' : null;
  if(reason){if(r.badSince===null)r.badSince=time;if(!r.fall&&(time-r.badSince>=.10||reason==='non_finite_physics'))r.fall={elapsed:t,phase:r.phase,reason,root,clearance,upright,contacts:r.badContacts.slice()};}else r.badSince=null;
  if(time<r.last-.01&&!r.reset)r.reset={elapsed:t,phase:r.phase};r.last=time;
  const reading=event.detail,eff=[0,1].map(h=>Math.hypot(...reading.commanded.slice(h*3,h*3+3))),measured=Array.from(reading.measured);
  for(let h=0;h<2;h++){r.maxEffective=Math.max(r.maxEffective,eff[h]);if(r.phase==='motion'&&eff[h]>.01){r.forceErrorSum+=Math.abs(measured[h]-eff[h]);r.forceErrorCount++;}}
  if(time-r.lastTrace>=.20){r.lastTrace=time;r.samples.push({t,phase:r.phase,root,q,surface,clearance,upright,effective:eff,measured,contacts:r.badContacts.slice(),steering:r.latestCommand,generatorContext:r.latestContext});}
 });
}
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1440,height:1000},recordVideo:{dir:path.join(OUT,'videos'),size:{width:960,height:666}}});
 const cache=new Map();const prefix=new globalThis.URL(URL).origin;
 await context.route('**/*',async route=>{const req=route.request(),url=req.url();if(req.method()!=='GET'||!url.startsWith(prefix)){return route.continue();}
  try{let item=cache.get(url);if(!item){const response=await route.fetch({timeout:120000}),body=await response.body(),headers=response.headers();delete headers['content-encoding'];delete headers['content-length'];delete headers['transfer-encoding'];item={status:response.status(),headers,body};if(response.ok()&&!url.endsWith('/'))cache.set(url,item);if(response.ok())report.resourceHashes[url]=crypto.createHash('sha256').update(body).digest('hex');}
   if(url.includes('brace-equipment-plugin.js')){const source=item.body.toString();const needle="this.apply('none'); return true;";if(!source.includes(needle))throw Error('Published equipment plugin changed; telemetry hook unavailable');const modified=source.replace(needle,"window.__BraceAudit={m,d:this.context.mjData,bn,gn,torso:this.torso}; "+needle);return route.fulfill({...item,body:modified});}
   return route.fulfill(item);
  }catch(error){console.error('ASSET',url,error.message);await route.abort();}
 });
 await context.addInitScript(monitor);
 let page=null,keys=[],connectionLost=false;
 async function key(next){const release=keys.filter(k=>!next.includes(k)),press=next.filter(k=>!keys.includes(k));keys=next;await page.evaluate(({release,press})=>{for(const k of release)window.dispatchEvent(new KeyboardEvent('keyup',{key:k}));for(const k of press)window.dispatchEvent(new KeyboardEvent('keydown',{key:k}));},{release,press});}
 async function state(){return page.evaluate(()=>window.__robust);}
 async function wait(seconds){const initial=(await state()).time??(await page.evaluate(()=>Number(window.__BraceAudit.d.time))),deadline=Date.now()+90000;while(Date.now()<deadline){const s=await state();if(s.fall||s.reset||connectionLost)return false;if(s.time-initial>=seconds)return true;await page.waitForTimeout(100);}throw Error('Simulation failed to advance');}
 async function force(f){await page.getByRole('button',{name:f.mode==='exert'?'Exert force':'Advance Force Controls',exact:true}).click();await page.getByRole('combobox',{name:'Force hand',exact:true}).selectOption(f.hand);await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption(f.direction);await page.getByRole('button',{name:`Apply ${f.value} newtons`,exact:true}).click();}
 save();console.log(`START ${selected.length} scenarios × ${REPEATS} repetitions, public ${URL}`);
 try{for(let rep=1;rep<=REPEATS;rep++){
  // Rotate order between repetitions so scenario groups do not share a fixed planner phase.
  const offset=(rep-1)*7,ordered=selected.map((_,i)=>selected[(i+offset)%selected.length]);
  for(const c of ordered){
   if(report.trials.some(t=>t.case===c.id&&t.repetition===rep&&t.validity!=='interrupted'&&t.status!=='error'))continue;
   const previous=report.trials.filter(t=>t.case===c.id&&t.repetition===rep).length;
   const id=`${c.id}-r${rep}${previous?'-attempt'+(previous+1):''}`,row={id,case:c.id,repetition:rep,startedAt:new Date().toISOString(),status:'error'},started=Date.now();let errs=[],trialRunning=false;
   page=await context.newPage();page.setDefaultTimeout(20000);keys=[];connectionLost=false;page.on('pageerror',e=>errs.push(e.message));let wsClosed=false;page.on('websocket',ws=>{row.generatorURL=ws.url();ws.on('framereceived',({payload})=>{if(typeof payload==='string'){try{const msg=JSON.parse(payload);if(msg.type==='hello')row.controlDt=msg.control_dt;}catch{}}});ws.on('close',()=>{wsClosed=true;connectionLost=true;if(trialRunning){row.disconnectedAt=new Date().toISOString();page.evaluate(()=>({time:Number(window.__BraceAudit.d.time),fall:window.__robust.fall})).then(s=>row.stateAtDisconnect=s).catch(()=>{});}});});
   try{
    const destination=new globalThis.URL(URL);if(process.env.BRACE_GENERATOR_URL)destination.searchParams.set('stream',process.env.BRACE_GENERATOR_URL);
    row.pageURL=destination.href;row.transport=process.env.BRACE_GENERATOR_URL?'isolated-generator':'public-tunnel';
    await page.goto(destination.href,{timeout:90000});await page.waitForFunction(()=>window.BraceGym?.ready&&window.__BraceAudit,null,{timeout:90000});wsClosed=false;
    connectionLost=false;trialRunning=true;await page.evaluate(()=>window.__robustStart());await page.locator(`.brace-style[aria-label="${c.style}"]`).click();
    if(!await wait(2)){row.status='fall';}
    if(connectionLost)throw Error('Generator disconnected during configuration');
    if(!row.status.startsWith('fall')&&c.payload){await page.getByRole('region',{name:'Gym equipment'}).locator(`[data-equipment=${c.payload}]`).click();const deadline=Date.now()+45000;while(Date.now()<deadline){if((await state()).fall||connectionLost)break;if(await page.evaluate(name=>window.BraceGym.physics().applied===name,c.payload))break;await page.waitForTimeout(100);}row.payloadApplied=await page.evaluate(()=>window.BraceGym.physics().applied);}
    if(connectionLost)throw Error('Generator disconnected during equipment preparation');
    if(!((await state()).fall)&&c.force){await force(c.force);await wait(4);}
    if(!((await state()).fall)&&c.terrain==='slope'){const box=page.getByRole('checkbox',{name:'Slope ahead',exact:true});for(let j=0;j<rep;j++){if(j)await box.uncheck();await wait(.12);await box.check();await wait(.12);}row.course=await page.evaluate(()=>window.__robustCourse());row.rampAngleDeg=await page.evaluate(()=>window.BraceGym.force().rampAngleDeg);}
    row.physicsDt=await page.evaluate(()=>window.__BraceAudit.m.opt.timestep);row.commandAtMotionStart=await page.evaluate(()=>({force:window.BraceGym.force().raw,reaction:window.BraceGym.force().reaction,masses:window.BraceGym.physics().masses}));
    await page.evaluate(()=>{window.__robust.phase='motion';window.__robust.motionStart=Number(window.__BraceAudit.d.time);});const motionStart=(await state()).time;
    const maxDuration=c.terrain==='slope'?(c.style==='Slow walk'?60:45):c.transition==='settled-push'?25:c.transition?18:15;let action=0,crossedAt=null;
    if(c.motion==='backward')await key(['s']);else if(c.motion==='left')await key(['a']);else if(c.motion==='turn')await key(['w','q']);else await key(['w']);
    const deadline=Date.now()+180000;
    while(Date.now()<deadline){const s=await state();if(s.fall||s.reset||connectionLost)break;const elapsed=s.time-motionStart;
     if(c.transition==='stop-start-style-switch'){
      if(action===0&&elapsed>=5){await key([]);action++;}
      if(action===1&&elapsed>=8){await page.locator('.brace-style[aria-label="Slow walk"]').click();await key(['w']);action++;}
      if(action===2&&elapsed>=13){await page.locator('.brace-style[aria-label="Careful"]').click();action++;}
     }else if(c.transition==='equip-while-walking'&&action===0&&elapsed>=5){await page.getByRole('region',{name:'Gym equipment'}).locator('[data-equipment=kettlebell]').click();action++;}
     else if(c.transition==='stop-then-push'){
      if(action===0&&elapsed>=5){await key([]);action++;}
      if(action===1&&elapsed>=5.5){await force({mode:'exert',hand:'right',direction:'X:1',value:5});action++;}
     }else if(c.transition==='settled-push'){
      if(action===0&&elapsed>=5){await key([]);action++;}
      if(action===1&&elapsed>=5+c.settleSeconds){await force({mode:'exert',hand:'right',direction:'X:1',value:c.rampForce?2:5});action++;}
      if(action===2&&c.rampForce&&elapsed>=7+c.settleSeconds){await force({mode:'exert',hand:'right',direction:'X:1',value:5});action++;}
     }
     if(row.course){const p=s.lastRoot,e=row.course.end,h=row.course.heading,progress=(p[0]-e[0])*h[0]+(p[1]-e[1])*h[1];if(progress>=.10&&s.maxGround>=row.course.plateauHeight-.06){if(crossedAt===null)crossedAt=s.time;if(s.time-crossedAt>=2)break;}}
     if(elapsed>=maxDuration)break;await page.waitForTimeout(100);
    }
    const s=await state();row.fall=s.fall;row.reset=s.reset;row.survived=!s.fall;row.minPelvisClearance=s.minClearance;row.minUpright=s.minUpright;row.maxTerrainHeight=s.maxGround;row.peakRootHeight=s.maxHeight;row.simulatedSeconds=s.elapsed;row.motionSeconds=s.time-motionStart;row.initial=s.initial;row.final=s.lastRoot;row.distance=Math.hypot(s.lastRoot[0]-s.initial[0],s.lastRoot[1]-s.initial[1]);row.forceMaeN=s.forceErrorCount?s.forceErrorSum/s.forceErrorCount:null;row.maxEffectiveForceN=s.maxEffective;row.payloadAtEnd=await page.evaluate(()=>window.BraceGym.physics().applied);row.referenceMode=await page.evaluate(()=>window.BraceReference.mode);row.errors=errs;
    row.completed=Boolean(!s.reset&&!wsClosed&&!errs.length&&row.motionSeconds>=Math.min(8,maxDuration)&&row.distance>.5&&(!row.course||crossedAt!==null)&&(!c.payload||row.payloadApplied===c.payload)&&(!c.force||c.force.mode!=='exert'||s.maxEffective>.1)&&(!c.transition||action>=({ 'stop-start-style-switch':3,'equip-while-walking':1,'stop-then-push':2,'settled-push':c.rampForce?3:2}[c.transition]))&&(c.transition!=='equip-while-walking'||row.payloadAtEnd==='kettlebell')&&(c.transition!=='settled-push'||s.maxEffective>=4.9));
    row.status=s.fall?'fall':(s.reset||wsClosed||errs.length)?'error':row.completed?'success':'incomplete';
    fs.writeFileSync(path.join(OUT,'traces',`${id}.json`),JSON.stringify(s,null,2));
    if(row.status!=='success'){await page.screenshot({path:path.join(OUT,'screenshots',`${id}.png`),timeout:15000});row.screenshot=`screenshots/${id}.png`;}
   }catch(error){row.error=error.message;try{const s=await state();if(s.fall){row.status='fall';row.fall=s.fall;row.survived=false;}fs.writeFileSync(path.join(OUT,'traces',`${id}.json`),JSON.stringify(s,null,2));await page.screenshot({path:path.join(OUT,'screenshots',`${id}.png`),timeout:5000});row.screenshot=`screenshots/${id}.png`;}catch{} }
   finally{trialRunning=false;row.connectionClosedDuringTrial=wsClosed;row.validity=wsClosed||row.status==='error'?'interrupted':'valid';try{await key([]);}catch{}try{await page.evaluate(()=>window.__robust.active=false);}catch{}const video=page.video();await page.close();if(video){try{const from=await video.path(),to=path.join(OUT,'videos',`${id}.webm`);fs.renameSync(from,to);row.video=`videos/${id}.webm`;}catch{}}row.wallSeconds=(Date.now()-started)/1000;report.trials.push(row);save();console.log(JSON.stringify({trial:report.trials.length,total:selected.length*REPEATS,id,status:row.status,validity:row.validity,fall:row.fall,angle:row.rampAngleDeg,seconds:row.simulatedSeconds,error:row.error}));}
  }
 }}finally{report.finishedAt=new Date().toISOString();save();await context.close();await browser.close();}
 console.log('FINISHED',JSON.stringify(report.trials.reduce((a,r)=>(a[r.status]=(a[r.status]||0)+1,a),{})));
})().catch(error=>{report.fatalError=error.stack;save();console.error(error);process.exitCode=1;});
