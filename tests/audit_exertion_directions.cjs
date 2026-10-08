// Fresh sessions, real generator commands and sticky respawn/fall outcomes. Raw user requests stay unchanged.
const{chromium}=require('playwright'),fs=require('node:fs');
const out=process.env.BRACE_AUDIT_OUTPUT||'/tmp/brace-direction-audit';fs.mkdirSync(out,{recursive:true});
const defaults=[
 {id:'forward',direction:'X:1',hand:'right',force:8,keys:['w']},
 {id:'side-left',direction:'X:1',hand:'right',force:8,keys:['a']},
 {id:'side-right',direction:'X:1',hand:'right',force:8,keys:['d']},
 {id:'diagonal-left',direction:'X:1',hand:'both',force:8,keys:['w','a']},
 {id:'turn-left',direction:'X:1',hand:'both',force:8,keys:['q']},
 {id:'turn-right',direction:'X:1',hand:'both',force:8,keys:['e']},
 {id:'left-push-forward',direction:'Y:1',hand:'both',force:8,keys:['w']},
 {id:'left-push-aligned',direction:'Y:1',hand:'both',force:8,keys:['a']},
 {id:'left-push-turn',direction:'Y:1',hand:'both',force:8,keys:['q']},
 {id:'back-push-back',direction:'X:-1',hand:'right',force:5,keys:['s']},
 {id:'back-push-turn',direction:'X:-1',hand:'right',force:5,keys:['e']},
 {id:'up-push-side',direction:'Z:1',hand:'both',force:5,keys:['a']}
];
const cases=process.env.BRACE_AUDIT_CASES_FILE?JSON.parse(fs.readFileSync(process.env.BRACE_AUDIT_CASES_FILE,'utf8')):process.env.BRACE_AUDIT_CASES?JSON.parse(process.env.BRACE_AUDIT_CASES):defaults;
(async()=>{const b=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});const results=[];try{
async function trial(config){const c=await b.newContext({viewport:{width:Number(process.env.BRACE_AUDIT_WIDTH||1100),height:Number(process.env.BRACE_AUDIT_HEIGHT||900)}});let page;const row={...config,outcome:'incomplete',errors:[],samples:[]};try{
await c.addInitScript(({backwardSideArc,backwardTurnDeg})=>{window.__resets=[];window.__commands=[];window.__streamStats={receivedTo:0,cursor:0,minFuture:Infinity,shortHorizonSamples:0};window.addEventListener('brace:auto-respawn',e=>window.__resets.push({...e.detail,force:window.BraceGym?.force?.()}));const W=window.WebSocket;window.WebSocket=class extends W{constructor(...args){super(...args);this.addEventListener('message',event=>{if(event.data instanceof ArrayBuffer){const h=new DataView(event.data);window.__streamStats.receivedTo=h.getInt32(0,true)+h.getInt32(4,true)}})}send(s){try{const v=JSON.parse(s);if(v.type==='context'&&v.frame!==undefined){const d=window.__streamStats;d.cursor=v.frame;d.future=d.receivedTo-1-v.frame;d.minFuture=Math.min(d.minFuture,d.future);if(d.future<20)d.shortHorizonSamples++}if(v.type==='command'){if(backwardTurnDeg!==undefined&&v.forward<0&&v.turn){v.turn=Math.sign(v.turn)*backwardTurnDeg;s=JSON.stringify(v)}if(backwardSideArc&&v.forward<0&&v.turn){v.forward=0;v.lateral=Math.sign(v.turn)*.45;s=JSON.stringify(v)}window.__commands.push(v)}}catch{}super.send(s)}}},{backwardSideArc:config.backwardSideArc,backwardTurnDeg:config.backwardTurnDeg});
if(config.arcCap!==undefined || config.sideCap!==undefined || config.turnDeg!==undefined || config.backwardRestCap!==undefined || config.armPolicyBlend!==undefined)await c.route('**/brace-equipment-plugin.js*',async route=>{const response=await route.fetch();let body=await response.text();if(config.armPolicyBlend!==undefined){const blend=Number(config.armPolicyBlend);if(!Number.isFinite(blend)||blend<0||blend>1)throw Error('Invalid arm blend');body=body.replace('const w = this.holdWeight = this.holdStartWeight*(1-s)+goalWeight*s;',`let w = this.holdWeight = this.holdStartWeight*(1-s)+goalWeight*s; if(this.holdName==='exertion' && window.BraceSteering?.command?.some(v=>Math.abs(v)>.01)) w*=${blend};`);}for(const [key,needle] of [['arcCap','turning ? 3 : Infinity'],['sideCap','cross ? 2 : turning'],['turnDeg','slope ? 1 : 5'],['backwardRestCap','if(f<-.01)']])if(config[key]!==undefined){const value=Number(config[key]);if(!Number.isFinite(value)||value<0||value>9)throw Error('Invalid audit parameter');body=body.replace(needle,key==='arcCap'?`turning ? ${value} : Infinity`:key==='sideCap'?`cross ? ${value} : turning`:key==='turnDeg'?`slope ? 1 : ${value}`:`if([raw[1],raw[4]].some(v=>v<-.05)) cap=Math.min(cap,${value}); if(f<-.01)`);}await route.fulfill({response,body})});
page=await c.newPage();page.on('pageerror',e=>row.errors.push(e.message));await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');await page.waitForFunction(()=>window.BraceGym?.ready,null,{timeout:120000});
if(config.slope)await page.getByRole('checkbox',{name:'Slope ahead',exact:true}).check();
if(config.style)await page.locator(`.brace-style[aria-label="${config.style}"]`).click();
if(config.force>0){await page.getByRole('button',{name:'Exert force',exact:true}).click();await page.getByRole('combobox',{name:'Force hand',exact:true}).selectOption(config.hand);await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption(config.direction);await page.getByRole('button',{name:`Apply ${config.force} newtons`,exact:true}).click();}
const expected=new Array(6).fill(0);const axis={X:0,Y:1,Z:2}[config.direction[0]],sign=Number(config.direction.split(':')[1]);for(const h of config.hand==='both'?[0,1]:[config.hand==='left'?0:1])expected[h*3+axis]=sign*config.force;
await page.waitForFunction(expected=>{const raw=window.BraceGym.force().raw;return raw.length===7&&expected.every((v,i)=>Math.abs(raw[i+1]-v)<.01)},expected,{timeout:30000});
let start=await page.evaluate(()=>window.BraceGym.force().time);await page.waitForFunction(t=>window.BraceGym.force().time>t+2||window.__resets.length,start,{timeout:30000});
start=await page.evaluate(()=>window.BraceGym.force().time);row.start=start;
await page.evaluate(keys=>keys.forEach(key=>window.dispatchEvent(new KeyboardEvent('keydown',{key}))),config.keys);
const wall=Date.now(),seconds=config.seconds||10;let activeKeys=config.keys,phase=0,endTime=start+seconds;row.appliedPhases=[];
while(Date.now()-wall<(config.wallSeconds||300)*1000){await page.waitForTimeout(150);const snap=await page.evaluate(()=>({force:window.BraceGym.force(),reading:window.BraceExert?.reading,envelope:window.BraceExertionEnvelope,steering:window.BraceSteering,resets:window.__resets.length,referenceSourceAnchor:window.BraceReference?.sourceAnchor,streamStats:{...window.__streamStats},referenceStream:window.BraceReferenceStream}));row.samples.push(snap);const progress=Math.floor((snap.force.time-start)/10);if(progress>(row.progress??-1)){row.progress=progress;console.log(JSON.stringify({progress:config.id,seconds:Math.round(snap.force.time-start),stream:snap.streamStats,referenceStream:snap.referenceStream}));}
while(config.phases && phase<config.phases.length && snap.force.time>=start+config.phases[phase].at && !snap.resets){
 const nextKeys=config.phases[phase++].keys;
 const applied=await page.evaluate(({old,next})=>{old.forEach(key=>window.dispatchEvent(new KeyboardEvent('keyup',{key})));next.forEach(key=>window.dispatchEvent(new KeyboardEvent('keydown',{key})));return window.BraceGym.force().time},{old:activeKeys,next:nextKeys});activeKeys=nextKeys;
 row.appliedPhases.push({at:applied-start,requestedAt:config.phases[phase-1].at,keys:nextKeys});
 if(phase===config.phases.length&&!nextKeys.length)endTime=Math.max(endTime,applied+seconds-config.phases[phase-1].at);
}
if(snap.resets||snap.force.time>=endTime)break;}
await page.evaluate(keys=>keys.forEach(key=>window.dispatchEvent(new KeyboardEvent('keyup',{key}))),activeKeys);
const meta=await page.evaluate(()=>({resets:window.__resets,commands:window.__commands,blocked:window.BraceSteering?.blockedKeys,streamStats:window.__streamStats}));Object.assign(row,meta);
row.requestedForcePreserved=row.samples.filter(s=>s.force.raw.length===7).filter(s=>s.force.raw.some((v,i)=>i>0&&Math.abs(v)>.01)).every(s=>s.force.raw.slice(1).filter(v=>Math.abs(v)>.01).every(v=>Math.abs(Math.abs(v)-config.force)<.01));
row.duration=meta.resets.length ? meta.resets[0].force.time-start : row.samples.at(-1)?.force.time-start;
row.requiredEnd=endTime;row.outcome=meta.resets.length?'failure':row.samples.at(-1)?.force.time>=endTime?'success':'interrupted';row.headingChangeDeg=0;let previousYaw=null;for(const sample of row.samples){const q=sample.force.rootQuat,yaw=Math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));if(previousYaw!==null)row.headingChangeDeg+=Math.atan2(Math.sin(yaw-previousYaw),Math.cos(yaw-previousYaw))*180/Math.PI;previousYaw=yaw;}
if(row.outcome==='success' && config.keys.length && config.keys.every(k=>meta.blocked?.includes(k))) row.outcome='blocked';
row.minUpright=Math.min(...row.samples.map(s=>1-2*(s.force.rootQuat[1]**2+s.force.rootQuat[2]**2)));row.minHeight=Math.min(...row.samples.map(s=>s.force.root[2]));row.displacement=row.samples.length?Math.hypot(row.samples.at(-1).force.root[0]-row.samples[0].force.root[0],row.samples.at(-1).force.root[1]-row.samples[0].force.root[1]):0;
row.pathLength=0;row.referencePathLength=0;for(let i=1;i<row.samples.length;i++){const a=row.samples[i-1],b=row.samples[i];row.pathLength+=Math.hypot(b.force.root[0]-a.force.root[0],b.force.root[1]-a.force.root[1]);if(a.referenceSourceAnchor&&b.referenceSourceAnchor)row.referencePathLength+=Math.hypot(b.referenceSourceAnchor[0]-a.referenceSourceAnchor[0],b.referenceSourceAnchor[2]-a.referenceSourceAnchor[2]);}
row.maxHeight=Math.max(...row.samples.map(s=>s.force.root[2]));
if(config.requireRampCrossing && row.outcome==='success' && (row.maxHeight<1.1 || row.samples.at(-1).force.root[2]>.95 || row.samples.at(-1).force.root[0]-row.samples[0].force.root[0]<10))row.outcome='incomplete_ramp';
if(config.requireRotation && row.outcome==='success' && Math.abs(row.headingChangeDeg)<350)row.outcome='incomplete_rotation';
if(config.requireTurnDeg && row.outcome==='success' && row.headingChangeDeg*(config.keys.includes('q')?1:-1)<config.requireTurnDeg)row.outcome='incomplete_turn';
if(config.minArcMeters && row.outcome==='success' && (row.pathLength<config.minArcMeters || row.referencePathLength<config.minArcMeters))row.outcome='incomplete_arc';
await page.screenshot({path:`${out}/${config.id}.png`});
}catch(e){row.outcome='error';row.error=String(e)}finally{results.push(row);fs.writeFileSync(`${out}/results.json`,JSON.stringify(results,null,2));console.log(JSON.stringify({...row,samples:undefined}));await c.close()}}
let next=0;await Promise.all(Array.from({length:Number(process.env.BRACE_AUDIT_CONCURRENCY||2)},async()=>{while(next<cases.length)await trial(cases[next++])}));
}finally{await b.close()}console.log(JSON.stringify({success:results.filter(r=>r.outcome==='success').length,total:results.length}));})().catch(e=>{console.error(e);process.exitCode=1});
