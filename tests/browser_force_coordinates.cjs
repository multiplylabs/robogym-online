// Real MuJoCo/ONNX page: turn first, then verify local dial and world reaction at 8 N.
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs');
const url=process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765';
(async()=>{const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});const results=[];
try{for(const direction of ['X:1','Y:-1']){
const context=await browser.newContext({viewport:{width:1200,height:900}});
try{const page=await context.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));await page.goto(url);await page.waitForFunction(()=>window.BraceGym?.ready,null,{timeout:120000});
let time=await page.evaluate(()=>window.BraceGym.force().time);await page.keyboard.down('q');await page.waitForFunction(t=>window.BraceGym.force().time>t+4.5,time,{timeout:30000});await page.keyboard.up('q');
time=await page.evaluate(()=>window.BraceGym.force().time);await page.waitForFunction(t=>window.BraceGym.force().time>t+3,time,{timeout:30000});
await page.getByRole('button',{name:'Exert force',exact:true}).click();await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption(direction);await page.getByRole('button',{name:'Apply 8 newtons',exact:true}).click();
const index=direction[0]==='X'?3:4,sign=Number(direction.split(':')[1]);
await page.waitForFunction(({index,sign})=>Math.abs(window.BraceGym.force().raw[index+1]-8*sign)<.01,{index,sign});
time=await page.evaluate(()=>window.BraceGym.force().time);await page.waitForFunction(t=>window.BraceGym.force().time>t+5,time,{timeout:30000});
const reading=await page.evaluate(()=>({...window.BraceGym.force(),measured:Array.from(window.BraceExert.reading.measured)}));assert(reading.torsoQuat,'The force diagnostics must expose the actual torso quaternion');const [w,x,y,z]=reading.torsoQuat,yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));assert(Math.abs(yaw)>.6,'Robot must face away from original world X');
const local=reading.axis.slice(3,6);assert(Math.abs(local[index-3]-sign)<1e-5);assert(Math.abs(reading.raw[index+1]-8*sign)<.01);
const n=[Math.cos(yaw)*local[0]-Math.sin(yaw)*local[1],Math.sin(yaw)*local[0]+Math.cos(yaw)*local[1],local[2]],reaction=reading.reaction[1],magnitude=Math.hypot(...reaction);assert(magnitude>.01,'Need a nonzero measured contact reaction');const projection=reaction.reduce((s,v,i)=>s-v*n[i],0);assert(Math.abs(projection)>magnitude*.995,'Reaction must lie along actual torso-heading axis');assert(Math.abs(projection-reading.measured[1])<Math.max(.05,magnitude*.01),'Measured exertion must be the equal/opposite robot reaction');
assert.match(await page.locator('.brace-force-frame').innerText(),/Local torso heading/);assert.deepEqual(errors,[]);
results.push({direction,torsoYawDeg:yaw*180/Math.PI,local,worldAxis:n,reactionWorld:reaction,measured:reading.measured,requested:reading.raw.slice(1),effective:reading.commanded});console.log(`PASS ${direction}: local 8 N at torso yaw ${(yaw*180/Math.PI).toFixed(1)}°, world reaction follows heading`);
}finally{await context.close()}}
if(process.env.BRACE_COORDINATE_REPORT)fs.writeFileSync(process.env.BRACE_COORDINATE_REPORT,JSON.stringify(results,null,2));
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
