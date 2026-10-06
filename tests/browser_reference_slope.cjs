// Run sequentially: the generator permits one active browser.
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs');
(async()=>{const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});const samples=[];try{
 const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');await page.waitForFunction(()=>window.BraceGym?.ready&&window.BraceReference?.policyAnchorXY.length===2,null,{timeout:90000});
 await page.getByRole('checkbox',{name:'Slope ahead',exact:true}).check();
 const snapshot=()=>page.evaluate(()=>({reference:window.BraceReference,force:window.BraceGym.force()}));const initial=await snapshot();let captured=false;
 await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keydown',{key:'w'})));
 try{while((await snapshot()).force.time<initial.force.time+24){await page.waitForTimeout(200);const s=await snapshot();samples.push(s);const r=s.reference;
   assert.equal(r.mode,'body');assert.equal(r.displayAnchor[1],r.sourceAnchor[1],'reference acquired robot/ramp height');
   assert(Math.hypot(r.displayAnchor[0]-r.robotAnchor[0],r.displayAnchor[2]-r.robotAnchor[2])<1e-5,'horizontal alignment lost');assert(Math.hypot(...r.policyAnchorXY)<1e-5);
   if(process.env.BRACE_SCREENSHOT&&!captured&&s.force.root[2]>initial.force.root[2]+.22){await page.screenshot({path:process.env.BRACE_SCREENSHOT});captured=true;}
 }}finally{await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keyup',{key:'w'})));}
 const peak=Math.max(...samples.map(s=>s.force.root[2])),gap=Math.max(...samples.map(s=>s.reference.robotAnchor[1]-s.reference.displayAnchor[1]));
 assert(peak>initial.force.root[2]+.12,'robot did not climb');assert(gap>.15,'flat reference followed ramp');assert(Math.min(...samples.map(s=>s.force.root[2]))>.6,'robot fell');assert.deepEqual(errors,[]);
 console.log(JSON.stringify({peakRobotHeight:peak,maxRobotReferenceHeightGap:gap,rampAngleDeg:samples.at(-1).force.rampAngleDeg,samples:samples.length}));console.log('PASS slope: flat-ground reference height preserved; horizontal alignment and policy origin retained');
}finally{fs.writeFileSync(process.env.BRACE_REFERENCE_AUDIT||'/tmp/brace-ground-reference-audit.json',JSON.stringify(samples,null,2));await browser.close();}})().catch(e=>{console.error(e);process.exitCode=1});
