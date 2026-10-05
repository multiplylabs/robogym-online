// Run sequentially: the shared generator permits one active browser.
const {chromium}=require('playwright');
const fs=require('node:fs'),assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[],results=[];
 page.on('pageerror',e=>errors.push(e.message));
 const snapshot=()=>page.evaluate(()=>({force:window.BraceGym.force(),reading:window.BraceExert.reading}));
 async function collect(label,seconds,moving){
  const start=(await snapshot()).force.time,samples=[];
  if(moving)await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keydown',{key:'w'})));
  try{
   while((await snapshot()).force.time<start+seconds){await page.waitForTimeout(200);samples.push(await snapshot());}
  }finally{if(moving)await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keyup',{key:'w'})));}
  const settled=samples.filter(s=>s.force.time>start+(moving?2:10));
  const row={label,minHeight:Math.min(...samples.map(s=>s.force.root[2])),maxHeight:Math.max(...samples.map(s=>s.force.root[2])),rampAngleDeg:samples.at(-1).force.rampAngleDeg,minUpright:Math.min(...samples.map(s=>1-2*(s.force.rootQuat[1]**2+s.force.rootQuat[2]**2))),mae:settled.reduce((v,s)=>v+Math.abs(s.reading.measured[1]-Math.hypot(...s.reading.commanded.slice(3,6))),0)/settled.length,start:samples[0].force.root,end:samples.at(-1).force.root};
  results.push(row);console.log(JSON.stringify(row));assert(row.minHeight>.6&&row.minUpright>.65,'robot fell');assert(row.mae<(moving?1.5:.65),'force tracking error');
  if(label==='Stealth on slope'){assert(row.rampAngleDeg>=6&&row.rampAngleDeg<=10);assert(row.maxHeight>row.start[2]+.12,'did not climb the ramp');assert(Math.hypot(row.end[0]-row.start[0],row.end[1]-row.start[1])>10.1,'did not cross the course');}
 }
 try{
  await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
  await page.waitForFunction(()=>window.BraceGym?.ready,null,{timeout:90000});
  await page.getByRole('button',{name:'Exert force',exact:true}).click();
  await page.getByRole('button',{name:'Apply 5 newtons',exact:true}).click();
  await page.locator('.brace-force-status').filter({hasText:'Exerting 5 N'}).waitFor();
  await collect('Idle forward',13,false);
  if(process.env.BRACE_SCREENSHOTS){
   const cdp=await page.context().newCDPSession(page);await cdp.send('Emulation.setVirtualTimePolicy',{policy:'pause'});
   await page.screenshot({path:`${process.env.BRACE_SCREENSHOTS}/brace-exert-gauge.png`,timeout:20000});
   await cdp.send('Emulation.setVirtualTimePolicy',{policy:'advance'});
  }
  await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption('Y:1');
  await page.getByRole('button',{name:'Apply 5 newtons',exact:true}).click();
  await collect('Idle leftward',13,false);
  await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption('X:1');
  await page.getByRole('button',{name:'Apply 5 newtons',exact:true}).click();
  for(const style of ['Stealth','Slow walk','Object carrying','Careful']){
   await page.locator(`.brace-style[aria-label="${style}"]`).click();
   await collect(style,8,true);await page.waitForTimeout(2000);
  }
  await page.locator('.brace-style[aria-label="Stealth"]').click();
  await page.getByRole('checkbox',{name:'Slope ahead',exact:true}).check();
  await collect('Stealth on slope',24,true);
  assert.deepEqual(errors,[]);console.log('PASS 5 N exertion in all four walking styles and across a slope');
 }finally{
  fs.writeFileSync(process.env.BRACE_MOTION_AUDIT||'/tmp/brace-motion-force-audit.json',JSON.stringify(results,null,2));
  await browser.close();
 }
})().catch(e=>{console.error(e);process.exitCode=1;});
