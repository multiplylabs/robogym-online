// Run browser tests sequentially: the shared generator permits one active client.
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs');
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const results=[];
 try{
 const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
 await page.waitForFunction(()=>window.BraceGym?.ready,null,{timeout:90000});
 await page.getByRole('button',{name:'Exert force',exact:true}).click();
 await page.getByRole('region',{name:'Live force gauge'}).waitFor();
 for(const [hand,direction,value] of [['right','X:1',2],['right','X:1',5],['right','X:1',8],['left','X:1',5],['both','X:1',5],['right','X:-1',5],['right','Z:1',5],['right','Z:-1',5],['right','Y:-1',5],['right','Y:1',5]]){
  await page.getByRole('combobox',{name:'Force hand',exact:true}).selectOption(hand);
  await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption(direction);
  await page.getByRole('button',{name:`Apply ${value} newtons`,exact:true}).click();
  await page.locator('.brace-force-status').filter({hasText:`Exerting ${value} N`}).waitFor();
  const start=await page.evaluate(()=>window.BraceGym.force().time);
  await page.waitForFunction(t=>window.BraceGym.force().time>=t+10,start,{timeout:45000});
  const samples=[];let stop=(await page.evaluate(()=>window.BraceGym.force().time))+3;
  while((await page.evaluate(()=>window.BraceGym.force().time))<stop){
   await page.waitForTimeout(100);
   samples.push(await page.evaluate(()=>({force:window.BraceGym.force(),reading:window.BraceExert.reading,physics:window.BraceGym.physics()})));
  }
  const selected=hand==='both'?[0,1]:[hand==='left'?0:1];
  const summary=selected.map(h=>{
   const errors=samples.map(s=>s.reading.measured[h]-Math.hypot(...s.reading.commanded.slice(h*3,h*3+3)));
   return {hand:h===0?'left':'right',mean:samples.reduce((v,s)=>v+s.reading.measured[h],0)/samples.length,target:samples.reduce((v,s)=>v+Math.hypot(...s.reading.commanded.slice(h*3,h*3+3)),0)/samples.length,mae:errors.reduce((v,e)=>v+Math.abs(e),0)/errors.length,maxError:Math.max(...errors.map(Math.abs))};
  });
  const row={hand,direction,value,summary,minHeight:Math.min(...samples.map(s=>s.force.root[2]))};results.push(row);console.log(JSON.stringify(row));
  assert(row.minHeight>.65,'robot fell');for(const h of summary)assert(h.mae<.65,`force error: ${JSON.stringify(row)}`);
  assert.equal(await page.evaluate(()=>window.BraceGym.lastContext.equipment),'exertion');
 }
 await page.getByRole('button',{name:'Clear force',exact:true}).click();
 await page.waitForFunction(()=>window.BraceGym.force().reaction.every(v=>Math.hypot(...v)<.001));
 assert.equal(await page.locator('.brace-gauge-hand').count(),0);
 await page.getByRole('button',{name:'Compensate with weights',exact:true}).click();
 await page.getByRole('region',{name:'Gym equipment'}).locator('[data-equipment=barbell]').click();
 await page.waitForFunction(()=>window.BraceGym.physics().applied==='barbell',null,{timeout:45000});
 assert.deepEqual(await page.evaluate(()=>window.BraceGym.physics().masses),[2,0]);
 await page.getByRole('button',{name:'Reset',exact:true}).click();
 await page.waitForFunction(()=>!window.BraceExert.enabled&&window.BraceGym.physics().applied==='none');
 await page.setViewportSize({width:390,height:844});assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
 assert.deepEqual(errors,[]);console.log('PASS exertion tracking, signed axes, both hands, clear, weights, reset and mobile');
 }finally{fs.writeFileSync(process.env.BRACE_FORCE_AUDIT||'/tmp/brace-force-audit.json',JSON.stringify(results,null,2));await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
