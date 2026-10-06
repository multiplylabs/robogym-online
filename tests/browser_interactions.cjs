// Local/public smoke test: concurrent users, reconnection, physical impacts and force arrows.
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs');
const url=process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765';
const out=process.env.BRACE_TEST_OUTPUT||'/tmp/brace-interactions';fs.mkdirSync(out,{recursive:true});
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const results={url,checks:[],trials:[],errors:[]};
 try {
  const context=await browser.newContext({viewport:{width:1440,height:1000}});
  await context.addInitScript(()=>{
   const Native=window.WebSocket;window.__testSockets=[];
   window.WebSocket=class extends Native {constructor(...args){super(...args);window.__testSockets.push(this);}};
  });
  await context.route('**/brace-equipment-plugin.js*',async route=>{
   const response=await route.fetch();const code=(await response.text()).replace("this.apply('none'); return true;","window.__BraceTest={m,d:this.context.mjData,scene:this.context.scene,bodies:this.context.bodies}; this.apply('none'); return true;");
   await route.fulfill({response,body:code});
  });
  async function open(){const page=await context.newPage();page.on('pageerror',e=>results.errors.push(e.message));await page.goto(url,{timeout:120000});await page.waitForFunction(()=>window.BraceGym?.ready&&window.BraceStability,null,{timeout:120000});return page;}
  const a=await open();await a.getByRole('button',{name:'Equip barbell 2 kg',exact:true}).click();
  await a.waitForFunction(()=>window.BraceGym.physics().applied==='barbell',null,{timeout:60000});
  const b=await open();
  assert(await a.evaluate(()=>window.BraceGym.ready));
  await b.getByRole('button',{name:'Equip dumbbells 1 kg each',exact:true}).click();
  await b.waitForFunction(()=>window.BraceGym.physics().applied==='dumbbells',null,{timeout:60000});
  assert.equal(await a.evaluate(()=>window.BraceGym.physics().applied),'barbell');
  results.checks.push('Two simultaneous equipped users remain connected and independent');
  const before=await a.evaluate(()=>({mass:window.BraceGym.physics().masses,time:window.BraceGym.force().time,count:window.__testSockets.length}));
  await a.evaluate(()=>window.__testSockets.at(-1).close());
  await a.waitForFunction(count=>window.__testSockets.length>count&&window.BraceGym.ready,before.count,{timeout:30000});
  const after=await a.evaluate(()=>({mass:window.BraceGym.physics().masses,time:window.BraceGym.force().time,selected:window.BraceGym.selected}));
  assert.deepEqual(after.mass,before.mass);assert.equal(after.selected,'barbell');assert(after.time>before.time);assert(await b.evaluate(()=>window.BraceGym.ready));
  results.checks.push('Transient disconnect reconnects automatically, preserving payload mass and motion');
  await b.close();
  // Each trial gets a fresh standing robot; retain actual failures as failures.
  await a.close();
  for(const shape of ['burst-one','burst-two']) {
   const page=await open();
   await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keydown',{key:'w'})));
   const start=await page.evaluate(()=>window.BraceGym.force().time);
   await page.waitForFunction(t=>window.BraceGym.force().time>t+2,start,{timeout:30000});
   await page.evaluate(()=>window.dispatchEvent(new KeyboardEvent('keyup',{key:'w'})));
   await page.waitForFunction(t=>window.BraceGym.force().time>t+5,start,{timeout:30000});
   assert(await page.evaluate(()=>window.BraceStability.launch()));
   await page.waitForFunction(()=>window.BraceStability.state.launched>=3,null,{timeout:30000});
   await page.screenshot({path:`${out}/${shape}-impact.png`});
   await page.waitForFunction(()=>window.BraceStability.state.phase!=='testing',null,{timeout:60000});
   const state=await page.evaluate(()=>window.BraceStability.state);results.trials.push(state);
   assert.equal(state.launched,5); assert.equal(state.count,5);
   state.launchTimes.forEach((t,i)=>assert(Math.abs(t-i*.2)<.025, `Launch ${i} timing: ${t}`));
   assert(state.hit, 'Burst must physically hit the robot');
   assert(['recovered','partial','failed'].includes(state.phase));
   if(state.fallen) assert.equal(state.phase,'failed');
   await page.screenshot({path:`${out}/${shape}-result.png`});await page.close();
  }
  const page=await open();await page.getByRole('button',{name:'Exert force',exact:true}).click();
  await page.getByRole('button',{name:'Apply 5 newtons',exact:true}).click();
  await page.waitForFunction(()=>Math.max(...window.BraceExert.reading.commanded.map(Math.abs))>1,null,{timeout:30000});
  const visuals=await page.evaluate(()=>{const seen=[];window.__BraceTest.scene.traverse(o=>{if(/hand-force|hand-contact-spring/.test(o.name))seen.push({name:o.name,visible:o.visible,data:o.userData});});return seen;});
  assert(visuals.some(o=>o.name.includes('commanded')&&o.visible));assert(visuals.some(o=>o.name.includes('exerted')&&o.visible));assert(!visuals.some(o=>o.name.includes('gauge')));
  assert(!visuals.some(o=>o.name.includes('contact-spring')));
  assert.equal(await page.locator('.brace-error-track, .brace-error-labels').count(),0);
  results.checks.push('Blue effective-force and amber measured-force arrows render; spring, moving gauge and lower error row are absent');
  await page.screenshot({path:`${out}/force-arrows.png`});
  assert(await page.evaluate(()=>window.BraceStability.launch()));
  assert.equal(await page.evaluate(()=>window.BraceStability.state.mass),.25);
  assert((await page.evaluate(()=>Array.from(window.__BraceTest.m.body_mass).slice(-5))).every(v=>v===.25));
  await page.waitForFunction(()=>window.BraceStability.state.phase!=='testing',null,{timeout:60000});
  results.trials.push(await page.evaluate(()=>window.BraceStability.state));
  await page.getByRole('button',{name:'Compensate with weights',exact:true}).click();
  await page.waitForFunction(()=>!window.BraceExert.enabled);
  assert(await page.evaluate(()=>window.BraceStability.launch()));
  assert.equal(await page.evaluate(()=>window.BraceStability.state.mass),.75);
  assert((await page.evaluate(()=>Array.from(window.__BraceTest.m.body_mass).slice(-5))).every(v=>v===.75));
  results.checks.push('Actual projectile mass is 0.25 kg during exertion and restores to 0.75 kg otherwise');
  assert.deepEqual(results.errors,[]);console.log(JSON.stringify(results,null,2));
 } finally {fs.writeFileSync(`${out}/results.json`,JSON.stringify(results,null,2));await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
