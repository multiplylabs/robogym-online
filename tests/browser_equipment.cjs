// NODE_PATH=mjswan/template/node_modules node tests/browser_equipment.cjs
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try {
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
  await page.waitForFunction(()=>window.BraceGym?.physics && !document.querySelector('[data-equipment=dumbbells]')?.disabled,null,{timeout:90000});
  assert.equal(await page.getByRole('checkbox',{name:'Show reference',exact:true}).isChecked(),true);
  console.log('PASS reference trajectory is visible by default');
  const snapshot=()=>page.evaluate(()=>window.BraceGym.physics());
  const equipment=page.getByRole('region',{name:'Gym equipment'});
  assert.deepEqual((await snapshot()).masses,[0,0]);
  assert.equal((await snapshot()).meshes,0);
  console.log('PASS empty hands have no payload or visible weights');
  for(const [name,mass,each,meshCount] of [['dumbbells',2,[1,1],10],['kettlebell',3.5,[3.5,0],4],['barbell',2,[2,0],5]]){
   await equipment.locator(`[data-equipment=${name}]`).click();
   await page.waitForFunction(name=>window.BraceGym.physics().applied===name,name,{timeout:45000});
   let p=await snapshot();assert.deepEqual(p.masses,each);assert.equal(p.meshes,meshCount);
   assert.equal(p.welds.barbell,name==='barbell');assert.equal(p.welds.kettlebell,name==='kettlebell');
   assert.equal(await page.evaluate(()=>window.BraceGym.selected),name);
   await page.waitForTimeout(3000);p=await snapshot();assert(p.root[2]>.6,JSON.stringify(p));
   console.log(`PASS ${name}: ${mass} kg, correct shared grip, visible geometry, stable hold`);
   if(process.env.BRACE_SCREENSHOTS){
    const cdp=await page.context().newCDPSession(page);await cdp.send('Emulation.setVirtualTimePolicy',{policy:'pause'});
    await page.screenshot({path:`${process.env.BRACE_SCREENSHOTS}/brace-${name}.png`,timeout:15000});
    await cdp.send('Emulation.setVirtualTimePolicy',{policy:'advance'});
   }
   await equipment.getByRole('button',{name:'Put down',exact:true}).click();
   await page.waitForFunction(()=>window.BraceGym.physics().applied==='none');
   assert.deepEqual((await snapshot()).masses,[0,0]);assert.equal((await snapshot()).meshes,0);
   await page.waitForTimeout(2000);
  }
  await equipment.locator('[data-equipment=barbell]').click();
  await page.waitForFunction(()=>window.BraceGym.physics().applied==='barbell');
  await page.getByRole('button',{name:'Reset',exact:true}).click();
  await page.waitForFunction(()=>window.BraceGym.selected==='none' && window.BraceGym.physics().applied==='none');
  assert.deepEqual((await snapshot()).masses,[0,0]);console.log('PASS reset restores empty hands');
  await page.getByRole('button',{name:'Forces & interaction',exact:true}).click();
  await page.getByRole('button',{name:'Apply 10 newtons',exact:true}).click();
  await page.locator('.brace-force-status').filter({hasText:'Compensating 10 N'}).waitFor();
  await equipment.locator('[data-equipment=dumbbells]').click();
  await page.waitForFunction(()=>window.BraceGym.physics().applied==='dumbbells');
  assert(await page.locator('.brace-force-section [role=slider]').evaluateAll(sliders=>sliders.every(s=>Number(s.getAttribute('aria-valuenow'))===0)));
  console.log('PASS equipping clears manual forces without double-counting load');
  await page.getByRole('button',{name:'Apply 5 newtons',exact:true}).click();
  await page.waitForFunction(()=>window.BraceGym.physics().applied==='none');
  assert.deepEqual((await snapshot()).masses,[0,0]);console.log('PASS force preset removes equipment');
  await page.getByRole('button',{name:'Exert force',exact:true}).click();
  await page.getByRole('button',{name:'Apply 8 newtons',exact:true}).click();
  await page.locator('.brace-force-status').filter({hasText:'Exerting 8 N'}).waitFor();
  assert.equal(await equipment.isVisible(),false);
  await page.getByRole('group',{name:'Robot interaction'}).locator('[data-interaction=weights]').click();
  await equipment.waitFor({state:'visible'});
  await page.waitForFunction(()=>document.querySelector('.brace-force-actions [data-mode=compensate]')?.getAttribute('aria-pressed')==='true');
  await page.locator('.brace-force-status').filter({hasText:'No force applied.'}).waitFor({state:'attached'});
  assert(await page.locator('.brace-force-section [role=slider]').evaluateAll(sliders=>sliders.every(s=>Number(s.getAttribute('aria-valuenow'))===0)));
  console.log('PASS visible Exert force action applies a real force; Compensate returns to weights and clears it');
  await page.setViewportSize({width:390,height:844});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  assert.equal(await equipment.evaluate(e=>e.scrollWidth>e.clientWidth),false);
  assert.deepEqual(errors,[]);console.log('PASS mobile layout and no browser errors');
 } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
