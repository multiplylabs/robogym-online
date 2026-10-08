const {chromium}=require('playwright'),assert=require('node:assert/strict');
(async()=>{
  const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  try {
    const context=await browser.newContext();
    await context.addInitScript(()=>{
      window.__moves=[];window.__sockets=[];
      const W=window.WebSocket;
      window.WebSocket=class extends W {
        constructor(...args){super(...args);window.__sockets.push(this)}
        send(data){try{const v=JSON.parse(data);if(v.type==='command')window.__moves.push(v)}catch{}super.send(data)}
      };
    });
    const page=await context.newPage(),errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
    await page.waitForFunction(()=>window.BraceGym?.ready,null,{timeout:120000});
    await page.getByRole('button',{name:'Exert force',exact:true}).click();
    await page.getByRole('combobox',{name:'Force direction',exact:true}).selectOption('Z:1');
    await page.getByRole('button',{name:'Apply 8 newtons',exact:true}).click();
    await page.waitForFunction(()=>window.BraceSteering?.motion?.adaptive);
    for(const name of ['s','a','d']) {
      const key=page.locator(`[data-steering-key="${name}"]`);
      assert.equal(await key.getAttribute('aria-disabled'),'false');
    }
    await page.keyboard.down('q');
    await page.waitForFunction(()=>window.__moves.at(-1)?.turn===5);
    assert.equal(await page.locator('.brace-style-override').isVisible(),true);
    let command=await page.evaluate(()=>window.__moves.at(-1));
    assert(command.forward>0);assert.equal(command.lateral,0);
    assert.equal(command.speed_limit,.5);assert.equal(command.movement_profile,'exertion_adaptive');
    await page.keyboard.up('q');
    await page.evaluate(()=>window.__sockets.at(-1).close());
    await page.waitForFunction(()=>window.__sockets.length>=2&&window.__sockets.at(-1).readyState===1);
    await page.waitForFunction(()=>window.__moves.at(-1)?.movement_profile==='exertion_adaptive');
    await page.getByRole('checkbox',{name:'Slope ahead',exact:true}).check();
    await page.waitForFunction(()=>window.__moves.at(-1)?.movement_profile==='exertion_slope_adaptive');
    for(const name of ['Walk forward','Turn left','Turn right']) {
      const key=page.getByRole('button',{name,exact:true});
      assert.equal(await key.getAttribute('aria-disabled'),'false');
    }
    await page.keyboard.down('w');await page.keyboard.down('q');
    await page.waitForFunction(()=>window.__moves.at(-1)?.turn===1);
    command=await page.evaluate(()=>window.__moves.at(-1));
    assert(command.forward>0);assert.equal(command.turn,1);assert.equal(command.speed_limit,.8);
    await page.keyboard.up('w');await page.keyboard.up('q');
    await page.getByRole('checkbox',{name:'Slope ahead',exact:true}).uncheck();
    await page.waitForFunction(()=>!window.BraceSteering.blockedKeys.includes('q'));
    await page.getByRole('button',{name:'Clear force',exact:true}).click();
    await page.waitForFunction(()=>window.BraceSteering.blockedKeys.length===0);
    assert.equal(await page.locator('.brace-style-override').isVisible(),false);
    await page.keyboard.down('q');await page.waitForFunction(()=>window.__moves.at(-1)?.turn===20);
    command=await page.evaluate(()=>window.__moves.at(-1));
    assert.equal(command.movement_profile,null);assert.equal(command.speed_limit,null);
    await page.keyboard.up('q');
    assert.deepEqual(errors,[]);
    console.log('PASS upward moving arc, side/back controls, reconnect profile, broad ramp arc and clear restore');
  } finally {await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
