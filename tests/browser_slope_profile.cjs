const {chromium}=require('playwright'),assert=require('node:assert/strict');
(async()=>{const b=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const c=await b.newContext();await c.addInitScript(()=>{window.__moves=[];window.__sockets=[];const W=window.WebSocket;window.WebSocket=class extends W{constructor(...a){super(...a);window.__sockets.push(this)}send(s){try{const v=JSON.parse(s);if(v.type==='command')window.__moves.push(v)}catch{}super.send(s)}}});
 const p=await c.newPage(),errors=[];p.on('pageerror',e=>errors.push(e.message));
 await p.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8766');await p.waitForFunction(()=>window.BraceGym?.ready,null,{timeout:120000});
 const style=p.locator('.brace-style[aria-label="Slow walk"]'),note=p.locator('.brace-style-override');await style.click();
 await p.getByRole('button',{name:'Exert force',exact:true}).click();await p.getByRole('button',{name:'Apply 5 newtons',exact:true}).click();
 const slope=p.getByRole('checkbox',{name:'Slope ahead',exact:true});await slope.check();
 await p.waitForFunction(()=>window.__moves.at(-1)?.movement_profile==='exertion_slope');
 assert(await note.isVisible());assert.match(await note.innerText(),/Careful/);assert.match(await style.getAttribute('class'),/is-active/);
 await p.keyboard.down('w');await p.waitForFunction(()=>window.__moves.at(-1)?.forward>0);const command=await p.evaluate(()=>window.__moves.at(-1));
 assert.equal(command.movement_profile,'exertion_slope');assert.equal(command.speed_limit,null);assert.equal(command.turn,0);
 // An unsafe request must stop an already-held key, without reducing the hand force.
 await p.getByRole('button',{name:'Apply 8 newtons',exact:true}).click();
 await p.waitForFunction(()=>window.__moves.at(-1)?.forward===0&&window.BraceSteering.blockedKeys.includes('w'));
 const forward=p.locator('[data-steering-key="w"]');assert.equal(await forward.getAttribute('aria-disabled'),'true');assert.match(await forward.getAttribute('title'),/ramp caused falls/);
 assert.equal(await note.isVisible(),false);assert.equal(await p.evaluate(()=>window.BraceGym.force().raw[4]),8);await p.keyboard.up('w');
 await p.evaluate(()=>window.__sockets.at(-1).close());await p.waitForFunction(()=>window.__sockets.length>=2&&window.__sockets.at(-1).readyState===1);
 await p.waitForFunction(()=>window.__moves.at(-1)?.movement_profile==='exertion'&&window.__moves.at(-1)?.forward===0);
 await slope.uncheck();await p.waitForFunction(()=>window.BraceSteering&&!window.BraceSteering.blockedKeys.includes('w'));assert.equal(await note.isVisible(),false);
 await p.keyboard.down('w');await p.waitForFunction(()=>window.__moves.at(-1)?.forward>0);await p.keyboard.up('w');
 assert.match(await style.getAttribute('class'),/is-active/);assert.equal(await p.evaluate(()=>window.BraceGym.force().raw[4]),8);
 await p.getByRole('button',{name:'Clear force',exact:true}).click();await p.waitForFunction(()=>window.__moves.at(-1)?.movement_profile===null);
 assert.deepEqual(errors,[]);console.log('PASS loaded ramp profile, gait override, unsafe held-key stop and tooltip, blocked reconnect, force unchanged and clear/slope-off restoration');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exitCode=1});
