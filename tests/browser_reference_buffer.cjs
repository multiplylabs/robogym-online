// Network stalls pause physics; a walking reference must never become a frozen moving pose.
const {chromium}=require('playwright'),assert=require('node:assert/strict');
(async()=>{const browser=await chromium.launch({args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
try{const page=await browser.newPage();const errors=[];page.on('pageerror',e=>errors.push(e.message));
await page.addInitScript(()=>{window.__ReferenceBufferFalls=[];window.addEventListener('brace:auto-respawn',e=>window.__ReferenceBufferFalls.push(e.detail))});
// Force each frame over its 20 ms budget to expose resolved-promise loop starvation.
await page.route('**/brace-equipment-plugin.js*',async route=>{
 const response=await route.fetch(),body=await response.text(),hook='  update(dt) {\n    if (!this.resolved && !this.resolve()) return;';
 assert(body.includes(hook));await route.fulfill({response,body:body.replace(hook,'  update(dt) {\n    const until=performance.now()+25;while(performance.now()<until) {}\n    if (!this.resolved && !this.resolve()) return;')});
});
let hold=false,queue=[];
await page.routeWebSocket(/(?:127\.0\.0\.1|trycloudflare\.com).*$/,ws=>{
 const server=ws.connectToServer();server.onMessage(message=>{if(typeof message!=='string'&&hold)queue.push(()=>ws.send(message));else ws.send(message)});
});
await page.goto(process.env.BRACE_TEST_URL||'http://127.0.0.1:8080/?stream=ws%3A%2F%2F127.0.0.1%3A8765');
await page.waitForFunction(()=>window.BraceGym?.ready&&window.BraceReferenceStream&&!window.BraceReferenceStream.waiting,null,{timeout:120000});
await page.evaluate(()=>{window.__BufferHeartbeatLast=performance.now();window.__BufferHeartbeatMax=0;window.__BufferHeartbeatCount=0;setInterval(()=>{const t=performance.now();window.__BufferHeartbeatCount++;window.__BufferHeartbeatMax=Math.max(window.__BufferHeartbeatMax,t-window.__BufferHeartbeatLast);window.__BufferHeartbeatLast=t},100)});
await page.getByRole('button',{name:'Exert force',exact:true}).click();
await page.getByRole('button',{name:'Apply 8 newtons',exact:true}).click();
await page.keyboard.down('q');hold=true;
await page.waitForFunction(()=>window.BraceReferenceStream?.waiting,null,{timeout:30000});
const before=await page.evaluate(()=>({time:window.BraceGym.force().time,stream:{...window.BraceReferenceStream},root:window.BraceGym.force().root,callbacks:window.__BufferHeartbeatCount}));
await page.waitForTimeout(1200);
const stopped=await page.evaluate(()=>({time:window.BraceGym.force().time,stream:{...window.BraceReferenceStream},root:window.BraceGym.force().root,callbacks:window.__BufferHeartbeatCount}));
assert(stopped.callbacks>before.callbacks);assert.equal(stopped.time,before.time);assert.deepEqual(stopped.root,before.root);assert(stopped.stream.future<stopped.stream.horizon);
assert(await page.locator('.brace-stream-status').isVisible());
hold=false;for(const deliver of queue)deliver();queue=[];
await page.waitForFunction(t=>window.BraceGym.force().time>t+2,before.time,{timeout:30000});
await page.keyboard.up('q');assert.deepEqual(errors,[]);
const resumed=await page.evaluate(()=>({falls:window.__ReferenceBufferFalls,raw:window.BraceGym.force().raw,maxCallbackGap:window.__BufferHeartbeatMax,callbacks:window.__BufferHeartbeatCount}));
assert.deepEqual(resumed.falls,[]);assert.equal(resumed.raw[0],1);assert(resumed.raw.slice(1).some(v=>Math.abs(v-8)<.01));
assert(resumed.callbacks>stopped.callbacks);
console.log(`PASS frame overruns keep browser callbacks live (max gap ${Math.round(resumed.maxCallbackGap)} ms); delayed reference freezes clock/pose and resumes exertion arcs without a fall`);
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
