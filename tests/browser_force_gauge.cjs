// Verify the displayed gauge against the real seven-field exert command contract.
// This DOM-only fixture never opens a generator or changes simulation state.
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
const path=require('node:path');
(async()=>{
 const browser=await chromium.launch({args:['--no-sandbox']});
 try{
  const page=await browser.newPage();
  await page.setContent(`<section class="brace-controls"><button class="brace-disclosure">Advance Force Controls</button>
   <section class="brace-force-section"><h3>Hand Force</h3><div role="slider" aria-label="Right hand X (N)" aria-valuemax="20"></div></section>
   <section class="brace-force-section"><h3>Exert</h3><input type="checkbox"><div role="slider" aria-label="Right hand X (N)" aria-valuemax="8"></div></section></section>`);
  await page.evaluate(()=>{window.BraceGym={ready:true,selected:'none',force:()=>({raw:window.testRequest})};window.BraceExert={};});
  await page.addScriptTag({path:path.resolve('assets/brace-forces.js')});
  const cases=[
   {name:'Mode flag alone produces no phantom hand force',raw:[1,0,0,0,0,0,0],effective:[0,0,0,0,0,0],expected:[]},
   {name:'Right forward request excludes the left mode flag',raw:[1,0,0,0,5,0,0],effective:[0,0,0,5,0,0],expected:[['Right hand','Requested 5.0 N']]},
   {name:'Right downward request includes the last vector component',raw:[1,0,0,0,0,0,-5],effective:[0,0,0,0,0,-5],expected:[['Right hand','Requested 5.0 N']]},
   {name:'Both hands use their own XYZ components',raw:[1,3,4,0,0,0,-5],effective:[3,4,0,0,0,-5],expected:[['Left hand','Requested 5.0 N'],['Right hand','Requested 5.0 N']]},
   {name:'Six-component telemetry remains supported',raw:[0,0,0,5,0,0],effective:[0,0,0,5,0,0],expected:[['Right hand','Requested 5.0 N']]},
  ];
  for(const c of cases){
   await page.waitForTimeout(120);
   await page.evaluate(c=>{window.testRequest=c.raw;window.dispatchEvent(new CustomEvent('brace:force-reading',{detail:{commanded:c.effective,measured:[0,0]}}));},c);
   const actual=await page.locator('.brace-gauge-hand').evaluateAll(rows=>rows.map(row=>[row.querySelector('strong').textContent,row.querySelector('.brace-requested').textContent]));
   assert.deepEqual(actual,c.expected,c.name);console.log('PASS',c.name);
  }
 }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
