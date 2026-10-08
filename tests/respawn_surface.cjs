const assert=require('node:assert/strict'),fs=require('node:fs'),Module=require('node:module');
const mod=new Module(`${process.env.NODE_PATH}/respawn-test.cjs`);mod.filename=mod.id;mod.paths=Module._nodeModulePaths(process.env.NODE_PATH);
mod._compile(require('esbuild').transformSync(fs.readFileSync('assets/respawn.ts','utf8'),{loader:'ts',format:'cjs'}).code,mod.filename);
const {slabHeight}=mod.exports;
assert.equal(slabHeight(2,3,[2,3,.44],[1,0,0,0],[1,2,.06]),.5);
assert.equal(slabHeight(4,3,[2,3,.44],[1,0,0,0],[1,2,.06]),null);
assert.equal(slabHeight(0,0,[0,0,-80],[1,0,0,0],[1,2,.06]),null);
for(const yaw of [0,Math.PI/2,Math.PI])for(const pitch of [-.2,.2]) {
  const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2),cp=Math.cos(pitch/2),sp=Math.sin(pitch/2);
  const q=[cy*cp,-sy*sp,cy*sp,sy*cp],pos=[5,-3,.4];
  const x=pos[0]+.5*Math.cos(yaw),y=pos[1]+.5*Math.sin(yaw);
  assert(Math.abs(slabHeight(x,y,pos,q,[1,2,.06])-(.4+.06/Math.cos(pitch)-.5*Math.tan(pitch)))<1e-10);
}
console.log('PASS terrain top heights on yawed ascent/descent, plateau, edges and parked ramps');
