import type { CommandTermContext } from './types';

type RespawnTarget = {xy: number[]; mocapPos: number[]; mocapQuat: number[]};
type RespawnWindow = Window & {BraceRespawn?: RespawnTarget};

export function pendingRespawn(): RespawnTarget | undefined {
  return typeof window === 'undefined' ? undefined : (window as RespawnWindow).BraceRespawn;
}

/** Height of the existing slab's top face at a world XY point. */
export function slabHeight(x: number, y: number, pos: ArrayLike<number>, q: ArrayLike<number>, size: ArrayLike<number>): number | null {
  const [w,a,b,c]=Array.from(q);
  const R=[1-2*(b*b+c*c),2*(a*b-w*c),2*(a*c+w*b),
    2*(a*b+w*c),1-2*(a*a+c*c),2*(b*c-w*a),
    2*(a*c-w*b),2*(b*c+w*a),1-2*(a*a+b*b)];
  if(R[8]<.5 || pos[2]<-10)return null;
  const z=pos[2]+size[2]/R[8]-(R[2]*(x-pos[0])+R[5]*(y-pos[1]))/R[8];
  const dx=x-pos[0],dy=y-pos[1],dz=z-pos[2];
  const u=R[0]*dx+R[3]*dy+R[6]*dz,v=R[1]*dx+R[4]*dy+R[7]*dz;
  return Math.abs(u)<=size[0]+1e-6 && Math.abs(v)<=size[1]+1e-6 ? z : null;
}

export function placeRespawn(context: CommandTermContext): void {
  const target=pendingRespawn();
  if(!target)return;
  const {mjModel:m,mjData:d,mujoco}=context;
  if(!m || !d)return;
  d.mocap_pos.set(target.mocapPos);d.mocap_quat.set(target.mocapQuat);
  const bytes=new Uint8Array(m.names),decoder=new TextDecoder();
  const bodyName=(id:number)=>{let end=m.name_bodyadr[id];while(bytes[end])end++;return decoder.decode(bytes.subarray(m.name_bodyadr[id],end));};
  const ramps:number[]=[],feet:number[]=[];
  for(let i=0;i<m.nbody;i++) {
    const name=bodyName(i);
    if(name.startsWith('slope_'))ramps.push(i);
    if(['left_ankle_roll_link','right_ankle_roll_link'].includes(name))feet.push(i);
  }
  const height=(x:number,y:number)=>{
    let h=0;
    for(const body of ramps) {
      const id=m.body_mocapid[body],geom=m.body_geomadr[body];
      if(id<0 || geom<0)continue;
      const z=slabHeight(x,y,d.mocap_pos.subarray(id*3,id*3+3),d.mocap_quat.subarray(id*4,id*4+4),m.geom_size.subarray(geom*3,geom*3+3));
      if(z!==null)h=Math.max(h,z);
    }
    return h;
  };
  d.qpos[0]=target.xy[0];d.qpos[1]=target.xy[1];
  const q=d.qpos.subarray(3,7),yaw=Math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
  d.qpos.set([Math.cos(yaw/2),0,0,Math.sin(yaw/2)],3);
  d.qvel.fill(0);mujoco.mj_forward(m,d);
  let lift=-Infinity;
  for(let g=0;g<m.ngeom;g++) {
    if(!feet.includes(m.geom_bodyid[g]) || m.geom_type[g]!==3 || !m.geom_contype[g])continue;
    const p=d.geom_xpos.subarray(g*3,g*3+3),R=d.geom_xmat.subarray(g*9,g*9+9);
    const radius=m.geom_size[g*3],half=m.geom_size[g*3+1];
    for(const sign of [-1,1]) {
      const x=p[0]+sign*half*R[2],y=p[1]+sign*half*R[5],z=p[2]+sign*half*R[8];
      lift=Math.max(lift,height(x,y)+radius-z);
    }
  }
  if(Number.isFinite(lift))d.qpos[2]+=lift+.008;
  mujoco.mj_forward(m,d);
}
