/* Standard DH, matching src/cdlbt/kinematics.c: Rz(theta) Tz(d) Tx(a) Rx(alpha).
 * Matrices are row-major and multiply column vectors. Joint i rotates about
 * z of the PREVIOUS frame, not the z axis of its resulting link frame.
 */
const DHMath = (() => {
  const identity = () => [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
  const mul = (a,b) => a.map(row => b[0].map((_,j) => row.reduce((s,v,k) => s+v*b[k][j],0)));
  const point = (m,p=[0,0,0]) => m.slice(0,3).map(row => row[3]+p.reduce((s,v,k)=>s+v*row[k],0));
  const axis = (m,k) => m.slice(0,3).map(row => row[k]);
  function dh(row,theta) {
    const alpha=row.alpha_pi*Math.PI,c=Math.cos(theta),s=Math.sin(theta),ca=Math.cos(alpha),sa=Math.sin(alpha);
    return [[c,-s*ca,s*sa,row.a*c],[s,c*ca,-c*sa,row.a*s],[0,sa,ca,row.d],[0,0,0,1]];
  }
  function fk(model,q) {
    let t=model.world.map(row=>row.slice());
    const frames=[t], links=[], joints=[];
    model.moving.forEach((row,i)=>{
      joints.push({origin:point(t),axis:axis(t,2)});
      const corner=point(t,[0,0,row.d]);
      const next=mul(t,dh(row,q[i]+(row.theta_pi??0)*Math.PI));
      links.push({start:point(t),corner,end:point(next)});
      frames.push(next); t=next;
    });
    const tool=mul(t,dh(model.tool,model.tool.theta_pi*Math.PI));
    const toolLink={start:point(t),corner:point(t,[0,0,model.tool.d]),end:point(tool)};
    return {frames,joints,links,tool,toolLink};
  }
  return {identity,mul,point,axis,dh,fk};
})();
if (typeof window !== 'undefined') window.DHMath=DHMath;
