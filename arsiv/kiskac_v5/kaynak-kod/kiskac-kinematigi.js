const GRIPPER_CONFIG={forwardClosed:47.5,verticalClosed:-37,gapMin:4,gapMax:79,referencePivotSpacing:18.1,referenceFingerScale:.84,referenceContactHeight:6.045};
function robotGripper(p,baseDeg,openingPercent=0){
 const c=GRIPPER_CONFIG,opening=Math.max(0,Math.min(100,openingPercent));
 const gap=c.gapMin+(c.gapMax-c.gapMin)*opening/100,half=c.referencePivotSpacing/2;
 const a=Math.atan2(p[6][1]-p[7][1],p[6][0]-p[7][0])-Math.atan2(17.23,10.16),f=[Math.cos(a),Math.sin(a)],u=[-Math.sin(a),Math.cos(a)],yaw=baseDeg*Math.PI/180;
 const world=local=>{const x=p[7][0]+f[0]*local[0]+u[0]*local[1],y=p[7][1]+f[1]*local[0]+u[1]*local[1],z=local[2];return [x*Math.cos(yaw)-z*Math.sin(yaw),y,x*Math.sin(yaw)+z*Math.cos(yaw)];};
 const fingers=[{key:'fingerR',sign:1,pivotY:10.53,contactY:1.6},{key:'fingerL',sign:-1,pivotY:3.63,contactY:12.63}].map(k=>{
  const dx=(47.5-4.5)*c.referenceFingerScale,dy=k.sign*(k.contactY-k.pivotY)*c.referenceFingerScale,r=Math.hypot(dx,dy),initialTarget=c.gapMin/2-half;
  const pivotForward=c.forwardClosed-Math.sqrt(r*r-initialTarget*initialTarget),target=gap/2-half;
  const rotation=Math.asin(target/r)-Math.atan2(dy,dx);
  const local=q=>{const x=(47.5-q[0])*c.referenceFingerScale,z=k.sign*(q[1]-k.pivotY)*c.referenceFingerScale;return [pivotForward+x*Math.cos(rotation)-z*Math.sin(rotation),c.verticalClosed+(q[2]-c.referenceContactHeight)*c.referenceFingerScale,k.sign*(half+x*Math.sin(rotation)+z*Math.cos(rotation))];};
  const contact=local([4.5,k.contactY,c.referenceContactHeight]);
  return {...k,pivotForward,rotation,radius:r,local,world:q=>world(local(q)),contact};
 });
 const K2=fingers[0].contact,K3=fingers[1].contact,M=K2.map((v,i)=>(v+K3[i])/2);
 return {opening,gap,angle:a,fingers,world,localPoints:{K1:[0,0,0],K2,K3,M},worldPoints:{K1:world([0,0,0]),K2:world(K2),K3:world(K3),M:world(M)}};
}
