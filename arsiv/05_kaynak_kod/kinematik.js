function robotPose(shoulderDeg, driveDeg) {
 const sub=(a,b)=>[a[0]-b[0],a[1]-b[1]],add=(a,b)=>[a[0]+b[0],a[1]+b[1]],len=a=>Math.hypot(...a);
 const polar=(r,a)=>[r*Math.cos(a*Math.PI/180),r*Math.sin(a*Math.PI/180)];
 function circle(a,r,b,s){
  const v=sub(b,a),d=len(v);if(d<1e-8||d>r+s+1e-7||d<Math.abs(r-s)-1e-7)return null;
  const u=v.map(x=>x/d),t=(r*r-s*s+d*d)/(2*d),h=Math.sqrt(Math.max(0,r*r-t*t)),c=add(a,u.map(x=>x*t));
  return [c[0]-h*u[1],c[1]+h*u[0]];
 }
 const p={1:[0,61]};
 const refLen=Math.hypot(36.1,17.23);
 p[2]=[-36.1*40/refLen,61+17.23*40/refLen];
 p[5]=add(p[1],polar(77,shoulderDeg));
 p[3]=circle(p[2],85,p[5],40);if(!p[3])return null;
 p[4]=circle(p[3],47,p[5],20);if(!p[4])return null;
 p.V=add(p[1],polar(40,driveDeg));
 p.R=circle(p.V,80,p[5],35);if(!p.R)return null;
 const v=sub(p[5],p.R),d=len(v);p[7]=add(p[5],v.map(x=>x*80/d));
 p[6]=circle(p[4],80,p[7],23);if(!p[6])return null;
 return p;
}
