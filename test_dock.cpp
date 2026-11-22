#include "../Source/DockMath.h"
#include <cstdio>
#include <chrono>
using namespace dk;
static const double MU=3.986004418e14, RE=6371010.0;
// orbit from elements -> state (right-handed math; handedness irrelevant)
static void orb(double a,double e,double inc,double raan,double argp,double nu,V3&r,V3&v){
  double p=a*(1-e*e), rn=p/(1+e*cos(nu));
  V3 rp(rn*cos(nu),rn*sin(nu),0), vp(-sqrt(MU/p)*sin(nu), sqrt(MU/p)*(e+cos(nu)),0);
  auto rot=[&](V3 x){ // Rz(raan) Rx(inc) Rz(argp)
    double c=cos(argp),s=sin(argp); x=V3(c*x.x-s*x.y,s*x.x+c*x.y,x.z);
    c=cos(inc);s=sin(inc); x=V3(x.x,c*x.y-s*x.z,s*x.y+c*x.z);
    c=cos(raan);s=sin(raan); x=V3(c*x.x-s*x.y,s*x.x+c*x.y,x.z); return x;};
  r=rot(rp); v=rot(vp);
}
static void run(const char*name,V3 rc,V3 vc,V3 rt,V3 vt,double omegaScale){
  PlanIn in; in.mu=MU; in.rMin=RE+120e3; in.rc=rc; in.vc=vc; in.rt=rt; in.vt=vt;
  in.wp.axis0=unit(V3(0.3,0.5,0.8)); in.wp.standoff=2000;
  // LVLH-held axis => rotates at orbit rate about orbit normal
  V3 h=cross(rt,vt); in.wp.omega=unit(h)*(len(h)/dot(rt,rt))*omegaScale;
  Planner pl; auto t0=std::chrono::steady_clock::now(); pl.start(in);
  int n=0; while(!pl.step(0.01)) n++;
  double el=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
  const Plan&p=pl.result();
  printf("%s: valid=%d slices=%d time=%.2fs\n",name,p.valid,n,el);
  if(!p.valid) return;
  printf("  t0=%.0f s tof=%.0f s N=%d br=%d dv1=%.1f dv2=%.1f cost=%.1f rmin=%.0f km miss=%.1f m\n",p.t0,p.tof,p.nrev,p.branch,p.dv1m,p.dv2m,p.cost,(p.rMinTransfer-RE)/1e3,p.missAtArrival);
  // independent check: propagate chaser, apply dv1, propagate to arrival, compare with waypoint & velocity after dv2
  V3 r,v; kepler(rc,vc,MU,p.t0,r,v); v=v+p.dv1; V3 r2,v2; kepler(r,v,MU,p.tof,r2,v2);
  V3 rt2,vt2; kepler(rt,vt,MU,p.t0+p.tof,rt2,vt2);
  V3 w=in.wp.pos(rt2,p.t0+p.tof), vw=in.wp.vel(rt2,vt2,p.t0+p.tof);
  printf("  arrival pos err %.2f m ; rel vel after dv2 %.3f m/s\n",len(r2-w),len(v2+p.dv2-vw));
}
int main(){
  V3 rc,vc,rt,vt;
  double D=M_PI/180;
  // 1) coplanar, chaser 300 km, target 400 km 20 deg ahead
  orb(RE+300e3,0,51.6*D,0,0,0,rc,vc); orb(RE+400e3,0,51.6*D,0,0,20*D,rt,vt);
  run("coplanar 20deg ahead, lower orbit",rc,vc,rt,vt,1.0);
  // 2) coplanar, target behind
  orb(RE+400e3,0,51.6*D,0,0,-60*D,rt,vt); run("coplanar 60deg behind",rc,vc,rt,vt,1.0);
  // 3) same orbit 5 deg ahead
  orb(RE+300e3,0,51.6*D,0,0,5*D,rt,vt); run("same orbit 5deg ahead",rc,vc,rt,vt,1.0);
  // 4) non-coplanar: chaser 28.5, target 51.6 raan 40 deg
  orb(RE+250e3,0.001,28.5*D,0,0,30*D,rc,vc); orb(RE+410e3,0.0005,51.6*D,40*D,0,100*D,rt,vt);
  run("inclination change 28.5->51.6",rc,vc,rt,vt,1.0);
  orb(RE+250e3,0.001,50.0*D,1.0*D,0,30*D,rc,vc); orb(RE+410e3,0.0005,51.6*D,2.5*D,0,100*D,rt,vt);
  run("small plane diff (1.6 deg inc, 1.5 deg raan)",rc,vc,rt,vt,1.0);
  orb(RE+250e3,0.001,51.6*D,0,0,30*D,rc,vc); orb(RE+410e3,0.0005,51.6*D,0,0,100*D,rt,vt);
  run("coplanar 70deg ahead, 160km lower",rc,vc,rt,vt,1.0);
  run("same, inertial-fixed axis",rc,vc,rt,vt,0.0);
  // 5) inertial-fixed axis
  run("same, inertial-fixed axis",rc,vc,rt,vt,0.0);
  // 6) target 180 deg ahead same alt
  orb(RE+300e3,0,51.6*D,0,0,0,rc,vc); orb(RE+300e3,0,51.6*D,0,0,180*D,rt,vt); run("180deg ahead same alt",rc,vc,rt,vt,1.0);
}
