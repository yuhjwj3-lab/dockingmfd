#include "../Source/DockCtl.h"
#include <cstdio>
using namespace dk;
static const double MU=3.986004418e14, RE=6371010.0;
static V3 grav(const V3&r){ double n=len(r); return r*(-MU/(n*n*n)); }
int main(){
  for(int scen=0;scen<6;scen++){
    double a=RE+400e3, n=sqrt(MU/(a*a*a));
    V3 rt(a,0,0), vt(0,a*n,0); V3 nrm(0,0,1);
    V3 d0 = scen<3? V3(0,1,0) : V3(1,0,0);            // port axis: along V-bar, or radial
    V3 omega = nrm*n;                                   // LVLH held
    // initial chaser relative to port: in front along axis 2000 m with lateral offsets
    V3 rel0[6]={ d0*2000+V3(0,0,300), d0*2000+V3(0,0,-600)+nrm*0, d0*800+V3(0,0,0), d0*(-500)+V3(0,0,0)+V3(0,0,200), V3(0,2000,0)+V3(500,0,0), d0*1500+V3(200,200,200)};
    V3 relv0[6]={V3(),V3(),V3(),V3(),V3(),V3()};
    V3 rc=rt+rel0[scen], vc=vt+relv0[scen]+cross(omega,rel0[scen])*0;
    double dt=0.25, t=0, fuel=0, aCap=0.25;
    bool docked=false; V3 rho,rhoD; double s=1e9;
    double tmax=5000;
    while(t<tmax){
      V3 rtn,vtn; kepler(rt,vt,MU,t,rtn,vtn);
      V3 dT=rotateAbout(d0,nrm,n*t);
      rho=rc-rtn; rhoD=vc-vtn;                       // port at CoG (arm 0)
      FunnelOut f=funnelVelocity(rho,dT,1.0);
      V3 vDes; V3 tgtW; 
      if(f.inFunnel) vDes=f.vDes;
      else { // reposition: go to point 800 m ahead on axis
        V3 W=dT*800; V3 toW=W-rho; vDes=goToVelocity(toW,6.0,aCap*0.8);
      }
      V3 gdiff=grav(rc)-grav(rtn);
      V3 a=accelToTrack(vDes,rhoD,4.0,aCap*0.8)-gdiff;
      double an=len(a); if(an>aCap) a=a*(aCap/an);
      fuel+=len(a)*dt;
      // integrate exactly-ish (velocity verlet)
      V3 acc=grav(rc)+a;
      rc=rc+vc*dt+acc*(0.5*dt*dt);
      V3 acc2=grav(rc)+a; vc=vc+(acc+acc2)*(0.5*dt);
      t+=dt;
      V3 rtn2,vtn2; kepler(rt,vt,MU,t,rtn2,vtn2); V3 dT2=rotateAbout(d0,nrm,n*t);
      V3 rho2=rc-rtn2; s=dot(rho2,dT2);
      if(len(rho2)<0.3){docked=true;break;}
      if(s<-1.0 && len(rho2)<50) break;
    }
    V3 rtn,vtn; kepler(rt,vt,MU,t,rtn,vtn); V3 dT=rotateAbout(d0,nrm,n*t);
    V3 rr=rc-rtn; V3 rv=vc-vtn; double sv=dot(rv,dT);
    printf("scen %d: docked=%d t=%.0fs range=%.2f m s=%.2f lat=%.2f closing=%.3f m/s lateralV=%.3f fuel dv=%.1f m/s\n",scen,docked,t,len(rr),dot(rr,dT),len(rr-dT*dot(rr,dT)),-sv,len(rv-dT*sv),fuel);
  }
}
