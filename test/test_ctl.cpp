#include "../Source/DockCtl.h"
#include <cstdio>
#include <random>
using namespace dk;
static Mat3 expRot(const V3 &rv){ double th=len(rv); Mat3 R; if(th<1e-12) return R; V3 k=rv/th; double c=cos(th),s=sin(th);
  // Rodrigues with algebraic cross: columns = rotate basis vectors
  V3 ex=rotateAbout(V3(1,0,0),k,th), ey=rotateAbout(V3(0,1,0),k,th), ez=rotateAbout(V3(0,0,1),k,th);
  R.m[0][0]=ex.x;R.m[1][0]=ex.y;R.m[2][0]=ex.z; R.m[0][1]=ey.x;R.m[1][1]=ey.y;R.m[2][1]=ey.z; R.m[0][2]=ez.x;R.m[1][2]=ez.y;R.m[2][2]=ez.z; return R; }
int main(){
  std::mt19937 rng(5); std::uniform_real_distribution<double> U(-1,1);
  int fails=0;
  for(int trial=0;trial<40;trial++){
    double gc[3]={ (0.1+0.5*fabs(U(rng)))*(U(rng)>0?1:-1), (0.1+0.5*fabs(U(rng)))*(U(rng)>0?1:-1), (0.1+0.5*fabs(U(rng)))*(U(rng)>0?1:-1)};
    // random initial attitude and rates
    Mat3 R = expRot(V3(U(rng),U(rng),U(rng))*3.0);
    V3 w(U(rng)*0.05,U(rng)*0.05,U(rng)*0.05);
    V3 L(0,0,1), M(0,1,0), D=unit(V3(U(rng),U(rng),U(rng))), Md=unit(cross(D,V3(0.3,0.5,0.7)));
    // Md must be perpendicular to D
    double dt=(trial%2)?0.05:0.2; double errP=9,errR=9; Mat3 Rprev=R; V3 wm;
    for(int k=0;k<(int)(240/dt);k++){
      wm = bodyRateFromMatrices(Rprev,R,dt); if(k==0) wm=w;
      V3 u=attitudeControl(R,wm,L,D,true,M,Md,gc,0.15,errP,errR);
      V3 acc(gc[0]*u.x,gc[1]*u.y,gc[2]*u.z);
      w=w+acc*dt; Rprev=R; R=mulMM(R,expRot(w*dt));
    }
    bool ok=errP<0.01&&errR<0.02&&len(w)<2e-3; if(!ok) fails++;
    printf("trial %2d dt=%.2f gains %+.2f %+.2f %+.2f -> errP=%.3f deg errR=%.3f deg w=%.5f %s\n",trial,dt,gc[0],gc[1],gc[2],errP*57.3,errR*57.3,len(w),ok?"ok":"FAIL");
  }
  printf("fails=%d\n",fails);
}
