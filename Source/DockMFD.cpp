// DOCK MFD for Orbiter 2016: autonomous rendezvous and docking with any other vessel orbiting the same body.
//
// HONEST NOTE: written and tested only OUTSIDE Orbiter (orbital maths and the control laws were run in an offline
// simulation; the Orbiter glue was only syntax-checked). Expect to need a flight or two of tuning. Send DockMFD_flight.log.
//
// Flow:  pick target (UP/DN/SEL)  ->  PLN (plan: burn 1, coast, burn 2)  ->  ENG twice (confirm)  ->
//        calibrate RCS -> burn 1 -> coast with mid-course corrections -> braking burn -> closing -> docking funnel -> docked.
#define STRICT
#define NOMINMAX
#define ORBITER_MODULE
#include "Orbitersdk.h"
#include "DrawAPI.h"
#include "MFDAPI.h"
#include "DockMath.h"
#include "DockCtl.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <map>
#include <vector>
#include <algorithm>

typedef dk::V3 V3;
typedef dk::Mat3 Mat3;

static const double DEG2 = 180.0 / 3.14159265358979323846;
static const DWORD C_BG = RGB(0, 0, 0), C_BAND = RGB(0, 70, 0), C_ACC = RGB(0, 230, 0), C_LAB = RGB(0, 150, 0), C_VAL = RGB(0, 235, 0),
    C_AMBER = RGB(255, 191, 0), C_RED = RGB(255, 60, 60), C_LINE = RGB(0, 70, 0), C_CYAN = RGB(120, 220, 255);
static int g_mode = -1;

static V3 toV(const VECTOR3 &a) { return V3(a.x, a.y, a.z); }
static VECTOR3 fromV(const V3 &a) { VECTOR3 r; r.x = a.x; r.y = a.y; r.z = a.z; return r; }
static Mat3 toM(const MATRIX3 &M)
{
    Mat3 R;
    R.m[0][0] = M.m11; R.m[0][1] = M.m12; R.m[0][2] = M.m13;
    R.m[1][0] = M.m21; R.m[1][1] = M.m22; R.m[1][2] = M.m23;
    R.m[2][0] = M.m31; R.m[2][1] = M.m32; R.m[2][2] = M.m33;
    return R;
}

// ============================================================================ state
enum Phase {
    PH_IDLE, PH_PLANNING, PH_READY, PH_CAL, PH_COAST, PH_ORIENT1, PH_BURN1, PH_TRANSIT, PH_MCC_ORIENT, PH_MCC_BURN,
    PH_BRAKE_ORIENT, PH_BRAKE, PH_NEAR, PH_DOCKED, PH_ABORT
};
static const char *phaseName(int p)
{
    static const char *n[] = { "IDLE", "PLANNING", "PLAN READY", "RCS CHECK", "COAST TO BURN", "ORIENT BURN 1", "BURN 1", "TRANSFER COAST",
        "ORIENT CORR", "CORRECTION BURN", "ORIENT BRAKE", "BRAKING BURN", "CLOSE APPROACH", "DOCKED", "ABORTED" };
    return n[p];
}

struct Cand { OBJHANDLE h; char name[40]; char cls[28]; double dist, vrel; int nFree, nTot; };

struct Caps {
    bool ok = false;
    double mainF = 0, mainIsp = 0;
    V3 mainAxis;
    double lin[6] = { 0, 0, 0, 0, 0, 0 };      // max acceleration +x,-x,+y,-y,+z,-z [m/s^2]
    double aLin = 0;                           // conservative
    double fuel = 0, mass = 0, dvAvail = 0;
    double rcsFuel = 0;
};

struct Snap {
    bool ok = false;
    double t = 0, dt = 0;
    V3 rc, vc, rt, vt;
    Mat3 Rc, Rt;
    V3 wc, wtG;                       // chaser body rate (local), target rate (global)
    V3 pc, dc, mc, pt, dtp, mt;       // ports (local)
    V3 dT, mT, rho, rhoDot;
    double s = 0, lat = 0, range = 0, closing = 0;
};

struct Auto {
    int ph = PH_IDLE;
    OBJHANDLE hCh = NULL, hTgt = NULL, hRef = NULL;
    int cPort = 0, tPort = 0;
    double standoff = 2000.0;
    bool fast = false, autoWarp = false;
    char msg[120] = "SELECT A TARGET";
    char tgtName[40] = "";
    std::vector<Cand> cands;
    int cur = 0;
    double candT = -1e9;
    // plan
    dk::Planner planner;
    dk::Plan plan;
    double tRef = 0;
    bool closeOnly = false;
    double mu = 0, R = 0, rMin = 0;
    double tIgn1 = 0, tb1 = 0, tb2 = 0, tArr = 0;
    double engArmT = -1e9;
    // attitude calibration
    bool cal = false;
    double gcal[3] = { 0.2, 0.2, 0.2 };
    int calAxis = 0, calStage = 0;
    double calT = 0, calW0 = 0, calDur = 0;
    // burn
    V3 vReq;                   // target velocity for current burn
    V3 holdD;
    bool holdSet = false;
    double burnTol = 0.1;
    double nextMcc = 0;
    // rate trackers
    Mat3 prevRc, prevRt; bool haveRc = false, haveRt = false;
    V3 wcBody, wtGlobal;
    Snap S;
    Caps C;
    double errP = 0, errR = 0;
    double logT = -1e9;
    char sub[40] = "";
    double stateAge = 0;
    OBJHANDLE capsFor = NULL;
    double dockContactT = -1;
} A;

// ============================================================================ helpers
static void setMsg(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt); vsnprintf(A.msg, sizeof(A.msg) - 1, fmt, ap); va_end(ap);
}

static VESSEL *chaser() { return (A.hCh && oapiIsVessel(A.hCh)) ? oapiGetVesselInterface(A.hCh) : NULL; }
static VESSEL *target() { return (A.hTgt && oapiIsVessel(A.hTgt)) ? oapiGetVesselInterface(A.hTgt) : NULL; }

static bool freePort(VESSEL *v, int i)
{
    DOCKHANDLE d = v->GetDockHandle(i);
    return d && v->GetDockStatus(d) == NULL;
}
static int nextFreePort(VESSEL *v, int cur)
{
    int n = (int)v->DockCount();
    for (int k = 1; k <= n; k++) { int i = (cur + k) % n; if (freePort(v, i)) return i; }
    return cur;
}
static int firstFreePort(VESSEL *v)
{
    int n = (int)v->DockCount();
    for (int i = 0; i < n; i++) if (freePort(v, i)) return i;
    return -1;
}
static bool portInfo(VESSEL *v, int idx, V3 &pos, V3 &dir, V3 &rot)
{
    DOCKHANDLE d = v->GetDockHandle(idx);
    if (!d) return false;
    VECTOR3 p, di, r;
    v->GetDockParams(d, p, di, r);
    pos = toV(p); dir = dk::unit(toV(di)); rot = dk::unit(toV(r));
    return true;
}

// ---------------------------------------------------------------- ship capabilities
static void computeCaps(VESSEL *v)
{
    Caps c;
    c.mass = v->GetMass();
    V3 sum; double ispW = 0, F = 0;
    std::vector<PROPELLANT_HANDLE> props;
    DWORD n = v->GetGroupThrusterCount(THGROUP_MAIN);
    for (DWORD i = 0; i < n; i++) {
        THRUSTER_HANDLE th = v->GetGroupThruster(THGROUP_MAIN, i);
        double f = v->GetThrusterMax0(th);
        VECTOR3 d; v->GetThrusterDir(th, d);
        sum = sum + dk::unit(toV(d)) * f; F += f;
        ispW += f * v->GetThrusterIsp0(th);
        PROPELLANT_HANDLE ph = v->GetThrusterResource(th);
        if (ph && std::find(props.begin(), props.end(), ph) == props.end()) props.push_back(ph);
    }
    if (F > 0) {
        c.mainAxis = dk::unit(sum); c.mainF = dk::dot(sum, c.mainAxis); c.mainIsp = ispW / F;
        for (size_t i = 0; i < props.size(); i++) c.fuel += v->GetPropellantMass(props[i]);
        if (c.mainIsp > 100 && c.fuel > 0 && c.fuel < c.mass) c.dvAvail = c.mainIsp * log(c.mass / (c.mass - c.fuel));
    }
    static const THGROUP_TYPE lg[6] = { THGROUP_ATT_RIGHT, THGROUP_ATT_LEFT, THGROUP_ATT_UP, THGROUP_ATT_DOWN, THGROUP_ATT_FORWARD, THGROUP_ATT_BACK };
    static const double ax[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    double mn = 1e9;
    std::vector<PROPELLANT_HANDLE> rp;
    for (int g = 0; g < 6; g++) {
        DWORD k = v->GetGroupThrusterCount(lg[g]);
        double f = 0;
        for (DWORD i = 0; i < k; i++) {
            THRUSTER_HANDLE th = v->GetGroupThruster(lg[g], i);
            VECTOR3 d; v->GetThrusterDir(th, d);
            double pr = dk::dot(dk::unit(toV(d)), V3(ax[g][0], ax[g][1], ax[g][2]));
            if (pr > 0) f += v->GetThrusterMax0(th) * pr;
            PROPELLANT_HANDLE ph = v->GetThrusterResource(th);
            if (ph && std::find(rp.begin(), rp.end(), ph) == rp.end()) rp.push_back(ph);
        }
        c.lin[g] = c.mass > 0 ? f / c.mass : 0;
        mn = (std::min)(mn, c.lin[g]);
    }
    for (size_t i = 0; i < rp.size(); i++) c.rcsFuel += v->GetPropellantMass(rp[i]);
    c.aLin = mn < 1e8 ? mn : 0;
    c.ok = (c.aLin > 1e-4);
    A.C = c; A.capsFor = v->GetHandle();
}

static double burnTime(double dv)
{
    const Caps &c = A.C;
    if (c.mainF <= 0 || c.mainIsp <= 0) return 1e9;
    double m = c.mass;
    return m * c.mainIsp / c.mainF * (1.0 - exp(-dv / c.mainIsp));
}

// ---------------------------------------------------------------- thruster output (per-thruster accumulation so
// that rotation and translation commands which share RCS jets add up instead of overwriting each other)
static const THGROUP_TYPE ROTG[6] = { THGROUP_ATT_PITCHUP, THGROUP_ATT_PITCHDOWN, THGROUP_ATT_YAWRIGHT, THGROUP_ATT_YAWLEFT, THGROUP_ATT_BANKRIGHT, THGROUP_ATT_BANKLEFT };
static const THGROUP_TYPE LING[6] = { THGROUP_ATT_RIGHT, THGROUP_ATT_LEFT, THGROUP_ATT_UP, THGROUP_ATT_DOWN, THGROUP_ATT_FORWARD, THGROUP_ATT_BACK };

static void applyRCS(VESSEL *v, const V3 &uRot, const V3 &aLinLoc)
{
    double rot[6] = { (std::max)(uRot.x, 0.0), (std::max)(-uRot.x, 0.0), (std::max)(uRot.y, 0.0), (std::max)(-uRot.y, 0.0), (std::max)(uRot.z, 0.0), (std::max)(-uRot.z, 0.0) };
    double lin[6] = { 0, 0, 0, 0, 0, 0 };
    const Caps &c = A.C;
    double av[3] = { aLinLoc.x, aLinLoc.y, aLinLoc.z };
    for (int i = 0; i < 3; i++) {
        int p = 2 * i, m = 2 * i + 1;
        if (av[i] > 0 && c.lin[p] > 1e-6) lin[p] = dk::clampd(av[i] / c.lin[p], 0, 1);
        else if (av[i] < 0 && c.lin[m] > 1e-6) lin[m] = dk::clampd(-av[i] / c.lin[m], 0, 1);
    }
    std::map<THRUSTER_HANDLE, double> lv;
    for (int i = 0; i < 6; i++) {
        DWORD n = v->GetGroupThrusterCount(ROTG[i]);
        for (DWORD k = 0; k < n; k++) lv[v->GetGroupThruster(ROTG[i], k)] += rot[i];
        n = v->GetGroupThrusterCount(LING[i]);
        for (DWORD k = 0; k < n; k++) lv[v->GetGroupThruster(LING[i], k)] += lin[i];
    }
    for (std::map<THRUSTER_HANDLE, double>::iterator it = lv.begin(); it != lv.end(); ++it)
        v->SetThrusterLevel(it->first, (std::min)(1.0, it->second));
}
static void zeroAll(VESSEL *v)
{
    if (!v) return;
    applyRCS(v, V3(), V3());
    v->SetThrusterGroupLevel(THGROUP_MAIN, 0); v->SetThrusterGroupLevel(THGROUP_RETRO, 0); v->SetThrusterGroupLevel(THGROUP_HOVER, 0);
}
static void mainLevel(VESSEL *v, double l)
{
    v->SetThrusterGroupLevel(THGROUP_MAIN, dk::clampd(l, 0, 1));
    v->SetThrusterGroupLevel(THGROUP_RETRO, 0);
}

// ---------------------------------------------------------------- log
static void logLine(const char *fmt, ...)
{
    FILE *f = fopen("DockMFD_flight.log", "a");
    if (!f) return;
    va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
    fputc('\n', f); fclose(f);
}

// ============================================================================ state gathering
static bool gather(double dt)
{
    Snap &S = A.S; S.ok = false;
    VESSEL *v = chaser(), *tv = target();
    if (!v || !tv || !A.hRef) return false;
    VECTOR3 p, q; MATRIX3 M;
    v->GetRelativePos(A.hRef, p); v->GetRelativeVel(A.hRef, q);
    S.rc = toV(p); S.vc = toV(q);
    oapiGetRelativePos(A.hTgt, A.hRef, &p); oapiGetRelativeVel(A.hTgt, A.hRef, &q);
    S.rt = toV(p); S.vt = toV(q);
    v->GetRotationMatrix(M); S.Rc = toM(M);
    oapiGetRotationMatrix(A.hTgt, &M); S.Rt = toM(M);
    S.t = oapiGetSimTime(); S.dt = dt;
    if (dt > 1e-6) {
        if (A.haveRc) A.wcBody = dk::bodyRateFromMatrices(A.prevRc, S.Rc, dt);
        if (A.haveRt) A.wtGlobal = dk::mulM(S.Rt, dk::bodyRateFromMatrices(A.prevRt, S.Rt, dt));
        A.prevRc = S.Rc; A.prevRt = S.Rt; A.haveRc = A.haveRt = true;
    }
    S.wc = A.wcBody; S.wtG = A.wtGlobal;
    if (!portInfo(v, A.cPort, S.pc, S.dc, S.mc) || !portInfo(tv, A.tPort, S.pt, S.dtp, S.mt)) return false;
    V3 armT = dk::mulM(S.Rt, S.pt), armC = dk::mulM(S.Rc, S.pc);
    V3 Pt = S.rt + armT, Pc = S.rc + armC;
    V3 vPt = S.vt + dk::cross(S.wtG, armT);
    V3 vPc = S.vc + dk::cross(dk::mulM(S.Rc, S.wc), armC);
    S.rho = Pc - Pt; S.rhoDot = vPc - vPt;
    S.dT = dk::unit(dk::mulM(S.Rt, S.dtp)); S.mT = dk::mulM(S.Rt, S.mt);
    S.s = dk::dot(S.rho, S.dT);
    S.lat = dk::len(S.rho - S.dT * S.s);
    S.range = dk::len(S.rho);
    S.closing = -dk::dot(S.rhoDot, S.dT);
    S.ok = true;
    return true;
}

// waypoint on the target's port axis, evaluated from the LIVE target state
static void wpState(double tAbs, V3 &pos, V3 &vel)
{
    const Snap &S = A.S;
    double dtp = tAbs - S.t;
    V3 rt, vt;
    dk::kepler(S.rt, S.vt, A.mu, dtp, rt, vt);
    dk::Waypoint w; w.axis0 = S.dT; w.omega = S.wtG; w.standoff = A.standoff;
    pos = w.pos(rt, dtp); vel = w.vel(rt, vt, dtp);
}

// ============================================================================ candidate list
static void refreshCands()
{
    VESSEL *v = oapiGetFocusInterface();
    if (A.ph >= PH_CAL && A.ph < PH_DOCKED) v = chaser();
    if (!v) return;
    OBJHANDLE hv = v->GetHandle(), ref = v->GetGravityRef();
    A.cands.clear();
    VECTOR3 pc, vc; v->GetRelativePos(ref, pc); v->GetRelativeVel(ref, vc);
    for (DWORD i = 0; i < oapiGetVesselCount(); i++) {
        OBJHANDLE h = oapiGetVesselByIndex(i);
        if (h == hv) continue;
        VESSEL *t = oapiGetVesselInterface(h);
        if (!t || t->GetGravityRef() != ref) continue;
        if (t->GetFlightStatus() & 1) continue;                  // landed
        int nt = (int)t->DockCount(), nf = 0;
        for (int k = 0; k < nt; k++) if (freePort(t, k)) nf++;
        if (nf == 0) continue;
        Cand c; c.h = h;
        snprintf(c.name, 39, "%s", t->GetName()); snprintf(c.cls, 27, "%s", t->GetClassName());
        VECTOR3 rp, rv; v->GetRelativePos(h, rp); v->GetRelativeVel(h, rv);
        c.dist = length(rp); c.vrel = length(rv); c.nFree = nf; c.nTot = nt;
        A.cands.push_back(c);
    }
    std::sort(A.cands.begin(), A.cands.end(), [](const Cand &a, const Cand &b) { return a.dist < b.dist; });
    if (A.cur >= (int)A.cands.size()) A.cur = (int)A.cands.size() - 1;
    if (A.cur < 0) A.cur = 0;
}

static void selectTarget()
{
    if (A.ph >= PH_PLANNING && A.ph < PH_DOCKED && A.ph != PH_READY) { setMsg("ABORT FIRST TO CHANGE TARGET"); return; }
    refreshCands();
    if (A.cands.empty()) { setMsg("NO TARGET WITH A FREE PORT IN THIS ORBIT"); return; }
    VESSEL *v = oapiGetFocusInterface();
    Cand &c = A.cands[A.cur];
    A.hCh = v->GetHandle(); A.hTgt = c.h; A.hRef = v->GetGravityRef();
    snprintf(A.tgtName, 39, "%s", c.name);
    VESSEL *t = target();
    A.cPort = (std::max)(0, firstFreePort(v)); A.tPort = (std::max)(0, firstFreePort(t));
    if (firstFreePort(v) < 0) { setMsg("YOUR SHIP HAS NO FREE DOCKING PORT"); A.hTgt = NULL; return; }
    A.haveRc = A.haveRt = false; A.wcBody = V3(); A.wtGlobal = V3();
    A.ph = PH_IDLE; A.plan = dk::Plan();
    A.mu = GGRAV * oapiGetMass(A.hRef); A.R = oapiGetSize(A.hRef);
    const ATMCONST *ac = oapiGetPlanetAtmConstants(A.hRef);
    A.rMin = A.R + (ac ? (std::max)(90e3, 0.6 * ac->altlimit) : 15e3);
    computeCaps(v);
    setMsg("TARGET %s - PRESS PLN", A.tgtName);
}

// ============================================================================ planning
static void startPlan()
{
    VESSEL *v = chaser(), *tv = target();
    if (!v || !tv) { setMsg("SELECT A TARGET FIRST"); return; }
    if (A.ph >= PH_CAL && A.ph < PH_DOCKED) { setMsg("ALREADY ENGAGED - ABORT FIRST"); return; }
    if (v->DockingStatus(A.cPort) || !freePort(tv, A.tPort)) { setMsg("A SELECTED PORT IS OCCUPIED"); return; }
    if (v->GetFlightStatus() & 1) { setMsg("SHIP IS LANDED"); return; }
    computeCaps(v);
    if (!A.C.ok) { setMsg("SHIP HAS NO LINEAR RCS - CANNOT DOCK"); return; }
    if (A.C.mainF <= 0) { setMsg("SHIP HAS NO MAIN ENGINE (RCS-ONLY WORKS ONLY WHEN CLOSE)"); }
    if (!gather(0)) { setMsg("CANNOT READ PORT DATA"); return; }
    A.closeOnly = false;
    const Snap &S = A.S;
    double dist = dk::len(S.rc - S.rt), vrel = dk::len(S.vc - S.vt);
    A.tRef = S.t;
    if (dist < 12000.0 && vrel < 30.0) {
        A.closeOnly = true; A.ph = PH_READY; A.plan = dk::Plan();
        setMsg("CLOSE RANGE %.1f KM - DIRECT APPROACH. ENG TWICE TO START", dist / 1000.0);
        return;
    }
    {
        double E = dk::dot(S.vc, S.vc) / 2 - A.mu / dk::len(S.rc);
        if (E >= 0) { setMsg("NOT IN A BOUND ORBIT - GET INTO ORBIT FIRST"); return; }
        double a = -A.mu / (2 * E); V3 h = dk::cross(S.rc, S.vc);
        double e = sqrt((std::max)(0.0, 1.0 + 2.0 * E * dk::dot(h, h) / (A.mu * A.mu)));
        double rp = a * (1 - e);
        if (rp < A.rMin - 30e3) { setMsg("PERIAPSIS %.0f KM IS TOO LOW - CIRCULARISE FIRST", (rp - A.R) / 1000.0); return; }
    }
    dk::PlanIn in;
    in.mu = A.mu; in.rMin = A.rMin; in.rc = S.rc; in.vc = S.vc; in.rt = S.rt; in.vt = S.vt;
    in.wp.axis0 = S.dT; in.wp.omega = S.wtG; in.wp.standoff = A.standoff;
    in.tLead = 240.0;
    in.wTime = A.fast ? 0.004 : 0.0004;
    in.dvMax = A.C.mainF > 0 ? (std::min)(6000.0, 0.85 * A.C.dvAvail) : 0.0;
    if (in.dvMax < 1.0) { setMsg("NO MAIN-ENGINE DELTA-V AVAILABLE FOR A LONG-RANGE TRANSFER"); return; }
    A.planner.start(in);
    A.ph = PH_PLANNING;
    setMsg("PLANNING...");
}

static void planFinished()
{
    const dk::Plan &p = A.planner.result();
    if (!p.valid) { A.ph = PH_IDLE; setMsg("NO SAFE PLAN WITHIN DELTA-V LIMIT (ORBITS TOO DIFFERENT?)"); return; }
    A.plan = p;
    A.tb1 = burnTime(p.dv1m); A.tb2 = burnTime(p.dv2m);
    A.tIgn1 = A.tRef + p.t0 - 0.5 * A.tb1;
    A.tArr = A.tRef + p.t0 + p.tof;
    A.ph = PH_READY;
    double need = p.dv1m + p.dv2m;
    if (need > 0.9 * A.C.dvAvail) setMsg("PLAN NEEDS %.0f M/S - ONLY %.0f AVAILABLE", need, A.C.dvAvail);
    else setMsg("PLAN READY - ENG TWICE TO START");
}

// ============================================================================ control primitives
static double warpAllowed();

// point the main-engine axis along D (unit, global); returns pointing error [rad]
static double pointMain(VESSEL *v, const V3 &D)
{
    const Snap &S = A.S;
    V3 u = dk::attitudeControl(S.Rc, S.wc, A.C.mainAxis, D, false, V3(0, 1, 0), V3(0, 1, 0), A.gcal, 0.18, A.errP, A.errR);
    applyRCS(v, u, V3());
    return A.errP;
}

// execute a main-engine burn towards velocity vReq (global, planet-relative). returns true when finished.
static bool burnTo(VESSEL *v, const V3 &vReq, double tol)
{
    const Snap &S = A.S;
    V3 dvRem = vReq - S.vc;
    double dvm = dk::len(dvRem);
    if (dvm < tol) { zeroAll(v); A.holdSet = false; return true; }
    double aMain = A.C.mass > 0 ? A.C.mainF / v->GetMass() : 0;
    if (dvm < 1.5 || aMain <= 0) {                        // small: trim with linear RCS, hold attitude
        mainLevel(v, 0);
        if (!A.holdSet) { A.holdD = dk::mulM(S.Rc, A.C.mainAxis); A.holdSet = true; }
        V3 uRot = dk::attitudeControl(S.Rc, S.wc, A.C.mainAxis, A.holdD, false, V3(0, 1, 0), V3(0, 1, 0), A.gcal, 0.1, A.errP, A.errR);
        V3 aLoc = dk::tmulM(S.Rc, dvRem * 0.5);
        double n = dk::len(aLoc), amax = (std::max)(0.05, 0.7 * A.C.aLin);
        if (n > amax) aLoc = aLoc * (amax / n);
        applyRCS(v, uRot, aLoc);
        return false;
    }
    A.holdSet = false;
    V3 D = dk::unit(dvRem);
    double e = pointMain(v, D);
    if (e < 4.0 / DEG2) mainLevel(v, dk::clampd(dvm / (aMain * 1.0), 0.03, 1.0));
    else mainLevel(v, 0);
    return false;
}

// ============================================================================ near-field (final approach)
static void nearField(VESSEL *v)
{
    Snap &S = A.S;
    V3 dT = S.dT;
    double sz = oapiGetSize(A.hTgt);
    double Rk = (std::max)(45.0, 1.5 * sz + 25.0);
    double aAv = (std::max)(0.02, 0.7 * A.C.aLin);
    bool nearDock = S.s < 60.0 && S.s > -5.0;
    // required chaser attitude: port direction anti-parallel to the target's, roll aligned
    V3 Dc = dT * -1.0;
    double eP = 0, eR = 0;
    V3 uRot = dk::attitudeControl(S.Rc, S.wc, S.dc, Dc, S.range < 250.0, S.mc, S.mT, A.gcal, 0.12, eP, eR);
    A.errP = eP; A.errR = eR;
    double align = 1.0;
    if (S.s < 250.0) align = (eP < 3.0 / DEG2 && (S.range > 40.0 || eR < 6.0 / DEG2)) ? 1.0 : 0.0;
    else align = eP < 20.0 / DEG2 ? 1.0 : 0.0;

    dk::FunnelOut f = dk::funnelVelocity(S.rho, dT, align);
    V3 vDes;
    if (S.s < -1.0 && S.range < 60.0) {                   // passed the port without docking: back off
        snprintf(A.sub, 39, "MISSED - BACKING OFF");
        V3 W = dT * 60.0;
        vDes = dk::goToVelocity(W - S.rho, 0.5, aAv);
    } else if (f.inFunnel) {
        snprintf(A.sub, 39, S.s > 12.0 ? "APPROACH" : "FINAL");
        vDes = f.vDes;
    } else {
        snprintf(A.sub, 39, "REPOSITION");
        double D = (std::min)(2000.0, (std::max)(150.0, 0.5 * S.range));
        V3 W = dT * D;
        V3 A0 = S.rho, B0 = W;
        V3 AB = B0 - A0;
        double den = dk::dot(AB, AB), ts = den > 1e-6 ? dk::clampd(-dk::dot(A0, AB) / den, 0, 1) : 0;
        V3 cp = A0 + AB * ts;
        V3 tgtPt = W;
        if (dk::len(cp) < Rk) {                           // straight path clips the target: go round it
            V3 push = dk::len(cp) > 1.0 ? dk::unit(cp) : dk::unit(dk::cross(dT, V3(0.3, 0.5, 0.8)));
            tgtPt = push * (Rk * 1.8 + 30.0);
            snprintf(A.sub, 39, "DETOUR");
        }
        double vmax = S.range > 3000.0 ? 8.0 : (S.range > 600.0 ? 4.0 : 1.5);
        vDes = dk::goToVelocity(tgtPt - S.rho, vmax, aAv * 0.8);
    }
    V3 gC = S.rc * (-A.mu / pow(dk::len(S.rc), 3));
    V3 Pt = S.rt + dk::mulM(S.Rt, S.pt);
    V3 gT = Pt * (-A.mu / pow(dk::len(Pt), 3));
    V3 a = dk::accelToTrack(vDes, S.rhoDot, S.range > 100.0 ? 4.0 : 3.0, aAv) - (gC - gT);
    double an = dk::len(a), amx = A.C.aLin;
    if (an > amx) a = a * (amx / an);
    applyRCS(v, uRot, dk::tmulM(S.Rc, a));
    (void)nearDock;
}

// ============================================================================ calibration
static bool calStep(VESSEL *v)
{
    const Snap &S = A.S;
    double w[3] = { S.wc.x, S.wc.y, S.wc.z };
    if (A.calStage == 0 && A.calT <= 0) { A.calW0 = w[A.calAxis]; A.calT = 1e-9; A.calDur = 0; }
    A.calDur += S.dt;
    if (A.calStage == 0 && A.calDur >= 0.8) {
        double g = (w[A.calAxis] - A.calW0) / (0.5 * A.calDur);
        if (fabs(g) < 1e-4) { A.gcal[A.calAxis] = 0.15; logLine("CAL axis %d FAILED (g=%g)", A.calAxis, g); }
        else A.gcal[A.calAxis] = g;
        logLine("CAL axis %d gain %.4f rad/s^2 per unit", A.calAxis, A.gcal[A.calAxis]);
        A.calStage = 1; A.calDur = 0;
    } else if (A.calStage == 1 && A.calDur >= 0.8) {
        A.calStage = 0; A.calT = 0; A.calAxis++;
        applyRCS(v, V3(), V3());
        if (A.calAxis >= 3) { A.cal = true; return true; }
        return false;
    }
    V3 u; double lvl = (A.calStage == 0 ? 0.5 : -0.5);
    if (A.calAxis == 0) u.x = lvl; else if (A.calAxis == 1) u.y = lvl; else u.z = lvl;
    applyRCS(v, u, V3());
    return false;
}

// ============================================================================ phase handling
static void goPhase(int p, const char *m = NULL)
{
    A.ph = p;
    if (m) setMsg("%s", m);
    A.holdSet = false; A.stateAge = 0;
    logLine("PHASE -> %s %s", phaseName(p), m ? m : "");
}

static void abortAll(const char *why)
{
    VESSEL *v = chaser();
    if (v) { zeroAll(v); v->DeactivateNavmode(NAVMODE_KILLROT); }
    goPhase(PH_ABORT, why);
}

static bool mccCompute(V3 &dv, double tAbs)
{
    const Snap &S = A.S;
    double tau = tAbs - S.t;
    if (tau < 300.0) return false;
    V3 wp, wv; wpState(tAbs, wp, wv);
    V3 hc = dk::cross(S.rc, S.vc);
    int tm = dk::dot(dk::cross(S.rc, wp), hc) >= 0 ? 1 : -1;
    double best = 1e30; bool have = false;
    for (int N = 0; N <= 5; N++)
        for (int br = 0; br < (N == 0 ? 1 : 2); br++) {
            V3 v1, v2;
            if (!dk::lambert(S.rc, wp, tau, A.mu, tm, N, br, v1, v2)) continue;
            double d = dk::len(v1 - S.vc);
            if (d < best) { best = d; dv = v1 - S.vc; have = true; }
        }
    return have && best < 500.0;
}

static void step(double simt, double dt)
{
    // planner time slice
    if (A.ph == PH_PLANNING) {
        if (A.planner.step(0.008)) planFinished();
        else setMsg("PLANNING... %d%%", (int)(100.0 * A.planner.progress()));
        return;
    }
    if (A.ph == PH_IDLE || A.ph == PH_READY || A.ph == PH_DOCKED || A.ph == PH_ABORT) {
        if (A.hTgt && A.hCh) gather(dt);               // keep the rate estimators warm
        return;
    }
    VESSEL *v = chaser();
    if (!v || !target()) { abortAll("VESSEL LOST"); return; }
    if (!gather(dt)) { abortAll("PORT DATA LOST"); return; }
    const Snap &S = A.S;
    // safety
    if (dk::len(S.rc) < A.rMin - 2000.0 && A.ph != PH_NEAR) { abortAll("ORBIT TOO LOW"); return; }
    if (v->DockingStatus(A.cPort) && A.ph >= PH_NEAR) {
        zeroAll(v); goPhase(PH_DOCKED, "DOCKED - AUTOPILOT RELEASED"); return;
    }
    switch (A.ph) {
    case PH_CAL:
        v->DeactivateNavmode(NAVMODE_KILLROT);
        if (calStep(v)) {
            if (A.closeOnly) goPhase(PH_NEAR, "CLOSE APPROACH");
            else { goPhase(PH_COAST, "COASTING TO BURN 1"); v->ActivateNavmode(NAVMODE_KILLROT); }
        }
        break;
    case PH_COAST:
        if (simt >= A.tIgn1 - 100.0) { v->DeactivateNavmode(NAVMODE_KILLROT); goPhase(PH_ORIENT1, "ORIENTING FOR BURN 1"); }
        break;
    case PH_ORIENT1: {
        pointMain(v, dk::unit(A.plan.dv1));
        if (simt >= A.tIgn1) { A.vReq = A.plan.v1; goPhase(PH_BURN1, "BURN 1"); }
        break; }
    case PH_BURN1:
        if (burnTo(v, A.vReq, 0.15)) {
            goPhase(PH_TRANSIT, "COASTING - MID-COURSE CORRECTIONS ARMED");
            v->ActivateNavmode(NAVMODE_KILLROT);
            A.nextMcc = simt + (std::max)(300.0, 0.3 * (A.tArr - simt));
        }
        break;
    case PH_TRANSIT: {
        double tbOr = A.tArr - 0.5 * A.tb2 - 120.0;
        if (simt >= tbOr) {
            v->DeactivateNavmode(NAVMODE_KILLROT);
            goPhase(PH_BRAKE_ORIENT, "ORIENTING FOR BRAKING BURN");
        } else if (simt >= A.nextMcc && A.tArr - simt > 1500.0) {
            V3 dv;
            if (mccCompute(dv, A.tArr)) {
                logLine("MCC dv=%.3f m/s", dk::len(dv));
                if (dk::len(dv) > 0.15) {
                    A.vReq = S.vc + dv;
                    if (dk::len(dv) < 1.5) { v->DeactivateNavmode(NAVMODE_KILLROT); goPhase(PH_MCC_BURN, "CORRECTION BURN (RCS)"); }
                    else { v->DeactivateNavmode(NAVMODE_KILLROT); goPhase(PH_MCC_ORIENT, "ORIENTING FOR CORRECTION"); }
                }
            }
            A.nextMcc = simt + (std::max)(300.0, 0.4 * (A.tArr - simt));
        }
        break; }
    case PH_MCC_ORIENT: {
        V3 dvRem = A.vReq - S.vc;
        pointMain(v, dk::unit(dvRem));
        if (A.errP < 3.0 / DEG2 && A.stateAge > 5.0) goPhase(PH_MCC_BURN, "CORRECTION BURN");
        A.stateAge += dt;
        break; }
    case PH_MCC_BURN:
        if (burnTo(v, A.vReq, 0.1)) { goPhase(PH_TRANSIT, "COASTING"); v->ActivateNavmode(NAVMODE_KILLROT); A.stateAge = 0; }
        break;
    case PH_BRAKE_ORIENT: {
        V3 wp, wv; wpState(A.tArr, wp, wv);
        V3 vArr, rArr; dk::kepler(S.rc, S.vc, A.mu, (std::max)(0.0, A.tArr - simt), rArr, vArr);
        V3 dvPred = wv - vArr;
        pointMain(v, dk::unit(dvPred));
        if (simt >= A.tArr - 0.5 * A.tb2) goPhase(PH_BRAKE, "BRAKING BURN");
        break; }
    case PH_BRAKE: {
        V3 wp, wv; wpState(simt, wp, wv);
        if (burnTo(v, wv, 0.4)) { goPhase(PH_NEAR, "CLOSE APPROACH"); }
        break; }
    case PH_NEAR:
        nearField(v);
        break;
    default: break;
    }

    // time-acceleration governor
    double wa = warpAllowed();
    double cur = oapiGetTimeAcceleration();
    if (cur > wa + 1e-6) oapiSetTimeAcceleration((std::max)(1.0, wa));
    else if (A.autoWarp && cur < wa * 0.5) oapiSetTimeAcceleration((std::min)(wa, 1000.0));

    // flight log
    if (simt - A.logT >= 2.0) {
        A.logT = simt;
        logLine("t=%.0f ph=%s sub=%s range=%.1f s=%.1f lat=%.1f clos=%.3f eP=%.2f eR=%.2f vrel=%.2f fuelRCS=%.1f warp=%.0f",
                simt, phaseName(A.ph), A.sub, S.range, S.s, S.lat, S.closing, A.errP * DEG2, A.errR * DEG2, dk::len(S.rhoDot), A.C.rcsFuel, cur);
    }
}

static double warpAllowed()
{
    double t = oapiGetSimTime();
    switch (A.ph) {
    case PH_COAST: return (std::max)(1.0, (A.tIgn1 - 100.0 - t) / 25.0);
    case PH_TRANSIT: {
        double ev = (std::min)(A.nextMcc, A.tArr - 0.5 * A.tb2 - 120.0);
        return (std::max)(1.0, (ev - t) / 25.0); }
    case PH_NEAR: {
        double s = fabs(A.S.s);
        if (A.S.range > 3000.0) return 10.0;
        if (s > 400.0) return 4.0;
        if (s > 80.0) return 2.0;
        return 1.0; }
    case PH_PLANNING: case PH_READY: case PH_IDLE: case PH_DOCKED: case PH_ABORT: return 1e9;
    default: return 1.0;
    }
}

static void engage()
{
    VESSEL *v = chaser();
    if (!v || !A.hTgt) { setMsg("SELECT A TARGET AND PLAN FIRST"); return; }
    if (A.ph != PH_READY) { setMsg("PLAN FIRST"); return; }
    double now = oapiGetSimTime();
    if (now - A.engArmT > 8.0) { A.engArmT = now; setMsg("PRESS ENG AGAIN WITHIN 8 S TO CONFIRM"); return; }
    A.engArmT = -1e9;
    computeCaps(v);
    if (!A.closeOnly) {
        if (A.tIgn1 - now < 200.0) { setMsg("PLAN TOO OLD - PRESS PLN AGAIN"); return; }
        if (A.plan.dv1m + A.plan.dv2m > 0.9 * A.C.dvAvail) { setMsg("NOT ENOUGH DELTA-V FOR THIS PLAN"); return; }
    }
    A.haveRc = A.haveRt = false;
    A.cal = false; A.calAxis = 0; A.calStage = 0; A.calT = 0;
    logLine("==== ENGAGE target=%s closeOnly=%d dv1=%.1f dv2=%.1f tof=%.0f nrev=%d", A.tgtName, A.closeOnly ? 1 : 0, A.plan.dv1m, A.plan.dv2m, A.plan.tof, A.plan.nrev);
    goPhase(PH_CAL, "CHECKING RCS RESPONSE");
}

static void doAbort()
{
    if (A.ph == PH_IDLE || A.ph == PH_DOCKED) { setMsg("NOTHING TO ABORT"); return; }
    if (A.ph == PH_PLANNING || A.ph == PH_READY) { A.ph = PH_IDLE; setMsg("PLAN CANCELLED"); return; }
    abortAll("ABORTED BY PILOT - YOU HAVE CONTROL");
}

// ============================================================================ MFD
static void fmtDist(char *b, double m)
{
    if (m >= 1e6) snprintf(b, 24, "%.0f km", m / 1000.0);
    else if (m >= 10000.0) snprintf(b, 24, "%.1f km", m / 1000.0);
    else if (m >= 1000.0) snprintf(b, 24, "%.2f km", m / 1000.0);
    else if (m >= 10.0) snprintf(b, 24, "%.0f m", m);
    else snprintf(b, 24, "%.2f m", m);
}
static void fmtTime(char *b, double s)
{
    if (s < 0) s = 0;
    int t = (int)s;
    if (t >= 3600) snprintf(b, 24, "%dh%02dm", t / 3600, (t / 60) % 60);
    else snprintf(b, 24, "%dm%02ds", t / 60, t % 60);
}

class DOCK_MFD : public MFD2 {
public:
    DOCK_MFD(DWORD w, DWORD h, VESSEL *v) : MFD2(w, h, v), page(-1)
    {
        fMed = oapiCreateFont((int)(h / 19), true, "Arial", FONT_NORMAL);
        fLab = oapiCreateFont((int)(h / 22), true, "Arial", FONT_NORMAL);
        fBig = oapiCreateFont((int)(h / 13), true, "Arial", FONT_NORMAL);
        bgB = oapiCreateBrush(C_BG); bandB = oapiCreateBrush(C_BAND); selB = oapiCreateBrush(RGB(0, 45, 0));
        penLine = oapiCreatePen(1, 1, C_LINE);
    }
    ~DOCK_MFD()
    {
        oapiReleaseFont(fMed); oapiReleaseFont(fLab); oapiReleaseFont(fBig);
        oapiReleaseBrush(bgB); oapiReleaseBrush(bandB); oapiReleaseBrush(selB); oapiReleasePen(penLine);
    }
    void Txt(oapi::Sketchpad *s, oapi::Font *f, DWORD col, int x, int yy, const char *t, bool right = false)
    {
        s->SetFont(f); s->SetTextColor(col);
        s->SetTextAlign(right ? oapi::Sketchpad::RIGHT : oapi::Sketchpad::LEFT);
        s->Text(x, yy, t, (int)strlen(t));
        s->SetTextAlign(oapi::Sketchpad::LEFT);
    }
    void Row(oapi::Sketchpad *s, const char *k, const char *v, DWORD vc = C_VAL)
    {
        Txt(s, fLab, C_LAB, pad, y, k);
        Txt(s, fMed, vc, W - pad, y - 1, v, true);
        y += rowH;
        s->SetPen(penLine); s->MoveTo(pad, y - 3); s->LineTo(W - pad, y - 3);
    }
    int ActivePage() const
    {
        if (page >= 0) return page;
        return (A.ph >= PH_CAL && A.ph < PH_DOCKED) || A.ph == PH_DOCKED ? 1 : 0;
    }
    bool Update(oapi::Sketchpad *skp)
    {
        double now = oapiGetSimTime();
        if (now - A.candT > 1.0 || now < A.candT) { A.candT = now; if (A.ph < PH_PLANNING || A.ph == PH_READY) refreshCands(); }
        pad = (int)(W * 0.035); rowH = (int)(H / 14.5);
        skp->SetBackgroundMode(oapi::Sketchpad::BK_TRANSPARENT);
        skp->SetBrush(bgB); skp->SetPen(NULL); skp->Rectangle(0, 0, W, H);
        int hb = (int)(H * 0.085);
        skp->SetBrush(bandB); skp->Rectangle(0, 0, W, hb);
        Txt(skp, fMed, C_VAL, pad, hb / 2 - (int)(H / 19) / 2 - 1, "DOCK");
        Txt(skp, fMed, (A.ph == PH_ABORT) ? C_RED : C_AMBER, W - pad, hb / 2 - (int)(H / 19) / 2 - 1, phaseName(A.ph), true);
        y = hb + 6;
        if (ActivePage() == 0) DrawSetup(skp); else DrawRun(skp);
        // message line
        int my = H - (int)(H * 0.075);
        skp->SetPen(penLine); skp->MoveTo(pad, my - 3); skp->LineTo(W - pad, my - 3);
        DWORD mc = (strstr(A.msg, "ABORT") || strstr(A.msg, "NO ") || strstr(A.msg, "NOT ") || strstr(A.msg, "LOST") || strstr(A.msg, "LOW")) ? C_RED : C_VAL;
        Txt(skp, fLab, mc, pad, my, A.msg);
        return true;
    }
    void DrawSetup(oapi::Sketchpad *skp)
    {
        char b[96], d[24];
        Txt(skp, fLab, C_LAB, pad, y, A.hTgt ? "TARGET" : "TARGETS IN THIS ORBIT");
        if (A.hTgt) Txt(skp, fMed, C_CYAN, W - pad, y - 1, A.tgtName, true);
        y += rowH;
        int shown = 0;
        for (size_t i = 0; i < A.cands.size() && shown < 5; i++, shown++) {
            const Cand &c = A.cands[i];
            bool sel = ((int)i == A.cur);
            if (sel) { skp->SetBrush(selB); skp->SetPen(NULL); skp->Rectangle(0, y - 2, W, y + rowH - 2); }
            fmtDist(d, c.dist);
            snprintf(b, 95, "%s%.16s", sel ? "> " : "  ", c.name);
            Txt(skp, fMed, sel ? C_AMBER : C_VAL, pad, y, b);
            snprintf(b, 95, "%s  %d/%d", d, c.nFree, c.nTot);
            Txt(skp, fLab, C_LAB, W - pad, y + 2, b, true);
            y += rowH;
        }
        if (A.cands.empty()) { Txt(skp, fLab, C_AMBER, pad, y, "NONE (needs a free docking port)"); y += rowH; }
        y = (int)(H * 0.092) + 6 + rowH * 7;
        VESSEL *v = A.hCh ? chaser() : oapiGetFocusInterface();
        VESSEL *t = target();
        if (v && A.hTgt) {
            snprintf(b, 95, "%d  ->  %d", A.cPort + 1, A.tPort + 1);
            Row(skp, "PORTS (YOURS -> TARGET)", b);
        }
        snprintf(b, 95, "%.1f km   %s", A.standoff / 1000.0, A.fast ? "FAST" : "ECONOMY");
        Row(skp, "STANDOFF / PLAN STYLE", b);
        if (A.ph == PH_READY && A.closeOnly) Row(skp, "MODE", "DIRECT APPROACH", C_AMBER);
        else if (A.ph == PH_READY && A.plan.valid) {
            char t1[24], t2[24];
            fmtTime(t1, A.tIgn1 - oapiGetSimTime()); fmtTime(t2, A.tArr - oapiGetSimTime());
            snprintf(b, 95, "%.0f + %.0f = %.0f m/s", A.plan.dv1m, A.plan.dv2m, A.plan.dv1m + A.plan.dv2m);
            Row(skp, "DELTA-V", b, (A.plan.dv1m + A.plan.dv2m > 0.9 * A.C.dvAvail) ? C_RED : C_VAL);
            snprintf(b, 95, "%s   /   %s", t1, t2);
            Row(skp, "BURN IN  /  ARRIVAL IN", b);
        }
        (void)t;
    }
    void DrawRun(oapi::Sketchpad *skp)
    {
        char b[96], d[24], d2[24];
        const Snap &S = A.S;
        Txt(skp, fLab, C_LAB, pad, y, "TARGET"); Txt(skp, fMed, C_CYAN, W - pad, y - 1, A.tgtName, true); y += rowH;
        if (A.ph == PH_DOCKED) {
            Txt(skp, fBig, C_VAL, W / 2 - (int)(W * 0.30), y + rowH, "DOCKED");
            return;
        }
        double now = oapiGetSimTime();
        if (A.ph <= PH_ABORT && S.ok) {
            fmtDist(d, S.range);
            snprintf(b, 95, "%s", d); Row(skp, "RANGE", b);
            snprintf(b, 95, "%.2f m/s", S.closing); Row(skp, "CLOSING SPEED", b);
            fmtDist(d, fabs(S.s)); fmtDist(d2, S.lat);
            snprintf(b, 95, "%s  /  %s", d, d2); Row(skp, "ON-AXIS / OFF-AXIS", b, (S.lat > 0.08 * fabs(S.s) + 1.0 && S.s < 300) ? C_AMBER : C_VAL);
            snprintf(b, 95, "%.1f deg   /   %.1f deg", A.errP * DEG2, A.errR * DEG2); Row(skp, "POINTING / ROLL ERROR", b);
        }
        if (A.ph == PH_COAST || A.ph == PH_ORIENT1) { fmtTime(d, A.tIgn1 - now); Row(skp, "BURN 1 IN", d); }
        if (A.ph >= PH_TRANSIT && A.ph <= PH_BRAKE) { fmtTime(d, A.tArr - now); Row(skp, "ARRIVAL IN", d); }
        if (A.ph == PH_NEAR) Row(skp, "STAGE", A.sub, C_AMBER);
        snprintf(b, 95, "%.0f x", oapiGetTimeAcceleration()); Row(skp, "TIME ACCELERATION", b);
        snprintf(b, 95, "%.0f kg RCS  /  %.0f m/s MAIN", A.C.rcsFuel, A.C.dvAvail); Row(skp, "FUEL", b);
    }
    bool ConsumeButton(int bt, int ev)
    {
        if (!(ev & PANEL_MOUSE_LBDOWN)) return false;
        switch (bt) {
        case 0: if (A.cur > 0) A.cur--; break;
        case 1: if (A.cur + 1 < (int)A.cands.size()) A.cur++; break;
        case 2: selectTarget(); break;
        case 3: { VESSEL *v = chaser(); if (v && A.ph < PH_CAL) { A.cPort = nextFreePort(v, A.cPort); A.ph = (A.ph == PH_READY) ? PH_IDLE : A.ph; setMsg("YOUR PORT %d - PRESS PLN", A.cPort + 1); } break; }
        case 4: { VESSEL *t = target(); if (t && A.ph < PH_CAL) { A.tPort = nextFreePort(t, A.tPort); A.ph = (A.ph == PH_READY) ? PH_IDLE : A.ph; setMsg("TARGET PORT %d - PRESS PLN", A.tPort + 1); } break; }
        case 5: if (A.ph < PH_CAL) { A.standoff = (A.standoff < 1500) ? 2000 : (A.standoff < 3000 ? 5000 : 1000); if (A.ph == PH_READY) A.ph = PH_IDLE; } break;
        case 6: startPlan(); break;
        case 7: engage(); break;
        case 8: doAbort(); break;
        case 9: if (A.ph < PH_CAL) { A.fast = !A.fast; if (A.ph == PH_READY) A.ph = PH_IDLE; setMsg(A.fast ? "FAST PLANS: MORE DELTA-V, LESS WAITING" : "ECONOMY PLANS: LESS DELTA-V, MORE WAITING"); } break;
        case 10: A.autoWarp = !A.autoWarp; setMsg(A.autoWarp ? "AUTO TIME-WARP ON" : "AUTO TIME-WARP OFF (WARP IS STILL LIMITED FOR SAFETY)"); break;
        case 11: page = (ActivePage() == 0) ? 1 : 0; break;
        default: return false;
        }
        InvalidateButtons(); InvalidateDisplay();
        return true;
    }
    bool ConsumeKeyBuffered(DWORD key)
    {
        switch (key) {
        case OAPI_KEY_UP: return ConsumeButton(0, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_DOWN: return ConsumeButton(1, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_S: return ConsumeButton(2, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_P: return ConsumeButton(6, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_E: return ConsumeButton(7, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_A: return ConsumeButton(8, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_W: return ConsumeButton(10, PANEL_MOUSE_LBDOWN);
        case OAPI_KEY_G: return ConsumeButton(11, PANEL_MOUSE_LBDOWN);
        }
        return false;
    }
    char *ButtonLabel(int bt)
    {
        switch (bt) {
        case 0: return (char *)"UP";
        case 1: return (char *)"DN";
        case 2: return (char *)"SEL";
        case 3: return (char *)"PRT";
        case 4: return (char *)"TPT";
        case 5: return (char *)"STO";
        case 6: return (char *)"PLN";
        case 7: return (char *)(oapiGetSimTime() - A.engArmT < 8.0 ? "ENG?" : "ENG");
        case 8: return (char *)"ABT";
        case 9: return (char *)(A.fast ? "FST" : "ECO");
        case 10: return (char *)(A.autoWarp ? "WRP*" : "WRP");
        case 11: return (char *)"PG";
        }
        return 0;
    }
    int ButtonMenu(const MFDBUTTONMENU **menu) const
    {
        static const MFDBUTTONMENU m[12] = {
            { "Cursor up", 0, 0 }, { "Cursor down", 0, 0 }, { "Select highlighted target", 0, 'S' },
            { "Your docking port", "cycles free ports", 0 }, { "Target docking port", "cycles free ports", 0 },
            { "Standoff distance", "1 / 2 / 5 km", 0 },
            { "Plan the rendezvous", 0, 'P' }, { "Engage (press twice)", 0, 'E' }, { "Abort - you get control", 0, 'A' },
            { "Plan style", "FAST = more fuel, ECO = more waiting", 0 },
            { "Auto time-warp", "speeds up the long coasts", 'W' }, { "Page: setup / status", 0, 'G' } };
        if (menu) *menu = m; return 12;
    }
    void WriteStatus(FILEHANDLE scn) const
    {
        oapiWriteScenario_float(scn, "STANDOFF", A.standoff);
        oapiWriteScenario_int(scn, "FAST", A.fast ? 1 : 0);
        oapiWriteScenario_int(scn, "AUTOWARP", A.autoWarp ? 1 : 0);
    }
    void ReadStatus(FILEHANDLE scn)
    {
        char *line;
        while (oapiReadScenario_nextline(scn, line)) {
            if (!_strnicmp(line, "STANDOFF ", 9)) A.standoff = dk::clampd(atof(line + 9), 500.0, 10000.0);
            else if (!_strnicmp(line, "FAST ", 5)) A.fast = atoi(line + 5) != 0;
            else if (!_strnicmp(line, "AUTOWARP ", 9)) A.autoWarp = atoi(line + 9) != 0;
        }
    }
private:
    int page, pad, rowH, y;
    oapi::Font *fMed, *fLab, *fBig;
    oapi::Brush *bgB, *bandB, *selB;
    oapi::Pen *penLine;
};

static int MsgProc(UINT msg, UINT mfd, WPARAM wp, LPARAM lp)
{
    switch (msg) {
    case OAPI_MSG_MFD_OPENED: return (int)(size_t) new DOCK_MFD(LOWORD(wp), HIWORD(wp), (VESSEL *)lp);
    }
    return 0;
}

DLLCLBK void InitModule(HINSTANCE hDLL)
{
    static char name[] = "Dock";
    MFDMODESPECEX spec; spec.name = name; spec.key = OAPI_KEY_D; spec.context = NULL; spec.msgproc = MsgProc;
    g_mode = oapiRegisterMFDMode(spec);
}
DLLCLBK void ExitModule(HINSTANCE hDLL)
{
    VESSEL *v = chaser();
    if (v && A.ph >= PH_CAL && A.ph < PH_DOCKED) zeroAll(v);
    if (g_mode >= 0) oapiUnregisterMFDMode(g_mode);
}
DLLCLBK void opcPostStep(double simt, double simdt, double mjd) { step(simt, simdt); }
