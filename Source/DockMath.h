// DOCK MFD - pure orbital mechanics (no Orbiter dependencies, so it can be tested outside the simulator).
// All vectors use plain algebraic cross products; this is valid in Orbiter's left-handed frame as long as the
// same formulas are used throughout (nothing here depends on handedness).
#pragma once
#include <cmath>
#include <vector>

namespace dk {

const double PI = 3.14159265358979323846;

struct V3 {
    double x = 0, y = 0, z = 0;
    V3() {}
    V3(double a, double b, double c) : x(a), y(b), z(c) {}
};
inline V3 operator+(const V3 &a, const V3 &b) { return V3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline V3 operator-(const V3 &a, const V3 &b) { return V3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline V3 operator*(const V3 &a, double s) { return V3(a.x * s, a.y * s, a.z * s); }
inline V3 operator*(double s, const V3 &a) { return a * s; }
inline V3 operator/(const V3 &a, double s) { return V3(a.x / s, a.y / s, a.z / s); }
inline double dot(const V3 &a, const V3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(const V3 &a, const V3 &b) { return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
inline double len(const V3 &a) { return std::sqrt(dot(a, a)); }
inline V3 unit(const V3 &a) { double l = len(a); return l > 1e-300 ? a / l : V3(); }

V3 rotateAbout(const V3 &v, const V3 &axisUnit, double ang);
double wrapPi(double a);

// two-body propagation (universal variables)
bool kepler(const V3 &r0, const V3 &v0, double mu, double dt, V3 &r, V3 &v);

// Lambert solver (universal variables, multi-revolution).
//  tm = +1: transfer angle < pi, tm = -1: transfer angle > pi (caller decides from geometry / orbit direction)
//  nrev = 0..N, branch = 0 (low-z/"left") or 1 ("right"), only used for nrev >= 1
bool lambert(const V3 &r1, const V3 &r2, double dt, double mu, int tm, int nrev, int branch, V3 &v1, V3 &v2);

struct Waypoint {                 // point fixed in the target's port frame (standoff along the port axis)
    V3 axis0;                     // port approach direction at t=0 (unit, global)
    V3 omega;                     // target rotation rate vector [rad/s, global]
    double standoff = 2000.0;
    V3 pos(const V3 &rt, double t) const;
    V3 vel(const V3 &rt, const V3 &vt, double t) const;
    V3 axisAt(double t) const;
};

struct PlanIn {
    double mu = 0;
    double rMin = 0;              // minimum allowed radius along the transfer
    V3 rc, vc;                    // chaser state (relative to planet), t = 0
    V3 rt, vt;                    // target CoG state, t = 0
    Waypoint wp;
    double tLead = 240.0;         // earliest burn [s]
    double horizon = 0;           // departure search window [s]  (0 = auto)
    double step = 0;              // grid step [s]                (0 = auto)
    double tofMax = 0;            // max transfer time [s]        (0 = auto)
    int maxRev = 5;
    double wTime = 0.0004;        // cost weight [m/s per s of total time]
    double dvMax = 4000.0;
};

struct Plan {
    bool valid = false;
    double t0 = 0, tof = 0;       // departure time and transfer time from t=0 [s]
    int nrev = 0, branch = 0;
    V3 v1, v2;                    // Lambert departure / arrival velocities
    V3 dv1;                       // departure impulse (global)
    V3 dv2;                       // arrival impulse (global), brings you to waypoint velocity
    double dv1m = 0, dv2m = 0, cost = 0;
    double rMinTransfer = 0;
    double missAtArrival = 0;     // [m] distance between predicted chaser and waypoint (sanity)
};

// Time-sliced planner: call step(budgetSeconds) until it returns true.
class Planner {
public:
    void start(const PlanIn &in);
    bool step(double budgetSec);          // returns true when finished
    double progress() const { return prog; }
    const Plan &result() const { return best; }
    bool failed() const { return done && !best.valid; }
private:
    struct Cand { double cost; double t0, tof; int nrev, branch; };
    PlanIn in;
    bool done = false;
    double prog = 0;
    int i0 = 0, iT = 0, n0 = 0, nT = 0, kT0 = 0;
    double ds = 60, tauFirst = 0;
    std::vector<V3> cr, cv, wr, wv;       // tables at k*ds
    std::vector<Cand> top;
    int tmSign(const V3 &r1, const V3 &r2) const;
    bool evalCand(double t0, double tof, int nrev, int branch, Plan &p) const;
    bool validate(Plan &p) const;
    void consider(double cost, double t0, double tof, int nrev, int branch);
    Plan best;
    V3 hc;
    friend struct PlannerAccess;
};

}  // namespace dk
