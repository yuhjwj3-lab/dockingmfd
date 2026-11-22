// DOCK MFD - control laws (pure maths, no Orbiter dependencies, testable offline).
#pragma once
#include "DockMath.h"
#include <algorithm>

namespace dk {

struct Mat3 {
    double m[3][3];
    Mat3() { for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i][j] = (i == j); }
};
inline V3 mulM(const Mat3 &A, const V3 &v)
{
    return V3(A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
              A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
              A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z);
}
inline V3 tmulM(const Mat3 &A, const V3 &v)
{
    return V3(A.m[0][0] * v.x + A.m[1][0] * v.y + A.m[2][0] * v.z,
              A.m[0][1] * v.x + A.m[1][1] * v.y + A.m[2][1] * v.z,
              A.m[0][2] * v.x + A.m[1][2] * v.y + A.m[2][2] * v.z);
}
inline Mat3 mulMM(const Mat3 &A, const Mat3 &B)
{
    Mat3 C;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        double s = 0; for (int k = 0; k < 3; k++) s += A.m[i][k] * B.m[k][j];
        C.m[i][j] = s;
    }
    return C;
}
inline Mat3 transposeM(const Mat3 &A)
{
    Mat3 T; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) T.m[i][j] = A.m[j][i];
    return T;
}
inline double clampd(double x, double a, double b) { return x < a ? a : (x > b ? b : x); }

// Angular velocity [rad/s] in the BODY frame derived from two successive rotation matrices (local->global).
// Consistent with the algebraic cross product used everywhere else, so no handedness assumptions are needed.
inline V3 bodyRateFromMatrices(const Mat3 &Rprev, const Mat3 &Rnow, double dt)
{
    Mat3 Rr = mulMM(transposeM(Rprev), Rnow);
    V3 k(Rr.m[2][1] - Rr.m[1][2], Rr.m[0][2] - Rr.m[2][0], Rr.m[1][0] - Rr.m[0][1]);   // 2 sin(theta) * axis
    double tr = Rr.m[0][0] + Rr.m[1][1] + Rr.m[2][2];
    double c = clampd((tr - 1.0) * 0.5, -1.0, 1.0);
    double th = std::acos(c), s2 = len(k);
    if (dt <= 0) return V3();
    if (s2 < 1e-12) return V3();
    double scale = th / s2;                // k = 2 sin(th) * axis, so axis * th = k * th / (2 sin th) ; s2 = |k| = 2 sin(th)
    return k * (scale / dt);
}

// ---------------------------------------------------------------- attitude control
// Point body vector Lloc (local) along global direction D (unit) and, optionally, align local Mloc with global Md.
// gcal[i] = measured angular acceleration [rad/s^2] per unit command on axis i (signed; found by calibration).
// Returns the 3 unit commands (each -1..1) in the same sign convention as used during calibration.
inline V3 attitudeControl(const Mat3 &R, const V3 &w, const V3 &Lloc, const V3 &D, bool useRoll, const V3 &Mloc,
                          const V3 &Md, const double gcal[3], double wmax, double &errP, double &errR)
{
    V3 g = mulM(R, Lloc);
    V3 ax = cross(g, D);
    double s = len(ax), c = dot(g, D);
    errP = std::atan2(s, c);
    errR = 0;
    V3 eG;
    if (s > 1e-9) eG = ax * (errP / s);
    else if (c < 0) {
        V3 p = std::fabs(g.x) < 0.9 ? V3(1, 0, 0) : V3(0, 1, 0);
        eG = unit(cross(g, p)) * PI;
    }
    if (useRoll && errP < 12.0 * PI / 180.0) {
        V3 Mg = mulM(R, Mloc);
        V3 a = Mg - D * dot(D, Mg), b = Md - D * dot(D, Md);
        if (len(a) > 1e-6 && len(b) > 1e-6) {
            double psi = std::atan2(dot(D, cross(a, b)), dot(a, b));
            errR = std::fabs(psi);
            eG = eG + D * psi;
        }
    }
    V3 e = tmulM(R, eG);
    double ev[3] = { e.x, e.y, e.z }, wv[3] = { w.x, w.y, w.z }, u[3];
    for (int i = 0; i < 3; i++) {
        double alpha = std::fabs(gcal[i]);
        double ab = 0.45 * alpha;
        double ae = std::fabs(ev[i]), e0 = 0.02, wd;
        if (ae < e0) wd = ev[i] * std::sqrt(2.0 * ab / e0);
        else wd = (ev[i] > 0 ? 1.0 : -1.0) * std::min(wmax, std::sqrt(2.0 * ab * ae));
        wd = clampd(wd, -wmax, wmax);
        double gi = gcal[i];
        if (std::fabs(gi) < 1e-6) gi = 1e-6;
        u[i] = clampd(((wd - wv[i]) / 0.5) / gi, -1.0, 1.0);
        if (ae < 0.0008 && std::fabs(wv[i]) < 2.5e-4) u[i] = 0;
    }
    return V3(u[0], u[1], u[2]);
}

// ---------------------------------------------------------------- translation
// Desired RELATIVE velocity (chaser port w.r.t. target port, global frame) for the docking funnel.
// rho = chaser port - target port, dT = target port approach direction (unit, pointing away from the target).
struct FunnelOut {
    V3 vDes;
    double s = 0, lat = 0;
    bool inFunnel = false;
    double vs = 0;
};
inline FunnelOut funnelVelocity(const V3 &rho, const V3 &dT, double alignFactor)
{
    FunnelOut o;
    double s = dot(rho, dT);
    V3 latv = rho - dT * s;
    double L = len(latv);
    o.s = s; o.lat = L;
    double vs;
    if (s > 12.0) vs = clampd(0.012 * s, 0.10, 4.0);
    else vs = 0.12;
    double fun = 0.08 * std::max(s, 0.0) + 0.4;
    if (L > fun) vs *= clampd(1.0 - (L - fun) / (0.25 * std::max(s, 0.0) + 2.0), 0.0, 1.0);
    vs *= alignFactor;
    double vlatMax = std::max(0.05, std::min(1.5, 0.15 * std::max(s, 0.0)));
    V3 vlat = L > 1e-6 ? latv * (-std::min(0.08 * L, vlatMax) / L) : V3();
    o.vDes = dT * (-vs) + vlat;
    o.vs = vs;
    o.inFunnel = (s > 0) && (L < std::tan(35.0 * PI / 180.0) * s + 5.0);
    return o;
}

// velocity command towards a point (relative position 'toPt' = target point - chaser), with braking curve
inline V3 goToVelocity(const V3 &toPt, double vmax, double aAvail)
{
    double d = len(toPt);
    if (d < 1e-6) return V3();
    double v = std::min(vmax, std::sqrt(2.0 * 0.3 * aAvail * d));
    if (d < 5.0) v = std::min(v, 0.2 * d + 0.02);
    return toPt * (v / d);
}

// thrust acceleration command to track vDes
inline V3 accelToTrack(const V3 &vDes, const V3 &vRel, double tau, double aAvail)
{
    V3 a = (vDes - vRel) / tau;
    double n = len(a);
    if (n > aAvail) a = a * (aAvail / n);
    return a;
}

}  // namespace dk
