#include "DockMath.h"
#include <algorithm>
#include <chrono>

namespace dk {

V3 rotateAbout(const V3 &v, const V3 &k, double ang)
{
    double c = std::cos(ang), s = std::sin(ang);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1.0 - c));
}

double wrapPi(double a)
{
    while (a > PI) a -= 2 * PI;
    while (a <= -PI) a += 2 * PI;
    return a;
}

static void stumpff(double z, double &C, double &S)
{
    if (z > 1e-6) {
        double sz = std::sqrt(z);
        C = (1.0 - std::cos(sz)) / z;
        S = (sz - std::sin(sz)) / (sz * sz * sz);
    } else if (z < -1e-6) {
        double sz = std::sqrt(-z);
        C = (std::cosh(sz) - 1.0) / (-z);
        S = (std::sinh(sz) - sz) / (sz * sz * sz);
    } else {
        C = 0.5 - z / 24.0 + z * z / 720.0;
        S = 1.0 / 6.0 - z / 120.0 + z * z / 5040.0;
    }
}

bool kepler(const V3 &r0, const V3 &v0, double mu, double dt, V3 &r, V3 &v)
{
    if (dt == 0.0) { r = r0; v = v0; return true; }
    double r0n = len(r0), v0n2 = dot(v0, v0);
    double sm = std::sqrt(mu);
    double alpha = 2.0 / r0n - v0n2 / mu;
    double rv = dot(r0, v0) / sm;
    double chi;
    if (alpha > 1e-12) chi = sm * dt * alpha; else chi = sm * dt / r0n;
    double C = 0, S = 0;
    for (int it = 0; it < 80; it++) {
        double z = alpha * chi * chi;
        stumpff(z, C, S);
        double F = rv * chi * chi * C + (1.0 - alpha * r0n) * chi * chi * chi * S + r0n * chi - sm * dt;
        double dF = chi * chi * C + rv * chi * (1.0 - z * S) + r0n * (1.0 - z * C);
        if (std::fabs(dF) < 1e-14) break;
        double dchi = F / dF;
        chi -= dchi;
        if (std::fabs(dchi) < 1e-9 * std::max(1.0, std::fabs(chi))) break;
        if (it == 79) return false;
    }
    double z = alpha * chi * chi;
    stumpff(z, C, S);
    double f = 1.0 - chi * chi / r0n * C;
    double g = dt - chi * chi * chi / sm * S;
    r = r0 * f + v0 * g;
    double rn = len(r);
    double fd = sm / (rn * r0n) * chi * (z * S - 1.0);
    double gd = 1.0 - chi * chi / rn * C;
    v = r0 * fd + v0 * gd;
    return true;
}

// ----------------------------------------------------------------------------- Lambert
static double tofOf(double psi, double A, double R1, double R2, double sm, double &y)
{
    double C, S;
    stumpff(psi, C, S);
    if (C <= 1e-15) { y = -1; return NAN; }
    y = R1 + R2 + A * (psi * S - 1.0) / std::sqrt(C);
    if (!(y > 0)) return NAN;
    double chi = std::sqrt(y / C);
    return (chi * chi * chi * S + A * std::sqrt(y)) / sm;
}

bool lambert(const V3 &r1, const V3 &r2, double dt, double mu, int tm, int nrev, int branch, V3 &v1, V3 &v2)
{
    double R1 = len(r1), R2 = len(r2);
    if (R1 < 1 || R2 < 1 || dt <= 0) return false;
    double cosd = dot(r1, r2) / (R1 * R2);
    double A = tm * std::sqrt(R1 * R2 * (1.0 + cosd));
    if (std::fabs(A) < 1e-6 * std::sqrt(R1 * R2)) return false;
    double sm = std::sqrt(mu), y = 0, psi = 0;
    if (nrev == 0) {
        double lo = -16.0 * PI * PI, hi = 4.0 * PI * PI - 1e-9;
        for (int it = 0; it < 200; it++) {
            psi = 0.5 * (lo + hi);
            double T = tofOf(psi, A, R1, R2, sm, y);
            if (std::isnan(T) || T < dt) lo = psi; else hi = psi;
            if (hi - lo < 1e-13 * std::max(1.0, std::fabs(psi))) break;
        }
        psi = 0.5 * (lo + hi);
        double T = tofOf(psi, A, R1, R2, sm, y);
        if (std::isnan(T) || std::fabs(T - dt) > 1e-3 * dt) return false;
    } else {
        double zl = std::pow(2.0 * PI * nrev, 2) + 1e-7, zh = std::pow(2.0 * PI * (nrev + 1), 2) - 1e-7;
        double a = zl, b = zh;
        for (int it = 0; it < 120; it++) {        // ternary search for the minimum of T(psi)
            double m1 = a + (b - a) / 3, m2 = b - (b - a) / 3, yy;
            double t1 = tofOf(m1, A, R1, R2, sm, yy), t2 = tofOf(m2, A, R1, R2, sm, yy);
            if (std::isnan(t1)) t1 = 1e30;
            if (std::isnan(t2)) t2 = 1e30;
            if (t1 < t2) b = m2; else a = m1;
        }
        double pm = 0.5 * (a + b);
        double Tm = tofOf(pm, A, R1, R2, sm, y);
        if (std::isnan(Tm) || Tm > dt) return false;
        double lo, hi;
        if (branch == 0) { lo = zl; hi = pm; } else { lo = pm; hi = zh; }
        for (int it = 0; it < 200; it++) {
            psi = 0.5 * (lo + hi);
            double T = tofOf(psi, A, R1, R2, sm, y);
            if (std::isnan(T)) T = 1e30;
            bool tooSmall = T < dt;
            if (branch == 0) { if (tooSmall) hi = psi; else lo = psi; }
            else { if (tooSmall) lo = psi; else hi = psi; }
            if (hi - lo < 1e-13 * std::max(1.0, std::fabs(psi))) break;
        }
        psi = 0.5 * (lo + hi);
        double T = tofOf(psi, A, R1, R2, sm, y);
        if (std::isnan(T) || std::fabs(T - dt) > 1e-3 * dt) return false;
    }
    double f = 1.0 - y / R1, g = A * std::sqrt(y / mu), gd = 1.0 - y / R2;
    if (std::fabs(g) < 1e-12) return false;
    v1 = (r2 - r1 * f) / g;
    v2 = (r2 * gd - r1) / g;
    return std::isfinite(v1.x + v1.y + v1.z + v2.x + v2.y + v2.z);
}

// ----------------------------------------------------------------------------- waypoint
V3 Waypoint::axisAt(double t) const
{
    double w = len(omega);
    if (w < 1e-12) return axis0;
    return rotateAbout(axis0, omega / w, w * t);
}
V3 Waypoint::pos(const V3 &rt, double t) const { return rt + axisAt(t) * standoff; }
V3 Waypoint::vel(const V3 &rt, const V3 &vt, double t) const { (void)rt; return vt + cross(omega, axisAt(t)) * standoff; }

// ----------------------------------------------------------------------------- planner
// lowest radius reached on a transfer arc (r1 -> r2 sweeping angle theta + 2*pi*nrev in the direction of motion)
static double arcMinRadius(const V3 &r1, const V3 &r2, const V3 &v1, double mu, int tm, int nrev)
{
    double R1 = len(r1), R2 = len(r2);
    V3 h = cross(r1, v1);
    double hh = dot(h, h), p = hh / mu;
    double E = dot(v1, v1) / 2 - mu / R1;
    double e2 = 1.0 + 2.0 * E * hh / (mu * mu);
    double e = e2 > 0 ? std::sqrt(e2) : 0;
    double rp = p / (1.0 + e);
    double base = std::min(R1, R2);
    double cd = dot(r1, r2) / (R1 * R2);
    cd = cd > 1 ? 1 : (cd < -1 ? -1 : cd);
    double th = std::acos(cd);
    if (tm < 0) th = 2 * PI - th;
    if (nrev >= 1 && E < 0) return std::min(base, rp);
    double nu1 = std::atan2(std::sqrt(p / mu) * dot(r1, v1), p - R1);
    if (nu1 < 0) nu1 += 2 * PI;
    if (nu1 + th >= 2 * PI) return std::min(base, rp);
    return base;
}

static double periodOf(const V3 &r, const V3 &v, double mu)
{
    double a2 = 2.0 / len(r) - dot(v, v) / mu;
    if (a2 <= 1e-12) return 0;
    double a = 1.0 / a2;
    return 2 * PI * std::sqrt(a * a * a / mu);
}

void Planner::start(const PlanIn &i)
{
    in = i; done = false; prog = 0; best = Plan(); top.clear();
    double Pc = periodOf(in.rc, in.vc, in.mu), Pt = periodOf(in.rt, in.vt, in.mu);
    if (Pc <= 0 || Pt <= 0) { done = true; return; }
    double P = std::max(Pc, Pt);
    double Pmn = std::min(Pc, Pt);
    ds = in.step > 0 ? in.step : std::min(300.0, std::max(20.0, Pmn / 36.0));
    double horizon = in.horizon > 0 ? in.horizon : 3.2 * Pc;
    double tofMax = in.tofMax > 0 ? in.tofMax : 4.5 * P;
    hc = cross(in.rc, in.vc);
    kT0 = (int)std::ceil(in.tLead / ds);
    n0 = (int)(horizon / ds) + 1;
    int kTmin = std::max(2, (int)std::ceil(std::max(0.2 * std::min(Pc, Pt), 400.0) / ds));
    nT = (int)(tofMax / ds) - kTmin + 1;
    tauFirst = kTmin * ds;
    int K = kT0 + n0 + kTmin + nT + 2;
    cr.assign(K, V3()); cv.assign(K, V3()); wr.assign(K, V3()); wv.assign(K, V3());
    for (int k = 0; k < K; k++) {
        double t = k * ds;
        V3 r, v, rt, vt;
        if (!kepler(in.rc, in.vc, in.mu, t, r, v)) { done = true; return; }
        if (!kepler(in.rt, in.vt, in.mu, t, rt, vt)) { done = true; return; }
        cr[k] = r; cv[k] = v;
        wr[k] = in.wp.pos(rt, t); wv[k] = in.wp.vel(rt, vt, t);
    }
    i0 = 0; iT = 0;
}

int Planner::tmSign(const V3 &r1, const V3 &r2) const { return dot(cross(r1, r2), hc) >= 0 ? 1 : -1; }

void Planner::consider(double cost, double t0, double tof, int nrev, int branch)
{
    Cand c = { cost, t0, tof, nrev, branch };
    top.push_back(c);
    std::sort(top.begin(), top.end(), [](const Cand &a, const Cand &b) { return a.cost < b.cost; });
    if (top.size() > 24) top.pop_back();
}

bool Planner::evalCand(double t0, double tof, int nrev, int branch, Plan &p) const
{
    V3 r1, v1c, rt, vt, r2, v2, va, vb;
    if (!kepler(in.rc, in.vc, in.mu, t0, r1, v1c)) return false;
    if (!kepler(in.rt, in.vt, in.mu, t0 + tof, rt, vt)) return false;
    r2 = in.wp.pos(rt, t0 + tof);
    V3 vw = in.wp.vel(rt, vt, t0 + tof);
    if (!lambert(r1, r2, tof, in.mu, tmSign(r1, r2), nrev, branch, va, vb)) return false;
    if (arcMinRadius(r1, r2, va, in.mu, tmSign(r1, r2), nrev) < in.rMin) return false;
    p.t0 = t0; p.tof = tof; p.nrev = nrev; p.branch = branch;
    p.v1 = va; p.v2 = vb; p.dv1 = va - v1c; p.dv2 = vw - vb;
    p.dv1m = len(p.dv1); p.dv2m = len(p.dv2);
    p.cost = p.dv1m + p.dv2m + in.wTime * (t0 + tof);
    return true;
}

bool Planner::validate(Plan &p) const
{
    V3 r1, v1c;
    if (!kepler(in.rc, in.vc, in.mu, p.t0, r1, v1c)) return false;
    double rmin = 1e30;
    V3 r, v;
    const int N = 90;
    for (int i = 0; i <= N; i++) {
        if (!kepler(r1, p.v1, in.mu, p.tof * i / N, r, v)) return false;
        rmin = std::min(rmin, len(r));
    }
    // arrival check
    V3 rt, vt;
    kepler(in.rt, in.vt, in.mu, p.t0 + p.tof, rt, vt);
    V3 w = in.wp.pos(rt, p.t0 + p.tof);
    p.missAtArrival = len(r - w);
    p.rMinTransfer = rmin;
    if (rmin < in.rMin) return false;
    if (p.missAtArrival > 200.0) return false;
    if (p.dv1m + p.dv2m > in.dvMax) return false;
    return true;
}

bool Planner::step(double budgetSec)
{
    if (done) return true;
    using clk = std::chrono::steady_clock;
    auto t_start = clk::now();
    int cnt = 0;
    while (i0 < n0) {
        int k0 = kT0 + i0;
        double t0 = k0 * ds;
        const V3 &r1 = cr[k0], &v1c = cv[k0];
        while (iT < nT) {
            int kA = k0 + (int)std::lround(tauFirst / ds) + iT;
            double tof = (kA - k0) * ds;
            const V3 &r2 = wr[kA], &vw = wv[kA];
            int tm = tmSign(r1, r2);
            for (int N = 0; N <= in.maxRev; N++) {
                for (int br = 0; br < (N == 0 ? 1 : 2); br++) {
                    V3 va, vb;
                    if (!lambert(r1, r2, tof, in.mu, tm, N, br, va, vb)) continue;
                    double d1 = len(va - v1c), d2 = len(vw - vb);
                    if (arcMinRadius(r1, r2, va, in.mu, tm, N) < in.rMin) continue;
                    double cost = d1 + d2 + in.wTime * (t0 + tof);
                    if (d1 + d2 > in.dvMax) continue;
                    if (top.size() >= 24 && cost >= top.back().cost) continue;
                    consider(cost, t0, tof, N, br);
                }
            }
            iT++;
            if ((++cnt & 7) == 0) {
                double el = std::chrono::duration<double>(clk::now() - t_start).count();
                if (el > budgetSec) { prog = (i0 + (double)iT / std::max(1, nT)) / std::max(1, n0); return false; }
            }
        }
        iT = 0; i0++;
    }
    // refine + validate the best candidates
    Plan bestP; bool have = false;
    for (size_t ci = 0; ci < top.size(); ci++) {
        Plan p;
        double t0 = top[ci].t0, tof = top[ci].tof;
        int N = top[ci].nrev, br = top[ci].branch;
        if (!evalCand(t0, tof, N, br, p)) continue;
        double stp = ds * 0.5;
        for (int it = 0; it < 120 && stp > 0.25; it++) {
            bool imp = false;
            double dts[4][2] = { { stp, 0 }, { -stp, 0 }, { 0, stp }, { 0, -stp } };
            for (int q = 0; q < 4; q++) {
                double nt0 = t0 + dts[q][0], ntof = tof + dts[q][1];
                if (nt0 < in.tLead || ntof < 300.0) continue;
                Plan c;
                if (evalCand(nt0, ntof, N, br, c) && c.cost < p.cost - 1e-6) { p = c; t0 = nt0; tof = ntof; imp = true; break; }
            }
            if (!imp) stp *= 0.5;
        }
        if (!validate(p)) continue;
        if (!have || p.cost < bestP.cost) { bestP = p; have = true; }
    }
    if (have) { best = bestP; best.valid = true; }
    done = true; prog = 1.0;
    return true;
}

}  // namespace dk
