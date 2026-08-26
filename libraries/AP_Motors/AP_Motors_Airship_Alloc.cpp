/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Motors_Airship_Alloc.h"

#include <math.h>
#include <string.h>
#include <float.h>

namespace FloatAlloc {

static const double PI_D = 3.14159265358979323846;
static const double TWO_PI_D = 2.0 * PI_D;

// A cone wider than this stops being convex, so the per-cycle slew window is
// capped here no matter how fast the servo is or how long the cycle is
// (alloc.sector.MAX_CONE_HALFWIDTH_RAD).
static const double MAX_CONE_HALFWIDTH_RAD = 89.0 * PI_D / 180.0;
// The horizon the tilt slew constraint is written over. Constraining the
// command to one 10 ms cycle's travel is both physically wrong — a rate
// limited servo wants a target to chase — and numerically nasty, because a
// cone half a degree wide is two nearly antiparallel half planes.
static const double SLEW_HORIZON_S = 0.2;

// The two 180 degree cones whose union is the whole legal arc. A cone of
// exactly half a turn is a half plane, hence convex, hence one linear row.
static const double HALF_CONE_LO[2] = { -60.0 * PI_D / 180.0, 60.0 * PI_D / 180.0 };
static const double HALF_CONE_HI[2] = { 120.0 * PI_D / 180.0, 240.0 * PI_D / 180.0 };

static const double POLISH_REG = 1e-11;
static const uint16_t MAX_AS_ROUNDS = 400;
static const double MULT_TOL = 1e-10;

// ---------------------------------------------------------------- utilities

double wrap_to_arc(double phi, double arc_min)
{
    double d = fmod(phi - arc_min, TWO_PI_D);
    if (d < 0.0) {
        d += TWO_PI_D;
    }
    return arc_min + d;
}

double clamp_to_arc(double phi, double arc_min, double arc_max)
{
    // Two traps live here. The branch cut: the arc runs from -60 to +240
    // degrees, so an atan2 result of -60.000001 wraps to +299.999999 and an
    // unclamped caller walks the servo a whole turn the wrong way to reach a
    // stop it was already standing on. And the wedge itself: an angle inside
    // it is not a small error to be clipped at one end, it is a request that
    // has to be resolved to whichever stop it is actually nearer.
    const double w = wrap_to_arc(phi, arc_min);
    if (w <= arc_max) {
        return w;
    }
    const double mid = 0.5 * (arc_max + arc_min + TWO_PI_D);
    return (w < mid) ? arc_max : arc_min;
}

void components(double thrust, double tilt, double &tx, double &tz)
{
    tx = thrust * sin(tilt);
    tz = -thrust * cos(tilt);
}

void recover(double tx, double tz, double &thrust, double &tilt)
{
    thrust = sqrt(tx * tx + tz * tz);
    tilt = atan2(tx, -tz);
}

void reference_and_cone(double phi_meas, double slew_rad,
                        double arc_min, double arc_max,
                        const double *phi_target,
                        double &ref, double &cone_lo, double &cone_hi)
{
    double d = slew_rad;
    if (d < 1e-6) {
        d = 1e-6;
    }
    if (d > MAX_CONE_HALFWIDTH_RAD) {
        d = MAX_CONE_HALFWIDTH_RAD;
    }
    const double phi = clamp_to_arc(phi_meas, arc_min, arc_max);
    double lo, hi;
    if (phi_target == nullptr) {
        ref = phi;
        lo = phi - d;
        hi = phi + d;
    } else {
        const double tgt = clamp_to_arc(*phi_target, arc_min, arc_max);
        const double step = tgt - phi;
        if (fabs(step) <= d) {
            ref = tgt;
            lo = tgt - d;
            hi = tgt + d;
        } else if (step > 0.0) {
            // commit the transit at the servo's full rate; the optimiser is
            // still free to choose the thrust magnitude, including zero,
            // which is the right answer for a rotor that is only being aimed
            ref = phi + d;
            lo = ref;
            hi = ref + d;
        } else {
            ref = phi - d;
            lo = ref - d;
            hi = ref;
        }
    }
    if (ref < arc_min) {
        ref = arc_min;
    }
    if (ref > arc_max) {
        ref = arc_max;
    }
    if (lo < arc_min) {
        lo = arc_min;
    }
    if (hi > arc_max) {
        hi = arc_max;
    }
    // A cone clipped against a mechanical stop can collapse to zero width, and
    // a zero width cone written as two half planes is the whole line through
    // the origin — which quietly readmits the opposite direction, 180 degrees
    // away and deep in the wedge.
    if (hi - lo < d) {
        if (hi >= arc_max - 1e-12) {
            lo = arc_max - d;
            if (lo < arc_min) {
                lo = arc_min;
            }
            hi = arc_max;
        } else if (lo <= arc_min + 1e-12) {
            lo = arc_min;
            hi = arc_min + d;
            if (hi > arc_max) {
                hi = arc_max;
            }
        } else {
            const double mid = 0.5 * (lo + hi);
            lo = mid - 0.5 * d;
            hi = mid + 0.5 * d;
            if (lo < arc_min) {
                lo = arc_min;
            }
            if (hi > arc_max) {
                hi = arc_max;
            }
        }
    }
    cone_lo = lo;
    cone_hi = hi;
}

void rotor_block(const Geometry &g, double b[NW][NU])
{
    memset(b, 0, sizeof(double) * NW * NU);
    for (uint8_t i = 0; i < NROTOR; i++) {
        const double x = g.pos[i][0];
        const double y = g.pos[i][1];
        const double z = g.pos[i][2];
        // Tx columns: surge, yaw, and (only for an off axis mount) pitch
        b[0][i] = 1.0;
        b[4][i] = z;
        b[5][i] = -y;
        // Tz columns: heave, roll, pitch
        b[2][NROTOR + i] = 1.0;
        b[3][NROTOR + i] = y;
        b[4][NROTOR + i] = -x;
    }
}

// ------------------------------------------------------- dense linear solve

// LU with partial pivoting, in place on a row major n x n. Returns false on a
// structurally singular system rather than handing back a plausible answer.
static bool lu_solve(double *a, uint8_t n, double *b)
{
    for (uint8_t k = 0; k < n; k++) {
        uint8_t best = k;
        double bestv = fabs(a[k * n + k]);
        for (uint8_t r = (uint8_t)(k + 1); r < n; r++) {
            const double v = fabs(a[r * n + k]);
            if (v > bestv) {
                bestv = v;
                best = r;
            }
        }
        if (bestv < 1e-30) {
            return false;
        }
        if (best != k) {
            for (uint8_t c = 0; c < n; c++) {
                const double t = a[k * n + c];
                a[k * n + c] = a[best * n + c];
                a[best * n + c] = t;
            }
            const double tb = b[k];
            b[k] = b[best];
            b[best] = tb;
        }
        const double d = a[k * n + k];
        for (uint8_t r = (uint8_t)(k + 1); r < n; r++) {
            const double f = a[r * n + k] / d;
            if (fabs(f) < 1e-30) {
                continue;
            }
            a[r * n + k] = f;
            for (uint8_t c = (uint8_t)(k + 1); c < n; c++) {
                a[r * n + c] -= f * a[k * n + c];
            }
            b[r] -= f * b[k];
        }
    }
    for (int16_t i = (int16_t)n - 1; i >= 0; i--) {
        double s = b[i];
        for (uint8_t c = (uint8_t)(i + 1); c < n; c++) {
            s -= a[i * n + c] * b[c];
        }
        b[i] = s / a[i * n + i];
    }
    return true;
}

// Equality constrained minimiser of ws.p/ws.q on the m rows in ws.aact/ws.bact,
// with its multipliers in ws.nu, and one step of iterative refinement: the
// regularisation that makes the indefinite system solvable is also what
// leaves a residual, so it is worth a second solve to remove it.
static bool kkt_solve(Workspace &ws, uint8_t m, double x[NU])
{
    const uint8_t n = NU;
    const uint8_t dim = (uint8_t)(n + m);
    double *k = ws.kkt;
    memset(k, 0, sizeof(double) * (size_t)dim * dim);
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            k[i * dim + j] = ws.p[i][j];
        }
        k[i * dim + i] += POLISH_REG;
        ws.rhs[i] = -ws.q[i];
    }
    for (uint8_t r = 0; r < m; r++) {
        for (uint8_t c = 0; c < n; c++) {
            k[c * dim + (n + r)] = ws.aact[r][c];
            k[(n + r) * dim + c] = ws.aact[r][c];
        }
        k[(n + r) * dim + (n + r)] = -POLISH_REG;
        ws.rhs[n + r] = ws.bact[r];
    }
    memcpy(ws.kkt_work, k, sizeof(double) * (size_t)dim * dim);
    memcpy(ws.sol, ws.rhs, sizeof(double) * dim);
    if (!lu_solve(ws.kkt_work, dim, ws.sol)) {
        return false;
    }
    for (uint8_t i = 0; i < dim; i++) {
        double s = ws.rhs[i];
        for (uint8_t j = 0; j < dim; j++) {
            s -= k[i * dim + j] * ws.sol[j];
        }
        ws.resid[i] = s;
    }
    memcpy(ws.kkt_work, k, sizeof(double) * (size_t)dim * dim);
    if (lu_solve(ws.kkt_work, dim, ws.resid)) {
        for (uint8_t i = 0; i < dim; i++) {
            ws.sol[i] += ws.resid[i];
        }
    }
    for (uint8_t i = 0; i < dim; i++) {
        if (!isfinite(ws.sol[i])) {
            return false;
        }
    }
    memcpy(x, ws.sol, sizeof(double) * n);
    for (uint8_t r = 0; r < m; r++) {
        ws.nu[r] = ws.sol[n + r];
    }
    return true;
}

static double objective(const Workspace &ws, const double x[NU])
{
    double s = 0.0;
    for (uint8_t i = 0; i < NU; i++) {
        double px = 0.0;
        for (uint8_t j = 0; j < NU; j++) {
            px += ws.p[i][j] * x[j];
        }
        s += 0.5 * x[i] * px + ws.q[i] * x[i];
    }
    return s;
}

// ---------------------------------------------------------- the QP itself

bool solve_qp(Workspace &ws, uint8_t nrows, const double *x_seed,
              double x[NU], uint16_t &iterations)
{
    // A textbook primal active set solve from a feasible start, and it is
    // primal on purpose. The allocation problem is flat where it matters: the
    // rotor block has a three dimensional null space and only the Tikhonov
    // term, weighted 1e-4, picks a point inside it, so a solver stopped at a
    // residual tolerance can sit a long way from the optimum in exactly the
    // directions the allocation is deciding. This method keeps the iterate
    // feasible at every step, walks toward the working set's minimiser,
    // stops at the first constraint that blocks, and only drops one when its
    // multiplier has the wrong sign. Each round either strictly decreases the
    // objective or shrinks the working set, so it terminates, and it
    // terminates at a KKT point rather than at a tolerance. That is what lets
    // this file and the Python package agree to ten digits instead of three.
    memset(ws.lower, 0, sizeof(ws.lower));
    memset(ws.upper, 0, sizeof(ws.upper));

    if (x_seed != nullptr) {
        memcpy(x, x_seed, sizeof(double) * NU);
    } else {
        memset(x, 0, sizeof(double) * NU);   // the origin is always feasible here
    }
    for (uint8_t r = 0; r < nrows; r++) {
        double s = 0.0;
        for (uint8_t c = 0; c < NU; c++) {
            s += ws.a[r][c] * x[c];
        }
        ws.az[r] = s;
        if (isfinite(ws.lo[r]) && s <= ws.lo[r] + 1e-12) {
            ws.lower[r] = true;
        } else if (isfinite(ws.hi[r]) && s >= ws.hi[r] - 1e-12) {
            ws.upper[r] = true;
        }
    }

    double x_eq[NU];
    double d[NU];

    for (uint16_t round = 0; round < MAX_AS_ROUNDS; round++) {
        iterations = (uint16_t)(round + 1);
        uint8_t m = 0;
        for (uint8_t r = 0; r < nrows && m < MAXACT; r++) {
            if (ws.lower[r] || ws.upper[r]) {
                for (uint8_t c = 0; c < NU; c++) {
                    ws.aact[m][c] = ws.a[r][c];
                }
                ws.bact[m] = ws.lower[r] ? ws.lo[r] : ws.hi[r];
                ws.idx[m] = r;
                m++;
            }
        }
        if (!kkt_solve(ws, m, x_eq)) {
            return false;
        }
        double dmax = 0.0;
        double xmax = 1.0;
        for (uint8_t c = 0; c < NU; c++) {
            d[c] = x_eq[c] - x[c];
            if (fabs(d[c]) > dmax) {
                dmax = fabs(d[c]);
            }
            if (fabs(x[c]) > xmax) {
                xmax = fabs(x[c]);
            }
        }
        if (dmax <= 1e-12 * xmax) {
            if (m == 0) {
                return true;
            }
            int16_t drop = -1;
            double worst = 0.0;
            for (uint8_t r = 0; r < m; r++) {
                const bool wrong = ws.lower[ws.idx[r]] ? (ws.nu[r] > MULT_TOL) : (ws.nu[r] < -MULT_TOL);
                if (wrong && fabs(ws.nu[r]) > worst) {
                    worst = fabs(ws.nu[r]);
                    drop = ws.idx[r];
                }
            }
            if (drop < 0) {
                return true;
            }
            ws.lower[drop] = false;
            ws.upper[drop] = false;
            continue;
        }
        // longest feasible step toward the working set's minimiser
        double step = 1.0;
        int16_t block = -1;
        bool block_lower = false;
        for (uint8_t r = 0; r < nrows; r++) {
            if (ws.lower[r] || ws.upper[r]) {
                continue;
            }
            double ad = 0.0;
            for (uint8_t c = 0; c < NU; c++) {
                ad += ws.a[r][c] * d[c];
            }
            if (ad < -1e-14 && isfinite(ws.lo[r])) {
                double s_r = (ws.lo[r] - ws.az[r]) / ad;
                if (s_r < step) {
                    step = s_r > 0.0 ? s_r : 0.0;
                    block = r;
                    block_lower = true;
                }
            } else if (ad > 1e-14 && isfinite(ws.hi[r])) {
                double s_r = (ws.hi[r] - ws.az[r]) / ad;
                if (s_r < step) {
                    step = s_r > 0.0 ? s_r : 0.0;
                    block = r;
                    block_lower = false;
                }
            }
        }
        for (uint8_t c = 0; c < NU; c++) {
            x[c] += step * d[c];
        }
        for (uint8_t r = 0; r < nrows; r++) {
            double s = 0.0;
            for (uint8_t c = 0; c < NU; c++) {
                s += ws.a[r][c] * x[c];
            }
            ws.az[r] = s;
        }
        if (block >= 0) {
            ws.lower[block] = block_lower;
            ws.upper[block] = !block_lower;
        }
    }
    return false;
}

void weighted_right_inverse(const double b[NW][NU], const double w_des[NW],
                            const double r_u[NU], double u[NU])
{
    // u = R^-1 B' (B R^-1 B')^-1 W_des over the five reachable rows. Note
    // which matrix is inverted: the 5x5 gram, square and positive definite.
    // B itself is 6x8 and is never inverted, here or anywhere else.
    static const uint8_t REACH[5] = { 0, 2, 3, 4, 5 };
    const double ridge = 1e-9;
    double gram[5 * 5];
    double rhs[5];
    for (uint8_t i = 0; i < 5; i++) {
        rhs[i] = w_des[REACH[i]];
        for (uint8_t j = 0; j < 5; j++) {
            double s = 0.0;
            for (uint8_t c = 0; c < NU; c++) {
                s += b[REACH[i]][c] * b[REACH[j]][c] / r_u[c];
            }
            gram[i * 5 + j] = s + ((i == j) ? ridge : 0.0);
        }
    }
    if (!lu_solve(gram, 5, rhs)) {
        memset(u, 0, sizeof(double) * NU);
        return;
    }
    for (uint8_t c = 0; c < NU; c++) {
        double s = 0.0;
        for (uint8_t i = 0; i < 5; i++) {
            s += b[REACH[i]][c] * rhs[i];
        }
        u[c] = s / r_u[c];
    }
}

// --------------------------------------------------------------- Allocator

void Allocator::init(const Geometry &g, const Weights &w, uint8_t supervisor_period)
{
    _geom = g;
    _weights = w;
    _supervisor_period = supervisor_period == 0 ? 1 : supervisor_period;
    rotor_block(_geom, _b);
    reset();
}

void Allocator::reset()
{
    memset(_u_prev, 0, sizeof(_u_prev));
    memset(_tilt_target, 0, sizeof(_tilt_target));
    _have_target = false;
    _cycle = 0;
}

void Allocator::cost(const double w_des[NW])
{
    for (uint8_t i = 0; i < NU; i++) {
        for (uint8_t j = 0; j < NU; j++) {
            double s = 0.0;
            for (uint8_t r = 0; r < NW; r++) {
                s += _b[r][i] * _weights.q_w[r] * _b[r][j];
            }
            _ws.p[i][j] = 2.0 * s;
        }
        _ws.p[i][i] += 2.0 * _weights.gamma * _weights.r_u[i];
        double s = 0.0;
        for (uint8_t r = 0; r < NW; r++) {
            s += _b[r][i] * _weights.q_w[r] * w_des[r];
        }
        _ws.q[i] = -2.0 * s;
    }
}

uint8_t Allocator::constraints(const double cone_lo[NROTOR], const double cone_hi[NROTOR])
{
    memset(_ws.a, 0, sizeof(_ws.a));
    uint8_t r = 0;
    // component box
    for (uint8_t i = 0; i < NU; i++, r++) {
        _ws.a[r][i] = 1.0;
        _ws.lo[r] = -_geom.thrust_max_n;
        _ws.hi[r] = _geom.thrust_max_n;
    }
    const double disc_b = _geom.thrust_max_n * cos(PI_D / (double)DISC_FACETS);
    for (uint8_t i = 0; i < NROTOR; i++) {
        const double clo = cone_lo[i];
        const double chi = cone_hi[i];
        // Rotating e(phi) = (sin phi, -cos phi) by a quarter turn puts the
        // inward normal of the low edge at (cos lo, sin lo) and of the high
        // edge at -(cos hi, sin hi). Both pass through the origin, so a zero
        // thrust command is always feasible: the cone never forces a rotor to
        // keep pushing.
        _ws.a[r][i] = cos(clo);
        _ws.a[r][NROTOR + i] = sin(clo);
        _ws.lo[r] = 0.0;
        _ws.hi[r] = INFINITY;
        r++;
        if (chi - clo < PI_D) {
            _ws.a[r][i] = -cos(chi);
            _ws.a[r][NROTOR + i] = -sin(chi);
            _ws.lo[r] = 0.0;
            _ws.hi[r] = INFINITY;
            r++;
        }
        for (uint8_t k = 0; k < DISC_FACETS; k++) {
            const double ang = TWO_PI_D * ((double)k + 0.5) / (double)DISC_FACETS;
            _ws.a[r][i] = cos(ang);
            _ws.a[r][NROTOR + i] = sin(ang);
            _ws.lo[r] = -INFINITY;
            _ws.hi[r] = disc_b;
            r++;
        }
    }
    return r;
}

void Allocator::seed_point(const double cone_lo[NROTOR], const double cone_hi[NROTOR], double seed[NU])
{
    // The unconstrained minimiser, clipped into each rotor's own feasible
    // set: angle onto the cone, magnitude onto the inscribed circle of the
    // facetted disc. Feasible by construction, and on this problem it stands
    // on the right active set in the overwhelming majority of cycles.
    const double t_inscribed = _geom.thrust_max_n * cos(PI_D / (double)DISC_FACETS);
    double xs[NU];
    for (uint8_t i = 0; i < NU; i++) {
        xs[i] = -_ws.q[i];
        for (uint8_t j = 0; j < NU; j++) {
            _ws.pw[i * NU + j] = _ws.p[i][j];
        }
    }
    if (!lu_solve(_ws.pw, NU, xs)) {
        memset(seed, 0, sizeof(double) * NU);
        return;
    }
    for (uint8_t i = 0; i < NROTOR; i++) {
        double t, phi;
        recover(xs[i], xs[NROTOR + i], t, phi);
        if (t < 1e-12) {
            seed[i] = 0.0;
            seed[NROTOR + i] = 0.0;
            continue;
        }
        // put phi on the branch nearest the cone before clamping
        double ph = wrap_to_arc(phi, cone_lo[i] - PI_D);
        if (ph < cone_lo[i]) {
            ph = cone_lo[i];
        }
        if (ph > cone_hi[i]) {
            ph = cone_hi[i];
        }
        if (t > t_inscribed) {
            t = t_inscribed;
        }
        components(t, ph, seed[i], seed[NROTOR + i]);
    }
}

void Allocator::supervise_targets(const double w_des[NW])
{
    // Sixteen slew free programs, one per assignment of each rotor to one of
    // the two half cones whose union is the arc; the best objective wins.
    // This is the only place in the class that looks at the arc globally, and
    // it is what stops a rotor whose best home is 200 degrees away from
    // walking into the near stop and staying there.
    cost(w_des);
    double best_obj = DBL_MAX;
    double best_u[NU];
    bool have = false;
    for (uint8_t combo = 0; combo < 16; combo++) {
        double clo[NROTOR];
        double chi[NROTOR];
        for (uint8_t i = 0; i < NROTOR; i++) {
            const uint8_t sel = (uint8_t)((combo >> i) & 1u);
            clo[i] = HALF_CONE_LO[sel] > _geom.arc_min_rad ? HALF_CONE_LO[sel] : _geom.arc_min_rad;
            chi[i] = HALF_CONE_HI[sel] < _geom.arc_max_rad ? HALF_CONE_HI[sel] : _geom.arc_max_rad;
        }
        const uint8_t nrows = constraints(clo, chi);
        double seed[NU];
        seed_point(clo, chi, seed);
        double u[NU];
        uint16_t iters = 0;
        solve_qp(_ws, nrows, seed, u, iters);
        const double obj = objective(_ws, u);
        if (!have || obj < best_obj) {
            best_obj = obj;
            memcpy(best_u, u, sizeof(best_u));
            have = true;
        }
    }
    if (!have) {
        return;
    }
    for (uint8_t i = 0; i < NROTOR; i++) {
        double t, phi;
        recover(best_u[i], best_u[NROTOR + i], t, phi);
        _tilt_target[i] = clamp_to_arc(phi, _geom.arc_min_rad, _geom.arc_max_rad);
    }
    _have_target = true;
}

void Allocator::allocate(const double w_des[NW], double dt,
                         const double tilt_meas[NROTOR], const double thrust_meas[NROTOR],
                         int8_t supervise, Result &out)
{
    (void)thrust_meas;   // thrust slew is not constrained: the rotor is a
                         // first order lag, not a rate limit, and the
                         // bandwidth gap is carried in the effort weights
    const bool run_super = (supervise < 0)
                           ? ((_cycle % _supervisor_period) == 0)
                           : (supervise != 0);
    if (run_super) {
        supervise_targets(w_des);
    }
    _cycle++;

    const double slew = _geom.tilt_rate_rad_s * (dt > SLEW_HORIZON_S ? dt : SLEW_HORIZON_S);
    double clo[NROTOR];
    double chi[NROTOR];
    double tilt_ref[NROTOR];
    for (uint8_t i = 0; i < NROTOR; i++) {
        reference_and_cone(tilt_meas[i], slew, _geom.arc_min_rad, _geom.arc_max_rad,
                           _have_target ? &_tilt_target[i] : nullptr,
                           tilt_ref[i], clo[i], chi[i]);
    }

    cost(w_des);
    const uint8_t nrows = constraints(clo, chi);
    double seed[NU];
    seed_point(clo, chi, seed);

    double u[NU];
    uint16_t iters = 0;
    bool ok = solve_qp(_ws, nrows, seed, u, iters);
    for (uint8_t i = 0; i < NU && ok; i++) {
        if (!isfinite(u[i])) {
            ok = false;
        }
    }
    uint8_t fallback = 0;
    if (!ok) {
        double cand[NU];
        weighted_right_inverse(_b, w_des, _weights.r_u, cand);
        for (uint8_t i = 0; i < NU; i++) {
            if (cand[i] < _ws.lo[i]) {
                cand[i] = _ws.lo[i];
            }
            if (cand[i] > _ws.hi[i]) {
                cand[i] = _ws.hi[i];
            }
        }
        bool feasible = true;
        for (uint8_t r = 0; r < nrows && feasible; r++) {
            double s = 0.0;
            for (uint8_t c = 0; c < NU; c++) {
                s += _ws.a[r][c] * cand[c];
            }
            if (s < _ws.lo[r] - 1e-6 || s > _ws.hi[r] + 1e-6) {
                feasible = false;
            }
        }
        if (feasible) {
            memcpy(u, cand, sizeof(u));
            fallback = 1;
        } else {
            memcpy(u, _u_prev, sizeof(u));
            fallback = 2;
        }
    }
    memcpy(_u_prev, u, sizeof(_u_prev));

    memcpy(out.u, u, sizeof(out.u));
    for (uint8_t i = 0; i < NROTOR; i++) {
        double t, phi;
        recover(u[i], u[NROTOR + i], t, phi);
        // A rotor the optimiser leaves unloaded has no defined thrust
        // direction, so its servo takes the supervisor's reference instead:
        // aiming an idle rotor is exactly what the slow actuator should be
        // doing while the fast one carries the wrench.
        if (t <= IDLE_THRUST_N) {
            phi = tilt_ref[i];
        }
        out.thrust_n[i] = t;
        out.tilt_rad[i] = clamp_to_arc(phi, _geom.arc_min_rad, _geom.arc_max_rad);
        out.tilt_target_rad[i] = _have_target ? _tilt_target[i] : tilt_ref[i];
    }
    for (uint8_t r = 0; r < NW; r++) {
        double s = 0.0;
        for (uint8_t c = 0; c < NU; c++) {
            s += _b[r][c] * u[c];
        }
        out.wrench[r] = s;
    }
    out.converged = ok && (fallback == 0);
    out.fallback = fallback;
    out.iterations = iters;
}

}  // namespace FloatAlloc
