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
#pragma once

/*
  The Float V0 control allocator: desired body wrench in, four rotor thrusts
  and four tilt angles out. This is the C++ half of Atlas Sections 4.7 and
  4.12 and it is a deliberate line-for-line port of the Python package
  `alloc/` in Float-Cargo/controls-test, which stays the oracle: when the two
  disagree on the same inputs the Python package is what says which one is
  wrong, and gates/g5_motors_parity.py is what asks.

  Three things about this file are worth knowing before reading it.

  It has no ArduPilot dependencies on purpose. Only <cmath> and <stdint.h>,
  so the same translation unit compiles standalone against a test driver on a
  workstation and inside the firmware, and the parity gate therefore tests the
  code that flies rather than a transcription of it.

  The wrench is bilinear in the physical command (T_i, phi_i) and linear in
  the thrust components (Tx_i, Tz_i) = (T sin phi, -T cos phi), so the whole
  solve is carried in components and the trigonometry is evaluated once, on a
  known answer, in recover(). phi is measured from straight up, positive
  toward forward: 0 points the thrust up, +90 degrees forward, +180 straight
  down. The mechanical arc is -60 to +240 degrees and the 60 degree wedge
  centred on straight aft is what is missing between the stops.

  The excluded wedge is nonconvex, so the per-cycle problem is a convex
  restriction to one angular sector per rotor and a supervisor — sixteen
  slew-free solves, one per assignment of each rotor to one of the two half
  cones that cover the arc — decides which sector each rotor belongs in. A
  rotor the optimiser unloads takes the supervisor's reference angle rather
  than the undefined direction of a zero-length thrust vector, without which
  the allocation deadlocks.
 */

#include <stdint.h>

namespace FloatAlloc {

static const uint8_t NROTOR = 4;
static const uint8_t NU = 2 * NROTOR;   // [Tx_0..3, Tz_0..3]
static const uint8_t NW = 6;            // [Fx, Fy, Fz, Mx, My, Mz] about the CV
static const uint8_t DISC_FACETS = 16;  // polytopic inner approximation of the thrust disc
// component box + per rotor (two cone half planes + the disc facets)
static const uint8_t MAXROWS = NU + NROTOR * (2 + DISC_FACETS);

// Below this the recovered tilt angle is numerically meaningless, so the
// supervisor's reference angle is commanded instead (alloc.allocate).
static const double IDLE_THRUST_N = 0.1;

struct Geometry {
    // rotor positions in body FRD metres about the CENTRE OF VOLUME, which is
    // where the plant takes moments and is not where v0sim.config records
    // actuator positions (that file is CG referenced).
    double pos[NROTOR][3];
    double thrust_max_n;
    double arc_min_rad;
    double arc_max_rad;
    double tilt_rate_rad_s;
};

struct Weights {
    double q_w[NW];   // which wrench components matter; sway is zero, it is unreachable
    double r_u[NU];   // effort weights, resolving the redundancy
    double gamma;     // Tikhonov scale on the effort term
};

// Scratch storage for the solver. It lives here rather than on the stack
// because a flight controller's main thread has a small stack and the
// ChibiOS build refuses any frame over 1300 bytes: the KKT matrix alone is
// eight kilobytes. One workspace per Allocator, heap resident with the motors
// object, and every routine below that needs more than a few hundred bytes
// takes it by reference.
static const uint8_t MAXACT = 24;   // active rows the KKT solve can hold
static const uint8_t KKT_DIM = NU + MAXACT;

struct Workspace {
    double a[MAXROWS][NU];
    double lo[MAXROWS];
    double hi[MAXROWS];
    double az[MAXROWS];
    double p[NU][NU];
    double q[NU];
    double kkt[KKT_DIM * KKT_DIM];
    double kkt_work[KKT_DIM * KKT_DIM];
    double rhs[KKT_DIM];
    double sol[KKT_DIM];
    double resid[KKT_DIM];
    double aact[MAXACT][NU];
    double bact[MAXACT];
    double nu[MAXACT];
    uint8_t idx[MAXACT];
    bool lower[MAXROWS];
    bool upper[MAXROWS];
    double pw[NU * NU];
};

struct Result {
    double u[NU];
    double thrust_n[NROTOR];
    double tilt_rad[NROTOR];
    double tilt_target_rad[NROTOR];
    double wrench[NW];
    bool converged;
    uint8_t fallback;      // 0 none, 1 clipped right inverse, 2 previous command
    uint16_t iterations;
};

// ---- free functions, all pure, all shared with the Python package ----

// Put an angle in [arc_min, arc_min + 2*pi).
double wrap_to_arc(double phi, double arc_min);

// Put an angle on the legal arc, snapping a wedge angle to the nearer stop.
double clamp_to_arc(double phi, double arc_min, double arc_max);

// (T, phi) -> (Tx, Tz) and back. T is non-negative by construction.
void components(double thrust, double tilt, double &tx, double &tz);
void recover(double tx, double tz, double &thrust, double &tilt);

// Where this rotor should point next cycle and the convex cone it may use.
void reference_and_cone(double phi_meas, double slew_rad,
                        double arc_min, double arc_max,
                        const double *phi_target,  // nullptr for "no supervisor"
                        double &ref, double &cone_lo, double &cone_hi);

// The 6x8 constant rotor block of B_full, row major.
void rotor_block(const Geometry &g, double b[NW][NU]);

class Allocator {
public:
    void init(const Geometry &g, const Weights &w, uint8_t supervisor_period);
    void reset();

    // One control cycle. `supervise` is -1 for "on the supervisor's own
    // cadence", 0 for never and 1 for always; the parity gate replays with 1
    // so that a replayed sample is a pure function of its inputs.
    void allocate(const double w_des[NW], double dt,
                  const double tilt_meas[NROTOR], const double thrust_meas[NROTOR],
                  int8_t supervise, Result &out);

    // True once the supervisor has run at least once since reset().
    bool have_tilt_target() const { return _have_target; }
    uint32_t cycle() const { return _cycle; }

private:
    // Fill ws.p and ws.q for this wrench demand.
    void cost(const double w_des[NW]);
    // Assemble `lo <= A x <= hi` into ws.a, ws.lo, ws.hi for the given cones.
    uint8_t constraints(const double cone_lo[NROTOR], const double cone_hi[NROTOR]);
    // Unconstrained minimiser of the current cost, clipped into the cones.
    void seed_point(const double cone_lo[NROTOR], const double cone_hi[NROTOR], double seed[NU]);
    void supervise_targets(const double w_des[NW]);

    Workspace _ws;

    Geometry _geom;
    Weights _weights;
    double _b[NW][NU];
    uint8_t _supervisor_period;
    double _u_prev[NU];
    double _tilt_target[NROTOR];
    bool _have_target;
    uint32_t _cycle;
};

// The dense convex quadratic program behind all of the above:
//     minimise 0.5 x' P x + q' x   subject to   lo <= A x <= hi
// solved to a KKT point by primal active set refinement, seeded by clipping
// the unconstrained minimiser into each rotor's own feasible set. Exposed
// here because the standalone parity driver exercises it directly.
// The program is read from ws.p, ws.q, ws.a, ws.lo and ws.hi; nrows says how
// many rows of ws.a are in use.
bool solve_qp(Workspace &ws, uint8_t nrows, const double *x_seed,
              double x[NU], uint16_t &iterations);

// Minimum effort exact solution over the five reachable wrench rows, the
// deterministic fallback when the program above does not converge.
void weighted_right_inverse(const double b[NW][NU], const double w_des[NW],
                            const double r_u[NU], double u[NU]);

}  // namespace FloatAlloc
