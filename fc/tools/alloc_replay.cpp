/*
  Standalone driver for the Float allocator, so that the firmware's own
  translation unit — not a transcription of it — can be compared against the
  Python reference package `alloc/` without a build of ArduPilot, a Software
  In The Loop stack or a flight.

  Build:  c++ -O2 -std=c++11 -o alloc_replay fc/tools/alloc_replay.cpp \
                libraries/AP_Motors/AP_Motors_Airship_Alloc.cpp

  Protocol, whitespace separated on standard input:
    line 1   12 rotor position components, thrust_max_n, arc_min, arc_max,
             tilt_rate (radians and radians per second)
    line 2   6 wrench weights, 8 effort weights, gamma, supervisor period
    then one record per cycle:
             6 wrench components, dt, 4 measured tilts, 4 measured thrusts,
             supervise flag (-1 cadence, 0 never, 1 always)
  and one line per record on standard output:
             4 thrusts, 4 tilts, converged, fallback, iterations
 */
#include "../../libraries/AP_Motors/AP_Motors_Airship_Alloc.h"
#include <cstdio>

int main()
{
    FloatAlloc::Geometry g{};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            if (scanf("%lf", &g.pos[i][j]) != 1) { return 2; }
        }
    }
    if (scanf("%lf %lf %lf %lf", &g.thrust_max_n, &g.arc_min_rad,
              &g.arc_max_rad, &g.tilt_rate_rad_s) != 4) { return 2; }

    FloatAlloc::Weights w{};
    for (int i = 0; i < FloatAlloc::NW; i++) {
        if (scanf("%lf", &w.q_w[i]) != 1) { return 2; }
    }
    for (int i = 0; i < FloatAlloc::NU; i++) {
        if (scanf("%lf", &w.r_u[i]) != 1) { return 2; }
    }
    int period = 10;
    if (scanf("%lf %d", &w.gamma, &period) != 2) { return 2; }

    static FloatAlloc::Allocator alloc;   // static: the workspace is ~30 kB
    alloc.init(g, w, (uint8_t)period);

    double wd[FloatAlloc::NW], dt, tilt[4], thrust[4];
    int sup;
    for (;;) {
        bool ok = true;
        for (int i = 0; i < FloatAlloc::NW && ok; i++) { ok = scanf("%lf", &wd[i]) == 1; }
        if (!ok) { break; }
        if (scanf("%lf", &dt) != 1) { break; }
        for (int i = 0; i < 4 && ok; i++) { ok = scanf("%lf", &tilt[i]) == 1; }
        for (int i = 0; i < 4 && ok; i++) { ok = scanf("%lf", &thrust[i]) == 1; }
        if (!ok || scanf("%d", &sup) != 1) { break; }
        FloatAlloc::Result r{};
        alloc.allocate(wd, dt, tilt, thrust, (int8_t)sup, r);
        printf("%.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g %d %u %u\n",
               r.thrust_n[0], r.thrust_n[1], r.thrust_n[2], r.thrust_n[3],
               r.tilt_rad[0], r.tilt_rad[1], r.tilt_rad[2], r.tilt_rad[3],
               r.converged ? 1 : 0, (unsigned)r.fallback, (unsigned)r.iterations);
    }
    return 0;
}
