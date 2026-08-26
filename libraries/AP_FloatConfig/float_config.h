#pragma once

/*
  Float Cargo feature macros.

  Every piece of Float code is compiled under one of these so that a build can
  be diffed back toward stock ArduPilot behavior by flipping a single flag,
  which is what makes an A/B comparison on the bench or in the simulator a
  one-variable experiment. Defaults are on; override per build with waf's
  --define AP_FLOAT_PATCHES_ENABLED=0 (which works for SITL and ChibiOS
  targets alike) or, on ChibiOS targets only, with an --extra-hwdef overlay.

  The #ifndef guards are what make the --define override work: waf's --define
  reaches the compiler as -D, which is seen before this header.

  This header lives under libraries/ rather than in the vehicle directory
  because the Float diff reaches into libraries/AP_Motors, and a library may
  not include a vehicle header. ArduPlane/float_config.h includes this file,
  so the vehicle-side spelling that fc/main opened with still works.
 */

#ifndef AP_FLOAT_MODES_ENABLED
#define AP_FLOAT_MODES_ENABLED 1
#endif

#ifndef AP_FLOAT_MOTORS_ENABLED
#define AP_FLOAT_MOTORS_ENABLED 1
#endif

/*
  AP_FLOAT_PATCHES_ENABLED covers the four surgical edits to the stock
  quadplane path (Atlas projects/controls-test.md G4, from
  test-flights/docs/Stabilized_forward_mode_design.md section 3):

    1. the hover-feedforward floor becomes the Q_M_THST_HVR_MIN parameter
       instead of the hardcoded 0.125, on both the read and the learn clamp;
    2. commanded tilt may pass 90 degrees up to Q_TILT_MAX_EXT, so the
       nacelles can reach the powered-descent band the servo arc already has;
    3. the vectored-yaw wedge gets its own limit, Q_TILT_YAW_MAX, decoupled
       from Q_TILT_YAW_ANGLE's role in the tilt-to-servo mapping;
    4. Tiltrotor::vectoring() gains a fore/aft pitch term scaled by
       sin(tilt), which is the pitch authority that cos(tilt) takes away.

  With the macro off the compiled behavior is stock Plane-4.7.0 in all four
  places: every edit below is inside a guard and every added parameter is
  compiled out with it.
 */
#ifndef AP_FLOAT_PATCHES_ENABLED
#define AP_FLOAT_PATCHES_ENABLED 1
#endif
