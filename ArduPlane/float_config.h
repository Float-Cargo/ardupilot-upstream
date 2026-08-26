#pragma once

/*
  Float Cargo feature macros.

  Every piece of Float code is compiled under one of these so that a build can
  be diffed back toward stock ArduPilot behavior by flipping a single flag,
  which is what makes an A/B comparison on the bench or in the simulator a
  one-variable experiment. Defaults are on; override per build with waf's
  --define AP_FLOAT_MODES_ENABLED=0 (which works for SITL and ChibiOS targets
  alike) or, on ChibiOS targets only, with an --extra-hwdef overlay.

  The #ifndef guards are what make the --define override work: waf's --define
  reaches the compiler as -D, which is seen before this header.
 */

#ifndef AP_FLOAT_MODES_ENABLED
#define AP_FLOAT_MODES_ENABLED 1
#endif

#ifndef AP_FLOAT_MOTORS_ENABLED
#define AP_FLOAT_MOTORS_ENABLED 1
#endif
