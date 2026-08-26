#pragma once

/*
  Float Cargo feature macros, vehicle-side spelling.

  The definitions moved to libraries/AP_FloatConfig/float_config.h when the
  first Float patch reached into libraries/AP_Motors, because a library may
  not include a vehicle header and the two halves of one A/B switch must not
  be two different macros. This file stays so that the include path fc/main
  opened with keeps resolving.
 */

#include <AP_FloatConfig/float_config.h>
