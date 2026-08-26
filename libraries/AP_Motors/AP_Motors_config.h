#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <SRV_Channel/SRV_Channel_config.h>

#ifndef AP_MOTORS_MAX_NUM_MOTORS
#if AP_SCRIPTING_ENABLED
#define AP_MOTORS_MAX_NUM_MOTORS 32
#else
#define AP_MOTORS_MAX_NUM_MOTORS 12
#endif

// doesn't make sense to have more motors than servo channels, so clamp:
#if NUM_SERVO_CHANNELS < AP_MOTORS_MAX_NUM_MOTORS
#undef AP_MOTORS_MAX_NUM_MOTORS
#define AP_MOTORS_MAX_NUM_MOTORS NUM_SERVO_CHANNELS
#endif

// various Motors backends will not compile if we don't have 16 motors
// available (eg. AP_Motors6DOF).  Until we stop compiling those
// backends in when there aren't enough motors to support those
// backends we will support a minimum of 12 motors, the limit before
// we moved to 32 motor support:
#if AP_MOTORS_MAX_NUM_MOTORS < 12
#undef AP_MOTORS_MAX_NUM_MOTORS
#define AP_MOTORS_MAX_NUM_MOTORS 12
#endif

#endif  // defined (AP_MOTORS_MAX_NUM_MOTORS)

#ifndef AP_MOTORS_TRI_ENABLED
#define AP_MOTORS_TRI_ENABLED 1
#endif  // AP_MOTORS_TRI_ENABLED

#ifndef AP_MOTORS_FRAME_DEFAULT_ENABLED
#define AP_MOTORS_FRAME_DEFAULT_ENABLED 1
#endif

#ifndef AP_MOTORS_FRAME_QUAD_ENABLED
#define AP_MOTORS_FRAME_QUAD_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_HEXA_ENABLED
#define AP_MOTORS_FRAME_HEXA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_OCTA_ENABLED
#define AP_MOTORS_FRAME_OCTA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_DECA_ENABLED
#define AP_MOTORS_FRAME_DECA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#define AP_MOTORS_FRAME_DODECAHEXA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_Y6_ENABLED
#define AP_MOTORS_FRAME_Y6_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#define AP_MOTORS_FRAME_OCTAQUAD_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

// scale factor for top layer to prevent beat frequency between top and bottom
// layers of co-rotating motors. Must be less than 1.0
#ifndef AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR
#define AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR 0.9
#endif

// The Float airship allocation frame class. It carries a dense quadratic
// program and a sixteen-way sector supervisor, so it is opt-in per board
// rather than on by default; the Float boards and SITL turn it on.
// It follows AP_FLOAT_MOTORS_ENABLED when that has been set on the command
// line, so that one --define turns the whole Float mixer off across the
// vehicle directory and this library together.
#ifndef AP_MOTORS_AIRSHIP_ENABLED
#if defined(AP_FLOAT_MOTORS_ENABLED)
#define AP_MOTORS_AIRSHIP_ENABLED AP_FLOAT_MOTORS_ENABLED
#else
#define AP_MOTORS_AIRSHIP_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#endif
