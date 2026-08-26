/// @file   AP_Motors_Airship.h
/// @brief  Wrench-based control allocation for the Float V0 airship.
///
/// The stock quadplane mixer answers "what fraction of each motor's thrust
/// makes this roll demand?" with a table of constant factors. An airship with
/// four rotors on a 300 degree tilt arc has no such table: the same rotor
/// makes heave, surge, yaw and pitch depending only on where its servo is
/// pointing, and the useful question is the other way round — given a desired
/// body wrench, which four thrusts and four tilt angles realise it. That
/// question is a constrained quadratic program, it is what this class solves
/// every control cycle, and `AP_Motors_Airship_Alloc` is where the solving
/// lives.
///
/// Three consequences are worth naming before the declarations.
///
/// The tilt servos are written here rather than by `Tiltrotor::vectoring()`.
/// For this frame class tilt is an allocation output, not a mode-dependent
/// schedule, so `Q_TILT_ENABLE` is expected to be zero and this class owns
/// the four tilt functions directly, in the ship's own PWM rigging.
///
/// `get_throttle_hover()` is a live buoyancy-trim estimate and it is not
/// clamped at `AP_MOTORS_THST_HOVER_MIN`. That floor is 0.125 against a true
/// value that runs from roughly 0.06 to 0.35 depending on ballast, and it
/// moves within minutes of takeoff as the gas warms; a constant here is a
/// modelling error, not a safety margin.
///
/// Sway is never demanded. The Fy row of the effectiveness matrix is
/// structurally zero across all eight rotor columns, so a lateral force
/// request is not something this airframe can trade against — guidance
/// resolves lateral error by heading plus surge instead.
#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_AIRSHIP_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMulticopter.h"
#include "AP_Motors_Airship_Alloc.h"

class AP_Motors_Airship : public AP_MotorsMulticopter {
public:
    AP_Motors_Airship(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_update_rate(uint16_t speed_hz) override;
    void output_to_motors() override;
    uint32_t get_motor_mask() override;

    // The whole point of the class: unclamped, live, and the vertical law's
    // feedforward slot in AC_PosControl.
    float get_throttle_hover() const override;

    // Vertical state, pushed in by QuadPlane because a motors library has no
    // business reaching into the attitude and height estimate itself. The
    // buoyancy-trim filter only integrates while the ship is quasi-steady in
    // the vertical, which is when commanded vertical force equals heaviness.
    void set_air_state(float climb_rate_ms, float accel_d_mss);

    // Per-rotor allocation outputs, in SI units, for the parity gate and for
    // anything upstream that wants to know what was actually asked of the
    // airframe rather than what came out of a servo.
    float thrust_n(uint8_t i) const { return (i < FloatAlloc::NROTOR) ? (float)_thrust_n[i] : 0.0f; }
    float tilt_rad(uint8_t i) const { return (i < FloatAlloc::NROTOR) ? (float)_tilt_cmd_rad[i] : 0.0f; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void output_armed_stabilizing() override;
    const char* _get_frame_string() const override { return "AIRSHIP"; }
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

private:
    void build_geometry();
    void update_hover_estimate(float dt);
    // Thrust in newtons to a normalised actuator command, through the inverse
    // of the measured bench curve rather than through the generic thrust
    // linearisation: the plant realises the bench curve exactly, so this is
    // what makes commanded newtons and delivered newtons the same number.
    float thrust_to_actuator_n(double thrust) const;
    // Commanded tilt angle to a servo pulse in the ship's own rigging.
    uint16_t tilt_pwm(uint8_t i, double tilt_rad) const;
#if HAL_LOGGING_ENABLED
    void log_allocation(const double w_des[FloatAlloc::NW], float dt,
                        const double tilt_in[FloatAlloc::NROTOR],
                        const double thrust_in[FloatAlloc::NROTOR]) const;
#endif

    FloatAlloc::Allocator _alloc;
    FloatAlloc::Geometry _geom;
    FloatAlloc::Weights _weights;
    FloatAlloc::Result _result;

    // modelled tilt state: the servo is rate limited and unsensed, so the
    // allocator is handed the position the servo can actually be at rather
    // than the position it was last told to reach
    double _tilt_state_rad[FloatAlloc::NROTOR];
    double _thrust_n[FloatAlloc::NROTOR];
    double _tilt_cmd_rad[FloatAlloc::NROTOR];
    double _w_des[FloatAlloc::NW];
    double _w_realized[FloatAlloc::NW];
    float _actuator[FloatAlloc::NROTOR];

    float _hover_est;           // throttle units, the live buoyancy trim
    float _climb_rate_ms;
    float _accel_d_mss;
    bool _air_state_valid;
    bool _geometry_ready;

    // parameters
    AP_Float _tmax_n;           // per rotor maximum thrust, newtons
    AP_Float _x_fore;           // rotor stations and beams about the CENTRE OF VOLUME
    AP_Float _x_aft;
    AP_Float _y_fore;
    AP_Float _y_aft;
    AP_Float _z_off;
    AP_Float _arc_min_deg;      // mechanical stops of the tilt arc
    AP_Float _arc_max_deg;
    AP_Float _arc_rate_dps;
    AP_Float _mom_roll;         // full stick demands, newton-metres
    AP_Float _mom_pitch;
    AP_Float _mom_yaw;
    AP_Float _fx_max;           // full forward demand, newtons
    AP_Float _hover_tc;         // buoyancy trim filter time constant, seconds
    AP_Float _hover_init;       // buoyancy trim at boot, throttle units
    AP_Float _heave_mass_kg;    // mass plus heave added mass, the a_up coefficient
    AP_Float _heave_damp_nsm;   // linear heave damping, the v_up coefficient
    AP_Float _pwm_vert_l;       // tilt rigging: pulse at vertical, port side
    AP_Float _pwm_vert_r;       // tilt rigging: pulse at vertical, starboard side
    AP_Float _pwm_per_deg;      // tilt rigging: microseconds per degree
    AP_Int8  _alloc_log;        // log the allocation, one record per supervisor cycle
    AP_Int8  _sup_period;       // control cycles between supervisor solves
};

#endif  // AP_MOTORS_AIRSHIP_ENABLED
