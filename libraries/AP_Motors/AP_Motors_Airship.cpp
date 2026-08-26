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

#include "AP_Motors_Airship.h"

#if AP_MOTORS_AIRSHIP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// The measured bench curve of the flown motor and propeller, normalised.
// Thrust divided by its own maximum against throttle fraction, on the nine
// points of the 2026-05 bench table (Power 46 + APC 11x5.5E). The shape is
// what matters here rather than the magnitude: across the 17 to 21 volt span
// the tables agree on this normalised shape to about two percent while the
// magnitude moves by half, so the voltage lives in the AS_TMAX parameter and
// the shape lives here. Inverting this is what makes a thrust commanded in
// newtons arrive as that many newtons.
static const uint8_t THRUST_CURVE_N = 9;
static const float THRUST_CURVE_FRAC[THRUST_CURVE_N] = {
    0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f
};
static const float THRUST_CURVE_NORM[THRUST_CURVE_N] = {
    0.0f, 0.044355f, 0.114113f, 0.211290f, 0.336290f,
    0.487903f, 0.647581f, 0.830645f, 1.0f
};

const AP_Param::GroupInfo AP_Motors_Airship::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: AS_TMAX
    // @DisplayName: Airship rotor maximum thrust
    // @Description: Per-rotor maximum thrust at the flight pack voltage. This is the magnitude the whole allocation is scaled by: the wrench demand, the thrust disc constraint and the hover estimate are all expressed against it.
    // @Units: N
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("AS_TMAX", 1, AP_Motors_Airship, _tmax_n, 25.84f),

    // @Param: AS_XFORE
    // @DisplayName: Airship fore rotor station
    // @Description: Distance of the fore rotor pair ahead of the centre of volume. Moments are taken about the centre of volume because that is where the plant takes them; the vehicle configuration records actuator positions about the centre of gravity and the difference is a real pitch cross term.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("AS_XFORE", 2, AP_Motors_Airship, _x_fore, 2.3202f),

    // @Param: AS_XAFT
    // @DisplayName: Airship aft rotor station
    // @Description: Distance of the aft rotor pair behind the centre of volume, entered as a negative number. The V0's arms are strongly asymmetric.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("AS_XAFT", 3, AP_Motors_Airship, _x_aft, -1.3718f),

    // @Param: AS_YFORE
    // @DisplayName: Airship fore beam half width
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("AS_YFORE", 4, AP_Motors_Airship, _y_fore, 1.511f),

    // @Param: AS_YAFT
    // @DisplayName: Airship aft beam half width
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("AS_YAFT", 5, AP_Motors_Airship, _y_aft, 1.490f),

    // @Param: AS_ZOFF
    // @DisplayName: Airship rotor height about the centre of volume
    // @Description: Positive is below. The props sit on the hull axis, which is the centre of volume height, so this is zero on the V0 and is carried only so that an off-axis mount does not silently lose its pitch coupling.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("AS_ZOFF", 6, AP_Motors_Airship, _z_off, 0.0f),

    // @Param: AS_ARCMIN
    // @DisplayName: Airship tilt arc lower stop
    // @Description: Tilt is measured from straight up, positive toward forward. The mechanical arc runs from 60 degrees behind vertical-up, through forward and straight down, to 60 degrees behind vertical-down; the 60 degree wedge centred on straight aft is what is missing between the stops.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("AS_ARCMIN", 7, AP_Motors_Airship, _arc_min_deg, -60.0f),

    // @Param: AS_ARCMAX
    // @DisplayName: Airship tilt arc upper stop
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("AS_ARCMAX", 8, AP_Motors_Airship, _arc_max_deg, 240.0f),

    // @Param: AS_ARCRATE
    // @DisplayName: Airship tilt servo rate
    // @Description: The tilt servo is the binding actuator on this airframe, not the motors, so this number sets the per-cycle angular sector the allocation may use.
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("AS_ARCRATE", 9, AP_Motors_Airship, _arc_rate_dps, 43.0f),

    // @Param: AS_MOMR
    // @DisplayName: Airship full-stick roll moment
    // @Units: N*m
    // @User: Advanced
    AP_GROUPINFO("AS_MOMR", 10, AP_Motors_Airship, _mom_roll, 39.0f),

    // @Param: AS_MOMP
    // @DisplayName: Airship full-stick pitch moment
    // @Units: N*m
    // @User: Advanced
    AP_GROUPINFO("AS_MOMP", 11, AP_Motors_Airship, _mom_pitch, 45.0f),

    // @Param: AS_MOMY
    // @DisplayName: Airship full-stick yaw moment
    // @Description: The full-arc yaw couple this airframe can make is about 78 newton-metres against a Munk moment near 10; the stock wedge makes under one. This is the scale the attitude controller's normalised yaw demand is multiplied by.
    // @Units: N*m
    // @User: Advanced
    AP_GROUPINFO("AS_MOMY", 12, AP_Motors_Airship, _mom_yaw, 40.0f),

    // @Param: AS_FXMAX
    // @DisplayName: Airship full-stick surge force
    // @Units: N
    // @User: Advanced
    AP_GROUPINFO("AS_FXMAX", 13, AP_Motors_Airship, _fx_max, 40.0f),

    // @Param: AS_HOVTC
    // @DisplayName: Airship buoyancy trim time constant
    // @Description: Time constant of the filter behind get_throttle_hover(). Buoyancy moves with gas temperature and fabric stretch within minutes of takeoff, so this tracks rather than learns-and-saves; at 0.3 kg/min the lag this introduces is 0.0095 in throttle units per 20 s of time constant. It is long compared with the vertical loop's hunting period so that what is left of the heave transient after AS_HEAVEM and AS_HEAVEC have been subtracted averages out.
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("AS_HOVTC", 14, AP_Motors_Airship, _hover_tc, 20.0f),

    // @Param: AS_HOVINI
    // @DisplayName: Airship buoyancy trim at boot
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("AS_HOVINI", 15, AP_Motors_Airship, _hover_init, 0.32f),

    // @Param: AS_PWMVL
    // @DisplayName: Airship port tilt pulse at vertical
    // @Description: Ship rigging. The port servos run the opposite way to the starboard ones, so their zero is at the top of the band and their pulse falls as the tilt angle rises.
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("AS_PWMVL", 16, AP_Motors_Airship, _pwm_vert_l, 1817.5f),

    // @Param: AS_PWMVR
    // @DisplayName: Airship starboard tilt pulse at vertical
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("AS_PWMVR", 17, AP_Motors_Airship, _pwm_vert_r, 1198.3f),

    // @Param: AS_PWMDEG
    // @DisplayName: Airship tilt microseconds per degree
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("AS_PWMDEG", 18, AP_Motors_Airship, _pwm_per_deg, 3.33333f),

    // @Param: AS_LOG
    // @DisplayName: Airship allocation logging
    // @Description: Write FLAW/FLAM/FLAC on every supervisor cycle. This is what gates/g5_motors_parity.py replays through the Python reference allocator, so it is off by default and on for a parity run.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AS_LOG", 19, AP_Motors_Airship, _alloc_log, 0),

    // @Param: AS_HEAVEM
    // @DisplayName: Airship effective heave mass
    // @Description: Inertial mass plus the Lamb added mass in heave, the coefficient on vertical acceleration in the heave equation the hover estimate inverts: weight = lift - AS_HEAVEM * a_up - AS_HEAVEC * v_up. For the V0 the plant's mass matrix gives 26.6 kg of mass and 20.4 kg of entrained air.
    // @Units: kg
    // @User: Advanced
    AP_GROUPINFO("AS_HEAVEM", 21, AP_Motors_Airship, _heave_mass_kg, 46.9f),

    // @Param: AS_HEAVEC
    // @DisplayName: Airship linear heave damping
    // @Description: Coefficient on vertical velocity in the heave equation the hover estimate inverts. The plant's placeholder residual linear damping in heave; a steady 0.1 m/s climb otherwise reads as 1.8 N of extra weight.
    // @Units: N.s/m
    // @User: Advanced
    AP_GROUPINFO("AS_HEAVEC", 22, AP_Motors_Airship, _heave_damp_nsm, 18.0f),

    // @Param: AS_SUPER
    // @DisplayName: Airship supervisor period
    // @Description: Control cycles between full supervisor solves. The supervisor enumerates the sixteen assignments of the four rotors to the two half cones that cover the arc and is what lets a rotor cross the excluded wedge; the operating point moves slowly enough that it need not run every cycle.
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("AS_SUPER", 20, AP_Motors_Airship, _sup_period, 10),

    AP_GROUPEND
};

AP_Motors_Airship::AP_Motors_Airship(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        _tilt_state_rad[i] = 0.0;
        _tilt_cmd_rad[i] = 0.0;
        _thrust_n[i] = 0.0;
        _actuator[i] = 0.0f;
    }
    for (uint8_t i = 0; i < FloatAlloc::NW; i++) {
        _w_des[i] = 0.0;
        _w_realized[i] = 0.0;
    }
    _hover_est = 0.32f;
    _climb_rate_ms = 0.0f;
    _accel_d_mss = 0.0f;
    _air_state_valid = false;
    _geometry_ready = false;
    set_update_rate(speed_hz);
}

void AP_Motors_Airship::build_geometry()
{
    // Rotor order is the ship's: fore port, fore starboard, aft port, aft
    // starboard, which is servo outputs 9 to 12 in that order and therefore
    // motor functions 3, 1, 2 and 4.
    const double xf = (double)_x_fore.get();
    const double xa = (double)_x_aft.get();
    const double yf = (double)_y_fore.get();
    const double ya = (double)_y_aft.get();
    const double z = (double)_z_off.get();
    const double pos[FloatAlloc::NROTOR][3] = {
        { xf, -yf, z },
        { xf,  yf, z },
        { xa, -ya, z },
        { xa,  ya, z },
    };
    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            _geom.pos[i][j] = pos[i][j];
        }
    }
    _geom.thrust_max_n = (double)MAX(_tmax_n.get(), 0.1f);
    _geom.arc_min_rad = (double)radians(_arc_min_deg.get());
    _geom.arc_max_rad = (double)radians(_arc_max_deg.get());
    _geom.tilt_rate_rad_s = (double)radians(MAX(_arc_rate_dps.get(), 1.0f));

    // Sway is weighted to zero because it is unreachable; asking the
    // optimiser to trade against a component no actuator touches only
    // corrupts the components that are reachable. Moments are weighted an
    // order above forces because this ship's failures are attitude failures.
    const double q_w[FloatAlloc::NW] = { 1.0, 0.0, 1.0, 10.0, 10.0, 10.0 };
    for (uint8_t i = 0; i < FloatAlloc::NW; i++) {
        _weights.q_w[i] = q_w[i];
    }
    for (uint8_t i = 0; i < FloatAlloc::NU; i++) {
        _weights.r_u[i] = 1.0;
    }
    _weights.gamma = 1e-4;

    _alloc.init(_geom, _weights, (uint8_t)MAX(_sup_period.get(), (int8_t)1));
    _geometry_ready = true;
}

void AP_Motors_Airship::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    (void)frame_type;
    // The ship's canonical map: motors on outputs 9 to 12 as functions
    // motor3, motor1, motor2, motor4, tilts on outputs 5 to 8 as the four
    // tilt functions. Defaults only — the parameter file is the authority.
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor3, CH_9);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_10);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_11);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor4, CH_12);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_5);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_6);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRearLeft, CH_7);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRearRight, CH_8);

    build_geometry();
    _hover_est = constrain_float(_hover_init.get(), 0.01f, 0.95f);

    _mav_type = MAV_TYPE_AIRSHIP;
    set_initialised_ok(frame_class == MOTOR_FRAME_AIRSHIP);
}

void AP_Motors_Airship::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    (void)frame_type;
    set_initialised_ok(frame_class == MOTOR_FRAME_AIRSHIP);
}

void AP_Motors_Airship::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor1, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor2, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor3, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor4, speed_hz);
}

uint32_t AP_Motors_Airship::get_motor_mask()
{
    uint32_t mask = 0;
    const SRV_Channel::Function fns[FloatAlloc::NROTOR] = {
        SRV_Channel::k_motor3, SRV_Channel::k_motor1,
        SRV_Channel::k_motor2, SRV_Channel::k_motor4
    };
    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        uint8_t chan;
        if (SRV_Channels::find_channel(fns[i], chan)) {
            mask |= 1U << chan;
        }
    }
    mask |= AP_MotorsMulticopter::get_motor_mask();
    return mask;
}

void AP_Motors_Airship::set_air_state(float climb_rate_ms, float accel_d_mss)
{
    _climb_rate_ms = climb_rate_ms;
    _accel_d_mss = accel_d_mss;
    _air_state_valid = true;
}

float AP_Motors_Airship::get_throttle_hover() const
{
    // Deliberately NOT constrained to AP_MOTORS_THST_HOVER_MIN. The 0.125
    // floor is a multicopter assumption; on a near-neutrally-buoyant hull the
    // true value can sit at half of it, and clamping it makes the vertical
    // law fight a feedforward it cannot switch off.
    return constrain_float(_hover_est, 0.01f, 0.95f);
}

void AP_Motors_Airship::update_hover_estimate(float dt)
{
    if (dt <= 0.0f || !is_positive(_hover_tc.get())) {
        return;
    }
    if (_spool_state != SpoolState::THROTTLE_UNLIMITED || !_air_state_valid) {
        return;
    }
    // The heave equation, solved for the weight the rotors are carrying:
    //
    //     L = W + m_eff * a_up + c * v_up      so      W = L - m_eff a_up - c v_up
    //
    // The acceleration term is not small on this hull: the entrained air
    // nearly doubles the mass in heave, so the vertical loop's ordinary
    // hunting at a few tenths of a metre per second squared moves the lift by
    // a tenth of full scale, and an estimate that only gates on "quiet
    // enough" admits exactly the turning points where that error peaks.
    // Subtracting the modelled terms is what lets the filter see the weight
    // through the transient rather than waiting for one that never comes.
    // The gates that remain guard against manoeuvres, not against hunting.
    if (fabsf(_climb_rate_ms) > 1.0f || fabsf(_accel_d_mss) > 1.0f) {
        return;
    }
    const float total = 4.0f * MAX(_tmax_n.get(), 0.1f);
    const float lift_n = (float)(-_w_realized[2]);
    const float a_up = -_accel_d_mss;
    const float weight_n = lift_n - _heave_mass_kg.get() * a_up - _heave_damp_nsm.get() * _climb_rate_ms;
    const float alpha = dt / (dt + _hover_tc.get());
    _hover_est += alpha * (constrain_float(weight_n / total, 0.0f, 1.0f) - _hover_est);
    _hover_est = constrain_float(_hover_est, 0.01f, 0.95f);
}

float AP_Motors_Airship::thrust_to_actuator_n(double thrust) const
{
    const float tn = constrain_float((float)thrust / MAX(_tmax_n.get(), 0.1f), 0.0f, 1.0f);
    for (uint8_t i = 1; i < THRUST_CURVE_N; i++) {
        if (tn <= THRUST_CURVE_NORM[i]) {
            const float span = THRUST_CURVE_NORM[i] - THRUST_CURVE_NORM[i - 1];
            const float f = is_positive(span) ? (tn - THRUST_CURVE_NORM[i - 1]) / span : 0.0f;
            return THRUST_CURVE_FRAC[i - 1] + f * (THRUST_CURVE_FRAC[i] - THRUST_CURVE_FRAC[i - 1]);
        }
    }
    return 1.0f;
}

uint16_t AP_Motors_Airship::tilt_pwm(uint8_t i, double tilt) const
{
    // Port rotors (index 0 and 2) run the servo the opposite way to the
    // starboard pair; both zeros come from the firmware's own hover output
    // rather than from the rigging trim, which is a distinction that once
    // cost a phantom standing yaw attractor.
    const bool port = (i == 0) || (i == 2);
    const float vert = port ? _pwm_vert_l.get() : _pwm_vert_r.get();
    const float dir = port ? -1.0f : 1.0f;
    const float deg = degrees((float)tilt);
    const float pwm = vert + dir * deg * _pwm_per_deg.get();
    return (uint16_t)constrain_float(pwm, 800.0f, 2200.0f);
}

void AP_Motors_Airship::output_armed_stabilizing()
{
    if (!_geometry_ready) {
        build_geometry();
    }
    float dt = get_dt_s();
    if (!(dt > 0.0f) || dt > 0.2f) {
        dt = 1.0f / MAX((float)_speed_hz, 1.0f);
    }

    const float compensation_gain = thr_lin.get_compensation_gain();
    const float roll_in = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_in = (_pitch_in + _pitch_in_ff) * compensation_gain;
    const float yaw_in = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle_in = get_throttle() * compensation_gain;
    throttle_in = constrain_float(throttle_in, 0.0f, _throttle_thrust_max * compensation_gain);

    const float total = 4.0f * MAX(_tmax_n.get(), 0.1f);

    // The wrench demand. Body FRD about the centre of volume, so up is
    // negative z. Sway is identically zero: no actuator on this airframe
    // makes it and the guidance discipline never asks for it.
    _w_des[0] = (double)constrain_float(_forward_in, -1.0f, 1.0f) * (double)_fx_max.get();
    _w_des[1] = 0.0;
    _w_des[2] = -(double)throttle_in * (double)total;
    _w_des[3] = (double)constrain_float(roll_in, -1.0f, 1.0f) * (double)_mom_roll.get();
    _w_des[4] = (double)constrain_float(pitch_in, -1.0f, 1.0f) * (double)_mom_pitch.get();
    _w_des[5] = (double)constrain_float(yaw_in, -1.0f, 1.0f) * (double)_mom_yaw.get();

    // The state handed to the allocator, kept because the parity gate has to
    // replay the exact inputs of the cycle it is checking and both arrays are
    // about to be overwritten with the cycle's answer.
    double tilt_in[FloatAlloc::NROTOR];
    double thrust_in[FloatAlloc::NROTOR];
    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        tilt_in[i] = _tilt_state_rad[i];
        thrust_in[i] = _thrust_n[i];
    }

    _alloc.allocate(_w_des, dt, tilt_in, thrust_in, -1, _result);

    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        _thrust_n[i] = _result.thrust_n[i];
        _tilt_cmd_rad[i] = _result.tilt_rad[i];
    }
    for (uint8_t i = 0; i < FloatAlloc::NW; i++) {
        _w_realized[i] = _result.wrench[i];
    }

    // Advance the modelled tilt state at the servo's own rate. The servo is
    // unsensed on this ship, so this model is the only tilt feedback the
    // allocator has; it is the same rate limiter the plant runs.
    const double step = _geom.tilt_rate_rad_s * dt;
    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        double d = _tilt_cmd_rad[i] - _tilt_state_rad[i];
        if (d > step) {
            d = step;
        } else if (d < -step) {
            d = -step;
        }
        _tilt_state_rad[i] += d;
    }

    // Saturation flags, so the attitude controller stops winding an
    // integrator into an axis the airframe has run out of authority in.
    const float mom_floor = 1.0f;
    const float force_floor = 1.0f;
    limit.roll = fabsf((float)(_w_realized[3] - _w_des[3])) >
                 0.05f * MAX(fabsf((float)_w_des[3]), mom_floor);
    limit.pitch = fabsf((float)(_w_realized[4] - _w_des[4])) >
                  0.05f * MAX(fabsf((float)_w_des[4]), mom_floor);
    limit.yaw = fabsf((float)(_w_realized[5] - _w_des[5])) >
                0.05f * MAX(fabsf((float)_w_des[5]), mom_floor);
    const float fz_err = (float)(_w_realized[2] - _w_des[2]);
    limit.throttle_upper = fz_err > 0.05f * MAX(fabsf((float)_w_des[2]), force_floor);
    limit.throttle_lower = fz_err < -0.05f * MAX(fabsf((float)_w_des[2]), force_floor);

    // _throttle_out is what the rest of the vehicle reads as "how hard are
    // the motors working" — notch tracking, control-surface scaling, the
    // throttle-suppression logic. The honest number for this frame is the
    // realised vertical lift as a fraction of what the rotors could make.
    _throttle_out = constrain_float((float)(-_w_realized[2]) / total, 0.0f, 1.0f);

    update_hover_estimate(dt);

#if HAL_LOGGING_ENABLED
    const uint8_t period = (uint8_t)MAX(_sup_period.get(), (int8_t)1);
    if (_alloc_log.get() != 0 && (period == 1 || (_alloc.cycle() % period) == 1)) {
        log_allocation(_w_des, dt, tilt_in, thrust_in);
    }
#endif
}

void AP_Motors_Airship::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }
    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
            _actuator[i] = 0.0f;
        }
        break;
    case SpoolState::GROUND_IDLE:
        for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
            set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
        }
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
            set_actuator_with_slew(_actuator[i], thrust_to_actuator_n(_thrust_n[i]));
        }
        break;
    }

    const SRV_Channel::Function motor_fn[FloatAlloc::NROTOR] = {
        SRV_Channel::k_motor3, SRV_Channel::k_motor1,
        SRV_Channel::k_motor2, SRV_Channel::k_motor4
    };
    const SRV_Channel::Function tilt_fn[FloatAlloc::NROTOR] = {
        SRV_Channel::k_tiltMotorLeft, SRV_Channel::k_tiltMotorRight,
        SRV_Channel::k_tiltMotorRearLeft, SRV_Channel::k_tiltMotorRearRight
    };
    for (uint8_t i = 0; i < FloatAlloc::NROTOR; i++) {
        SRV_Channels::set_output_pwm(motor_fn[i], output_to_pwm(_actuator[i]));
        // The tilt pulse is written directly rather than through a scaled
        // range because the arc is 300 degrees and the servo range parameters
        // on this ship describe the sixty degrees native firmware used to be
        // able to ask for. The rigging model here is the plant's.
        SRV_Channels::set_output_pwm(tilt_fn[i], tilt_pwm(i, _tilt_cmd_rad[i]));
    }
}

void AP_Motors_Airship::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    const SRV_Channel::Function motor_fn[FloatAlloc::NROTOR] = {
        SRV_Channel::k_motor3, SRV_Channel::k_motor1,
        SRV_Channel::k_motor2, SRV_Channel::k_motor4
    };
    if (motor_seq >= 1 && motor_seq <= FloatAlloc::NROTOR) {
        SRV_Channels::set_output_pwm(motor_fn[motor_seq - 1], pwm);
    }
}

#if HAL_LOGGING_ENABLED
void AP_Motors_Airship::log_allocation(const double w_des[FloatAlloc::NW], float dt,
                                       const double tilt_in[FloatAlloc::NROTOR],
                                       const double thrust_in[FloatAlloc::NROTOR]) const
{
    const uint64_t now = AP_HAL::micros64();
    // Three messages rather than one because the dataflash label string is 64
    // characters and twenty-two named fields do not fit in it. They share a
    // timestamp, which is what the parity gate joins them on.
    AP::logger().WriteStreaming(
        "FLAW", "TimeUS,Fx,Fz,Mx,My,Mz,dt,Hov,FB", "Qfffffffb", now,
        (float)w_des[0], (float)w_des[2], (float)w_des[3],
        (float)w_des[4], (float)w_des[5], dt, _hover_est,
        (int8_t)_result.fallback);
    AP::logger().WriteStreaming(
        "FLAM", "TimeUS,M0,M1,M2,M3,H0,H1,H2,H3", "Qffffffff", now,
        (float)tilt_in[0], (float)tilt_in[1], (float)tilt_in[2], (float)tilt_in[3],
        (float)thrust_in[0], (float)thrust_in[1], (float)thrust_in[2], (float)thrust_in[3]);
    AP::logger().WriteStreaming(
        "FLAC", "TimeUS,C0,C1,C2,C3,T0,T1,T2,T3", "Qffffffff", now,
        (float)_result.tilt_rad[0], (float)_result.tilt_rad[1],
        (float)_result.tilt_rad[2], (float)_result.tilt_rad[3],
        (float)_result.thrust_n[0], (float)_result.thrust_n[1],
        (float)_result.thrust_n[2], (float)_result.thrust_n[3]);
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_MOTORS_AIRSHIP_ENABLED
