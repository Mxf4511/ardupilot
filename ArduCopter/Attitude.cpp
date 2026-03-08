#include "Copter.h"

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

// update rate controllers and output to roll, pitch and yaw actuators
//  called at 400hz by default
void Copter::run_rate_controller()
{
    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    motors->set_dt(last_loop_time_s);
    attitude_control->set_dt(last_loop_time_s);
    pos_control->set_dt(last_loop_time_s);

    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();
    // reset sysid and other temporary inputs
    attitude_control->rate_controller_target_reset();
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
    // if not armed or landed or on standby then exit
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        fabsf(ahrs.roll_sensor-attitude_control->get_roll_trim_cd()) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }

#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float desired_rate = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// Compute altitude-dependent max descent speed (positive cm/s) for 3-stage descent and rangefinder/EK3 logic per readme
float Copter::get_max_descent_speed_cms() const
{
    const float speed_dn = (g2.pilot_speed_dn > 0) ? (float)abs(g2.pilot_speed_dn) : (float)abs(g.pilot_speed_up);
    const float speed_dn_low = (float)abs(g2.pilot_speed_dn_low);
    const float alt_hi = (float)g2.pilot_dn_alt_high;
    const float alt_lo = (float)g2.pilot_dn_alt_low;

    if (alt_hi <= alt_lo) {
        return speed_dn;
    }

    const float rel_alt_cm = (float)current_loc.alt;
    bool rngf_valid = false;
    float rngf_alt_cm = 0.0f;
    int32_t rngf_alt_cm_i = 0;
#if AP_RANGEFINDER_ENABLED
    rngf_valid = rangefinder_alt_ok() && get_rangefinder_height_interpolated_cm(rngf_alt_cm_i);
    rngf_alt_cm = (float)rngf_alt_cm_i;
    const int16_t rf_max_cm_i = rangefinder.max_distance_cm_orient(ROTATION_PITCH_270);
    const float rf_max_cm = (rf_max_cm_i > 0) ? (float)rf_max_cm_i : 0.0f;
#else
    const float rf_max_cm = 0.0f;
#endif

    // Helper: max descent speed for a given height using standard 3-stage (scenario a)
    const auto speed_at_height = [&](float alt_cm) {
        if (alt_cm >= alt_hi) {
            return speed_dn;
        }
        if (alt_cm <= alt_lo) {
            return speed_dn_low;
        }
        return linear_interpolate(speed_dn_low, speed_dn, alt_cm, alt_lo, alt_hi);
    };

    // (c) Land higher than takeoff: rel_alt > rf_max and rangefinder valid -> use rangefinder height, linear from rf_max->PILOT_SPEED_DN to 0->PILOT_SPD_DN_LOW
#if AP_RANGEFINDER_ENABLED
    if (rf_max_cm > 0.0f && rngf_valid && rel_alt_cm > rf_max_cm) {
        const float rngf_clamped = constrain_float(rngf_alt_cm, 0.0f, rf_max_cm);
        return linear_interpolate(speed_dn_low, speed_dn, rngf_clamped, alt_lo, rf_max_cm);
    }
#endif

    // Height for standard calculation: prefer rangefinder when valid, else EK3 relative altitude
    const float alt_for_speed = rngf_valid ? rngf_alt_cm : rel_alt_cm;
    float speed = speed_at_height(alt_for_speed);

    // (b) Land lower than takeoff: rel_alt < 70%*rf_max and rangefinder invalid -> clamp to speed at 70%*rf_max (do not reduce further)
#if AP_RANGEFINDER_ENABLED
    if (rf_max_cm > 0.0f && !rngf_valid && rel_alt_cm < 0.7f * rf_max_cm) {
        const float speed_at_70 = speed_at_height(0.7f * rf_max_cm);
        speed = MAX(speed, speed_at_70);
    }
#endif

    return speed;
}

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
// When PILOT_SPEED_DN is set, returns altitude-dependent max descent speed (rangefinder/EK3 3-stage logic).
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn == 0) {
        return (uint16_t)abs(g.pilot_speed_up);
    }
    const float speed = get_max_descent_speed_cms();
    return (uint16_t)constrain_float(speed, 0.0f, 65535.0f);
}
