#include "Rover.h"

// File-scope state: no header change needed.
static bool s_returning_to_center = false;

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    // start in drift/hold (not returning) until pushed outside radius
    s_returning_to_center = false;

    return true; 
}

void ModeLoiter::_exit()
{
    _loiter_radius_override_m = 0.0f;
}

void ModeLoiter::set_radius_override(float radius_m)
{
    _loiter_radius_override_m = MAX(0.0f, radius_m);
}

void ModeLoiter::update()
{
    // distance to the waypoint center
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    // user's configured loiter radius (e.g., 30 m for motorboats, sailboat-specific otherwise)
    float loiter_radius =
        g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;
    if (is_positive(_loiter_radius_override_m)) {
        loiter_radius = _loiter_radius_override_m;
    }

    // "good enough" tolerance around the exact center for stopping a return
    const float center_tol = 3.5f;  // TODO: consider parameterizing (e.g., LOIT_CENTER_TOL)

    // --- STATE UPDATE: decide when to start/stop returning to center ---
    if (_distance_to_destination > loiter_radius) {
        // drifted beyond user ring: begin (or continue) a committed return to center
        s_returning_to_center = true;
    } else if (_distance_to_destination <= center_tol) {
        // close enough to center: stop returning and resume drift/hold behavior
        s_returning_to_center = false;
    }

    if (s_returning_to_center) {
        // --------- DRIVE TO CENTER branch ---------
        // Speed tapers to zero at center_tol to reduce overshoot
        const float dist_eff = MAX(0.0f, _distance_to_destination - center_tol);
        _desired_speed = MIN(dist_eff * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // Heading: aim to center
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);

        // Reverse logic preserved
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd     = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed   = -_desired_speed;
        }

        // Reduce desired speed if yaw error is large (same as original)
        const float yaw_error_ratio =
            1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;

    } else {
        // --------- DRIFT / HOLD branch (inside loiter radius) ---------
        // sailboats should not stop unless motoring
        const float desired_speed_within_radius = g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(
            desired_speed_within_radius, rover.G_Dt);

        // if we have a sail but not trying to use it then point into the wind
        if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
        // Otherwise keep existing desired heading (latched at _enter)
    }

    // 0 turn rate is no limit
    float turn_rate = 0.0f;

    // make sure sailboats don't try and sail directly into the wind
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
