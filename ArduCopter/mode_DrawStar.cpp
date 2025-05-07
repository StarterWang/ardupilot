#include "Copter.h"



/*
 * Init and run calls for guided flight mode
 */
// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks) 
{ 
    wp_control_start();
    path_num = 0;
    generate_path();
    return true;
}

void ModeDrawStar::generate_path()
{
    float radius_cm = g2.star_radius_cm;
    wp_nav->get_wp_stopping_point(path[0]);
    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    if (path_num < 6){
        if (wp_nav->reached_wp_destination()){
            path_num ++;
            wp_nav->set_wp_destination(path[path_num], false);
        }
    }
    // run pause control if the vehicle is paused
    wp_control_run();
}

// initialise guided mode's waypoint navigation controller
void ModeDrawStar::wp_control_start()
{

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    wp_nav->set_wp_destination(path[0], false);
    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run guided mode's waypoint navigation controller
void ModeDrawStar::wp_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

bool ModeDrawStar::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station or scripting
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return (copter.g2.guided_options & (uint32_t)Options::AllowArmingFromTX) != 0;
};

