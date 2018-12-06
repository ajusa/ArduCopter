#include <iostream>
#include <algorithm>
#include "Copter.h"
#include "Parameters.h"
using namespace std;

/*
 * Init and run calls for custom flight mode (largely based off of the AltHold flight mode)
 */
enum DIR {Front, Right, Back, Left};
DIR dir = Front;
float noise = 10;
int i = 0;
int obstacles = 0;
char mode = 'b';
int del = 0;
// custom_init - initialise custom controller
bool Copter::custom_init(bool ignore_checks)
{
    del = 0;
    dir = Front;
    noise = 10;
    i = 0;
    obstacles = 0;
    mode = 'b';
    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    // stop takeoff if running
    takeoff_stop();

    // reset integrators for roll and pitch controllers
    g.pid_roll.reset_I();
    g.pid_pitch.reset_I();

    return true;
}



// custom_controller - computes target climb rate, roll, pitch, and yaw rate for custom flight mode
// returns true to continue flying, and returns false to land
/*bool avoid_walls(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate){
    float speed = g.custom_param2; //5 degrees?
    float crash = g.custom_param1;
    vector<float> dists(4);
    DIR oldDir = dir;
    for (int i = 0; i < 4; ++i) g2.proximity.get_horizontal_distance(i*90, dists[i]);
    dists[(dir + 2) % 4] = 0; //make the opposite direction 0
    if(i % 20 == 0){
        if(dists[dir] < crash){ //if we hit a wall 10 centimeters away
            dir = (DIR)distance(dists.begin(), max_element(dists.begin(), dists.end()));
            if(oldDir != dir) obstacles++;
        } //then change direction.
    }
    if(obstacles == 5) {
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Landed!", dir);
        return false;
    }
    if(i == 400){ //i is a terrible timer
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Front: %f, Back: %f", dists[0], dists[1]);
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Left: %f, Right: %f", dists[2], dists[3]);
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Longest Path %d", dir);
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Obstacles Avoided %d", obstacles);
        i = 0;
    }
    ++i;
    switch(dir){
        case Front: target_pitch = -speed; break;
        case Right: target_roll = speed; break;
        case Back: target_pitch = speed; break;
        case Left: target_roll = -speed; break;
    }
    return true;
}
bool avoid_door(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate){
    return false;
}*/

bool Copter::custom_controller(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate)
{
    float speed = g.custom_param2; //get our speed in centi-degrees from custom params
    float crash = g.custom_param1; //get the distance at which we want to detect an obstacle and turn
    vector<float> dists(4); //create a vector of distances read by the quadcopter
    for (int i = 0; i < 4; ++i) g2.proximity.get_horizontal_distance(i*90, dists[i]); //fill it with values we read from each direction
    DIR oldDir = dir; //create a copy of our current direction
    dists[(dir + 2) % 4] = 0; //make the opposite direction 0
    if(i % 20 == 0){ //used for timing purposes, this code runs 20 times a second instead of 400
        if(dists[dir] < crash) dir = (DIR)(max_element(dists.begin(), dists.end()) - dists.begin()); //if we see a wall change direction.
        if(oldDir != dir) obstacles++; //if the direction changed, we saw an obstacle
    }
    if(obstacles == 4) return false; //5th obstacle is when we land
    switch(dir){
        case Front: target_pitch = -speed; break;
        case Right: target_roll = speed; break;
        case Back: target_pitch = speed; break;
        case Left: target_roll = -speed; break;
    }
    if(i == 400){ //i is a terrible timer
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Front: %f, Right: %f", dists[0], dists[1]);
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Back: %f, Left: %f", dists[2], dists[3]);
        char human_dir[100];
        if(dir == Front) strcpy(human_dir, "Front");
        if(dir == Right) strcpy(human_dir, "Right");
        if(dir == Back) strcpy(human_dir, "Back");
        if(dir == Left) strcpy(human_dir, "Left");
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Longest Path %s", human_dir);
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Obstacles Avoided %d", obstacles);
        i = 0;
    }
    ++i;
    return true;
}


// custom_run - runs the custom controller
// should be called at 100hz or more

void Copter::custom_run() {
    AltHoldModeState althold_state;
    //writing my code here 
    float takeoff_climb_rate = 0.0f;
    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // desired roll, pitch, and yaw_rate
    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif
    target_climb_rate = 0.0f;

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }
    //althold_state = AltHold_Flying;
    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming = true;
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming = false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming = false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        // compute the target climb rate, roll, pitch and yaw rate
        // land if custom_controller returns false
        if (!custom_controller(target_climb_rate, target_roll, target_pitch, target_yaw_rate)) {
            // switch to land mode
            set_mode(LAND, MODE_REASON_MISSION_END);
            break;
        }

        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}