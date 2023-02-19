// -*- C++ -*-
// -*- coding: utf-8 -*-
#ifndef DATA_STRUCT
#define DATA_STRUCT

struct State
{
    double x; // x-axis position [m]
    double y; // y-axis position [m]
    double theta; // posture [rad]
    double v; // x-axis velocity [m/s]
    double omega; // angular velocity [rad/s]
};

struct Parameter
{
    double max_speed; // maximum x-axis velocity [m/s]
    double min_speed; // minimum x-axis velocity [m/s]
    double max_omega; // maximum angular velocity [rad/s]

    double max_accel; // maximum acceleration [m/s^2]
    double max_accel_omega; // maximum angular acceleration [rad/s^2]

    double v_resolution; // velocity resolution [m/s]
    double omega_resolution; // angular velocity resolution [rad/s]

    double predict_time; // trajectory prediction time [s]
    double dt; // time interval for trajectory prediction [s]

    double goal_cost_gain;    // goal cost gain
    double speed_cost_gain;   // speed cost gain
    double obstacle_cost_gain; // obstacle cost gain

    double robot_radius;  // robot radius [m]

    double goal_tolerance; // threshold for determining arrival at the goal
};

#endif