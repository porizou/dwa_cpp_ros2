// -*- C++ -*-
// -*- coding: utf-8 -*-
#ifndef DATA_STRUCT
#define DATA_STRUCT

struct State
{
    double x;        //x軸位置[m]
    double y;        //y軸位置[m]
    double theta;    //姿勢[rad]
    double v;        //x軸速度[m/s]
    double omega;    //回転速度[rad/s]
};

struct Parameter
{
    float max_speed; //最大X軸速度[m/s]
    float min_speed; //最小X軸速度[m/s]
    float max_omega; //最大旋回角速度[rad/s]

    float max_accel; //最大加速度[m/ss]
    float max_accel_omega; //最大旋回角加速度[rad/ss]

    float v_resolution; //速度分解能[m/s]
    float omega_resolution; //旋回角速度分解能[rad/s]

    float predict_time; //軌道予測時間[s]
    float dt; //軌道予測の時間間隔[s]

    double goal_cost_gain;    //ゴールコストゲイン
    double speed_cost_gain;   //スピードコストゲイン
    double obstacle_cost_gain; //障害物コストゲイン

    float robot_radius;  //ロボット半径[m]
};

#endif