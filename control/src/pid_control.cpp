/*
    Role:
        PID (Proportional-Integral-Derivative) controller for velocity control.
    Input:
        Target velocity
        Current velocity
    Output:
        Acceleration or braking command
*/

#include "pid_control.h"

PidControl::PidControl()
{
    p_gain_ = 0.50;
    i_gain_ = 0.00068;
    d_gain_ = 0.035;
    prev_error_ = 0.0;
    i_control_ = 0.0;
    control_time_ = 0.02;
}

double PidControl::Pid(double target_vel, double current_vel)
{
    double error = target_vel - current_vel;

    double p_control = p_gain_ * error;
    i_control_ += i_gain_ * error * control_time_;
    double d_control = d_gain_ * (error - prev_error_) / control_time_;

    double output = p_control + i_control_ + d_control;
    prev_error_ = error;

    return output;
}
