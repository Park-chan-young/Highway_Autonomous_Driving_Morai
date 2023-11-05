/*
    Role:
        Planning the velocity of robot.
    Input:
        Global path
        Road friction coefficient
    Output:
        Planned velocities based on road curvature
*/

class PidControl {
public:
    PidControl();

    double Pid(double target_vel, double current_vel);

private:
    double p_gain_;
    double i_gain_;
    double d_gain_;

    double prev_error_;

    double i_control_;

    double control_time_;
};
