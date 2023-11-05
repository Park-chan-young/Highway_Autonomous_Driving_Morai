/*
    Role:
        This class is used for planning the robot's velocity.
        It plans the robot's velocity based on the given global path and road friction coefficient, considering the curvature of the road.
    Input:
        Global path
        Road friction coefficient
    Output:
        Planned velocities based on road curvature
*/

#include <vector>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <nav_msgs/Path.h>

class VelocityPlanning
{
public:
    VelocityPlanning(double car_max_speed, double road_friction);

    double car_max_speed_;
    double road_friction_;

    // Function
    std::vector<double> CurvedBaseVelocity(const nav_msgs::Path& global_path, int point_num);
};
