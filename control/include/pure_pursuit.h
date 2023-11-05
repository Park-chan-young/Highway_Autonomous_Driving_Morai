/*
    Role:
        Pure pursuit controller.
    Input:
        /global_path: Global path
        /lattice_path: Local path
        /odom: Robot's odometry
        /Ego_topic: Robot's status
    Output:
        /ctrl_cmd: Lateral and longitudinal control commands
*/

#include "pid_control.h"
#include "velocity_planning.h"

#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include "morai_msgs/CtrlCmd.h"
#include "morai_msgs/EgoVehicleStatus.h"


class PurePursuit
{
public:
    PurePursuit();

    bool is_lattice_path_;
    bool is_odom_;
    bool is_status_;
    bool is_global_path_;
    bool is_look_forward_point_;

    double vehicle_length_;
    double lfd_;
    double min_lfd_;
    double max_lfd_;
    double lfd_gain_;
    double target_velocity_;
    double road_friction_;
    double current_waypoint_;\

    std::vector<double> velocity_list_;

    nav_msgs::Path lattice_path_msg;
    nav_msgs::Path global_path_msg;
    nav_msgs::Odometry odometry_msg;
    morai_msgs::EgoVehicleStatus status_msg;
    morai_msgs::CtrlCmd ctrl_cmd_msg;

    geometry_msgs::Point forward_point_msg;
    geometry_msgs::Point current_position_msg;

    ros::NodeHandle nh;
    ros::Publisher ctrl_cmd_pub;
    ros::Subscriber global_path_sub;
    ros::Subscriber path_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber status_sub;

    PidControl* pid_controller;
    VelocityPlanning* vel_planner;

    // Callback
    void LatticePathCallback(const nav_msgs::Path& msg);
    void OdomCallback(const nav_msgs::Odometry& msg);
    void StatusCallback(const morai_msgs::EgoVehicleStatus& msg);
    void GlobalPathCallback(const nav_msgs::Path& msg);

    // Function
    int GetCurrentWaypoint(const morai_msgs::EgoVehicleStatus& ego_status, const nav_msgs::Path& global_path);
    double CalcPurePursuit();
};
