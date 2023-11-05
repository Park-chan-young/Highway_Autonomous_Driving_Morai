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

#include "pure_pursuit.h"

PurePursuit::PurePursuit() : nh("~")
{
    is_lattice_path_ = false;
    is_odom_ = false;
    is_status_ = false;
    is_global_path_ = false;
    is_look_forward_point_ = false;

    vehicle_length_ = 2.72;
    lfd_ = 6.0;
    min_lfd_ = 2.0;
    max_lfd_ = 15.0;
    lfd_gain_ = 0.78;
    target_velocity_ = 35.0;
    road_friction_ = 0.15;
    
    // Publisher and Subscriber
    ctrl_cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("ctrl_cmd", 10);
    global_path_sub = nh.subscribe("/global_path", 10, &PurePursuit::GlobalPathCallback, this);
    path_sub = nh.subscribe("/lattice_path", 10, &PurePursuit::LatticePathCallback, this);
    odom_sub = nh.subscribe("/odom", 10, &PurePursuit::OdomCallback, this);
    status_sub = nh.subscribe("/Ego_topic", 10, &PurePursuit::StatusCallback, this);

    pid_controller = new PidControl();
    vel_planner = new VelocityPlanning(target_velocity_/3.6, road_friction_);

    while (true)
    {
        if (is_global_path_)
        {
            velocity_list_ = vel_planner-> CurvedBaseVelocity(global_path_msg, 50); // point number = 50
            break;
        }
        else
        {
            ROS_INFO("Waiting for global path data ...");
        }
    }

    ros::Rate rate(20);
    while (ros::ok())
    {
        if(is_lattice_path_ && is_odom_ && is_status_)
        {
            double prev_time = ros::Time().toSec();
            current_waypoint_ = GetCurrentWaypoint(status_msg, global_path_msg);
            target_velocity_ = velocity_list_[current_waypoint_] *3.6;
            double steering = CalcPurePursuit();

            if (is_look_forward_point_)
            {
                ctrl_cmd_msg.steering = steering;
            }
            else
            {
                ROS_INFO("No forward point found.");
                ctrl_cmd_msg.steering = 0.0;
            }

            double output = pid_controller->Pid(target_velocity_, status_msg.velocity.x *3.6);

            if (output > 0.0)
            {
                ctrl_cmd_msg.accel = output;
                ctrl_cmd_msg.brake = 0.0;
                ROS_INFO("Throttle: %f", output);
            }
            else
            {
                ctrl_cmd_msg.accel = 0.0;
                ctrl_cmd_msg.brake = -output;
                ROS_INFO("Brake: %f", -output);
            }

            ctrl_cmd_pub.publish(ctrl_cmd_msg);

            ROS_INFO("Steering: %f", steering);
            ros::Duration loopDuration = ros::Time::now() - ros::Time(prev_time);
            rate.sleep();
        }
        else
        {
            ROS_INFO("Path received: %d", is_lattice_path_);
            ROS_INFO("Odom received: %d", is_odom_);
            ROS_INFO("Status received: %d", is_status_);
        }

        ros::spinOnce();
    }
}

void PurePursuit::LatticePathCallback(const nav_msgs::Path& msg)
{
    is_lattice_path_ = true;
    lattice_path_msg = msg;
}

void PurePursuit::OdomCallback(const nav_msgs::Odometry& msg)
{
    is_odom_ = true;
    odometry_msg = msg;
}

void PurePursuit::StatusCallback(const morai_msgs::EgoVehicleStatus& msg)
{
    is_status_ = true;
    status_msg = msg;
}

void PurePursuit::GlobalPathCallback(const nav_msgs::Path& msg)
{
    global_path_msg = msg;
    is_global_path_ = true;
}

int PurePursuit::GetCurrentWaypoint(const morai_msgs::EgoVehicleStatus& ego_status, const nav_msgs::Path& global_path)
{
    double min_dist = std::numeric_limits<double>::infinity();
    int current_waypoint = -1;
    for (size_t i = 0; i < global_path.poses.size(); ++i)
    {
        double dx = ego_status.position.x - global_path.poses[i].pose.position.x;
        double dy = ego_status.position.y - global_path.poses[i].pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            current_waypoint = static_cast<int>(i);
        }
    }
    return current_waypoint;
}

double PurePursuit::CalcPurePursuit()
{
    lfd_ = status_msg.velocity.x * lfd_gain_;
    if (lfd_ < min_lfd_) {
        lfd_ = min_lfd_;
    } else if (lfd_ > max_lfd_) {
        lfd_ = max_lfd_;
    }
    is_look_forward_point_ = false;
    geometry_msgs::Point translation;
    translation.x = status_msg.position.x;
    translation.y = status_msg.position.y;
    tf::Matrix3x3 trans_matrix;
    trans_matrix.setEulerYPR(0, 0, status_msg.heading);
    tf::Vector3 global_path_point;
    tf::Vector3 local_path_point;
    for (size_t i = 0; i < lattice_path_msg.poses.size(); ++i) {
        const geometry_msgs::Pose& path_pose = lattice_path_msg.poses[i].pose;
        global_path_point.setValue(path_pose.position.x, path_pose.position.y, 1);
        tf::Vector3 local_path_point = trans_matrix.inverse() * global_path_point;
        if (local_path_point.x() > 0) {
            double dis = std::sqrt(local_path_point.x() * local_path_point.x() + local_path_point.y() * local_path_point.y());
            if (dis >= lfd_) {
                forward_point_msg = path_pose.position;
                is_look_forward_point_ = true;
                break;
            }
        }
    }
    if (!is_look_forward_point_) {
        ROS_WARN("No found forward point");
        return 0.0;
    }
    double theta = std::atan2(local_path_point.y(), local_path_point.x());
    double steering = std::atan2(vehicle_length_ * 2 * std::sin(theta), lfd_);
    if (std::isnan(steering)) {
        ROS_ERROR("[ERROR] Steering calculation is NaN.");
        exit(EXIT_FAILURE);
    }
    return steering;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "pure_pursuit_node");  // Initialize the ROS node
    PurePursuit pure_pursuit;  // Create an instance of the PurePursuit class

    ros::spin();  // Enter the ROS spin loop to handle callbacks

    return 0;
}
