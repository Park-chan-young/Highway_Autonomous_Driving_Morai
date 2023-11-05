/*
    Role:
        Planning the velocity of robot.
    Input:
        Global path
        Road friction coefficient
    Output:
        Planned velocities based on road curvature
*/

#include "velocity_planning.h"

VelocityPlanning::VelocityPlanning(double car_max_speed, double road_friction)
{
    car_max_speed_ = car_max_speed;
    road_friction_ = road_friction;
}

std::vector<double> VelocityPlanning::CurvedBaseVelocity(const nav_msgs::Path& global_path, int point_num)
{
    std::vector<double> out_vel_plan;

    for (int i = 0; i < point_num; ++i) {
        out_vel_plan.push_back(car_max_speed_);
    }

    for (int i = point_num; i < static_cast<int>(global_path.poses.size()) - point_num; ++i) {
        std::vector<std::vector<double>> x_list;
        std::vector<double> y_list;

        for (int box = -point_num; box <= point_num; ++box) {
            int idx = i + box;
            double x = global_path.poses[idx].pose.position.x;
            double y = global_path.poses[idx].pose.position.y;
            x_list.push_back({-2 * x, -2 * y, 1});
            y_list.push_back((-x * x) - (y * y));
        }

        Eigen::MatrixXd x_matrix(point_num * 2 + 1, 3);
        Eigen::VectorXd y_matrix(point_num * 2 + 1);

        for (int j = 0; j < point_num * 2 + 1; ++j) {
            x_matrix.row(j) = Eigen::Map<Eigen::MatrixXd>(x_list[j].data(), 1, 3);
            y_matrix(j) = y_list[j];
        }

        // Road Curvature
        Eigen::MatrixXd x_trans = x_matrix.transpose();
        Eigen::MatrixXd a_matrix = (x_trans * x_matrix).inverse() * x_trans * y_matrix;
        double a = a_matrix(0);
        double b = a_matrix(1);
        double c = a_matrix(2);
        double r = std::sqrt(a * a + b * b - c);

        // Velocity based on curvature
        double v_max = std::sqrt(r * 9.8 * road_friction_);

        if (v_max > car_max_speed_) {
            v_max = car_max_speed_;
        }

        out_vel_plan.push_back(v_max);
    }

    for (size_t i = global_path.poses.size() - point_num; i < global_path.poses.size() - 10; ++i) {
        out_vel_plan.push_back(30);
    }

    for (size_t i = global_path.poses.size() - 10; i < global_path.poses.size(); ++i) {
        out_vel_plan.push_back(0);
    }

    return out_vel_plan;
}
