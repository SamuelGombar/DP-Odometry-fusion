// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp>


struct Point {
    float x;
    float y;
};

class KalmanFilter {
public:
    explicit KalmanFilter(const Eigen::Vector2d& x_init);

    void prediction(double dt, nav_msgs::msg::Odometry::SharedPtr wheel_msg, double w_L, double w_R);

    void correction(double dt, nav_msgs::msg::Odometry::SharedPtr lidar_msg);

    Eigen::VectorXd get_message();

private:
    Eigen::MatrixXd P;

    Eigen::MatrixXd A;

    Eigen::MatrixXd Q;

    Eigen::MatrixXd R;

    Eigen::MatrixXd K;

    Eigen::MatrixXd C;

    Eigen::Vector2d x_hat;

    Eigen::Vector3d fused_odom;
};

#endif
