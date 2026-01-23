#include "robot_control_cpp/kalman_filter.hpp"
#include <cmath>
#include <iostream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

using namespace std;

KalmanFilter::KalmanFilter(const Eigen::Vector2d& x_init)
    : x_hat(x_init)
{
    P = Eigen::MatrixXd::Identity(2, 2);

    A = Eigen::MatrixXd::Identity(2, 2);

    Q = Eigen::MatrixXd::Identity(2, 2);

    R = Eigen::MatrixXd::Identity(2, 2);

    K = Eigen::MatrixXd::Identity(2, 2);

    C = Eigen::MatrixXd::Identity(2, 2);

    fused_odom = Eigen::Vector3d::Zero();
}


void KalmanFilter::prediction(double dt, nav_msgs::msg::Odometry::SharedPtr wheel_msg, double w_L, double w_R)
{   
    static double r = 0.035;
    static double d = 0.23;

    double v = r*((w_R + w_L)/2);
    double omega = r*((w_R - w_L)/(d));
    x_hat(0) = v;
    x_hat(1) = omega;

    A << r/2, r/2,
         r/d, -r/d;

    P = A * P * A.transpose() + Q;
}


void KalmanFilter::correction(double dt, nav_msgs::msg::Odometry::SharedPtr lidar_msg)
{
    double v = lidar_msg->twist.twist.linear.x;
    double omega = lidar_msg->twist.twist.angular.z;
    Eigen::Vector2d y(v, omega);
    
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (y - C * x_hat);
    P = P - K * C * P;

    static double theta_hat = 0;
    theta_hat += x_hat(1) * dt;
    fused_odom(0) += x_hat(0)*sin(theta_hat) * dt;
    fused_odom(1) += x_hat(0)*cos(theta_hat) * dt;
    fused_odom(2) = theta_hat;
}


Eigen::VectorXd KalmanFilter::get_message()
{
    Eigen::VectorXd vector(fused_odom.size() + x_hat.size());
    vector.head(fused_odom.size()) = fused_odom;
    vector.tail(x_hat.size()) = x_hat;

    return vector;
}