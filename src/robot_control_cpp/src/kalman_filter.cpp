#include "robot_control_cpp/kalman_filter.hpp"
#include <cmath>
#include <iostream>

using namespace std;

KalmanFilter::KalmanFilter(const Eigen::Vector3d& x_init)
    : x_hat(x_init)
{
    A = Eigen::Matrix3d::Identity();

    P = Eigen::Matrix3d::Identity();

    Q = Eigen::Matrix3d::Identity();

    C = Eigen::Matrix3d::Identity();

    R = Eigen::Matrix3d::Identity();

    R *= 100;
}


void KalmanFilter::prediction(Eigen::Vector3d wheel_odom, double velocity)
{

    // x_hat = wheel_odom;
    x_hat(0) += v * std::cos(wheel_odom(2)) * dt;
    x_hat(1) += v * std::sin(wheel_odom(2)) * dt;
    x_hat(2) += omega * dt;

    A << 1, 0, -velocity * (-std::sin(wheel_odom(2))),
         0, 1,  velocity * std::cos(wheel_odom(2)),
         0, 0, 1;
    P = A * P * A.transpose() + Q;
}


void KalmanFilter::correction(Eigen::Vector3d lidar_odom)
{
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

    x_hat = x_hat + K * (lidar_odom - x_hat); 

    P = P - K * C * P;

    // numerically "more stable (Gemini)"
    // P = (Eigen::Matrix3d::Identity() - K * C) * P;
}


Eigen::Vector3d KalmanFilter::get_x_hat()
{
    return x_hat;
}
