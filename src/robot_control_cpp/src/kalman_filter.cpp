#include "robot_control_cpp/kalman_filter.hpp"
#include <cmath>
#include <iostream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

using namespace std;

KalmanFilter::KalmanFilter(const Eigen::Vector3d& x_init)
    : x_hat(x_init)
{
    P = Eigen::Matrix3d::Identity();

    A = Eigen::Matrix3d::Identity();

    Q = Eigen::Matrix3d::Identity();

    // R = Eigen::MatrixXd::Identity();

    // K = Eigen::MatrixXd::Identity();

    // R *= 1000;
    // Q *= 0.01;
}


// void KalmanFilter::prediction(Eigen::Vector3d wheel_odom, double velocity)
void KalmanFilter::prediction(double dt, double v, double omega, double theta)
{   
    // MALO BY BYT DOBRE, ked tak mozes aj wheel_odom pouzit, ten isty model
    x_hat(0) += v * std::cos(theta) * dt;   //no pozor... mozno zle, nieco z lidar odom to bude
    x_hat(1) += v * std::sin(theta) * dt;
    x_hat(2) += omega * dt;

    A << 1, 0, -v * std::sin(theta) * dt,
         0, 1,  v *  std::cos(theta) * dt,
         0, 0, 1;

    P = A * P * A.transpose() + Q;
}


void KalmanFilter::correction(Eigen::Vector3d lidar_odom, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    int N = scan_msg->ranges.size();
    R.resize(2*N, 2*N);
    K.resize(3, 2*N);
    R.setIdentity();
    K.setZero();

    std::vector<Point> points = getLidarPoints(scan_msg);
    
    C = calculateC(points, lidar_odom(2));
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (lidar_odom - C * x_hat);  //je ta zatvorka dobre? hlavne C * x_hat... idk, ale musis polozit current scan ku x_hat
    P = P - K * C * P;

    // numerically "more stable (Gemini)"
    // P = (Eigen::Matrix3d::Identity() - K * C) * P;
}


Eigen::Vector3d KalmanFilter::get_x_hat()
{
    return x_hat;
}

Eigen::MatrixXd KalmanFilter::calculateC(std::vector<Point> points, double theta) {
    int N = points.size();
    Eigen::MatrixXd C(2*N, 3);
    for (int i = 0; i < N; ++i) {
        Eigen::Matrix<double, 2, 3> C_i;
        C_i << 1, 0, -points[i].x * sin(theta) - points[i].y * cos(theta),
               0, 1,  points[i].x * cos(theta) - points[i].y * sin(theta);
        C.block<2,3>(2*i, 0) = C_i;
    }

    return C;
}

std::vector<Point> KalmanFilter::getLidarPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    int N = scan_msg->ranges.size();
    points.resize(N);
    double angle = 0.0;
    for (int i = 0; i < N; i++) {
        float x = cos(angle)*scan_msg->ranges[i];
        float y = sin(angle)*scan_msg->ranges[i];
        angle += scan_msg->angle_increment;
        points[i] = Point{x,y};
    }
    return points;
}
