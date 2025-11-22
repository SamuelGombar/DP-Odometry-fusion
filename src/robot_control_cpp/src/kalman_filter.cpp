#include "robot_control_cpp/kalman_filter.hpp"
#include <cmath>
#include <iostream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

using namespace std;

KalmanFilter::KalmanFilter(const Eigen::Vector3d& x_init)
    : x_hat(x_init)
{
    P = Eigen::MatrixXd::Identity(3, 3);

    A = Eigen::MatrixXd::Identity(3, 3);

    Q = Eigen::MatrixXd::Identity(3, 3);
    Q *= 0.1;

    R = Eigen::MatrixXd::Identity(3, 3);
    // R *= 1000;

    K = Eigen::MatrixXd::Identity(3, 3);

    C = Eigen::MatrixXd::Identity(3, 3);
}


void KalmanFilter::prediction(double dt, double v, Eigen::Vector3d wheel_odom_vec)
{   
    x_hat = wheel_odom_vec;

    static double unwrapped_rot = wheel_odom_vec(2);
    static double curr_rot = wheel_odom_vec(2);
    static double prev_rot = wheel_odom_vec(2);
    static double pis = 0.0;
    static double diff = 0.0;
    curr_rot = wheel_odom_vec(2);
    diff = curr_rot - prev_rot;
    if (diff > M_PI) pis -= 2 * M_PI;
    if (diff < -M_PI) pis += 2 * M_PI;
    unwrapped_rot = curr_rot + pis;
    x_hat(2) = unwrapped_rot;


    A << 1, 0, -v * std::sin(curr_rot) * dt,
         0, 1,  v *  std::cos(curr_rot) * dt,
         0, 0, 1;

    P = A * P * A.transpose() + Q;

    prev_rot = curr_rot;
}


void KalmanFilter::correction(Eigen::Vector3d lidar_odom, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    static double unwrapped_rot = lidar_odom(2);
    static double curr_rot = lidar_odom(2);
    static double prev_rot = lidar_odom(2);
    static double pis = 0.0;
    static double diff = 0.0;
    curr_rot = lidar_odom(2);
    diff = curr_rot - prev_rot;
    if (diff > M_PI) pis -= 2 * M_PI;
    if (diff < -M_PI) pis += 2 * M_PI;
    unwrapped_rot = curr_rot + pis;
    lidar_odom(2) = unwrapped_rot;

    // R = updateR(scan_msg, R);

    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (lidar_odom - C * x_hat);        //podla zadania 2 z opt je to C * x_hat
    P = P - K * C * P;                      
    // P = (Eigen::Matrix3d::Identity() - K * C) * P;

    prev_rot = curr_rot;
}


Eigen::MatrixXd KalmanFilter::updateR(sensor_msgs::msg::LaserScan::SharedPtr msg, Eigen::MatrixXd R) {
    std::vector<Point> points = getLidarPoints(msg);
    
    // R << 

    // return R;
}


std::vector<Point> KalmanFilter::getLidarPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    int N = scan_msg->ranges.size();
    std::vector<Point>points(N);
    double angle = 0.0;
    for (int i = 0; i < N; i++) {
        float x = cos(angle)*scan_msg->ranges[i];
        float y = sin(angle)*scan_msg->ranges[i];
        angle += scan_msg->angle_increment;
        points[i] = Point{x,y};
    }
    return points;
}


Eigen::Vector3d KalmanFilter::get_x_hat()
{
    return x_hat;
}