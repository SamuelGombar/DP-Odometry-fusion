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

    Q *= 0.1;

    // R = Eigen::MatrixXd::Identity();

    // K = Eigen::MatrixXd::Identity();

    // R *= 1000;
    // Q *= 0.01;
}


// void KalmanFilter::prediction(Eigen::Vector3d wheel_odom, double velocity)
void KalmanFilter::prediction(double dt, double v, double omega, double theta, Eigen::Vector3d wheel_odom_vec)
{   
    // MALO BY BYT DOBRE, ked tak mozes aj wheel_odom pouzit, ten isty model
    // x_hat(0) += v * std::cos(theta) * dt;
    // x_hat(1) += v * std::sin(theta) * dt;
    // x_hat(2) += omega * dt;

    // static double unwrapped_theta_w = wheel_odom_vec(2);
    // static double theta_r = wheel_odom_vec(2);
    // static double last_theta_r = wheel_odom_vec(2);
    // unwrapped_theta_w = wheel_odom_vec(2) + unwrapped_theta_w;
    // theta_r = wheel_odom_vec(2);

    // double diff = theta_r - last_theta_r;
    // if (diff > M_PI) {
    //     unwrapped_theta_w += 2*M_PI;
    // }

    // if (diff < M_PI) {
    //     unwrapped_theta_w -= 2*M_PI;
    // }
    static double unwrapped_rot = wheel_odom_vec(2);
    static double curr_rot = wheel_odom_vec(2);
    static double prev_rot = wheel_odom_vec(2);
    curr_rot = wheel_odom_vec(2);

    unwrapped_rot = unwrapYaw(prev_rot, curr_rot); //TU SI SKONCIL

    x_hat = wheel_odom_vec;

    A << 1, 0, -v * std::sin(theta) * dt,
         0, 1,  v *  std::cos(theta) * dt,
         0, 0, 1;

    P = A * P * A.transpose() + Q;

    // last_theta_r = wheel_odom_vec(2);
    prev_rot = curr_rot;
    std::cout << unwrapped_rot << "  ";
}


void KalmanFilter::correction(Eigen::Vector3d lidar_odom, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    int N = scan_msg->ranges.size();
    R.resize(2*N, 2*N);
    K.resize(3, 2*N);
    R.setIdentity();
    K.setZero();

    std::vector<Point> points = getLidarPoints(scan_msg);
    
    // C = calculateC(points, lidar_odom(2));
    Eigen::Matrix3d C_c, R_r;
    C_c.setIdentity();
    R_r.setIdentity();
    K = P * C_c.transpose() * (C_c * P * C_c.transpose() + R_r).inverse();
    x_hat += K * (lidar_odom - C_c * x_hat);  //je ta zatvorka dobre? hlavne C * x_hat... idk, ale musis polozit current scan ku x_hat
    // wrapThetaToPi();
    P = P - K * C_c * P;                      //podla zadania 2 z opt je to C * x_hat
    
    std::cout << lidar_odom(2) << "  " 
          << x_hat(2) << "  "  << std::endl;
 




    // numerically "more stable (Gemini)"
    // P = (Eigen::Matrix3d::Identity() - K * C) * P;
}

void KalmanFilter::wrapThetaToPi() {
    double theta = x_hat(2);

    while (theta > M_PI)
        theta -= 2.0 * M_PI;

    while (theta <= -M_PI)
        theta += 2.0 * M_PI;

    x_hat(2) = theta;
}


Eigen::VectorXd KalmanFilter::stack(std::vector<Point> points) 
{
    int N = points.size();
    Eigen::VectorXd stacked(2*N);

    for (int i = 0; i < N; ++i) {
        stacked(2*i) = points[i].x;
        stacked(2*i + 1) = points[i].y; 
    }

    return stacked;
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

    // std::cout << "C size: " << C.rows() << " x " << C.cols() << std::endl;
    return C;
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


double KalmanFilter::unwrapYaw(double prev_yaw, double curr_yaw) {
    double delta = curr_yaw - prev_yaw;
    if (delta > M_PI) {
        delta -= 2 * M_PI;
    } else if (delta < -M_PI) {
        delta += 2 * M_PI;
    }
    return prev_yaw + delta;
}