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
    Q *= 0.4;

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

    R = updateR(scan_msg, unwrapped_rot);

    for (int i = 0; i < R.rows(); ++i) {
        for (int j = 0; j < R.cols(); ++j) {
            std::cout << std::setw(10) << R(i, j) << " ";
        }
        std::cout << std::endl;
    }

    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (lidar_odom - C * x_hat);        //podla zadania 2 z opt je to C * x_hat
    P = P - K * C * P;                      
    // P = (Eigen::Matrix3d::Identity() - K * C) * P;

    prev_rot = curr_rot;
}


Eigen::MatrixXd KalmanFilter::updateR(sensor_msgs::msg::LaserScan::SharedPtr msg, double theta) {
    std::vector<Point> points = getLidarPoints(msg, theta);
    size_t N = points.size();

    std::vector<double> mean_values = mean(points);
    
    double sigma_x = 0;
    double sigma_y = 0;
    double covar = 0;

    for (size_t i = 0; i < N; i++) {
        sigma_x += pow(points[i].x - mean_values[0], 2);
        sigma_y += pow(points[i].y - mean_values[1], 2);
        covar += (points[i].x - mean_values[0]) * (points[i].y - mean_values[1]);
    }

    sigma_x /= N;
    sigma_y /= N;
    covar /= N;

    double rx = sigma_y/sigma_x;
    double ry = sigma_x/sigma_y;
    static double rx_max = 1.0, ry_max = 1.0;

    if (rx > rx_max) {
        rx_max = rx;
    }

    if (ry > ry_max) {
        ry_max = ry;
    }

    cout << rx << endl;
    // cout << "1: " << x_var << endl;
    // cout << "2: " << y_var << endl;

    // Eigen::Matrix2d M;
    // M << sigma_x, covar,
    //      covar, sigma_y;
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(M);
    // auto evals = solver.eigenvalues();

    // cout << "1: " << evals(0) << endl;
    // cout << "2: " << evals(1) << endl;
    // cout << endl;
    
    Eigen::MatrixXd R_new(3, 3);
    R_new << pow(rx_max, 2)*10, covar, 0,
             covar, pow(ry_max, 2)*10, 0,
             0,     0,     1;


    
    return R_new;
}


std::vector<double> KalmanFilter::mean(const std::vector<Point>& points) {
    size_t N = points.size();
    double x_sum = 0;
    double y_sum = 0;
    for (size_t i = 0; i < N; i++) {
        x_sum += points[i].x;
        y_sum += points[i].y;
    }

    return {x_sum / N, y_sum / N};
}


std::vector<Point> KalmanFilter::getLidarPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double theta) {
    size_t N = scan_msg->ranges.size();
    std::vector<Point>points(N);
    double angle = scan_msg->angle_min;

    for (int i = 0; angle <= scan_msg->angle_max; i++) {
        float x = sin(angle)*scan_msg->ranges[i];
        float y = cos(angle)*scan_msg->ranges[i];
        angle += scan_msg->angle_increment;
        points[i] = Point{x,y};
    }
    return points;
}


Eigen::Vector3d KalmanFilter::get_x_hat()
{
    return x_hat;
}