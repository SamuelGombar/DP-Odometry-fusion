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
    Q *= 0.4;

    R = Eigen::MatrixXd::Identity(2, 2);
    // R *= 1000;

    K = Eigen::MatrixXd::Identity(2, 2);

    C = Eigen::MatrixXd::Identity(2, 2);

    fused_odom = Eigen::Vector3d::Zero();
}


void KalmanFilter::prediction(double dt, Eigen::Vector2d wheel_speed_vec)
{   
    x_hat = wheel_speed_vec;

    // A << r/2, r/2;   //pri realnom asi bude treba - r: polomer kolesa, L: dlzka spojnice kolies
    //      r/L, -r/L;

    P = A * P * A.transpose() + Q;
}


void KalmanFilter::correction(Eigen::Vector2d lidar_speed_vec, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    R = updateR(scan_msg, fused_odom(2));

    // for (int i = 0; i < R.rows(); ++i) {
    //     for (int j = 0; j < R.cols(); ++j) {
    //         std::cout << std::setw(10) << R(i, j) << " ";
    //     }
    //     std::cout << std::endl;
    // }

    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (lidar_speed_vec - C * x_hat);        //podla zadania 2 z opt je to C * x_hat
    P = P - K * C * P;           
    fused_odom(0) += x_hat(0)*cos(fused_odom(2));
    fused_odom(1) += x_hat(0)*sin(fused_odom(2));
    fused_odom(2) += x_hat(1);
    // P = (Eigen::Matrix3d::Identity() - K * C) * P;
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

    // cout << rx << endl;
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
    
    Eigen::MatrixXd R_new(2, 2);
    R_new << pow(rx_max, 2)*10, covar,
             covar, pow(ry_max, 2)*10;
    
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


Eigen::VectorXd KalmanFilter::get_message()
{
    Eigen::VectorXd vector(fused_odom.size() + x_hat.size());
    vector.head(fused_odom.size()) = fused_odom;
    vector.tail(x_hat.size()) = x_hat;

    return vector;
}