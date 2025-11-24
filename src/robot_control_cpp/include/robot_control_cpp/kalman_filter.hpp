// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"


struct Point {
    float x;
    float y;
};

class KalmanFilter {
public:
    explicit KalmanFilter(const Eigen::Vector2d& x_init);

    void prediction(double dt, Eigen::Vector2d wheel_speed_vec);

    void correction(Eigen::Vector2d lidar_speed_vec, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    Eigen::MatrixXd updateR(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double theta);

    std::vector<Point> getLidarPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double theta);

    Eigen::VectorXd get_message();

    std::vector<double> mean(const std::vector<Point>& points);

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
