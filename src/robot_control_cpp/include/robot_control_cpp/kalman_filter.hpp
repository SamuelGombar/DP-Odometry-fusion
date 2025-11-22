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
    explicit KalmanFilter(const Eigen::Vector3d& x_init);

    void prediction(double dt, double v, Eigen::Vector3d wheel_odom_vec);

    void correction(Eigen::Vector3d lidar_odom, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    Eigen::MatrixXd updateR(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, Eigen::MatrixXd R);

    std::vector<Point> getLidarPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    Eigen::Vector3d get_x_hat();

private:
    Eigen::MatrixXd P;

    Eigen::MatrixXd A;

    Eigen::MatrixXd Q;

    Eigen::MatrixXd R;

    Eigen::MatrixXd K;

    Eigen::MatrixXd C;

    Eigen::Vector3d x_hat;
};

#endif
