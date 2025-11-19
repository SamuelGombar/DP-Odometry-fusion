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

    // void prediction(Eigen::Vector3d wheel_odom, double velocity);

    void prediction(double dt, double v, double omega, double theta);

    void correction(Eigen::Vector3d lidar_odom, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    Eigen::MatrixXd calculateC(std::vector<Point> points, double theta);

    std::vector<Point> getLidarPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    Eigen::Vector3d get_x_hat();

private:
    Eigen::Matrix3d P;

    Eigen::Matrix3d A;

    Eigen::Matrix3d Q;

    Eigen::MatrixXd R;

    Eigen::MatrixXd K;

    Eigen::MatrixXd C;

    Eigen::Vector3d x_hat;

    std::vector<Point> points;
};

#endif // KALMANFILTER_H
