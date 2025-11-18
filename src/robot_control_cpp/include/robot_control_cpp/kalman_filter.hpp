// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>

class KalmanFilter {
public:
    explicit KalmanFilter(const Eigen::Vector3d& x_init);

    void prediction(Eigen::Vector3d wheel_odom, double velocity);

    void correction(Eigen::Vector3d lidar_odom);

    Eigen::Vector3d get_x_hat();

private:
    Eigen::Matrix3d Q;

    Eigen::Matrix3d R;

    Eigen::Vector3d x_hat;

    Eigen::Matrix3d A;
    
    Eigen::Matrix3d P;

    Eigen::Matrix3d C;

    Eigen::MatrixXd K;
};

#endif // KALMANFILTER_H
