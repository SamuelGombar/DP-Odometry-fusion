// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>

class KalmanFilter {
public:
    explicit KalmanFilter(const Eigen::Vector3d& x_init);
    void prediction(Eigen::Vector3d wheel_odom, double velocity);
    void correction(const std::vector<std::pair<double,double>>& scan_pairs, float angle_increment);
    void compute_C(const std::vector<std::pair<double,double>>& scan_pairs, float angle_increment);
    void flatten(const std::vector<std::pair<double,double>>& scan_pairs);
    Eigen::Vector3d get_x_hat();

private:
    // Process noise
    Eigen::Matrix3d Q;
    // Measurement noise
    Eigen::MatrixXd R;       // dynamic based on scan size


    // State vector
    Eigen::Vector3d x_hat;   // [x, y, theta]

    Eigen::Matrix3d A;
    
    // Covariance
    Eigen::Matrix3d P;

    // Measurement matrix
    Eigen::MatrixXd C;

    // Kalman gain
    Eigen::MatrixXd K;

    Eigen::VectorXd scan_pairs_vector;

};

#endif // KALMANFILTER_H
