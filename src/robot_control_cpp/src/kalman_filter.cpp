#include "robot_control_cpp/kalman_filter.hpp"
#include <cmath>
#include <iostream>

using namespace std;

KalmanFilter::KalmanFilter(const Eigen::Vector3d& x_init)
    : x_hat(x_init)
{
    A = Eigen::Matrix3d::Identity();
    P = Eigen::Matrix3d::Identity();
    // C: das resize() pre matrixXd
    Q = Eigen::Matrix3d::Identity();
}

// Private methods
void KalmanFilter::prediction(Eigen::Vector3d wheel_odom, double velocity)
{
    x_hat = wheel_odom;
    A << 1, 0, -velocity * (-std::sin(wheel_odom(2))),
         0, 1,  velocity * std::cos(wheel_odom(2)),
         0, 0, 1;
    P = A * P * A.transpose() + Q;
    printKFState();
}

void KalmanFilter::correction(const std::vector<std::pair<double,double>>& scan_pairs, float angle_increment)
{
    R.resize(2*scan_pairs.size(), 2*scan_pairs.size());
    R.setIdentity();
    R *= 0.000000001;
    compute_C(scan_pairs, angle_increment);
    
    // cout << "P: " << P.rows() << "x" << P.cols() << endl;
    // cout << "C: " << C.rows() << "x" << C.cols() << endl;
    // cout << "R: " << R.rows() << "x" << R.cols() << endl;
    // cout << "scan_pairs_vector: " << scan_pairs_vector.size() << endl;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

    // cout << K << endl;

    flatten(scan_pairs);
    x_hat = x_hat + K * scan_pairs_vector;
    P = P - K * C * P;
}

void KalmanFilter::compute_C(const std::vector<std::pair<double,double>>& scan_pairs, float angle_increment)
{
    const size_t N = scan_pairs.size();
    C.resize(2*N, 3);

    float angle = 0.0;
    for (size_t i = 0; i < N; ++i) {
        double px = scan_pairs[i].first;
        double py = scan_pairs[i].second;

        double s = std::sin(angle);
        double c = std::cos(angle);

        Eigen::Matrix<double, 2, 3> Ci;

        Ci(0,0) = -1;  Ci(0,1) =  0;  Ci(0,2) =  px * s + py * c;
        Ci(1,0) =  0;  Ci(1,1) = -1;  Ci(1,2) = -px * c + py * s;

        C.block<2,3>(2 * i, 0) = Ci;

        angle += angle_increment;
    }
}

void KalmanFilter::flatten(const std::vector<std::pair<double,double>>& scan_pairs) 
{
    const size_t N = scan_pairs.size();
    scan_pairs_vector.resize(2 * N);

    for (size_t i = 0; i < N; ++i) {
        scan_pairs_vector(2 * i)     = scan_pairs[i].first;
        scan_pairs_vector(2 * i + 1) = scan_pairs[i].second;
    }
}

Eigen::Vector3d KalmanFilter::get_x_hat()
{
    return x_hat;
}

void KalmanFilter::printKFState()
{
    cout << "===== Kalman Filter State =====" << endl;
    cout << "x_hat =\n" << x_hat << endl;
    cout << "A =\n" << A << endl;
    cout << "P =\n" << P << endl;
    cout << "==============================" << endl;
}
