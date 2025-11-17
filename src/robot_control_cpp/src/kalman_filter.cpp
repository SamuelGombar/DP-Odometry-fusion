#include "robot_control_cpp/kalman_filter.hpp"
#include <cmath>

using namespace std;

KalmanFilter::KalmanFilter(const Eigen::Vector3d& x_init)
    : x_hat(x_init)
{
    
}

// Private methods
void KalmanFilter::prediction()
{

}

void KalmanFilter::correction()
{

}
