// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>

class KalmanFilter {
public:
    explicit KalmanFilter(const Eigen::Vector3d& x_init);

private:
    void prediction();
    void correction();
};

#endif // KALMANFILTER_H
