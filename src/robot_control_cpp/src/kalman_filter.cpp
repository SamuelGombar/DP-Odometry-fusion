#include <vector>
#include <Eigen/Dense>

class KalmanFilter {
  public:
    double x_hat;           //n = 3 stavov modelu systemu, m = 3 stavov modelu merania, r = 2 vstupy
    Eigen::Vector3d A;      //3x1
    Eigen::Matrix3d P;      //3x3
    Eigen::Vector3d y;      //3x1
    Eigen::Vector3d y_prev; //3x1
    Eigen::Vector3d C;      //3x1
    Eigen::Matrix3d K;      //3x3
    Eigen::Matrix3d Q;      //3x3
    Eigen::Matrix3d R;

    Eigen::Vector3d wheel_odom;
    Eigen::Vector3d lidar_odom;

    Eigen::Vector3d wheel_odom_diff;
    Eigen::Vector3d lidar_odom_diff;

    // Constructor
    KalmanFilter(double x_init, const Eigen::MatrixXd& P_init)
        : x_hat(x_init), P(P_init)
    {
        // Optionally initialize
        A.setZero();
        C.setZero();
        y.setZero();
    }

  private:
    void state_prediction() {
        x_hat = wheel_odom_output;
    }

    void A_jacobian() {
        A = wheel_odom_diff;
    }

    void apriori_update_cov() {
        P = A*P*A.transpose() + Q;
    }

    void C_jacobian() {
        C = lidar_odom_diff;
    }

    void calc_kalman_gain() {
        K = P*C.transpose()*(C*P*C.transpose() + R);
    }

    void state_correction() {
        x_hat = x_hat + K*(y - y_prev)
    }

    void posteriori_update_cov() {
        P = P - K*C*P
    }
};
