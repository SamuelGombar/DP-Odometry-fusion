#include <vector>
#include <Eigen/Dense>

class KalmanFilter {
  public:
    
    //Model 
    Eigen::Vector3d x_hat;           //n = 3 stavov modelu systemu, m = 3 stavov modelu merania, r = 2 vstupy
    Eigen::Vector3d A;      //3x1
    Eigen::Matrix3d Q;      //3x3
    
    Eigen::Matrix3d P;      //3x3
    
    //Measurement
    Eigen::Vector3d C;      //3x1
    Eigen::Matrix3d R;      //3x3
    
    Eigen::Matrix3d K;      //3x3

    // Constructor
    KalmanFilter(double x_init, const Eigen::MatrixXd& P_init)
        : x_hat(x_init)
    {
        Q << 1, 0, 0
             0, 1, 0
             0, 0, 1;

        R << 1, 0, 0
             0, 1, 0
             0, 0, 1;

        P.setIdentity();

        A.setZero();
        C.setZero();
        K.setZero();
    }

  private:
    void prediction(Eigen::Vector3d wheel_odom) {
        x_hat = wheel_odom;
    }

    void A_jacobian(Eigen::Vector3d wheel_odom_delta) {
        A = wheel_odom_delta;
    }

    void apriori_update_cov() {
        P = A*P*A.transpose() + Q;
    }

    void C_jacobian(Eigen::Vector3d lidar_odom_delta) {     //asi zle, tu si skoncil
        C.setIdentity();
    }

    void calc_kalman_gain() {
        K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    }

    void correction(lidar_odom) {
        x_hat = x_hat + K*lidar_odom;
    }

    void posteriori_update_cov() {
        P = P - K*C*P;
    }
};





// #include <Eigen/Dense>
// #include <iostream>

// class KalmanFilter {
// public:
//     // State
//     Eigen::Vector3d x_hat;      // State estimate: [x, y, phi]
//     Eigen::Matrix3d P;          // Covariance

//     // Model
//     Eigen::Matrix3d A;          // State transition Jacobian
//     Eigen::Matrix3d Q;          // Process noise

//     // Measurement
//     Eigen::Matrix3d C;          // Measurement Jacobian
//     Eigen::Matrix3d R;          // Measurement noise
//     Eigen::Vector3d y;          // Measurement residual

//     // Kalman gain
//     Eigen::Matrix3d K;

//     // Inputs
//     Eigen::Vector3d wheel_odom;
//     Eigen::Vector3d lidar_odom;

//     KalmanFilter(const Eigen::Vector3d& x0, const Eigen::Matrix3d& P0)
//         : x_hat(x0), P(P0)
//     {
//         A.setIdentity();
//         Q.setZero();
//         C.setIdentity();
//         R.setZero();
//         y.setZero();
//         K.setZero();
//     }

//     // Predict step
//     void predict(const Eigen::Vector3d& wheel_delta) {
//         // Simple motion model: x_hat_k = x_hat_{k-1} + wheel_delta
//         x_hat += wheel_delta;

//         // Update state transition Jacobian if needed (here identity)
//         A.setIdentity();

//         // Covariance prediction
//         P = A * P * A.transpose() + Q;
//     }

//     // Update step
//     void update(const Eigen::Vector3d& lidar_measurement) {
//         // Measurement residual
//         y = lidar_measurement - x_hat;

//         // Measurement Jacobian (identity if direct measurement)
//         C.setIdentity();

//         // Kalman gain
//         K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

//         // State update
//         x_hat += K * y;

//         // Covariance update
//         P = (Eigen::Matrix3d::Identity() - K * C) * P;
//     }
// };

// int main() {
//     Eigen::Vector3d x0(0,0,0);
//     Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity();

//     KalmanFilter kf(x0, P0);

//     // Example step
//     Eigen::Vector3d wheel_delta(0.1, 0.05, 0.01);   // wheel odometry change
//     Eigen::Vector3d lidar_meas(0.12, 0.04, 0.02);    // lidar measurement

//     kf.predict(wheel_delta);
//     kf.update(lidar_meas);

//     std::cout << "Updated state:\n" << kf.x_hat << "\n";
//     std::cout << "Updated covariance:\n" << kf.P << "\n";

//     return 0;
// }