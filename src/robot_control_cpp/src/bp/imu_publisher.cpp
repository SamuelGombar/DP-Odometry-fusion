#include "rclcpp/rclcpp.hpp"
#include "hw_layer_msgs/msg/imu_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

class GyroPublisher : public rclcpp::Node {
public:
    GyroPublisher() : Node("gyro_publisher"), prev_angle_x_(0.0), prev_angle_y_(0.0), prev_angle_z_(0.0), prev_timestamp_(0.0) {
        // Create subscription to hw_layer IMU topic
        imu_subscription_ = this->create_subscription<hw_layer_msgs::msg::IMUMsg>(
            "/hw_layer/imu/sensor/data",
            rclcpp::SensorDataQoS(),
            std::bind(&GyroPublisher::imu_callback, this, std::placeholders::_1)
        );

        // Create publisher for standard ROS Imu message
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu",
            rclcpp::SensorDataQoS()
        );

        RCLCPP_INFO(this->get_logger(), "GyroPublisher node initialized");
    }

private:
    rclcpp::Subscription<hw_layer_msgs::msg::IMUMsg>::SharedPtr imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    double prev_angle_x_;
    double prev_angle_y_;
    double prev_angle_z_;
    double prev_timestamp_;

    void imu_callback(const hw_layer_msgs::msg::IMUMsg::SharedPtr msg) {
        // Create output Imu message
        sensor_msgs::msg::Imu imu_msg;

        // Set header
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        // Convert euler angles (XYZ) to quaternion
        // Using intrinsic XYZ (roll-pitch-yaw) convention
        double roll = msg->angle_x;
        double pitch = msg->angle_y;
        double yaw = msg->angle_z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        // Set orientation covariance (0.01 rad^2 diagonal, -1 means unknown)
        imu_msg.orientation_covariance[0] = 0.01;
        imu_msg.orientation_covariance[4] = 0.01;
        imu_msg.orientation_covariance[8] = 0.01;

        // Calculate angular velocity from angle differentiation
        double current_timestamp = msg->timestamp;
        double angular_velocity_x = 0.0;
        double angular_velocity_y = 0.0;
        double angular_velocity_z = 0.0;

        if (prev_timestamp_ > 0.0) {
            double dt = current_timestamp - prev_timestamp_;
            if (dt > 1e-6) {  // Avoid division by very small numbers
                angular_velocity_x = (msg->angle_x - prev_angle_x_) / dt;
                angular_velocity_y = (msg->angle_y - prev_angle_y_) / dt;
                angular_velocity_z = (msg->angle_z - prev_angle_z_) / dt;
            }
        }

        // Set angular velocity (calculated from angle differentiation)
        imu_msg.angular_velocity.x = angular_velocity_x;
        imu_msg.angular_velocity.y = angular_velocity_y;
        imu_msg.angular_velocity.z = angular_velocity_z;

        // Set angular velocity covariance
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;

        // Set linear acceleration
        imu_msg.linear_acceleration.x = msg->acceleration_x;
        imu_msg.linear_acceleration.y = msg->acceleration_y;
        imu_msg.linear_acceleration.z = msg->acceleration_z;

        // Set linear acceleration covariance
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;

        // Publish the message
        imu_publisher_->publish(imu_msg);

        // Update previous values for next iteration
        prev_angle_x_ = msg->angle_x;
        prev_angle_y_ = msg->angle_y;
        prev_angle_z_ = msg->angle_z;
        prev_timestamp_ = current_timestamp;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GyroPublisher>());
    rclcpp::shutdown();
    return 0;
}
