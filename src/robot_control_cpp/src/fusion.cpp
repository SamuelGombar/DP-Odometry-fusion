#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_control_cpp/kalman_filter.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>


class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("odom_subscriber"), kf(Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        // Subscriber to /odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1)
        );

        // Subscriber to /odom_icp
        odom_icp_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_icp", 10,
            std::bind(&OdomSubscriber::odom_icp_callback, this, std::placeholders::_1)
        );

        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/velocity", 10,
            std::bind(&OdomSubscriber::velocity_callback, this, std::placeholders::_1)
        );
        
        fusion_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_fused", 10);

        double x_0 = wheel_odom_->pose.pose.position.x;
        double y_0 = wheel_odom_->pose.pose.position.y;
        double theta_0 = wheel_odom_->pose.pose.orientation.z;
        Eigen::Vector3d x0(x_0, y_0, theta_0);
        KalmanFilter kf(x0);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received /odom: position=(%.2f, %.2f, %.2f)",
        //             msg->pose.pose.position.x,
        //             msg->pose.pose.position.y,
        //             msg->pose.pose.position.z);
        wheel_odom_ = msg;
        kf.prediction()
    }

    void odom_icp_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received /odom_icp: position=(%.2f, %.2f, %.2f)",
        //             msg->pose.pose.position.x,
        //             msg->pose.pose.position.y,
        //             msg->pose.pose.position.z);
        lidar_odom_ = msg;
        kf.correction()
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        velocity_ = msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_icp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fusion_publisher_;

    nav_msgs::msg::Odometry::SharedPtr wheel_odom_;
    nav_msgs::msg::Odometry::SharedPtr lidar_odom_;
    std_msgs::msg::Float64::SharedPtr velocity_;

    KalmanFilter kf;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
