#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>


class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("odom_subscriber")
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
        
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /odom: position=(%.2f, %.2f, %.2f)",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
        wheel_odom_ = msg;
    }

    void odom_icp_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /odom_icp: position=(%.2f, %.2f, %.2f)",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
        lidar_odom_ = msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_icp_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
