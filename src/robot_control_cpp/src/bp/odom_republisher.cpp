#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomRepublisher : public rclcpp::Node
{
public:
  OdomRepublisher() : Node("odom_republisher")
  {
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_c", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto out = *msg;
        out.header.frame_id = "odom";
        // out.child_frame_id = "base_link";
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRepublisher>());
  rclcpp::shutdown();
  return 0;
}
