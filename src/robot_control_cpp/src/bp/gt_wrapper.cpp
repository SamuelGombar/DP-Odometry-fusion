#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class GtWrapper : public rclcpp::Node
{
public:
  GtWrapper() : Node("gt_wrapper"), first_(true),
    offset_x_(0.0), offset_y_(0.0), offset_yaw_(0.0)
  {
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ground_truth_wrapper", 10);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x   = msg->pose.pose.position.x;
        double y   = msg->pose.pose.position.y;
        double yaw = tf2::getYaw(msg->pose.pose.orientation);

        if (first_) {
          offset_x_   = x;// + 0.21;
          offset_y_   = y;
          offset_yaw_ = yaw;
          // offset_yaw_ = 3.118;
          first_ = false;
        }

        double dx  = x - offset_x_;
        double dy  = y - offset_y_;
        double cx   =  std::cos(offset_yaw_) * dx + std::sin(offset_yaw_) * dy;
        double cy   = -std::sin(offset_yaw_) * dx + std::cos(offset_yaw_) * dy;
        double cyaw = yaw - offset_yaw_;
        cyaw = std::atan2(std::sin(cyaw), std::cos(cyaw));

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, cyaw);

        auto out = *msg;
        out.header.stamp            = this->now();
        out.header.frame_id         = "odom";
        out.child_frame_id          = "base_link";
        out.pose.pose.position.x    = cx;
        out.pose.pose.position.y    = cy;
        out.pose.pose.position.z    = 0.0;
        out.pose.pose.orientation   = tf2::toMsg(q);
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  bool   first_;
  double offset_x_, offset_y_, offset_yaw_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtWrapper>());
  rclcpp::shutdown();
  return 0;
}
