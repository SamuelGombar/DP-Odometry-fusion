#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>

class GtWrapper : public rclcpp::Node
{
public:
  GtWrapper() : Node("gt_wrapper"), first_(true),
    offset_x_(0.0), offset_y_(0.0), offset_yaw_(0.0)
  {
    this->declare_parameter("kobuki", false);
    bool kobuki = this->get_parameter("kobuki").as_bool();

    if (kobuki) {
      pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/optitrack", 10);

      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vrpn_mocap/RigidBody_002/pose", rclcpp::QoS(10).best_effort(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          double x   = -msg->pose.position.z;
          double y   = -msg->pose.position.x;
          double yaw = tf2::getYaw(msg->pose.orientation);

          if (first_) {
            offset_x_   = x;
            offset_y_   = y;
            offset_yaw_ = yaw;
            first_ = false;
          }

          double dx = x - offset_x_;
          double dy = y - offset_y_;
          double cx =  std::cos(offset_yaw_) * dx + std::sin(offset_yaw_) * dy;
          double cy = -std::sin(offset_yaw_) * dx + std::cos(offset_yaw_) * dy;
          double cyaw = yaw - offset_yaw_;
          cyaw = std::atan2(std::sin(cyaw), std::cos(cyaw));

          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, cyaw);

          nav_msgs::msg::Odometry out;
          out.header.stamp    = msg->header.stamp;
          out.header.frame_id = "odom";
          out.child_frame_id  = "base_link";
          out.pose.pose.position.x = cx;
          out.pose.pose.position.y = cy;
          out.pose.pose.position.z = 0.0;
          out.pose.pose.orientation = tf2::toMsg(q);
          pose_pub_->publish(out);
        });
    } else {
      pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ground_truth_wrapper", 10);

      sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          double x   = msg->pose.pose.position.x;
          double y   = msg->pose.pose.position.y;
          double yaw = tf2::getYaw(msg->pose.pose.orientation);

          if (first_) {
            offset_x_ = x;
            offset_y_ = y;
            offset_yaw_ = yaw;
            first_ = false;
          }

          double dx = x - offset_x_;
          double dy = y - offset_y_;
          double cx =  std::cos(offset_yaw_) * dx + std::sin(offset_yaw_) * dy;
          double cy = -std::sin(offset_yaw_) * dx + std::cos(offset_yaw_) * dy;
          double cyaw = yaw - offset_yaw_;
          cyaw = std::atan2(std::sin(cyaw), std::cos(cyaw));

          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, cyaw);

          auto out = *msg;
          out.header.stamp = this->now();
          out.header.frame_id = "odom";
          out.child_frame_id = "base_link";
          out.pose.pose.position.x = cx;
          out.pose.pose.position.y = cy;
          out.pose.pose.position.z = 0.0;
          out.pose.pose.orientation = tf2::toMsg(q);
          pub_->publish(out);
        });
    }
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

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
