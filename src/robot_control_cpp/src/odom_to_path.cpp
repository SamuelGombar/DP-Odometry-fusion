#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class PathNode : public rclcpp::Node {
public:
  PathNode()
  : Node("odometry_path")
  {
    odom_topic_ = this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/odometry/path");
    max_poses_ = this->declare_parameter<int>("max_poses", 30000);
    path_frame_ = this->declare_parameter<std::string>("path_frame", "odom");
    optitrack_ = this->declare_parameter<bool>("optitrack", false);

    if (optitrack_) {
      auto qos = rclcpp::QoS(10).best_effort();
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        odom_topic_, qos,
        std::bind(&PathNode::poseCallback, this, std::placeholders::_1));
    } else {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&PathNode::odomCallback, this, std::placeholders::_1));
    }

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    path_msg_.header.frame_id = path_frame_;
    RCLCPP_INFO(this->get_logger(), "Odometry Path node initialized: listening to '%s' (%s), publishing '%s' in frame '%s'",
                odom_topic_.c_str(), optitrack_ ? "OptiTrack/PoseStamped" : "Odometry",
                path_topic_.c_str(), path_frame_.c_str());
  }

private:
  void appendPose(const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose_in) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose = pose_in;
    path_msg_.poses.push_back(pose);
    path_msg_.header.stamp = header.stamp;
    path_msg_.header.frame_id = path_frame_;

    if (max_poses_ > 0 && static_cast<int>(path_msg_.poses.size()) > max_poses_) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_pub_->publish(path_msg_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    appendPose(msg->header, msg->pose.pose);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // OptiTrack Y-up: horizontal movement is in Z, negated to match ROS convention
    double x   = -msg->pose.position.x;
    double y   = msg->pose.position.z;
    double yaw =  tf2::getYaw(msg->pose.orientation);

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

    // Additional 90° CCW rotation
    double rx = -cy;
    double ry = cx;

    double cyaw = (yaw - offset_yaw_) + (M_PI / 2.0);
    cyaw = std::atan2(std::sin(cyaw), std::cos(cyaw));

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, cyaw);

    geometry_msgs::msg::Pose pose;
    pose.position.x = rx;
    pose.position.y = ry;
    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(q);
    appendPose(msg->header, pose);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string path_frame_;
  int max_poses_;
  bool optitrack_;
  bool first_ = true;
  double offset_x_ = 0.0, offset_y_ = 0.0, offset_yaw_ = 0.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}
