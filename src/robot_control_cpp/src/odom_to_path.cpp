#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>

static constexpr double OPTITRACK_ROTATE_DEG = 2.5;

class PathNode : public rclcpp::Node {
public:
  PathNode()
  : Node("odometry_path")
  {
    odom_topic_ = this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/odometry/path");
    max_poses_ = this->declare_parameter<int>("max_poses", 30000);
    path_frame_ = this->declare_parameter<std::string>("path_frame", "odom");

    const double rad = OPTITRACK_ROTATE_DEG * M_PI / 180.0;
    rot_cos_ = (odom_topic_ == "/optitrack") ? std::cos(rad) : 1.0;
    rot_sin_ = (odom_topic_ == "/optitrack") ? std::sin(rad) : 0.0;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PathNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    path_msg_.header.frame_id = path_frame_;
    RCLCPP_INFO(this->get_logger(), "Odometry Path node initialized: listening to '%s', publishing '%s' in frame '%s'",
                odom_topic_.c_str(), path_topic_.c_str(), path_frame_.c_str());
  }

private:
  void appendPose(const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose_in) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose = pose_in;
    double x = pose_in.position.x;
    double y = pose_in.position.y;
    pose.pose.position.x = rot_cos_ * x - rot_sin_ * y;
    pose.pose.position.y = rot_sin_ * x + rot_cos_ * y;
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string path_frame_;
  int max_poses_;
  double rot_cos_, rot_sin_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}
