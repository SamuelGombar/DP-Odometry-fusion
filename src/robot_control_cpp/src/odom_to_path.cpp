#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathNode : public rclcpp::Node {
public:
  PathNode()
  : Node("odometry_path")
  {
    // topic names can be adjusted via parameters if needed
    odom_topic_ = this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/odometry/path");
    max_poses_ = this->declare_parameter<int>("max_poses", 30000);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PathNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    path_msg_.header.frame_id = "odom";  // will be overridden by incoming message if desired
    RCLCPP_INFO(this->get_logger(), "Odometry Path node initialized: listening to '%s' and publishing '%s'",
                odom_topic_.c_str(), path_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path_msg_.poses.push_back(pose);
    path_msg_.header = msg->header;

    if (max_poses_ > 0 && static_cast<int>(path_msg_.poses.size()) > max_poses_) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_pub_->publish(path_msg_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  std::string odom_topic_;
  std::string path_topic_;
  int max_poses_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}
