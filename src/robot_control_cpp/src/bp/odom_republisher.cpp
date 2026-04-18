#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomRepublisher : public rclcpp::Node
{
public:
  OdomRepublisher() : Node("odom_republisher"), first_(true), offset_x_(0.0), offset_y_(0.0), offset_theta_(0.0)
  {
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_c", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_c_path", 10);
    path_msg_.header.frame_id = "odom";
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/amrapi/sensor/pose", 10,
      std::bind(&OdomRepublisher::odomCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = this->now();
    out.header.frame_id = "odom";
    out.child_frame_id = "base_link";

    double x = msg->position.y;
    double y = msg->position.x;
    double yaw = tf2::getYaw(msg->orientation);
    out.pose.pose = *msg;

    if (first_) {
      offset_x_ = x;
      offset_y_ = y;
      offset_theta_ = yaw;
      first_ = false;
    }

    double corrected_x = x - offset_x_;
    double corrected_y = y - offset_y_;
    double corrected_theta = yaw - offset_theta_;
    corrected_theta = std::atan2(std::sin(corrected_theta), std::cos(corrected_theta));

    out.pose.pose.position.x = corrected_x;
    out.pose.pose.position.y = corrected_y;
    tf2::Quaternion corrected_q;
    corrected_q.setRPY(0.0, 0.0, corrected_theta);
    out.pose.pose.orientation = tf2::toMsg(corrected_q);

    pub_->publish(out);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose = out.pose.pose;
    path_msg_.poses.push_back(ps);
    path_msg_.header = out.header;
    path_pub_->publish(path_msg_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = out.header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = corrected_x;
    tf.transform.translation.y = corrected_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = out.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
  bool first_;
  double offset_x_, offset_y_, offset_theta_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRepublisher>());
  rclcpp::shutdown();
  return 0;
}
