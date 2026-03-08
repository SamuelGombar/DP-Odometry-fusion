#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <amrapi_msgs/msg/vehicle_velocity_msg.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class WheelOdomPublisher : public rclcpp::Node
{
public:
  WheelOdomPublisher()
  : Node("wheel_odom_publisher"),
    x_(0.0), y_(0.0), theta_(0.0)
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    vel_sub_ = this->create_subscription<amrapi_msgs::msg::VehicleVelocityMsg>(
      "/amrapi/sensor/velocity", 10,
      std::bind(&WheelOdomPublisher::velocity_callback, this, std::placeholders::_1));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Wheel odometry publisher started");
  }

private:
  void velocity_callback(const amrapi_msgs::msg::VehicleVelocityMsg::SharedPtr msg)
  {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
      return;  // skip invalid or too-large time steps
    }

    double v = msg->linear;
    double w = msg->angular;

    // Integrate pose using midpoint method
    double delta_theta = w * dt;
    double mid_theta = theta_ + delta_theta / 2.0;
    x_ += v * std::cos(mid_theta) * dt;
    y_ += v * std::sin(mid_theta) * dt;
    theta_ += delta_theta;

    // Normalize theta to [-pi, pi]
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    // Create quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // Publish odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    odom_pub_->publish(odom);

    // Broadcast odom -> base_link transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    // tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<amrapi_msgs::msg::VehicleVelocityMsg>::SharedPtr vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_time_;
  double x_, y_, theta_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
