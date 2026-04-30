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
    x_(0.0), y_(0.0), theta_(0.0), first_odom_(true), offset_x_(0.0), offset_y_(0.0), offset_theta_(0.0),
    last_v_(0.0), last_w_(0.0), last_pub_time_(0, 0, RCL_ROS_TIME)
  {
    this->declare_parameter<bool>("is_kinematic", false);
    is_kinematic_ = this->get_parameter("is_kinematic").as_bool();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    vel_sub_ = this->create_subscription<amrapi_msgs::msg::VehicleVelocityMsg>(
      "/amrapi/sensor/velocity", 10,
      std::bind(&WheelOdomPublisher::velocity_callback, this, std::placeholders::_1));

    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    publishInitialOdom();

    if (is_kinematic_) {
      keepalive_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&WheelOdomPublisher::keepaliveCallback, this));
    }

    RCLCPP_INFO(this->get_logger(), "Wheel odometry publisher started");
  }

private:
  void velocity_callback(const amrapi_msgs::msg::VehicleVelocityMsg::SharedPtr msg)
  {
    auto current_time = this->now();
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      start_time_ = current_time;
      return;
    }
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if ((current_time - start_time_).seconds() < 10.0 && (dt <= 0.0 || dt > 1.0)) {
      return;
    }

    if ((current_time - start_time_).seconds() < 10.0) std::cout << "IM IN EFFECT" << std::endl;

    double v = msg->linear;
    double w = -msg->angular;

    double delta_theta = w * dt;
    double mid_theta = theta_ + delta_theta / 2.0;
    x_ += v * std::cos(mid_theta) * dt;
    y_ += v * std::sin(mid_theta) * dt;
    theta_ += delta_theta;

    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    if (first_odom_) {
      offset_x_ = x_;
      offset_y_ = y_;
      offset_theta_ = theta_;
      first_odom_ = false;
    }

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    double corrected_x = x_ - offset_x_;
    double corrected_y = y_ - offset_y_;
    double corrected_theta = theta_ - offset_theta_;
    corrected_theta = std::atan2(std::sin(corrected_theta), std::cos(corrected_theta));

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, corrected_theta);

    odom.pose.pose.position.x = corrected_x;
    odom.pose.pose.position.y = corrected_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    if(is_kinematic_) {
      last_v_ = v; 
      last_w_ = w;
      last_q_ = q;
      last_pub_time_ = current_time;
    }

    odom.twist.covariance[0] = 0.0000000001;  // variance of linear x
    odom.twist.covariance[7] = 0.0000000001;  // variance of linear y
    odom.twist.covariance[35] = 0.0000000001; // variance of angular z (yaw)

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

    if (is_kinematic_) {
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void keepaliveCallback()
  {
    auto now = this->now();
    if (now.nanoseconds() == 0) return;
    if ((now - last_pub_time_).seconds() < 0.04) return;

    // Re-stamp and republish last known pose so TF buffer stays fresh
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = last_q_.x();
    tf.transform.rotation.y = last_q_.y();
    tf.transform.rotation.z = last_q_.z();
    tf.transform.rotation.w = last_q_.w();
    tf_broadcaster_->sendTransform(tf);

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    double corrected_x = x_ - offset_x_;
    double corrected_y = y_ - offset_y_;
    double corrected_theta = theta_ - offset_theta_;
    corrected_theta = std::atan2(std::sin(corrected_theta), std::cos(corrected_theta));
    odom.pose.pose.position.x = corrected_x;
    odom.pose.pose.position.y = corrected_y;
    odom.pose.pose.orientation.x = last_q_.x();
    odom.pose.pose.orientation.y = last_q_.y();
    odom.pose.pose.orientation.z = last_q_.z();
    odom.pose.pose.orientation.w = last_q_.w();
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;
    odom_pub_->publish(odom);

    last_pub_time_ = now;
  }

  void publishInitialOdom()
  {
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;

    // Set covariances
    // odom.pose.covariance[0] = 0.000001;  // var x
    // odom.pose.covariance[7] = 0.000001;  // var y
    // odom.pose.covariance[35] = 0.000001; // var yaw
    odom.twist.covariance[0] = 0.000001;  // var linear x
    odom.twist.covariance[7] = 0.000001;  // var linear y
    odom.twist.covariance[35] = 0.000001; // var angular z

    odom_pub_->publish(odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<amrapi_msgs::msg::VehicleVelocityMsg>::SharedPtr vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr keepalive_timer_;
  rclcpp::Time last_time_;
  rclcpp::Time last_pub_time_;
  rclcpp::Time start_time_;
  double x_, y_, theta_;
  bool first_odom_;
  double offset_x_, offset_y_, offset_theta_;
  double last_v_, last_w_;
  tf2::Quaternion last_q_ = tf2::Quaternion(0, 0, 0, 1);
  bool is_kinematic_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
