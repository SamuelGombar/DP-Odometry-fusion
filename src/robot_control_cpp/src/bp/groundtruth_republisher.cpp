#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>


class GroundtruthRepublisher : public rclcpp::Node
{
public:
  GroundtruthRepublisher()
  : Node("groundtruth_republisher")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ground_truth", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&GroundtruthRepublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "groundtruth_republisher started. Publishing map->base_footprint on /ground_truth");
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        "map", "base_footprint",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Could not get map->base_footprint transform: %s", ex.what());
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = tf_stamped.header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id  = "base_footprint";

    odom.pose.pose.position.x = tf_stamped.transform.translation.x;
    odom.pose.pose.position.y = tf_stamped.transform.translation.y;
    odom.pose.pose.position.z = tf_stamped.transform.translation.z;

    odom.pose.pose.orientation = tf_stamped.transform.rotation;

    pub_->publish(odom);
  }

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundtruthRepublisher>());
  rclcpp::shutdown();
  return 0;
}
