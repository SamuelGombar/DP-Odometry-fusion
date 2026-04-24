#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfPublisher : public rclcpp::Node
{
public:
  TfPublisher() : Node("tf_publisher")
  {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Static: base_link -> base_laser (z = 0.15, no rotation)
    geometry_msgs::msg::TransformStamped base_link_to_laser;
    base_link_to_laser.header.stamp = rclcpp::Time(0);
    base_link_to_laser.header.frame_id = "base_link";
    base_link_to_laser.child_frame_id = "base_laser";
    base_link_to_laser.transform.translation.z = 0.15;
    base_link_to_laser.transform.rotation.w = 1.0;

    // Static: map -> odom (identity) — never changes during bag playback
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = rclcpp::Time(0);
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform({base_link_to_laser, map_to_odom});
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfPublisher>());
  rclcpp::shutdown();
  return 0;
}

