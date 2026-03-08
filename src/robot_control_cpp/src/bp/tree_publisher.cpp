#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TreePublisher : public rclcpp::Node
{
public:
  TreePublisher() : Node("tree_publisher")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms
    publishStaticTransforms();
    RCLCPP_INFO(this->get_logger(), "TF tree published: map -> odom -> base_link -> base_laser");
  }

private:
  void publishStaticTransforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // map -> odom (identity)
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.x = 0.0;
    map_to_odom.transform.rotation.y = 0.0;
    map_to_odom.transform.rotation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;
    transforms.push_back(map_to_odom);

    // odom -> base_link (identity, assuming no initial offset)
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = this->now();
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";
    odom_to_base.transform.translation.x = 0.0;
    odom_to_base.transform.translation.y = 0.0;
    odom_to_base.transform.translation.z = 0.0;
    odom_to_base.transform.rotation.x = 0.0;
    odom_to_base.transform.rotation.y = 0.0;
    odom_to_base.transform.rotation.z = 0.0;
    odom_to_base.transform.rotation.w = 1.0;
    transforms.push_back(odom_to_base);

    // base_link -> base_laser (example: laser 0.2m forward, 0.1m up)
    geometry_msgs::msg::TransformStamped base_to_laser;
    base_to_laser.header.stamp = this->now();
    base_to_laser.header.frame_id = "base_link";
    base_to_laser.child_frame_id = "base_laser";
    base_to_laser.transform.translation.x = 0.0;
    base_to_laser.transform.translation.y = 0.0;
    base_to_laser.transform.translation.z = 0.15;
    base_to_laser.transform.rotation.x = 0.0;
    base_to_laser.transform.rotation.y = 0.0;
    base_to_laser.transform.rotation.z = 0.0;
    base_to_laser.transform.rotation.w = 1.0;
    transforms.push_back(base_to_laser);

    tf_static_broadcaster_->sendTransform(transforms);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TreePublisher>());
  rclcpp::shutdown();
  return 0;
}
