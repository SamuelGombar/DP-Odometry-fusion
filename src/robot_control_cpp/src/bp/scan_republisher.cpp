#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ScanRepublisher : public rclcpp::Node
{
public:
  ScanRepublisher() : Node("scan_republisher")
  {
    this->declare_parameter<bool>("kobuki", false);
    kobuki_ = this->get_parameter("kobuki").as_bool();
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_merged_c", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transform: base_link -> base_laser (z = 0.15)
    geometry_msgs::msg::TransformStamped static_tf;
    static_tf.header.stamp = this->now();
    static_tf.header.frame_id = "base_link";
    static_tf.child_frame_id = "base_laser";
    static_tf.transform.translation.x = 0.0;
    static_tf.transform.translation.y = 0.0;
    static_tf.transform.translation.z = 0.15;
    static_tf.transform.rotation.x = 0.0;
    static_tf.transform.rotation.y = 0.0;
    static_tf.transform.rotation.z = 0.0;
    static_tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(static_tf);

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_merged", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time stamp = kobuki_ ? rclcpp::Time(msg->header.stamp) : this->now() ;

        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = stamp;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(map_to_odom);

        auto out = *msg;
        out.header.frame_id = "base_laser";
        out.header.stamp = stamp;
        // obmedzenie lucov
        // for (auto & r : out.ranges)
        //   if (r > 4.0f) r = std::numeric_limits<float>::infinity();
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  bool kobuki_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRepublisher>());
  rclcpp::shutdown();
  return 0;
}
