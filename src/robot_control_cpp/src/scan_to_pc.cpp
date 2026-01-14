#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

class LaserToPointCloudNode : public rclcpp::Node
{
public:
    LaserToPointCloudNode() : Node("laser_to_pointcloud_node")
    {
        // Publisher for PointCloud2
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_topic", rclcpp::QoS(10));

        // Subscriber to LaserScan
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10),
            std::bind(&LaserToPointCloudNode::laserCallback, this, std::placeholders::_1));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);
        cloud_msg.header = scan_msg->header;
        pointcloud_pub_->publish(cloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
