#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iomanip>
#include <iostream>

class ScanPrinter : public rclcpp::Node
{
public:
    ScanPrinter() : Node("scan_printer")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ScanPrinter::scanCallback, this, std::placeholders::_1));
    }

private:
    int flag = 0;
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (flag == 0) {
            std::cout << std::fixed << std::setprecision(2);
            for (const auto &r : msg->ranges)
                std::cout << r << '\n';
            std::cout << std::flush;
            flag = 1;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanPrinter>());
    rclcpp::shutdown();
    return 0;
}
