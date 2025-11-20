#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_control_cpp/kalman_filter.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>


class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("odom_subscriber"), kf(Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1)
        );

        odom_icp_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_icp", 10,
            std::bind(&OdomSubscriber::odom_icp_callback, this, std::placeholders::_1)
        );

        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/speed", 10,
            std::bind(&OdomSubscriber::velocity_callback, this, std::placeholders::_1)
        );
        
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&OdomSubscriber::scan_callback, this, std::placeholders::_1)
        );
        
        fusion_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_fused", 10);
        
        RCLCPP_INFO(this->get_logger(), "Kalman filter initialized");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        wheel_odom_setup = true;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double yaw = tf2::getYaw(q);
        wheel_odom_vec << msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          yaw;

        rclcpp::Time current_time = msg->header.stamp;

        // FROM CHAT
        dt = 0.0;
        if (last_time.nanoseconds() != 0) {
            dt = (current_time - last_time).seconds(); // dt in seconds
        }

        last_time = current_time;

        // extract velocities from wheel odometry
        v = msg->twist.twist.linear.x;
        omega = msg->twist.twist.angular.z;

        // call EKF prediction
        // FROM CHAT

        double theta = wheel_odom_vec(2);
        if (isPredictionReady()) {
            prediction_finished = false;
            // kf.prediction(wheel_odom_vec, velocity_);
            //FROM CHAT
            kf.prediction(dt, v, omega, theta, wheel_odom_vec);
            //FROM CHAT
            prediction_finished = true;
        }
    }

    void odom_icp_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        lidar_odom_setup = true;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double yaw = tf2::getYaw(q);

        lidar_odom_vec << msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          yaw;


        if (isCorrectionReady()) {
            correction_finished = false;
            kf.correction(lidar_odom_vec, scan_msg_);
            fused_output = kf.get_x_hat();
            publish();
            correction_finished = true;
            scan_ready = false;
        }
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        velocity_setup = true;

        velocity_ = msg->data;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_setup = true;

        scan_msg_ = msg;
        scan_ready = true;
    }

    void publish()
    {
        auto fused_msg = nav_msgs::msg::Odometry();
 
        fused_msg.header.stamp = this->now();
        fused_msg.header.frame_id = "odom";     
        fused_msg.child_frame_id = "base_link";

        fused_msg.pose.pose.position.x = fused_output(0);
        fused_msg.pose.pose.position.y = fused_output(1);

        double yaw = fused_output(2);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();         



        fused_msg.pose.pose.orientation.x = q.x();
        fused_msg.pose.pose.orientation.y = q.y();
        fused_msg.pose.pose.orientation.z = q.z();
        fused_msg.pose.pose.orientation.w = q.w();

        // fused_msg.pose.pose.orientation.z = 

        fusion_publisher_->publish(fused_msg);
    }

    bool isPredictionReady() {
        return (wheel_odom_setup && lidar_odom_setup && velocity_setup && scan_setup && correction_finished);
    }

    bool isCorrectionReady() {
        return (wheel_odom_setup && lidar_odom_setup && velocity_setup && scan_setup && prediction_finished && scan_ready);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_icp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fusion_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    Eigen::Vector3d wheel_odom_vec;
    Eigen::Vector3d lidar_odom_vec;
    double velocity_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    
    Eigen::Vector3d fused_output;

    bool wheel_odom_setup = false;
    bool lidar_odom_setup = false;
    bool velocity_setup = false;
    bool scan_setup = false;

    bool scan_ready = false;
    
    bool correction_finished = true;
    bool prediction_finished = false;

    double dt;
    double v;
    double omega;
    
    KalmanFilter kf;

    rclcpp::Time last_time;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
