#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_control_cpp/kalman_filter.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("odom_subscriber"), kf(Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        // Subscriber to /odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1)
        );

        // Subscriber to /odom_icp
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
    }

private:
    void scan_to_coords(std::vector<float> scan)
    {
        scan_pairs.clear();
        scan_pairs.reserve(scan.size());

        float angle = 0.0;

        for (float d : scan)
        {
            double x = d * std::cos(angle);
            double y = d * std::sin(angle);

            scan_pairs.emplace_back(x, y);

            angle += angle_increment_;   // must store increment somewhere
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received /odom: position=(%.2f, %.2f, %.2f)",
        //             msg->pose.pose.position.x,
        //             msg->pose.pose.position.y,
        //             msg->pose.pose.position.z);
        // wheel_odom_ = msg;
        wheel_odom_ready = true;
        wheel_odom_vec << msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.orientation.z;  // yaw only


        if (wheel_odom_ready && velocity_ready && correction_finished) {
            kf.prediction(wheel_odom_vec, velocity_);
            prediction_ready = true;
        }
    }

    void odom_icp_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received /odom_icp: position=(%.2f, %.2f, %.2f)",
        //             msg->pose.pose.position.x,
        //             msg->pose.pose.position.y,
        //             msg->pose.pose.position.z);
        lidar_odom_ready = true;
        
        if (prediction_ready && scan_ready) {
            scan_copy = scan_;
            angle = angle_increment_;
            lidar_odom_ = msg;
            scan_to_coords(scan_copy);
            flatten_scan_pairs();
            scan_into_wheel_pose();
            correction_finished = false;
            kf.correction(scan_pairs, angle);
            fused_output = kf.get_x_hat();
            publish();
            correction_finished = true;
            prediction_ready = false;
            scan_ready = false;
        }
    }

    void scan_into_wheel_pose()
    {
        double x0 = wheel_odom_vec(0);
        double y0 = wheel_odom_vec(1);
        double theta0 = wheel_odom_vec(2);

        // Reshape scan_xy_vector into Nx2 matrix
        int n_points = scan_pairs_vector.size() / 2;
        if (n_points == 0) {
            printf("MAS 0\n");
        }
        Eigen::MatrixXd scan_points(n_points, 2);
        for (int i = 0; i < n_points; ++i)
        {
            scan_points(i, 0) = scan_pairs_vector(2*i);
            scan_points(i, 1) = scan_pairs_vector(2*i + 1);
        }

        // Rotation matrix
        Eigen::Matrix2d Rot;
        Rot << std::cos(theta0), -std::sin(theta0),
            std::sin(theta0),  std::cos(theta0);

        // Transform points
        Eigen::MatrixXd scan_points_transformed = (Rot * scan_points.transpose()).transpose();
        scan_points_transformed.rowwise() += Eigen::RowVector2d(x0, y0);

        // Flatten back to scan_xy_vector
        for (int i = 0; i < n_points; ++i)
        {
            scan_pairs_vector(2*i)     = scan_points_transformed(i, 0);
            scan_pairs_vector(2*i + 1) = scan_points_transformed(i, 1);
        }
    }
    
    void flatten_scan_pairs()
    {
        scan_pairs_vector.resize(scan_pairs.size() * 2);

        for (size_t i = 0; i < scan_pairs.size(); ++i)
        {
            scan_pairs_vector(2 * i)     = scan_pairs[i].first;
            scan_pairs_vector(2 * i + 1) = scan_pairs[i].second;
        }
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        velocity_ready = true;

        velocity_ = msg->data;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", msg->ranges.size());
        // Example: access first range
        scan_ = msg->ranges;
        angle_increment_ = msg->angle_increment;
        scan_ready = true;
    }

    void publish()
    {
        auto fused_msg = nav_msgs::msg::Odometry();
 
        fused_msg.header.stamp = this->now();        // current ROS2 time
        fused_msg.header.frame_id = "odom";          // parent frame
        fused_msg.child_frame_id = "base_link";      // child frame

        fused_msg.pose.pose.position.x = wheel_odom_vec(0);
        fused_msg.pose.pose.position.y = wheel_odom_vec(1);

        double yaw = wheel_odom_vec(2);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);  // roll = 0, pitch = 0, yaw = theta
        q.normalize();         // ensures quaternion is valid

        fused_msg.pose.pose.orientation.x = q.x();
        fused_msg.pose.pose.orientation.y = q.y();
        fused_msg.pose.pose.orientation.z = q.z();
        fused_msg.pose.pose.orientation.w = q.w();

        fusion_publisher_->publish(fused_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_icp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fusion_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    nav_msgs::msg::Odometry::SharedPtr wheel_odom_;
    nav_msgs::msg::Odometry::SharedPtr lidar_odom_;
    double velocity_;
    Eigen::Vector3d wheel_odom_vec;
    Eigen::VectorXd scan_pairs_vector;
    std::vector<float> scan_, scan_copy;
    std::vector<std::pair<double,double>> scan_pairs;
    float angle_increment_, angle;
    Eigen::Vector3d fused_output;

    bool wheel_odom_ready = false;
    bool lidar_odom_ready = false; 
    bool velocity_ready = false;
    bool prediction_ready = false;
    bool scan_ready = false;
    bool correction_finished = true;

    KalmanFilter kf;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
