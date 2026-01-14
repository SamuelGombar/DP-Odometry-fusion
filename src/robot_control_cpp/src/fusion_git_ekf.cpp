#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ekf.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <system_simulator.hpp>

class OdomSubscriberNode : public rclcpp::Node
{
public:
    OdomSubscriberNode() : Node("odom_subscriber_node")
    {
        // Subscribe to wheel odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomSubscriberNode::odomCallback, this, std::placeholders::_1));

        // Subscribe to ICP odometry
        icp_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_icp", 10,
            std::bind(&OdomSubscriberNode::icpCallback, this, std::placeholders::_1));

        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_fused", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan", 10,
                    std::bind(&OdomSubscriberNode::scan_callback, this, std::placeholders::_1));

        x0.resize(4);
        x0 << 0.0, 0.0, 0.0, 0.0;
        Q.resize(4, 4);
        Q << 0.01, 0.0, 0.0, 0.0,
             0.0, 0.01, 0.0, 0.0,
             0.0, 0.0, 0.01, 0.0,
             0.0, 0.0, 0.0, 0.01;
        R.resize(3, 3);
        R << 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.05; 

        double dt = 0.0;
        model = std::make_shared<SystemModelA>(x0, R, Q, dt);

        P.resize(4,4);
        P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
        ekf = std::make_shared<ExtendedKF>(model, P);
        ekf->init(x0);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        wheel_odom_setup = true;
        rclcpp::Time current_time = msg->header.stamp;
        double dt = 0.0;
        if (last_time.nanoseconds() != 0) {
            dt = (current_time - last_time).seconds();
        }

        Eigen::Vector3d u;
        u(0) = msg->pose.pose.position.x;
        u(1) = msg->pose.pose.position.y;
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        double yaw = q.toRotationMatrix().eulerAngles(2, 1, 0)[0];
        u(2) = yaw;
        last_time = current_time;
        if (isPredictionReady()) {
            prediction_finished = false;
            model->set_dt(dt);
            ekf->predict(u);
            prediction_finished = true;
        }
        // do something with wheel odometry
    }


    void icpCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        lidar_odom_setup = true;
        Eigen::VectorXd u_meas(3);
        u_meas(0) = msg->pose.pose.position.x;
        u_meas(1) = msg->pose.pose.position.y;
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        double yaw = q.toRotationMatrix().eulerAngles(2, 1, 0)[0];
        u_meas(2) = yaw;
        if (isCorrectionReady()) {
            correction_finished = false;
            ekf->update(u_meas);
            correction_finished = true;
            scan_ready = false;

            Eigen::VectorXd x_est = ekf->get_state();

            auto fused_msg = nav_msgs::msg::Odometry();
            fused_msg.header.stamp = this->get_clock()->now();
            fused_msg.header.frame_id = "odom";
            fused_msg.child_frame_id = "base_link";

            // Set Position
            fused_msg.pose.pose.position.x = x_est(0);
            fused_msg.pose.pose.position.y = x_est(1);
            fused_msg.pose.pose.position.z = 0.0;
            Eigen::Quaterniond q_fused;
            q_fused = Eigen::AngleAxisd(x_est(2), Eigen::Vector3d::UnitZ());

            fused_msg.pose.pose.orientation.x = q_fused.x();
            fused_msg.pose.pose.orientation.y = q_fused.y();
            fused_msg.pose.pose.orientation.z = q_fused.z();
            fused_msg.pose.pose.orientation.w = q_fused.w();

            fused_odom_pub_->publish(fused_msg);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_setup = true;
        scan_msg_ = msg;
        scan_ready = true;
    }

    bool isPredictionReady() {
        return (wheel_odom_setup && lidar_odom_setup && scan_setup && correction_finished);
    }

    bool isCorrectionReady() {
        return (wheel_odom_setup && lidar_odom_setup && scan_setup && prediction_finished && scan_ready);
    }

    bool wheel_odom_setup = false;
    bool lidar_odom_setup = false;
    bool scan_setup = false;

    bool scan_ready = false;
    
    bool correction_finished = true;
    bool prediction_finished = false;
    
    rclcpp::Time last_time;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    Eigen::VectorXd x0;
    Eigen::MatrixXd P, Q, R;
    std::shared_ptr<SystemModelA> model;
    std::shared_ptr<ExtendedKF> ekf;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
