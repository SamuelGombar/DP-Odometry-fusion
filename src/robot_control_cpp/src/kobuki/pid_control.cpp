#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Geometry> 

using namespace std::chrono_literals;


struct Point {
    double x;
    double y;
};


class PidControl : public rclcpp::Node
{
public:
	PidControl() : Node("pid_control")
	{
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/odom", 10,
			std::bind(&PidControl::odom_callback, this, std::placeholders::_1));

		timer_ = this->create_wall_timer(100ms, std::bind(&PidControl::on_timer, this));
	}

	void publish_cmd(double linear_x, double angular_z)
	{
		auto msg = geometry_msgs::msg::Twist();
		msg.linear.x = linear_x;
		msg.angular.z = angular_z;
		cmd_vel_pub_->publish(msg);
	}

private:
	void on_timer()
	{
        static double Kp_r = 1.0;
        static double Kp_t = 0.001;
        static double Ki_t = 0.0001;
        static double integral_pos = 0;
        static double error_r = 0;
        static double error_t = 0;
        static double target_phi = 0;
        static double output_r = 0;
        static double output_t = 0;
        static std::vector<Point> checkpoints;
        static bool flag = true;
        static double max_r = 0.3;
        static double max_t = 0.3;
        static double incr_r = M_PI/40;
        static double incr_t = 0.1;
        static bool is_moving = false;
        static bool is_rotating = true;
        static bool combined_regulator = true;
        static double TOLER = 0.1;
        static bool finish = false;
        static Point last_checkpoint;
        static double datacounter = 0.0;

        if (flag) {
            checkpoints.push_back({0, 3.5});
            checkpoints.push_back({2.9, 3.5});
            checkpoints.push_back({2.9, 1.75});
            // checkpoints.push_back({0.8, 1.5});
            // checkpoints.push_back({0, 3.5});
            // checkpoints.push_back({0, 0});
            flag = false;
        }
		
        if (!finish) {
            if (this->x > checkpoints[0].x - TOLER && this->x < checkpoints[0].x + TOLER) {
                if (this->y > checkpoints[0].y - TOLER && this->y < checkpoints[0].y + TOLER) {
                    last_checkpoint = checkpoints[0];
                    checkpoints.erase(checkpoints.begin());
                    RCLCPP_INFO(this->get_logger(), "VYMAYAL SOM CHECKPOINT");
                    datacounter = 0.0;
                    integral_pos = 0.0;
                }
            }
        }

        if (checkpoints.empty()) {
            checkpoints.push_back(last_checkpoint);
            finish = true;
        }

        target_phi = atan2(checkpoints[0].y - this->y, checkpoints[0].x - this->x);
        error_r = normalizeAngle(target_phi - this->phi);
        error_t = sqrt(pow(checkpoints[0].x - this->x, 2) + pow(checkpoints[0].y - this->y, 2));
        integral_pos = integral_pos + error_t*datacounter;

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "\nTarget phi: %.3f\nAngular error: %.3f\nDistance error: %.3f\n",
        //     target_phi,
        //     error_r,
        //     error_t
        // );

        if (combined_regulator) {
            //rampa
            if (error_r >= 0) {
                output_r = output_r + incr_r;
                if (output_r >= Kp_r*error_r) {
                    output_r = Kp_r*error_r;
                }
            }
            else if (error_r <= 0) {
                output_r = output_r - incr_r;
                if (output_r <= Kp_r*error_r) {
                    output_r = Kp_r*error_r;
                }
            }

            //maximalna rychlost otacania
            if (output_r > max_r) output_r = max_r;
            else if (output_r < -max_r) output_r = -max_r;

            //ak error > M_PI/4, tak zastane a zacne sa tocit na mieste
            if (abs(error_r) > M_PI/10) {
                output_t = output_t - incr_t;
                if (output_t <= 0) output_t = 0;
            }
            else {
                output_t = output_t + incr_t;
                if (output_t >= Kp_t*error_t + Ki_t*integral_pos) {
                    output_t = Kp_t*error_t + Ki_t*integral_pos;
                }
                if (output_t > max_t) output_t = max_t;
                else if (output_t < -max_t) output_t = -max_t;

                // radius_of_rot = (robotCom.b/2)*
                //                 ((r_wheel_v + l_wheel_v)/
                //                  (r_wheel_v - l_wheel_v));
            }

            if (finish) publish_cmd(0.0, 0.0);
            else publish_cmd(output_t, output_r);
            // std::cout << output_t << std::endl;
            // std::cout << output_r << std::endl;
        }
		datacounter++;
	}

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		// TODO: Use odometry data for PID control
		// RCLCPP_INFO(this->get_logger(), "Received odom: position (%.2f, %.2f)",
        // msg->pose.pose.position.x, msg->pose.pose.position.y);
        this->x = msg->pose.pose.position.x;
        this->y = msg->pose.pose.position.y;

        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );

        Eigen::Matrix3d R = q.toRotationMatrix();
        this->phi = std::atan2(R(1,0), R(0,0));  // yaw from rotation matrix
	}

    double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Point> checkpoints;
    double x;
    double y;
    double phi;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PidControl>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

