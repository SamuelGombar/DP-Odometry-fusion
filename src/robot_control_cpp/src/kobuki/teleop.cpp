#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <algorithm>

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_cmd_vel")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Use arrow keys to move the robot. Space to stop. Ctrl-C to quit.");
        this->run();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    const double MAX_LINEAR_SPEED  = 0.5;
    const double MAX_ANGULAR_SPEED  = 3.14 / 2.0;
    const double LINEAR_INCREMENT   = 0.05;  // m/s per key press
    const double ANGULAR_INCREMENT  = 0.1;   // rad/s per key press
    const double LINEAR_STEP        = 0.03;  // m/s per ~100 ms ramp step
    const double ANGULAR_STEP       = 0.08;  // rad/s per ~100 ms ramp step

    double target_linear_  = 0.0;
    double target_angular_ = 0.0;
    double actual_linear_  = 0.0;
    double actual_angular_ = 0.0;

    static double ramp(double actual, double target, double step)
    {
        double diff = target - actual;
        if (std::abs(diff) <= step) return target;
        return actual + (diff > 0 ? step : -step);
    }

    char getKey()
    {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 0;
        old.c_cc[VTIME] = 1;
        if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0) perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
        return buf;
    }

    void run()
    {
        geometry_msgs::msg::Twist msg;
        while (rclcpp::ok())
        {
            char c = getKey();

            if (c == '\033') // escape sequence
            {
                getKey(); // skip '['
                switch (getKey())
                {
                case 'A': target_linear_  = std::clamp(target_linear_  + LINEAR_INCREMENT,  -MAX_LINEAR_SPEED,  MAX_LINEAR_SPEED);  break; // up
                case 'B': target_linear_  = std::clamp(target_linear_  - LINEAR_INCREMENT,  -MAX_LINEAR_SPEED,  MAX_LINEAR_SPEED);  break; // down
                case 'C': target_angular_ = std::clamp(target_angular_ - ANGULAR_INCREMENT, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED); break; // right
                case 'D': target_angular_ = std::clamp(target_angular_ + ANGULAR_INCREMENT, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED); break; // left
                default: break;
                }
            }
            else if (c == ' ') // stop
            {
                target_linear_  = 0.0;
                target_angular_ = 0.0;
            }
            else if (c == 'q')
            {
                break;
            }

            actual_linear_  = ramp(actual_linear_,  target_linear_,  LINEAR_STEP);
            actual_angular_ = ramp(actual_angular_, target_angular_, ANGULAR_STEP);

            // Kobuki bug workaround: always enforce a minimum angular velocity
            if (std::abs(actual_angular_) < 0.001)
                actual_angular_ = (actual_angular_ >= 0.0) ? 0.001 : -0.001;

            msg.linear.x  = actual_linear_;
            msg.angular.z = actual_angular_;
            pub_->publish(msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
