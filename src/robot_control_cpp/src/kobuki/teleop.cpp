#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_cmd_vel")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Use arrow keys to move the robot. Ctrl-C to quit.");
        this->run();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    double current_linear_speed_ = 0.0;
    double current_angular_speed_ = 0.0;
    const double LINEAR_INCREMENT = 0.1;
    const double ANGULAR_INCREMENT = 3.14 / 40.0;
    const double MAX_LINEAR_SPEED = 500;
    const double MAX_ANGULAR_SPEED = 3.14 / 4;
    const double DECELERATION_RATE = 0.05;

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
            // Arrow keys produce escape sequences: "\033[A" up, "\033[B" down, "\033[C" right, "\033[D" left
            if (c == '\033') // escape sequence
            {
                getKey(); // skip '['
                switch (getKey())
                {
                case 'A': // up - increment forward speed
                    current_linear_speed_ += LINEAR_INCREMENT;
                    if (current_linear_speed_ > MAX_LINEAR_SPEED)
                        current_linear_speed_ = MAX_LINEAR_SPEED;
                    break;
                case 'B': // down - decrement forward speed
                    current_linear_speed_ -= LINEAR_INCREMENT;
                    if (current_linear_speed_ < -MAX_LINEAR_SPEED)
                        current_linear_speed_ = -MAX_LINEAR_SPEED;
                    break;
                case 'C': // right - decrease angular speed (turn right)
                    current_angular_speed_ -= ANGULAR_INCREMENT;
                    if (current_angular_speed_ < -MAX_ANGULAR_SPEED)
                        current_angular_speed_ = -MAX_ANGULAR_SPEED;
                    break;
                case 'D': // left - increase angular speed (turn left)
                    current_angular_speed_ += ANGULAR_INCREMENT;
                    if (current_angular_speed_ > MAX_ANGULAR_SPEED)
                        current_angular_speed_ = MAX_ANGULAR_SPEED;
                    break;
                default:
                    break;
                }
            }
            else if (c == 'q') // quit
            {
                break;
            }
            else if (c == 0) // no key pressed - apply deceleration
            {
                // Gradually slow down linear speed
                if (current_linear_speed_ > 0)
                    current_linear_speed_ -= DECELERATION_RATE;
                else if (current_linear_speed_ < 0)
                    current_linear_speed_ += DECELERATION_RATE;
                
                // Gradually slow down angular speed
                if (current_angular_speed_ > 0)
                    current_angular_speed_ -= DECELERATION_RATE;
                else if (current_angular_speed_ < 0)
                    current_angular_speed_ += DECELERATION_RATE;
            }
            
            msg.linear.x = current_linear_speed_;
            msg.angular.z = current_angular_speed_;
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
