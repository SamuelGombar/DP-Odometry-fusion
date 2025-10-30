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

    char getKey()
    {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
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
                case 'A': msg.linear.x = 0.4; msg.angular.z = 0.0; break; // up
                case 'B': msg.linear.x = -0.4; msg.angular.z = 0.0; break; // down
                case 'C': msg.linear.x = 0.0; msg.angular.z = -1.5; break; // right
                case 'D': msg.linear.x = 0.0; msg.angular.z = 1.5; break; // left
                default: msg.linear.x = 0.0; msg.angular.z = 0.0; break;
                }
                pub_->publish(msg);
            }
            else if (c == 'q') // quit
            {
                break;
            }
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
