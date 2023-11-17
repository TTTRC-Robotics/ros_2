#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

extern "C" {
#include "serial_megapi/serial_megapi.h"
}

class CmdVelSubscriber : public rclcpp::Node
{
public:
    CmdVelSubscriber() : Node("cmd_vel_subscriber")
    {
        // Initialize the serial communication and store the file descriptor
        serial_fd_ = init_serial("/dev/ttyAMA0", 115200);
        if (serial_fd_ < 0) {
            // fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
            return;
        }

        // Subscribe to the cmd_vel topic
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelSubscriber::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void driveRobot(double speed_left, double speed_right){
        set_speed(serial_fd_, motor_left_, static_cast<int>(speed_left));
        set_speed(serial_fd_, motor_right_, static_cast<int>(speed_right));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Handle the received cmd_vel message
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel message - Linear: (%f, %f, %f), Angular: (%f, %f, %f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);

        // Convert cmd_vel to motor commands (simple linear mapping)
        const double linear_scaling = 100.0;  // Adjust this scaling factor based on your robot
        const double angular_scaling = 50.0; // Adjust this scaling factor based on your robot

        double left_motor_cmd = linear_scaling * msg->linear.x + angular_scaling * msg->angular.z;
        double right_motor_cmd = -1 * (linear_scaling * msg->linear.x - angular_scaling * msg->angular.z);

        RCLCPP_INFO(this->get_logger(), "Speed_left: (%f), Speed_Right: (%f)",
                    left_motor_cmd,
                    right_motor_cmd);

        driveRobot(left_motor_cmd, right_motor_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    int serial_fd_;
    const int motor_left_ = 1;
    const int motor_right_ = 2;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the subscriber node
    auto cmd_vel_subscriber = std::make_shared<CmdVelSubscriber>();

    // Spin the node
    rclcpp::spin(cmd_vel_subscriber);

    rclcpp::shutdown();

    return 0;
}
