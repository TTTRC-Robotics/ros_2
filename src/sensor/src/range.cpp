#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

extern "C" {
#include "serial_megapi/serial_megapi.h"
}

using namespace std::chrono_literals;

const int motor_left_ = 1;
const int motor_right_ = 2;

const int uss_port_ = 0x7;

const int max_speed_ = 255;
const int min_speed_ = 15;
const int set_speed_ = 50;

class Roscontrol : public rclcpp::Node
{
public:
    Roscontrol()
        : Node("Roscontrol"), count_(0)
    {
        range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("range_topic", 20);

        // Initialize the serial communication and store the file descriptor
        serial_fd_ = init_serial("/dev/ttyAMA0", 115200);
        if (serial_fd_ < 0) {
            // fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
            return;
        }

        front_distance_ = 0;
        uss_port_ = 0x7;

        timer_ = this->create_wall_timer(
            500ms, std::bind(&Roscontrol::timer_callback, this));

        // Set up non-blocking input
        struct termios t;
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag &= ~ICANON;
        t.c_lflag &= ~ECHO;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

    ~Roscontrol()
    {
        // Reset terminal settings on program exit
        struct termios t;
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag |= ICANON;
        t.c_lflag |= ECHO;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

private:
    void timer_callback()
    {
      char key;
        if (kbhit() && read(STDIN_FILENO, &key, 1) >= 0) {
            handle_keyboard_input(key);
        }

        drive_forward(set_speed_, serial_fd_, motor_left_, motor_right_);

        auto range_msg = sensor_msgs::msg::Range();
        range_msg.header.stamp = this->now();
        range_msg.header.frame_id = "sensor_frame";
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg.field_of_view = 0.30;  // Example field of view in radians
        range_msg.min_range = 0.1;      // Example minimum range in meters
        range_msg.max_range = 240.0;    // Example maximum range in meters
        range_msg.range = getSensorData();  // Replace this with a function that gets your sensor data
        range_publisher_->publish(range_msg);
    }

    float getSensorData()
    {
        // Replace this with code to get the actual sensor data
        // Example: Simulated data between 1.0 and 10.0 meters
        // For now, use the serial communication to get the actual sensor data
        request_uss(serial_fd_, uss_port_);

        if (is_ultrasonic_new_data(uss_port_)) {
            front_distance_ = get_uss_cm(uss_port_);
            // printf(" / uss_cm : %f \n", front_ditance);
        }
        return front_distance_;
    }

    void stop_motors(int fd_)
    {
        // Implement the logic to stop the motors using the provided file descriptor (fd)
        // For example:
        set_speed(fd_, motor_left_, 0);
        set_speed(fd_, motor_right_, 0);
    }

    void drive_forward(const int wheel_speed, const int fd_, const int motor_left_, const int motor_right_)
    {
        if (front_distance_ > 10 && front_distance_ < 0) {  // Obstacle detected, implement avoidance logic
            // Move backward
            set_speed(fd_, motor_left_, -wheel_speed);
            set_speed(fd_, motor_right_, wheel_speed);
        } else if (front_distance_ >= 23 && front_distance_ < 5) {
            // Turn left based on uss data
            set_speed(fd_, motor_left_, wheel_speed);
            set_speed(fd_, motor_right_, wheel_speed);
        } else {
            // No obstacle, drive forward
            set_speed(fd_, motor_left_, wheel_speed);
            set_speed(fd_, motor_right_, -wheel_speed);
        }

        // Add a condition to stop motors when no movement is desired
        if (front_distance_ == 0) {
            stop_motors(fd_);
        }
    }

    void handle_keyboard_input(char key)
    {
        switch (key) {
        case 'f':
            // Move forward
            drive_forward(set_speed_, serial_fd_, motor_left_, motor_right_);
            break;
        case 'b':
            // Move backward
            drive_forward(-set_speed_, serial_fd_, motor_left_, motor_right_);
            break;
        case 'r':
            // Turn right
            set_speed(serial_fd_, motor_left_, set_speed_);
            set_speed(serial_fd_, motor_right_, set_speed_);
            break;
        case 'l':
            // Turn left
            set_speed(serial_fd_, motor_left_, -set_speed_);
            set_speed(serial_fd_, motor_right_, -set_speed_);
            break;
        case 's':
            // Stop
            stop_motors(serial_fd_);
            break;
        default:
            break;
        }
    }

    int kbhit()
    {
        struct timeval tv;
        fd_set fds;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds); // STDIN_FILENO is the file descriptor for standard input
        select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        return FD_ISSET(STDIN_FILENO, &fds);
    }

    int uss_port_;
    int serial_fd_;
    float front_distance_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Roscontrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
key is not working
