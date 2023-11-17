#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

extern "C" {
#include "serial_megapi/serial_megapi.h"
}

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
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
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    ~MinimalPublisher()
    {}

private:
    void timer_callback()
    {
        auto range_msg = sensor_msgs::msg::Range();
        range_msg.header.stamp = this->now();
        range_msg.header.frame_id = "sensor_frame";
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg.field_of_view = 0.30;  // Example field of view in radians
        range_msg.min_range = 0.1;      // Example minimum range in meters
        range_msg.max_range = 240.0;     // Example maximum range in meters
        range_msg.range = getSensorData();  // Replace this with a function that gets your sensor data
        range_publisher_->publish(range_msg);
    }

    float getSensorData()
    {
        // Replace this with code to get the actual sensor data
        // Example: Simulated data between 1.0 and 10.0 meters
        // For now, use the serial communication to get the actual sensor data
        request_uss(serial_fd_, uss_port_);

        if(is_ultrasonic_new_data(uss_port_)){
            front_distance_ = get_uss_cm(uss_port_);
            // printf(" / uss_cm : %f \n", front_ditance);
        }

        return front_distance_;
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
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
