#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"  // Fix the include path

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0) {
        // Correct the namespace and include path for the Range message
        publisher_ = this->create_publisher<sensor_msgs::msg::Range>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = sensor_msgs::msg::Range();
        message.range = 5.0 + count_++; // Adjust this based on your desired range
        message.field_of_view = 0.1;    // Replace with your desired field of view
        message.min_range = 0.0;        // Replace with your desired minimum range
        message.max_range = 10.0;       // Replace with your desired maximum range

        RCLCPP_INFO(this->get_logger(), "Publishing range: %.2f", message.range);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
