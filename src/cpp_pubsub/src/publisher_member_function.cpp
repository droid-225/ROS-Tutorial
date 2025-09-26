#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() 
   to register a memeber function as a callback from the timer*/

class MinimalPublisher : public rclcpp::Node {
    public:
        MinimalPublisher() : Node ("minimal_publisher"), count_(0) {
            // DEF: std::shared_ptr<PublisherT> create_publisher(const std::string &topic_name, size_t queue_depth);
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

            // DEF: rclcpp::WallTimer<CallbackT>::SharedPtr create_wall_timer(std::chrono::duration<int64_t, DurationT> period, CallbackT callback);
            timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

    private:
        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char * argv[]) {
    // Initializes ROS 2 communication
    rclcpp::init(argc, argv);

    // Creates an instance of Minimal Publisher and enters a loop, 
    // keeping the node alive and responsive to callbacks
    rclcpp::spin(std::make_shared<MinimalPublisher>());

    // Cleans up and shuts down ROS 2 communication
    rclcpp::shutdown();
    return 0;
}