/**
 * @file listener.cpp
 * @author Davy Refss
 * @brief 
 * @note 
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>
#include <string>

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("my_subscriber") {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic_comm", 10, std::bind(&Subscriber::subs_cb, this, std::placeholders::_1)
        );
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    void subs_cb(const std_msgs::msg::String &msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: %s", msg.data.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();

    return 0;
}
