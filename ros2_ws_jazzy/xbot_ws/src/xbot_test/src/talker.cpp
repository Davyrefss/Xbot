/**
 * @file talker.cpp
 * @author Davy Refss
 * @brief 
 * @note 
 */

/* include files */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>   
#include <memory>
#include <string>    
#include <functional> 

class Publisher : public rclcpp::Node {
public:
    Publisher() : Node("my_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "topic_comm", 10); 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&Publisher::timer_callback, this)
        );    
    }  
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ =  0;
    void timer_callback() {
        std_msgs::msg::String msg;
        msg.data = "Hello ROS2: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
        publisher_->publish(msg);
    }   
};

int main(int agrc, char **argv) {
    rclcpp::init(agrc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();

    return 0;

}
