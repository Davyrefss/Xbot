/**
 * @file teleop_keyboard.hpp
 * @author Davyrefss
 * @brief keyboard teleop for XBot
 * 
 */

#ifndef TELEOP_KEYBOARD_HPP
#define TELEOP_KEYBOARD_HPP

/* include files */
#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/twist.hpp>   
#include <termios.h>

namespace xbot_control
{
class TeleopKeyboard : public rclcpp::Node {
public:
    TeleopKeyboard(); 
    ~TeleopKeyboard() = default;
    void run();

private:
    /* 定义了一个智能指针指向发布Twist消息的发布者对象 */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double vel_linear_ = 0.0;  /* 线速度 m/s */
    double vel_angular_ = 0.0; /* 角速度 rad/s */
    struct termios oldt_, newt_; /* 用于保存终端设置 */

    void publish();
    void set_terminal_mode_raw();
    void reset_terminal_mode();
    void handle_key(char key);
    

};   

}

#endif /* TELEOP_KEYBOARD_HPP */
