/**
 * @file teleop_keyboard.cpp
 * @author Davyrefss
 * @brief keyboard teleop for XBot
 * 
 */

/* include files */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <chrono>
#include <functional>
#include <cstdint>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

#include "xbot_control/teleop_keyboard.hpp"

namespace xbot_control
{
    /**
     * @brief 构造函数，初始化节点和发布者  
     * 
     */
    TeleopKeyboard::TeleopKeyboard()  : Node("teleop_keyboard") {
        RCLCPP_INFO(this->get_logger(), "Teleop Keyboard node initialized");
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            /* 将publish函数绑定到定时器 */
            std::bind(&TeleopKeyboard::publish, this)
        );
    }

    /**
     * @brief 发布Twist消息
     * 
     */
    void TeleopKeyboard::publish() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vel_linear_;
        msg.angular.z = vel_angular_;
        publisher_->publish(msg);
    }

    /**
     * @brief 设置终端为原始模式，关闭行缓冲和回显
     * 
     */
    void TeleopKeyboard::set_terminal_mode_raw() {
        tcgetattr(STDIN_FILENO, &oldt_);    /* 保存当前终端设置到oldt_ */
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);  /* 关掉 ICANON（行缓冲）和 ECHO（回显）*/
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
    }

    /**
     * @brief 恢复终端为原始模式前的设置
     * 
     */
    void TeleopKeyboard::reset_terminal_mode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

    /**
     * @brief 处理按键输入，设置线速度和角速度
     * @param key 按键字符
     * 
     */
    void TeleopKeyboard::handle_key(char key) {
        switch (key) {
            case 'w':
                vel_linear_ = 0.5;
                vel_angular_ = 0.0;
                break;
            case 's':
                vel_linear_ = -0.5;
                vel_angular_ = 0.0;
                break;
            case 'a':
                vel_linear_ = 0.0;
                vel_angular_ = 1.0;
                break;
            case 'd':
                vel_linear_ = 0.0;
                vel_angular_ = -1.0;
                break;
            case 'q':
                vel_linear_ = 0.0;
                vel_angular_ = 0.0;
                break;
        }
    }

    /**
     * @brief 运行
     * 
     */
    void TeleopKeyboard::run() {
        set_terminal_mode_raw();
        /* 将标准输入设置为非阻塞模式 */
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        while (rclcpp::ok())
        {
            int8_t key;
            if (read(STDIN_FILENO, &key, 1) > 0) {
                handle_key(key);
            }
            /* 设置为非阻塞--处理定时器事件 */
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        reset_terminal_mode();
        
    }
}   

int main(int argc, char const** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xbot_control::TeleopKeyboard>();
    node->run();
    rclcpp::shutdown();

    return 0;
}
    