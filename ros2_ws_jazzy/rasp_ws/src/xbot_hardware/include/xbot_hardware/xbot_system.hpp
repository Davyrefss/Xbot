/**
 * @file xbot_system.hpp
 * @author Davy Refss
 * @brief 底盘系统节点
 */

#ifndef XBOT_SYSTEM_HPP
#define XBOT_SYSTEM_HPP

/* include files */
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <string>
#include "xbot_serial/serial_client.hpp"

namespace xbot_hardware {

class XbotSystem final : public hardware_interface::SystemInterface {
public:
    XbotSystem() = default;
    ~XbotSystem() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;  
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
private:
    static constexpr double kTwoPi = 6.28318530717958647692;
    static constexpr std::array<const char *, 2> kJointNames{
        "left_rear_wheel_joint",
        "right_rear_wheel_joint"};
    static int64_t position_delta(int32_t currnet, int32_t previous);
    bool parse_parameters(const hardware_interface::HardwareInfo &info);
    bool validate_joint_interfaces(const hardware_interface::HardwareInfo &info) const; 
    std::string interface_name(size_t motor_index, const char *interface_type) const;
    
    xbot_serial::SerialClient serial_client_;
    std::string device_{"/dev/ttyUSB0"};
    uint32_t baud_rate_{115200};
    int enc_cpr_{1320};
    double max_rpm_{300.0};
    int read_timeout_ms_{5};
    int command_watchdog_ms_{300};
    int telemetry_timeout_ms_{500};
    // Sign converts command / encoder conventions to each URDF joint axis.
    // Index order matches the MCU protocol: M1 LF, M2 RF, M3 LR, M4 RR.
    std::array<double, 4> command_signs_{{1.0, 1.0, 1.0, 1.0}};
    std::array<double, 4> encoder_signs_{{1.0, 1.0, 1.0, 1.0}};

    std::array<int32_t, 2> last_encoder_count_{};
    std::array<int64_t, 2> accumulated_encoder_count_{};
    bool have_encoder_baseline_{false};
    std::atomic<bool> active_{false};
    std::chrono::steady_clock::time_point activated_at_{};
};


}   // namespace xbot_hardware



#endif  /* XBOT_SYSTEM_HPP */
