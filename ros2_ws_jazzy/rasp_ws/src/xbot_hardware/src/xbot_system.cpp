/**
 * @file xbot_system.cpp
 * @author Davy Refss
 * @brief XBot hardware system
 */

#include "xbot_hardware/xbot_system.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace xbot_hardware {
namespace {

bool contains_interface(
    const std::vector<hardware_interface::InterfaceInfo> &interfaces,
    const std::string &name) {
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [&name](const auto &interface_info) {
                       return interface_info.name == name;
                     });
}

bool parse_int(const std::unordered_map<std::string, std::string> &params,
               const std::string &key, int &out) {
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  try {
    size_t used = 0;
    const int value = std::stoi(it->second, &used);
    if (used != it->second.size()) {
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_double(const std::unordered_map<std::string, std::string> &params,
                  const std::string &key, double &out) {
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  try {
    size_t used = 0;
    const double value = std::stod(it->second, &used);
    if (used != it->second.size() || !std::isfinite(value)) {
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

} // namespace

XbotSystem::~XbotSystem() {
  serial_client_.set_active(false);
  serial_client_.close();
}

hardware_interface::CallbackReturn
XbotSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parse_parameters(info) || !validate_joint_interfaces(info)) {
    RCLCPP_ERROR(get_logger(),
                 "Invalid Xbot hardware parameters or joint interfaces");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(),
              "XbotSystem initialized: device=%s baud=%u CPR=%d max_rpm=%.1f",
              device_.c_str(), baud_rate_, enc_cpr_, max_rpm_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool XbotSystem::parse_parameters(
    const hardware_interface::HardwareInfo &info) {
  const auto &params = info.hardware_parameters;
  const auto device_it = params.find("device");
  if (device_it == params.end() || device_it->second.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing required hardware parameter: device");
    return false;
  }
  device_ = device_it->second;

  int baud = static_cast<int>(baud_rate_);
  if (!parse_int(params, "baud_rate", baud) || baud <= 0) {
    RCLCPP_ERROR(get_logger(), "Invalid or missing baud_rate");
    return false;
  }
  baud_rate_ = static_cast<uint32_t>(baud);

  if (!parse_int(params, "enc_cpr", enc_cpr_) || enc_cpr_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Invalid or missing enc_cpr");
    return false;
  }
  if (!parse_double(params, "max_rpm", max_rpm_) || max_rpm_ <= 0.0 ||
      max_rpm_ > 32767.0) {
    RCLCPP_ERROR(get_logger(), "Invalid or missing max_rpm (must be <= 32767)");
    return false;
  }

  // Optional parameters have conservative defaults.
  if (params.count("read_timeout_ms") != 0 &&
      (!parse_int(params, "read_timeout_ms", read_timeout_ms_) ||
       read_timeout_ms_ < 1)) {
    return false;
  }
  if (params.count("command_watchdog_ms") != 0 &&
      (!parse_int(params, "command_watchdog_ms", command_watchdog_ms_) ||
       command_watchdog_ms_ < 50)) {
    return false;
  }
  if (params.count("telemetry_timeout_ms") != 0 &&
      (!parse_int(params, "telemetry_timeout_ms", telemetry_timeout_ms_) ||
       telemetry_timeout_ms_ < 50)) {
    return false;
  }

  const auto parse_sign_array =
      [this, &params](const std::string &key,
                      std::array<double, 4> &signs) -> bool {
    const auto it = params.find(key);
    if (it == params.end()) {
      return true; // default: all directions positive
    }
    std::stringstream stream(it->second);
    std::string token;
    size_t index = 0;
    while (std::getline(stream, token, ',')) {
      if (index >= signs.size()) {
        RCLCPP_ERROR(get_logger(), "%s must contain exactly four values",
                     key.c_str());
        return false;
      }
      try {
        size_t used = 0;
        const double sign = std::stod(token, &used);
        if (used != token.size() || !std::isfinite(sign) ||
            std::abs(std::abs(sign) - 1.0) > 1e-9) {
          RCLCPP_ERROR(get_logger(), "%s entries must each be +1 or -1",
                       key.c_str());
          return false;
        }
        signs[index++] = sign;
      } catch (const std::exception &) {
        return false;
      }
    }
    if (index != signs.size()) {
      RCLCPP_ERROR(get_logger(),
                   "%s must contain exactly four comma-separated values",
                   key.c_str());
      return false;
    }
    return true;
  };

  if (!parse_sign_array("command_signs", command_signs_) ||
      !parse_sign_array("encoder_signs", encoder_signs_)) {
    return false;
  }

  return true;
}

bool XbotSystem::validate_joint_interfaces(
    const hardware_interface::HardwareInfo &info) const {
  if (info.joints.size() != kJointNames.size()) {
    RCLCPP_ERROR(get_logger(), "Expected exactly four wheel joints, got %zu",
                 info.joints.size());
    return false;
  }

  for (const char *expected_name : kJointNames) {
    const auto joint_it = std::find_if(info.joints.begin(), info.joints.end(),
                                       [expected_name](const auto &joint) {
                                         return joint.name == expected_name;
                                       });
    if (joint_it == info.joints.end()) {
      RCLCPP_ERROR(get_logger(), "Missing wheel joint '%s' in ros2_control tag",
                   expected_name);
      return false;
    }
    if (!contains_interface(joint_it->command_interfaces,
                            hardware_interface::HW_IF_VELOCITY) ||
        !contains_interface(joint_it->state_interfaces,
                            hardware_interface::HW_IF_POSITION) ||
        !contains_interface(joint_it->state_interfaces,
                            hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_ERROR(
          get_logger(),
          "Joint '%s' must expose velocity command and position/velocity state",
          expected_name);
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn
XbotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    if (!serial_client_.open(device_, baud_rate_, read_timeout_ms_, 20,
                             command_watchdog_ms_)) {
      RCLCPP_ERROR(get_logger(), "Failed to open Xbot MCU serial device: %s",
                   device_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to start serial client: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Xbot MCU serial client connected on %s",
              device_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
XbotSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  active_ = false;
  serial_client_.set_active(false);
  serial_client_.close();
  have_encoder_baseline_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
XbotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!serial_client_.is_healthy()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate: serial link is not healthy");
    return hardware_interface::CallbackReturn::ERROR;
  }

  have_encoder_baseline_ = false;
  accumulated_encoder_count_.fill(0);
  last_encoder_count_.fill(0);
  activated_at_ = std::chrono::steady_clock::now();
  active_ = true;
  serial_client_.set_active(true);

  for (size_t i = 0; i < kJointNames.size(); ++i) {
    set_state(interface_name(i, hardware_interface::HW_IF_POSITION), 0.0);
    set_state(interface_name(i, hardware_interface::HW_IF_VELOCITY), 0.0);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
XbotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  active_ = false;
  serial_client_.set_active(
      false); // schedules BRAKE_ALL on the IO worker thread
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
XbotSystem::read(const rclcpp::Time & /*time*/,
                 const rclcpp::Duration & /*period*/) {
  if (!serial_client_.is_healthy()) {
    RCLCPP_ERROR(get_logger(), "Serial client is unhealthy");
    return hardware_interface::return_type::ERROR;
  }

  xbot_serial::Telemetry telemetry;
  if (!serial_client_.get_telemetry(telemetry)) {
    const auto age = std::chrono::steady_clock::now() - activated_at_;
    if (age > std::chrono::milliseconds(telemetry_timeout_ms_)) {
      RCLCPP_ERROR(get_logger(),
                   "No valid telemetry received after activation");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - telemetry.received_at >
      std::chrono::milliseconds(telemetry_timeout_ms_)) {
    RCLCPP_ERROR(get_logger(),
                 "Telemetry is stale; refusing to publish old wheel states");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < kJointNames.size(); ++i) {
    if (!have_encoder_baseline_) {
      last_encoder_count_[i] = telemetry.encoder_count[i];
      accumulated_encoder_count_[i] = 0;
    } else {
      accumulated_encoder_count_[i] +=
          position_delta(telemetry.encoder_count[i], last_encoder_count_[i]);
      last_encoder_count_[i] = telemetry.encoder_count[i];
    }

    const double position_rad =
        static_cast<double>(accumulated_encoder_count_[i]) * encoder_signs_[i] *
        kTwoPi / static_cast<double>(enc_cpr_);
    const double velocity_rad_s = static_cast<double>(telemetry.rpm[i]) *
                                  encoder_signs_[i] * kTwoPi / 60.0;

    set_state(interface_name(i, hardware_interface::HW_IF_POSITION),
              position_rad);
    set_state(interface_name(i, hardware_interface::HW_IF_VELOCITY),
              velocity_rad_s);
  }
  have_encoder_baseline_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
XbotSystem::write(const rclcpp::Time & /*time*/,
                  const rclcpp::Duration & /*period*/) {
  if (!serial_client_.is_healthy()) {
    return hardware_interface::return_type::ERROR;
  }
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  std::array<int16_t, 4> target_rpm{};
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    const std::string command_name =
        interface_name(i, hardware_interface::HW_IF_VELOCITY);
    if (!has_command(command_name)) {
      RCLCPP_ERROR(get_logger(), "Missing command interface '%s'",
                   command_name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    double velocity = get_command<double>(command_name);
    if (!std::isfinite(velocity)) {
      velocity = 0.0;
    }

    double rpm = velocity * 60.0 / kTwoPi * command_signs_[i];
    rpm = std::clamp(rpm, -max_rpm_, max_rpm_);
    target_rpm[i] = static_cast<int16_t>(std::lround(rpm));
  }

  serial_client_.set_target_rpm(target_rpm);
  return hardware_interface::return_type::OK;
}

std::string XbotSystem::interface_name(size_t motor_index,
                                       const char *interface_type) const {
  return std::string(kJointNames.at(motor_index)) + "/" + interface_type;
}

int64_t XbotSystem::position_delta(int32_t current, int32_t previous) {
  const uint32_t difference =
      static_cast<uint32_t>(current) - static_cast<uint32_t>(previous);
  return difference <= 0x7FFFFFFFu
             ? static_cast<int64_t>(difference)
             : static_cast<int64_t>(difference) - 0x100000000LL;
}

} // namespace xbot_hardware

PLUGINLIB_EXPORT_CLASS(xbot_hardware::XbotSystem,
                       hardware_interface::SystemInterface)
