/**
 * @file serial_client.hpp
 * @author Davy Refss
 * @brief 串口通信客户端
 */

#ifndef SERIAL_CLIENT_HPP
#define SERIAL_CLIENT_HPP

/* include files */
#include <array>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include "xbot_serial/protocol.hpp"
#include "xbot_serial/serial_port.hpp"

namespace xbot_serial {

/* 遥测数据 */
struct Telemetry {
    std::array<int16_t, protocol::xMotorNum> rpm{};
    std::array<int16_t, protocol::xMotorNum> encoder_count{};
    std::chrono::steady_clock::time_point received_at{};
    bool valid{false};
};  
/* ACK 数据*/
struct AckInfo
{
  uint8_t acknowledged_cmd{0};
  uint8_t result{0};
  uint8_t seq{0};
  std::chrono::steady_clock::time_point received_at{};
  bool valid{false};
};
/* 电池数据 */
struct BatteryInfo
{
  uint16_t voltage_mv{0};
  uint8_t soc_percent{0};
  uint16_t capacity_mah{0};
  uint8_t flags{0};
  std::chrono::steady_clock::time_point received_at{};
  bool valid{false};
};


/**
 * @brief Threaded client for the Xbot MCU protocol.
 *
 * One worker thread owns all serial read/write operations. The controller
 * loop only copies a four-element RPM command into a short mutex-protected
 * mailbox and reads the latest telemetry snapshot; it never waits on UART IO.
 */
class SerialClient {
public:
    SerialClient() = default;
    ~SerialClient();

    SerialClient(const SerialClient&) = delete;
    SerialClient &operator=(const SerialClient&) = delete;

    bool open(
        const std::string &device, 
        uint32_t baud_rate, 
        int read_timeout_ms = 5, 
        int write_timeout_ms = 20, 
        int command_watchdog_ms = 300
    );

    void close();

    bool is_open() const noexcept;
    bool is_healthy() const noexcept;

    void set_active(bool active);
    void set_target_rpm(const std::array<int16_t, protocol::xMotorNum> &rpm);
    void request_brake_all();

    bool get_telemetry(Telemetry &out) const;
    bool get_battery(BatteryInfo &out) const;
    bool get_last_ack(AckInfo &out) const;

private:
    void worker_loop();
    bool send_frame(uint8_t cmd, const uint8_t * data, uint8_t len);
    void handle_frame(const protocol::Frame & frame);

    SerialPort serial_;
    protocol::ProtocolParser parser_;

    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<bool> active_{false};
    std::atomic<bool> brake_requested_{false};
    std::thread worker_;

    int command_watchdog_ms_{300};
    uint8_t tx_seq_{0};

    mutable std::mutex command_mutex_;
    std::array<int16_t, protocol::xMotorNum> target_rpm_{};
    std::chrono::steady_clock::time_point last_command_update_{};
    bool have_command_update_{false};
    bool watchdog_latched_{false};

    mutable std::mutex telemetry_mutex_;
    Telemetry telemetry_{};

    mutable std::mutex battery_mutex_;
    BatteryInfo battery_{};

    mutable std::mutex ack_mutex_;
    AckInfo last_ack_{};

};

}

#endif /* SERIAL_CLIENT_HPP */ 
