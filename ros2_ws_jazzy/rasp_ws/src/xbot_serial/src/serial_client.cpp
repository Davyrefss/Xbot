/**
 * @file serial_client.cpp
 * @author Davy Refss
 * @brief 串口通信客户端
 */

/* include files */
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include "xbot_serial/serial_client.hpp"

namespace xbot_serial {

SerialClient::~SerialClient(){
    close();
}

bool SerialClient::open(
        const std::string &device, 
        uint32_t baud_rate, 
        int read_timeout_ms, 
        int write_timeout_ms, 
        int command_watchdog_ms
    ) 
    {
        if (serial_.IsOpen() || running_.load()) {
            return false;
        }
        if (command_watchdog_ms <= 0) {
            return false;
        }
        if (!serial_.open(device, baud_rate)) {
            return false;
        }

        serial_.SetTimeoutRead(std::max(1, read_timeout_ms));
        serial_.SetTimeoutWrite(std::max(1, write_timeout_ms));
        command_watchdog_ms_ = command_watchdog_ms;
        tx_seq_ = 0;
        parser_.reset();

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            target_rpm_.fill(0);
            have_command_update_ = false;
            watchdog_latched_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            telemetry_ = Telemetry{};
        }
        {
            std::lock_guard<std::mutex> lock(battery_mutex_);
            battery_ = BatteryInfo{};
        }
        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            last_ack_ = AckInfo{};
        }

        active_.store(false);
        brake_requested_.store(false);
        healthy_.store(true);
        running_.store(true);
        try {
            worker_ = std::thread(&SerialClient::worker_loop, this);
        } catch (...) {
            running_.store(false);
            healthy_.store(false);
            serial_.close();
            throw;
        }
        return true;
    }

void SerialClient::close() {
    // Give the IO worker a brief opportunity to transmit BRAKE_ALL before the
    // port is closed. UART operations remain owned by the worker thread.
    if (active_.exchange(false)) {
        brake_requested_.store(true);
    }
    const auto brake_deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(50);
    while (running_.load() && brake_requested_.load() &&
    std::chrono::steady_clock::now() < brake_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    serial_.close();
    healthy_.store(false);
}

bool SerialClient::is_open() const noexcept {
    return serial_.IsOpen();
}

bool SerialClient::is_healthy() const noexcept {
    return healthy_.load();
}

void SerialClient::set_active(bool active) {
    if (active) {
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        target_rpm_.fill(0);
        last_command_update_ = std::chrono::steady_clock::now();
        have_command_update_ = true;
        watchdog_latched_ = false;
    }
    brake_requested_.store(false);
    active_.store(true);
    } else {
        active_.store(false);
        request_brake_all();
    }
}

void SerialClient::set_target_rpm(const std::array<int16_t, protocol::xMotorNum> & rpm) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    target_rpm_ = rpm;
    last_command_update_ = std::chrono::steady_clock::now();
    have_command_update_ = true;
    watchdog_latched_ = false;
}

void SerialClient::request_brake_all() {
    brake_requested_.store(true);
}

bool SerialClient::get_telemetry(Telemetry & out) const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    out = telemetry_;
    return out.valid;
}

bool SerialClient::get_battery(BatteryInfo & out) const {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    out = battery_;
    return out.valid;
}

bool SerialClient::get_last_ack(AckInfo & out) const {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    out = last_ack_;
    return out.valid;
}

bool SerialClient::send_frame(uint8_t cmd, const uint8_t * data, uint8_t len) {
    std::array<uint8_t, protocol::xMaxFrame> frame{};
    const size_t frame_size = protocol::build_frame(
        frame.data(), cmd, tx_seq_++, data, len);
    if (frame_size == 0) {
        return false;
    }
    const int written = serial_.write(frame.data(), frame_size);
    return written == static_cast<int>(frame_size);
}

void SerialClient::handle_frame(const protocol::Frame & frame) {
    if (frame.cmd == protocol::xCmdAck) {
        if (frame.len < protocol::xAckLen) {
            return;
        }
        AckInfo next;
        next.acknowledged_cmd = frame.data[0];
        next.result = frame.data[1];
        next.seq = frame.seq;
        next.received_at = std::chrono::steady_clock::now();
        next.valid = true;
        std::lock_guard<std::mutex> lock(ack_mutex_);
        last_ack_ = next;
        return;
    }

    if (frame.cmd == protocol::xCmdTelemetry) {
        if (frame.len < protocol::xTelemetryLen) {
            return;
        }

        Telemetry next;
        for (size_t i = 0; i < protocol::xMotorNum; ++i) {
            next.rpm[i] = protocol::unpack_i16(&frame.data[2 * i]);
            next.encoder_count[i] = protocol::unpack_i32(&frame.data[8 + 4 * i]);
        }
        next.received_at = std::chrono::steady_clock::now();
        next.valid = true;
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_ = next;
        return;
    }

    if (frame.cmd == protocol::xCmdBattery) {
        if (frame.len < protocol::xBatteryLen) {
            return;
        }
        BatteryInfo next;
        next.voltage_mv = static_cast<uint16_t>(
            protocol::unpack_i16(&frame.data[0]));
        next.soc_percent = frame.data[2];
        next.capacity_mah = static_cast<uint16_t>(
            protocol::unpack_i16(&frame.data[3]));
        next.flags = frame.data[5];
        next.received_at = std::chrono::steady_clock::now();
        next.valid = true;
        std::lock_guard<std::mutex> lock(battery_mutex_);
        battery_ = next;
    }
    // Unknown frame types are ignored here; a diagnostics layer can inspect
    // the typed snapshots without logging synchronously in the IO thread.
}

void SerialClient::worker_loop() {
    using clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;

    auto next_speed = clock::now();
    auto next_telemetry = clock::now();
    auto next_battery = clock::now() + 5s;
    auto next_ping = clock::now() + 1s;
    std::array<uint8_t, 256> rx{};
    while (running_.load()) {
        const int bytes = serial_.read(rx.data(), rx.size());
        if (bytes < 0) {
            healthy_.store(false);
            running_.store(false);
            break;
        }

        if (bytes == 0) {
            parser_.poll(protocol::monotonic_ms());
        } else {
            for (int i = 0; i < bytes; ++i) {
                if (parser_.feed(rx[static_cast<size_t>(i)], protocol::monotonic_ms())) {
                    handle_frame(parser_.get_frame());
                }
            }
        }

        const auto now = clock::now();

        if (brake_requested_.exchange(false)) {
            if (!send_frame(protocol::xCmdBrakeAll, nullptr, 0)) {
                healthy_.store(false);
                running_.store(false);
                break;
            }
        }

        if (active_.load() && now >= next_speed) {
            std::array<int16_t, protocol::xMotorNum> rpm{};
            bool command_stale = false;
            bool should_send = false;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                command_stale = !have_command_update_ ||
                    (now - last_command_update_ > std::chrono::milliseconds(command_watchdog_ms_));
                if (command_stale) {
                    if (!watchdog_latched_) {
                    watchdog_latched_ = true;
                    should_send = true;  // one BRAKE_ALL, not an endless stream of old speeds
                    }
                } else {
                    rpm = target_rpm_;
                    should_send = true;
                }
            }

            if (should_send) {
                bool ok = false;
                if (command_stale) {
                    ok = send_frame(protocol::xCmdBrakeAll, nullptr, 0);
                } else {
                    std::array<uint8_t, protocol::xMotorNum * 2> payload{};
                    for (size_t i = 0; i < protocol::xMotorNum; ++i) {
                    protocol::pack_i16(&payload[2 * i], rpm[i]);
                    }
                    ok = send_frame(
                    protocol::xCmdSetAllSpeed, payload.data(),
                    static_cast<uint8_t>(payload.size()));
                }
                if (!ok) {
                    healthy_.store(false);
                    running_.store(false);
                    break;
                }
            }
            next_speed = now + 10ms;
        }

        // The current firmware responds to polling requests; do not block the
        // ros2_control read() call waiting for telemetry.
        if (now >= next_telemetry) {
            if (!send_frame(protocol::xCmdGetTelemetry, nullptr, 0)) {
            healthy_.store(false);
            running_.store(false);
            break;
            }
            next_telemetry = now + 50ms;
        }

        if (now >= next_battery) {
            if (!send_frame(protocol::xCmdGetBattery, nullptr, 0)) {
            healthy_.store(false);
            running_.store(false);
            break;
            }
            next_battery = now + 5s;
        }

        if (now >= next_ping) {
            if (!send_frame(protocol::xCmdPing, nullptr, 0)) {
            healthy_.store(false);
            running_.store(false);
            break;
            }
            next_ping = now + 1s;
        }
    }
}

}   // namespace xbot_serial