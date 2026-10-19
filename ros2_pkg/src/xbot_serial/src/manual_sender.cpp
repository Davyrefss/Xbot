/**
 * @file manual_sender.cpp
 * @brief 手动发帧调试节点: 键盘输入文本命令 → 组帧发送, 收到的帧解析打印
 *
 * 用法:
 *   ros2 run xbot_serial manual_sender --ros-args -p port:=/dev/ttyUSB0
 *
 * 命令(终端输入, 回车发送):
 *   ping                             心跳/探测
 *   speed <id> <rpm>                 单电机调速, id=0..3
 *   speed_all <r1> <r2> <r3> <r4>     四电机调速(转速 RPM)
 *   brake <id>                       单电机刹车, id=0..3
 *   brake_all                        全部刹车
 *   duty <id> <duty>                 单电机开环占空比(-1000~1000), 调试用
 *   ctrl <id> <kp> <ki> <ff> <ff_off> <stiction> <slew> <fade>
 *                                    控制参数, id=0..3 或 255(广播)
 *   telemetry                        请求一次遥测
 *   battery                          请求一次电池信息
 *   help                             打印本命令表
 *   quit                             退出
 */

#include <rclcpp/rclcpp.hpp>
#include "xbot_serial/protocol.hpp"
#include "xbot_serial/serial_port.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

namespace xbot_serial {

class ManualSender : public rclcpp::Node {
public:
    ManualSender() : Node("manual_sender") {
        /* 声明节点参数 */
        declare_parameter<std::string>("port", "/dev/ttyUSB0"); /* 串口设备名 */
        declare_parameter<int>("baud_rate", 115200);   /* 波特率, 与下位机 USART1 一致 */
        declare_parameter<int>("timeout_read", 100);   /* 读取超时时间, 单位 ms */

        /* 获取参数值 */
        std::string port = get_parameter("port").as_string();
        int32_t baud_rate = get_parameter("baud_rate").as_int();
        int32_t timeout_read = get_parameter("timeout_read").as_int();

        /* 打开串口(BaudToSpeed 对不支持的波特率会抛异常, 捕获后按失败处理) */
        bool opened = false;
        try {
            opened = serial_.open(port, static_cast<uint32_t>(baud_rate));
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
        if (!opened) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port.c_str());
            rclcpp::shutdown();
            return;
        }
        serial_.SetTimeoutRead(timeout_read);
        RCLCPP_INFO(get_logger(), "Serial port opened: %s at %d baud, "
                    "type 'help' for commands", port.c_str(), baud_rate);

        /* 接收线程 + 键盘线程 */
        running_ = true;
        rx_thread_ = std::thread(&ManualSender::receiver_loop, this);
        kb_thread_ = std::thread(&ManualSender::keyboard_loop, this);
    }

    ~ManualSender() override {
        running_ = false;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (kb_thread_.joinable()) {
            kb_thread_.join();
        }
        serial_.close();
    }

private:
    /* ================= 发送 ================= */

    /* 组帧并发送, 帧序号自增; 锁覆盖 seq 自增+组帧+写入整体 */
    void send_frame(uint8_t cmd, const uint8_t *data, uint8_t len) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        uint8_t buf[protocol::kMaxFrame];
        size_t n = protocol::build_frame(buf, cmd, tx_seq_++, data, len);
        serial_.write(buf, n);

        /* 打印发送的帧字节, 便于与 golden 向量对照 */
        std::string hex;
        char tmp[4];
        for (size_t i = 0; i < n; i++) {
            std::snprintf(tmp, sizeof(tmp), "%02X ", buf[i]);
            hex += tmp;
        }
        RCLCPP_INFO(get_logger(), "[TX] cmd=0x%02X seq=%u: %s", cmd, tx_seq_ - 1,
                    hex.c_str());
    }

    /* ================= 接收 ================= */

    /* 接收线程: 循环读串口, 把收到的字节交给状态机解析 */
    void receiver_loop() {
        uint8_t buffer[256];
        while (running_ && rclcpp::ok()) {
            /* read 内部用 select 实现了超时, 超时返回 0, 不会忙等 */
            int n = serial_.read(buffer, sizeof(buffer));
            uint32_t now_ms = protocol::monotonic_ms();
            if (n <= 0) {
                /* 读超时 -> 帧超时复位(对应下位机 comm_protocol_poll 的 20ms) */
                parser_.poll(now_ms);
                continue;
            }
            for (int i = 0; i < n; i++) {
                if (parser_.feed(buffer[i], now_ms)) {
                    handle_frame(parser_.get_frame());
                }
            }
        }
    }

    /* 处理完整的一帧数据(CRC 已校验通过) */
    void handle_frame(const protocol::Frame &frame) {
        switch (frame.cmd) {
        case protocol::kCmdAck:
            handle_ack(frame);
            break;
        case protocol::kCmdTelemetry:
            handle_telemetry(frame);
            break;
        case protocol::kCmdBattery:
            handle_battery(frame);
            break;
        default:
            RCLCPP_WARN(get_logger(), "[RX] unknown cmd 0x%02X seq=%u, frame dropped",
                        frame.cmd, frame.seq);
            break;
        }
    }

    /* 0x80 ACK: 下位机对每条下行指令的执行结果 */
    void handle_ack(const protocol::Frame &frame) {
        if (frame.len < protocol::kAckLen) {
            RCLCPP_WARN(get_logger(), "[RX] ACK frame too short: %u bytes", frame.len);
            return;
        }
        uint8_t ack_cmd = frame.data[0];
        uint8_t result = frame.data[1];
        const char *desc = (result == protocol::kResultOk) ? "OK"
                         : (result == protocol::kResultErrLen) ? "ERR_LEN"
                         : (result == protocol::kResultErrParam) ? "ERR_PARAM"
                         : "UNKNOWN";
        RCLCPP_INFO(get_logger(), "[RX] ACK cmd=0x%02X seq=%u result=%s",
                    ack_cmd, frame.seq, desc);
    }

    /* 0x81 遥测: 4x 转速(RPM) + 4x 累计位置(编码器计数) */
    void handle_telemetry(const protocol::Frame &frame) {
        if (frame.len < protocol::kTelemetryLen) {
            RCLCPP_WARN(get_logger(), "[RX] TELEMETRY frame too short: %u bytes",
                        frame.len);
            return;
        }
        int16_t rpm[protocol::kMotorNum];
        int32_t pos[protocol::kMotorNum];
        for (uint8_t i = 0; i < protocol::kMotorNum; i++) {
            rpm[i] = protocol::unpack_i16(&frame.data[2 * i]);
            pos[i] = protocol::unpack_i32(&frame.data[8 + 4 * i]);
        }

        /* 遥测帧 20Hz 上报会刷屏, 平时 1Hz 节流;
         * 用户刚敲过 telemetry 命令时无条件打印, 保证立即看到响应 */
        bool armed = telemetry_armed_.exchange(false);
        if (armed) {
            RCLCPP_INFO(get_logger(), "[RX] TELEMETRY rpm=[%d %d %d %d] pos=[%d %d %d %d]",
                        rpm[0], rpm[1], rpm[2], rpm[3],
                        pos[0], pos[1], pos[2], pos[3]);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[RX] TELEMETRY rpm=[%d %d %d %d] pos=[%d %d %d %d]",
                rpm[0], rpm[1], rpm[2], rpm[3], pos[0], pos[1], pos[2], pos[3]);
        }
    }

    /* 0x82 电池: voltage_mv[u16] + soc[u8] + capacity_mah[u16] + flags[u8]
     * 固件用 pack_i16 携带 u16 值(< 32768 位模式相同), 解包后转 u16 显示 */
    void handle_battery(const protocol::Frame &frame) {
        if (frame.len < protocol::kBatteryLen) {
            RCLCPP_WARN(get_logger(), "[RX] BATTERY frame too short: %u bytes",
                        frame.len);
            return;
        }
        uint16_t voltage = static_cast<uint16_t>(protocol::unpack_i16(&frame.data[0]));
        uint8_t soc = frame.data[2];
        uint16_t capacity = static_cast<uint16_t>(protocol::unpack_i16(&frame.data[3]));
        uint8_t flags = frame.data[5];
        bool low = (flags & protocol::kBatteryFlagLow) != 0;
        RCLCPP_INFO(get_logger(), "[RX] BATTERY voltage=%umV soc=%u%% capacity=%umAh%s",
                    voltage, soc, capacity, low ? " LOW_BATTERY" : "");
    }

    /* ================= 键盘输入 ================= */

    /* 键盘线程: poll 轮询 stdin 并自建行缓冲
     * (不用裸 getline: SIGINT 时若阻塞在 read 上可能永久悬挂, 析构 join 卡死) */
    void keyboard_loop() {
        struct pollfd pfd = { STDIN_FILENO, POLLIN, 0 };
        char buf[256];
        std::string line;

        while (running_ && rclcpp::ok()) {
            int ret = poll(&pfd, 1, 200);   /* 200ms 轮询一次, 及时检查退出标志 */
            if (ret <= 0) {
                continue;
            }
            ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
            if (n <= 0) {
                break;   /* EOF(Ctrl+D)或读错误 -> 退出 */
            }
            for (ssize_t i = 0; i < n; i++) {
                if (buf[i] == '\n') {
                    process_line(line);
                    line.clear();
                } else {
                    line.push_back(buf[i]);
                }
            }
        }
        request_shutdown();
    }

    /* 解析一行文本命令并发送对应帧, 参数非法时打印用法不崩溃 */
    void process_line(const std::string &line) {
        std::istringstream iss(line);
        std::string cmd;
        if (!(iss >> cmd)) {
            return;   /* 空行 */
        }

        if (cmd == "help") {
            print_help();
        } else if (cmd == "quit" || cmd == "exit") {
            RCLCPP_INFO(get_logger(), "bye");
            request_shutdown();
        } else if (cmd == "ping") {
            send_frame(protocol::kCmdPing, nullptr, 0);
        } else if (cmd == "speed") {
            int id, rpm;
            if (!(iss >> id >> rpm)) { usage("speed <id> <rpm>"); return; }
            if (!check_id(id)) return;
            uint8_t p[3];
            p[0] = static_cast<uint8_t>(id);
            protocol::pack_i16(&p[1], clamp_i16(rpm));
            send_frame(protocol::kCmdSetSpeed, p, sizeof(p));
        } else if (cmd == "speed_all") {
            int rpm[protocol::kMotorNum];
            bool ok = true;
            for (uint8_t i = 0; i < protocol::kMotorNum && ok; i++) {
                ok = static_cast<bool>(iss >> rpm[i]);
            }
            if (!ok) { usage("speed_all <r1> <r2> <r3> <r4>"); return; }
            uint8_t p[8];
            for (uint8_t i = 0; i < protocol::kMotorNum; i++) {
                protocol::pack_i16(&p[2 * i], clamp_i16(rpm[i]));
            }
            send_frame(protocol::kCmdSetAllSpeed, p, sizeof(p));
        } else if (cmd == "brake") {
            int id;
            if (!(iss >> id)) { usage("brake <id>"); return; }
            if (!check_id(id)) return;
            uint8_t p[1] = { static_cast<uint8_t>(id) };
            send_frame(protocol::kCmdBrake, p, sizeof(p));
        } else if (cmd == "brake_all") {
            send_frame(protocol::kCmdBrakeAll, nullptr, 0);
        } else if (cmd == "duty") {
            int id, duty;
            if (!(iss >> id >> duty)) { usage("duty <id> <duty>"); return; }
            if (!check_id(id)) return;
            if (duty < -1000 || duty > 1000) {
                RCLCPP_WARN(get_logger(), "duty=%d out of -1000~1000, still sent", duty);
            }
            uint8_t p[3];
            p[0] = static_cast<uint8_t>(id);
            protocol::pack_i16(&p[1], clamp_i16(duty));
            send_frame(protocol::kCmdSetDuty, p, sizeof(p));
        } else if (cmd == "ctrl") {
            int id;
            double kp, ki, ff;
            int ff_offset, stiction, slew, fade;
            if (!(iss >> id >> kp >> ki >> ff >> ff_offset >> stiction >> slew >> fade)) {
                usage("ctrl <id> <kp> <ki> <ff> <ff_off> <stiction> <slew> <fade>");
                return;
            }
            if (id != protocol::kCtrlBroadcastId && !check_id(id)) return;
            uint8_t p[protocol::kSetCtrlLen];
            p[0] = static_cast<uint8_t>(id);
            protocol::pack_f32(&p[1], static_cast<float>(kp));
            protocol::pack_f32(&p[5], static_cast<float>(ki));
            protocol::pack_f32(&p[9], static_cast<float>(ff));
            protocol::pack_i16(&p[13], clamp_i16(ff_offset));
            protocol::pack_i16(&p[15], clamp_i16(stiction));
            protocol::pack_i16(&p[17], clamp_i16(slew));
            protocol::pack_i16(&p[19], clamp_i16(fade));
            send_frame(protocol::kCmdSetCtrl, p, sizeof(p));
        } else if (cmd == "telemetry") {
            telemetry_armed_ = true;
            send_frame(protocol::kCmdGetTelemetry, nullptr, 0);
        } else if (cmd == "battery") {
            send_frame(protocol::kCmdGetBattery, nullptr, 0);
        } else {
            RCLCPP_WARN(get_logger(), "unknown command '%s', type 'help'", cmd.c_str());
        }
    }

    /* 电机 ID 校验, 合法返回 true */
    bool check_id(int id) {
        if (id < 0 || id >= protocol::kMotorNum) {
            RCLCPP_WARN(get_logger(), "motor id %d out of 0..%d", id,
                        protocol::kMotorNum - 1);
            return false;
        }
        return true;
    }

    /* int -> int16 限幅 */
    static int16_t clamp_i16(int v) {
        if (v > 32767) return 32767;
        if (v < -32768) return -32768;
        return static_cast<int16_t>(v);
    }

    void usage(const char *u) {
        RCLCPP_WARN(get_logger(), "usage: %s", u);
    }

    void print_help() {
        RCLCPP_INFO(get_logger(),
            "commands:\n"
            "  ping\n"
            "  speed <id> <rpm>\n"
            "  speed_all <r1> <r2> <r3> <r4>\n"
            "  brake <id>\n"
            "  brake_all\n"
            "  duty <id> <duty>\n"
            "  ctrl <id> <kp> <ki> <ff> <ff_off> <stiction> <slew> <fade>\n"
            "  telemetry\n"
            "  battery\n"
            "  help\n"
            "  quit");
    }

    /* 请求退出: 停止线程 + 关闭 rclcpp(线程安全, 幂等) */
    void request_shutdown() {
        running_ = false;
        rclcpp::shutdown();
    }

    /* 串口 */
    SerialPort serial_;
    protocol::ProtocolParser parser_;
    std::mutex write_mutex_;            /* 保护串口写 + 帧序号自增 */
    std::atomic<uint8_t> tx_seq_{0};    /* 发送帧序号, 8 位自动回绕 */
    std::atomic<bool> running_{false};
    std::atomic<bool> telemetry_armed_{false};   /* telemetry 命令后无条件打印下一帧 */

    /* 线程 */
    std::thread rx_thread_;
    std::thread kb_thread_;
};

} /* namespace xbot_serial */

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xbot_serial::ManualSender>();
    if (!rclcpp::ok()) {
        /* 串口打开失败, 构造函数中已打印错误并调用 shutdown */
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);   /* 无订阅/定时器, 仅挂起并处理 SIGINT/quit */
    rclcpp::shutdown();
    return 0;
}
