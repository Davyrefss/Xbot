#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "xbot_serial/protocol.hpp"
#include "xbot_serial/serial_port.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

namespace xbot_serial {

/* 串口协议帧格式与编解码见 xbot_serial/protocol.hpp, 与下位机 comm_protocol.c 一致:
 * ┌──────┬─────┬─────┬─────┬───────────────┬──────┐
 * │ 0xA5 │ LEN │ SEQ │ CMD │ DATA(LEN)     │ CRC8 │
 * └──────┴─────┴─────┴─────┴───────────────┴──────┘
 *
 * 本节点使用:
 *   发送: 0x02 SET_ALL_SPEED 四电机调速, 负载 4x int16 转速 RPM(下位机闭环);
 *         0x04 BRAKE_ALL 全部刹车; 0x00 PING 链路检测;
 *         0x10 GET_TELEMETRY 每 50ms(20Hz) 请求遥测(固件不主动上报, 必须主动请求)
 *   接收: 0x80 ACK; 0x81 TELEMETRY(4x int16 转速 + 4x int32 累计位置); 0x82 BATTERY
 *
 * 底盘: 四轮差速(滑移转向)
 *   电机编号: M1=左前, M2=右前, M3=左后, M4=右后
 *   左轮 = (M1+M3)/2, 右轮 = (M2+M4)/2
 */
class SerialNode : public rclcpp::Node {
public:
    /* 构造函数 ‘:’-->初始化的开始，调用基类构造函数 */
    SerialNode() : Node("xbot_serial_node"){
        /* 声明节点参数--declare_parameter<类型>("参数名", 默认值) */
        declare_parameter<std::string>("port", "/dev/ttyUSB0"); /* 串口设备名 */
        declare_parameter<int>("baud_rate", 115200);    /* 波特率, 与下位机 USART1 一致 */
        declare_parameter<int>("timeout_read", 100);    /* 读取超时时间, 单位 ms */
        declare_parameter<double>("wheel_radius", 0.0325); /* 轮子半径 m, 需按实际测量 */
        declare_parameter<double>("track", 0.20);       /* 左右轮距 m, 需按实际底盘测量 */
        declare_parameter<int>("enc_cpr", 44);          /* 编码器每圈计数, 与下位机 MOTOR_ENC_CPR 一致 */
        declare_parameter<double>("cmd_timeout", 0.5);  /* cmd_vel 超时刹车时间, 单位 s */

        /* 获取参数值--链式调用 */
        std::string port = get_parameter("port").as_string();
        int32_t baud_rate = get_parameter("baud_rate").as_int();
        int32_t timeout_read = get_parameter("timeout_read").as_int();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        track_ = get_parameter("track").as_double();
        enc_cpr_ = get_parameter("enc_cpr").as_int();
        cmd_timeout_ = get_parameter("cmd_timeout").as_double();

        /* 打开串口 */
        if (!serial_port_.open(port, baud_rate)) {
            /* 串口打开失败, 构造函数中已打印错误并调用 shutdown */
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port.c_str());
            rclcpp::shutdown();
            return;
        }
        serial_port_.SetTimeoutRead(timeout_read);
        RCLCPP_INFO(get_logger(), "Serial port opened: %s at %d baud", port.c_str(), baud_rate);

        /* publisher */
        /* this表示“当前对象的指针”, 通过这个指针访问当前对象的成员 */
        /* <nav_msgs::msg::Odometry> 消息模板 */
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        /* subscriber */
        /* 速度指令 */
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&SerialNode::twist_callback, this, std::placeholders::_1)
        );

        /* receiver */
        running_ = true;
        receiver_thread_ = std::thread(&SerialNode::receiver_loop, this);

        /* 遥测请求: 固件只在收到 GET_TELEMETRY 时才回遥测(不主动上报),
         * 按 20Hz 周期请求, 驱动 odom 更新 */
        telemetry_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SerialNode::telemetry_callback, this)
        );

        /* watchdog: 每 100ms 检查一次 cmd_vel 是否超时 */
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialNode::watchdog_callback, this)
        );

        /* 链路检测: 每秒发一次 PING, 并检查下位机是否还在上报数据 */
        last_rx_time_ = this->now();
        link_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialNode::link_check_callback, this)
        );

        RCLCPP_INFO(get_logger(), "SerialNode initialized.");
    }

    /* override 虚函数(用virtual 修饰的函数，允许在派生类中重写，通过指针和引用调用时，实际执行的是派生类的版本)的说明符 */
    /* 普通的函数名必须要求一样，析构函数使用派生类的版本 */
    ~SerialNode() override {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        serial_port_.close();
    }

private:
    /* ================= 速度指令 ================= */

    /* speed command */
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        /* 用于获取当前时间，返回类型为 rclcpp::Time */
        last_cmd_time_ = this->now();
        stopped_ = false;
        send_speed_command(msg->linear.x, msg->angular.z);
    }

    /* 轮速 -> int16 限幅 */
    static int16_t rpm_clamp(double rpm) {
        if (rpm > 32767.0) return 32767;
        if (rpm < -32768.0) return -32768;
        return static_cast<int16_t>(rpm);
    }

    /* 四轮差速反解并发送 SET_ALL_SPEED: vx m/s, wz rad/s
     * 载荷为转速 RPM(下位机 motor_service_set_all_speed 按闭环 RPM 处理) */
    void send_speed_command(double vx, double wz) {
        /* 车体速度 -> 左右轮线速度(m/s) */
        double vl = vx - wz * track_ / 2.0;
        double vr = vx + wz * track_ / 2.0;
        /* 线速度 -> 轮子转速(RPM): rpm = v / (2*pi*R) * 60 */
        double rpm_l = vl / wheel_radius_ * 60.0 / (2.0 * kPi);
        double rpm_r = vr / wheel_radius_ * 60.0 / (2.0 * kPi);

        /* 负载: [M1 左前][M2 右前][M3 左后][M4 右后], 小端 */
        uint8_t payload[8];
        protocol::pack_i16(&payload[0], rpm_clamp(rpm_l));
        protocol::pack_i16(&payload[2], rpm_clamp(rpm_r));
        protocol::pack_i16(&payload[4], rpm_clamp(rpm_l));
        protocol::pack_i16(&payload[6], rpm_clamp(rpm_r));

        send_frame(protocol::kCmdSetAllSpeed, payload, sizeof(payload));
    }

    /* 全部刹车(下位机短路刹车, 比占空比 0 的滑行停车更安全) */
    void send_brake_all() {
        send_frame(protocol::kCmdBrakeAll, nullptr, 0);
    }

    /* 组帧并发送, 帧序号自增 */
    void send_frame(uint8_t cmd, const uint8_t *data, uint8_t len) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        uint8_t buf[protocol::kMaxFrame];
        size_t n = protocol::build_frame(buf, cmd, tx_seq_++, data, len);
        serial_port_.write(buf, n);
    }

    /* ================= 定时器回调 ================= */

    /* 20Hz 请求遥测(固件不主动上报) */
    void telemetry_callback() {
        send_frame(protocol::kCmdGetTelemetry, nullptr, 0);
    }

    /* 看门狗: cmd_vel 超时未收到新指令时主动刹车, 防止失控 */
    void watchdog_callback() {
        if (!stopped_ &&
            (this->now() - last_cmd_time_).seconds() > cmd_timeout_) {
            stopped_ = true;
            RCLCPP_WARN(get_logger(), "cmd_vel timeout, sending BRAKE_ALL");
            send_brake_all();
        }
    }

    /* 链路检测: 每秒发一次 PING(下位机会回 ACK), 并检查遥测是否超时 */
    void link_check_callback() {
        send_frame(protocol::kCmdPing, nullptr, 0);
        if ((this->now() - last_rx_time_).seconds() > 2.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                "No data from MCU for over 2s, check serial connection");
        }
    }

    /* ================= 接收解析 ================= */

    /* 接收线程: 循环读串口, 把收到的字节交给状态机解析 */
    void receiver_loop() {
        uint8_t buffer[256];
        while (running_ && rclcpp::ok()) {
            /* read 内部用 select 实现了超时, 超时返回 0, 不会忙等 */
            int n = serial_port_.read(buffer, sizeof(buffer));
            uint32_t now_ms = protocol::monotonic_ms();
            if (n <= 0) {
                /* 读超时 -> 帧超时复位, 防止残缺帧拼接
                 * (对应下位机 comm_protocol_poll 的 20ms 帧超时) */
                parser_.poll(now_ms);
                continue;
            }
            last_rx_time_ = this->now();
            for (int i = 0; i < n; ++i) {
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
            log_unknown_cmd(frame.cmd);
            break;
        }
    }

    /* 0x80 ACK 应答: 下位机对每条下行指令的执行结果 */
    void handle_ack(const protocol::Frame &frame) {
        if (frame.len < protocol::kAckLen) {
            RCLCPP_WARN(get_logger(), "ACK frame too short: %u bytes", frame.len);
            return;
        }
        uint8_t ack_cmd = frame.data[0];
        uint8_t result = frame.data[1];
        if (result == protocol::kResultOk) {
            return;    /* 成功, 不打印, 避免刷屏 */
        }
        const char *desc = (result == protocol::kResultErrLen) ? "payload length error"
                         : (result == protocol::kResultErrParam) ? "invalid parameter"
                         : "unknown result";
        RCLCPP_WARN(get_logger(), "MCU rejected cmd 0x%02X: %s (%d)",
                    ack_cmd, desc, result);
    }

    /* 0x81 遥测: 4x 转速(RPM) + 4x 累计位置(编码器计数) -> 差速模型航迹推算, 发布里程计 */
    void handle_telemetry(const protocol::Frame &frame) {
        if (frame.len < protocol::kTelemetryLen) {
            RCLCPP_WARN(get_logger(), "TELEMETRY frame too short: %u bytes", frame.len);
            return;
        }
        /* 小端序解析, 与发送端保持一致 */
        int16_t rpm[protocol::kMotorNum];
        int32_t pos[protocol::kMotorNum];
        for (int i = 0; i < protocol::kMotorNum; ++i) {
            rpm[i] = protocol::unpack_i16(&frame.data[2 * i]);
            pos[i] = protocol::unpack_i32(&frame.data[8 + 4 * i]);
        }

        /* 左右各取两轮平均 -> 车体速度 */
        double rpm_l = (rpm[0] + rpm[2]) / 2.0;   /* M1+M3 左轮 */
        double rpm_r = (rpm[1] + rpm[3]) / 2.0;   /* M2+M4 右轮 */
        double vl = rpm_l * kPi / 30.0 * wheel_radius_;   /* rpm -> rad/s -> m/s */
        double vr = rpm_r * kPi / 30.0 * wheel_radius_;
        double vx = (vl + vr) / 2.0;        /* 车体线速度 m/s */
        double wz = (vr - vl) / track_;     /* 车体角速度 rad/s */

        /* 里程计积分(航迹推算): 用编码器累计位置差算位移, 比速度积分更准 */
        if (has_odom_) {
            double dl = ((pos[0] - last_pos_[0]) + (pos[2] - last_pos_[2])) / 2.0
                        / enc_cpr_ * 2.0 * kPi * wheel_radius_;  /* 左轮弧长 m */
            double dr = ((pos[1] - last_pos_[1]) + (pos[3] - last_pos_[3])) / 2.0
                        / enc_cpr_ * 2.0 * kPi * wheel_radius_;  /* 右轮弧长 m */
            double ds = (dl + dr) / 2.0;
            double dth = (dr - dl) / track_;
            /* 中点法: 先转一半角度再平移, 比假设 dt 内航向不变的近似更准 */
            x_ += ds * std::cos(yaw_ + dth / 2.0);
            y_ += ds * std::sin(yaw_ + dth / 2.0);
            yaw_ += dth;
        }
        for (int i = 0; i < protocol::kMotorNum; ++i) {
            last_pos_[i] = pos[i];
        }
        has_odom_ = true;
        /**/
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);   /* 偏航角转四元数 */
        odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = wz;
        odom_pub_->publish(odom);
        /* TODO: 如需在 RViz 中显示机器人, 还需要广播 odom->base_footprint 的 TF 变换 */
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = odom.header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.z = std::sin(yaw_ / 2.0);
        t.transform.rotation.w = std::cos(yaw_ / 2.0);
        tf_broadcaster_->sendTransform(t);
    }

    /* 0x82 电池: voltage_mv[u16] + soc[u8] + capacity_mah[u16] + flags[u8]
     * 固件用 pack_i16 携带 u16 值(< 32768 位模式相同), 解包后转 u16 */
    void handle_battery(const protocol::Frame &frame) {
        if (frame.len < protocol::kBatteryLen) {
            RCLCPP_WARN(get_logger(), "BATTERY frame too short: %u bytes", frame.len);
            return;
        }
        uint16_t voltage = static_cast<uint16_t>(protocol::unpack_i16(&frame.data[0]));
        uint8_t soc = frame.data[2];
        uint16_t capacity = static_cast<uint16_t>(protocol::unpack_i16(&frame.data[3]));
        bool low = (frame.data[5] & protocol::kBatteryFlagLow) != 0;
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
            "Battery: %umV %u%% %umAh%s", voltage, soc, capacity,
            low ? " LOW_BATTERY" : "");
    }

    /* 未知命令ID, 每秒最多打印一次, 避免刷屏 */
    void log_unknown_cmd(uint8_t cmd) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_unknown_log_ > std::chrono::seconds(1)) {
            last_unknown_log_ = now;
            RCLCPP_WARN(get_logger(), "Unknown cmd id 0x%02X, frame dropped", cmd);
        }
    }

    static constexpr double kPi = 3.14159265358979323846;

    /* 串口 */
    SerialPort serial_port_;
    std::mutex serial_mutex_;   /* 保护串口的读写, 防止订阅回调与接收线程同时写 */
    protocol::ProtocolParser parser_;

    /* 发布者 */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* 订阅者 */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    /* 接收线程 */
    std::atomic<bool> running_{false};
    std::thread receiver_thread_;

    /* 遥测请求定时器(固件不主动上报, 必须主动请求) */
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    /* cmd_vel 看门狗 */
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    double cmd_timeout_ = 0.5;      /* 超时时间, 单位 s */
    rclcpp::Time last_cmd_time_;    /* 最近一次收到 cmd_vel 的时间 */
    bool stopped_ = true;           /* 是否已发过刹车指令 */

    /* 链路检测 */
    rclcpp::TimerBase::SharedPtr link_timer_;
    rclcpp::Time last_rx_time_;     /* 最近一次收到下位机数据的时间 */

    /* 底盘参数 */
    double wheel_radius_ = 0.0325;  /* 轮子半径 m */
    double track_ = 0.20;           /* 左右轮距 m */
    int enc_cpr_ = 44;              /* 编码器每圈计数 */

    /* 发送帧序号, 8 位自动回绕, 与下位机一致 */
    uint8_t tx_seq_ = 0;

    /* 里程计状态 */
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    bool has_odom_ = false;
    int32_t last_pos_[protocol::kMotorNum] = {};

    std::chrono::steady_clock::time_point last_unknown_log_{};
};

} // namespace xbot_serial

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xbot_serial::SerialNode>();
    if (!rclcpp::ok()) {
        /* 串口打开失败, 构造函数中已打印错误并调用 shutdown */
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
