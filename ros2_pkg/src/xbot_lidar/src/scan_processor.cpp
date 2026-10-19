/**
 * @file scan_processor.cpp
 * @author davyrefss
 * @brief 处理激光雷达扫描数据的节点
 * @version 0.1
 * @date 2026-10-1
 * @addtogroup xbot_lidar
 */

/* include files*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <functional>
#include <memory>
#include <limits>

class ScanProcessor : public rclcpp::Node {
public:
    ScanProcessor() : Node("scan_processor") {
        /* 订阅雷达的扫描数据 */
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            10,
            std::bind(&ScanProcessor::on_scan, this, std::placeholders::_1)
        );

        /* 发布一个最近障碍物距离的话题 */
        pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/min_obstacle_distance",
            rclcpp::QoS(10)
        );

        /* 显示节点启动信息 */
        RCLCPP_INFO(this->get_logger(), "ScanProcessor node has been started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;

    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_dist = std::numeric_limits<float>::infinity();
        int valid_count = 0;

        for (float r : msg->ranges) {
            if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
                if (r < min_dist) {
                    min_dist = r;
                }
                valid_count++;
            }
        }
        
        std_msgs::msg::Float32 out;
        out.data = std::isfinite(min_dist) ? min_dist : -1.0f;
        pub_->publish(out);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Total %d points, %d valid, min distance %.3f m",
            static_cast<int>(msg->ranges.size()),
            valid_count,
            static_cast<double>(out.data)
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanProcessor>());
    rclcpp::shutdown();

    return 0;
}
