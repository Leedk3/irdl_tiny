#ifndef FAKE_GPS_PUB_H
#define FAKE_GPS_PUB_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <random>
#include <chrono>

class FakeGpsPub : public rclcpp::Node {
public:
    FakeGpsPub(const std::string &node_name);
    virtual ~FakeGpsPub();

private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;

    double base_lat_, base_lon_, base_alt_;
    std::mt19937 gen_; // 난수 생성기
};

#endif // FAKE_GPS_PUB_H