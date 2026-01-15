#ifndef GPS_TO_UTM_H
#define GPS_TO_UTM_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <GeographicLib/UTMUPS.hpp>

class GpsToUtm : public rclcpp::Node {
public:
    GpsToUtm(const std::string &node_name);
    virtual ~GpsToUtm();

private:
    // /gps/fix 토픽을 구독하는 콜백
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr utm_pub_;
};

#endif // GPS_TO_UTM_H