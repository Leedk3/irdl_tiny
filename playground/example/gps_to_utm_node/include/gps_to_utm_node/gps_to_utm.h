#ifndef GPS_TO_UTM_H
#define GPS_TO_UTM_H

#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <GeographicLib/UTMUPS.hpp>
#include <tf2/LinearMath/Quaternion.h> // tf2::Quaternion 사용을 위해 필요

class GpsToUtm : public rclcpp::Node {
public:
    explicit GpsToUtm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~GpsToUtm();

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr utm_pub_;
    
    // 기존 변수들
    double last_x_;
    double last_y_;
    bool first_run_;

    // [추가된 부분] 현재 방향을 저장할 쿼터니언 변수 선언
    tf2::Quaternion current_q_; 
};

#endif