#ifndef RANDOM_FLIGHT_GPS_H
#define RANDOM_FLIGHT_GPS_H

#include <chrono>
#include <memory>
#include <random>
#include <cmath> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class RandomFlightGps : public rclcpp::Node {
public:
    explicit RandomFlightGps(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~RandomFlightGps();

private:
    void update_position();

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double base_lat, base_lon, base_alt;
    double current_lat, current_lon, current_alt;
    double vx, vy, vz;

    std::random_device rd_;
    std::mt19937 gen_;
};

#endif // RANDOM_FLIGHT_GPS_H