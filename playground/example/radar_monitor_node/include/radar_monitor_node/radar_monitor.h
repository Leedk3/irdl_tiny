#ifndef RADAR_MONITOR_H
#define RADAR_MONITOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iomanip>

class RadarMonitor : public rclcpp::Node {
public:
    explicit RadarMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~RadarMonitor();

private:
    void radarCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr utm_sub_;
    
    const double BASE_X = 0.0;
    const double BASE_Y = 0.0;
    const double BASE_Z = 0.0;

    const double base_lat = 37.5268303; 
    const double base_lon = 126.9271195;
    const double base_alt = 10.0;

    const double lat_dist_per_degree = 111132.0;
    double lon_dist_per_degree = 111132.0 * std::cos(base_lat * M_PI / 180.0);
    
};

#endif // RADAR_MONITOR_H