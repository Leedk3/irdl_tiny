#ifndef RADAR_SYSTEM_H
#define RADAR_SYSTEM_H

#include <deque> 
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class RadarSystem : public rclcpp::Node {
public:
    explicit RadarSystem(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~RadarSystem();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_radar_marker();
    void publish_laser_marker(double tx, double ty, double tz);

    std::deque<geometry_msgs::msg::Point> trajectory_points_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // RViz 마커용 추가 필요
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr utm_pos_pub_; // 절대 좌표 발행용
    
    const double BASE_X = 0.0;
    const double BASE_Y = 0.0;
    const double BASE_Z = 0.0;
    const double RADAR_RADIUS = 3000.0; 
    const size_t MAX_POINTS = 100;
};

#endif // RADAR_SYSTEM_H



