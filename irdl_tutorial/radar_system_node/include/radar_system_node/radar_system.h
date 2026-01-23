#ifndef RADAR_SYSTEM_H
#define RADAR_SYSTEM_H

#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

struct TargetTrace {
    geometry_msgs::msg::Point point;
    rclcpp::Time timestamp;
};

class RadarSystem : public rclcpp::Node {
public:
    explicit RadarSystem(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~RadarSystem();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_laser_marker(double tx, double ty, double tz);
    void publish_scanning_beam();
    void publish_visual_markers();
    void cleanup_target_traces();

    // 빔이 지나간 면(삼각형 정점들)을 담는 벡터
    std::vector<geometry_msgs::msg::Point> accumulated_beam_points_;
    std::vector<TargetTrace> target_traces_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr utm_pos_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

    double current_angle_;
    const double BASE_X = 0.0;
    const double BASE_Y = 0.0;
    const double BASE_Z = 0.0;
    const double RADAR_RADIUS = 3000.0;
    const size_t MAX_POINTS = 1000;
};

#endif