#ifndef RADAR_H_
#define RADAR_H_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>

struct TargetTrace {
    geometry_msgs::msg::Point point;
    rclcpp::Time timestamp;
};

class Radar : public rclcpp::Node {
public:
    explicit Radar(const rclcpp::NodeOptions & options);
    virtual ~Radar();

private:
    //[함수1] intruder 위치 정보 구독
    void intruder_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg); 
    //[함수2] radar 스캔 영역 가시화
    void publish_scanning_beam();
    
    //[함수3] intruder 감지시 점 표시
    void publish_visual_markers();
    
    //[함수4] intruder의 위치 정보 발행
    void publish_detection(double tx, double ty, double tz);
    
    //[함수5] intruder 감지 점 들 제거 
    void cleanup_target_traces();

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; 
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr utm_pos_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

    double current_angle_; // 수평 회전각 (Azimuth)
    std::vector<geometry_msgs::msg::Point> accumulated_beam_points_;
    std::vector<TargetTrace> target_traces_;

    const double RADAR_RADIUS = 3000.0;
    const double BASE_X = 0.0, BASE_Y = 0.0, BASE_Z = 0.0;
    const double base_lat = 37.5268303; 
    const double base_lon = 126.9271195; 
    const double base_alt = 10.0;
};

#endif