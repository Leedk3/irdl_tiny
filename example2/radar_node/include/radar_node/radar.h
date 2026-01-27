#ifndef RADAR_H_
#define RADAR_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" 
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <cmath>

// 인트루더의 궤적 정보를 담는 구조체
struct TargetTrace {
    geometry_msgs::msg::Point point;
    rclcpp::Time timestamp;
};

class Radar : public rclcpp::Node {
public:
    explicit Radar(const rclcpp::NodeOptions & options);
    virtual ~Radar();

private:
    // [함수1] intruder 위치 정보 구독
    void intruder_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg); 
    
    // [함수2] radar 스캔 영역 가시화 (타이머 콜백)
    void publish_scanning_beam();
    
    // [함수3] 레이더 영역 및 intruder 감지 시 마커 표시
    void publish_visual_markers();
    
    // [함수4] intruder의 위치 정보 관리 및 로그 출력
    void publish_detection(double tx, double ty, double tz);
    
    // [함수5] 10초 지난 감지 점들 제거 
    void cleanup_target_traces();

    // ROS 2 인터페이스 객체
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; 
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr utm_pos_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

    // 상태 변수
    double current_angle_; // 수평 회전각 (Azimuth)
    std::vector<geometry_msgs::msg::Point> accumulated_beam_points_; // 돔 잔상 좌표 저장소
    std::vector<TargetTrace> target_traces_; // 감지 궤적 저장소

    // 레이더 설정 상수
    static constexpr double RADAR_RADIUS = 3000.0;
    static constexpr double BASE_X = 0.0, BASE_Y = 0.0, BASE_Z = 0.0;
    static constexpr double base_lat = 37.5268303;
    static constexpr double base_lon = 126.9271195;
    static constexpr double base_alt = 10.0;
};

#endif