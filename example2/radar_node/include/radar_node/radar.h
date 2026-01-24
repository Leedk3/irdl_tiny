#ifndef RADAR_H_
#define RADAR_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <vector>

class Radar : public rclcpp::Node {
public:
    explicit Radar(const rclcpp::NodeOptions & options);
    virtual ~Radar();

private:
    // [함수1] 1Hz 스캔 루프
    void perform_scan();

    // [함수2] 인트루더 위치 업데이트 (백그라운드에서 위치만 갱신)
    void intruder_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // [함수3] 감지된 위치에 점 찍기
    void publish_detection_point(double x, double y, double z);

    // [함수4] 레이더 돔 가시화
    void publish_radar_dome();

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr intruder_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 레이더 좌표 및 설정
    double radar_lat, radar_lon, radar_alt;
    double detection_range = 3000.0; // 3km
    double current_angle_ = 0.0; // 현재 스캔 빔의 각도
    double beam_width_ = 20.0;   // 스캔 빔의 너비 (도)
    
    // 현재 인트루더의 최신 위치 (레이더가 스캔할 때 꺼내 쓰는 '상태')
    sensor_msgs::msg::NavSatFix current_intruder_gps_;
    bool has_gps_data_ = false;

    int detection_id_cnt = 0;
};

#endif