#ifndef INTRUDER_H_
#define INTRUDER_H_

#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker.hpp"

class Intruder : public rclcpp::Node {
public:
    explicit Intruder(const rclcpp::NodeOptions & options);
    virtual ~Intruder();

private:
    // [함수1] 물리 계산 및 1000개 화살표 궤적 발행
    void update_and_publish_intruder(); 

    // [함수2] 데이터 브리핑 (GPS 및 UTM 터미널 출력)
    void report_status(); 

    // 마커 생성 유틸리티
    visualization_msgs::msg::Marker createArrowMarker(double x, double y, double z, double yaw, double pitch, int id, double alpha);

    // ROS2 인터페이스 및 변수들
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    double base_lat, base_lon, base_alt;
    double current_lat, current_lon, current_alt;
    double vx, vy, vz;
    int marker_id_cnt = 0;
    const int max_trail_size = 1000;
};

#endif