#ifndef OWNSHIP_H_
#define OWNSHIP_H_

#include <random>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

class Ownship : public rclcpp::Node {
public:
    explicit Ownship(const rclcpp::NodeOptions & options);
    virtual ~Ownship();

private:
    // [함수1] 물리 계산 및 1000개 화살표 궤적 발행
    void update_and_publish_ownship(); 

    // [함수2] 데이터 브리핑 (GPS 및 UTM 터미널 출력)
    void report_status(); 

    // 마커 생성 유틸리티
    visualization_msgs::msg::Marker createArrowMarker(double x, double y, double z, double yaw, double pitch, int id, double alpha);

    // ROS2 인터페이스 및 변수들
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
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