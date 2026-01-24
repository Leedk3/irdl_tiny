#include "radar_node/radar.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

Radar::Radar(const rclcpp::NodeOptions & options) : Node("radar", options) {
    radar_lat = 37.5268303; radar_lon = 126.9271195; radar_alt = 10.0;
    
    intruder_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/intruder/gps", 10, std::bind(&Radar::intruder_callback, this, std::placeholders::_1));
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/radar/visuals", 10);

    // 0.1초마다 타이머를 돌려 빔을 회전시킵니다 (1초에 한바퀴)
    timer_ = this->create_wall_timer(100ms, std::bind(&Radar::perform_scan, this));
}

void Radar::perform_scan() {
    // 1. 스캔 빔 회전 (0.1초마다 36도씩 이동 -> 1초에 360도)
    current_angle_ += 36.0;
    if (current_angle_ >= 360.0) current_angle_ -= 360.0;

    // 빔 시각화 (현재 각도에 맞춰 부채꼴 빔 발행)
    publish_radar_beam();

    if (!has_gps_data_) return;

    // 2. 인트루더와의 상대 위치 계산
    const double lat_const = 111319.9;
    const double lon_const = 111319.9 * std::cos(radar_lat * M_PI / 180.0);
    double dx = (current_intruder_gps_.longitude - radar_lon) * lon_const;
    double dy = (current_intruder_gps_.latitude - radar_lat) * lat_const;
    double distance = std::sqrt(dx*dx + dy*dy);

    // 3. 물리적 스캔 판정 (거리 AND 각도)
    double target_angle = std::atan2(dy, dx) * 180.0 / M_PI;
    if (target_angle < 0) target_angle += 360.0;

    // 현재 빔의 각도 범위 내에 인트루더가 있는지 확인
    double angle_diff = std::abs(current_angle_ - target_angle);
    if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;

    if (distance <= detection_range && angle_diff <= (beam_width_ / 2.0)) {
        RCLCPP_WARN(this->get_logger(), "!!! TARGET DETECTED BY SCAN BEAM !!! Dist: %.1fm", distance);
        publish_detection_point(dx, dy, current_intruder_gps_.altitude);
    }
}

// 회전하는 레이더 빔 (삼각형/피라미드 형태)
void Radar::publish_radar_beam() {
    visualization_msgs::msg::Marker beam;
    beam.header.frame_id = "map";
    beam.header.stamp = this->now();
    beam.ns = "scan_beam";
    beam.id = 1;
    beam.type = visualization_msgs::msg::Marker::CYLINDER; // 실린더를 얇게 펴서 빔처럼 표현
    beam.action = visualization_msgs::msg::Marker::ADD;

    // 빔의 중심 위치 (레이더 원점에서 거리의 절반만큼 앞)
    double rad = current_angle_ * M_PI / 180.0;
    beam.pose.position.x = (detection_range / 2.0) * std::cos(rad);
    beam.pose.position.y = (detection_range / 2.0) * std::sin(rad);
    beam.pose.position.z = 0.0;

    // 빔의 방향 설정
    tf2::Quaternion q;
    q.setRPY(0, 1.57, rad); // 옆으로 눕혀서 회전
    beam.pose.orientation = tf2::toMsg(q);

    // 빔의 크기 (길이는 탐지거리, 너비는 아주 얇게)
    beam.scale.x = 2.0;               // 빔의 두께
    beam.scale.y = detection_range;   // 빔의 길이
    beam.scale.z = 2.0;               // 빔의 높이

    beam.color.r = 0.0; beam.color.g = 1.0; beam.color.b = 0.0;
    beam.color.a = 0.6; // 선명한 초록색 빔
    marker_pub_->publish(beam);
}

void Radar::publish_detection_point(double x, double y, double z) {
    visualization_msgs::msg::Marker point;
    point.header.frame_id = "map";
    point.header.stamp = this->now();
    point.ns = "detections";
    point.id = ++detection_id_cnt; // 매 감지마다 고유 ID (그래야 점이 여러개 찍힘)
    point.type = visualization_msgs::msg::Marker::SPHERE;
    point.action = visualization_msgs::msg::Marker::ADD;

    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = z;
    point.pose.orientation.w = 1.0;

    point.scale.x = 20.0; point.scale.y = 20.0; point.scale.z = 20.0;
    point.color.r = 1.0; point.color.g = 0.0; point.color.b = 0.0; // 감지된 점은 빨간색
    point.color.a = 1.0;

    point.lifetime = rclcpp::Duration(10s); // ★ 10초 유지 후 삭제
    marker_pub_->publish(point);
}