#include "radar_system_node/radar_system.h"
#include <cmath>

RadarSystem::RadarSystem(const rclcpp::NodeOptions & options) 
: Node("radar_system_node", options) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/gps/utm", 10, std::bind(&RadarSystem::odom_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Radar System Initialized (Range: 3km)");
}

RadarSystem::~RadarSystem() {}

void RadarSystem::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double dx = msg->pose.pose.position.x;
    double dy = msg->pose.pose.position.y;
    double dz = msg->pose.pose.position.z;

    // 3차원 유클리드 거리 계산
    double distance = std::sqrt(std::pow(dx - BASE_X, 2) + std::pow(dy - BASE_Y, 2) + std::pow(dz - BASE_Z, 2));

    // 항상 레이더 망은 띄워둠
    publish_radar_marker();

    // 3km 반경 이내 진입 시 레이저 추적
    if (distance <= RADAR_RADIUS) {
        publish_laser_marker(dx, dy, dz);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "DETECTED: [%.1f m]", distance);
    }
}

void RadarSystem::publish_radar_marker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "radar_dome";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = BASE_X;
    marker.pose.position.y = BASE_Y;
    marker.pose.position.z = BASE_Z;

    // 지름 설정 (반경 3000 * 2)
    marker.scale.x = RADAR_RADIUS * 2.0;
    marker.scale.y = RADAR_RADIUS * 2.0;
    marker.scale.z = RADAR_RADIUS * 2.0;

    marker.color.a = 0.15; // 반투명
    marker.color.r = 0.0;  marker.color.g = 0.3;  marker.color.b = 0.8; // 심해 파란색

    marker_pub_->publish(marker);
}

void RadarSystem::publish_laser_marker(double tx, double ty, double tz) {
    // 현재 위치를 점 리스트에 추가
    geometry_msgs::msg::Point current_p;
    current_p.x = tx; 
    current_p.y = ty; 
    current_p.z = tz;
    
    trajectory_points_.push_back(current_p);

    // 최대 100개 유지 (오래된 점 삭제)
    if (trajectory_points_.size() > MAX_POINTS) {
        trajectory_points_.pop_front();
    }

    // 절대 UTM 좌표 발행 (나중에 다른 노드에서 GPS로 바꿀 용도)
    auto utm_msg = geometry_msgs::msg::PointStamped();
    utm_msg.header.stamp = this->now();
    utm_msg.header.frame_id = "map";
    utm_msg.point = current_p;
    utm_pos_pub_->publish(utm_msg);

    // 백엔드(터미널)에 레이더로 부터 상대 좌표 출력
    RCLCPP_INFO(this->get_logger(), " Target UTM - X: %.2f, Y: %.2f, Z: %.2f", BASE_X - tx, BASE_Y - ty, BASE_Z - tz);

    // RViz2 시각화용 점(POINTS) 마커 생성
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = this->now();
    points_marker.ns = "trajectory_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::msg::Marker::POINTS; // 타입을 POINTS로 변경
    points_marker.action = visualization_msgs::msg::Marker::ADD;

    // 점의 크기 설정 (가로, 세로 크기)
    points_marker.scale.x = 15.0; 
    points_marker.scale.y = 15.0;

    // 점의 색상 (초록색으로 설정하여 레이저와 구분)
    points_marker.color.a = 1.0;
    points_marker.color.r = 0.0;
    points_marker.color.g = 1.0;
    points_marker.color.b = 0.0;

    // 저장된 모든 점들을 마커에 삽입
    for (const auto& p : trajectory_points_) {
        points_marker.points.push_back(p);
    }

    marker_pub_->publish(points_marker);
}