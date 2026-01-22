#include "radar_system_node/radar_system.h"
#include <cmath>
#include <chrono>
#include <algorithm>

using namespace std::placeholders;

RadarSystem::RadarSystem(const rclcpp::NodeOptions & options) 
: Node("radar_system_node", options), current_angle_(0.0) {
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/gps/utm", 10, std::bind(&RadarSystem::odom_callback, this, _1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    utm_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/radar/detected_utm", 10);

    // 20Hz 주기로 회전 및 탐지 관리
    scan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        [this]() {
            this->publish_scanning_beam();
            this->cleanup_target_traces();
        });

    RCLCPP_INFO(this->get_logger(), "3D Shell-Accumulating Radar System Online");
}

RadarSystem::~RadarSystem() {}

void RadarSystem::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;

    double rel_x = tx - BASE_X;
    double rel_y = ty - BASE_Y;
    double rel_z = tz - BASE_Z;
    double distance = std::sqrt(std::pow(rel_x, 2) + std::pow(rel_y, 2) + std::pow(rel_z, 2));

    if (distance <= RADAR_RADIUS) {
        double target_azimuth = std::atan2(rel_y, rel_x);
        if (target_azimuth < 0) target_azimuth += 2.0 * M_PI;

        double angle_diff = std::abs(target_azimuth - current_angle_);
        if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

        // 빔 판의 두께 안에 비행체가 있을 때만 탐지 점 생성
        if (angle_diff < (M_PI / 20.0)) { 
            publish_laser_marker(tx, ty, tz);
        }
    }
}

void RadarSystem::publish_scanning_beam() {
    visualization_msgs::msg::Marker beam;
    beam.header.frame_id = "map";
    beam.header.stamp = this->now();
    beam.ns = "active_beam";
    beam.id = 10;
    beam.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    beam.action = visualization_msgs::msg::Marker::ADD;

    beam.pose.position.x = BASE_X;
    beam.pose.position.y = BASE_Y;
    beam.pose.position.z = BASE_Z;
    beam.pose.orientation.w = 1.0; 
    beam.scale.x = 1.0; beam.scale.y = 1.0; beam.scale.z = 1.0;
    
    // 현재 회전하는 빔은 선명한 하늘색
    beam.color.a = 0.6; beam.color.r = 0.0; beam.color.g = 0.8; beam.color.b = 1.0;

    int segments = 25; // 부드러운 호를 위한 분할 수
    double max_el = M_PI / 2.0;

    for (int i = 0; i < segments; ++i) {
        double el1 = max_el * i / segments;
        double el2 = max_el * (i + 1) / segments;

        geometry_msgs::msg::Point p0, p1, p2;
        p0.x = 0; p0.y = 0; p0.z = 0;

        p1.x = RADAR_RADIUS * cos(el1) * cos(current_angle_);
        p1.y = RADAR_RADIUS * cos(el1) * sin(current_angle_);
        p1.z = RADAR_RADIUS * sin(el1);

        p2.x = RADAR_RADIUS * cos(el2) * cos(current_angle_);
        p2.y = RADAR_RADIUS * cos(el2) * sin(current_angle_);
        p2.z = RADAR_RADIUS * sin(el2);

        // 실시간 빔 정점 추가
        beam.points.push_back(p0);
        beam.points.push_back(p1);
        beam.points.push_back(p2);

        // 지나간 자리에 남길 면 정점 누적
        accumulated_beam_points_.push_back(p0);
        accumulated_beam_points_.push_back(p1);
        accumulated_beam_points_.push_back(p2);
    }
    marker_pub_->publish(beam);

    // 잔상 면(반구 모형)과 비행체 궤적 발행
    publish_visual_markers();

    // 360도 회전 시 초기화
    current_angle_ += (2.0 * M_PI / 100.0);
    if (current_angle_ >= 2.0 * M_PI) {
        current_angle_ = 0.0;
        accumulated_beam_points_.clear(); // 파란색 궤적 면 초기화
        RCLCPP_INFO(this->get_logger(), "Full Scan Complete. Resetting Accumulation.");
    }
}

void RadarSystem::publish_visual_markers() {
    // 1. 누적된 파란색 빔 궤적 (반구 모형)
    visualization_msgs::msg::Marker dome;
    dome.header.frame_id = "map";
    dome.header.stamp = this->now();
    dome.ns = "accumulated_dome";
    dome.id = 20;
    dome.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    dome.scale.x = 1.0; dome.scale.y = 1.0; dome.scale.z = 1.0;
    
    // 잔상은 더 투명하고 짙은 파란색으로 설정 (중첩 효과)
    dome.color.a = 0.15; 
    dome.color.r = 0.0; dome.color.g = 0.3; dome.color.b = 0.7;

    dome.points = accumulated_beam_points_;
    marker_pub_->publish(dome);

    // 2. 비행체 탐지 노란색 점 (10초 유지)
    visualization_msgs::msg::Marker trace;
    trace.header.frame_id = "map";
    trace.header.stamp = this->now();
    trace.ns = "target_trace";
    trace.id = 30;
    trace.type = visualization_msgs::msg::Marker::POINTS;
    trace.scale.x = 15.0; trace.scale.y = 15.0;
    trace.color.a = 1.0; trace.color.r = 1.0; trace.color.g = 1.0; trace.color.b = 0.0;

    for (const auto& tt : target_traces_) trace.points.push_back(tt.point);
    marker_pub_->publish(trace);
}

void RadarSystem::publish_laser_marker(double tx, double ty, double tz) {
    TargetTrace tt;
    tt.point.x = tx; tt.point.y = ty; tt.point.z = tz;
    tt.timestamp = this->now();
    target_traces_.push_back(tt);

    auto utm_msg = geometry_msgs::msg::PointStamped();
    utm_msg.header.stamp = this->now();
    utm_msg.header.frame_id = "map";
    utm_msg.point = tt.point;
    utm_pos_pub_->publish(utm_msg);
}

void RadarSystem::cleanup_target_traces() {
    rclcpp::Time now_time = this->now();
    target_traces_.erase(
        std::remove_if(target_traces_.begin(), target_traces_.end(),
            [&now_time](const TargetTrace& tt) {
                return (now_time - tt.timestamp).seconds() > 10.0;
            }),
        target_traces_.end());
}