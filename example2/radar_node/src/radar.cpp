#include "radar_node/radar.h"
#include <cmath>
#include <algorithm>

using namespace std::placeholders;

Radar::Radar(const rclcpp::NodeOptions & options) 
: Node("radar_node", options), current_angle_(0.0) {
    
    subscriber_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/intruder/gps", 10, std::bind(&Radar::intruder_callback, this, _1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    utm_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/radar/detected_utm", 10);

    // 1초당 스캔 영역 설정 (1초에 얼만큼 실행 할껀지 1000ms(1초)를 360번 실행)
    scan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/360), 
        std::bind(&Radar::publish_scanning_beam, this));
}

// [함수1] intruder 위치 정보 구독
Radar::~Radar() {}
void Radar::intruder_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    const double lat_const = 111319.9;
    const double lon_const = 111319.9 * std::cos(base_lat * M_PI / 180.0);

    //gps값 utm값으로 변환
    double tx = (msg->longitude - base_lon) * lon_const;
    double ty = (msg->latitude - base_lat) * lat_const;
    double tz = msg->altitude - base_alt;
    
    double rel_x = tx - BASE_X;
    double rel_y = ty - BASE_Y;
    double rel_z = tz - BASE_Z;
    
    //intruder intruder간 거리 계산
    double distance = std::sqrt(std::pow(rel_x, 2) + std::pow(rel_y, 2) + std::pow(rel_z, 2));

    if (distance <= RADAR_RADIUS && rel_z >= 0) { // 거리가 반구 영역 안이면
        double target_azimuth = std::atan2(rel_y, rel_x); //x,y 좌표값을 알 받아옴
        if (target_azimuth < 0) target_azimuth += 2.0 * M_PI; //arctan이 음수 값으로 가는거 방지

        double angle_diff = std::abs(target_azimuth - current_angle_);
        if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

        // 세워진 원판(빔)의 두께 범위 내에 있으면 감지
        if (angle_diff < (M_PI)) { 
            publish_detection(tx, ty, tz);
        }
    }
}

void Radar::publish_scanning_beam() {
    cleanup_target_traces();

    visualization_msgs::msg::Marker beam;
    beam.header.frame_id = "map";
    beam.header.stamp = this->now();
    beam.ns = "active_beam_plate";
    beam.id = 10;
    beam.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    beam.action = visualization_msgs::msg::Marker::ADD;
    beam.pose.orientation.w = 1.0;
    beam.scale.x = 1.0; beam.scale.y = 1.0; beam.scale.z = 1.0;
    
    // 현재 회전하는 원판 빔 (밝은 하늘색)
    beam.color.a = 0.7; beam.color.r = 0.0; beam.color.g = 0.8; beam.color.b = 1.0;

    // z축 수직 1/4 원판 생성 (고도 0도 ~ 90도)
    int segments = 30; 
    for (int i = 0; i < segments; ++i) {
        double el1 = (M_PI / 2.0) * i / segments;
        double el2 = (M_PI / 2.0) * (i + 1) / segments;

        geometry_msgs::msg::Point p0, p1, p2;
        p0.x = BASE_X; p0.y = BASE_Y; p0.z = BASE_Z;

        // 수직 원판의 정점들 (Azimuth는 current_angle_로 고정)
        p1.x = BASE_X + RADAR_RADIUS * cos(el1) * cos(current_angle_);
        p1.y = BASE_Y + RADAR_RADIUS * cos(el1) * sin(current_angle_);
        p1.z = BASE_Z + RADAR_RADIUS * sin(el1);

        p2.x = BASE_X + RADAR_RADIUS * cos(el2) * cos(current_angle_);
        p2.y = BASE_Y + RADAR_RADIUS * cos(el2) * sin(current_angle_);
        p2.z = BASE_Z + RADAR_RADIUS * sin(el2);

        beam.points.push_back(p0); beam.points.push_back(p1); beam.points.push_back(p2);
        
        // 잔상 데이터에 누적 (이것이 돔 형태를 만듦)
        accumulated_beam_points_.push_back(p0);
        accumulated_beam_points_.push_back(p1);
        accumulated_beam_points_.push_back(p2);
    }
    marker_pub_->publish(beam);
    publish_visual_markers();

    // 1/4원(90도) 스캔 회전 로직
    current_angle_ += (M_PI / 2.0 / 20.0); 

    if (current_angle_ >= M_PI / 2.0) {
        current_angle_ = 0.0;
        accumulated_beam_points_.clear(); // 초기화
        RCLCPP_INFO(this->get_logger(), "Scan Reset");
    }
}

void Radar::publish_visual_markers() {
    // 1. 중첩되어 쌓이는 영역 (잔상 돔)
    visualization_msgs::msg::Marker dome;
    dome.header.frame_id = "map";
    dome.header.stamp = this->now();
    dome.ns = "accumulated_view";
    dome.id = 20;
    dome.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    dome.color.a = 0.15; // 겹칠수록 진해짐
    dome.color.r = 0.0; dome.color.g = 0.3; dome.color.b = 0.6;
    dome.points = accumulated_beam_points_;
    marker_pub_->publish(dome);

    // 2. 감지된 궤적 (10초 유지)
    visualization_msgs::msg::Marker trace;
    trace.header.frame_id = "map";
    trace.header.stamp = this->now();
    trace.ns = "detections";
    trace.id = 30;
    trace.type = visualization_msgs::msg::Marker::POINTS;
    trace.scale.x = 25.0; trace.scale.y = 25.0; 
    trace.color.a = 1.0; trace.color.r = 1.0; trace.color.g = 1.0; trace.color.b = 0.0; // 노란색 점

    for (const auto& tt : target_traces_) trace.points.push_back(tt.point);
    marker_pub_->publish(trace);
}

void Radar::publish_detection(double tx, double ty, double tz) {
    TargetTrace tt;
    tt.point.x = tx; tt.point.y = ty; tt.point.z = tz;
    tt.timestamp = this->now();
    target_traces_.push_back(tt);

    RCLCPP_WARN(this->get_logger(), "[TARGET SCAN] X: %.2f, Y: %.2f, Z: %.2f", tx, ty, tz);
}

void Radar::cleanup_target_traces() {
    auto now = this->now();
    target_traces_.erase(std::remove_if(target_traces_.begin(), target_traces_.end(),
        [&](const TargetTrace& tt) { return (now - tt.timestamp).seconds() > 10.0; }), target_traces_.end());
}