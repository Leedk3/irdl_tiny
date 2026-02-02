#include "radar_node/radar.h"
#include <cmath>
#include <algorithm>

using namespace std::placeholders;

Radar::Radar(const rclcpp::NodeOptions & options) 
: Node("radar_node", options), current_angle_(0.0) {
    
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/intruder/gps", 10, std::bind(&Radar::intruder_callback, this, _1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    utm_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/radar/detected_utm", 10);

    // 1초당 스캔 영역 설정 (1초에 얼만큼 실행 할껀지 1000ms(1초)를 360번 실행)
    scan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/36), //1000/36 1초에 36번 실행 10도씩 회전 
        std::bind(&Radar::publish_scanning_beam, this));
}

// [함수1] intruder 위치 정보 구독
Radar::~Radar() {}
// 메시지에서 UTM 좌표 및 속도 추출
void Radar::intruder_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    //UTM좌표 추출
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;

    //속도 벡터 추출
    latest_vx_ = msg->twist.twist.linear.x;
    latest_vy_ = msg->twist.twist.linear.y;
    latest_vz_ = msg->twist.twist.linear.z;

    //거리 계산
    double dx = tx - BASE_X;
    double dy = ty - BASE_Y;
    double dz = tz - BASE_Z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
if (distance <= RADAR_RADIUS && dz >= 0) {
        double target_azimuth = std::atan2(dy, dx);
        if (target_azimuth < 0) target_azimuth += 2.0 * M_PI;

        double angle_diff = std::abs(target_azimuth - current_angle_);
        if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

        if (angle_diff < (M_PI/18)) {
            publish_detection(tx, ty, tz, latest_vx_, latest_vy_, latest_vz_);
        }
    }
}

//[함수2] 레이더 스캔 영역을 시각화 
void Radar::publish_scanning_beam() {
    
    //레이더 스캔 영역을 담기 위한 토픽
    visualization_msgs::msg::Marker beam;
    beam.header.frame_id = "map";
    beam.header.stamp = this->now();
    beam.ns = "active_beam_plate";
    beam.id = SCAN_HISTORY;
    beam.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; //삼각형의 형태로 1/4원을 그려냄 
    beam.action = visualization_msgs::msg::Marker::ADD; //이 정보를 새로 그리거나 이미 있으면 정보를 업데이트
    
    //마커의 기본 방향 및 크기
    beam.pose.orientation.w = 1.0;
    beam.scale.x = 1.0; beam.scale.y = 1.0; beam.scale.z = 1.0;
    
    // 레이더 색깔 및 투명도
    beam.color.r = 0.0; beam.color.g = 0.8; beam.color.b = 1.0;
    beam.color.a = 0.1; 
    
    // z축 수직 1/4 원판 생성 (고도 0도 ~ 90도)
    int segments = 20;
    double step = (2.0 * M_PI / 36.0); //스캔 영역 10도를 표시함
    double next_angle = std::min(current_angle_ + step, 2.0 * M_PI); //10도 뒤의 다음점들 계산 및 최대 360을 안넘게 제한 

    //여기부터 3D 1/4 원형 조각 그리기 
    for (int i = 0; i < segments; ++i) {  
        double el1 = (M_PI / 2.0) * (double)i / segments; //첫번째 고도 점
        double el2 = (M_PI / 2.0) * (double)(i + 1) / segments; //등분 점중 첫번쨰 다음 점

        geometry_msgs::msg::Point p0, p1a, p2a, p1b, p2b; //0은 중심 a는 밑 원에서 첫번째 b 밑 원에서 10도 뒤의 점
        p0.x = BASE_X; p0.y = BASE_Y; p0.z = BASE_Z;

        // 현재 각도면 정점
        p1a.x = BASE_X + RADAR_RADIUS * cos(el1) * cos(current_angle_);
        p1a.y = BASE_Y + RADAR_RADIUS * cos(el1) * sin(current_angle_);
        p1a.z = BASE_Z + RADAR_RADIUS * sin(el1);
        p2a.x = BASE_X + RADAR_RADIUS * cos(el2) * cos(current_angle_);
        p2a.y = BASE_Y + RADAR_RADIUS * cos(el2) * sin(current_angle_);
        p2a.z = BASE_Z + RADAR_RADIUS * sin(el2);

        // 다음 각도면 정점 (10도 뒤)
        p1b.x = BASE_X + RADAR_RADIUS * cos(el1) * cos(next_angle);
        p1b.y = BASE_Y + RADAR_RADIUS * cos(el1) * sin(next_angle);
        p1b.z = BASE_Z + RADAR_RADIUS * sin(el1);
        p2b.x = BASE_X + RADAR_RADIUS * cos(el2) * cos(next_angle);
        p2b.y = BASE_Y + RADAR_RADIUS * cos(el2) * sin(next_angle);
        p2b.z = BASE_Z + RADAR_RADIUS * sin(el2);

        // 정점 순서를 조절하여 렌더링 면을 올바르게 설정
        auto add_tri = [&](auto a, auto b, auto c) {
            beam.points.push_back(a); beam.points.push_back(b); beam.points.push_back(c);
            accumulated_beam_points_.push_back(a); 
            accumulated_beam_points_.push_back(b); 
            accumulated_beam_points_.push_back(c);
        };

        add_tri(p0, p2a, p1a);  // 시작 벽면
        add_tri(p0, p1b, p2b);  // 종료 벽면
        add_tri(p1a, p2a, p2b); // 바깥 곡면 1
        add_tri(p1a, p2b, p1b); // 바깥 곡면 2
    }
    marker_pub_->publish(beam);
    publish_visual_markers();

    // 회전 및 리셋 로직
    current_angle_ += step;

    if (current_angle_ >= (2.0 * M_PI - 0.01)) {
            current_angle_ = 0.0;
            accumulated_beam_points_.clear(); 
            cleanup_target_traces(); // 궤적 정리
            
            // 리셋된 상태(빈 데이터)를 즉시 RViz에 송신하여 잔상 제거
            publish_visual_markers();
    }
} 

//[함수3] 레이더 영역 및 intruder 감지 시 마커 표시
void Radar::publish_visual_markers() {

    //중첩되어 쌓이는 영역 (잔상 돔)
    visualization_msgs::msg::Marker dome; 
    dome.header.frame_id = "map";
    dome.header.stamp = this->now();
    dome.ns = "accumulated_view";
    dome.id = STACK_RADAR;
    dome.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    
    dome.scale.x = 1.0; 
    dome.scale.y = 1.0; 
    dome.scale.z = 1.0;
    
    dome.color.a = 0.1; 
    dome.color.r = 0.0; dome.color.g = 0.3; dome.color.b = 0.6;
    dome.points = accumulated_beam_points_;
    marker_pub_->publish(dome);
    
    // 감지된 궤적 (10초 유지)
    visualization_msgs::msg::Marker trace;
    trace.header.frame_id = "map";
    trace.header.stamp = this->now();
    trace.ns = "detections";
    trace.id = MARKER_STAMP;
    trace.type = visualization_msgs::msg::Marker::POINTS;
    trace.scale.x = 25.0; trace.scale.y = 25.0; 
    trace.color.a = 1.0; 
    trace.color.r = 1.0; trace.color.g = 1.0; trace.color.b = 0.0; // 노란색 점

    for (const auto& tt : target_traces_) trace.points.push_back(tt.point);
    marker_pub_->publish(trace);
}

//[함수4] intruder의 위치 정보 발행
void Radar::publish_detection(double tx, double ty, double tz, double vx, double vy, double vz) {
    TargetTrace tt;
    tt.point.x = tx; tt.point.y = ty; tt.point.z = tz;
    tt.timestamp = this->now();
    target_traces_.push_back(tt);

    // 거리 및 각도 계산
    double dx = tx - BASE_X; double dy = ty - BASE_Y; double dz = tz - BASE_Z;
    double range = std::sqrt(dx*dx + dy*dy + dz*dz);
    double azimuth_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    // GPS 역변환
    const double lat_const = 111319.9;
    const double lon_const = 111319.9 * std::cos(base_lat * M_PI / 180.0);
    double converted_lat = base_lat + (ty / lat_const);
    double converted_lon = base_lon + (tx / lon_const);

    RCLCPP_INFO(this->get_logger(), 
        "\n================= [RADAR INFO] =================\n"
        "  [GPS] Lat: %.7f, Lon: %.7f, Alt: %.3f\n"
        "  [UTM] X: %.2f, Y: %.2f, Z: %.2f\n"
        "  -----------------------------------------------\n"
        "  - Range:    %.2f m\n"
        "  - Azimuth:  %.2f deg\n"
        "  - Velocity: [%.2f, %.2f, %.2f] (Speed: %.2f m/s)\n"
        "===================================================", 
        converted_lat, converted_lon, tz + base_alt,
        tx, ty, tz, range, azimuth_deg, vx, vy, vz, speed);
}

//[함수5] intruder 감지 점 들 제거
void Radar::cleanup_target_traces() {
    auto now = this->now();
    target_traces_.erase(std::remove_if(target_traces_.begin(), target_traces_.end(),
        [&](const TargetTrace& tt) { return (now - tt.timestamp).seconds() > 60.0; }), target_traces_.end()); //60초 뒤에 점들 제거
}