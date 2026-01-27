#include "intruder_node/intruder.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

Intruder::Intruder(const rclcpp::NodeOptions & options) 
: Node("intruder", options), gen_(rd_()) {
    // 파라미터 및 퍼블리셔 초기화
    this->declare_parameter("scale_x", 30.0);
    this->declare_parameter("scale_y", 10.0);
    this->declare_parameter("scale_z", 10.0);

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/intruder/gps", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/intruder/marker", 10);
    
    // 타이머는 첫 번째 함수를 호출
    timer_ = this->create_wall_timer(100ms, std::bind(&Intruder::update_and_publish_intruder, this)); 

    base_lat = 37.5268303; base_lon = 126.9271195; base_alt = 10.0;
    current_lat = base_lat; current_lon = base_lon; current_alt = base_alt; 
    vx = 5.0; vy = 5.0; vz = 1.0; 
}

Intruder::~Intruder() {}

// [함수1: 물리 및 시각화]
void Intruder::update_and_publish_intruder() {
    // 물리 계산 로직 (속도 변화, 경계 제한)
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    vx += dist(gen_) * 6.0; vy += dist(gen_) * 6.0; vz += dist(gen_) * 2.0;
    
    const double lat_const = 111319.9;
    const double lon_const = 111319.9 * std::cos(current_lat * M_PI / 180.0);

    current_lat += (vy * 0.1 / lat_const);
    current_lon += (vx * 0.1 / lon_const);
    current_alt += (vz * 0.1);

    if (std::abs(current_lat - base_lat) > 0.045) vy *= -1.1;
    if (std::abs(current_lon - base_lon) > 0.063) vx *= -1.1;
    if (current_alt < base_alt) {
        current_alt = base_alt;
        vz *= -1.1;
    }

    double utm_x = (current_lon - base_lon) * lon_const;
    double utm_y = (current_lat - base_lat) * lat_const;

    // 화살표 궤적 발행 
    double yaw = std::atan2(vy, vx);
    double pitch = std::atan2(-vz, std::sqrt(vx*vx + vy*vy));

    int trail_id = (marker_id_cnt % max_trail_size) + 1;
    marker_pub_->publish(createArrowMarker(utm_x, utm_y, current_alt, yaw, pitch, trail_id, 0.3));
    marker_id_cnt++;
    marker_pub_->publish(createArrowMarker(utm_x, utm_y, current_alt, yaw, pitch, 0, 1.0));

    //gps값 발행
    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.frame_id = "gps_link";
    gps_msg.header.stamp = this->now();

    gps_msg.latitude = current_lat;
    gps_msg.longitude = current_lon;
    gps_msg.altitude = current_alt;

    publisher_->publish(gps_msg); 

    // 함수2 호출 
    report_status();
}

// [함수2: 데이터 브리핑]
void Intruder::report_status() {
    // UTM 좌표 재계산 (상수 재정의 피하기 위해 update_position에서 인자로 넘겨받도록 설계할 수도 있음)
    const double lat_const = 111319.9;
    const double lon_const = 111319.9 * std::cos(current_lat * M_PI / 180.0);
    double utm_x = (current_lon - base_lon) * lon_const;
    double utm_y = (current_lat - base_lat) * lat_const;

    // 터미널에 GPS와 UTM 정보를 깔끔하게 출력
    RCLCPP_INFO(this->get_logger(), 
        "\n============[REPORT] ==================\n"
        "  GPS -> Lat: %.7f, Lon: %.7f, Alt: %.1f\n"
        "  UTM -> X: %.2f, Y: %.2f\n"
        "========================================", 
        current_lat, current_lon, current_alt, utm_x, utm_y);
}

// [유틸리티 함수: 마커 생성]
visualization_msgs::msg::Marker Intruder::createArrowMarker(double x, double y, double z, double yaw, double pitch, int id, double alpha) {
    visualization_msgs::msg::Marker m;
    //마커 설정 로직(createArrowMarker 내용 동일)
    m.header.frame_id = "map"; m.header.stamp = this->now();
    m.id = id; m.type = visualization_msgs::msg::Marker::ARROW;
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
    tf2::Quaternion q; q.setRPY(0, pitch, yaw);
    m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); 
    m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
    m.scale.x = 30.0; m.scale.y = 10.0; m.scale.z = 10.0;
    m.color.r = 1.0; m.color.a = alpha;
    return m;
}