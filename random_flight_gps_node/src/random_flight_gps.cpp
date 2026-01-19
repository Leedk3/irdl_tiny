// 1. 반드시 본인의 패키지 구조에 맞는 헤더 경로를 포함해야 합니다.
#include "random_flight_gps_node/random_flight_gps.h"
#include <random>
#include <cmath>

using namespace std::chrono_literals;

// 생성자 구현
RandomFlightGps::RandomFlightGps(const rclcpp::NodeOptions & options) 
: Node("random_flight_gps", options), gen_(rd_()) {
    
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/random/flight/gps", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RandomFlightGps::update_position, this));

    // 기준 위치 
    base_lat = 37.5268303; 
    base_lon = 126.9271195;
    base_alt = 10.0;
    
    current_lat = base_lat;
    current_lon = base_lon;
    current_alt = base_alt; 
    
    vx = 5.0; 
    vy = 5.0; 
    vz = 1.0; 

    RCLCPP_INFO(this->get_logger(), "Native GPS Generator Started.");
}

RandomFlightGps::~RandomFlightGps() {}

void RandomFlightGps::update_position() {
    // 1. 가속도 설정 (최대 속도에 빠르게 도달하도록 배율 조정)
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    vx += dist(gen_) * 6.0;  
    vy += dist(gen_) * 6.0;
    vz += dist(gen_) * 2.0;

    // 2. [핵심] 최대 속도를 시속 200km로 제한
    // 200km/h = 약 55.56m/s
    const double max_spd_ms = 55.56;
    vx = std::clamp(vx, -max_spd_ms, max_spd_ms);
    vy = std::clamp(vy, -max_spd_ms, max_spd_ms);
    vz = std::clamp(vz, -15.0, 15.0); // 고도 변화는 안전을 위해 초속 15m 제한

    // 3. 물리 기반 변화량 계산 (dt = 0.1s)
    double dx = vx * 0.1;
    double dy = vy * 0.1;

    const double lat_const = 111319.9;
    const double lon_const = 111319.9 * std::cos(current_lat * M_PI / 180.0);

    // 4. 좌표 업데이트
    current_lat += (dy / lat_const);
    current_lon += (dx / lon_const);
    current_alt += (vz * 0.1);

    // 5. 10km x 10km x 5km 영역 제한
    // 위도 방향 제한 : 약 10km (반경 5km = 약 0.045도)
    if (std::abs(current_lat - base_lat) > 0.045) {
        vy *= -1.1; 
        current_lat = (current_lat > base_lat) ? base_lat + 0.0449 : base_lat - 0.0449;
    }

    // 경도 방향 제한 : 약 10km (반경 5km = 약 0.063도)
    if (std::abs(current_lon - base_lon) > 0.063) {
        vx *= -1.1;
        current_lon = (current_lon > base_lon) ? base_lon + 0.0629 : base_lon - 0.0629;
    }

    // 고도 제한 : 5km (5,000m)
    if (current_alt > 5000.0) {
        vz *= -1.2;
        current_alt = 4980.0;
    }
    if (current_alt < 10.0) {
        vz *= -1.2;
        current_alt = 20.0;
    }

    // 6. ROS2 메시지 발행
    auto msg = sensor_msgs::msg::NavSatFix();
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps_link";
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.latitude = current_lat;
    msg.longitude = current_lon;
    msg.altitude = current_alt;

    publisher_->publish(msg);

    // 7. 결과 백엔드 출력 (시속 km/h 단위로 확인)
    double current_speed_kmh = std::sqrt(vx*vx + vy*vy + vz*vz) * 3.6;
    RCLCPP_INFO(this->get_logger(), 
        "[GPS Out] Lat: %.7f | Lon: %.7f | Alt: %.2f m",
        msg.latitude, msg.longitude, msg.altitude);
}