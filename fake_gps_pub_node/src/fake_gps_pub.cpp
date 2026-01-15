#include "fake_gps_pub_node/fake_gps_pub.h"

using namespace std::chrono_literals;

FakeGpsPub::FakeGpsPub(const std::string &node_name) : Node(node_name) {
    // 1. GPS 퍼블리셔만 생성 (토픽: /gps/fix)
    gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    // 2. 타이머 설정 (1초마다 GPS 신호 생성)
    timer_ = this->create_wall_timer(1s, std::bind(&FakeGpsPub::timer_callback, this));

    // 3. 기준 위치 설정 (서울시청 근처)
    base_lat_ = 37.5665;
    base_lon_ = 126.9780;
    base_alt_ = 20.0;

    // 난수 생성기 초기화
    std::random_device rd;
    gen_ = std::mt19937(rd());
}

FakeGpsPub::~FakeGpsPub() {}

void FakeGpsPub::timer_callback() {
    auto message = sensor_msgs::msg::NavSatFix();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "gps_link";

    // 랜덤 노이즈 생성 (위도/경도 미세 변화 및 고도 변화)
    std::uniform_real_distribution<> lat_lon_dist(-0.0001, 0.0001);
    std::uniform_real_distribution<> alt_dist(-0.5, 0.5);
    
    message.latitude = base_lat_ + lat_lon_dist(gen_);
    message.longitude = base_lon_ + lat_lon_dist(gen_);
    message.altitude = base_alt_ + alt_dist(gen_);

    // GPS 수신 상태 설정 (정상 FIX 상태)
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    RCLCPP_INFO(this->get_logger(), "Publishing Fake GPS -> Lat: %.6f, Lon: %.6f, Alt: %.2f", 
                message.latitude, message.longitude, message.altitude);
    
    gps_publisher_->publish(message);
}