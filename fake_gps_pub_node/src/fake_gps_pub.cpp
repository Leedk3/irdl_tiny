#include <chrono>
#include <functional>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" // GPS 메시지 타입으로 변경

using namespace std::chrono_literals;

class FakeGPSNode : public rclcpp::Node
{
public:
  FakeGPSNode() : Node("fake_gps_node")
  {
    // GPS 전용 토픽 이름으로 발행 (메시지 타입: NavSatFix)
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    
    // 1초에 한 번씩(1Hz) GPS 신호 갱신
    timer_ = this->create_wall_timer(1s, std::bind(&FakeGPSNode::timer_callback, this));

    // 기준 위치 설정 (예: 서울 시청 근처 위도/경도)
    base_lat_ = 37.5665;
    base_lon_ = 126.9780;
    base_alt_ = 20.0;
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::NavSatFix();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "gps_link";

    // 랜덤 노이즈 생성을 위한 설정
    std::random_device rd;
    std::mt19937 gen(rd());
    // 위도/경도의 아주 미세한 변화(약 1~10m 오차 범위) 생성
    std::uniform_real_distribution<> lat_dis(-0.0001, 0.0001);
    std::uniform_real_distribution<> lon_dis(-0.0001, 0.0001);
    std::uniform_real_distribution<> alt_dis(-0.5, 0.5);

    message.latitude = base_lat_ + lat_dis(gen);
    message.longitude = base_lon_ + lon_dis(gen);
    message.altitude = base_alt_ + alt_dis(gen);

    // GPS 수신 상태 설정 (3: 기지국 보정 수준의 정밀도)
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    RCLCPP_INFO(this->get_logger(), "Publishing Fake GPS -> Lat: %.6f, Lon: %.6f, Alt: %.2f", 
                message.latitude, message.longitude, message.altitude);
    
    publisher_->publish(message);
  }

  double base_lat_, base_lon_, base_alt_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeGPSNode>());
  rclcpp::shutdown();
  return 0;
}