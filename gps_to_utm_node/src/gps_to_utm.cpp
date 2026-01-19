#include "gps_to_utm_node/gps_to_utm.h"
#include <tf2/LinearMath/Quaternion.h> 
#include <cmath>

GpsToUtm::GpsToUtm(const rclcpp::NodeOptions & options) 
: Node("gps_to_utm_node", options), last_x_(0.0), last_y_(0.0), first_run_(true) {
    
    // 1. 구독 및 발행 설정
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/random/flight/gps", 10, std::bind(&GpsToUtm::gps_callback, this, std::placeholders::_1));

    utm_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/gps/utm", 10);

    // 2. 초기 방향값 설정 (기본값: 정면)
    current_q_.setRPY(0, 0, 0);

    RCLCPP_INFO(this->get_logger(), "GPS to UTM Converter with Heading Started.");
}

GpsToUtm::~GpsToUtm() {}

void GpsToUtm::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 1. GPS 고정 상태 확인 (Fix가 안 되면 리턴)
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for GPS Fix...");
        return;
    }

    // 기준 좌표 (원점) 설정
    double base_lat = 37.5268303; 
    double base_lon = 126.9271195;
    double utm_x, utm_y, utm_x_origin, utm_y_origin;
    int zone;
    bool northp;

    try {
        // 2. WGS84 -> UTM 좌표 변환
        GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, utm_x, utm_y);
        GeographicLib::UTMUPS::Forward(base_lat, base_lon, zone, northp, utm_x_origin, utm_y_origin);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        // 현재 원점 기준 상대 좌표 (m)
        double curr_x = utm_x - utm_x_origin;
        double curr_y = utm_y - utm_y_origin;

        // 3. 진행 방향(Heading) 계산
        if (!first_run_) {
            double dx = curr_x - last_x_;
            double dy = curr_y - last_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            // 0.1m 이상 이동 시에만 방향 업데이트 (노이즈 방지)
            if (dist > 0.1) { 
                double yaw = std::atan2(dy, dx);
                current_q_.setRPY(0, 0, yaw);
            }
        } else {
            first_run_ = false;
        }

        // 4. Odometry 메시지에 데이터 채우기
        odom_msg.pose.pose.position.x = curr_x;
        odom_msg.pose.pose.position.y = curr_y;
        odom_msg.pose.pose.position.z = msg->altitude;

        odom_msg.pose.pose.orientation.x = current_q_.x();
        odom_msg.pose.pose.orientation.y = current_q_.y();
        odom_msg.pose.pose.orientation.z = current_q_.z();
        odom_msg.pose.pose.orientation.w = current_q_.w();

        // 다음 계산을 위해 현재 위치 저장
        last_x_ = curr_x;
        last_y_ = curr_y;

        // 5. 토픽 발행
        utm_pub_->publish(odom_msg);

        // 6. 백엔드 터미널 실시간 정보 출력
        RCLCPP_INFO(this->get_logger(), 
            "[Live] X: %7.2f | Y: %7.2f | Z: %7.2f",
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        );

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Conversion Error: %s", e.what());
    }
}