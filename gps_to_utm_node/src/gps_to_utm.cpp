#include "gps_to_utm_node/gps_to_utm.h"

GpsToUtm::GpsToUtm(const std::string &node_name) : Node(node_name) {
    // 가짜 GPS 노드가 보내는 토픽 구독
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, std::bind(&GpsToUtm::gps_callback, this, std::placeholders::_1));

    // 변환된 UTM 좌표를 발행할 퍼블리셔 (Odometry 타입 사용)
    utm_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/gps/utm", 10);

    RCLCPP_INFO(this->get_logger(), "GPS to UTM Converter Node Started.");
}

GpsToUtm::~GpsToUtm() {}

void GpsToUtm::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 위도, 경도가 유효하지 않으면 무시
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        RCLCPP_WARN(this->get_logger(), "No GPS Fix...");
        return;
    }

    double utm_x, utm_y;
    int zone;
    bool northp;

    try {
        // WGS84(위경도) -> UTM 변환
        GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, utm_x, utm_y);

        double utm_x_origin;
        double utm_y_origin;

        GeographicLib::UTMUPS::Forward(37.5268303, 126.9271195, zone, northp, utm_x_origin, utm_y_origin);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "utm_frame";
        odom_msg.child_frame_id = "";

        // UTM 좌표 대입 (미터 단위)
        odom_msg.pose.pose.position.x = utm_x - utm_x_origin;
        odom_msg.pose.pose.position.y = utm_y - utm_y_origin;
        odom_msg.pose.pose.position.z = msg->altitude;

        utm_pub_->publish(odom_msg);

        RCLCPP_INFO(this->get_logger(), "UTM Converted -> X: %.2f, Y: %.2f, Z: %.2f (Zone: %d%c)", 
                    odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, zone, msg->altitude, (northp ? 'N' : 'S'));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Conversion Error: %s", e.what());
    }
}