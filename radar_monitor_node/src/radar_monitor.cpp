#include "radar_monitor_node/radar_monitor.h"

RadarMonitor::RadarMonitor(const rclcpp::NodeOptions & options) 
: Node("radar_monitor_node", options) {
    utm_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/radar/detected_utm", 10, 
        std::bind(&RadarMonitor::radarCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Radar Monitor Started. Waiting for target detection...");
}

RadarMonitor::~RadarMonitor() {}

void RadarMonitor::radarCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // 상대 (UTM)
    double rel_x = msg->point.x;
    double rel_y = msg->point.y;
    double rel_z = msg->point.z;

    // 절대 (GPS)
    double target_lat = base_lat + (rel_y / lat_dist_per_degree);
    double target_lon = base_lon + (rel_x / lon_dist_per_degree);
    double target_alt = base_alt + rel_z;

    // 터미널 시각화 출력
    std::cout << "\033[2J\033[1;1H"; // 터미널 화면 갱신 (Clear)
    std::cout << "[RADAR MONITOR SYSTEM]" << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "GPS (절대):  Lattitude: " << target_lat << " | Lontitude: " << target_lon "| Altituce: " << target_alt << std::endl;
    std::cout << "UTM (상대):  X: " << rel_x << " | Y: " << rel_y << " | Z: " << r_z << std::endl;
}