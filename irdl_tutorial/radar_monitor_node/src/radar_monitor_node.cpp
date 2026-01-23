#include "radar_monitor_node/radar_monitor.h"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RadarMonitor>(); 
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}