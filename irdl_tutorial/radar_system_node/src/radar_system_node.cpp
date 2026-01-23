#include "radar_system_node/radar_system.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarSystem>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}