#include "gps_to_utm_node/gps_to_utm.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsToUtm>("gps_to_utm_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}