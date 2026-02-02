#include "zeromq_bridge/zeromq_bridge_pub.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZeroMQBridgePub>("zeromq_bridge_publisher_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}