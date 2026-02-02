#include "zeromq_bridge/zeromq_bridge_sub.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZeroMQBridgeSub>("zeromq_bridge_subscriber_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}