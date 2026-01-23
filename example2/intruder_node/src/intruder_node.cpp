#include "intruder_node/intruder.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Intruder>(rclcpp::NodeOptions());
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}