#include "ownship_node/ownship.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ownship>(rclcpp::NodeOptions());
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}