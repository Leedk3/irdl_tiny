#include "aam_controller/aam_controller.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AAMController>("aam_controller_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}