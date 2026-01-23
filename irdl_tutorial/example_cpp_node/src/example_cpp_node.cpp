#include "example_cpp_node/example_cpp.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleCPP>("example_cpp_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}