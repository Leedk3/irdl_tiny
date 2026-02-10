#include "image_communication/image_communication.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageComm>("image_communication_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}