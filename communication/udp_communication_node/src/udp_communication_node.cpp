#include "udp_communication/udp_communication.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPComm>("udp_communication_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}