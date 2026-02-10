#include "mjpeg_to_ros/mjpeg_to_ros.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MJPEGToRosPublisher>("mjpeg_to_ros_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}