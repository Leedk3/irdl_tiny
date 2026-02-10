#include "uam_route_planner/uam_route_planner.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutePlanner>("uam_route_planner"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}