#include "local_planner/local_planner.h"

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>("local_planner_node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}