#include "random_flight_gps_node/random_flight_gps.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomFlightGps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}