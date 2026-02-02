#include "data_handler.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataHandler>());
  rclcpp::shutdown();

  return 0;
}
