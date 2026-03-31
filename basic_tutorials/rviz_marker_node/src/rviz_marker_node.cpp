#include "rviz_marker_node/rviz_marker.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RvizMarker>("rviz_marker_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
