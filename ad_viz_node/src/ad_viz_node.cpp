#include "ad_viz_node/ad_viz.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdViz>("ad_viz_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
