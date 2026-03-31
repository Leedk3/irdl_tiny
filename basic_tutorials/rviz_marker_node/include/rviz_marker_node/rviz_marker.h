#ifndef RVIZ_MARKER_H
#define RVIZ_MARKER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>

class RvizMarker : public rclcpp::Node
{
public:
  RvizMarker(const std::string & node_name);
  ~RvizMarker();

private:
  void getParams();
  void TimerCallback();
  visualization_msgs::msg::Marker makeMarker(const geometry_msgs::msg::Pose & pose);

  rclcpp::TimerBase::SharedPtr Timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMarker;

  std::string m_frame_id;
  double m_scale;
};

#endif  // RVIZ_MARKER_H
