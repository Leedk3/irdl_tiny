#ifndef AD_VIZ_H
#define AD_VIZ_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <vector>
#include <cmath>

class AdViz : public rclcpp::Node
{
public:
  AdViz(const std::string & node_name);
  ~AdViz();

private:
  void getParams();
  void timerCallback();

  // Marker generators
  visualization_msgs::msg::Marker makeVehicleMarker(double x, double y, double yaw);
  visualization_msgs::msg::Marker makeSafetyCircle(double x, double y);
  visualization_msgs::msg::MarkerArray makeObstacleMarkers(double vx, double vy);
  nav_msgs::msg::Path makeGlobalPath();
  void broadcastTF(double x, double y, double yaw);

  // Publishers
  rclcpp::TimerBase::SharedPtr Timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubVehicle;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubSafety;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubObstacles;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGlobalPath;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

  // State
  double m_t;

  // Params
  double m_radius;
  double m_speed;
  double m_danger_dist;
  double m_caution_dist;

  // Obstacle positions (x, y)
  std::vector<std::pair<double, double>> m_obstacles;
};

#endif  // AD_VIZ_H
