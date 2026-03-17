#include <ad_viz_node/ad_viz.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

AdViz::AdViz(const std::string & node_name)
: Node(node_name), m_t(0.0)
{
  this->Timer = this->create_wall_timer(50ms, std::bind(&AdViz::timerCallback, this));

  this->pubVehicle = this->create_publisher<visualization_msgs::msg::Marker>("/ad/vehicle", 10);
  this->pubSafety = this->create_publisher<visualization_msgs::msg::Marker>("/ad/safety_circle", 10);
  this->pubObstacles = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ad/obstacles", 10);
  this->pubGlobalPath = this->create_publisher<nav_msgs::msg::Path>("/ad/global_path", 10);

  this->tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Obstacle positions: some close to path (radius=8), some far
  m_obstacles = {
    { 8.5,  1.0},   // near path → will turn red
    {-7.5,  3.0},   // near path → will turn red
    { 1.5, -8.5},   // near path → will turn red
    {14.0,  0.0},   // far → stays green
    {-13.0, 5.0},   // far → stays green
    { 0.0, 14.0},   // far → stays green
    { 5.5,  6.5},   // mid distance → yellow zone
    {-5.0, -6.0},   // mid distance → yellow zone
  };

  getParams();

  // Publish global path once
  pubGlobalPath->publish(makeGlobalPath());
}

AdViz::~AdViz() {}

void AdViz::getParams()
{
  this->declare_parameter<double>("radius", 8.0);
  this->declare_parameter<double>("speed", 0.01);
  this->declare_parameter<double>("danger_dist", 3.0);
  this->declare_parameter<double>("caution_dist", 6.0);

  m_radius = this->get_parameter("radius").as_double();
  m_speed = this->get_parameter("speed").as_double();
  m_danger_dist = this->get_parameter("danger_dist").as_double();
  m_caution_dist = this->get_parameter("caution_dist").as_double();
}

void AdViz::timerCallback()
{
  m_t += m_speed;

  double vx = m_radius * std::cos(m_t);
  double vy = m_radius * std::sin(m_t);
  double yaw = m_t + M_PI / 2.0;  // tangent direction

  pubVehicle->publish(makeVehicleMarker(vx, vy, yaw));
  pubSafety->publish(makeSafetyCircle(vx, vy));
  pubObstacles->publish(makeObstacleMarkers(vx, vy));
  broadcastTF(vx, vy, yaw);
}

visualization_msgs::msg::Marker AdViz::makeVehicleMarker(double x, double y, double yaw)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "vehicle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = 2.5;  // arrow length
  marker.scale.y = 0.8;  // arrow width
  marker.scale.z = 0.8;  // arrow height

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;  // yellow vehicle

  return marker;
}

visualization_msgs::msg::Marker AdViz::makeSafetyCircle(double x, double y)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "safety";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = -0.1;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = m_caution_dist * 2.0;
  marker.scale.y = m_caution_dist * 2.0;
  marker.scale.z = 0.05;

  marker.color.a = 0.15;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;  // transparent cyan

  return marker;
}

visualization_msgs::msg::MarkerArray AdViz::makeObstacleMarkers(double vx, double vy)
{
  visualization_msgs::msg::MarkerArray array;

  for (size_t i = 0; i < m_obstacles.size(); ++i) {
    double ox = m_obstacles[i].first;
    double oy = m_obstacles[i].second;
    double dist = std::sqrt(std::pow(vx - ox, 2) + std::pow(vy - oy, 2));

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = ox;
    marker.pose.position.y = oy;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    marker.color.a = 0.9;

    if (dist < m_danger_dist) {
      // RED: danger
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else if (dist < m_caution_dist) {
      // YELLOW: caution
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      // GREEN: safe
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    // Distance label
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = this->now();
    text.ns = "obstacle_labels";
    text.id = static_cast<int>(i);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = ox;
    text.pose.position.y = oy;
    text.pose.position.z = 2.5;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.8;
    text.color.a = 1.0;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;

    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f m", dist);
    text.text = buf;

    array.markers.push_back(marker);
    array.markers.push_back(text);
  }

  return array;
}

nav_msgs::msg::Path AdViz::makeGlobalPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();

  int N = 360;
  for (int i = 0; i <= N; ++i) {
    double angle = 2.0 * M_PI * i / N;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = this->now();
    ps.pose.position.x = m_radius * std::cos(angle);
    ps.pose.position.y = m_radius * std::sin(angle);
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }

  return path;
}

void AdViz::broadcastTF(double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";

  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tfBroadcaster->sendTransform(tf);
}
