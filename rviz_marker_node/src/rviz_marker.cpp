#include <rviz_marker_node/rviz_marker.h>
#include <chrono>

using namespace std::chrono_literals;

RvizMarker::RvizMarker(const std::string & node_name)
: Node(node_name)
{
  this->Timer = this->create_wall_timer(100ms, std::bind(&RvizMarker::TimerCallback, this));
  this->pubMarker = this->create_publisher<visualization_msgs::msg::Marker>(
    "/visualization_marker", 10);

  getParams();
}

RvizMarker::~RvizMarker() {}

void RvizMarker::getParams()
{
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("scale", 1.0);
  this->m_frame_id = this->get_parameter("frame_id").as_string();
  this->m_scale = this->get_parameter("scale").as_double();
}

void RvizMarker::TimerCallback()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  pubMarker->publish(makeMarker(pose));
}

visualization_msgs::msg::Marker RvizMarker::makeMarker(const geometry_msgs::msg::Pose & pose)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = m_frame_id;
  marker.header.stamp = this->now();
  marker.ns = "rviz_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = m_scale;
  marker.scale.y = m_scale;
  marker.scale.z = m_scale;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 1.0;

  return marker;
}
