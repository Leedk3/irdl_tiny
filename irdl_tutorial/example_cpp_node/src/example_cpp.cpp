#include <example_cpp_node/example_cpp.h>
#include <chrono>

using namespace std::chrono_literals;
#define INFINITE 999999

ExampleCPP::ExampleCPP(const std::string& node_name) : Node(node_name)
{
  this->Timer = this->create_wall_timer(0.1s, std::bind(&ExampleCPP::TimerCallback, this));
  this->pubUAMmodel = this->create_publisher<visualization_msgs::msg::Marker>("/uam_model", rclcpp::SensorDataQoS());

  this->subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/gps/utm", rclcpp::SensorDataQoS(), std::bind(&ExampleCPP::odomCallback, this, std::placeholders::_1));

  getParams();
};
ExampleCPP::~ExampleCPP(){};

void ExampleCPP::getParams()
{
  this->declare_parameter<double>("example_param", double(0.0));
  this->m_example_param = this->get_parameter("example_param").as_double();
}

void ExampleCPP::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  this->m_odom = *msg;
}

void ExampleCPP::TimerCallback()
{
  static auto prev_time = std::chrono::high_resolution_clock::now();

  auto cur_time = std::chrono::high_resolution_clock::now();
  m_dt = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_time - prev_time).count() / 1e9;
  prev_time = cur_time;

  geometry_msgs::msg::Pose post_buf;
  post_buf.position.x = 0.0;
  post_buf.position.y = 0.0;
  post_buf.position.z = 0.0;
  post_buf.orientation.x = 0.0;
  post_buf.orientation.y = 0.0;
  post_buf.orientation.z = 0.0;
  post_buf.orientation.w = 1.0;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "dt: %f [msec]", m_dt);
}

visualization_msgs::msg::Marker ExampleCPP::MeshMarker(const geometry_msgs::msg::Pose& pose_in)
{
   // Define the marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link"; 
  marker.header.stamp = this->now();
  marker.ns = "test";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose_in;
  marker.scale.x = 1.0;  
  marker.scale.y = 1.0;  
  marker.scale.z = 1.0;  
  marker.color.a = 1.0;  
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

  return marker;
}
