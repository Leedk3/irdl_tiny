#include <talker_node/talker.h>
#include <chrono>

using namespace std::chrono_literals;

Talker::Talker(const std::string & node_name)
: Node(node_name), m_count(0)
{
  this->Timer = this->create_wall_timer(1000ms, std::bind(&Talker::TimerCallback, this));
  this->pubChatter = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

  getParams();
}

Talker::~Talker() {}

void Talker::getParams()
{
  this->declare_parameter<std::string>("message", "Hello, ROS2!");
  this->m_message = this->get_parameter("message").as_string();
}

void Talker::TimerCallback()
{
  std_msgs::msg::String msg;
  msg.data = m_message + " [" + std::to_string(m_count++) + "]";
  pubChatter->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
}
