#include <listener_node/listener.h>

Listener::Listener(const std::string & node_name)
: Node(node_name)
{
  this->subChatter = this->create_subscription<std_msgs::msg::String>(
    "/chatter", 10,
    std::bind(&Listener::chatterCallback, this, std::placeholders::_1));
}

Listener::~Listener() {}

void Listener::chatterCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg->data.c_str());
}
