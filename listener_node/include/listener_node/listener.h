#ifndef LISTENER_H
#define LISTENER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class Listener : public rclcpp::Node
{
public:
  Listener(const std::string & node_name);
  ~Listener();

private:
  void chatterCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subChatter;
};

#endif  // LISTENER_H
