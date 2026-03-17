#ifndef TALKER_H
#define TALKER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class Talker : public rclcpp::Node
{
public:
  Talker(const std::string & node_name);
  ~Talker();

private:
  void getParams();
  void TimerCallback();

  rclcpp::TimerBase::SharedPtr Timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubChatter;

  std::string m_message;
  int m_count;
};

#endif  // TALKER_H
