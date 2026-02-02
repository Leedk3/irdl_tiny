#include <zeromq_bridge/zeromq_bridge_pub.h>
#include <zeromq_bridge/sdu.hpp>

#include <chrono>

using namespace std::chrono_literals;
#define INFINITE 999999

ZeroMQBridgePub::ZeroMQBridgePub(const std::string &node_name) : Node(node_name), context_(1), publisher_(context_, ZMQ_PUB)
{
  this->Timer = this->create_wall_timer(200ms, std::bind(&ZeroMQBridgePub::TimerCallback, this));
  getParams();

  RCLCPP_INFO(this->get_logger(), "ZMQ publisher bound to: %s", connection_address_.c_str());
  publisher_.bind(connection_address_);

  std::this_thread::sleep_for(std::chrono::seconds(5)); // Give subscriber time to connect

  // this->pubUAMmodel = this->create_publisher<visualization_msgs::msg::Marker>("/uam_model", rclcpp::SensorDataQoS());

  // this->subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
  //     "/Odometry/msfs", rclcpp::SensorDataQoS(), std::bind(&ZeroMQBridgePub::odomCallback, this, std::placeholders::_1));

};
ZeroMQBridgePub::~ZeroMQBridgePub() {};

void ZeroMQBridgePub::getParams()
{
  this->declare_parameter<std::string>("pub_connection_address", "tcp://*:5566");
  this->get_parameter("pub_connection_address", connection_address_);
}

void ZeroMQBridgePub::TimerCallback()
{
  static double _cnt = 0.0;
  V2VB_PL0_MSG_T pl0;

  pl0.payload_type_code = 0;
  pl0.address_qualifier = 0;
  pl0.icao = 0x1ABBA1;
  pl0.security_mode = 0;
  pl0.latitude_wgs84 = _cnt -80.12345678;
  pl0.longitude_wgs84 = _cnt + 127.87654321;
  pl0.true_altitude_ft = 200.1;
  pl0.pressure_altitude_ft = 200.2;
  pl0.absolute_altitude_ft = 200.3;

  std::string topic = "0";
  zmq::message_t topic_msg(topic.c_str(), topic.size());

  zmq::message_t body_msg(sizeof(V2VB_PL0_MSG_T));
  memcpy(body_msg.data(), &pl0, sizeof(V2VB_PL0_MSG_T));

  publisher_.send(topic_msg, ZMQ_SNDMORE);
  publisher_.send(body_msg, 0);

  printf("[PUB] send:\n");
  printf("  payload_type_code    = %d\n", pl0.payload_type_code);
  printf("  address_qualifier    = %d\n", pl0.address_qualifier);
  printf("  icao                 = 0x%06X\n", pl0.icao);
  printf("  security_mode        = %d\n", pl0.security_mode);
  printf("  latitude_wgs84       = %.8f\n", pl0.latitude_wgs84);
  printf("  longitude_wgs84      = %.8f\n", pl0.longitude_wgs84);
  printf("  true_altitude_ft     = %.2f\n", pl0.true_altitude_ft);
  printf("  pressure_altitude_ft = %.2f\n", pl0.pressure_altitude_ft);
  printf("  absolute_altitude_ft = %.2f\n", pl0.absolute_altitude_ft);
  printf("  ...\n");
  printf("--------------------------------------------------\n");
  _cnt = _cnt + 0.1;
}
