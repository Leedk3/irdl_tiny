#include <zeromq_bridge/zeromq_bridge_sub.h>
#include <zeromq_bridge/sdu.hpp>

#include <chrono>

using namespace std::chrono_literals;
#define INFINITE 999999

ZeroMQBridgeSub::ZeroMQBridgeSub(const std::string &node_name) : Node(node_name), context_(2),
                                                                 subscriber_(context_, zmq::socket_type::sub)
{
  getParams();
  this->Timer = this->create_wall_timer(200ms, std::bind(&ZeroMQBridgeSub::TimerCallback, this));
  std::cout << connection_address_ << std::endl;
  RCLCPP_INFO(this->get_logger(), "Connecting to ZMQ address: %s", connection_address_.c_str());
  subscriber_.connect(connection_address_);
  // subscriber_.connect("tcp://192.168.1.1:5566");
  // subscriber_.connect("tcp://localhost:5566");

  std::this_thread::sleep_for(std::chrono::seconds(5)); // Give subscriber time to connect

  subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all topics

  // this->pubUAMmodel = this->create_publisher<visualization_msgs::msg::Marker>("/uam_model", rclcpp::SensorDataQoS());

  // this->subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
  //     "/Odometry/msfs", rclcpp::SensorDataQoS(), std::bind(&ZeroMQBridgeSub::odomCallback, this, std::placeholders::_1));

};
ZeroMQBridgeSub::~ZeroMQBridgeSub() {};

void ZeroMQBridgeSub::getParams()
{
  this->declare_parameter<std::string>("sub_connection_address", "tcp://localhost:5566");
  this->get_parameter("sub_connection_address", connection_address_);

  // this->declare_parameter<double>("example_param", double(0.0));
  // this->m_example_param = this->get_parameter("example_param").as_double();
}

void ZeroMQBridgeSub::TimerCallback()
{
  zmq::message_t topic;
  zmq::message_t msg;

  bool got_topic = subscriber_.recv(&topic, ZMQ_DONTWAIT);

  if (!got_topic)
    return;

  std::string topic_msg(static_cast<char *>(topic.data()), topic.size());

  bool got_msg = subscriber_.recv(&msg, ZMQ_DONTWAIT);

  if (!got_msg || msg.size() != sizeof(V2VB_PL0_MSG_T))
    return;

  if (topic_msg != "0")
    return;

  V2VB_PL0_MSG_T received;
  memcpy(&received, msg.data(), sizeof(V2VB_PL0_MSG_T));

  printf("[SUB] Received:\n");
  printf("  topic                = %s\n", topic_msg.c_str());
  printf("  payload_type_code    = %d\n", received.payload_type_code);
  printf("  address_qualifier    = %d\n", received.address_qualifier);
  printf("  icao                 = 0x%06X\n", received.icao);
  printf("  security_mode        = %d\n", received.security_mode);
  printf("  latitude_wgs84       = %.8f\n", received.latitude_wgs84);
  printf("  longitude_wgs84      = %.8f\n", received.longitude_wgs84);
  printf("  true_altitude_ft     = %.2f\n", received.true_altitude_ft);
  printf("  pressure_altitude_ft = %.2f\n", received.pressure_altitude_ft);
  printf("  absolute_altitude_ft = %.2f\n", received.absolute_altitude_ft);
  printf("  ...\n");
  printf("--------------------------------------------------\n");
}
