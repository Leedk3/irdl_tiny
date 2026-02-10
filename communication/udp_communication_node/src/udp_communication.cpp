#include <udp_communication/udp_communication.h>
#include <chrono>
#include <iomanip> // setprecision을 사용하기 위해 필요
#include "geodetic_converter/UTM.h"

using namespace std::chrono_literals;
#define INFINITE 999999

UDPComm::UDPComm(const std::string &node_name) : Node(node_name)
{
  this->recvTimer = this->create_wall_timer(10ms, std::bind(&UDPComm::recvTimerCallback, this));
  this->sendTimer = this->create_wall_timer(10ms, std::bind(&UDPComm::sendTimerCallback, this));

  // this->ImageTimer = this->create_wall_timer(50ms, std::bind(&UDPComm::ImageTimerCallback, this));
  // this->image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/Image/msfs", ai3ct::common::constants::QOS_SENSOR_DATA);
  this->pubMsfsOdom = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry/msfs", ai3ct::common::constants::QOS_EGO_ODOMETRY);
  this->pubMsfsImu = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", ai3ct::common::constants::QOS_SENSOR_DATA);

  this->pub_flight_state_ = this->create_publisher<msfs_msgs::msg::FlightState>("/flight_state", ai3ct::common::constants::QOS_SENSOR_DATA);
  this->pub_switch_state_ = this->create_publisher<msfs_msgs::msg::SwitchState>("/switch_state", ai3ct::common::constants::QOS_SENSOR_DATA);
  this->pub_navsatfix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/NavSatFix/msfs", ai3ct::common::constants::QOS_EGO_ODOMETRY);

  this->sub_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/aam/cmd_vel", ai3ct::common::constants::QOS_CONTROL_CMD_OVERRIDE, std::bind(&UDPComm::cmdCallback, this, std::placeholders::_1));

  this->getParams();
  RCLCPP_INFO_ONCE(this->get_logger(), "Origin Lat : %f, Origin Lon : %f", m_originLat, m_originLon);

  m_UDP_Receiver = std::make_shared<UDPReceiver>();
  m_UDP_Sender = std::make_shared<UDPSender>();

  m_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  m_base_to_gps_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // UDP 소켓 초기화
  bool ok = false;

  if(m_use_cmd)
    ok = m_UDP_Sender->initSocket(m_ip, m_port);

  if (!ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to init UDP Sender socket");
    if(!m_use_cmd) RCLCPP_WARN(this->get_logger(), "use_cmd parameter is not activated.");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Successfully initialized UDP Sender socket");
  }
};

UDPComm::~UDPComm()
{
  m_UDP_Receiver->~UDPReceiver();
};

void UDPComm::getParams()
{
  this->declare_parameter<double>("origin_lat", double(0.0));
  this->m_originLat = this->get_parameter("origin_lat").as_double();
  this->declare_parameter<double>("origin_lon", double(0.0));
  this->m_originLon = this->get_parameter("origin_lon").as_double();

  this->declare_parameter<std::string>("target_ip", "0.0.0.0"); // for UDP sender. Because UDP sender is not necessary every time.
  this->m_ip = get_parameter("target_ip").as_string();

  this->declare_parameter<int>("target_port", 50001); // UDP sender
  this->m_port = get_parameter("target_port").as_int();

  this->declare_parameter<bool>("use_cmd", false); // UDP sender
  this->m_use_cmd = get_parameter("use_cmd").as_bool();
}

void UDPComm::cmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  this->m_cmd = *msg;
}

void UDPComm::recvTimerCallback()
{
  static auto prev_time = std::chrono::high_resolution_clock::now();

  // Receiver //
  m_UDP_Receiver->receiveData();
  auto flight_state = m_UDP_Receiver->getFlightState();
  auto switch_state = m_UDP_Receiver->getSwitchState();

  // Debug
  // std::cout << std::fixed << std::setprecision(8)
  //           << "[FLIGHT] LAT=" << flight_state.latitude
  //           << " LON=" << flight_state.longitude
  //           << " ALT=" << flight_state.altitude
  //           << " HEADING=" << flight_state.heading
  //           << " PITCH=" << flight_state.pitch
  //           << " ROLL=" << flight_state.bank
  //           << " AIRSPEED=" << flight_state.airspeed
  //           << " GNDSPD=" << flight_state.groundspeed
  //           << " WIND_VELOCITY=" << flight_state.windvelocity
  //           << " WIND_DIRECTION=" << flight_state.winddirection << std::endl;

  this->Convert2Odometry(flight_state);
  this->convertToRosMessage(flight_state);
  this->convertToRosMessage(switch_state);
  this->Convert2Imu(flight_state);
  // Debug
  auto cur_time = std::chrono::high_resolution_clock::now();
  m_dt = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_time - prev_time).count() / 1e9;
  prev_time = cur_time;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Publishing messages... dt: %f [msec]", m_dt);
}


void UDPComm::sendTimerCallback()
{
  // Sender //
  if (m_use_cmd)
  {
    AVI_STATE tx_data;
	  tx_data.avi_aileron = m_cmd.twist.angular.x;
	  tx_data.avi_elevator = m_cmd.twist.angular.y;
	  tx_data.avi_rudder = m_cmd.twist.angular.z;
	  tx_data.avi_throttle1 = m_cmd.twist.linear.z;
	  tx_data.avi_throttle2 = m_cmd.twist.linear.z;
	  tx_data.avi_throttle3 = m_cmd.twist.linear.z;
	  tx_data.avi_throttle4 = m_cmd.twist.linear.z;

    RCLCPP_INFO(this->get_logger(),
                "Timer TX: (aileron=%.2f, elevator=%.2f, rudder=%.2f, thr=%.2f)",
                tx_data.avi_aileron, tx_data.avi_elevator, tx_data.avi_rudder, tx_data.avi_throttle1);
    // 전송

    int bytes = m_UDP_Sender->sendData(tx_data);

  }
}

// void UDPComm::ImageTimerCallback() {
//     cv::Mat image;
//     std::string windowName;

//     if (m_UDP_Receiver->receiveImageData(image, windowName)) {
//         // 수신된 이미지를 ROS2 토픽으로 퍼블리시
//         sensor_msgs::msg::Image rosImage;
//         cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg(rosImage);

//         image_publisher_->publish(rosImage);

//         RCLCPP_WARN(this->get_logger(), "Received image for window: %s", windowName.c_str());
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Failed to receive image data");
//     }
// }

visualization_msgs::msg::Marker UDPComm::MeshMarker(const geometry_msgs::msg::Pose &pose_in)
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

void UDPComm::Convert2Odometry(const Flight_STATE &flight_state_in)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = "map";
  odom_msg.header.stamp = this->now();

  sensor_msgs::msg::NavSatFix origin_llh;
  origin_llh.latitude = m_originLat;
  origin_llh.longitude = m_originLon;
  geo_converter::UtmProjector UtmProjector(origin_llh);
  // geo_converter::EcefProjector EcefProjector(origin_llh);

  sensor_msgs::msg::NavSatFix gpsTmp;
  gpsTmp.latitude = flight_state_in.latitude;
  gpsTmp.longitude = flight_state_in.longitude;
  gpsTmp.altitude = flight_state_in.altitude;

  gpsTmp.header.stamp = this->now();
  gpsTmp.header.frame_id = "gps_link";

  this->pub_navsatfix_->publish(gpsTmp); 

  geometry_msgs::msg::Pose2D pose2DTmp = UtmProjector.forward(gpsTmp);
  // geometry_msgs::msg::Point pointTmp = EcefProjector.forward(gpsTmp);

  double yaw_bais_deg = 0.0; // m_YawBias;
  double yaw_bias = yaw_bais_deg * M_PI / 180;

  odom_msg.pose.pose.position.x = pose2DTmp.x * cos(yaw_bias) - pose2DTmp.y * sin(yaw_bias);
  odom_msg.pose.pose.position.y = pose2DTmp.x * sin(yaw_bias) + pose2DTmp.y * cos(yaw_bias);
  odom_msg.pose.pose.position.z = flight_state_in.altitude * ai3ct::common::constants::FEET_TO_METER;
 
  // odom_msg.pose.pose.position.x = pointTmp.x * cos(yaw_bias) - pointTmp.y * sin(yaw_bias);
  // odom_msg.pose.pose.position.y = pointTmp.x * sin(yaw_bias) + pointTmp.y * cos(yaw_bias);
  // odom_msg.pose.pose.position.z = flight_state_in.altitude * ai3ct::common::constants::FEET_TO_METER;

  tf2::Quaternion quat;
  double heading = 90.0 - 1.0 * flight_state_in.heading_true; // NED --> ENU
  quat.setRPY(-1.0 * flight_state_in.bank * M_PI / 180.0,  
              flight_state_in.pitch * M_PI / 180.0,  
              heading * M_PI / 180.0);

  // linear_velocity
  odom_msg.twist.twist.linear.x = flight_state_in.airspeed * ai3ct::common::constants::KNOT_TO_MPS;
  odom_msg.twist.twist.linear.y = 0.0; 
  odom_msg.twist.twist.linear.z = flight_state_in.vertical_speed * ai3ct::common::constants::KNOT_TO_MPS;

  // angular_velocity
  odom_msg.twist.twist.angular.x = -1.0 * flight_state_in.heading_vel;
  odom_msg.twist.twist.angular.y = flight_state_in.pitch_vel;
  odom_msg.twist.twist.angular.z = -1.0 * flight_state_in.roll_vel; //-1.0 * flight_state_in.heading_vel;

  odom_msg.pose.pose.orientation = tf2::toMsg(quat);

  this->pubMsfsOdom->publish(odom_msg);

  geometry_msgs::msg::TransformStamped map_to_base_tf{};
  map_to_base_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  map_to_base_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  map_to_base_tf.transform.translation.z = odom_msg.pose.pose.position.z;
  map_to_base_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  map_to_base_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  map_to_base_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  map_to_base_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  map_to_base_tf.header.stamp = this->now();
  map_to_base_tf.header.frame_id = "map";
  map_to_base_tf.child_frame_id = "base_link";
  m_broadcaster->sendTransform(map_to_base_tf);


  geometry_msgs::msg::TransformStamped base_to_gps_tf{};
  base_to_gps_tf.transform.translation.x = 0.0;
  base_to_gps_tf.transform.translation.y = 0.0;
  base_to_gps_tf.transform.translation.z = -1.0 * odom_msg.pose.pose.position.z;
  base_to_gps_tf.transform.rotation.w = 1.0;
  base_to_gps_tf.transform.rotation.x = 0.0;
  base_to_gps_tf.transform.rotation.y = 0.0;
  base_to_gps_tf.transform.rotation.z = 0.0;
  base_to_gps_tf.header.stamp = odom_msg.header.stamp;
  base_to_gps_tf.header.frame_id = "base_link";
  base_to_gps_tf.child_frame_id = "gps_link";
  m_base_to_gps_broadcaster->sendTransform(base_to_gps_tf);
}


void UDPComm::Convert2Imu(const Flight_STATE &flight_state_in)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "map";
  imu_msg.header.stamp = this->now();

  // orientation
  tf2::Quaternion quat;
  double heading = 90.0 - 1.0 * flight_state_in.heading; // NED --> ENU // magnetic field value
  quat.setRPY(-1.0 * flight_state_in.bank * M_PI / 180.0,  
              flight_state_in.pitch * M_PI / 180.0,  
              heading * M_PI / 180.0);
  imu_msg.orientation = tf2::toMsg(quat);

  // angular_velocity
  imu_msg.angular_velocity.x = -1.0 * flight_state_in.roll_vel;
  imu_msg.angular_velocity.y = flight_state_in.pitch_vel;
  imu_msg.angular_velocity.z = -1.0 * flight_state_in.heading_vel;

  // linear_acceleration
  imu_msg.linear_acceleration.x = ai3ct::common::constants::FEET_TO_METER * flight_state_in.altitude_acc; //there is bug. I found the matching between x with altitude_acc
  imu_msg.linear_acceleration.y = ai3ct::common::constants::FEET_TO_METER * -1.0 * flight_state_in.latitude_acc;
  imu_msg.linear_acceleration.z = ai3ct::common::constants::FEET_TO_METER * flight_state_in.longitude_acc;

  this->pubMsfsImu->publish(imu_msg);
}

void UDPComm::convertToRosMessage(const Flight_STATE &state)
{

  msfs_msgs::msg::FlightState flight_state_msg;
  flight_state_msg.header.frame_id = "base_link";
  flight_state_msg.header.stamp = this->now();

  flight_state_msg.title = state.title;
  flight_state_msg.kohlsmann = state.kohlsmann;
  flight_state_msg.altitude = state.altitude;
  flight_state_msg.latitude = state.latitude;
  flight_state_msg.longitude = state.longitude;
  flight_state_msg.pitch = state.pitch;
  flight_state_msg.bank = state.bank;
  flight_state_msg.heading = state.heading;
  flight_state_msg.heading_true = state.heading_true;
  flight_state_msg.velocity_x = state.velocityX;
  flight_state_msg.velocity_y = state.velocityY;
  flight_state_msg.velodity_z = state.velodityZ;
  flight_state_msg.temperature = state.temperature;
  flight_state_msg.airpressure = state.airpressure;
  flight_state_msg.airdensity = state.airdensity;
  flight_state_msg.windvelocity = state.windvelocity;
  flight_state_msg.winddirection = state.winddirection;
  flight_state_msg.windx = state.windx;
  flight_state_msg.windy = state.windy;
  flight_state_msg.windz = state.windz;
  flight_state_msg.airspeed = state.airspeed;
  flight_state_msg.groundspeed = state.groundspeed;
  flight_state_msg.vertical_speed = state.vertical_speed;
  flight_state_msg.latitude_vel = state.latitude_vel;
  flight_state_msg.longitude_vel = state.longitude_vel;
  flight_state_msg.altitude_vel = state.altitude_vel;
  flight_state_msg.latitude_acc = state.latitude_acc;
  flight_state_msg.longitude_acc = state.longitude_acc;
  flight_state_msg.altitude_acc = state.altitude_acc;
  flight_state_msg.pitch_vel = state.pitch_vel;
  flight_state_msg.roll_vel = state.roll_vel;
  flight_state_msg.heading_vel = state.heading_vel;
  flight_state_msg.atc_message = state.atcMessage;

  pub_flight_state_->publish(flight_state_msg);
}

void UDPComm::convertToRosMessage(const SW_STATE &state)
{
  msfs_msgs::msg::SwitchState sw_msg;
  sw_msg.header.frame_id = "base_link";
  sw_msg.header.stamp = this->now();

  sw_msg.title = state.title;
  sw_msg.engine_r = state.engineR;
  sw_msg.engine_l = state.engineL;
  sw_msg.starter = state.starter;
  sw_msg.alternator = state.alternator;
  sw_msg.battery = state.battery;
  sw_msg.avionics = state.avionics;
  sw_msg.parking_brake = state.parking_brake;
  sw_msg.fuel_pump = state.fuel_pump;
  sw_msg.beacon = state.beacon;
  sw_msg.landing = state.landing;
  sw_msg.taxing = state.taxing;
  sw_msg.navigation = state.navigation;
  sw_msg.strobe = state.strobe;
  sw_msg.pitot_heat = state.pitot_heat;
  sw_msg.aileron_pos = state.aileron_pos;
  sw_msg.elevator_pos = state.elevator_pos;
  sw_msg.fuel_tank_selector = state.fuel_tank_selector;
  sw_msg.trim_position = state.trim_position;
  sw_msg.throttle = state.throttle;
  sw_msg.prop = state.prop;
  sw_msg.mixture = state.mixture;
  sw_msg.flaps = state.flaps;
  sw_msg.gear = state.gear;
  sw_msg.left_brake = state.left_brake;
  sw_msg.right_brake = state.right_brake;

  pub_switch_state_->publish(sw_msg);
}