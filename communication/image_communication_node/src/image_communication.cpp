#include <image_communication/image_communication.h>
#include <chrono>
#include <iomanip> // setprecision을 사용하기 위해 필요
// #include "geodetic_converter/geodetic_converter.h"

using namespace std::chrono_literals;
#define INFINITE 999999

ImageComm::ImageComm(const std::string &node_name) : Node(node_name)
{
  // this->recvTimer = this->create_wall_timer(10ms, std::bind(&ImageComm::recvTimerCallback, this));
  // this->sendTimer = this->create_wall_timer(10ms, std::bind(&ImageComm::sendTimerCallback, this));

  this->ImageTimer = this->create_wall_timer(50ms, std::bind(&ImageComm::ImageTimerCallback, this));
  this->image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/Image/top_challenge", ai3ct::common::constants::QOS_SENSOR_DATA);

  // this->sub_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
  //     "/aam/cmd_vel", ai3ct::common::constants::QOS_CONTROL_CMD_OVERRIDE, std::bind(&ImageComm::cmdCallback, this, std::placeholders::_1));

  this->getParams();

  m_UDP_Receiver = std::make_shared<ImageReceiver>();
  // m_UDP_Sender = std::make_shared<UDPSender>();

  // UDP 소켓 초기화
  bool ok = false;

  // if(m_use_cmd)
  //   ok = m_UDP_Sender->initSocket(m_ip, m_port);

  // if (!ok)
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to init UDP Sender socket");
  //   if(!m_use_cmd) RCLCPP_WARN(this->get_logger(), "use_cmd parameter is not activated.");
  // }
  // else
  // {
  //   RCLCPP_INFO(this->get_logger(), "Successfully initialized UDP Sender socket");
  // }
};

ImageComm::~ImageComm()
{
  m_UDP_Receiver->~ImageReceiver();
};

void ImageComm::getParams()
{

}


void ImageComm::ImageTimerCallback() {
    cv::Mat image;
    std::string windowName;

    if (m_UDP_Receiver->receiveUdpImageOnce(image)) {
        // 수신된 이미지를 ROS2 토픽으로 퍼블리시
        sensor_msgs::msg::Image rosImage;
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg(rosImage);

        image_publisher_->publish(rosImage);

        RCLCPP_WARN(this->get_logger(), "Received image for window: %s", windowName.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to receive image data");
    }
}
