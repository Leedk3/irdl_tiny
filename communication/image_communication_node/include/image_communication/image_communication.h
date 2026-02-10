#ifndef IMAGE_COMM_H
#define IMAGE_COMM_H

#include <rclcpp/rclcpp.hpp>
#include "ai3ct_common/constants.h"
#include "udp_receiver.h"
#include "udp_sender.h"

#include "msfs_msgs/msg/flight_state.hpp"
#include "msfs_msgs/msg/switch_state.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// headers in STL
#include <memory>
#include <cmath>
#include <queue>
#include <vector>

// OpenCV Libraries
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

// #define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
// #undef BOOST_NO_CXX11_SCOPED_ENUMS

// #include <opencv2/viz/types.hpp>

class ImageComm : public rclcpp::Node{
public:
    ImageComm(const std::string &node_name);
    ~ImageComm();

private:

    void getParams();
    void recvTimerCallback();
    void sendTimerCallback();

    void ImageTimerCallback();


private:
    // ros::Subscriber object_cluster_sub;
    rclcpp::TimerBase::SharedPtr recvTimer;
    rclcpp::TimerBase::SharedPtr sendTimer;
    rclcpp::TimerBase::SharedPtr ImageTimer;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubMsfsOdom;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    std::shared_ptr<ImageReceiver> m_UDP_Receiver;



};

#endif //IMAGE_COMM_H