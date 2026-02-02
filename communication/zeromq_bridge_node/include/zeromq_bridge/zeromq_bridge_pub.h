#ifndef ZEROMQ_BRIDGE_PUB_H
#define ZEROMQ_BRIDGE_PUB_H

#include <rclcpp/rclcpp.hpp>
// #include "ai3ct_common/constants.h"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// headers in STL
#include <memory>
#include <cmath>
#include <queue>
#include <vector>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <zmq.hpp>

// OpenCV Libraries
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// #define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
// #undef BOOST_NO_CXX11_SCOPED_ENUMS

// #include <opencv2/viz/types.hpp>

class ZeroMQBridgePub : public rclcpp::Node{
public:
    ZeroMQBridgePub(const std::string &node_name);
    ~ZeroMQBridgePub();

private:

    void getParams();
    void TimerCallback();

private:
    // ros::Subscriber object_cluster_sub;
    rclcpp::TimerBase::SharedPtr Timer;

    zmq::context_t context_;
    zmq::socket_t publisher_;
    std::string connection_address_;

    double m_dt = 0.0;

    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    // visualization_msgs::msg::Marker MeshMarker(const geometry_msgs::msg::Pose& pose_in);
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubUAMmodel;

    // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // nav_msgs::msg::Odometry m_odom;
    // double m_example_param;
};

#endif //ZEROMQ_BRIDGE_PUB_H