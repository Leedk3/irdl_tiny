#ifndef EXAMPLE_CPP_H
#define EXAMPLE_CPP_H

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

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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

// #define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
// #undef BOOST_NO_CXX11_SCOPED_ENUMS

// #include <opencv2/viz/types.hpp>

class ExampleCPP : public rclcpp::Node{
public:
    ExampleCPP(const std::string &node_name);
    ~ExampleCPP();

private:

    void getParams();
    void TimerCallback();

private:
    // ros::Subscriber object_cluster_sub;
    rclcpp::TimerBase::SharedPtr Timer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    visualization_msgs::msg::Marker MeshMarker(const geometry_msgs::msg::Pose& pose_in);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubUAMmodel;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    double m_dt = 0.0;
    nav_msgs::msg::Odometry m_odom;
    double m_example_param;
};

#endif //EXAMPLE_CPP_H