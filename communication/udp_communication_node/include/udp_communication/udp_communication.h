#ifndef UDP_COMM_H
#define UDP_COMM_H

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

class UDPComm : public rclcpp::Node{
public:
    UDPComm(const std::string &node_name);
    ~UDPComm();

private:

    void getParams();
    void recvTimerCallback();
    void sendTimerCallback();

    void ImageTimerCallback();
    void convertToRosMessage(const Flight_STATE &state);
    void convertToRosMessage(const SW_STATE &state);
    void Convert2Imu(const Flight_STATE &flight_state_in);


private:
    // ros::Subscriber object_cluster_sub;
    rclcpp::TimerBase::SharedPtr recvTimer;
    rclcpp::TimerBase::SharedPtr sendTimer;
    visualization_msgs::msg::Marker MeshMarker(const geometry_msgs::msg::Pose& pose_in);
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubMsfsOdom;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    rclcpp::Publisher<msfs_msgs::msg::FlightState>::SharedPtr pub_flight_state_;
    rclcpp::Publisher<msfs_msgs::msg::SwitchState>::SharedPtr pub_switch_state_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsatfix_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubMsfsImu;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    double m_dt = 0.0;
    nav_msgs::msg::Odometry m_odom;
    geometry_msgs::msg::TwistStamped m_cmd;
    double m_example_param;

    std::shared_ptr<UDPReceiver> m_UDP_Receiver;
    std::shared_ptr<UDPSender> m_UDP_Sender;

    double m_originLat;
    double m_originLon;
    std::string m_ip;
    int m_port;
    bool m_use_cmd;

    void Convert2Odometry(const Flight_STATE& flight_state_in);

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_base_to_gps_broadcaster;

};

#endif //UDP_COMM_H