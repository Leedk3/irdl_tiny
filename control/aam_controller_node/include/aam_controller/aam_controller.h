#ifndef AAM_CONTROLLER_H
#define AAM_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "ai3ct_common/constants.h"
#include "pid_controller.h"

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
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

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

#include <algorithm>
#include <Eigen/Dense>
class AAMController : public rclcpp::Node{
public:
    AAMController(const std::string &node_name);
    ~AAMController();

private:
    void getParams();
    void TimerCallback();

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void targetSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &params);
    void control();
    void getRefControlPose(const int& look_ahead_idx);
    bool getXYFittingAndControlPose(const int &look_ahead_idx);

    void checkLandingMode();

private:
    // ros::Subscriber object_cluster_sub;
    rclcpp::TimerBase::SharedPtr Timer;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subMarkers;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTargetSpeed;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ParamHandler;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubCmdVel;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubControlPose;

    double m_dt = 0.0;
    geometry_msgs::msg::TwistStamped m_manual_cmd_msg;
    geometry_msgs::msg::TwistStamped m_auto_cmd_msg;
    std::vector<geometry_msgs::msg::Point> m_landing_mode_switch_pos;


    nav_msgs::msg::Odometry m_odom;
    nav_msgs::msg::Path m_path;
    geometry_msgs::msg::Pose m_control_pose;
    double m_ego_airspeed;
    double m_ego_vertical_speed;
    double m_count_stable_time = 0;
    double m_stable_time_thres;
    double m_crosstrack_error;
    double m_heading_ref;
    double m_ref_control_dist;
    double m_mode_switch_z_thres;
    double m_mode_switch_speed_thres;
    double m_mode_switch_lat_thres;
    double m_hover_mode_switch_dist_thres;  

    // PID control objects
    std::unique_ptr<PIDController> m_hover_pid_speed;
    std::unique_ptr<PIDController> m_hover_pid_altitude;
    std::unique_ptr<PIDController> m_hover_pid_longi_pos;
    std::unique_ptr<PIDController> m_hover_pid_lat_pos;
    std::unique_ptr<PIDController> m_hover_pid_heading;

    std::unique_ptr<PIDController> m_flight_pid_speed;
    std::unique_ptr<PIDController> m_flight_pid_altitude;
    // std::unique_ptr<PIDController> m_flight_pid_longi_pos;
    std::unique_ptr<PIDController> m_flight_pid_lat_pos;
    std::unique_ptr<PIDController> m_flight_pid_heading;
    std::unique_ptr<PIDController> m_flight_pid_pitch;
    std::unique_ptr<PIDController> m_flight_pid_roll;

    std::unique_ptr<LowPassFilter> m_throttle_low_pass_filter;
    std::unique_ptr<LowPassFilter> m_flight_pitch_low_pass_filter;

    // control 
    int m_look_ahead_idx;
    double m_hover_max_speed;
    double m_hover_to_flight_speed;
    double m_flight_max_speed;

    // mode 
    bool m_auto_mode = false;
    bool m_flight_mode_on = false;
    bool m_ready_to_land = false;

    double m_target_speed;
};

#endif //AAM_CONTROLLER_H