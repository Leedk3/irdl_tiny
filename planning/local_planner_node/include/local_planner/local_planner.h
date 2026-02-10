#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include "ai3ct_common/constants.h"

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// headers in STL
#include <memory>
#include <cmath>
#include <queue>
#include <vector>
#include <Eigen/Dense>

// OpenCV Libraries
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// #define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
// #undef BOOST_NO_CXX11_SCOPED_ENUMS


class LocalPlanner : public rclcpp::Node{
public:
    LocalPlanner(const std::string &node_name);
    ~LocalPlanner();

private:

    void getParams();
    void TimerCallback();

    // 최근접 포인트 인덱스 찾기 (거리+헤딩 고려)
    int findClosestPointIndex(const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &current_pose);

    // Closest Point 인덱스로부터 일정 거리만큼 sub path 추출
    nav_msgs::msg::Path extractLocalPath(const nav_msgs::msg::Path &global_path, int closest_idx, double forward_distance);

    // Global -> Body frame 변환
    nav_msgs::msg::Path transformPathToBody(const nav_msgs::msg::Path &path_in_global, const geometry_msgs::msg::Pose &current_pose);
    
    // these functions can be moved to utils;
    double shortestAngularDistance(double from, double to);
    double normalize_angle(double angle);


private:
    // ros::Subscriber object_cluster_sub;
    rclcpp::TimerBase::SharedPtr Timer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSpeedProfile;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLocalPathBody;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLocalPathGlobal;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubTargetSpeed;


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void speedProfileCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    double m_dt = 0.0;
    nav_msgs::msg::Odometry m_odom;
    nav_msgs::msg::Path m_path;
    double m_local_path_length;
    double m_heading_error_weight;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_speedProfilePoints;

};

#endif //LOCAL_PLANNER_H