/*
 * uam_route_planner.h
 *
 *  Created on: Mar 18, 2024
 *      Author: Daegyu Lee
 */
#ifndef UAM_ROUTE_PLANNER_H
#define UAM_ROUTE_PLANNER_H

// headers in ROS
#include "ai3ct_common/constants.h"
#include "geodetic_converter/UTM.h"
#include "geodetic_converter/ECEF.h"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

#include <pugixml.hpp>

// User defined messages
#include <route_planner_msgs/msg/osm_parser.hpp>
#include <route_planner_msgs/msg/way.hpp>
#include <route_planner_msgs/msg/node.hpp>
#include <route_planner_msgs/msg/tag.hpp>


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

// service call
#include "std_srvs/srv/trigger.hpp"  

// Define a point cloud type
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct Point2D
{
  double x;
  double y;
  Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}
};

class RoutePlanner : public rclcpp::Node
{
public:
  RoutePlanner(const std::string &node_name);
  ~RoutePlanner();

  void OsmParsing(const std::string OsmFileName,
                  route_planner_msgs::msg::OsmParser &OsmParser,
                  bool &ParsingComplete);

private:
  rclcpp::TimerBase::SharedPtr Timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFinalPathCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSpeedCloud;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubFinalPath;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubModeSwitchMarkers;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr subInitService;


  void TimerCallback();
  void getParams();
  void initPlanner();

  void DenseWptPath(const route_planner_msgs::msg::OsmParser &OsmParser, const bool &ParsingComplete,
                    nav_msgs::msg::Path &FinalPath, PointCloud::Ptr &FinalPathPoints,
                    PointCloud::Ptr &MissionPoints, PointCloud::Ptr &SpeedLimitPoints,
                    PointCloud::Ptr &VelGainPoints);
  void InterpolatePath(const route_planner_msgs::msg::OsmParser &OsmParser, const bool &ParsingComplete,
                       nav_msgs::msg::Path &FinalPath, PointCloud::Ptr &FinalPathPoints,
                       PointCloud::Ptr &MissionPoints, PointCloud::Ptr &SpeedLimitPoints,
                       PointCloud::Ptr &VelGainPoints);

  void initPlannerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  double m_originLat;
  double m_originLon;
  double m_YawBias = 0.0;
  double m_flightAlt;
  std::string m_osmFileName;
  bool bDenseWpt;
  double m_interpolateStep;

  route_planner_msgs::msg::OsmParser m_OsmParser;
  bool bParsingComplete = false;

  nav_msgs::msg::Path m_FinalPath;
  PointCloud::Ptr m_FinalPathPoints;
  PointCloud::Ptr m_MissionPoints;
  PointCloud::Ptr m_RiskyAreaPoints;
  PointCloud::Ptr m_SpeedPoints;
  PointCloud::Ptr m_VelGainPoints;

  std::deque<geometry_msgs::msg::Point> m_ready_to_landing_que;
};

#endif // UAM_ROUTE_PLANNER_H
