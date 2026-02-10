/*
 * uam_route_planner.cpp
 *
 *  Created on: Mar 18, 2024
 *      Author: Daegyu Lee
 */
#include "uam_route_planner/uam_route_planner.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <assert.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <chrono>

using namespace geo_converter;
using namespace std::chrono_literals;

#define INF 9999999
// Constructor
RoutePlanner::RoutePlanner(const std::string &node_name) : Node(node_name)
{
  this->Timer = this->create_wall_timer(1000ms, std::bind(&RoutePlanner::TimerCallback, this));
  this->pubFinalPathCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/PointCloud2/final_path", ai3ct::common::constants::QOS_EGO_ODOMETRY);
  this->pubSpeedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/PointCloud2/speed", ai3ct::common::constants::QOS_EGO_ODOMETRY);
  this->pubFinalPath = this->create_publisher<nav_msgs::msg::Path>("/Path/final_path", ai3ct::common::constants::QOS_EGO_ODOMETRY);
  this->pubModeSwitchMarkers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Markers/ready_to_hover", ai3ct::common::constants::QOS_EGO_ODOMETRY);

  this->subInitService = this->create_service<std_srvs::srv::Trigger>(
      "load_wpt",  
      std::bind(&RoutePlanner::initPlannerCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  this->getParams();
  RCLCPP_INFO_ONCE(this->get_logger(), "Origin Lat : %f, Origin Lon : %f", m_originLat, m_originLon);

  this->initPlanner();
}

RoutePlanner::~RoutePlanner() {}

void RoutePlanner::getParams()
{
  this->declare_parameter<std::string>("osm_file_name", std::string(""));
  this->m_osmFileName = this->get_parameter("osm_file_name").as_string();

  this->declare_parameter<double>("origin_lat", double(0.0));
  this->m_originLat = this->get_parameter("origin_lat").as_double();
  this->declare_parameter<double>("origin_lon", double(0.0));
  this->m_originLon = this->get_parameter("origin_lon").as_double();

  this->declare_parameter<double>("flight_altitude", double(0.0));
  this->m_flightAlt = this->get_parameter("flight_altitude").as_double();

  this->declare_parameter<bool>("is_dense_waypoint", bool(false));
  this->bDenseWpt = this->get_parameter("is_dense_waypoint").as_bool();
  
  this->declare_parameter<double>("interpolate_step", double(0.0));
  this->m_interpolateStep = this->get_parameter("interpolate_step").as_double();



}

void RoutePlanner::initPlannerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Trigger 서비스의 request는 사용 안 함

  // 실제 함수 호출
  m_ready_to_landing_que.clear();
  initPlanner();

  // 응답 설정
  response->success = true;
  response->message = "Initialize the route planner ...";
  RCLCPP_INFO(this->get_logger(), "RoutePlanner is successfully initialized.");
}

void RoutePlanner::initPlanner()
{
  OsmParsing(m_osmFileName, m_OsmParser, bParsingComplete);

  m_FinalPathPoints.reset(new PointCloud());
  m_MissionPoints.reset(new PointCloud());
  m_RiskyAreaPoints.reset(new PointCloud());
  m_SpeedPoints.reset(new PointCloud());
  m_VelGainPoints.reset(new PointCloud());

  if (bDenseWpt)
  {
    // When using dense waypoint, directly attributes are directly integrated.
    DenseWptPath(m_OsmParser, bParsingComplete,
                 m_FinalPath, m_FinalPathPoints,
                 m_MissionPoints, m_SpeedPoints,
                 m_VelGainPoints);
  }
  else
  {
    // When using sparse waypoint, interpolation should be done.
    InterpolatePath(m_OsmParser, bParsingComplete,
                    m_FinalPath, m_FinalPathPoints,
                    m_MissionPoints, m_SpeedPoints,
                    m_VelGainPoints);
  }
}

void RoutePlanner::OsmParsing(const std::string OsmFileName, route_planner_msgs::msg::OsmParser &OsmParser, bool &ParsingComplete)
{
  RCLCPP_INFO(this->get_logger(), "opening the osm map file..");
  // Open the file
  std::string package_path = ament_index_cpp::get_package_share_directory("uam_route_planner");
  std::string FullFilePath =
      (package_path + "/waypoints/" + OsmFileName).c_str();
  pugi::xml_document doc;
  pugi::xml_parse_result parser = doc.load_file(FullFilePath.c_str());

  // file opening result
  if (!parser)
  {
    std::cout << "Parse error: " << parser.description()
              << ", character pos= " << parser.offset << std::endl;
    std::cout << "Tried to open .. \n"
              << FullFilePath.c_str() << std::endl;
    RCLCPP_ERROR(this->get_logger(), "Invalid file path.. ");
    return;
  }
  else
  {
    std::cout << "Opening ... " << OsmFileName.c_str() << std::endl;
    std::cout << "Parse result: " << parser.description()
              << ", character pos= " << parser.offset << std::endl;
    std::cout << "Tried to open .. \n"
              << FullFilePath.c_str() << std::endl;
    RCLCPP_INFO(this->get_logger(), "Vaild file!");
  }

  sensor_msgs::msg::NavSatFix origin_llh;
  origin_llh.latitude = m_originLat;
  origin_llh.longitude = m_originLon;
  geo_converter::UtmProjector UtmProjector(origin_llh);
  // geo_converter::EcefProjector m_EcefProjector(origin_llh);

  // Node data
  for (pugi::xml_node node : doc.child("osm").children("node"))
  {
    // std::cout << node.attribute("id").value() << ", lat: " <<
    // node.attribute("lat").value() << ", lon: " <<
    // node.attribute("lon").value()<< std::endl;

    route_planner_msgs::msg::Node nodeTmp;
    sensor_msgs::msg::NavSatFix gpsTmp;
    nodeTmp.id = node.attribute("id").as_int();
    nodeTmp.lat = node.attribute("lat").as_double();
    nodeTmp.lon = node.attribute("lon").as_double();
    gpsTmp.latitude = node.attribute("lat").as_double();
    gpsTmp.longitude = node.attribute("lon").as_double();

    geometry_msgs::msg::Pose2D pose2DTmp = UtmProjector.forward(gpsTmp);
  // geometry_msgs::msg::Point pointTmp = EcefProjector.forward(gpsTmp);
    double yaw_bais_deg = m_YawBias;
    double yaw_bias = yaw_bais_deg * M_PI / 180;

    nodeTmp.x = pose2DTmp.x * cos(yaw_bias) - pose2DTmp.y * sin(yaw_bias);
    nodeTmp.y = pose2DTmp.x * sin(yaw_bias) + pose2DTmp.y * cos(yaw_bias);

    // nodeTmp.x = pointTmp.x * cos(yaw_bias) - pointTmp.y * sin(yaw_bias);
    // nodeTmp.y = pointTmp.x * sin(yaw_bias) + pointTmp.y * cos(yaw_bias);

    // Tags
    for (pugi::xml_node tag : node.children("tag"))
    {
      std::string heading = "yaw";
      if (tag.attribute("k").as_string() == heading)
      {
        // std::cout << "yaw: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.heading = tag.attribute("v").as_double();
      }
      std::string speed = "speed";
      if (tag.attribute("k").as_string() == speed)
      {
        // std::cout << "speed: " << tag.attribute("v").as_double() <<
        // std::endl;
        nodeTmp.velocity = tag.attribute("v").as_double();
      }
      std::string height = "height";
      if (tag.attribute("k").as_string() == height)
      {
        // std::cout << "height: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.z = tag.attribute("v").as_double();
      }
      std::string mission = "mission";
      if (tag.attribute("k").as_string() == mission)
      {
        nodeTmp.mission = tag.attribute("v").as_int();
      }
      std::string risky_area = "risky_area";
      if (tag.attribute("k").as_string() == risky_area)
      {
        nodeTmp.risky_area = tag.attribute("v").as_int();
      }
      std::string vel_profile_gain = "vel_profile_gain";
      if (tag.attribute("k").as_string() == vel_profile_gain)
      {
        nodeTmp.vel_profile_gain = tag.attribute("v").as_double();
      }
      std::string ready_to_landing = "ready_to_landing";
      if (tag.attribute("k").as_string() == ready_to_landing)
      {
        nodeTmp.ready_to_landing = tag.attribute("v").as_bool();
      }
    }

    OsmParser.nodes.push_back(nodeTmp);
  }

  // Way data
  for (pugi::xml_node way : doc.child("osm").children("way"))
  {
    // Way id
    // std::cout << "---------------" << std::endl;
    // std::cout << "way id: " << way.attribute("id").value() << std::endl;
    route_planner_msgs::msg::Way wayTmp;
    wayTmp.id = way.attribute("id").as_int();
    // attribute of way data : Node IDs
    for (pugi::xml_node way_data : way.children("nd"))
    {
      // std::cout << "way id: "<< way.attribute("id").value() <<", ref: " <<
      // way_data.attribute("ref").value() << std::endl;
      route_planner_msgs::msg::Node nodeRef; // Node Ids in the each way
      nodeRef.id = way_data.attribute("ref").as_int();
      for (auto node : OsmParser.nodes)
      {
        if (node.id == nodeRef.id)
        {
          nodeRef = node;
        }
      }
      wayTmp.nodes.push_back(nodeRef);
    }
    // std::cout << "way id: "<< way.attribute("id").value() <<", first: " <<
    // way.first_child().attribute("ref").as_int() << std::endl; std::cout <<
    // "way id: "<< way.attribute("id").value() <<", last: " <<
    // way.last_child().attribute("ref").as_int() << std::endl;
    wayTmp.first_node_id = wayTmp.nodes[0].id;
    wayTmp.last_node_id = wayTmp.nodes[wayTmp.nodes.size() - 1].id;
    wayTmp.cost = sqrt(
        pow(wayTmp.nodes[0].x - wayTmp.nodes[wayTmp.nodes.size() - 1].x, 2) +
        pow(wayTmp.nodes[0].y - wayTmp.nodes[wayTmp.nodes.size() - 1].y, 2));
    // Tags
    for (pugi::xml_node tag : way.children("tag"))
    {
      // current_LinkID
      std::string currentWay = "ID";
      // R_LinkID
      std::string rightWay = "R_LinkID";
      // L_LinKID
      std::string leftWay = "L_LinKID";
      // MaxSpeed
      std::string maxSpeed = "MaxSpeed";
      // RoadRank
      std::string RoadRank = "RoadRank";
      // RoadType
      std::string RoadType = "RoadType";
      // LinkType
      std::string LinkType = "LinkType";

      if (tag.attribute("k").as_string() == currentWay)
      {
        wayTmp.current_link_id = tag.attribute("v").as_string();
      }
      else if (tag.attribute("k").as_string() == rightWay)
      {
        wayTmp.right_link_id = tag.attribute("v").as_string();
      }
      else if (tag.attribute("k").as_string() == leftWay)
      {
        wayTmp.left_link_id = tag.attribute("v").as_string();
      }
      else if (tag.attribute("k").as_string() == maxSpeed)
      {
        wayTmp.speed_limit = tag.attribute("v").as_double();
      }
      else if (tag.attribute("k").as_string() == RoadRank)
      {
        wayTmp.road_rank = tag.attribute("v").as_int();
      }
      else if (tag.attribute("k").as_string() == RoadType)
      {
        wayTmp.road_type = tag.attribute("v").as_int();
      }
      else if (tag.attribute("k").as_string() == LinkType)
      {
        wayTmp.link_type = tag.attribute("v").as_int();
      }
    }

    // Relations
    for (pugi::xml_node relation : doc.child("osm").children("relation"))
    {
      // attribute of member data
      std::vector<int> memberTmp;
      bool bIdExist = false;
      for (pugi::xml_node member : relation.children("member"))
      {
        memberTmp.push_back(member.attribute("ref").as_int());
        if (member.attribute("ref").as_int() == wayTmp.id)
        {
          bIdExist = true;
          // memberTmp.pop_back();
        }
      }

      if (!bIdExist)
        continue;

      // add ids, if wayTmp.id is existing in the array
      for (auto exist : memberTmp)
      {
        wayTmp.adj_way_ids.push_back(exist);
      }
      // std::cout << relation.attribute("id").as_int() << std::endl;
    }

    OsmParser.ways.push_back(wayTmp);
  }

  RCLCPP_INFO(this->get_logger(), "Converting OSM file to the ros system is completed!");
  std::cout << "------------------------------------" << std::endl;
  std::cout << "node size: " << OsmParser.nodes.size() << std::endl;
  std::cout << "way size: " << OsmParser.ways.size() << "\n"
            << std::endl;

  ParsingComplete = true;
}

// void RoutePlanner::InterpolatePath(const route_planner_msgs::msg::OsmParser &OsmParser, const bool &ParsingComplete,
//                                    nav_msgs::msg::Path &FinalPath, PointCloud::Ptr &FinalPathPoints,
//                                    PointCloud::Ptr &MissionPoints, PointCloud::Ptr &SpeedLimitPoints,
//                                    PointCloud::Ptr &VelGainPoints)
// {
//   if (!ParsingComplete)
//   {
//     RCLCPP_ERROR(this->get_logger(), "Parsing ERROR");
//     return;
//   }

//   if (OsmParser.ways.size() != 1)
//   {
//     RCLCPP_ERROR(this->get_logger(), "Make referential waypoint in 1 segments.");
//     RCLCPP_ERROR(this->get_logger(), "Current way size: %ld", OsmParser.ways.size());
//     return;
//   }

//   FinalPath.poses.clear();
//   size_t cnt = 0;
//   for (size_t i = 0; i < OsmParser.ways[0].nodes.size() - 1; i++)
//   {
//     auto current_node = OsmParser.ways[0].nodes[i];
//     auto next_node = OsmParser.ways[0].nodes[i + 1];
//     double BetweenPointsDist = sqrt(
//         pow(next_node.x - current_node.x, 2) +
//         pow(next_node.y - current_node.y, 2));

//     double step_thres = 1.0;
//     int step_number = BetweenPointsDist / step_thres;

//     tf2::Quaternion q_tf;
//     double yaw = atan2(next_node.y - current_node.y,
//                        next_node.x - current_node.x);
//     q_tf.setRPY(0.0, 0.0, yaw);

//     int mission_num = OsmParser.ways[0].nodes[i].mission;
//     double speed_limit = OsmParser.ways[0].nodes[i].velocity;
//     double next_speed_limit = OsmParser.ways[0].nodes[i + 1].velocity;

//     double vel_gain = OsmParser.ways[0].nodes[i].vel_profile_gain;
//     double next_vel_gain = OsmParser.ways[0].nodes[i + 1].vel_profile_gain;

//     if (BetweenPointsDist < step_thres)
//     {
//       geometry_msgs::msg::PoseStamped poseTmp;
//       poseTmp.pose.position.x = current_node.x;
//       poseTmp.pose.position.y = current_node.y;
//       poseTmp.pose.position.z = m_flightAlt;

//       poseTmp.pose.orientation = tf2::toMsg(q_tf);

//       FinalPath.poses.push_back(poseTmp);

//       PointT current_point;
//       current_point.x = poseTmp.pose.position.x;
//       current_point.y = poseTmp.pose.position.y;
//       current_point.intensity = cnt;
//       FinalPathPoints->points.push_back(current_point);

//       auto mission_point = current_point;
//       mission_point.intensity = mission_num;
//       MissionPoints->points.push_back(mission_point);

//       auto speed_point = current_point;
//       speed_point.intensity = speed_limit * vel_gain;
//       SpeedLimitPoints->points.push_back(speed_point);

//       auto vel_gain_point = current_point;
//       vel_gain_point.intensity = vel_gain;
//       VelGainPoints->points.push_back(vel_gain_point);

//       cnt++;
//     }
//     else
//     {
//       for (int j = 0; j < step_number; j++)
//       {
//         geometry_msgs::msg::PoseStamped poseTmp;
//         poseTmp.pose.position.x =
//             current_node.x +
//             (next_node.x - current_node.x) * j / step_number;
//         poseTmp.pose.position.y =
//             current_node.y +
//             (next_node.y - current_node.y) *
//                 j / step_number;
//         poseTmp.pose.position.z = m_flightAlt;

//         poseTmp.pose.orientation = tf2::toMsg(q_tf);

//         FinalPath.poses.push_back(poseTmp);

//         PointT current_point;
//         current_point.x = poseTmp.pose.position.x;
//         current_point.y = poseTmp.pose.position.y;
//         current_point.intensity = cnt;
//         FinalPathPoints->points.push_back(current_point);
//         cnt++;

//         auto mission_point = current_point;
//         mission_point.intensity = mission_num;
//         MissionPoints->points.push_back(mission_point);

//         auto speed_point = current_point;
//         speed_point.intensity = (speed_limit +
//                                  (next_speed_limit - speed_limit) * j / step_number) *
//                                 (vel_gain +
//                                  (next_vel_gain - vel_gain) * j / step_number);
//         SpeedLimitPoints->points.push_back(speed_point);

//         auto vel_gain_point = current_point;
//         vel_gain_point.intensity = vel_gain +
//                                    (next_vel_gain - vel_gain) * j / step_number;
//         VelGainPoints->points.push_back(vel_gain_point);
//       }
//     }
//   }
// }

void RoutePlanner::InterpolatePath(const route_planner_msgs::msg::OsmParser &OsmParser, 
                                   const bool &ParsingComplete,
                                   nav_msgs::msg::Path &FinalPath, 
                                   PointCloud::Ptr &FinalPathPoints,
                                   PointCloud::Ptr &MissionPoints, 
                                   PointCloud::Ptr &SpeedLimitPoints,
                                   PointCloud::Ptr &VelGainPoints)
{
  if (!ParsingComplete)
  {
    RCLCPP_ERROR(this->get_logger(), "Parsing ERROR");
    return;
  }

  // 예시는 단일 Way만을 가정하는 기존 로직 그대로 유지
  if (OsmParser.ways.size() != 1)
  {
    RCLCPP_ERROR(this->get_logger(), "Make referential waypoint in 1 segments.");
    RCLCPP_ERROR(this->get_logger(), "Current way size: %ld", OsmParser.ways.size());
    return;
  }

  FinalPath.poses.clear();
  size_t cnt = 0;

  // -------------------------------
  // (1) 이륙 경로 (Takeoff) 추가
  // -------------------------------
  {
    // 첫 번째 노드의 x,y
    auto first_node = OsmParser.ways[0].nodes[0];
    double takeoff_x = first_node.x;
    double takeoff_y = first_node.y;
    double takeoff_z = first_node.z;
    
    // 이륙 스텝 (1.0m씩 올린다고 가정; 상황에 맞게 조절 가능)
    double step_alt = m_interpolateStep;
    int steps = static_cast<int>(takeoff_z / step_alt);

    // 첫 번째 노드에서 Yaw(방향)
    auto second_node = OsmParser.ways[0].nodes[1];
    tf2::Quaternion q_tf_first;
    double yaw_first = std::atan2(second_node.y - takeoff_y,
                                  second_node.x - takeoff_x);
    q_tf_first.setRPY(0.0, 0.0, yaw_first);

    for(int i = 0; i <= steps; i++)
    {
      double current_alt = i * step_alt;
      if(current_alt > takeoff_z) current_alt = takeoff_z;

      geometry_msgs::msg::PoseStamped poseTmp;
      poseTmp.pose.position.x = takeoff_x;
      poseTmp.pose.position.y = takeoff_y;
      poseTmp.pose.position.z = current_alt;
      poseTmp.pose.orientation = tf2::toMsg(q_tf_first);

      FinalPath.poses.push_back(poseTmp);

      // 포인트 클라우드, 미션, 속도, etc
      PointT current_point;
      current_point.x = takeoff_x;
      current_point.y = takeoff_y;
      current_point.z = current_alt;
      current_point.intensity = cnt;
      FinalPathPoints->points.push_back(current_point);

      auto mission_point = current_point;
      mission_point.intensity = 0; // 필요에 따라 미션번호 설정
      MissionPoints->points.push_back(mission_point);

      auto speed_point = current_point;
      speed_point.intensity = OsmParser.ways[0].nodes[0].velocity; 
      SpeedLimitPoints->points.push_back(speed_point);

      auto vel_gain_point = current_point;
      vel_gain_point.intensity = OsmParser.ways[0].nodes[0].vel_profile_gain; 
      VelGainPoints->points.push_back(vel_gain_point);

      cnt++;
    }
  }

  // -------------------------------
  // (2) 고도 유지 비행
  // -------------------------------
  for (size_t i = 0; i < OsmParser.ways[0].nodes.size() - 1; i++)
  {
    auto current_node = OsmParser.ways[0].nodes[i];
    auto next_node = OsmParser.ways[0].nodes[i + 1];
    double BetweenPointsDist = std::hypot(
        next_node.x - current_node.x,
        next_node.y - current_node.y);

    // 원하는 지상좌표 보간 간격
    double step_thres = m_interpolateStep;
    int step_number = BetweenPointsDist / step_thres;

    tf2::Quaternion q_tf;
    double yaw = std::atan2(next_node.y - current_node.y,
                            next_node.x - current_node.x);
    q_tf.setRPY(0.0, 0.0, yaw);

    int mission_num = current_node.mission;
    double speed_limit = current_node.velocity;
    double next_speed_limit = next_node.velocity;
    double vel_gain = current_node.vel_profile_gain;
    double next_vel_gain = next_node.vel_profile_gain;
    if(current_node.ready_to_landing)
    {
      geometry_msgs::msg::Point point_tmp;
      point_tmp.x = current_node.x;
      point_tmp.y = current_node.y;
      point_tmp.z = current_node.z;
      m_ready_to_landing_que.push_back(point_tmp);
    }
    if (BetweenPointsDist < step_thres)
    {
      // 실제 거리가 1m 미만이면 그냥 현재 노드만 추가
      geometry_msgs::msg::PoseStamped poseTmp;
      poseTmp.pose.position.x = current_node.x;
      poseTmp.pose.position.y = current_node.y;
      poseTmp.pose.position.z = current_node.z;  // 항상 비행 고도
      poseTmp.pose.orientation = tf2::toMsg(q_tf);

      FinalPath.poses.push_back(poseTmp);

      PointT current_point;
      current_point.x = poseTmp.pose.position.x;
      current_point.y = poseTmp.pose.position.y;
      current_point.z = poseTmp.pose.position.z;
      current_point.intensity = cnt;
      FinalPathPoints->points.push_back(current_point);

      auto mission_point = current_point;
      mission_point.intensity = mission_num;
      MissionPoints->points.push_back(mission_point);

      auto speed_point = current_point;
      speed_point.intensity = speed_limit * vel_gain;
      SpeedLimitPoints->points.push_back(speed_point);

      auto vel_gain_point = current_point;
      vel_gain_point.intensity = vel_gain;
      VelGainPoints->points.push_back(vel_gain_point);

      cnt++;
    }
    else
    {
      // 노드 간 선형보간
      for (int j = 0; j < step_number; j++)
      {
        geometry_msgs::msg::PoseStamped poseTmp;
        poseTmp.pose.position.x =
            current_node.x + (next_node.x - current_node.x) * (double)j / step_number;
        poseTmp.pose.position.y =
            current_node.y + (next_node.y - current_node.y) * (double)j / step_number;
        poseTmp.pose.position.z = 
            current_node.z + (next_node.z - current_node.z) * (double)j / step_number; 
        poseTmp.pose.orientation = tf2::toMsg(q_tf);

        FinalPath.poses.push_back(poseTmp);

        PointT current_point;
        current_point.x = poseTmp.pose.position.x;
        current_point.y = poseTmp.pose.position.y;
        current_point.z = poseTmp.pose.position.z;
        current_point.intensity = cnt;
        FinalPathPoints->points.push_back(current_point);
        cnt++;

        auto mission_point = current_point;
        mission_point.intensity = mission_num; 
        MissionPoints->points.push_back(mission_point);

        auto speed_point = current_point;
        // 속도는 양 노드 값 보간 * vel_gain도 보간
        double inter_speed = (speed_limit + (next_speed_limit - speed_limit) * (double)j / step_number);
        double inter_gain = (vel_gain + (next_vel_gain - vel_gain) * (double)j / step_number);
        speed_point.intensity = inter_speed * inter_gain;
        SpeedLimitPoints->points.push_back(speed_point);

        auto vel_gain_point = current_point;
        vel_gain_point.intensity = inter_gain;
        VelGainPoints->points.push_back(vel_gain_point);
      }
    }
  }

  // -------------------------------
  // (3) 착륙 경로 (Landing) 추가
  // -------------------------------
  {
    // 마지막 노드의 x,y
    auto last_node = OsmParser.ways[0].nodes.back();
    double landing_x = last_node.x;
    double landing_y = last_node.y;
    double landing_z = last_node.z;


    // 착륙 스텝 (1.0m씩 내린다고 가정; 상황에 맞게 조절 가능)
    double step_alt = m_interpolateStep;
    int steps = static_cast<int>(landing_z / step_alt);

    // 마지막 구간에서의 yaw (또는 0으로 설정)
    auto last_minus_one_node = OsmParser.ways[0].nodes[OsmParser.ways[0].nodes.size() - 2];
    tf2::Quaternion q_tf_last;
    double yaw_last = std::atan2(landing_y - last_minus_one_node.y,
                                 landing_x - last_minus_one_node.x);
    q_tf_last.setRPY(0.0, 0.0, yaw_last);

    for(int i = 0; i <= steps; i++)
    {
      double current_alt = landing_z - i * step_alt;
      if(current_alt < 0.0) current_alt = 0.0;

      geometry_msgs::msg::PoseStamped poseTmp;
      poseTmp.pose.position.x = landing_x;
      poseTmp.pose.position.y = landing_y;
      poseTmp.pose.position.z = current_alt;
      poseTmp.pose.orientation = tf2::toMsg(q_tf_last);

      FinalPath.poses.push_back(poseTmp);

      PointT current_point;
      current_point.x = landing_x;
      current_point.y = landing_y;
      current_point.z = current_alt;
      current_point.intensity = cnt;
      FinalPathPoints->points.push_back(current_point);

      auto mission_point = current_point;
      mission_point.intensity = 0; // 필요 시 설정
      MissionPoints->points.push_back(mission_point);

      auto speed_point = current_point;
      speed_point.intensity = last_node.velocity; 
      SpeedLimitPoints->points.push_back(speed_point);

      auto vel_gain_point = current_point;
      vel_gain_point.intensity = 0.0;
      speed_point.intensity = last_node.vel_profile_gain; 
      VelGainPoints->points.push_back(vel_gain_point);

      cnt++;
    }
  }

}

void RoutePlanner::DenseWptPath(const route_planner_msgs::msg::OsmParser &OsmParser, const bool &ParsingComplete,
                                nav_msgs::msg::Path &FinalPath, PointCloud::Ptr &FinalPathPoints,
                                PointCloud::Ptr &MissionPoints, PointCloud::Ptr &SpeedLimitPoints,
                                PointCloud::Ptr &VelGainPoints)
{
  if (!ParsingComplete)
  {
    RCLCPP_ERROR(this->get_logger(), "Parsing ERROR");
    return;
  }

  if (OsmParser.ways.size() != 1)
  {
    RCLCPP_ERROR(this->get_logger(), "Make referential waypoint in 1 segments.");
    RCLCPP_ERROR(this->get_logger(), "Current way size: %ld", OsmParser.ways.size());
    return;
  }

  FinalPath.poses.clear();
  size_t cnt = 0;
  for (size_t i = 0; i < OsmParser.ways[0].nodes.size() - 1; i++)
  {
    auto current_node = OsmParser.ways[0].nodes[i];
    auto next_node = OsmParser.ways[0].nodes[i + 1];

    tf2::Quaternion q_tf;
    double yaw = atan2(next_node.y - current_node.y,
                       next_node.x - current_node.x);
    q_tf.setRPY(0.0, 0.0, yaw);

    int mission_num = OsmParser.ways[0].nodes[i].mission;
    double speed_limit = OsmParser.ways[0].nodes[i].velocity;
    double vel_gain = OsmParser.ways[0].nodes[i].vel_profile_gain;

    geometry_msgs::msg::PoseStamped poseTmp;
    poseTmp.pose.position.x = current_node.x;
    poseTmp.pose.position.y = current_node.y;
    poseTmp.pose.position.z = current_node.z;

    poseTmp.pose.orientation = tf2::toMsg(q_tf);

    FinalPath.poses.push_back(poseTmp);

    PointT current_point;
    current_point.x = poseTmp.pose.position.x;
    current_point.y = poseTmp.pose.position.y;
    current_point.intensity = cnt;
    FinalPathPoints->points.push_back(current_point);

    auto mission_point = current_point;
    mission_point.intensity = mission_num;
    MissionPoints->points.push_back(mission_point);

    auto speed_point = current_point;
    speed_point.intensity = speed_limit * vel_gain;
    SpeedLimitPoints->points.push_back(speed_point);

    auto vel_gain_point = current_point;
    vel_gain_point.intensity = vel_gain;
    VelGainPoints->points.push_back(vel_gain_point);
    cnt++;
  }
  // last point
  int last_node_idx = OsmParser.ways[0].nodes.size() - 1;
  auto current_node = OsmParser.ways[0].nodes[last_node_idx];
  auto next_node = OsmParser.ways[0].nodes[0];

  tf2::Quaternion q_tf;
  double yaw = atan2(next_node.y - current_node.y,
                     next_node.x - current_node.x);
  q_tf.setRPY(0.0, 0.0, yaw);

  int mission_num = OsmParser.ways[0].nodes[last_node_idx].mission;
  double speed_limit = OsmParser.ways[0].nodes[last_node_idx].velocity;
  double vel_gain = OsmParser.ways[0].nodes[last_node_idx].vel_profile_gain;

  geometry_msgs::msg::PoseStamped poseTmp;
  poseTmp.pose.position.x = current_node.x;
  poseTmp.pose.position.y = current_node.y;
  poseTmp.pose.position.z = current_node.z;

  poseTmp.pose.orientation = tf2::toMsg(q_tf);

  FinalPath.poses.push_back(poseTmp);

  PointT current_point;
  current_point.x = poseTmp.pose.position.x;
  current_point.y = poseTmp.pose.position.y;
  current_point.intensity = cnt;
  FinalPathPoints->points.push_back(current_point);

  auto mission_point = current_point;
  mission_point.intensity = mission_num;
  MissionPoints->points.push_back(mission_point);

  auto speed_point = current_point;
  speed_point.intensity = speed_limit * vel_gain;
  SpeedLimitPoints->points.push_back(speed_point);

  auto vel_gain_point = current_point;
  vel_gain_point.intensity = vel_gain;
  VelGainPoints->points.push_back(vel_gain_point);
}

void RoutePlanner::TimerCallback()
{
  if(m_FinalPathPoints != nullptr)
  {
    if(!m_FinalPathPoints->points.empty())
    {
    sensor_msgs::msg::PointCloud2 FinalPathCloudMsg;
    pcl::toROSMsg(*m_FinalPathPoints, FinalPathCloudMsg); 
    FinalPathCloudMsg.header.frame_id = "map";
    FinalPathCloudMsg.header.stamp = this->now();
    pubFinalPathCloud->publish(FinalPathCloudMsg);
    }
  }

  if(m_SpeedPoints != nullptr)
  {
    if(!m_SpeedPoints->points.empty())
    {
    sensor_msgs::msg::PointCloud2 SpeedCloudMsg;
    pcl::toROSMsg(*m_SpeedPoints, SpeedCloudMsg); 
    SpeedCloudMsg.header.frame_id = "map";
    SpeedCloudMsg.header.stamp = this->now();
    pubSpeedCloud->publish(SpeedCloudMsg);
    }
  }


  if(!m_FinalPath.poses.empty())
  {
    m_FinalPath.header.frame_id = "map";
    m_FinalPath.header.stamp = this->now();
    pubFinalPath->publish(m_FinalPath);
  }

//       .push_back(point_tmp);
  if(!m_ready_to_landing_que.empty())
  {
    // 실제 시각화 메시지를 만드는 함수
    visualization_msgs::msg::MarkerArray marker_array;

    // 반복문을 돌며 Sphere + Text 마커 생성
    int i = 0;
    for(const auto & point : m_ready_to_landing_que)
    {
      // (1) 위치 표시용 Sphere 마커
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "map";  // 좌표계 (map, odom 등)
      sphere.header.stamp = this->now();
      sphere.ns = "mode_switch";
      sphere.id = i; // 각각 고유 id
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;

      // 위치/자세
      sphere.pose.position = point;  // x,y,z 그대로
      sphere.pose.orientation.w = 1.0; // 기본값

      // 크기
      sphere.scale.x = 30.0; 
      sphere.scale.y = 30.0;
      sphere.scale.z = 30.0;

      // 색상(RED)
      sphere.color.r = 1.0f;
      sphere.color.g = 1.0f;
      sphere.color.b = 1.0f;
      sphere.color.a = 1.0f; // 불투명

      marker_array.markers.push_back(sphere);

      // (2) 텍스트 표시용 마커 (위에 띄워두는 예시)
      visualization_msgs::msg::Marker text;
      text.header.frame_id = "map";
      text.header.stamp = this->now();
      text.ns = "mode_switch";
      text.id = i + 1000; // sphere와 충돌하지 않게 다른 id
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      // 텍스트 위치 (조금 위로 올리는 예시)
      text.pose.position.x = point.x;
      text.pose.position.y = point.y;
      text.pose.position.z = point.z + 15.0;  // sphere 위쪽
      text.pose.orientation.w = 1.0;

      // 텍스트 크기
      text.scale.z = 15.0;  // 글자 높이(폰트 사이즈)
      
      // 색상(GREEN)
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;

      // 실제 텍스트
      text.text = "Mode \n Switching";
      marker_array.markers.push_back(text);

      i++;
    }

    pubModeSwitchMarkers->publish(marker_array);
  }

}