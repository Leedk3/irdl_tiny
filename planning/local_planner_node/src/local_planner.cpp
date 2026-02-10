#include <local_planner/local_planner.h>
#include <chrono>

using namespace std::chrono_literals;
#define INFINITE 999999

LocalPlanner::LocalPlanner(const std::string& node_name) : Node(node_name)
{
  this->Timer = this->create_wall_timer(10ms, std::bind(&LocalPlanner::TimerCallback, this));

  this->subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry/msfs", ai3ct::common::constants::QOS_EGO_ODOMETRY, std::bind(&LocalPlanner::odomCallback, this, std::placeholders::_1));
  this->subPath = this->create_subscription<nav_msgs::msg::Path>(
      "/Path/final_path", ai3ct::common::constants::QOS_EGO_ODOMETRY, std::bind(&LocalPlanner::pathCallback, this, std::placeholders::_1));
  this->subSpeedProfile = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/PointCloud2/speed", ai3ct::common::constants::QOS_EGO_ODOMETRY, std::bind(&LocalPlanner::speedProfileCallback, this, std::placeholders::_1));

  this->pubLocalPathBody = this->create_publisher<nav_msgs::msg::Path>("/Path/on_body", ai3ct::common::constants::QOS_EGO_ODOMETRY);
  this->pubLocalPathGlobal = this->create_publisher<nav_msgs::msg::Path>("/Path/on_global", ai3ct::common::constants::QOS_EGO_ODOMETRY);
  this->pubTargetSpeed = this->create_publisher<std_msgs::msg::Float64>("/Float64/target_speed", ai3ct::common::constants::QOS_EGO_ODOMETRY);  

  getParams();
};
LocalPlanner::~LocalPlanner(){};

void LocalPlanner::getParams()
{
  this->declare_parameter<double>("local_path_length", double(0.0));
  this->m_local_path_length = this->get_parameter("local_path_length").as_double();

  this->declare_parameter<double>("heading_error_weight", double(0.0));
  this->m_heading_error_weight = this->get_parameter("heading_error_weight").as_double();

}

void LocalPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  this->m_odom = *msg;
}

void LocalPlanner::speedProfileCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{  
  m_speedProfilePoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_speedProfilePoints);

}

void LocalPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  this->m_path = *msg;
}

void LocalPlanner::TimerCallback()
{
  static auto prev_time = std::chrono::high_resolution_clock::now();
  auto cur_time = std::chrono::high_resolution_clock::now();
  m_dt = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_time - prev_time).count() / 1e9;
  prev_time = cur_time;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "dt: %f [msec]", m_dt);


  // 1. 현재 오돔(글로벌 좌표계에서 기체 포즈) 획득
  geometry_msgs::msg::Pose current_pose = m_odom.pose.pose; // /Odometry/msfs

  // 2. 전체 글로벌 경로(m_path) 중 가장 가까운 지점 인덱스 찾기
  int closest_idx = findClosestPointIndex(m_path, current_pose);
  if(closest_idx < 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No valid path or negative closest_idx");
    return;
  }

  // 3. Local Path 길이 : m_local_path_length
  double local_path_length = m_local_path_length;  
  nav_msgs::msg::Path local_path_global = extractLocalPath(m_path, closest_idx, local_path_length);
  pubLocalPathGlobal->publish(local_path_global);
  // 4. Local Path를 Body Frame으로 변환
  nav_msgs::msg::Path local_path_body = transformPathToBody(local_path_global, current_pose);

  // 5. 퍼블리시 (RViz에서 시각화 등)
  pubLocalPathBody->publish(local_path_body);


  // 6. Velocity Profile publish
  if(m_speedProfilePoints != nullptr)
  {
    if(!m_speedProfilePoints->points.empty() && m_speedProfilePoints->points.size()==m_path.poses.size())
    {
      double target_velocity = m_speedProfilePoints->points[closest_idx].intensity;
      std::cout << "target_velocity : " << target_velocity << std::endl;
      std_msgs::msg::Float64 target_speed_msg;
      target_speed_msg.data = target_velocity;
      pubTargetSpeed->publish(target_speed_msg);

    }
  }


}

int LocalPlanner::findClosestPointIndex(const nav_msgs::msg::Path &path,
                                        const geometry_msgs::msg::Pose &current_pose)
{
  if(path.poses.empty())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Reference path is empty");
    return -1;
  }

  // 현재 기체의 위치/헤딩
  double ego_x = current_pose.position.x;
  double ego_y = current_pose.position.y;
  double ego_z = current_pose.position.z;
  double ego_yaw = tf2::getYaw(current_pose.orientation);

  double heading_weight = m_heading_error_weight;  // 헤딩 차 가중치 (필요시 파라미터화)

  int closest_idx = 0;
  double min_cost = std::numeric_limits<double>::max();

  for(size_t i = 0; i < path.poses.size(); i++)
  {
    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;
    double pz = path.poses[i].pose.position.z;
    double pyaw = tf2::getYaw(path.poses[i].pose.orientation);

    double dx = px - ego_x;
    double dy = py - ego_y;
    double dz = pz - ego_z;
    double distance_3d = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    // 2d 기준
    // double distance = std::hypot(dx, dy);

    // 헤딩 오차
    double dyaw = shortestAngularDistance(ego_yaw, pyaw);

    // 코스트 = 거리 + w*(헤딩 편차)
    double cost = distance_3d + heading_weight * std::fabs(dyaw);

    if(cost < min_cost)
    {
      min_cost = cost;
      closest_idx = i;
    }
  }

  return closest_idx;
}

nav_msgs::msg::Path LocalPlanner::extractLocalPath(const nav_msgs::msg::Path &global_path,
                                                   int closest_idx,
                                                   double forward_distance)
{
  nav_msgs::msg::Path local_path;
  local_path.header = global_path.header;

  if(global_path.poses.empty() || closest_idx < 0 
     || static_cast<size_t>(closest_idx) >= global_path.poses.size())
  {
    // 유효하지 않은 경우 그냥 리턴
    return local_path;
  }

  // local_path에 먼저 closest_idx 포인트 추가
  local_path.poses.push_back(global_path.poses[closest_idx]);

  double accumulated_dist = 0.0;

  for(size_t i = closest_idx; i < global_path.poses.size() - 1; i++)
  {
    // i -> i+1까지 거리
    double x1 = global_path.poses[i].pose.position.x;
    double y1 = global_path.poses[i].pose.position.y;
    double x2 = global_path.poses[i+1].pose.position.x;
    double y2 = global_path.poses[i+1].pose.position.y;

    double segment_dist = std::hypot(x2 - x1, y2 - y1);

    if((accumulated_dist + segment_dist) > forward_distance)
    {
      // forward_distance를 초과하면 i+1을 완전히 추가하기보다는
      // 중간에 잘라서 추가할 수도 있음 (선형보간)
      // 여기서는 간단히 break
      break;
    }
    else
    {
      accumulated_dist += segment_dist;
      local_path.poses.push_back(global_path.poses[i+1]);
    }
  }

  return local_path;
}

nav_msgs::msg::Path LocalPlanner::transformPathToBody(const nav_msgs::msg::Path &path_in_global,
                                                      const geometry_msgs::msg::Pose &current_pose)
{
  nav_msgs::msg::Path path_on_body;
  path_on_body.header.frame_id = "base_link";  // 예: 바디 프레임 명시
  path_on_body.header.stamp = this->now();

  // (1) 현재 기체의 global pose → 4x4 변환 행렬 (World->Body)의 역
  //     사실상 Body->World 변환을 만든 뒤 inverse 하거나
  //     그냥 World->Body를 직접 구성해도 됩니다.

  // 1) roll, pitch, yaw 추출
  double rx, ry, rz, rw;
  rx = current_pose.orientation.x;
  ry = current_pose.orientation.y;
  rz = current_pose.orientation.z;
  rw = current_pose.orientation.w;
  tf2::Quaternion q(rx, ry, rz, rw);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 2) translation, rotation (World -> Body) 구성
  //    Body 원점이 World 좌표에서 (px, py, pz)에 있고
  //    회전(yaw, pitch, roll) 했다면,
  //    "Body->World" = T*R 이지만,
  //    "World->Body" = (Body->World)^(-1).

  Eigen::Matrix4d T_world_body = Eigen::Matrix4d::Identity();

  // 회전 행렬(Body->World) = Rz(yaw)*Ry(pitch)*Rx(roll)
  Eigen::Matrix3d R;
  R = (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

  // Body->World 변환행렬
  Eigen::Matrix4d T_body_world = Eigen::Matrix4d::Identity();
  T_body_world.block<3,3>(0,0) = R;
  T_body_world(0,3) = current_pose.position.x;
  T_body_world(1,3) = current_pose.position.y;
  T_body_world(2,3) = current_pose.position.z;

  // World->Body = (Body->World)^-1
  T_world_body = T_body_world.inverse();

  // (2) Path 각 점을 World->Body 로 변환
  for(const auto &pose_stamped : path_in_global.poses)
  {
    // 3D 좌표(동차좌표 4x1)
    Eigen::Vector4d p_world;
    p_world << pose_stamped.pose.position.x,
               pose_stamped.pose.position.y,
               pose_stamped.pose.position.z,
               1.0;

    Eigen::Vector4d pos_body = T_world_body * p_world;

    // Orientation도 회전
    // yaw만 중요하다면, Global yaw - 기체 yaw 로 대체 가능
    tf2::Quaternion qg(
      pose_stamped.pose.orientation.x,
      pose_stamped.pose.orientation.y,
      pose_stamped.pose.orientation.z,
      pose_stamped.pose.orientation.w
    );

    double r0, p0, y0;
    tf2::Matrix3x3(qg).getRPY(r0, p0, y0);

    // Body 좌표계에서의 yaw = y0 - (기체의 yaw)
    // 정확히 하려면 World->Body 회전행렬에 qg를 적용해야 하나,
    // 여기서는 “상대 yaw”만 필요하다는 가정으로 간단화:
    double r_body = r0 - roll;
    double p_body = p0 - pitch;
    double y_body = y0 - yaw;

    // 정규화
    r_body = normalize_angle(r_body);
    p_body = normalize_angle(p_body);
    y_body = normalize_angle(y_body);

    // 다시 쿼터니언
    tf2::Quaternion qb;
    qb.setRPY(r_body, p_body, y_body);

    geometry_msgs::msg::PoseStamped pose_in_body;
    pose_in_body.header = path_on_body.header;
    pose_in_body.pose.position.x = pos_body.x();
    pose_in_body.pose.position.y = pos_body.y();
    pose_in_body.pose.position.z = pos_body.z();
    pose_in_body.pose.orientation = tf2::toMsg(qb);

    path_on_body.poses.push_back(pose_in_body);
  }

  return path_on_body;
}

/**
 * @brief  두 각도 from, to(라디안) 사이의 최소 각도 차를 반환한다. 
 *         반환 범위는 [-π, π] 이다.
 * @param  from 시작 각도(라디안)
 * @param  to   목표 각도(라디안)
 * @return 두 각도 사이의 최소 차이(라디안)
 */
double LocalPlanner::shortestAngularDistance(double from, double to)
{
  // 2π로 모듈로 연산
  double difference = std::fmod(to - from, 2.0 * M_PI);

  // difference가 [-π, π] 범위 밖이면 조정
  if (difference > M_PI)
  {
    difference -= 2.0 * M_PI;
  }
  else if (difference < -M_PI)
  {
    difference += 2.0 * M_PI;
  }

  return difference;
}

/**
 * @brief  주어진 angle(라디안)을 [-π, π) 범위로 정규화한다.
 * @param  angle 라디안 값
 * @return [-π, π) 범위로 정규화된 라디안 값
 */
double LocalPlanner::normalize_angle(double angle)
{
  // angle을 2π로 나눈 나머지 (fmod는 부동소수점 나머지 연산)
  double result = std::fmod(angle, 2.0 * M_PI);

  // C++의 fmod는 음수 결과가 나올 수 있으므로 보정
  if (result < 0.0)
  {
    result += 2.0 * M_PI;
  }

  // 이제 result는 [0, 2π) 범위
  // [0, π) 구간은 그대로, [π, 2π) 구간은 -2π 해주어 [-π, π)로 맞춘다.
  if (result >= M_PI)
  {
    result -= 2.0 * M_PI;
  }

  return result; // 결과 범위: [-π, π)
}