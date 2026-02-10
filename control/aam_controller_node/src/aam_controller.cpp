#include <aam_controller/aam_controller.h>
#include <chrono>

using namespace std::chrono_literals;
#define INFINITE 999999

AAMController::AAMController(const std::string& node_name) : Node(node_name)
{
  this->Timer = this->create_wall_timer(10ms, std::bind(&AAMController::TimerCallback, this));
  this->pubCmdVel = this->create_publisher<geometry_msgs::msg::TwistStamped>("/aam/cmd_vel", ai3ct::common::constants::QOS_CONTROL_CMD_OVERRIDE);
  this->pubControlPose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/PoseStamped/control_point", ai3ct::common::constants::QOS_CONTROL_CMD_OVERRIDE);

  this->subJoy = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", ai3ct::common::constants::QOS_CONTROL_CMD_OVERRIDE, std::bind(&AAMController::joyCallback, this, std::placeholders::_1));
  this->subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry/msfs", ai3ct::common::constants::QOS_EGO_ODOMETRY, std::bind(&AAMController::odomCallback, this, std::placeholders::_1));
  this->subPath = this->create_subscription<nav_msgs::msg::Path>(
      "/Path/on_body", ai3ct::common::constants::QOS_EGO_ODOMETRY, std::bind(&AAMController::pathCallback, this, std::placeholders::_1));
  this->subMarkers = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/Markers/ready_to_hover", ai3ct::common::constants::QOS_EGO_ODOMETRY,
    std::bind(&AAMController::markersCallback, this, std::placeholders::_1));

  this->subTargetSpeed = this->create_subscription<std_msgs::msg::Float64>(
    "/Float64/target_speed", ai3ct::common::constants::QOS_EGO_ODOMETRY,
    std::bind(&AAMController::targetSpeedCallback, this, std::placeholders::_1));

  ParamHandler = this->add_on_set_parameters_callback(
    std::bind(&AAMController::paramCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Mode2 Flight Controller node has been started.");

  getParams();
};
AAMController::~AAMController(){};

void AAMController::getParams()
{
  this->declare_parameter<double>("hover.pid_speed.kp", double(1.0));
  this->declare_parameter<double>("hover.pid_speed.ki", double(0.0));
  this->declare_parameter<double>("hover.pid_speed.kd", double(0.0));
  this->declare_parameter<double>("hover.pid_altitude.kp", double(1.0));
  this->declare_parameter<double>("hover.pid_altitude.ki", double(0.0));
  this->declare_parameter<double>("hover.pid_altitude.kd", double(0.0));
  this->declare_parameter<double>("hover.pid_x.kp", double(1.0));
  this->declare_parameter<double>("hover.pid_x.ki", double(0.0));
  this->declare_parameter<double>("hover.pid_x.kd", double(0.0));
  this->declare_parameter<double>("hover.pid_y.kp", double(1.0));
  this->declare_parameter<double>("hover.pid_y.ki", double(0.0));
  this->declare_parameter<double>("hover.pid_y.kd", double(0.0));
  this->declare_parameter<double>("hover.pid_heading.kp", double(1.0));
  this->declare_parameter<double>("hover.pid_heading.ki", double(0.0));
  this->declare_parameter<double>("hover.pid_heading.kd", double(0.0));

  this->declare_parameter<double>("flight.pid_speed.kp", double(1.0));
  this->declare_parameter<double>("flight.pid_speed.ki", double(0.0));
  this->declare_parameter<double>("flight.pid_speed.kd", double(0.0));
  this->declare_parameter<double>("flight.pid_altitude.kp", double(1.0));
  this->declare_parameter<double>("flight.pid_altitude.ki", double(0.0));
  this->declare_parameter<double>("flight.pid_altitude.kd", double(0.0));
  // this->declare_parameter<double>("flight.pid_x.kp", double(1.0));
  // this->declare_parameter<double>("flight.pid_x.ki", double(0.0));
  // this->declare_parameter<double>("flight.pid_x.kd", double(0.0));
  this->declare_parameter<double>("flight.pid_y.kp", double(1.0));
  this->declare_parameter<double>("flight.pid_y.ki", double(0.0));
  this->declare_parameter<double>("flight.pid_y.kd", double(0.0));
  this->declare_parameter<double>("flight.pid_heading.kp", double(1.0));
  this->declare_parameter<double>("flight.pid_heading.ki", double(0.0));
  this->declare_parameter<double>("flight.pid_heading.kd", double(0.0));
  this->declare_parameter<double>("flight.pid_pitch.kp", double(1.0));
  this->declare_parameter<double>("flight.pid_pitch.ki", double(0.0));
  this->declare_parameter<double>("flight.pid_pitch.kd", double(0.0));
  this->declare_parameter<double>("flight.pid_roll.kp", double(1.0));
  this->declare_parameter<double>("flight.pid_roll.ki", double(0.0));
  this->declare_parameter<double>("flight.pid_roll.kd", double(0.0));

  this->declare_parameter<int>("look_ahead_idx", int(50));
  this->declare_parameter<double>("ref_control_dist", double(0.0));
  this->declare_parameter<double>("hover_max_speed", double(0.0));
  this->declare_parameter<double>("hover_to_flight_speed", double(0.0));
  this->declare_parameter<double>("flight_max_speed", double(0.0));
  this->declare_parameter<double>("throttle_low_pass_filter_alpha", double(0.0));
  this->declare_parameter<double>("flight_pitch_low_pass_filter_alpha", double(0.0));
  
  // mode switch
  this->declare_parameter<double>("mode_switch_z_thres", double(0.0));
  this->declare_parameter<double>("mode_switch_speed_thres", double(0.0));
  this->declare_parameter<double>("mode_switch_lat_thres", double(0.0));
  this->declare_parameter<double>("stable_time_thres", double(0.0));
  this->declare_parameter<double>("hover_mode_switch_dist_thres", double(0.0));
  this->declare_parameter<bool>("flight_mode_on", bool(0.0));


  // get params
  double hover_kp_speed = get_parameter("hover.pid_speed.kp").as_double();
  double hover_ki_speed = get_parameter("hover.pid_speed.ki").as_double();
  double hover_kd_speed = get_parameter("hover.pid_speed.kd").as_double();

  double hover_kp_alt = get_parameter("hover.pid_altitude.kp").as_double();
  double hover_ki_alt = get_parameter("hover.pid_altitude.ki").as_double();
  double hover_kd_alt = get_parameter("hover.pid_altitude.kd").as_double();

  double hover_kp_x = get_parameter("hover.pid_x.kp").as_double();
  double hover_ki_x = get_parameter("hover.pid_x.ki").as_double();
  double hover_kd_x = get_parameter("hover.pid_x.kd").as_double();

  double hover_kp_y = get_parameter("hover.pid_y.kp").as_double();
  double hover_ki_y = get_parameter("hover.pid_y.ki").as_double();
  double hover_kd_y = get_parameter("hover.pid_y.kd").as_double();

  double hover_kp_heading = get_parameter("hover.pid_heading.kp").as_double();
  double hover_ki_heading = get_parameter("hover.pid_heading.ki").as_double();
  double hover_kd_heading = get_parameter("hover.pid_heading.kd").as_double();

  // fixed wing mode
  double flight_kp_speed = get_parameter("flight.pid_speed.kp").as_double();
  double flight_ki_speed = get_parameter("flight.pid_speed.ki").as_double();
  double flight_kd_speed = get_parameter("flight.pid_speed.kd").as_double();

  double flight_kp_alt = get_parameter("flight.pid_altitude.kp").as_double();
  double flight_ki_alt = get_parameter("flight.pid_altitude.ki").as_double();
  double flight_kd_alt = get_parameter("flight.pid_altitude.kd").as_double();

  // double flight_kp_x = get_parameter("flight.pid_x.kp").as_double();
  // double flight_ki_x = get_parameter("flight.pid_x.ki").as_double();
  // double flight_kd_x = get_parameter("flight.pid_x.kd").as_double();

  double flight_kp_y = get_parameter("flight.pid_y.kp").as_double();
  double flight_ki_y = get_parameter("flight.pid_y.ki").as_double();
  double flight_kd_y = get_parameter("flight.pid_y.kd").as_double();

  double flight_kp_heading = get_parameter("flight.pid_heading.kp").as_double();
  double flight_ki_heading = get_parameter("flight.pid_heading.ki").as_double();
  double flight_kd_heading = get_parameter("flight.pid_heading.kd").as_double();

  double flight_kp_pitch = get_parameter("flight.pid_pitch.kp").as_double();
  double flight_ki_pitch = get_parameter("flight.pid_pitch.ki").as_double();
  double flight_kd_pitch = get_parameter("flight.pid_pitch.kd").as_double();

  double flight_kp_roll = get_parameter("flight.pid_roll.kp").as_double();
  double flight_ki_roll = get_parameter("flight.pid_roll.ki").as_double();
  double flight_kd_roll = get_parameter("flight.pid_roll.kd").as_double();


  m_look_ahead_idx = get_parameter("look_ahead_idx").as_int();
  m_ref_control_dist = get_parameter("ref_control_dist").as_double();
  m_hover_max_speed = get_parameter("hover_max_speed").as_double();
  m_hover_to_flight_speed = get_parameter("hover_to_flight_speed").as_double();
  m_flight_max_speed = get_parameter("flight_max_speed").as_double();
  m_flight_mode_on = get_parameter("flight_mode_on").as_bool();

  double throttle_low_pass_filter_alpha = get_parameter("throttle_low_pass_filter_alpha").as_double();
  double flight_pitch_low_pass_filter_alpha = get_parameter("flight_pitch_low_pass_filter_alpha").as_double();


  m_stable_time_thres = get_parameter("stable_time_thres").as_double();
  m_mode_switch_z_thres = get_parameter("mode_switch_z_thres").as_double();
  m_mode_switch_speed_thres = get_parameter("mode_switch_speed_thres").as_double();
  m_mode_switch_lat_thres = get_parameter("mode_switch_lat_thres").as_double();
  m_hover_mode_switch_dist_thres = get_parameter("hover_mode_switch_dist_thres").as_double();

  double max_look_ahead_dist = static_cast<double>(m_look_ahead_idx) * 2.0; // 2.0 is a just naive variable.

  //Hovering mode 
  m_hover_pid_speed = std::make_unique<PIDController>(hover_kp_speed, hover_ki_speed, hover_kd_speed,  1.0, -1.0, "hover_speed");
  m_hover_pid_altitude = std::make_unique<PIDController>(hover_kp_alt, hover_ki_alt, hover_kd_alt,  1.0, -1.0, "hover_alt");
  m_hover_pid_longi_pos        = std::make_unique<PIDController>(hover_kp_x,  hover_ki_x,  hover_kd_x,   max_look_ahead_dist, -max_look_ahead_dist,  "hover_x");
  m_hover_pid_lat_pos        = std::make_unique<PIDController>(hover_kp_y,  hover_ki_y,  hover_kd_y,   1.0, -1.0,  "hover_y");
  m_hover_pid_heading  = std::make_unique<PIDController>(hover_kp_heading, hover_ki_heading, hover_kd_heading,  1.0, -1.0, "hover_heading");

  m_flight_pid_speed = std::make_unique<PIDController>(flight_kp_speed, flight_ki_speed, flight_kd_speed,  1.0, -1.0, "flight_speed");

  m_flight_pid_altitude = std::make_unique<PIDController>(flight_kp_alt, flight_ki_alt, flight_kd_alt,  0.52, -0.52, "flight_alt"); //0.52 : max pitch angle
  m_flight_pid_pitch  = std::make_unique<PIDController>(flight_kp_pitch, flight_ki_pitch, flight_kd_pitch,  1.0, -1.0, "flight_pitch");

  // m_flight_pid_longi_pos        = std::make_unique<PIDController>(flight_kp_x,  flight_ki_x,  flight_kd_x,   max_look_ahead_dist, -max_look_ahead_dist,  "flight_x");
  m_flight_pid_heading  = std::make_unique<PIDController>(flight_kp_heading, flight_ki_heading, flight_kd_heading,  1.0, -1.0, "flight_heading");

  m_flight_pid_lat_pos        = std::make_unique<PIDController>(flight_kp_y,  flight_ki_y,  flight_kd_y,   1.0, -1.0,  "flight_y");
  m_flight_pid_roll  = std::make_unique<PIDController>(flight_kp_roll, flight_ki_roll, flight_kd_roll,  1.0, -1.0, "flight_roll");


  m_throttle_low_pass_filter  = std::make_unique<LowPassFilter>(throttle_low_pass_filter_alpha, 0.5); //0.5 is idle position
  m_flight_pitch_low_pass_filter  = std::make_unique<LowPassFilter>(throttle_low_pass_filter_alpha, 0.0); //[-1.0 ~ 1.0]
}

rcl_interfaces::msg::SetParametersResult 
AAMController::paramCallback(const std::vector<rclcpp::Parameter> &params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for(const auto & param : params)
  {
    // 1) flight_max_speed
    if(param.get_name() == "flight_max_speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      double new_val = param.as_double();
      m_flight_max_speed = new_val;
      RCLCPP_INFO(this->get_logger(),
                  "Param flight_max_speed updated to %.2f", new_val);
    }
    // 2) flight_mode_on
    else if(param.get_name() == "flight_mode_on" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      bool new_mode = param.as_bool();
      m_flight_mode_on = new_mode;
      RCLCPP_INFO(this->get_logger(),
                  "Param flight_mode_on updated to %s", (new_mode ? "true" : "false"));
    }
  }

  return result;
}

void AAMController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  this->m_odom = *msg;
  this->m_ego_airspeed = msg->twist.twist.linear.x;
  this->m_ego_vertical_speed = msg->twist.twist.linear.z;
}

void AAMController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  this->m_path = *msg;
}

void AAMController::markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  // landing_points_를 갱신
  // MarkerArray에 들어있는 Marker들의 pose.position을 저장
  m_landing_mode_switch_pos.clear();  // 여기서는 매번 새로 clear 후 재저장

  for(const auto & marker : msg->markers)
  {
    // marker.pose.position 에 위치 정보가 있음
    geometry_msgs::msg::Point pt;
    pt.x = marker.pose.position.x;
    pt.y = marker.pose.position.y;
    pt.z = marker.pose.position.z;

    // 필요하다면 marker.ns, marker.id 등을 체크해서
    // "landing"에 해당하는 것만 추려낼 수도 있음
    // 여기서는 일단 전부 저장
    m_landing_mode_switch_pos.push_back(pt);
  }

  // RCLCPP_INFO(this->get_logger(), 
  //   "Received MarkerArray, total %zu markers, stored %zu landing points.",
  //   msg->markers.size(), landing_points_.size());
}

void AAMController::targetSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  this->m_target_speed = msg->data;
}


void AAMController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
    // joy_msg->axes 의 인덱스:
    //  [0]: 왼스틱 좌우 -> Yaw
    //  [1]: 왼스틱 상하 -> Throttle
    //  [3]: 오른스틱 좌우 -> Roll
    //  [4]: 오른스틱 상하 -> Pitch
    //
    // (필요시 부호 조정하거나, [2], [5] (L2, R2) 등 다른 버튼/축 사용 가능)

    double yaw      = 0.0;
    double throttle = 0.0;
    double roll     = 0.0;
    double pitch    = 0.0;

    // Joy 메시지의 axes 범위: [-1.0, 1.0]
    // 왼쪽 스틱 -> yaw, throttle
    if (joy_msg->axes.size() > 0) yaw      = joy_msg->axes[0];  // 좌우
    if (joy_msg->axes.size() > 1) throttle = joy_msg->axes[1];  // 상하

    // 오른쪽 스틱 -> roll, pitch
    if (joy_msg->axes.size() > 3) roll  = joy_msg->axes[3];     // 좌우
    if (joy_msg->axes.size() > 4) pitch = joy_msg->axes[4];     // 상하

    // 필요하다면 위/아래 반전, 좌/우 반전 등을 적용
    // 예) pitch = -joy_msg->axes[4];  // 드론을 위해서는 일반적으로 스틱 앞으로 밀 때 + 로 처리할 수도 있음.

    // geometry_msgs::msg::Twist 메시지 생성
    m_manual_cmd_msg.header.stamp = this->now();
    m_manual_cmd_msg.header.frame_id = "base_link";

    // 일반적으로 드론은
    //  - linear.x -> pitch (앞/뒤 움직임)
    //  - linear.y -> roll  (좌/우 움직임)
    //  - linear.z -> throttle (상/하)
    //  - angular.z -> yaw (제자리에서 회전)
    m_manual_cmd_msg.twist.linear.z  = (throttle + 1.0) * 0.5;
    m_manual_cmd_msg.twist.angular.y  = -1.0 * pitch;
    m_manual_cmd_msg.twist.angular.x  = -1.0 * roll;
    m_manual_cmd_msg.twist.angular.z = -1.0 * yaw;


    // mode switch auto <---> manual
    if(joy_msg->buttons[0] == 1)
      m_auto_mode = true;

    if(joy_msg->buttons[1] == 1)
      m_auto_mode = false;

}


void AAMController::TimerCallback()
{
  static auto prev_time = std::chrono::high_resolution_clock::now();

  auto cur_time = std::chrono::high_resolution_clock::now();
  m_dt = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_time - prev_time).count() / 1e9;
  prev_time = cur_time;

  if(m_path.poses.empty())
  {
    RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for reference path ...");
    return;
  }  

  // get control point
  int adaptive_look_ahead_idx;
  if(m_flight_mode_on)
  {
    adaptive_look_ahead_idx = m_look_ahead_idx * 2;
  }
  else
  {
    adaptive_look_ahead_idx = m_look_ahead_idx;
  }
  this->getRefControlPose(adaptive_look_ahead_idx);
  if(this->getXYFittingAndControlPose(adaptive_look_ahead_idx))
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Fitting is done");
  }


  this->checkLandingMode();

  // PID controller
  this->control();


  if(m_auto_mode)
    pubCmdVel->publish(m_auto_cmd_msg);
  else 
    pubCmdVel->publish(m_manual_cmd_msg);

  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "dt: %f [msec]", m_dt);
}


void AAMController::control()
{
  // 1) 현재 시각, dt 계산 (간단 예시)
  static auto prev_time = now();
  auto cur_time = now();
  double dt = (cur_time - prev_time).seconds();
  prev_time = cur_time;

  // 보호
  if(dt <= 0.0) return;

  double roll_cur, pitch_cur, yaw_cur;
  tf2::Quaternion q_cur(
    m_odom.pose.pose.orientation.x,
    m_odom.pose.pose.orientation.y,
    m_odom.pose.pose.orientation.z,
    m_odom.pose.pose.orientation.w
  );
  tf2::Matrix3x3(q_cur).getRPY(roll_cur, pitch_cur, yaw_cur);

  // 2) 목표값 : on the local coordinates. 
  double x_ref = m_control_pose.position.x;
  double y_ref = m_control_pose.position.y;
  double z_ref = m_control_pose.position.z;
  double roll_ref, pitch_ref, yaw_ref;
  tf2::Quaternion q_ref(
    m_control_pose.orientation.x,
    m_control_pose.orientation.y,
    m_control_pose.orientation.z,
    m_control_pose.orientation.w
  );
  tf2::Matrix3x3(q_ref).getRPY(roll_ref, pitch_ref, yaw_ref);

  // 4) 오차 계산
  double error_x = x_ref;
  double error_y = y_ref; //changed y_ref to cross_track_error 
  double error_z = z_ref; ////m_odom.pose.pose.position.z; //z_ref;
  double error_yaw = yaw_ref; 
  double error_speed = 0.0;

  // 7) 모드 전환 (10mps 기준)
  geometry_msgs::msg::Twist cmd_out;
  if(m_ego_airspeed < m_hover_max_speed * 1.1 && !m_flight_mode_on) //1.1 for margin
  {
    // Mode 2 기준
    // Hovering 모드
    // linear.z : throttle. 0.5 is the idle throttling. if you want to hover at static position, throttle should be 0.5
    // angluar.z : yaw
    // angular.y : vertical translation (elevator)
    // angular.x : lateral translation

    // Normalize if needed
    // 5) PID 업데이트
    double ctrl_altitude    = m_hover_pid_altitude->update(error_z, dt);
    double ctrl_y    = m_hover_pid_lat_pos->update(error_y, dt);
    double ctrl_yaw  = m_hover_pid_heading->update(error_yaw, dt);

    // 1) 위치 오차 → 목표 속도
    double position_error = x_ref;
    double desired_speed = m_hover_pid_longi_pos->update(position_error, dt);

    desired_speed = std::clamp(desired_speed, -m_hover_max_speed, m_hover_max_speed);
    // 2) 속도 오차 → 스로틀
    error_speed = desired_speed - m_ego_airspeed;
    double pid_val = m_hover_pid_speed->update(error_speed, dt);

    cmd_out.angular.x = ctrl_y; 
    cmd_out.angular.y = ctrl_altitude;
    cmd_out.angular.z = ctrl_yaw; 

    pid_val = std::clamp(pid_val, -1.0, +1.0);
    double throttle = 0.5 + 0.5 * pid_val;
    throttle = std::clamp(throttle, 0.0, 1.0);  

    // // 데드 존 설정
    // double dead_zone_half = 0.05; // 0.5 ± 0.05
    // double lower_bound = 0.5 - dead_zone_half; // 0.45
    // double upper_bound = 0.5 + dead_zone_half; // 0.55

    // if (throttle >= lower_bound && throttle <= upper_bound)
    // {
    //   // 데드 존이면 0.5로 고정
    //   throttle = 0.5;
    // }
    double filtered_output = m_throttle_low_pass_filter->filter(throttle);
    cmd_out.linear.z = filtered_output;

    // if altitude error is low, then throttle is activated.
    // cmd_out.linear.z = std::fabs(error_z) < 2.0 ? filtered_output : 0.5;
  }
  else if(m_flight_mode_on)
  {
    // Mode 2 기준
    // Fixed wing 모드
    // linear.z : throttle. 0.5 is the zero acceleration. acceleration is mapped between -0.3g ~ 0.3g
    // angluar.z : rudder(yaw)
    // angular.y : elevator
    // angular.x : aileron

    // 외부루프: Altitude -> Desired pitch
    double desired_pitch = -1.0 * m_flight_pid_altitude->update(error_z, dt); // -1.0 : ned to enu
    // 내부루프: Pitch -> Elevator
    double error_pitch = desired_pitch - pitch_cur;
    double ctrl_elevator = -1.0 * m_flight_pid_pitch->update(error_pitch, dt); // -1.0 : ned to enu

    std::cout << "desired_pitch : " << desired_pitch << ", cur : "<< pitch_cur << ", error p :" << error_pitch << std::endl;
    // cmd_out.angular.y = elevator
    cmd_out.angular.y = ctrl_elevator;

    double ctrl_yaw  = m_flight_pid_heading->update(error_yaw, dt);

    // 외부루프: lateral error -> Desired roll
    double desired_roll = -1.0 * m_flight_pid_lat_pos->update(error_y, dt);
    // 내부루프: roll -> aileron
    double error_roll = desired_roll - roll_cur;
    double ctrl_aileron = -1.0 * m_flight_pid_roll->update(error_roll, dt) + ctrl_yaw; //-1.0 : ned to enu

    std::cout << "desired_roll : " << desired_roll << ", cur : "<< roll_cur << ", error r :" << error_roll << std::endl;


    // 1) 위치 오차 → 목표 속도
    // double position_error = x_ref;
    // double desired_speed = m_flight_pid_longi_pos->update(position_error, dt);
    double desired_speed = std::clamp(m_target_speed, -m_flight_max_speed, m_flight_max_speed);
    // 2) 속도 오차 → 스로틀
    error_speed = desired_speed - m_ego_airspeed;
    double pid_val = m_flight_pid_speed->update(error_speed, dt);

    cmd_out.angular.x = ctrl_aileron; //ctrl_aileron; 
    cmd_out.angular.z = ctrl_yaw; 

    pid_val = std::clamp(pid_val, -1.0, +1.0);
    double throttle = 0.5 + 0.5 * pid_val;
    throttle = std::clamp(throttle, 0.0, 1.0);  

    // // 데드 존 설정
    // double dead_zone_half = 0.05; // 0.5 ± 0.05
    // double lower_bound = 0.5 - dead_zone_half; // 0.45
    // double upper_bound = 0.5 + dead_zone_half; // 0.55

    // if (throttle >= lower_bound && throttle <= upper_bound)
    // {
    //   // 데드 존이면 0.5로 고정
    //   throttle = 0.5;
    // }
    double filtered_output = m_throttle_low_pass_filter->filter(throttle);
    cmd_out.linear.z = filtered_output;

  }

  m_auto_cmd_msg.header.stamp = this->now();
  m_auto_cmd_msg.header.frame_id = "base_link";
  m_auto_cmd_msg.twist.angular.x = -1.0 * cmd_out.angular.x;
  m_auto_cmd_msg.twist.angular.y = cmd_out.angular.y;
  m_auto_cmd_msg.twist.angular.z = -1.0 * cmd_out.angular.z; //yaw motion is negative
  m_auto_cmd_msg.twist.linear.z = cmd_out.linear.z;

  std::cout  << "----" << std::endl;
  std::cout << "m_flight_mode_on : " << m_flight_mode_on << std::endl;
  std::cout  << "error_x : " <<  error_x << std::endl;
  std::cout  << "error_y : " <<  error_y << std::endl;
  std::cout  << "error_z : " <<  error_z << std::endl;
  std::cout  << "error_yaw : " <<  error_yaw << std::endl;
  std::cout  << "target_speed : " << m_target_speed << "error_speed : " <<  error_speed << std::endl;

  std::cout  << "angular.x : " <<  cmd_out.angular.x << std::endl;
  std::cout  << "angular.y : " <<  cmd_out.angular.y << std::endl;
  std::cout  << "angular.z : " <<  cmd_out.angular.z << std::endl;
  std::cout  << "linear.z : " <<  cmd_out.linear.z << std::endl;

  if(fabs(error_z) < m_mode_switch_z_thres && 
     fabs(error_speed) < m_mode_switch_speed_thres &&
     fabs(error_y) < m_mode_switch_lat_thres)
  {
    m_count_stable_time += dt;
  }
  else
  {
    m_count_stable_time -= dt;
  }
  m_count_stable_time = std::clamp(m_count_stable_time, 0.0, m_stable_time_thres + 2.0); //  

  if(m_count_stable_time > m_stable_time_thres && !m_ready_to_land)
  {
    m_flight_mode_on = true;
  }

  std::cout << "stable_time : " << m_count_stable_time << std::endl;
}

void AAMController::getRefControlPose(const int& look_ahead_idx)
{
  if(m_path.poses.size() > look_ahead_idx)
  {
    m_control_pose = m_path.poses[look_ahead_idx].pose;
  }
  else
  {
    int last_idx = m_path.poses.size() - 1;
    m_control_pose = m_path.poses[last_idx].pose;
  }

  // visualize the control pose
  geometry_msgs::msg::PoseStamped control_pose_msg;
  control_pose_msg.header.stamp = this->now();
  control_pose_msg.header.frame_id = "base_link";
  control_pose_msg.pose = m_control_pose;

  this->pubControlPose->publish(control_pose_msg);
}



void AAMController::checkLandingMode()
{
  // (1) 현재 위치
  double x_cur = m_odom.pose.pose.position.x;
  double y_cur = m_odom.pose.pose.position.y;
  double z_cur = m_odom.pose.pose.position.z;

  // (2) m_landing_mode_switch_pos가 비어 있으면 아무것도 안 함
  if(m_landing_mode_switch_pos.empty()) {
    return;
  }

  // (3) 가장 가까운 landing point까지의 거리 계산
  double min_dist = 1e9;
  for(const auto & pt : m_landing_mode_switch_pos)
  {
    double dx = pt.x - x_cur;
    double dy = pt.y - y_cur;
    double dz = pt.z - z_cur;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if(dist < min_dist)
    {
      min_dist = dist;
    }
  }

  // (4) 임계값과 비교
  if(min_dist < m_hover_mode_switch_dist_thres)
  {
    // 모드 스위치
    m_flight_mode_on = false;
    m_ready_to_land = true;
    RCLCPP_INFO(this->get_logger(), 
      "Switching to Landing Mode! (dist=%.2f < threshold=%.2f)",
      min_dist, m_hover_mode_switch_dist_thres);
  }
}


bool AAMController::getXYFittingAndControlPose(const int &look_ahead_idx)
{
  // std::cout <<  m_path.poses.size() << std::endl;


  int start_idx = 0;
  int end_idx   = std::min((int)m_path.poses.size() - 1, look_ahead_idx);

  // 2) xs, ys 추출
  std::vector<double> xs, ys;
  xs.reserve(end_idx - start_idx + 1);
  ys.reserve(end_idx - start_idx + 1);

  for(int i = start_idx; i <= end_idx; i++)
  {
    double px = m_path.poses[i].pose.position.x;
    double py = m_path.poses[i].pose.position.y;
    xs.push_back(px);
    ys.push_back(py);
  }

  int N = xs.size();
  // 3) A: Nx4, b: Nx1  ( 3차 -> 4계수: a0, a1, a2, a3 )
  Eigen::MatrixXd A(N, 4);
  Eigen::VectorXd b(N);
  for(int i = 0; i < N; i++)
  {
    double xx = xs[i];
    A(i,0) = 1.0;        // a0
    A(i,1) = xx;         // a1 * x
    A(i,2) = xx*xx;      // a2 * x^2
    A(i,3) = xx*xx*xx;   // a3 * x^3
    b(i)   = ys[i];
  }

  // 4) 최소제곱 해: coeff = [a0, a1, a2, a3]
  Eigen::Vector4d coeff = (A.transpose() * A).ldlt().solve(A.transpose() * b);
  double a0 = coeff(0);
  double a1 = coeff(1);
  double a2 = coeff(2);
  double a3 = coeff(3);

  // 6) cross track err (간단)
  double x_ref = m_ref_control_dist;
  double y_ref = a0 + a1*x_ref + a2*x_ref*x_ref + a3*x_ref*x_ref*x_ref;
  double crosstrack_err = y_ref; // 부호없이 거리만 보고 싶다면 fabs()

  // heading_ref = arctan(dy/dx) 
  // dy/dx = a1 + 2*a2*x + 3*a3*x^2
  double dydx = a1 + 2.0*a2*x_ref + 3.0*a3*x_ref*x_ref;
  double heading_ref = std::atan2(dydx, 1.0);

  m_crosstrack_error = crosstrack_err;
  m_heading_ref = heading_ref;
  
  // // 7) 컨트롤 포즈 설정
  // geometry_msgs::msg::Pose control_pose;
  // control_pose.position.x = x_ref;
  // control_pose.position.y = y_ref;
  // control_pose.position.z = 0.0; // 예시

  // // orientation
  // tf2::Quaternion q;
  // q.setRPY(0, 0, heading_ref);
  // control_pose.orientation = tf2::toMsg(q);

  // m_control_pose = control_pose;

  // // 8) control_pose 토픽 퍼블리시
  // geometry_msgs::msg::PoseStamped control_pose_msg;
  // control_pose_msg.header.stamp = this->now();
  // control_pose_msg.header.frame_id = "map"; 
  // control_pose_msg.pose = m_control_pose;
  // pubControlPose->publish(control_pose_msg);

  // // [디버그] 로그
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  //   "[CubicFit] crosstrack=%.2f, heading=%.2f deg, a3=%.5f",
  //   crosstrack_err, heading_ref*180.0/M_PI, a3);

  // //===============================
  // // [Visualize the cubic curve]
  // //===============================
  // // 1) min_x, max_x 구하기
  // double min_x = xs.front();
  // double max_x = xs.back();
  // if(min_x > max_x) std::swap(min_x, max_x);

  // // 2) Marker
  // visualization_msgs::msg::Marker poly_marker;
  // poly_marker.header.frame_id = "map";
  // poly_marker.header.stamp = this->now();
  // poly_marker.ns = "cubic_fitted_poly";
  // poly_marker.id = 0;
  // poly_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  // poly_marker.action = visualization_msgs::msg::Marker::ADD;

  // // 라인 두께
  // poly_marker.scale.x = 0.07;
  // // 색상(주황)
  // poly_marker.color.r = 1.0f;
  // poly_marker.color.g = 0.6f;
  // poly_marker.color.b = 0.0f;
  // poly_marker.color.a = 1.0f;

  // // 3) x를 샘플링
  // int sample_count = 50;
  // double dx = (max_x - min_x) / (sample_count - 1);
  // if(dx < 0.001) dx = 0.001;

  // for(int i=0; i<sample_count; i++)
  // {
  //   double xx = min_x + i*dx;
  //   double yy = a0 + a1*xx + a2*xx*xx + a3*xx*xx*xx;
  //   geometry_msgs::msg::Point pt;
  //   pt.x = xx;
  //   pt.y = yy;
  //   pt.z = 0.0;
  //   poly_marker.points.push_back(pt);
  // }

  // // 4) 퍼블리시
  // m_poly_marker_pub_->publish(poly_marker);

  return true;
}