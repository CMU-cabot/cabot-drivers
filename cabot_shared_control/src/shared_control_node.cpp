// Copyright (c) 2026  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "cabot_shared_control/shared_control_node.hpp"

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace
{
constexpr uint32_t kAxisStateIdle = 1;
constexpr uint32_t kAxisStateClosedLoopControl = 8;
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr int8_t kSharedControlModeNormal = 0;
constexpr int8_t kSharedControlModeSpeedOverlay = 1;
constexpr int8_t kSharedControlModeShared = 2;
constexpr int8_t kSharedControlModeSharedTorque = 3;
constexpr int8_t kSharedControlModeFree = 4;
constexpr uint32_t kControlModeTorque = 1;
constexpr uint32_t kInputModePassthrough = 1;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double moveToward(double current, double target, double max_delta)
{
  if (current < target) {
    return std::min(current + max_delta, target);
  }
  return std::max(current - max_delta, target);
}

bool isAssistMode(int8_t mode)
{
  return mode == kSharedControlModeSpeedOverlay ||
         mode == kSharedControlModeShared ||
         mode == kSharedControlModeSharedTorque;
}

double signedDeadband(double value, double threshold)
{
  if (std::abs(value) <= threshold) {
    return 0.0;
  }
  return std::copysign(std::abs(value) - threshold, value);
}

double squaredDistanceToSegment(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double vv = vx * vx + vy * vy;
  if (vv <= 1.0e-12) {
    const double dx = px - ax;
    const double dy = py - ay;
    return dx * dx + dy * dy;
  }
  const double t = clamp((wx * vx + wy * vy) / vv, 0.0, 1.0);
  const double cx = ax + t * vx;
  const double cy = ay + t * vy;
  const double dx = px - cx;
  const double dy = py - cy;
  return dx * dx + dy * dy;
}
}  // namespace

namespace cabot_shared_control
{
namespace
{
constexpr char kImuTopic[] = "/imu/data";
constexpr char kSharedControlModeTopic[] = "/shared_control_mode";
constexpr char kSharedTorqueAssistEnabledTopic[] = "/shared_torque_assist_enabled";
constexpr char kPauseControlTopic[] = "/cabot/pause_control";
constexpr char kCmdVelTopic[] = "/cabot/cmd_vel";
constexpr char kAutonomyCmdTopic[] = "/autonomy/cmd_vel";
constexpr char kEventTopic[] = "/cabot/event";
constexpr char kScanTopic[] = "/scan";
constexpr char kFootprintTopic[] = "/footprint";
constexpr char kOdomTopic[] = "/cabot/odom_raw";
constexpr char kFootprintFrame[] = "base_footprint";
}  // namespace

SharedControlNode::SharedControlNode()
: Node("shared_control_node")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const int initial_shared_control_mode =
    this->declare_parameter<int>("shared_control_mode", static_cast<int>(kSharedControlModeNormal));
  if (
    initial_shared_control_mode == kSharedControlModeNormal ||
    initial_shared_control_mode == kSharedControlModeSpeedOverlay ||
    initial_shared_control_mode == kSharedControlModeShared ||
    initial_shared_control_mode == kSharedControlModeFree ||
    initial_shared_control_mode == kSharedControlModeSharedTorque)
  {
    shared_control_mode_ = static_cast<int8_t>(initial_shared_control_mode);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "invalid shared_control_mode=%d at startup (expected: 0=normal, 1=speed_overlay, 2=shared, 3=shared_torque, 4=free), fallback to 0",
      initial_shared_control_mode);
    shared_control_mode_ = kSharedControlModeNormal;
  }
  use_imu_ = this->declare_parameter<bool>("use_imu", true);
  shared_torque_assist_enabled_ =
    this->declare_parameter<bool>("shared_torque_assist_enabled", false);

  wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.0855);
  wheel_separation_m_ = this->declare_parameter<double>("wheel_separation_m", 0.139);
  left_wheel_sign_ = this->declare_parameter<double>("left_wheel_sign", -1.0);
  right_wheel_sign_ = this->declare_parameter<double>("right_wheel_sign", 1.0);
  odrive_velocity_is_turns_per_sec_ =
    this->declare_parameter<bool>("odrive_velocity_is_turns_per_sec", true);

  control_mode_ = this->declare_parameter<int>("control_mode", 2);
  input_mode_ = this->declare_parameter<int>("input_mode", 1);
  request_closed_loop_on_startup_ =
    this->declare_parameter<bool>("request_closed_loop_on_startup", true);

  observer_gain_x_ = this->declare_parameter<double>("observer_gain_x", 10.0);
  observer_gain_z_ = this->declare_parameter<double>("observer_gain_z", 10.0);
  robot_mass_x_ = this->declare_parameter<double>("robot_mass_x", 24.0);
  robot_inertia_z_ = this->declare_parameter<double>("robot_inertia_z", 2.2);
  robot_damping_x_ = this->declare_parameter<double>("robot_damping_x", 40.0);
  robot_damping_z_ = this->declare_parameter<double>("robot_damping_z", 5.0);
  coulomb_friction_x_ = this->declare_parameter<double>("coulomb_friction_x", 8.0);
  coulomb_friction_z_ = this->declare_parameter<double>("coulomb_friction_z", 0.8);
  friction_smoothing_linear_ = this->declare_parameter<double>("friction_smoothing_linear", 0.05);
  friction_smoothing_angular_ = this->declare_parameter<double>("friction_smoothing_angular", 0.05);
  use_torque_estimate_ = this->declare_parameter<bool>("use_torque_estimate", true);
  motor_torque_constant_ = this->declare_parameter<double>("motor_torque_constant", 0.06);

  use_gravity_compensation_ = this->declare_parameter<bool>("use_gravity_compensation", true);
  gravity_comp_gain_ = this->declare_parameter<double>("gravity_comp_gain", 1.0);
  use_imu_linear_accel_ = this->declare_parameter<bool>("use_imu_linear_accel", false);
  imu_accel_blend_ = this->declare_parameter<double>("imu_accel_blend", 0.2);
  if (!use_imu_) {
    // IMU is disabled by configuration. Assume horizontal terrain.
    use_gravity_compensation_ = false;
    use_imu_linear_accel_ = false;
    imu_pitch_rad_ = 0.0;
    imu_accel_x_ = 0.0;
  }

  desired_mass_x_ = this->declare_parameter<double>("desired_mass_x", 11.0);
  desired_inertia_z_ = this->declare_parameter<double>("desired_inertia_z", 0.9);
  desired_damping_x_ = this->declare_parameter<double>("desired_damping_x", 24.0);
  desired_damping_z_ = this->declare_parameter<double>("desired_damping_z", 2.8);
  torque_assist_damping_x_ = this->declare_parameter<double>("torque_assist_damping_x", 24.0);
  torque_assist_damping_z_ = this->declare_parameter<double>("torque_assist_damping_z", 2.8);
  torque_assist_input_scale_ =
    this->declare_parameter<double>("torque_assist_input_scale", 0.25);
  torque_assist_torque_scale_ =
    this->declare_parameter<double>("torque_assist_torque_scale", 0.0);
  torque_assist_max_force_forward_ =
    this->declare_parameter<double>("torque_assist_max_force_forward", 20.0);
  torque_assist_max_force_reverse_ =
    this->declare_parameter<double>("torque_assist_max_force_reverse", 45.0);
  torque_assist_max_torque_z_ =
    this->declare_parameter<double>("torque_assist_max_torque_z", 2.5);
  torque_obstacle_stop_distance_m_ =
    this->declare_parameter<double>("torque_obstacle_stop_distance_m", 0.35);
  torque_obstacle_pushback_stiffness_ =
    this->declare_parameter<double>("torque_obstacle_pushback_stiffness", 70.0);
  torque_obstacle_pushback_max_force_ =
    this->declare_parameter<double>("torque_obstacle_pushback_max_force", 22.5);
  torque_obstacle_force_filter_alpha_ =
    this->declare_parameter<double>("torque_obstacle_force_filter_alpha", 0.12);
  torque_obstacle_brake_velocity_deadband_ =
    this->declare_parameter<double>("torque_obstacle_brake_velocity_deadband", 0.03);

  human_force_weight_ = this->declare_parameter<double>("human_force_weight", 1.0);
  autonomy_force_weight_ = this->declare_parameter<double>("autonomy_force_weight", 0.0);
  autonomy_virtual_stiffness_x_ =
    this->declare_parameter<double>("autonomy_virtual_stiffness_x", 45.0);
  autonomy_virtual_stiffness_z_ =
    this->declare_parameter<double>("autonomy_virtual_stiffness_z", 12.0);
  autonomy_timeout_sec_ = this->declare_parameter<double>("autonomy_timeout_sec", 0.3);
  human_force_x_sign_ = this->declare_parameter<double>("human_force_x_sign", -1.0);
  human_torque_z_sign_ = this->declare_parameter<double>("human_torque_z_sign", -1.0);
  force_deadband_x_ = this->declare_parameter<double>("force_deadband_x", 4.0);
  force_deadband_z_ = this->declare_parameter<double>("force_deadband_z", 1.2);
  local_speed_delta_max_forward_ =
    this->declare_parameter<double>("local_speed_delta_max_forward", 0.10);
  local_speed_delta_max_reverse_ =
    this->declare_parameter<double>("local_speed_delta_max_reverse", 0.10);
  local_speed_delta_gain_ = this->declare_parameter<double>("local_speed_delta_gain", 0.20);
  local_speed_delta_decay_ = this->declare_parameter<double>("local_speed_delta_decay", 0.25);
  push_pull_engage_threshold_ =
    this->declare_parameter<double>("push_pull_engage_threshold", 1.5);
  push_pull_release_threshold_ =
    this->declare_parameter<double>("push_pull_release_threshold", 0.75);
  speed_event_hold_sec_ = this->declare_parameter<double>("speed_event_hold_sec", 0.7);
  speed_event_retrigger_sec_ =
    this->declare_parameter<double>("speed_event_retrigger_sec", 1.0);
  creep_speed_max_ = this->declare_parameter<double>("creep_speed_max", 0.08);
  creep_force_threshold_ = this->declare_parameter<double>("creep_force_threshold", 2.0);
  speed_event_yaw_rate_threshold_ =
    this->declare_parameter<double>("speed_event_yaw_rate_threshold", 0.35);
  speedup_min_tracking_ratio_ =
    this->declare_parameter<double>("speedup_min_tracking_ratio", 0.5);

  loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 100.0);
  status_timeout_sec_ = this->declare_parameter<double>("status_timeout_sec", 0.2);
  cmd_vel_timeout_sec_ = this->declare_parameter<double>("cmd_vel_timeout_sec", 0.2);
  axis_state_request_interval_sec_ =
    this->declare_parameter<double>("axis_state_request_interval_sec", 0.5);
  max_acc_ = this->declare_parameter<double>("max_acc", 0.5);
  max_dec_ = this->declare_parameter<double>("max_dec", -0.5);
  const double max_linear_velocity_default =
    this->declare_parameter<double>("max_linear_velocity", 0.32);
  max_linear_velocity_forward_ = this->declare_parameter<double>(
    "max_linear_velocity_forward", 1.0);
  max_linear_velocity_reverse_ = this->declare_parameter<double>(
    "max_linear_velocity_reverse", 0.3);
  max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 1.0);
  const double max_linear_accel_default =
    this->declare_parameter<double>("max_linear_accel", 0.48);
  max_linear_accel_forward_ = this->declare_parameter<double>(
    "max_linear_accel_forward", 0.5);
  max_linear_accel_reverse_ = this->declare_parameter<double>(
    "max_linear_accel_reverse", 0.5);
  max_angular_accel_ = this->declare_parameter<double>("max_angular_accel", 1.5);
  obstacle_guard_enabled_ = this->declare_parameter<bool>("obstacle_guard_enabled", true);
  obstacle_guard_reverse_enabled_ =
    this->declare_parameter<bool>("obstacle_guard_reverse_enabled", false);
  obstacle_stop_distance_m_ = this->declare_parameter<double>("obstacle_stop_distance_m", 0.0);
  obstacle_slowdown_margin_m_ =
    this->declare_parameter<double>("obstacle_slowdown_margin_m", 0.6);
  obstacle_min_speed_scale_ = this->declare_parameter<double>("obstacle_min_speed_scale", 0.2);
  obstacle_pushback_enabled_ = this->declare_parameter<bool>("obstacle_pushback_enabled", true);
  obstacle_pushback_stiffness_ =
    this->declare_parameter<double>("obstacle_pushback_stiffness", 60.0);
  obstacle_pushback_max_force_ =
    this->declare_parameter<double>("obstacle_pushback_max_force", 30.0);
  obstacle_timeout_sec_ = this->declare_parameter<double>("obstacle_timeout_sec", 0.3);
  obstacle_point_min_z_ = this->declare_parameter<double>("obstacle_point_min_z", -0.3);
  obstacle_point_max_z_ = this->declare_parameter<double>("obstacle_point_max_z", 1.2);
  strict_frame_match_ = this->declare_parameter<bool>("strict_frame_match", true);
  sensor_guard_enabled_ = this->declare_parameter<bool>("sensor_guard_enabled", true);
  sensor_guard_half_width_m_ = this->declare_parameter<double>("sensor_guard_half_width_m", 0.35);

  const auto now = this->get_clock()->now();
  left_feedback_.stamp = now;
  right_feedback_.stamp = now;
  latest_autonomy_stamp_ = now;
  latest_cmd_vel_stamp_ = now;
  last_axis_state_request_stamp_ = now;
  last_step_stamp_ = now;
  footprint_stamp_ = now;
  obstacle_stamp_ = now;
  const auto event_ready_stamp =
    now - rclcpp::Duration::from_seconds(std::max(speed_event_retrigger_sec_, 0.0));
  last_speedup_event_stamp_ = event_ready_stamp;
  last_speeddown_event_stamp_ = event_ready_stamp;
  speed_saturation_start_stamp_ = now;

  ctrl_pub_left_ = this->create_publisher<odrive_can::msg::ControlMessage>(
    "/" + axis0_ns_ + "/control_message", 10);
  ctrl_pub_right_ = this->create_publisher<odrive_can::msg::ControlMessage>(
    "/" + axis1_ns_ + "/control_message", 10);
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/shared_control/external_wrench",
    10);
  cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("/shared_control/cmd_vel", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, 10);
  event_pub_ = this->create_publisher<std_msgs::msg::String>(kEventTopic, 10);

  left_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    "/" + axis0_ns_ + "/controller_status", 50,
    std::bind(&SharedControlNode::onAxis0Status, this, std::placeholders::_1));
  right_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    "/" + axis1_ns_ + "/controller_status", 50,
    std::bind(&SharedControlNode::onAxis1Status, this, std::placeholders::_1));
  if (use_imu_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      kImuTopic, 50, std::bind(&SharedControlNode::onImu, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "IMU input disabled (use_imu=false). Assuming horizontal terrain and turning off IMU-based compensation.");
  }
  shared_control_mode_sub_ = this->create_subscription<std_msgs::msg::Int8>(
    kSharedControlModeTopic, 20,
    std::bind(&SharedControlNode::onSharedControlMode, this, std::placeholders::_1));
  shared_torque_assist_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    kSharedTorqueAssistEnabledTopic, 20,
    std::bind(&SharedControlNode::onSharedTorqueAssistEnabled, this, std::placeholders::_1));
  pause_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    kPauseControlTopic, 20,
    std::bind(&SharedControlNode::onPauseControl, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    kCmdVelTopic, 20, std::bind(&SharedControlNode::onCmdVel, this, std::placeholders::_1));
  autonomy_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    kAutonomyCmdTopic, 20,
    std::bind(&SharedControlNode::onAutonomyCmd, this, std::placeholders::_1));
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
    kFootprintTopic, 10, std::bind(&SharedControlNode::onFootprint, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    kScanTopic, rclcpp::SensorDataQoS(),
    std::bind(&SharedControlNode::onScan, this, std::placeholders::_1));

  axis0_client_ = this->create_client<odrive_can::srv::AxisState>(
    "/" + axis0_ns_ + "/request_axis_state");
  axis1_client_ = this->create_client<odrive_can::srv::AxisState>(
    "/" + axis1_ns_ + "/request_axis_state");
  if (request_closed_loop_on_startup_) {
    startup_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&SharedControlNode::requestClosedLoopIfReady, this));
  }

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(loop_rate_hz_, 1.0)),
    std::bind(&SharedControlNode::controlStep, this));

  RCLCPP_INFO(
    this->get_logger(),
    "started: axis0=/%s axis1=/%s loop=%.1fHz imu=%s obstacle_guard=%s stop_distance=%.2fm torque_assist_default=%s",
    axis0_ns_.c_str(), axis1_ns_.c_str(), loop_rate_hz_,
    use_imu_ ? "true" : "false",
    obstacle_guard_enabled_ ? "true" : "false",
    obstacle_stop_distance_m_,
    shared_torque_assist_enabled_ ? "on" : "off");
}

void SharedControlNode::onAxis0Status(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
  left_feedback_.msg = *msg;
  left_feedback_.stamp = this->get_clock()->now();
  left_feedback_.received = true;
}

void SharedControlNode::onAxis1Status(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
  right_feedback_.msg = *msg;
  right_feedback_.stamp = this->get_clock()->now();
  right_feedback_.received = true;
}

void SharedControlNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!use_imu_) {
    return;
  }

  imu_accel_x_ = static_cast<double>(msg->linear_acceleration.x);

  if (msg->orientation_covariance[0] < 0.0) {
    return;
  }
  const double x = static_cast<double>(msg->orientation.x);
  const double y = static_cast<double>(msg->orientation.y);
  const double z = static_cast<double>(msg->orientation.z);
  const double w = static_cast<double>(msg->orientation.w);
  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    imu_pitch_rad_ = std::copysign(kPi / 2.0, sinp);
  } else {
    imu_pitch_rad_ = std::asin(sinp);
  }
}

void SharedControlNode::onAutonomyCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_autonomy_cmd_ = *msg;
  latest_autonomy_stamp_ = this->get_clock()->now();
}

void SharedControlNode::onSharedControlMode(const std_msgs::msg::Int8::SharedPtr msg)
{
  const int8_t mode = msg->data;
  if (
    mode != kSharedControlModeNormal &&
    mode != kSharedControlModeSpeedOverlay &&
    mode != kSharedControlModeShared &&
    mode != kSharedControlModeFree &&
    mode != kSharedControlModeSharedTorque)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "ignore invalid shared_control_mode=%d (expected: 0=normal, 1=speed_overlay, 2=shared, 3=shared_torque, 4=free)",
      static_cast<int>(mode));
    return;
  }
  if (shared_control_mode_ != mode) {
    const int8_t prev_mode = shared_control_mode_;
    RCLCPP_INFO(
      this->get_logger(),
      "shared_control_mode changed: %d -> %d",
      static_cast<int>(shared_control_mode_),
      static_cast<int>(mode));
    shared_control_mode_ = mode;

    // Entering an assist mode after any other mode can leave stale observer and command
    // state behind. Reset and immediately publish a zero command in the new mode.
    if (isAssistMode(shared_control_mode_)) {
      resetSharedState();
      publishZeroCommandForCurrentMode();
      requestAxisState(true, true);
      RCLCPP_INFO(
        this->get_logger(),
        "assist mode entry reset applied (prev_mode=%d)",
        static_cast<int>(prev_mode));
    }
  }
}

void SharedControlNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  latest_cmd_vel_stamp_ = this->get_clock()->now();
}

void SharedControlNode::onSharedTorqueAssistEnabled(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool enabled = msg->data;
  if (shared_torque_assist_enabled_ == enabled) {
    return;
  }

  const bool prev_enabled = shared_torque_assist_enabled_;
  shared_torque_assist_enabled_ = enabled;
  RCLCPP_INFO(
    this->get_logger(),
    "shared_torque assist changed: %s -> %s",
    prev_enabled ? "on" : "off",
    enabled ? "on" : "off");

  if (shared_control_mode_ == kSharedControlModeSharedTorque) {
    command_v_ = 0.0;
    command_wz_ = 0.0;
    publishZeroCommandForCurrentMode();
  }
}

void SharedControlNode::onPauseControl(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool pause = msg->data;
  if (pause_control_ != pause) {
    const bool prev_pause = pause_control_;
    pause_control_ = pause;
    RCLCPP_INFO(
      this->get_logger(),
      "pause_control changed: %s -> %s (mode=%d, closed_loop=%s)",
      prev_pause ? "true" : "false",
      pause ? "true" : "false",
      static_cast<int>(shared_control_mode_),
      desiredClosedLoopControl() ? "on" : "off");
  }
}

void SharedControlNode::onFootprint(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  footprint_points_.clear();
  footprint_points_.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    footprint_points_.push_back(XYPoint{static_cast<double>(p.x), static_cast<double>(p.y)});
  }

  // geometry_msgs/Polygon has no frame information.
  // Assume this footprint is represented in base_footprint.
  footprint_frame_id_ = kFootprintFrame;
  footprint_stamp_ = this->get_clock()->now();
  footprint_received_ = footprint_points_.size() >= 3;
}

void SharedControlNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!obstacle_guard_enabled_) {
    return;
  }
  if (!footprint_received_ && !sensor_guard_enabled_) {
    return;
  }

  geometry_msgs::msg::TransformStamped scan_to_footprint;
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "scan frame is empty, cannot transform to %s", footprint_frame_id_.c_str());
    return;
  }
  try {
    scan_to_footprint = tf_buffer_->lookupTransform(
      footprint_frame_id_,
      msg->header.frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "failed to transform scan frame %s to %s: %s",
      msg->header.frame_id.c_str(),
      footprint_frame_id_.c_str(),
      ex.what());
    return;
  }

  const auto & tr = scan_to_footprint.transform.translation;
  const auto & rot = scan_to_footprint.transform.rotation;
  const tf2::Quaternion q(rot.x, rot.y, rot.z, rot.w);
  const tf2::Vector3 t(tr.x, tr.y, tr.z);

  double front_clearance = std::numeric_limits<double>::infinity();
  double rear_clearance = std::numeric_limits<double>::infinity();
  bool has_valid_point = false;
  double front_clearance_sensor = std::numeric_limits<double>::infinity();
  double rear_clearance_sensor = std::numeric_limits<double>::infinity();
  bool has_sensor_guard_point = false;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double range = static_cast<double>(msg->ranges[i]);
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < static_cast<double>(msg->range_min) || range > static_cast<double>(msg->range_max)) {
      continue;
    }

    const double angle =
      static_cast<double>(msg->angle_min) + static_cast<double>(i) * static_cast<double>(msg->angle_increment);
    const double x_scan = range * std::cos(angle);
    const double y_scan = range * std::sin(angle);
    const tf2::Vector3 scan_point(x_scan, y_scan, 0.0);
    const tf2::Vector3 footprint_point = tf2::quatRotate(q, scan_point) + t;
    const double x = footprint_point.x();
    const double y = footprint_point.y();

    has_valid_point = true;
    if (footprint_received_) {
      const double clearance = distancePointToFootprint(x, y);
      if (x >= 0.0) {
        front_clearance = std::min(front_clearance, clearance);
      } else {
        rear_clearance = std::min(rear_clearance, clearance);
      }
    }

    if (sensor_guard_enabled_ && std::abs(y) <= sensor_guard_half_width_m_) {
      has_sensor_guard_point = true;
      if (x >= 0.0) {
        front_clearance_sensor = std::min(front_clearance_sensor, x);
      } else {
        rear_clearance_sensor = std::min(rear_clearance_sensor, -x);
      }
    }
  }

  if (!has_valid_point) {
    front_clearance = std::numeric_limits<double>::infinity();
    rear_clearance = std::numeric_limits<double>::infinity();
  }
  if (has_sensor_guard_point) {
    front_clearance = std::min(front_clearance, front_clearance_sensor);
    rear_clearance = std::min(rear_clearance, rear_clearance_sensor);
  }

  front_clearance_m_ = front_clearance;
  rear_clearance_m_ = rear_clearance;
  obstacle_stamp_ = this->get_clock()->now();
  obstacle_received_ = true;
}

void SharedControlNode::requestClosedLoopIfReady()
{
  if (!request_closed_loop_on_startup_) {
    return;
  }
  if (!axis0_client_ || !axis1_client_) {
    return;
  }
  if (axis0_client_->service_is_ready() && axis1_client_->service_is_ready()) {
    requestAxisState(desiredClosedLoopControl(), true);
    request_closed_loop_on_startup_ = false;
    if (startup_timer_) {
      startup_timer_->cancel();
    }
  }
}

void SharedControlNode::onAxisStateResponse(
  rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future,
  const std::string & axis_name)
{
  try {
    const auto response = future.get();
    RCLCPP_INFO(
      this->get_logger(),
      "%s state response: axis_state=%u errors=%u result=%u",
      axis_name.c_str(),
      static_cast<unsigned int>(response->axis_state),
      response->active_errors,
      static_cast<unsigned int>(response->procedure_result));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s state request failed: %s",
      axis_name.c_str(),
      ex.what());
  }
}

bool SharedControlNode::cmdVelFresh(const rclcpp::Time & now) const
{
  if (!latest_cmd_vel_.has_value()) {
    return false;
  }
  return (now - latest_cmd_vel_stamp_).seconds() <= cmd_vel_timeout_sec_;
}

double SharedControlNode::slewRate(double current, double target, double accel_limit, double dt) const
{
  const double step = std::max(0.0, accel_limit) * std::max(0.0, dt);
  const double diff = target - current;
  if (std::abs(diff) <= step) {
    return target;
  }
  return current + std::copysign(step, diff);
}

void SharedControlNode::requestAxisState(bool closed_loop, bool force)
{
  if (!axis0_client_ || !axis1_client_) {
    return;
  }

  const auto now = this->get_clock()->now();
  if (!force) {
    if (requested_closed_loop_ == closed_loop) {
      return;
    }
    if ((now - last_axis_state_request_stamp_).seconds() < axis_state_request_interval_sec_) {
      return;
    }
  }

  if (!axis0_client_->service_is_ready() || !axis1_client_->service_is_ready()) {
    return;
  }

  const uint32_t target_state = closed_loop ? kAxisStateClosedLoopControl : kAxisStateIdle;

  auto req0 = std::make_shared<odrive_can::srv::AxisState::Request>();
  req0->axis_requested_state = target_state;
  axis0_client_->async_send_request(
    req0,
    [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future) {
      this->onAxisStateResponse(future, this->axis0_ns_);
    });

  auto req1 = std::make_shared<odrive_can::srv::AxisState::Request>();
  req1->axis_requested_state = target_state;
  axis1_client_->async_send_request(
    req1,
    [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future) {
      this->onAxisStateResponse(future, this->axis1_ns_);
    });

  requested_closed_loop_ = closed_loop;
  last_axis_state_request_stamp_ = now;
}

bool SharedControlNode::desiredClosedLoopControl() const
{
  // Assist modes always keep loop control ON.
  if (isAssistMode(shared_control_mode_)) {
    return true;
  }
  // normal/free modes follow pause_control topic.
  return !pause_control_;
}

void SharedControlNode::updateOdometryFromStatus(const rclcpp::Time & now)
{
  if (!statusValid(left_feedback_, now) || !statusValid(right_feedback_, now)) {
    return;
  }

  const double left_pos =
    left_wheel_sign_ * toSiAngular(static_cast<double>(left_feedback_.msg.pos_estimate));
  const double right_pos =
    right_wheel_sign_ * toSiAngular(static_cast<double>(right_feedback_.msg.pos_estimate));
  const double left_dist = wheel_radius_m_ * left_pos;
  const double right_dist = wheel_radius_m_ * right_pos;

  if (!odom_initialized_) {
    last_left_dist_m_ = left_dist;
    last_right_dist_m_ = right_dist;
    odom_initialized_ = true;
  }

  const double delta_left = left_dist - last_left_dist_m_;
  const double delta_right = right_dist - last_right_dist_m_;
  last_left_dist_m_ = left_dist;
  last_right_dist_m_ = right_dist;

  const double delta_s = 0.5 * (delta_left + delta_right);
  const double delta_yaw = (delta_right - delta_left) / std::max(wheel_separation_m_, 1.0e-6);
  const double mid_yaw = odom_yaw_rad_ + 0.5 * delta_yaw;
  odom_x_m_ += delta_s * std::cos(mid_yaw);
  odom_y_m_ += delta_s * std::sin(mid_yaw);
  odom_yaw_rad_ += delta_yaw;

  const double left_w =
    left_wheel_sign_ * toSiAngular(static_cast<double>(left_feedback_.msg.vel_estimate));
  const double right_w =
    right_wheel_sign_ * toSiAngular(static_cast<double>(right_feedback_.msg.vel_estimate));
  const double v_meas = 0.5 * wheel_radius_m_ * (right_w + left_w);
  const double wz_meas = (wheel_radius_m_ / wheel_separation_m_) * (right_w - left_w);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = odom_x_m_;
  odom.pose.pose.position.y = odom_y_m_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(0.5 * odom_yaw_rad_);
  odom.pose.pose.orientation.w = std::cos(0.5 * odom_yaw_rad_);
  odom.twist.twist.linear.x = v_meas;
  odom.twist.twist.angular.z = wz_meas;
  odom_pub_->publish(odom);
}

bool SharedControlNode::statusValid(const AxisFeedback & feedback, const rclcpp::Time & now) const
{
  if (!feedback.received) {
    return false;
  }
  return (now - feedback.stamp).seconds() <= status_timeout_sec_;
}

bool SharedControlNode::obstacleDataFresh(const rclcpp::Time & now) const
{
  if (!obstacle_guard_enabled_ || !obstacle_received_) {
    return false;
  }
  return (now - obstacle_stamp_).seconds() <= obstacle_timeout_sec_;
}

double SharedControlNode::obstacleApproachScale(double clearance_m, double stop_distance_m) const
{
  if (!std::isfinite(clearance_m)) {
    return 1.0;
  }

  const double stop_distance = std::max(0.0, stop_distance_m);
  const double slowdown_margin = std::max(0.0, obstacle_slowdown_margin_m_);
  const double min_scale = clamp(obstacle_min_speed_scale_, 0.0, 1.0);
  if (clearance_m <= stop_distance) {
    return 0.0;
  }
  if (slowdown_margin <= 1.0e-6) {
    return 1.0;
  }

  const double slowdown_distance = stop_distance + slowdown_margin;
  if (clearance_m >= slowdown_distance) {
    return 1.0;
  }

  const double t = clamp((clearance_m - stop_distance) / slowdown_margin, 0.0, 1.0);
  const double smooth = t * t * (3.0 - 2.0 * t);  // smoothstep
  return min_scale + (1.0 - min_scale) * smooth;
}

double SharedControlNode::estimateWheelTorque(
  const odrive_can::msg::ControllerStatus & status,
  double sign) const
{
  if (use_torque_estimate_) {
    return sign * static_cast<double>(status.torque_estimate);
  }
  return sign * motor_torque_constant_ * static_cast<double>(status.iq_measured);
}

double SharedControlNode::toSiAngular(double odrive_velocity_value) const
{
  if (odrive_velocity_is_turns_per_sec_) {
    return odrive_velocity_value * 2.0 * kPi;
  }
  return odrive_velocity_value;
}

double SharedControlNode::fromSiAngular(double angular_velocity_rad_s) const
{
  if (odrive_velocity_is_turns_per_sec_) {
    return angular_velocity_rad_s / (2.0 * kPi);
  }
  return angular_velocity_rad_s;
}

bool SharedControlNode::pointInsideFootprint(double x, double y) const
{
  if (footprint_points_.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = footprint_points_.size() - 1; i < footprint_points_.size(); j = i++) {
    const auto & pi = footprint_points_[i];
    const auto & pj = footprint_points_[j];
    const bool intersects =
      ((pi.y > y) != (pj.y > y)) &&
      (x < (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1.0e-12) + pi.x);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

double SharedControlNode::distancePointToFootprint(double x, double y) const
{
  if (footprint_points_.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  if (pointInsideFootprint(x, y)) {
    return 0.0;
  }

  double min_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < footprint_points_.size(); ++i) {
    const auto & a = footprint_points_[i];
    const auto & b = footprint_points_[(i + 1) % footprint_points_.size()];
    min_sq = std::min(min_sq, squaredDistanceToSegment(x, y, a.x, a.y, b.x, b.y));
  }
  return std::sqrt(min_sq);
}

void SharedControlNode::resetSharedState()
{
  external_force_x_ = 0.0;
  external_torque_z_ = 0.0;
  filtered_obstacle_force_x_ = 0.0;
  filtered_human_force_x_ = 0.0;
  local_speed_delta_ = 0.0;
  speed_event_hold_direction_ = 0;
  push_pull_direction_ = 0;
  speed_event_neutral_rearmed_ = true;
  command_v_ = 0.0;
  command_wz_ = 0.0;
  last_v_meas_ = 0.0;
  last_wz_meas_ = 0.0;
  last_step_stamp_ = this->get_clock()->now();
}

void SharedControlNode::publishNavigationEvent(const std::string & event_name)
{
  std_msgs::msg::String msg;
  msg.data = event_name;
  event_pub_->publish(msg);
}

void SharedControlNode::updateLocalSpeedControl(
  const rclcpp::Time & now,
  const SharedControlState & state,
  double dt,
  double nav_v,
  double nav_wz)
{
  const double human_force_alpha = clamp(dt * 8.0, 0.0, 1.0);
  filtered_human_force_x_ += human_force_alpha * (state.human_force_x - filtered_human_force_x_);

  const double engage_threshold = std::max(push_pull_engage_threshold_, 0.0);
  const double release_threshold = clamp(push_pull_release_threshold_, 0.0, engage_threshold);
  if (push_pull_direction_ == 0) {
    if (filtered_human_force_x_ >= engage_threshold) {
      push_pull_direction_ = 1;
      speed_event_neutral_rearmed_ = true;
    } else if (filtered_human_force_x_ <= -engage_threshold) {
      push_pull_direction_ = -1;
      speed_event_neutral_rearmed_ = true;
    }
  } else if (std::abs(filtered_human_force_x_) <= release_threshold) {
    push_pull_direction_ = 0;
    speed_event_hold_direction_ = 0;
    speed_event_neutral_rearmed_ = true;
  }

  const bool obstacle_stop = state.obstacle_fresh && state.front_scale <= 1.0e-3;
  const bool front_blocked = state.obstacle_fresh && state.front_scale < 0.999;
  const bool can_forward_creep =
    !obstacle_stop &&
    std::abs(nav_v) < 1.0e-3 &&
    filtered_human_force_x_ >= std::max(creep_force_threshold_, engage_threshold);

  double local_delta_target = local_speed_delta_;
  if (push_pull_direction_ > 0) {
    if (front_blocked && !can_forward_creep) {
      local_delta_target = std::min(local_delta_target, 0.0);
    } else {
      local_delta_target += local_speed_delta_gain_ * filtered_human_force_x_ * dt;
    }
  } else if (push_pull_direction_ < 0) {
    local_delta_target += local_speed_delta_gain_ * filtered_human_force_x_ * dt;
  } else {
    local_speed_delta_ = moveToward(
      local_speed_delta_, 0.0, std::max(local_speed_delta_decay_, 0.0) * dt);
    local_delta_target = local_speed_delta_;
  }

  const double local_forward_limit = can_forward_creep ?
    std::max(0.0, std::min(local_speed_delta_max_forward_, creep_speed_max_)) :
    std::max(0.0, local_speed_delta_max_forward_);
  const double local_reverse_limit =
    std::abs(nav_v) < 1.0e-3 ? 0.0 : std::max(0.0, local_speed_delta_max_reverse_);
  local_speed_delta_ = clamp(local_delta_target, -local_reverse_limit, local_forward_limit);

  const bool saturated_up =
    local_forward_limit > 0.0 &&
    local_speed_delta_ >= (local_forward_limit - 1.0e-4) &&
    push_pull_direction_ > 0;
  const bool saturated_down =
    local_reverse_limit > 0.0 &&
    local_speed_delta_ <= (-local_reverse_limit + 1.0e-4) &&
    push_pull_direction_ < 0;
  const int saturated_direction = saturated_up ? 1 : (saturated_down ? -1 : 0);
  if (saturated_direction != speed_event_hold_direction_) {
    speed_event_hold_direction_ = saturated_direction;
    speed_saturation_start_stamp_ = now;
  }

  if (saturated_direction == 0 || !speed_event_neutral_rearmed_) {
    return;
  }

  const double hold_duration = (now - speed_saturation_start_stamp_).seconds();
  if (hold_duration < speed_event_hold_sec_) {
    return;
  }

  const bool large_yaw_rate =
    std::abs(nav_wz) > std::max(0.0, speed_event_yaw_rate_threshold_);
  const bool speedup_blocked =
    large_yaw_rate ||
    front_blocked ||
    (nav_v > 1.0e-3 &&
    state.v_meas < nav_v * std::max(0.0, speedup_min_tracking_ratio_));

  if (saturated_direction > 0 && speedup_blocked) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "suppress navigation_speedup: human_force=%.3f local_delta=%.3f nav_v=%.3f v_meas=%.3f front_scale=%.3f nav_wz=%.3f",
      filtered_human_force_x_, local_speed_delta_, nav_v, state.v_meas, state.front_scale, nav_wz);
    return;
  }

  const double since_last_event = saturated_direction > 0 ?
    (now - last_speedup_event_stamp_).seconds() :
    (now - last_speeddown_event_stamp_).seconds();
  if (since_last_event < speed_event_retrigger_sec_) {
    return;
  }

  if (saturated_direction > 0) {
    publishNavigationEvent("navigation_speedup");
    last_speedup_event_stamp_ = now;
  } else {
    publishNavigationEvent("navigation_speeddown");
    last_speeddown_event_stamp_ = now;
  }
  speed_event_neutral_rearmed_ = false;
  RCLCPP_INFO(
    this->get_logger(),
    "published %s: human_force=%.3f local_delta=%.3f nav_v=%.3f v_meas=%.3f hold=%.3fs",
    saturated_direction > 0 ? "navigation_speedup" : "navigation_speeddown",
    filtered_human_force_x_, local_speed_delta_, nav_v, state.v_meas, hold_duration);
}

void SharedControlNode::publishWheelVelocities(double left_wheel_rad_s, double right_wheel_rad_s)
{
  odrive_can::msg::ControlMessage left_msg;
  left_msg.control_mode = static_cast<uint32_t>(control_mode_);
  left_msg.input_mode = static_cast<uint32_t>(input_mode_);
  left_msg.input_pos = 0.0F;
  left_msg.input_vel =
    static_cast<float>(left_wheel_sign_ * fromSiAngular(left_wheel_rad_s));
  left_msg.input_torque = 0.0F;

  odrive_can::msg::ControlMessage right_msg;
  right_msg.control_mode = static_cast<uint32_t>(control_mode_);
  right_msg.input_mode = static_cast<uint32_t>(input_mode_);
  right_msg.input_pos = 0.0F;
  right_msg.input_vel =
    static_cast<float>(right_wheel_sign_ * fromSiAngular(right_wheel_rad_s));
  right_msg.input_torque = 0.0F;

  ctrl_pub_left_->publish(left_msg);
  ctrl_pub_right_->publish(right_msg);
}

void SharedControlNode::publishWheelTorques(
  double left_wheel_torque_nm,
  double right_wheel_torque_nm)
{
  odrive_can::msg::ControlMessage left_msg;
  left_msg.control_mode = kControlModeTorque;
  left_msg.input_mode = kInputModePassthrough;
  left_msg.input_pos = 0.0F;
  left_msg.input_vel = 0.0F;
  left_msg.input_torque = static_cast<float>(left_wheel_sign_ * left_wheel_torque_nm);

  odrive_can::msg::ControlMessage right_msg;
  right_msg.control_mode = kControlModeTorque;
  right_msg.input_mode = kInputModePassthrough;
  right_msg.input_pos = 0.0F;
  right_msg.input_vel = 0.0F;
  right_msg.input_torque = static_cast<float>(right_wheel_sign_ * right_wheel_torque_nm);

  ctrl_pub_left_->publish(left_msg);
  ctrl_pub_right_->publish(right_msg);
}

void SharedControlNode::publishStop()
{
  publishWheelVelocities(0.0, 0.0);
}

void SharedControlNode::publishZeroCommandForCurrentMode()
{
  if (shared_control_mode_ == kSharedControlModeSharedTorque) {
    publishWheelTorques(0.0, 0.0);
    return;
  }
  publishStop();
}

void SharedControlNode::publishSharedTelemetry(
  const rclcpp::Time & now,
  double linear_x,
  double angular_z)
{
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = now;
  wrench_msg.header.frame_id = "base_link";
  wrench_msg.wrench.force.x = external_force_x_;
  wrench_msg.wrench.torque.z = external_torque_z_;
  wrench_pub_->publish(wrench_msg);

  geometry_msgs::msg::TwistStamped cmd_msg;
  cmd_msg.header.stamp = now;
  cmd_msg.header.frame_id = "base_link";
  cmd_msg.twist.linear.x = linear_x;
  cmd_msg.twist.angular.z = angular_z;
  cmd_pub_->publish(cmd_msg);
}

void SharedControlNode::controlStepStop(double dt)
{
  external_force_x_ = 0.0;
  external_torque_z_ = 0.0;
  command_v_ = slewRate(
    command_v_, 0.0,
    command_v_ >= 0.0 ? max_linear_accel_forward_ : max_linear_accel_reverse_, dt);
  command_wz_ = slewRate(command_wz_, 0.0, max_angular_accel_, dt);
  const double left_cmd_w =
    (command_v_ - 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  const double right_cmd_w =
    (command_v_ + 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  if (shared_control_mode_ == kSharedControlModeSharedTorque) {
    command_v_ = 0.0;
    command_wz_ = 0.0;
    publishWheelTorques(0.0, 0.0);
    return;
  }
  publishWheelVelocities(left_cmd_w, right_cmd_w);
}

void SharedControlNode::controlStepAutonomy(double dt)
{
  external_force_x_ = 0.0;
  external_torque_z_ = 0.0;
  double target_v = 0.0;
  double target_wz = 0.0;
  const auto now = this->get_clock()->now();
  if (cmdVelFresh(now)) {
    target_v = latest_cmd_vel_->linear.x;
    target_wz = latest_cmd_vel_->angular.z;
  }

  target_v = clamp(target_v, -max_linear_velocity_reverse_, max_linear_velocity_forward_);
  target_wz = clamp(target_wz, -max_angular_velocity_, max_angular_velocity_);
  const double accel_linear = target_v >= command_v_ ? max_linear_accel_forward_ : max_linear_accel_reverse_;
  command_v_ = slewRate(command_v_, target_v, accel_linear, dt);
  command_wz_ = slewRate(command_wz_, target_wz, max_angular_accel_, dt);
  const double left_cmd_w =
    (command_v_ - 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  const double right_cmd_w =
    (command_v_ + 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  publishWheelVelocities(left_cmd_w, right_cmd_w);
}

void SharedControlNode::controlStepFree()
{
  external_force_x_ = 0.0;
  external_torque_z_ = 0.0;
  command_v_ = 0.0;
  command_wz_ = 0.0;
  last_v_meas_ = 0.0;
  last_wz_meas_ = 0.0;
}

bool SharedControlNode::computeSharedControlState(
  const rclcpp::Time & now,
  double dt,
  SharedControlState & state)
{
  if (!statusValid(left_feedback_, now) || !statusValid(right_feedback_, now)) {
    resetSharedState();
    return false;
  }

  const double left_w =
    left_wheel_sign_ * toSiAngular(static_cast<double>(left_feedback_.msg.vel_estimate));
  const double right_w =
    right_wheel_sign_ * toSiAngular(static_cast<double>(right_feedback_.msg.vel_estimate));

  state.v_meas = 0.5 * wheel_radius_m_ * (right_w + left_w);
  state.wz_meas = (wheel_radius_m_ / wheel_separation_m_) * (right_w - left_w);

  const double wheel_a_x = (state.v_meas - last_v_meas_) / dt;
  const double wheel_a_z = (state.wz_meas - last_wz_meas_) / dt;
  last_v_meas_ = state.v_meas;
  last_wz_meas_ = state.wz_meas;

  double a_x = wheel_a_x;
  if (use_imu_linear_accel_) {
    const double alpha = clamp(imu_accel_blend_, 0.0, 1.0);
    a_x = (1.0 - alpha) * wheel_a_x + alpha * imu_accel_x_;
  }

  const double tau_l = estimateWheelTorque(left_feedback_.msg, left_wheel_sign_);
  const double tau_r = estimateWheelTorque(right_feedback_.msg, right_wheel_sign_);

  const double f_motor_x = (tau_l + tau_r) / wheel_radius_m_;
  const double n_motor_z = (wheel_separation_m_ / (2.0 * wheel_radius_m_)) * (tau_r - tau_l);

  const double friction_x = coulomb_friction_x_ *
    std::tanh(state.v_meas / std::max(friction_smoothing_linear_, 1.0e-3));
  const double friction_z = coulomb_friction_z_ *
    std::tanh(state.wz_meas / std::max(friction_smoothing_angular_, 1.0e-3));

  double gravity_force_x = 0.0;
  if (use_gravity_compensation_) {
    gravity_force_x = gravity_comp_gain_ * robot_mass_x_ * kGravity * std::sin(imu_pitch_rad_);
  }

  const double residual_x =
    f_motor_x - robot_mass_x_ * a_x - robot_damping_x_ * state.v_meas - friction_x - gravity_force_x;
  const double residual_z =
    n_motor_z - robot_inertia_z_ * wheel_a_z - robot_damping_z_ * state.wz_meas - friction_z;

  external_force_x_ += dt * observer_gain_x_ * (residual_x - external_force_x_);
  external_torque_z_ += dt * observer_gain_z_ * (residual_z - external_torque_z_);

  const double human_force_x =
    signedDeadband(human_force_x_sign_ * external_force_x_, force_deadband_x_);
  const double human_torque_z =
    signedDeadband(human_torque_z_sign_ * external_torque_z_, force_deadband_z_);
  state.human_force_x = human_force_x;

  double auto_force_x = 0.0;
  double auto_torque_z = 0.0;
  if (
    latest_autonomy_cmd_.has_value() &&
    (now - latest_autonomy_stamp_).seconds() <= autonomy_timeout_sec_)
  {
    const double v_ref = static_cast<double>(latest_autonomy_cmd_->twist.linear.x);
    const double wz_ref = static_cast<double>(latest_autonomy_cmd_->twist.angular.z);
    auto_force_x = autonomy_virtual_stiffness_x_ * (v_ref - state.v_meas);
    auto_torque_z = autonomy_virtual_stiffness_z_ * (wz_ref - state.wz_meas);
  }

  const bool torque_assist_active =
    shared_control_mode_ != kSharedControlModeSharedTorque || shared_torque_assist_enabled_;
  const bool shared_torque_mode = shared_control_mode_ == kSharedControlModeSharedTorque;
  const double obstacle_stop_distance =
    shared_torque_mode ? torque_obstacle_stop_distance_m_ : obstacle_stop_distance_m_;
  const double obstacle_pushback_stiffness =
    shared_torque_mode ? torque_obstacle_pushback_stiffness_ : obstacle_pushback_stiffness_;
  const double obstacle_pushback_max_force =
    shared_torque_mode ? torque_obstacle_pushback_max_force_ : obstacle_pushback_max_force_;
  const double torque_force_input_scale =
    shared_torque_mode ? clamp(torque_assist_input_scale_, 0.0, 1.0) : 1.0;
  const double torque_torque_input_scale =
    shared_torque_mode ? clamp(torque_assist_torque_scale_, 0.0, 1.0) : 1.0;
  const double blended_force_x =
    torque_force_input_scale *
    (human_force_weight_ * human_force_x + autonomy_force_weight_ * auto_force_x);
  const double blended_torque_z =
    torque_torque_input_scale *
    (human_force_weight_ * human_torque_z + autonomy_force_weight_ * auto_torque_z);
  const double base_shared_force_x = torque_assist_active ? blended_force_x : 0.0;
  state.shared_force_x = base_shared_force_x;
  state.shared_torque_z = torque_assist_active ? blended_torque_z : 0.0;

  state.obstacle_fresh = obstacleDataFresh(now);
  state.front_scale = state.obstacle_fresh ?
    obstacleApproachScale(front_clearance_m_, obstacle_stop_distance) : 1.0;
  state.rear_scale = (state.obstacle_fresh && obstacle_guard_reverse_enabled_) ?
    obstacleApproachScale(rear_clearance_m_, obstacle_stop_distance) : 1.0;
  const bool front_intent = shared_torque_mode ? (human_force_x > 0.0) : (state.shared_force_x > 0.0);
  const bool rear_intent = shared_torque_mode ? (human_force_x < 0.0) : (state.shared_force_x < 0.0);

  // Distance-aware slowdown: reduce drive intent while approaching obstacles.
  if (state.shared_force_x > 0.0) {
    state.shared_force_x *= state.front_scale;
  } else if (state.shared_force_x < 0.0 && obstacle_guard_reverse_enabled_) {
    state.shared_force_x *= state.rear_scale;
  }

  if (state.obstacle_fresh && obstacle_pushback_enabled_) {
    // Apply virtual spring force only against the intended moving direction.
    if (front_clearance_m_ <= obstacle_stop_distance && front_intent) {
      const double penetration = obstacle_stop_distance - front_clearance_m_;
      const double pushback = clamp(
        obstacle_pushback_stiffness * penetration, 0.0, obstacle_pushback_max_force);
      state.shared_force_x -= pushback;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "front obstacle pushback: clearance=%.3fm pushback=%.2fN",
        front_clearance_m_, pushback);
    } else {
      if (
        obstacle_guard_reverse_enabled_ && rear_clearance_m_ <= obstacle_stop_distance &&
        rear_intent)
      {
        const double penetration = obstacle_stop_distance - rear_clearance_m_;
        const double pushback = clamp(
          obstacle_pushback_stiffness * penetration, 0.0, obstacle_pushback_max_force);
        state.shared_force_x += pushback;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rear obstacle pushback: clearance=%.3fm pushback=%.2fN",
          rear_clearance_m_, pushback);
      }
    }
  }

  const double obstacle_force_target =
    shared_torque_mode && !front_intent && !rear_intent ?
    0.0 : (state.shared_force_x - base_shared_force_x);
  const double obstacle_force_alpha = shared_torque_mode ?
    clamp(torque_obstacle_force_filter_alpha_, 0.0, 1.0) : 1.0;
  filtered_obstacle_force_x_ += obstacle_force_alpha *
    (obstacle_force_target - filtered_obstacle_force_x_);
  state.shared_force_x = base_shared_force_x + filtered_obstacle_force_x_;

  return true;
}

void SharedControlNode::controlStepSpeedOverlay(const rclcpp::Time & now, double dt)
{
  SharedControlState state;
  if (!computeSharedControlState(now, dt, state)) {
    publishZeroCommandForCurrentMode();
    return;
  }

  double nav_v = 0.0;
  double nav_wz = 0.0;
  if (cmdVelFresh(now)) {
    nav_v = latest_cmd_vel_->linear.x;
    nav_wz = latest_cmd_vel_->angular.z;
  }

  updateLocalSpeedControl(now, state, dt, nav_v, nav_wz);

  double target_v = nav_v + local_speed_delta_;
  const bool allow_forward_creep =
    std::abs(nav_v) < 1.0e-3 &&
    filtered_human_force_x_ >= std::max(creep_force_threshold_, push_pull_engage_threshold_) &&
    (!state.obstacle_fresh || state.front_scale > 1.0e-3);
  if (std::abs(nav_v) < 1.0e-3) {
    target_v = clamp(target_v, 0.0, allow_forward_creep ? creep_speed_max_ : max_linear_velocity_forward_);
  }

  target_v = clamp(target_v, -max_linear_velocity_reverse_, max_linear_velocity_forward_);
  nav_wz = clamp(nav_wz, -max_angular_velocity_, max_angular_velocity_);
  const double accel_linear = target_v >= command_v_ ?
    max_linear_accel_forward_ : max_linear_accel_reverse_;
  command_v_ = slewRate(command_v_, target_v, accel_linear, dt);
  command_wz_ = slewRate(command_wz_, nav_wz, max_angular_accel_, dt);

  // Final safety clamp: reduce approaching speed smoothly in obstacle slowdown
  // margin, then stop only inside stop_distance.
  if (state.obstacle_fresh) {
    const double front_limit = max_linear_velocity_forward_ * state.front_scale;
    if (command_v_ > front_limit) {
      command_v_ = front_limit;
    } else if (command_v_ > 0.0 && state.front_scale < 1.0) {
      const double slowdown_brake = max_linear_accel_forward_ * (1.0 - state.front_scale);
      command_v_ = std::max(0.0, command_v_ - slowdown_brake * dt);
      command_v_ = std::min(command_v_, front_limit);
    }
    if (state.front_scale <= 1.0e-3 && command_v_ > 0.0) {
      command_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "front approach blocked: clearance %.3fm <= %.3fm",
        front_clearance_m_, obstacle_stop_distance_m_);
    }

    if (obstacle_guard_reverse_enabled_) {
      const double rear_limit = max_linear_velocity_reverse_ * state.rear_scale;
      if (command_v_ < -rear_limit) {
        command_v_ = -rear_limit;
      } else if (command_v_ < 0.0 && state.rear_scale < 1.0) {
        const double slowdown_brake = max_linear_accel_reverse_ * (1.0 - state.rear_scale);
        command_v_ = std::min(0.0, command_v_ + slowdown_brake * dt);
        command_v_ = std::max(command_v_, -rear_limit);
      }
      if (state.rear_scale <= 1.0e-3 && command_v_ < 0.0) {
        command_v_ = 0.0;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rear approach blocked: clearance %.3fm <= %.3fm",
          rear_clearance_m_, obstacle_stop_distance_m_);
      }
    }

    if (state.front_scale < 0.999 || state.rear_scale < 0.999) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 200,
        "obstacle slowdown: front_clearance=%.3f front_scale=%.3f rear_clearance=%.3f rear_scale=%.3f command_v=%.3f",
        front_clearance_m_, state.front_scale, rear_clearance_m_, state.rear_scale, command_v_);
    }
    if (state.front_scale <= 1.0e-3 || state.rear_scale <= 1.0e-3) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "obstacle stop active: front_clearance=%.3f rear_clearance=%.3f stop_distance=%.3f",
        front_clearance_m_, rear_clearance_m_, obstacle_stop_distance_m_);
    }
  }

  const double left_cmd_w =
    (command_v_ - 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  const double right_cmd_w =
    (command_v_ + 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  publishWheelVelocities(left_cmd_w, right_cmd_w);
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 250,
    "speed overlay: human_force=%.3f filtered_force=%.3f nav_v=%.3f local_delta=%.3f target_v=%.3f v_meas=%.3f",
    state.human_force_x, filtered_human_force_x_, nav_v, local_speed_delta_, target_v, state.v_meas);
  publishSharedTelemetry(now, command_v_, command_wz_);
}

void SharedControlNode::controlStepShared(const rclcpp::Time & now, double dt)
{
  SharedControlState state;
  if (!computeSharedControlState(now, dt, state)) {
    publishZeroCommandForCurrentMode();
    return;
  }

  double cmd_accel_x =
    (state.shared_force_x - desired_damping_x_ * command_v_) / std::max(desired_mass_x_, 1.0e-6);
  double cmd_accel_z =
    (state.shared_torque_z - desired_damping_z_ * command_wz_) / std::max(desired_inertia_z_, 1.0e-6);
  cmd_accel_x = clamp(cmd_accel_x, -max_linear_accel_reverse_, max_linear_accel_forward_);
  cmd_accel_z = clamp(cmd_accel_z, -max_angular_accel_, max_angular_accel_);

  command_v_ += cmd_accel_x * dt;
  command_wz_ += cmd_accel_z * dt;
  command_v_ = clamp(command_v_, -max_linear_velocity_reverse_, max_linear_velocity_forward_);
  command_wz_ = clamp(command_wz_, -max_angular_velocity_, max_angular_velocity_);

  if (state.obstacle_fresh) {
    const double front_limit = max_linear_velocity_forward_ * state.front_scale;
    if (command_v_ > front_limit) {
      command_v_ = front_limit;
    } else if (command_v_ > 0.0 && state.front_scale < 1.0) {
      const double slowdown_brake = max_linear_accel_forward_ * (1.0 - state.front_scale);
      command_v_ = std::max(0.0, command_v_ - slowdown_brake * dt);
      command_v_ = std::min(command_v_, front_limit);
    }
    if (state.front_scale <= 1.0e-3 && command_v_ > 0.0) {
      command_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "front approach blocked: clearance %.3fm <= %.3fm",
        front_clearance_m_, obstacle_stop_distance_m_);
    }

    if (obstacle_guard_reverse_enabled_) {
      const double rear_limit = max_linear_velocity_reverse_ * state.rear_scale;
      if (command_v_ < -rear_limit) {
        command_v_ = -rear_limit;
      } else if (command_v_ < 0.0 && state.rear_scale < 1.0) {
        const double slowdown_brake = max_linear_accel_reverse_ * (1.0 - state.rear_scale);
        command_v_ = std::min(0.0, command_v_ + slowdown_brake * dt);
        command_v_ = std::max(command_v_, -rear_limit);
      }
      if (state.rear_scale <= 1.0e-3 && command_v_ < 0.0) {
        command_v_ = 0.0;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rear approach blocked: clearance %.3fm <= %.3fm",
          rear_clearance_m_, obstacle_stop_distance_m_);
      }
    }

    if (state.front_scale < 0.999 || state.rear_scale < 0.999) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 200,
        "obstacle slowdown: front_clearance=%.3f front_scale=%.3f rear_clearance=%.3f rear_scale=%.3f command_v=%.3f",
        front_clearance_m_, state.front_scale, rear_clearance_m_, state.rear_scale, command_v_);
    }
    if (state.front_scale <= 1.0e-3 || state.rear_scale <= 1.0e-3) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "obstacle stop active: front_clearance=%.3f rear_clearance=%.3f stop_distance=%.3f",
        front_clearance_m_, rear_clearance_m_, obstacle_stop_distance_m_);
    }
  }

  const double left_cmd_w =
    (command_v_ - 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  const double right_cmd_w =
    (command_v_ + 0.5 * wheel_separation_m_ * command_wz_) / wheel_radius_m_;
  publishWheelVelocities(left_cmd_w, right_cmd_w);
  publishSharedTelemetry(now, command_v_, command_wz_);
}

void SharedControlNode::controlStepSharedTorque(const rclcpp::Time & now, double dt)
{
  SharedControlState state;
  if (!computeSharedControlState(now, dt, state)) {
    publishZeroCommandForCurrentMode();
    return;
  }

  double force_cmd = state.shared_force_x;
  double torque_cmd = state.shared_torque_z;
  if (state.obstacle_fresh) {
    const double brake_velocity_deadband =
      std::max(0.0, torque_obstacle_brake_velocity_deadband_);
    const double front_velocity_limit = max_linear_velocity_forward_ * state.front_scale;
    if (state.v_meas > brake_velocity_deadband && state.front_scale < 1.0) {
      force_cmd -= desired_mass_x_ * max_linear_accel_forward_ * (1.0 - state.front_scale);
      if (state.v_meas > front_velocity_limit) {
        force_cmd -= torque_assist_damping_x_ * (state.v_meas - front_velocity_limit);
      }
    }

    if (obstacle_guard_reverse_enabled_) {
      const double rear_velocity_limit = max_linear_velocity_reverse_ * state.rear_scale;
      if (state.v_meas < -brake_velocity_deadband && state.rear_scale < 1.0) {
        force_cmd += desired_mass_x_ * max_linear_accel_reverse_ * (1.0 - state.rear_scale);
        if (-state.v_meas > rear_velocity_limit) {
          force_cmd += torque_assist_damping_x_ * ((-state.v_meas) - rear_velocity_limit);
        }
      }
    }
  }

  if (shared_torque_assist_enabled_) {
    force_cmd -= torque_assist_damping_x_ * state.v_meas;
    torque_cmd -= torque_assist_damping_z_ * state.wz_meas;
  } else {
    torque_cmd = 0.0;
  }
  force_cmd = clamp(
    force_cmd,
    -torque_assist_max_force_reverse_,
    torque_assist_max_force_forward_);
  torque_cmd = clamp(
    torque_cmd,
    -torque_assist_max_torque_z_,
    torque_assist_max_torque_z_);

  const double yaw_to_wheel = wheel_radius_m_ / std::max(wheel_separation_m_, 1.0e-6);
  const double left_torque =
    0.5 * wheel_radius_m_ * force_cmd - yaw_to_wheel * torque_cmd;
  const double right_torque =
    0.5 * wheel_radius_m_ * force_cmd + yaw_to_wheel * torque_cmd;
  command_v_ = 0.0;
  command_wz_ = 0.0;
  publishWheelTorques(left_torque, right_torque);
  publishSharedTelemetry(now, state.v_meas, state.wz_meas);
}

void SharedControlNode::controlStep()
{
  const auto now = this->get_clock()->now();
  double dt = (now - last_step_stamp_).seconds();
  if (dt <= 1.0e-6) {
    dt = 1.0 / std::max(loop_rate_hz_, 1.0);
  }
  last_step_stamp_ = now;

  updateOdometryFromStatus(now);
  requestAxisState(desiredClosedLoopControl());

  if (shared_control_mode_ == kSharedControlModeNormal) {
    controlStepAutonomy(dt);
    return;
  }

  if (shared_control_mode_ == kSharedControlModeSpeedOverlay) {
    controlStepSpeedOverlay(now, dt);
    return;
  }

  if (shared_control_mode_ == kSharedControlModeShared) {
    controlStepShared(now, dt);
    return;
  }

  if (shared_control_mode_ == kSharedControlModeSharedTorque) {
    controlStepSharedTorque(now, dt);
    return;
  }

  if (shared_control_mode_ == kSharedControlModeFree) {
    controlStepFree();
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "unknown shared_control_mode=%d, fallback to stop",
    static_cast<int>(shared_control_mode_));
  controlStepStop(dt);
}

}  // namespace cabot_shared_control
