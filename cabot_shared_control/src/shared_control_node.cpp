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
constexpr int8_t kSharedControlModeShared = 1;
constexpr int8_t kSharedControlModeFree = 2;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
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

SharedControlNode::SharedControlNode()
: Node("shared_control_node")
{
  axis0_ns_ = this->declare_parameter<std::string>("axis0_namespace", "odrive_axis0");
  axis1_ns_ = this->declare_parameter<std::string>("axis1_namespace", "odrive_axis1");
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  shared_control_mode_topic_ =
    this->declare_parameter<std::string>("shared_control_mode_topic", "/shared_control_mode");
  const int initial_shared_control_mode =
    this->declare_parameter<int>("shared_control_mode", static_cast<int>(kSharedControlModeNormal));
  if (
    initial_shared_control_mode == kSharedControlModeNormal ||
    initial_shared_control_mode == kSharedControlModeShared ||
    initial_shared_control_mode == kSharedControlModeFree)
  {
    shared_control_mode_ = static_cast<int8_t>(initial_shared_control_mode);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "invalid shared_control_mode=%d at startup (expected: 0=normal, 1=shared, 2=free), fallback to 0",
      initial_shared_control_mode);
    shared_control_mode_ = kSharedControlModeNormal;
  }
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cabot/cmd_vel");
  use_imu_ = this->declare_parameter<bool>("use_imu", true);
  autonomy_cmd_topic_ =
    this->declare_parameter<std::string>("autonomy_cmd_topic", "/autonomy/cmd_vel");
  const auto legacy_pointcloud_topic =
    this->declare_parameter<std::string>("pointcloud_topic", "/scan");
  scan_topic_ = this->declare_parameter<std::string>("scan_topic", legacy_pointcloud_topic);
  footprint_topic_ = this->declare_parameter<std::string>("footprint_topic", "/footprint");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/cabot/odom_raw");

  wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.0855);
  wheel_separation_m_ = this->declare_parameter<double>("wheel_separation_m", 0.419);
  left_wheel_sign_ = this->declare_parameter<double>("left_wheel_sign", -1.0);
  right_wheel_sign_ = this->declare_parameter<double>("right_wheel_sign", 1.0);
  odrive_velocity_is_turns_per_sec_ =
    this->declare_parameter<bool>("odrive_velocity_is_turns_per_sec", true);

  control_mode_ = this->declare_parameter<int>("control_mode", 2);
  input_mode_ = this->declare_parameter<int>("input_mode", 1);
  request_closed_loop_on_startup_ =
    this->declare_parameter<bool>("request_closed_loop_on_startup", true);

  observer_gain_x_ = this->declare_parameter<double>("observer_gain_x", 25.0);
  observer_gain_z_ = this->declare_parameter<double>("observer_gain_z", 25.0);
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
  desired_inertia_z_ = this->declare_parameter<double>("desired_inertia_z", 1.2);
  desired_damping_x_ = this->declare_parameter<double>("desired_damping_x", 14.0);
  desired_damping_z_ = this->declare_parameter<double>("desired_damping_z", 2.5);

  human_force_weight_ = this->declare_parameter<double>("human_force_weight", 1.0);
  autonomy_force_weight_ = this->declare_parameter<double>("autonomy_force_weight", 0.0);
  autonomy_virtual_stiffness_x_ =
    this->declare_parameter<double>("autonomy_virtual_stiffness_x", 45.0);
  autonomy_virtual_stiffness_z_ =
    this->declare_parameter<double>("autonomy_virtual_stiffness_z", 12.0);
  autonomy_timeout_sec_ = this->declare_parameter<double>("autonomy_timeout_sec", 0.3);
  human_force_x_sign_ = this->declare_parameter<double>("human_force_x_sign", 1.0);
  human_torque_z_sign_ = this->declare_parameter<double>("human_torque_z_sign", 1.0);
  force_deadband_x_ = this->declare_parameter<double>("force_deadband_x", 3.0);
  force_deadband_z_ = this->declare_parameter<double>("force_deadband_z", 0.3);

  loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 100.0);
  status_timeout_sec_ = this->declare_parameter<double>("status_timeout_sec", 0.2);
  cmd_vel_timeout_sec_ = this->declare_parameter<double>("cmd_vel_timeout_sec", 0.2);
  axis_state_request_interval_sec_ =
    this->declare_parameter<double>("axis_state_request_interval_sec", 0.5);
  max_acc_ = this->declare_parameter<double>("max_acc", 1.2);
  max_dec_ = this->declare_parameter<double>("max_dec", -1.2);
  const double max_linear_velocity_default =
    this->declare_parameter<double>("max_linear_velocity", 0.8);
  max_linear_velocity_forward_ = this->declare_parameter<double>(
    "max_linear_velocity_forward", max_linear_velocity_default);
  max_linear_velocity_reverse_ = this->declare_parameter<double>(
    "max_linear_velocity_reverse", max_linear_velocity_default);
  max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 1.8);
  const double max_linear_accel_default =
    this->declare_parameter<double>("max_linear_accel", max_acc_);
  max_linear_accel_forward_ = this->declare_parameter<double>(
    "max_linear_accel_forward", max_linear_accel_default);
  max_linear_accel_reverse_ = this->declare_parameter<double>(
    "max_linear_accel_reverse", std::max(0.0, -max_dec_));
  max_angular_accel_ = this->declare_parameter<double>("max_angular_accel", 2.5);
  obstacle_guard_enabled_ = this->declare_parameter<bool>("obstacle_guard_enabled", true);
  obstacle_guard_reverse_enabled_ =
    this->declare_parameter<bool>("obstacle_guard_reverse_enabled", false);
  obstacle_stop_distance_m_ = this->declare_parameter<double>("obstacle_stop_distance_m", 0.5);
  obstacle_slowdown_margin_m_ =
    this->declare_parameter<double>("obstacle_slowdown_margin_m", 0.0);
  obstacle_min_speed_scale_ = this->declare_parameter<double>("obstacle_min_speed_scale", 0.0);
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

  ctrl_pub_left_ = this->create_publisher<odrive_can::msg::ControlMessage>(
    "/" + axis0_ns_ + "/control_message", 10);
  ctrl_pub_right_ = this->create_publisher<odrive_can::msg::ControlMessage>(
    "/" + axis1_ns_ + "/control_message", 10);
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/shared_control/external_wrench",
    10);
  cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("/shared_control/cmd_vel", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  left_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    "/" + axis0_ns_ + "/controller_status", 50,
    std::bind(&SharedControlNode::onAxis0Status, this, std::placeholders::_1));
  right_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    "/" + axis1_ns_ + "/controller_status", 50,
    std::bind(&SharedControlNode::onAxis1Status, this, std::placeholders::_1));
  if (use_imu_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50, std::bind(&SharedControlNode::onImu, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "IMU input disabled (use_imu=false). Assuming horizontal terrain and turning off IMU-based compensation.");
  }
  shared_control_mode_sub_ = this->create_subscription<std_msgs::msg::Int8>(
    shared_control_mode_topic_, 20,
    std::bind(&SharedControlNode::onSharedControlMode, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 20, std::bind(&SharedControlNode::onCmdVel, this, std::placeholders::_1));
  autonomy_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    autonomy_cmd_topic_, 20,
    std::bind(&SharedControlNode::onAutonomyCmd, this, std::placeholders::_1));
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
    footprint_topic_, 10, std::bind(&SharedControlNode::onFootprint, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
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
    "started: axis0=/%s axis1=/%s loop=%.1fHz imu=%s obstacle_guard=%s stop_distance=%.2fm",
    axis0_ns_.c_str(), axis1_ns_.c_str(), loop_rate_hz_,
    use_imu_ ? "true" : "false",
    obstacle_guard_enabled_ ? "true" : "false",
    obstacle_stop_distance_m_);
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
    mode != kSharedControlModeShared &&
    mode != kSharedControlModeFree)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "ignore invalid shared_control_mode=%d (expected: 0=normal, 1=shared, 2=free)",
      static_cast<int>(mode));
    return;
  }
  if (shared_control_mode_ != mode) {
    RCLCPP_INFO(
      this->get_logger(),
      "shared_control_mode changed: %d -> %d",
      static_cast<int>(shared_control_mode_),
      static_cast<int>(mode));
    shared_control_mode_ = mode;
  }
}

void SharedControlNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  latest_cmd_vel_stamp_ = this->get_clock()->now();
}

void SharedControlNode::onFootprint(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  footprint_points_.clear();
  footprint_points_.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    footprint_points_.push_back(XYPoint{static_cast<double>(p.x), static_cast<double>(p.y)});
  }

  // geometry_msgs/Polygon has no frame information.
  footprint_frame_id_.clear();
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

  if (
    footprint_received_ && strict_frame_match_ && !footprint_frame_id_.empty() &&
    msg->header.frame_id != footprint_frame_id_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "frame mismatch: scan=%s footprint=%s (skip obstacle guard update)",
      msg->header.frame_id.c_str(), footprint_frame_id_.c_str());
    return;
  }

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
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

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
    requestAxisState(true, true);
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

double SharedControlNode::obstacleApproachScale(double clearance_m) const
{
  if (!std::isfinite(clearance_m)) {
    return 1.0;
  }

  const double stop_distance = std::max(0.0, obstacle_stop_distance_m_);
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

void SharedControlNode::publishStop()
{
  publishWheelVelocities(0.0, 0.0);
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

void SharedControlNode::controlStepShared(const rclcpp::Time & now, double dt)
{
  if (!statusValid(left_feedback_, now) || !statusValid(right_feedback_, now)) {
    command_v_ = 0.0;
    command_wz_ = 0.0;
    publishStop();
    return;
  }

  const double left_w =
    left_wheel_sign_ * toSiAngular(static_cast<double>(left_feedback_.msg.vel_estimate));
  const double right_w =
    right_wheel_sign_ * toSiAngular(static_cast<double>(right_feedback_.msg.vel_estimate));

  const double v_meas = 0.5 * wheel_radius_m_ * (right_w + left_w);
  const double wz_meas = (wheel_radius_m_ / wheel_separation_m_) * (right_w - left_w);

  const double wheel_a_x = (v_meas - last_v_meas_) / dt;
  const double wheel_a_z = (wz_meas - last_wz_meas_) / dt;
  last_v_meas_ = v_meas;
  last_wz_meas_ = wz_meas;

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
    std::tanh(v_meas / std::max(friction_smoothing_linear_, 1.0e-3));
  const double friction_z = coulomb_friction_z_ *
    std::tanh(wz_meas / std::max(friction_smoothing_angular_, 1.0e-3));

  double gravity_force_x = 0.0;
  if (use_gravity_compensation_) {
    gravity_force_x = gravity_comp_gain_ * robot_mass_x_ * kGravity * std::sin(imu_pitch_rad_);
  }

  const double residual_x =
    f_motor_x - robot_mass_x_ * a_x - robot_damping_x_ * v_meas - friction_x - gravity_force_x;
  const double residual_z =
    n_motor_z - robot_inertia_z_ * wheel_a_z - robot_damping_z_ * wz_meas - friction_z;

  external_force_x_ += dt * observer_gain_x_ * (residual_x - external_force_x_);
  external_torque_z_ += dt * observer_gain_z_ * (residual_z - external_torque_z_);

  const double human_force_x =
    signedDeadband(human_force_x_sign_ * external_force_x_, force_deadband_x_);
  const double human_torque_z =
    signedDeadband(human_torque_z_sign_ * external_torque_z_, force_deadband_z_);

  double auto_force_x = 0.0;
  double auto_torque_z = 0.0;
  if (
    latest_autonomy_cmd_.has_value() &&
    (now - latest_autonomy_stamp_).seconds() <= autonomy_timeout_sec_)
  {
    const double v_ref = static_cast<double>(latest_autonomy_cmd_->twist.linear.x);
    const double wz_ref = static_cast<double>(latest_autonomy_cmd_->twist.angular.z);
    auto_force_x = autonomy_virtual_stiffness_x_ * (v_ref - v_meas);
    auto_torque_z = autonomy_virtual_stiffness_z_ * (wz_ref - wz_meas);
  }

  double shared_force_x = human_force_weight_ * human_force_x + autonomy_force_weight_ * auto_force_x;
  const double shared_torque_z =
    human_force_weight_ * human_torque_z + autonomy_force_weight_ * auto_torque_z;

  const bool obstacle_fresh = obstacleDataFresh(now);
  const double front_scale = obstacle_fresh ? obstacleApproachScale(front_clearance_m_) : 1.0;
  const double rear_scale = (obstacle_fresh && obstacle_guard_reverse_enabled_) ?
    obstacleApproachScale(rear_clearance_m_) : 1.0;

  // Distance-aware slowdown: reduce drive intent while approaching obstacles.
  if (shared_force_x > 0.0) {
    shared_force_x *= front_scale;
  } else if (shared_force_x < 0.0 && obstacle_guard_reverse_enabled_) {
    shared_force_x *= rear_scale;
  }

  if (obstacle_fresh && obstacle_pushback_enabled_) {
    // Apply virtual spring force only against the intended moving direction.
    if (front_clearance_m_ <= obstacle_stop_distance_m_ && shared_force_x > 0.0) {
      const double penetration = obstacle_stop_distance_m_ - front_clearance_m_;
      const double pushback = clamp(
        obstacle_pushback_stiffness_ * penetration, 0.0, obstacle_pushback_max_force_);
      shared_force_x -= pushback;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "front obstacle pushback: clearance=%.3fm pushback=%.2fN",
        front_clearance_m_, pushback);
    } else {
      if (
        obstacle_guard_reverse_enabled_ && rear_clearance_m_ <= obstacle_stop_distance_m_ &&
        shared_force_x < 0.0)
      {
        const double penetration = obstacle_stop_distance_m_ - rear_clearance_m_;
        const double pushback = clamp(
          obstacle_pushback_stiffness_ * penetration, 0.0, obstacle_pushback_max_force_);
        shared_force_x += pushback;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rear obstacle pushback: clearance=%.3fm pushback=%.2fN",
          rear_clearance_m_, pushback);
      }
    }
  }

  double cmd_accel_x =
    (shared_force_x - desired_damping_x_ * command_v_) / std::max(desired_mass_x_, 1.0e-6);
  double cmd_accel_z =
    (shared_torque_z - desired_damping_z_ * command_wz_) / std::max(desired_inertia_z_, 1.0e-6);
  cmd_accel_x = clamp(cmd_accel_x, -max_linear_accel_reverse_, max_linear_accel_forward_);
  cmd_accel_z = clamp(cmd_accel_z, -max_angular_accel_, max_angular_accel_);

  command_v_ += cmd_accel_x * dt;
  command_wz_ += cmd_accel_z * dt;
  command_v_ = clamp(command_v_, -max_linear_velocity_reverse_, max_linear_velocity_forward_);
  command_wz_ = clamp(command_wz_, -max_angular_velocity_, max_angular_velocity_);

  // Final safety clamp: reduce approaching speed smoothly in obstacle slowdown
  // margin, then stop only inside stop_distance.
  if (obstacle_fresh) {
    const double front_limit = max_linear_velocity_forward_ * front_scale;
    if (command_v_ > front_limit) {
      command_v_ = front_limit;
    } else if (command_v_ > 0.0 && front_scale < 1.0) {
      const double slowdown_brake = max_linear_accel_forward_ * (1.0 - front_scale);
      command_v_ = std::max(0.0, command_v_ - slowdown_brake * dt);
      command_v_ = std::min(command_v_, front_limit);
    }
    if (front_scale <= 1.0e-3 && command_v_ > 0.0) {
      command_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "front approach blocked: clearance %.3fm <= %.3fm",
        front_clearance_m_, obstacle_stop_distance_m_);
    }

    if (obstacle_guard_reverse_enabled_) {
      const double rear_limit = max_linear_velocity_reverse_ * rear_scale;
      if (command_v_ < -rear_limit) {
        command_v_ = -rear_limit;
      } else if (command_v_ < 0.0 && rear_scale < 1.0) {
        const double slowdown_brake = max_linear_accel_reverse_ * (1.0 - rear_scale);
        command_v_ = std::min(0.0, command_v_ + slowdown_brake * dt);
        command_v_ = std::max(command_v_, -rear_limit);
      }
      if (rear_scale <= 1.0e-3 && command_v_ < 0.0) {
        command_v_ = 0.0;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "rear approach blocked: clearance %.3fm <= %.3fm",
          rear_clearance_m_, obstacle_stop_distance_m_);
      }
    }

    if (front_scale < 0.999 || rear_scale < 0.999) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 200,
        "obstacle slowdown: front_clearance=%.3f front_scale=%.3f rear_clearance=%.3f rear_scale=%.3f command_v=%.3f",
        front_clearance_m_, front_scale, rear_clearance_m_, rear_scale, command_v_);
    }
    if (front_scale <= 1.0e-3 || rear_scale <= 1.0e-3) {
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

  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = now;
  wrench_msg.header.frame_id = "base_link";
  wrench_msg.wrench.force.x = external_force_x_;
  wrench_msg.wrench.torque.z = external_torque_z_;
  wrench_pub_->publish(wrench_msg);

  geometry_msgs::msg::TwistStamped cmd_msg;
  cmd_msg.header.stamp = now;
  cmd_msg.header.frame_id = "base_link";
  cmd_msg.twist.linear.x = command_v_;
  cmd_msg.twist.angular.z = command_wz_;
  cmd_pub_->publish(cmd_msg);
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

  if (shared_control_mode_ == kSharedControlModeNormal) {
    requestAxisState(true);
    controlStepAutonomy(dt);
    return;
  }

  if (shared_control_mode_ == kSharedControlModeShared) {
    requestAxisState(true);
    controlStepShared(now, dt);
    return;
  }

  if (shared_control_mode_ == kSharedControlModeFree) {
    requestAxisState(false);
    controlStepFree();
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "unknown shared_control_mode=%d, fallback to stop",
    static_cast<int>(shared_control_mode_));
  requestAxisState(true);
  controlStepStop(dt);
}

}  // namespace cabot_shared_control
