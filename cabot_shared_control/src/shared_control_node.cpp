#include "cabot_shared_control/shared_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
constexpr uint32_t kAxisStateClosedLoopControl = 8;
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

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
  autonomy_cmd_topic_ =
    this->declare_parameter<std::string>("autonomy_cmd_topic", "/autonomy/cmd_vel");
  pointcloud_topic_ =
    this->declare_parameter<std::string>("pointcloud_topic", "/velodyne_points_cropped");
  footprint_topic_ = this->declare_parameter<std::string>("footprint_topic", "/footprint");

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
  force_deadband_x_ = this->declare_parameter<double>("force_deadband_x", 3.0);
  force_deadband_z_ = this->declare_parameter<double>("force_deadband_z", 0.3);

  loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 100.0);
  status_timeout_sec_ = this->declare_parameter<double>("status_timeout_sec", 0.2);
  max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 0.8);
  max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 1.8);
  max_linear_accel_ = this->declare_parameter<double>("max_linear_accel", 1.2);
  max_angular_accel_ = this->declare_parameter<double>("max_angular_accel", 2.5);
  obstacle_guard_enabled_ = this->declare_parameter<bool>("obstacle_guard_enabled", true);
  obstacle_stop_distance_m_ = this->declare_parameter<double>("obstacle_stop_distance_m", 0.5);
  obstacle_timeout_sec_ = this->declare_parameter<double>("obstacle_timeout_sec", 0.3);
  obstacle_point_min_z_ = this->declare_parameter<double>("obstacle_point_min_z", -0.3);
  obstacle_point_max_z_ = this->declare_parameter<double>("obstacle_point_max_z", 1.2);
  strict_frame_match_ = this->declare_parameter<bool>("strict_frame_match", true);

  const auto now = this->get_clock()->now();
  left_feedback_.stamp = now;
  right_feedback_.stamp = now;
  latest_autonomy_stamp_ = now;
  last_step_stamp_ = now;
  footprint_stamp_ = now;
  obstacle_stamp_ = now;

  ctrl_pub_left_ = this->create_publisher<odrive_can::msg::ControlMessage>(
    "/" + axis0_ns_ + "/control_message", 10);
  ctrl_pub_right_ = this->create_publisher<odrive_can::msg::ControlMessage>(
    "/" + axis1_ns_ + "/control_message", 10);
  wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("/shared_control/external_wrench", 10);
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/shared_control/cmd_vel", 10);

  left_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    "/" + axis0_ns_ + "/controller_status", 50,
    std::bind(&SharedControlNode::onAxis0Status, this, std::placeholders::_1));
  right_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    "/" + axis1_ns_ + "/controller_status", 50,
    std::bind(&SharedControlNode::onAxis1Status, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 50, std::bind(&SharedControlNode::onImu, this, std::placeholders::_1));
  autonomy_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    autonomy_cmd_topic_, 20,
    std::bind(&SharedControlNode::onAutonomyCmd, this, std::placeholders::_1));
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_, 10, std::bind(&SharedControlNode::onFootprint, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, 10, std::bind(&SharedControlNode::onPointCloud, this, std::placeholders::_1));

  if (request_closed_loop_on_startup_) {
    axis0_client_ = this->create_client<odrive_can::srv::AxisState>(
      "/" + axis0_ns_ + "/request_axis_state");
    axis1_client_ = this->create_client<odrive_can::srv::AxisState>(
      "/" + axis1_ns_ + "/request_axis_state");
    startup_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&SharedControlNode::requestClosedLoopIfReady, this));
  }

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(loop_rate_hz_, 1.0)),
    std::bind(&SharedControlNode::controlStep, this));

  RCLCPP_INFO(
    this->get_logger(),
    "started: axis0=/%s axis1=/%s loop=%.1fHz obstacle_guard=%s stop_distance=%.2fm",
    axis0_ns_.c_str(), axis1_ns_.c_str(), loop_rate_hz_,
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

void SharedControlNode::onFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  footprint_points_.clear();
  footprint_points_.reserve(msg->polygon.points.size());
  for (const auto & p : msg->polygon.points) {
    footprint_points_.push_back(XYPoint{static_cast<double>(p.x), static_cast<double>(p.y)});
  }

  footprint_frame_id_ = msg->header.frame_id;
  footprint_stamp_ = this->get_clock()->now();
  footprint_received_ = footprint_points_.size() >= 3;
}

void SharedControlNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!obstacle_guard_enabled_ || !footprint_received_) {
    return;
  }

  if (strict_frame_match_ && msg->header.frame_id != footprint_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "frame mismatch: pointcloud=%s footprint=%s (skip obstacle guard update)",
      msg->header.frame_id.c_str(), footprint_frame_id_.c_str());
    return;
  }

  double front_clearance = std::numeric_limits<double>::infinity();
  double rear_clearance = std::numeric_limits<double>::infinity();
  bool has_valid_point = false;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < obstacle_point_min_z_ || z > obstacle_point_max_z_) {
        continue;
      }

      has_valid_point = true;
      const double clearance = distancePointToFootprint(x, y);
      if (x >= 0.0) {
        front_clearance = std::min(front_clearance, clearance);
      } else {
        rear_clearance = std::min(rear_clearance, clearance);
      }
    }
  } catch (const std::runtime_error & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "pointcloud fields missing (need x,y,z): %s", ex.what());
    return;
  }

  if (!has_valid_point) {
    front_clearance = std::numeric_limits<double>::infinity();
    rear_clearance = std::numeric_limits<double>::infinity();
  }

  front_clearance_m_ = front_clearance;
  rear_clearance_m_ = rear_clearance;
  obstacle_stamp_ = this->get_clock()->now();
  obstacle_received_ = true;
}

void SharedControlNode::requestClosedLoopIfReady()
{
  if (!request_closed_loop_on_startup_ || !axis0_client_ || !axis1_client_) {
    return;
  }
  if (!axis0_client_->service_is_ready() || !axis1_client_->service_is_ready()) {
    return;
  }

  auto req0 = std::make_shared<odrive_can::srv::AxisState::Request>();
  req0->axis_requested_state = kAxisStateClosedLoopControl;
  axis0_client_->async_send_request(
    req0,
    [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future) {
      this->onAxisStateResponse(future, this->axis0_ns_);
    });

  auto req1 = std::make_shared<odrive_can::srv::AxisState::Request>();
  req1->axis_requested_state = kAxisStateClosedLoopControl;
  axis1_client_->async_send_request(
    req1,
    [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future) {
      this->onAxisStateResponse(future, this->axis1_ns_);
    });

  request_closed_loop_on_startup_ = false;
  if (startup_timer_) {
    startup_timer_->cancel();
  }
  RCLCPP_INFO(this->get_logger(), "requested CLOSED_LOOP_CONTROL on both ODrive axes");
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

void SharedControlNode::controlStep()
{
  const auto now = this->get_clock()->now();
  double dt = (now - last_step_stamp_).seconds();
  if (dt <= 1.0e-6) {
    dt = 1.0 / std::max(loop_rate_hz_, 1.0);
  }
  last_step_stamp_ = now;

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

  const double human_force_x = signedDeadband(external_force_x_, force_deadband_x_);
  const double human_torque_z = signedDeadband(external_torque_z_, force_deadband_z_);

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

  const double shared_force_x =
    human_force_weight_ * human_force_x + autonomy_force_weight_ * auto_force_x;
  const double shared_torque_z =
    human_force_weight_ * human_torque_z + autonomy_force_weight_ * auto_torque_z;

  double cmd_accel_x =
    (shared_force_x - desired_damping_x_ * command_v_) / std::max(desired_mass_x_, 1.0e-6);
  double cmd_accel_z =
    (shared_torque_z - desired_damping_z_ * command_wz_) / std::max(desired_inertia_z_, 1.0e-6);
  cmd_accel_x = clamp(cmd_accel_x, -max_linear_accel_, max_linear_accel_);
  cmd_accel_z = clamp(cmd_accel_z, -max_angular_accel_, max_angular_accel_);

  command_v_ += cmd_accel_x * dt;
  command_wz_ += cmd_accel_z * dt;
  command_v_ = clamp(command_v_, -max_linear_velocity_, max_linear_velocity_);
  command_wz_ = clamp(command_wz_, -max_angular_velocity_, max_angular_velocity_);

  if (obstacleDataFresh(now)) {
    if (command_v_ > 0.0 && front_clearance_m_ <= obstacle_stop_distance_m_) {
      command_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "forward motion blocked by obstacle clearance %.3fm <= %.3fm",
        front_clearance_m_, obstacle_stop_distance_m_);
    } else if (command_v_ < 0.0 && rear_clearance_m_ <= obstacle_stop_distance_m_) {
      command_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "reverse motion blocked by obstacle clearance %.3fm <= %.3fm",
        rear_clearance_m_, obstacle_stop_distance_m_);
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

}  // namespace cabot_shared_control
