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

#ifndef CABOT_SHARED_CONTROL__SHARED_CONTROL_NODE_HPP_
#define CABOT_SHARED_CONTROL__SHARED_CONTROL_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

namespace cabot_shared_control
{

class SharedControlNode : public rclcpp::Node
{
public:
  SharedControlNode();

private:
  struct AxisFeedback
  {
    odrive_can::msg::ControllerStatus msg{};
    rclcpp::Time stamp{};
    bool received{false};
  };

  void onAxis0Status(const odrive_can::msg::ControllerStatus::SharedPtr msg);
  void onAxis1Status(const odrive_can::msg::ControllerStatus::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onSharedControlMode(const std_msgs::msg::Int8::SharedPtr msg);
  void onSharedTorqueAssistEnabled(const std_msgs::msg::Bool::SharedPtr msg);
  void onPauseControl(const std_msgs::msg::Bool::SharedPtr msg);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onAutonomyCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onFootprint(const geometry_msgs::msg::Polygon::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void controlStep();
  void controlStepShared(const rclcpp::Time & now, double dt);
  void controlStepSharedTorque(const rclcpp::Time & now, double dt);
  void controlStepAutonomy(double dt);
  void controlStepStop(double dt);
  void controlStepFree();
  void requestAxisState(bool closed_loop, bool force = false);
  bool desiredClosedLoopControl() const;
  bool cmdVelFresh(const rclcpp::Time & now) const;
  void updateOdometryFromStatus(const rclcpp::Time & now);
  double slewRate(double current, double target, double accel_limit, double dt) const;
  void publishWheelVelocities(double left_wheel_rad_s, double right_wheel_rad_s);
  void publishWheelTorques(double left_wheel_torque_nm, double right_wheel_torque_nm);
  void publishStop();
  void publishZeroCommandForCurrentMode();
  void publishSharedTelemetry(const rclcpp::Time & now, double linear_x, double angular_z);

  void requestClosedLoopIfReady();
  void onAxisStateResponse(
    rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future,
    const std::string & axis_name);

  bool statusValid(const AxisFeedback & feedback, const rclcpp::Time & now) const;
  double estimateWheelTorque(
    const odrive_can::msg::ControllerStatus & status,
    double sign) const;
  struct SharedControlState
  {
    double v_meas{0.0};
    double wz_meas{0.0};
    double shared_force_x{0.0};
    double shared_torque_z{0.0};
    bool obstacle_fresh{false};
    double front_scale{1.0};
    double rear_scale{1.0};
  };
  bool computeSharedControlState(
    const rclcpp::Time & now,
    double dt,
    SharedControlState & state);
  void resetSharedState();
  double toSiAngular(double odrive_velocity_value) const;
  double fromSiAngular(double angular_velocity_rad_s) const;
  double distancePointToFootprint(double x, double y) const;
  bool pointInsideFootprint(double x, double y) const;
  bool obstacleDataFresh(const rclcpp::Time & now) const;
  double obstacleApproachScale(double clearance_m, double stop_distance_m) const;

  // Fixed topics / names
  std::string axis0_ns_{"odrive_axis0"};
  std::string axis1_ns_{"odrive_axis1"};
  std::string footprint_frame_id_{"base_footprint"};

  // Geometry
  double wheel_radius_m_{0.0855};
  double wheel_separation_m_{0.139};
  double left_wheel_sign_{-1.0};
  double right_wheel_sign_{1.0};
  bool odrive_velocity_is_turns_per_sec_{true};
  int8_t shared_control_mode_{1};
  bool shared_torque_assist_enabled_{false};
  bool pause_control_{false};

  // ODrive control
  int control_mode_{2};
  int input_mode_{1};
  bool request_closed_loop_on_startup_{true};

  // Observer parameters
  double observer_gain_x_{25.0};
  double observer_gain_z_{25.0};
  double robot_mass_x_{24.0};
  double robot_inertia_z_{2.2};
  double robot_damping_x_{40.0};
  double robot_damping_z_{5.0};
  double coulomb_friction_x_{8.0};
  double coulomb_friction_z_{0.8};
  double friction_smoothing_linear_{0.05};
  double friction_smoothing_angular_{0.05};
  bool use_torque_estimate_{true};
  double motor_torque_constant_{0.06};

  // IMU usage
  bool use_imu_{true};
  bool use_gravity_compensation_{true};
  double gravity_comp_gain_{1.0};
  bool use_imu_linear_accel_{false};
  double imu_accel_blend_{0.2};

  // Compliance parameters
  double desired_mass_x_{11.0};
  double desired_inertia_z_{0.9};
  double desired_damping_x_{24.0};
  double desired_damping_z_{2.8};
  double torque_assist_damping_x_{24.0};
  double torque_assist_damping_z_{2.8};
  double torque_assist_input_scale_{0.25};
  double torque_assist_torque_scale_{0.0};
  double torque_assist_max_force_forward_{20.0};
  double torque_assist_max_force_reverse_{45.0};
  double torque_assist_max_torque_z_{2.5};
  double torque_obstacle_stop_distance_m_{0.35};
  double torque_obstacle_pushback_stiffness_{70.0};
  double torque_obstacle_pushback_max_force_{22.5};
  double torque_obstacle_force_filter_alpha_{0.12};
  double torque_obstacle_brake_velocity_deadband_{0.03};

  // Shared control blending
  double human_force_weight_{1.0};
  double autonomy_force_weight_{0.0};
  double autonomy_virtual_stiffness_x_{45.0};
  double autonomy_virtual_stiffness_z_{12.0};
  double autonomy_timeout_sec_{0.3};
  double human_force_x_sign_{1.0};
  double human_torque_z_sign_{1.0};
  double force_deadband_x_{3.0};
  double force_deadband_z_{0.3};

  // Safety / timing
  double loop_rate_hz_{100.0};
  double status_timeout_sec_{0.2};
  double cmd_vel_timeout_sec_{0.2};
  double axis_state_request_interval_sec_{0.5};
  double max_acc_{1.2};
  double max_dec_{-1.2};
  double max_linear_velocity_forward_{0.8};
  double max_linear_velocity_reverse_{0.8};
  double max_angular_velocity_{1.8};
  double max_linear_accel_forward_{1.2};
  double max_linear_accel_reverse_{1.2};
  double max_angular_accel_{2.5};

  // Obstacle guard
  bool obstacle_guard_enabled_{true};
  bool obstacle_guard_reverse_enabled_{false};
  double obstacle_stop_distance_m_{0.0};
  double obstacle_slowdown_margin_m_{0.0};
  double obstacle_min_speed_scale_{0.0};
  bool obstacle_pushback_enabled_{true};
  double obstacle_pushback_stiffness_{60.0};
  double obstacle_pushback_max_force_{30.0};
  double obstacle_timeout_sec_{0.3};
  double obstacle_point_min_z_{-0.3};
  double obstacle_point_max_z_{1.2};
  bool strict_frame_match_{true};
  bool sensor_guard_enabled_{true};
  double sensor_guard_half_width_m_{0.35};

  struct XYPoint
  {
    double x{0.0};
    double y{0.0};
  };

  std::vector<XYPoint> footprint_points_;
  rclcpp::Time footprint_stamp_;
  bool footprint_received_{false};

  double front_clearance_m_{std::numeric_limits<double>::infinity()};
  double rear_clearance_m_{std::numeric_limits<double>::infinity()};
  rclcpp::Time obstacle_stamp_;
  bool obstacle_received_{false};

  AxisFeedback left_feedback_;
  AxisFeedback right_feedback_;
  std::optional<geometry_msgs::msg::TwistStamped> latest_autonomy_cmd_;
  std::optional<geometry_msgs::msg::Twist> latest_cmd_vel_;
  rclcpp::Time latest_autonomy_stamp_;
  rclcpp::Time latest_cmd_vel_stamp_;
  rclcpp::Time last_axis_state_request_stamp_;

  double imu_pitch_rad_{0.0};
  double imu_accel_x_{0.0};
  bool requested_closed_loop_{false};

  double external_force_x_{0.0};
  double external_torque_z_{0.0};
  double filtered_obstacle_force_x_{0.0};
  double command_v_{0.0};
  double command_wz_{0.0};
  double last_v_meas_{0.0};
  double last_wz_meas_{0.0};
  rclcpp::Time last_step_stamp_;
  bool odom_initialized_{false};
  double last_left_dist_m_{0.0};
  double last_right_dist_m_{0.0};
  double odom_x_m_{0.0};
  double odom_y_m_{0.0};
  double odom_yaw_rad_{0.0};

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr ctrl_pub_left_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr ctrl_pub_right_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr left_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr shared_control_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shared_torque_assist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis0_client_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis1_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

}  // namespace cabot_shared_control

#endif  // CABOT_SHARED_CONTROL__SHARED_CONTROL_NODE_HPP_
