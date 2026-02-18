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

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
  void onAutonomyCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void controlStep();
  void publishWheelVelocities(double left_wheel_rad_s, double right_wheel_rad_s);
  void publishStop();

  void requestClosedLoopIfReady();
  void onAxisStateResponse(
    rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future,
    const std::string & axis_name);

  bool statusValid(const AxisFeedback & feedback, const rclcpp::Time & now) const;
  double estimateWheelTorque(
    const odrive_can::msg::ControllerStatus & status,
    double sign) const;
  double toSiAngular(double odrive_velocity_value) const;
  double fromSiAngular(double angular_velocity_rad_s) const;
  double distancePointToFootprint(double x, double y) const;
  bool pointInsideFootprint(double x, double y) const;
  bool obstacleDataFresh(const rclcpp::Time & now) const;

  // Topics / names
  std::string axis0_ns_;
  std::string axis1_ns_;
  std::string imu_topic_;
  std::string autonomy_cmd_topic_;
  std::string pointcloud_topic_;
  std::string footprint_topic_;

  // Geometry
  double wheel_radius_m_{0.0855};
  double wheel_separation_m_{0.419};
  double left_wheel_sign_{-1.0};
  double right_wheel_sign_{1.0};
  bool odrive_velocity_is_turns_per_sec_{true};

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
  double desired_inertia_z_{1.2};
  double desired_damping_x_{14.0};
  double desired_damping_z_{2.5};

  // Shared control blending
  double human_force_weight_{1.0};
  double autonomy_force_weight_{0.0};
  double autonomy_virtual_stiffness_x_{45.0};
  double autonomy_virtual_stiffness_z_{12.0};
  double autonomy_timeout_sec_{0.3};
  double force_deadband_x_{3.0};
  double force_deadband_z_{0.3};

  // Safety / timing
  double loop_rate_hz_{100.0};
  double status_timeout_sec_{0.2};
  double max_linear_velocity_{0.8};
  double max_angular_velocity_{1.8};
  double max_linear_accel_{1.2};
  double max_angular_accel_{2.5};

  // Obstacle guard
  bool obstacle_guard_enabled_{true};
  double obstacle_stop_distance_m_{0.5};
  double obstacle_timeout_sec_{0.3};
  double obstacle_point_min_z_{-0.3};
  double obstacle_point_max_z_{1.2};
  bool strict_frame_match_{true};

  struct XYPoint
  {
    double x{0.0};
    double y{0.0};
  };

  std::vector<XYPoint> footprint_points_;
  std::string footprint_frame_id_;
  rclcpp::Time footprint_stamp_;
  bool footprint_received_{false};

  double front_clearance_m_{std::numeric_limits<double>::infinity()};
  double rear_clearance_m_{std::numeric_limits<double>::infinity()};
  rclcpp::Time obstacle_stamp_;
  bool obstacle_received_{false};

  AxisFeedback left_feedback_;
  AxisFeedback right_feedback_;
  std::optional<geometry_msgs::msg::TwistStamped> latest_autonomy_cmd_;
  rclcpp::Time latest_autonomy_stamp_;

  double imu_pitch_rad_{0.0};
  double imu_accel_x_{0.0};

  double external_force_x_{0.0};
  double external_torque_z_{0.0};
  double command_v_{0.0};
  double command_wz_{0.0};
  double last_v_meas_{0.0};
  double last_wz_meas_{0.0};
  rclcpp::Time last_step_stamp_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr ctrl_pub_left_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr ctrl_pub_right_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr left_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis0_client_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis1_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

}  // namespace cabot_shared_control

#endif  // CABOT_SHARED_CONTROL__SHARED_CONTROL_NODE_HPP_
