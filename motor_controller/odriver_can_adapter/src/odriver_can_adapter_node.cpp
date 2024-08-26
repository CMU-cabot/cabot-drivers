/*******************************************************************************
 * Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odriver_msgs/msg/motor_status.hpp>
#include <odriver_msgs/msg/motor_target.hpp>

class ODriverCanAdapterNode: public rclcpp::Node
{
public:
  ODriverCanAdapterNode()
    : Node("odriver_can_adapter_node")
  {
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    this->declare_parameter("wheel_diameter_m", 0.17);
    wheel_diameter_m_ = this->get_parameter("wheel_diameter_m").as_double();
    meter_per_round_ = wheel_diameter_m_ * M_PI;

    rclcpp::QoS control_message_left_qos(rclcpp::KeepAll{});
    control_message_left_pub_ = create_publisher<odrive_can::msg::ControlMessage>("/control_message_left", control_message_left_qos);

    rclcpp::QoS control_message_right_qos(rclcpp::KeepAll{});
    control_message_right_pub_ = create_publisher<odrive_can::msg::ControlMessage>("/control_message_right", control_message_right_qos);

    rclcpp::QoS motor_status_qos(rclcpp::KeepAll{});
    motor_status_pub_ = create_publisher<odriver_msgs::msg::MotorStatus>("/motor_status", motor_status_qos);

    rclcpp::QoS controller_status_left_qos(rclcpp::KeepAll{});
    controller_status_left_sub_ = create_subscription<odrive_can::msg::ControllerStatus>("/controller_status_left", controller_status_left_qos, std::bind(&ODriverCanAdapterNode::controllerStatusLeftCallback, this, _1));

    rclcpp::QoS controller_status_right_qos(rclcpp::KeepAll{});
    controller_status_right_sub_ = create_subscription<odrive_can::msg::ControllerStatus>("/controller_status_right", controller_status_right_qos, std::bind(&ODriverCanAdapterNode::controllerStatusRightCallback, this, _1));

    rclcpp::QoS motor_target_qos(rclcpp::KeepAll{});
    motor_target_sub_ = create_subscription<odriver_msgs::msg::MotorTarget>("/motor_target", motor_target_qos, std::bind(&ODriverCanAdapterNode::motorTargetCallback, this, _1));

    timer_ = create_wall_timer(500ms, std::bind(&ODriverCanAdapterNode::timerCallback, this));
  }

  ~ODriverCanAdapterNode()
  {
  }
private:
  void controllerStatusLeftCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    spd_left_c_ = msg->vel_estimate;
    dist_left_c_ = msg->pos_estimate;
    current_setpoint_left_ = msg->iq_setpoint;
    current_measured_left_ = msg->iq_measured;
  }

  void controllerStatusRightCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    spd_right_c_ = msg->vel_estimate;
    dist_right_c_ = msg->pos_estimate;
    current_setpoint_right_ = msg->iq_setpoint;
    current_measured_right_ = msg->iq_measured;
  }

  void motorTargetCallback(const odriver_msgs::msg::MotorTarget::SharedPtr msg)
  {
    // WIP: what's loop control
    odrive_can::msg::ControlMessage left_message;
    odrive_can::msg::ControlMessage right_message;
    
    // set velocity mode
    left_message.control_mode = kVelocityControlMode;
    right_message.control_mode = kVelocityControlMode;

    left_message.input_mode = kPassthroughInputMode;
    right_message.input_mode = kPassthroughInputMode;

    // meter/sec -> rotation/sec
    left_message.input_vel = msg->spd_left / meter_per_round_;
    right_message.input_vel = msg->spd_right / meter_per_round_;

    control_message_left_pub_->publish(left_message);
    control_message_right_pub_->publish(right_message);
  }

  void timerCallback()
  {
    odriver_msgs::msg::MotorStatus status;

    status.header.stamp = this->get_clock()->now();

    status.dist_left_c = dist_left_c_;
    status.dist_right_c = dist_right_c_;

    status.spd_left_c = spd_left_c_;
    status.spd_right_c = spd_right_c_;

    status.dist_left = dist_left_c_ * meter_per_round_;
    status.dist_right = dist_right_c_ * meter_per_round_;

    status.spd_left = spd_left_c_ * meter_per_round_;
    status.spd_right = spd_right_c_ * meter_per_round_;

    status.current_setpoint_left = current_setpoint_left_;
    status.current_setpoint_right = current_setpoint_right_;

    status.current_measured_left = current_measured_left_;
    status.current_measured_right = current_measured_right_;

    motor_status_pub_->publish(status);
  }

  // refer from: https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
  const int kVelocityControlMode = 2;

  // refer from: https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
  const int kPassthroughInputMode = 1;

  double wheel_diameter_m_;
  double meter_per_round_;

  double spd_left_c_;
  double spd_right_c_;

  double dist_left_c_;
  double dist_right_c_;

  double current_setpoint_left_;
  double current_setpoint_right_;

  double current_measured_left_;
  double current_measured_right_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr control_message_left_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr control_message_right_pub_;
  rclcpp::Publisher<odriver_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_status_left_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_status_right_sub_;
  rclcpp::Subscription<odriver_msgs::msg::MotorTarget>::SharedPtr motor_target_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriverCanAdapterNode>());
  rclcpp::shutdown();

  return 0;
}
