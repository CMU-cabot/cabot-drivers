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
#include <odrive_can/srv/axis_state.hpp>
#include <odriver_msgs/msg/motor_status.hpp>
#include <odriver_msgs/msg/motor_target.hpp>

#include "odrive_manager.hpp"

class ODriverCanAdapterNode: public rclcpp::Node
{
  struct AxisStateClientInfo {
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client;
    std::chrono::system_clock::time_point last_call_time;
    bool is_ready_axis_state_service;
    std::string name;
    unsigned int axis_state;
  };
public:
  ODriverCanAdapterNode()
    : Node("odriver_can_adapter_node")
  {
    using std::placeholders::_1;

    this->declare_parameter("wheel_diameter_m", 0.17);
    wheel_diameter_m_ = this->get_parameter("wheel_diameter_m").as_double();
    meter_per_round_ = wheel_diameter_m_ * M_PI;

    this->declare_parameter("is_clockwise", true);
    is_clockwise_ = this->get_parameter("is_clockwise").as_bool();

    this->declare_parameter("hz", 20.0);
    hz_ = this->get_parameter("hz").as_double();

    this->declare_parameter("service_timeout_ms", 2000);
    service_timeout_ms_ = this->get_parameter("service_timeout_ms").as_int();

    double sign_left, sign_right;
    if(is_clockwise_) {
      sign_left = -1.0;
      sign_right = 1.0;
    } else {
      sign_left = 1.0;
      sign_right = -1.0;
    }
    odrive_left_ = std::make_shared<ODriveManager>(
                      this,
                      "left",
                      sign_left,
                      wheel_diameter_m_);
    odrive_right_ = std::make_shared<ODriveManager>(
                      this,
                      "right",
                      sign_right,
                      wheel_diameter_m_);

    rclcpp::QoS motor_status_qos(rclcpp::KeepLast(10));
    motor_status_pub_ = create_publisher<odriver_msgs::msg::MotorStatus>(
                                  "/motor_status",
                                  motor_status_qos);

    rclcpp::QoS motor_target_qos(rclcpp::KeepLast(10));
    motor_target_sub_ = create_subscription<odriver_msgs::msg::MotorTarget>(
                          "/motor_target",
                          motor_target_qos,
                          std::bind(
                            &ODriverCanAdapterNode::motorTargetCallback,
                            this,
                            _1));

    timer_ = create_wall_timer(
              std::chrono::seconds(1) / hz_,
              std::bind(
                &ODriverCanAdapterNode::timerCallback,
                this));
  }

  ~ODriverCanAdapterNode()
  {
  }
private:
  // change odrive axis.controller.config.control_mode
  // closed_loop: true -> CLOSED_LOOP_CONTROL, false -> IDLE
  void changeAxisState(bool closed_loop)
  {
    if(closed_loop &&
          (odrive_left_->getAxisState() == kAxisStateIdle || odrive_right_->getAxisState() == kAxisStateIdle)) {
      odrive_left_->callAxisStateService(kAxisStateClosedLoopControl);
      odrive_right_->callAxisStateService(kAxisStateClosedLoopControl);
    } else if(!closed_loop &&
                (odrive_left_->getAxisState() == kAxisStateClosedLoopControl || odrive_right_->getAxisState() == kAxisStateClosedLoopControl)) {
      odrive_left_->callAxisStateService(kAxisStateIdle);
      odrive_right_->callAxisStateService(kAxisStateIdle);
    }
  }

  void motorTargetCallback(const odriver_msgs::msg::MotorTarget::SharedPtr msg)
  {
    changeAxisState(msg->loop_ctrl);

    // !msg->loop_ctrl is IDLE mode
    if(!msg->loop_ctrl) { return; }

    odrive_left_->publishControlMessage(
                    kVelocityControlMode,
                    kPassthroughInputMode,
                    msg->spd_left);

    odrive_right_->publishControlMessage(
                    kVelocityControlMode,
                    kPassthroughInputMode,
                    msg->spd_right);
  }

  void timerCallback()
  {
    odrive_left_->checkTimeoutServiceResponse(service_timeout_ms_);
    odrive_right_->checkTimeoutServiceResponse(service_timeout_ms_);

    if (!odrive_left_->isReady() || !odrive_right_->isReady()) {
      // do not publish MotorStatus if one of odrives is not ready
      return;
    }

    double sign_left = odrive_left_->getSign();
    double sign_right = odrive_right_->getSign();
    double dist_left_c = odrive_left_->getDistC();
    double dist_right_c = odrive_right_->getDistC();
    double spd_left_c = odrive_left_->getSpdC();
    double spd_right_c = odrive_right_->getSpdC();
    double current_setpoint_left = odrive_left_->getCurrentSetpoint();
    double current_setpoint_right = odrive_right_->getCurrentSetpoint();
    double current_measured_left = odrive_left_->getCurrentMeasured();
    double current_measured_right = odrive_right_->getCurrentMeasured();

    odriver_msgs::msg::MotorStatus status;

    status.header.stamp = this->get_clock()->now();

    status.dist_left_c = sign_left * dist_left_c;
    status.dist_right_c = sign_right * dist_right_c;

    status.spd_left_c = sign_left * spd_left_c;
    status.spd_right_c = sign_right * spd_right_c;

    status.dist_left = sign_left * dist_left_c * meter_per_round_;
    status.dist_right = sign_right * dist_right_c * meter_per_round_;

    status.spd_left = sign_left * spd_left_c * meter_per_round_;
    status.spd_right = sign_right * spd_right_c * meter_per_round_;

    status.current_setpoint_left = sign_left * current_setpoint_left;
    status.current_setpoint_right = sign_right * current_setpoint_right;

    status.current_measured_left = sign_left * current_measured_left;
    status.current_measured_right = sign_right * current_measured_right;

    motor_status_pub_->publish(status);
  }

  // refer from: https://github.com/odriverobotics/ros_odrive/blob/5e4dfe9df8e5ef4fb6c692c210eafb713cb41985/odrive_base/include/odrive_enums.h#L43-L58
  const unsigned int kAxisStateIdle = 1;
  const unsigned int kAxisStateClosedLoopControl = 8;

  // refer from: https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
  const int kVelocityControlMode = 2;

  // refer from: https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
  const int kPassthroughInputMode = 1;

  double wheel_diameter_m_;
  bool is_clockwise_;
  double hz_;
  int service_timeout_ms_;

  double meter_per_round_;

  rclcpp::Publisher<odriver_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;

  rclcpp::Subscription<odriver_msgs::msg::MotorTarget>::SharedPtr motor_target_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<ODriveManager> odrive_left_;
  std::shared_ptr<ODriveManager> odrive_right_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriverCanAdapterNode>());
  rclcpp::shutdown();

  return 0;
}
