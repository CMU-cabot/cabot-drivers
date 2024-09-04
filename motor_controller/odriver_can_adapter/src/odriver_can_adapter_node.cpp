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

class ODriverCanAdapterNode: public rclcpp::Node
{
public:
  ODriverCanAdapterNode()
    : Node("odriver_can_adapter_node"),
      is_ready_axis_state_left_service_(true),
      is_ready_axis_state_right_service_(true)
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

    if(is_clockwise_) {
      sign_left_ = -1.0;
      sign_right_ = 1.0;
    } else {
      sign_left_ = 1.0;
      sign_right_ = -1.0;
    }

    rclcpp::QoS control_message_left_qos(rclcpp::KeepAll{});
    control_message_left_pub_ = create_publisher<odrive_can::msg::ControlMessage>(
                                  "/control_message_left",
                                  control_message_left_qos);

    rclcpp::QoS control_message_right_qos(rclcpp::KeepAll{});
    control_message_right_pub_ = create_publisher<odrive_can::msg::ControlMessage>(
                                  "/control_message_right",
                                  control_message_right_qos);

    rclcpp::QoS motor_status_qos(rclcpp::KeepAll{});
    motor_status_pub_ = create_publisher<odriver_msgs::msg::MotorStatus>(
                                  "/motor_status",
                                  motor_status_qos);

    rclcpp::QoS controller_status_left_qos(rclcpp::KeepAll{});
    controller_status_left_sub_ = create_subscription<odrive_can::msg::ControllerStatus>(
                                    "/controller_status_left",
                                    controller_status_left_qos,
                                    std::bind(
                                      &ODriverCanAdapterNode::controllerStatusLeftCallback,
                                      this,
                                      _1));

    rclcpp::QoS controller_status_right_qos(rclcpp::KeepAll{});
    controller_status_right_sub_ = create_subscription<odrive_can::msg::ControllerStatus>(
                                    "/controller_status_right",
                                    controller_status_right_qos,
                                    std::bind(
                                      &ODriverCanAdapterNode::controllerStatusRightCallback,
                                      this,
                                      _1));

    rclcpp::QoS motor_target_qos(rclcpp::KeepAll{});
    motor_target_sub_ = create_subscription<odriver_msgs::msg::MotorTarget>(
                          "/motor_target",
                          motor_target_qos,
                          std::bind(
                            &ODriverCanAdapterNode::motorTargetCallback,
                            this,
                            _1));

    axis_state_left_client_ =
      create_client<odrive_can::srv::AxisState>("/request_axis_state_left");
    axis_state_right_client_ =
      create_client<odrive_can::srv::AxisState>("/request_axis_state_right");

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
  void controllerStatusLeftCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    spd_left_c_ = msg->vel_estimate;
    dist_left_c_ = msg->pos_estimate;
    current_setpoint_left_ = msg->iq_setpoint;
    current_measured_left_ = msg->iq_measured;
    axis_state_left_ = msg->axis_state;
  }

  void controllerStatusRightCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    spd_right_c_ = msg->vel_estimate;
    dist_right_c_ = msg->pos_estimate;
    current_setpoint_right_ = msg->iq_setpoint;
    current_measured_right_ = msg->iq_measured;
    axis_state_right_ = msg->axis_state;
  }

  void callAxisStateService(unsigned int axis_state)
  {
    using AxisState = odrive_can::srv::AxisState;
    using AxisStateClient = rclcpp::Client<AxisState>;
    using std::chrono::system_clock;
    RCLCPP_INFO(get_logger(), "call axis state service...");
    AxisState::Request::SharedPtr request = std::make_shared<AxisState::Request>();
    request->axis_requested_state = axis_state;

    if(is_ready_axis_state_left_service_) {
      RCLCPP_INFO(
        get_logger(),
        "axis_state_left to %d from %d",
        axis_state_left_,
        axis_state);
      while(!axis_state_left_client_->wait_for_service(std::chrono::seconds(1)) &&
              rclcpp::ok()) {
        RCLCPP_INFO(
          get_logger(),
          "request_axis_state_left service not available, waiting again...");
      }

      is_ready_axis_state_left_service_ = false;
      left_service_call_time_ = system_clock::now();
      AxisStateClient::SharedFutureWithRequestAndRequestId axis_state_left_future_ =
        axis_state_left_client_->async_send_request(
          request,
          [&](AxisStateClient::SharedFutureWithRequest future) {
            // WIP: check value
            (void) future;
            is_ready_axis_state_left_service_ = true;
            left_service_call_time_ = system_clock::now();
          });
    }

    if(is_ready_axis_state_right_service_) {
      RCLCPP_INFO(
        get_logger(),
        "axis_state_right to %d from %d",
        axis_state_right_,
        axis_state);
      while(!axis_state_right_client_->wait_for_service(std::chrono::seconds(1)) &&
              rclcpp::ok()) {
        RCLCPP_INFO(
          get_logger(),
          "request_axis_state_right service not available, waiting again...");
      }

      is_ready_axis_state_right_service_ = false;
      right_service_call_time_ = system_clock::now();
      AxisStateClient::SharedFutureWithRequestAndRequestId axis_state_right_future_ =
        axis_state_right_client_->async_send_request(
          request,
          [&](AxisStateClient::SharedFutureWithRequest future) {
            // WIP: check value
            (void) future;
            is_ready_axis_state_right_service_ = true;
            right_service_call_time_ = system_clock::now();
          });
    }
  }

  // change odrive axis.controller.config.control_mode
  // closed_loop: true -> CLOSED_LOOP_CONTROL, false -> IDOL
  void changeAxisState(bool closed_loop)
  {
    if(closed_loop &&
          (axis_state_left_ == kAxisStateIdol || axis_state_right_ == kAxisStateIdol)) {
      callAxisStateService(kAxisStateClosedLoopControl);
    } else if(!closed_loop &&
                (axis_state_left_ == kAxisStateClosedLoopControl || axis_state_right_ == kAxisStateClosedLoopControl)) {
      callAxisStateService(kAxisStateIdol);
    }
  }

  void motorTargetCallback(const odriver_msgs::msg::MotorTarget::SharedPtr msg)
  {
    changeAxisState(msg->loop_ctrl);

    // !msg->loop_ctrl is IDOL mode
    if(!msg->loop_ctrl) { return; }

    odrive_can::msg::ControlMessage left_message;
    odrive_can::msg::ControlMessage right_message;
    
    // set velocity mode
    left_message.control_mode = kVelocityControlMode;
    right_message.control_mode = kVelocityControlMode;

    left_message.input_mode = kPassthroughInputMode;
    right_message.input_mode = kPassthroughInputMode;

    // meter/sec -> rotation/sec
    left_message.input_vel = sign_left_ * msg->spd_left / meter_per_round_;
    right_message.input_vel = sign_right_ * msg->spd_right / meter_per_round_;

    if(is_ready_axis_state_left_service_) {
      control_message_left_pub_->publish(left_message);
    }
    if(is_ready_axis_state_right_service_) {
      control_message_right_pub_->publish(right_message);
    }
  }

  void timerCallback()
  {
    // check timeout ready flag that request_axis_state service
    if(!axis_state_left_client_->wait_for_service(std::chrono::seconds(0)) &&
        std::chrono::duration_cast<std::chrono::milliseconds>(left_service_call_time_ - std::chrono::system_clock::now()).count() >= service_timeout_ms_) {

      axis_state_left_client_->prune_pending_requests();
      is_ready_axis_state_left_service_ = true;
    }

    if(!axis_state_right_client_->wait_for_service(std::chrono::seconds(0)) &&
        std::chrono::duration_cast<std::chrono::milliseconds>(left_service_call_time_ - std::chrono::system_clock::now()).count() >= service_timeout_ms_) {

      axis_state_right_client_->prune_pending_requests();
      is_ready_axis_state_right_service_ = true;
    }

    odriver_msgs::msg::MotorStatus status;

    status.header.stamp = this->get_clock()->now();

    status.dist_left_c = sign_left_ * dist_left_c_;
    status.dist_right_c = sign_right_ * dist_right_c_;

    status.spd_left_c = sign_left_ * spd_left_c_;
    status.spd_right_c = sign_right_ * spd_right_c_;

    status.dist_left = sign_left_ * dist_left_c_ * meter_per_round_;
    status.dist_right = sign_right_ * dist_right_c_ * meter_per_round_;

    status.spd_left = sign_left_ * spd_left_c_ * meter_per_round_;
    status.spd_right = sign_right_ * spd_right_c_ * meter_per_round_;

    status.current_setpoint_left = sign_left_ * current_setpoint_left_;
    status.current_setpoint_right = sign_right_ * current_setpoint_right_;

    status.current_measured_left = sign_left_ * current_measured_left_;
    status.current_measured_right = sign_right_ * current_measured_right_;

    motor_status_pub_->publish(status);
  }

  // refer from: https://github.com/odriverobotics/ros_odrive/blob/5e4dfe9df8e5ef4fb6c692c210eafb713cb41985/odrive_base/include/odrive_enums.h#L43-L58
  const unsigned int kAxisStateIdol = 1;
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

  double spd_left_c_;
  double spd_right_c_;

  double dist_left_c_;
  double dist_right_c_;

  double current_setpoint_left_;
  double current_setpoint_right_;

  double current_measured_left_;
  double current_measured_right_;

  double sign_left_;
  double sign_right_;

  unsigned int axis_state_left_;
  unsigned int axis_state_right_;

  bool is_ready_axis_state_left_service_;
  bool is_ready_axis_state_right_service_;

  std::chrono::system_clock::time_point left_service_call_time_;
  std::chrono::system_clock::time_point right_service_call_time_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr control_message_left_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr control_message_right_pub_;
  rclcpp::Publisher<odriver_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_status_left_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_status_right_sub_;
  rclcpp::Subscription<odriver_msgs::msg::MotorTarget>::SharedPtr motor_target_sub_;

  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis_state_left_client_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis_state_right_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriverCanAdapterNode>());
  rclcpp::shutdown();

  return 0;
}
