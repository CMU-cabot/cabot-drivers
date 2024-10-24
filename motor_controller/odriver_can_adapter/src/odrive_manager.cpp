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

#include "odrive_manager.hpp"

ODriveManager::ODriveManager(rclcpp::Node *node, const std::string &axis_name,
                              double sign, double wheel_diameter_m)
  : node_(node),
    is_ready_axis_state_service_(true),
    axis_name_(axis_name),
    sign_(sign),
    wheel_diameter_m_(wheel_diameter_m)
{
  using std::placeholders::_1;

  meter_per_round_ = wheel_diameter_m_ * M_PI;

  rclcpp::QoS control_message_qos(rclcpp::KeepLast(10));
  control_message_pub_ = node->create_publisher<odrive_can::msg::ControlMessage>(
                                  (std::string("/control_message_") + axis_name).c_str(),
                                  control_message_qos);

  rclcpp::QoS controller_status_qos(rclcpp::KeepLast(10));
  controller_status_sub_ = node->create_subscription<odrive_can::msg::ControllerStatus>(
                                  (std::string("/controller_status_") + axis_name).c_str(),
                                  controller_status_qos,
                                  std::bind(
                                    &ODriveManager::controllerStatusCallback,
                                    this,
                                    _1));

  client_ = node->create_client<odrive_can::srv::AxisState>(
                    (std::string("/request_axis_state_") + axis_name).c_str());

}

ODriveManager::~ODriveManager()
{
}

void ODriveManager::controllerStatusCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
{
  spd_c_ = msg->vel_estimate;
  dist_c_ = msg->pos_estimate;
  current_setpoint_ = msg->iq_setpoint;
  current_measured_ = msg->iq_measured;
  axis_state_ = msg->axis_state;
}

void ODriveManager::callAxisStateService(unsigned int axis_state)
{
  using AxisState = odrive_can::srv::AxisState;
  using AxisStateClient = rclcpp::Client<AxisState>;
  using std::chrono::system_clock;

  AxisState::Request::SharedPtr request = std::make_shared<AxisState::Request>();
  request->axis_requested_state = axis_state;

  if(is_ready_axis_state_service_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "call axis_state_%s service from %d mode to %d mode",
      axis_name_.c_str(),
      axis_state_,
      axis_state);
    while(!client_->wait_for_service(std::chrono::seconds(1)) &&
            rclcpp::ok()) {
      RCLCPP_INFO(
        node_->get_logger(),
        "request_axis_state_%s service not available, waiting again...",
        axis_name_.c_str());
    }

    is_ready_axis_state_service_ = false;
    last_call_time_ = system_clock::now();
    AxisStateClient::SharedFutureWithRequestAndRequestId axis_state_client_future_ =
      client_->async_send_request(
        request,
        [&](AxisStateClient::SharedFutureWithRequest future) {
          (void) future;
          is_ready_axis_state_service_ = true;
        });
  }
}

void ODriveManager::publishControlMessage(const odrive_can::msg::ControlMessage &msg)
{
  if(!is_ready_axis_state_service_) { return; }
  control_message_pub_->publish(msg);
}

void ODriveManager::publishControlMessage(int control_mode, int input_mode,
                                            double spd_m_per_sec)
{
  if(!is_ready_axis_state_service_) { return; }

  odrive_can::msg::ControlMessage msg;
  msg.control_mode = control_mode;
  msg.input_mode = input_mode;
  // meter/sec -> rotation/sec
  msg.input_vel = sign_ * spd_m_per_sec / meter_per_round_;

  control_message_pub_->publish(msg);
}

void ODriveManager::checkTimeoutServiceResponse(double timeout)
{
  using Milliseconds = std::chrono::milliseconds;
  using Seconds = std::chrono::seconds;
  using std::chrono::system_clock;

  double diff_time =
    std::chrono::duration_cast<Milliseconds>(last_call_time_ - system_clock::now()).count();
  if (!client_->wait_for_service(Seconds(0)) && diff_time >= timeout) {
    client_->prune_pending_requests();
    is_ready_axis_state_service_ = true;
  }
}