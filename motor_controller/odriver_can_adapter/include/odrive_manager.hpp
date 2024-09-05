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

#pragma once

class ODriveManager
{
public:
  ODriveManager(rclcpp::Node *node, const std::string &axis_name, double sign);
  ~ODriveManager();

  void setControlStatusMessage(const odrive_can::msg::ControllerStatus::SharedPtr msg);
  void callAxisStateService(unsigned int axis_state);
private:
  rclcpp::Node *node_;

  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_;
  // call 
  std::chrono::system_clock::time_point last_call_time_;
  bool is_ready_axis_state_service_;
  // "left" or "right"
  std::string axis_name_;
  unsigned int axis_state_;

  double spd_c_;
  double dist_c_;
  double current_setpoint_;
  double current_measured_;

  double sign_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr control_message_pub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_status_sub_;
};