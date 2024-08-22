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

  }

  void controllerStatusRightCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {

  }

  void motorTargetCallback(const odriver_msgs::msg::MotorTarget::SharedPtr msg)
  {
    // WIP: what's loop control
    odrive_can::msg::ControlMessage left_message;
    odrive_can::msg::ControlMessage right_message;
    
    // set velocity mode
    left_message.control_mode = 2;
    right_message.control_mode = 2;
    // WIP: search this param
    left_message.input_mode = 1;
    right_message.input_mode = 1;

    double meter_per_sec_left = msg->spd_left;
    double meter_per_sec_right = msg->spd_right;

    // TEMP
    double wheel_diameter = 0.170;
    double meter_per_round = wheel_diameter * M_PI;

    // meter/sec -> rotation/sec
    left_message.input_vel = msg->spd_left / meter_per_round;
    right_message.input_vel = msg->spd_right / meter_per_round;

    control_message_left_pub_->publish(left_message);
    control_message_right_pub_->publish(right_message);
  }

  void timerCallback()
  {
    odriver_msgs::msg::MotorStatus status;

    status.header.stamp = this->get_clock()->now();

    motor_status_pub_->publish(status);
  }

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
