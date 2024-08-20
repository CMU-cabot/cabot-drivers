/*******************************************************************************
 * Copyright (c) 2024  Miraikan and Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

//include header file
#include <cstdio>
#include <regex>
#include <chrono>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>

// include ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

// include custom msg
#include "power_controller_msgs/msg/battery_array.hpp"
#include "power_controller_msgs/srv/fan_controller.hpp"

// font color
#define ANSI_COLOR_CYAN   "\x1b[36m"

class PowerController : public rclcpp::Node
{
public:
  PowerController()
  : Node("power_controller_node"), check_power_(true), check_send_can_(false)
  {
    using namespace std::chrono_literals;
    // number of batteries
    this->declare_parameter<int>("number_of_batteries", 4);
    this->declare_parameter<std::string>("can_interface", "can0");
    // service
    service_server_24v_odrive = this->create_service<std_srvs::srv::SetBool>(
      "set_24v_power_odrive",
      std::bind(
        &PowerController::set24vPower,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_1 = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_1",
      std::bind(
        &PowerController::set12vPowerD4551,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_2 = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_2",
      std::bind(
        &PowerController::set12vPowerD4552,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_3 = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_3",
      std::bind(
        &PowerController::set12vPowerD4553,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_5v_MCU = this->create_service<std_srvs::srv::SetBool>(
      "set_5v_power_mcu",
      std::bind(
        &PowerController::set5vPowerMCU,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_shutdown = this->create_service<std_srvs::srv::Empty>(
      "shutdown",
      std::bind(
        &PowerController::shutdown,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_reboot = this->create_service<std_srvs::srv::Empty>(
      "reboot",
      std::bind(
        &PowerController::reboot,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_fan_pwm = this->create_service<power_controller_msgs::srv::FanController>(
      "fan_control",
      std::bind(
        &PowerController::fanControl,
        this, std::placeholders::_1,
        std::placeholders::_2));
    // publisher
    publisher_ = this->create_publisher<power_controller_msgs::msg::BatteryArray>("battery_state", 10);
    pub_timer_ = this->create_wall_timer(0.05s, std::bind(&PowerController::publisherPowerStatus, this));
    send_can_timer_ = this->create_wall_timer(0.5s, std::bind(&PowerController::sendCanMessageIfReceived, this));
    // open can socket
    can_socket_ = openCanSocket();
  }

  ~PowerController()
  {
    close(can_socket_);
  }

private:
  // can
  int openCanSocket()
  {
    std::string can_interface_ = this->get_parameter("can_interface").as_string();
    RCLCPP_ERROR(this->get_logger(), "%s", can_interface_.c_str());
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket");
      return -1;
    }
    struct ifreq ifr;
    // strcpy(ifr.ifr_name, "can0");
    strcpy(ifr.ifr_name, can_interface_.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind");
      close(s);
      return -1;
    }
    return s;
  }
  void sendCanFrame(int can_socket, can_frame & frame)
  {
    int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "The number of bytes is not equal to the number of bytes");
    }
  }
  void set24vPower(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    id = 0x14;
    if (req->data == 1) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 24V_odrive");
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 24V_odrive");
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    res->success = true;
    check_send_can_ = true;
  }
  void set12vPowerD4551(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    id = 0x16;
    if (req->data == 1) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_1");
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_1");
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    res->success = true;
    check_send_can_ = true;
  }
  void set12vPowerD4552(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    id = 0x17;
    if (req->data == 1) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_2");
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_2");
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    res->success = true;
    check_send_can_ = true;
  }
  void set12vPowerD4553(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    id = 0x18;
    if (req->data == 1) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_3");
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_3");
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    res->success = true;
    check_send_can_ = true;
  }
  void set5vPowerMCU(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    id = 0x19;
    if (req->data == 1) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 5V MCU");
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 5V MCU");
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    res->success = true;
    check_send_can_ = true;
  }
  void shutdown(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    id = 0x15;
    bool shutdown_ = false;
    std::memcpy(&check_power_, &shutdown_, sizeof(bool));
    (void)req;
    (void)res;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "Shutdown");
    check_send_can_ = true;
  }
  void reboot(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    id = 0x15;
    int reboot_ = 255;
    std::memcpy(&check_power_, &reboot_, sizeof(int));
    (void)req;
    (void)res;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "reboot");
    check_send_can_ = true;
  }
  void fanControl(
    const std::shared_ptr<power_controller_msgs::srv::FanController::Request> req,
    const std::shared_ptr<power_controller_msgs::srv::FanController::Response> res)
  {
    int pwm = req->data;
    id = 0x1d;
    if (-1 < pwm && pwm < 101) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "pwm is %d", pwm);
      std::memcpy(&check_power_, &req->data, sizeof(bool));
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "Only values from 0 to 100 can be entered");
    }
    res->response = true;
    check_send_can_ = true;
  }
  void sendCanMessageIfReceived()
  {
    if (check_send_can_){
      can_frame frame;
      frame.can_id = id;
      frame.can_dlc = 1;
      std::memcpy(&frame.data[0], &check_power_, 1);
      sendCanFrame(can_socket_, frame);
      check_send_can_ = false;
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "Send data");
    }
  }
  void combiningBit(
    unsigned char * frame_data, unsigned short* data1,
    unsigned short* data2, unsigned short* data3, unsigned short* data4)
  {
    *data1 = (frame_data[1] << 8) | frame_data[0];
    *data2 = (frame_data[3] << 8) | frame_data[2];
    *data3 = (frame_data[5] << 8) | frame_data[4];
    *data4 = (frame_data[7] << 8) | frame_data[6];
  }
  void defineMSG(
    power_controller_msgs::msg::BatteryArray & msg, int location_,
    unsigned short* data1, unsigned short* data2, unsigned short* data3, unsigned short* data4)
  {
    int array_num = location_ - 1;
    msg.batteryarray[array_num].header.stamp = this->get_clock()->now();
    msg.batteryarray[array_num].voltage = *data1;
    msg.batteryarray[array_num].current = *data2;
    msg.batteryarray[array_num].percentage = *data3;
    msg.batteryarray[array_num].temperature = *data4;
    msg.batteryarray[array_num].location = std::to_string(location_);
  }
  void publisherPowerStatus()
  {
    unsigned short data1;
    unsigned short data2;
    unsigned short data3;
    unsigned short data4;
    int location_;
    int num_batteries = this->get_parameter("number_of_batteries").as_int();
    msg.batteryarray.resize(num_batteries);
    struct can_frame frame;
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes > 0) {
      switch (frame.can_id) {
        case 0x05:  //Battery 1 Info
	  RCLCPP_INFO(this->get_logger(), "Get data of Battery 1");
	  location_ = 1;
	  combiningBit(frame.data, &data1, &data2, &data3, &data4);
	  // define msg's value
	  defineMSG(msg, location_, &data1, &data2, &data3, &data4);
	  break;
        case 0x06:  //Battery 2 Info
	  RCLCPP_INFO(this->get_logger(), "Get data of Battery 2");
	  location_ = 2;
	  combiningBit(frame.data, &data1, &data2, &data3, &data4);
	  defineMSG(msg, location_, &data1, &data2, &data3, &data4);
	  break;
        case 0x07:  //Battery 3 Info
	  location_ = 3;
	  RCLCPP_INFO(this->get_logger(), "Get data of Battery 3");
	  combiningBit(frame.data, &data1, &data2, &data3, &data4);
	  defineMSG(msg, location_, &data1, &data2, &data3, &data4);
	  break;
        case 0x08:  //Battery 4 Info
	  RCLCPP_INFO(this->get_logger(), "Get data of Battery 4");
	  location_ = 4;
	  combiningBit(frame.data, &data1, &data2, &data3, &data4);
	  defineMSG(msg, location_, &data1, &data2, &data3, &data4);
	  break;
        case 0x1c: //0x1d
	  RCLCPP_INFO(this->get_logger(), "Battery serial number");
	  combiningBit(frame.data, &data1, &data2, &data3, &data4);
	  msg.batteryarray[0].serial_number = std::to_string(data1);
	  msg.batteryarray[1].serial_number = std::to_string(data2);
	  msg.batteryarray[2].serial_number = std::to_string(data3);
	  msg.batteryarray[3].serial_number = std::to_string(data4);
	  // publsih msg
	  publisher_->publish(msg);
	  break;
        default:
	  break;
      }
    }
  }

  // can socket
  int can_socket_;
  bool check_power_;
  bool check_send_can_;
  uint8_t id;
  // define publish msg
  power_controller_msgs::msg::BatteryArray msg;
  // service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_24v_odrive;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_1;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_2;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_3;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_5v_MCU;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_shutdown;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_reboot;
  rclcpp::Service<power_controller_msgs::srv::FanController>::SharedPtr service_server_fan_pwm;
  // publisher
  rclcpp::Publisher<power_controller_msgs::msg::BatteryArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr send_can_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PowerController>());
  rclcpp::shutdown();
  return 0;
}

/*output
---
batteryarray:
- battery:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    voltage: 10.0
    temperature: 0.0
    current: 0.20000000298023224
    charge: 0.0
    capacity: 0.0
    design_capacity: 0.0
    percentage: 0.0
    power_supply_status: 0
    power_supply_health: 0
    power_supply_technology: 0
    present: false
    cell_voltage: []
    cell_temperature: []
    location: ''
    serial_number: '1'
  temperature:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    temperature: 20.5
    variance: 0.0
- battery:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    voltage: 11.0
    temperature: 0.0
    current: 1.2000000476837158
    charge: 0.0
    capacity: 0.0
    design_capacity: 0.0
    percentage: 0.10000000149011612
    power_supply_status: 0
    power_supply_health: 0
    power_supply_technology: 0
    present: false
    cell_voltage: []
    cell_temperature: []
    location: ''
    serial_number: '2'
  temperature:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    temperature: 21.5
    variance: 0.0
- battery:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    voltage: 12.0
    temperature: 0.0
    current: 2.200000047683716
    charge: 0.0
    capacity: 0.0
    design_capacity: 0.0
    percentage: 0.20000000298023224
    power_supply_status: 0
    power_supply_health: 0
    power_supply_technology: 0
    present: false
    cell_voltage: []
    cell_temperature: []
    location: ''
    serial_number: '3'
  temperature:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    temperature: 22.5
    variance: 0.0
---*/
