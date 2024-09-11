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
#include <list>
#include <mutex>
#include <atomic>

// include ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

// include custom msg
#include "power_controller_msgs/msg/battery_array.hpp"
#include "power_controller_msgs/srv/fan_controller.hpp"

// font color
#define ANSI_COLOR_CYAN   "\x1b[36m"
#define PERIOD 0.001s
#define True_ 0x01
#define False_ 0x00
#define Kelvin 2731.0 // Kelvin = 273.1 * 10
#define Duty 2.55

class PowerController : public rclcpp::Node
{
public:
  PowerController()
  : Node("power_controller_node")
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
    service_server_12v_D455_front = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_front",
      std::bind(
        &PowerController::set12vPowerD455Front,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_left = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_left",
      std::bind(
        &PowerController::set12vPowerD455Left,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_right = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_right",
      std::bind(
        &PowerController::set12vPowerD455Right,
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
        &PowerController::shutdownALL,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_reboot = this->create_service<std_srvs::srv::Empty>(
      "reboot",
      std::bind(
        &PowerController::rebootALL,
        this, std::placeholders::_1,
        std::placeholders::_2));
    // fan subscriber
    subscriber_fan_ = this->create_subscription<std_msgs::msg::UInt8>(
      "fan_controller",
      10,
      std::bind(
        &PowerController::fanControllerCallBack,
        this,
        std::placeholders::_1));
    // publisher
    publisher_ = this->create_publisher<power_controller_msgs::msg::BatteryArray>("battery_state", 10);
    pub_timer_ = this->create_wall_timer(PERIOD, std::bind(&PowerController::publishPowerStatus, this));
    send_can_timer_ = this->create_wall_timer(PERIOD, std::bind(&PowerController::sendCanMessageIfReceived, this));
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
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
      RCLCPP_ERROR(this->get_logger(), "socket");
      return -1;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "bind");
      close(s);
      return -1;
    }
    return s;
  }
  void sendCanFrame(int can_socket, can_frame & frame)
  {
    int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes == sizeof(struct can_frame)) {
      check_send_data = true;
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "the number of bytes is not equal to the number of bytes");
    check_send_data = false;
  }
  void set24vPower(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx.lock();
    id = 0x15;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 24V_odrive");
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 24V_odrive");
      power_ = False_;
    }
    std::memcpy(&send_can_value, &power_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    mtx.unlock();
    res->success = true;
  }
  void set12vPowerD455Front(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx.lock();
    id = 0x17;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_1");
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_1");
      power_ = False_;
    }
    std::memcpy(&send_can_value, &power_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    mtx.unlock();
    res->success = true;
  }
  void set12vPowerD455Left(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx.lock();
    id = 0x18;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_2");
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_2");
      power_ = False_;
    }
    std::memcpy(&send_can_value, &power_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    mtx.unlock();
    res->success = true;
  }
  void set12vPowerD455Right(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx.lock();
    id = 0x19;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_3");
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_3");
      power_ = False_;
    }
    std::memcpy(&send_can_value, &power_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    mtx.unlock();
    res->success = true;
  }
  void set5vPowerMCU(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx.lock();
    id = 0x1a;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 5V MCU");
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 5V MCU");
      power_ = False_;
    }
    std::memcpy(&send_can_value, &power_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    mtx.unlock();
    res->success = true;
  }
  void shutdownALL(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    mtx.lock();
    id = 0x16;
    uint8_t shutdown_ = False_;
    std::memcpy(&send_can_value, &shutdown_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    (void)req;
    (void)res;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "shutdown");
    mtx.unlock();
  }
  void rebootALL(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    mtx.lock();
    id = 0x16;
    uint8_t reboot_ = 255;
    std::memcpy(&send_can_value, &reboot_, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    (void)req;
    (void)res;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "reboot");
    mtx.unlock();
  }
  void fanControllerCallBack(std_msgs::msg::UInt8 msg){
    uint8_t data_ = msg.data;
    if (data_ > 100) {
      RCLCPP_ERROR(this->get_logger(), "only values from 0 to 100 can be entered");
      return;
    }
    mtx.lock();
    id = 0x1e;
    uint8_t pwm = data_ * Duty;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "pwm is %d", pwm);
    std::memcpy(&send_can_value, &pwm, sizeof(bool));
    send_can_value_list.push_back({id, send_can_value});
    mtx.unlock();
  }
  void sendCanMessageIfReceived()
  {
    can_frame frame;
    if (send_can_value_list.empty()) {
      return;
    }
    mtx.lock();
    SendCanValueInfo send_can_value_temp = send_can_value_list.front();
    frame.can_id = send_can_value_temp.id;
    frame.can_dlc = 1;
    frame.data[0] = send_can_value_temp.data;
    sendCanFrame(can_socket_, frame);
    if (!check_send_data){
      RCLCPP_ERROR(this->get_logger(), "can not send data");
      return;
    }
    send_can_value_list.pop_front();
    mtx.unlock();
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "send data");
  }
  float convertUnit(uint16_t data_, bool temperature_flag_ = false){
    float result;
    if (temperature_flag_){
      result = (data_ - Kelvin) / 10.0;
      return result;
    }
    result = static_cast<float>(data_) / 1000.0;
    return result;
  }
  //message & combine bits
  void conbineBitAndUpdateMessage(
    power_controller_msgs::msg::BatteryArray & msg, int location_,
    uint8_t* frame_data, uint16_t data[4], bool battery_serial_number_flag = false){    
    data[0] = (frame_data[1] << 8) | frame_data[0];
    data[1] = (frame_data[3] << 8) | frame_data[2];
    data[2] = (frame_data[5] << 8) | frame_data[4];
    data[3] = (frame_data[7] << 8) | frame_data[6];
    if (battery_serial_number_flag){
      msg.batteryarray[0].serial_number = std::to_string(data[0]);
      msg.batteryarray[1].serial_number = std::to_string(data[1]);
      msg.batteryarray[2].serial_number = std::to_string(data[2]);
      msg.batteryarray[3].serial_number = std::to_string(data[3]);
      // publsih msg
      publisher_->publish(msg);
      return;
    }
    int array_num = location_ - 1;
    msg.batteryarray[array_num].header.stamp = this->get_clock()->now();
    // Converts voltage units from mV to V
    msg.batteryarray[array_num].voltage = convertUnit(data[0]);
    // Converts current units from mA to A
    msg.batteryarray[array_num].current = convertUnit(-data[1]); // Signs are inverted because hexadecimal data is negative.
    msg.batteryarray[array_num].percentage = data[2];
    // Convert absolute temperature to Celsius
    msg.batteryarray[array_num].temperature = convertUnit(data[3], temperature_flag_);
    msg.batteryarray[array_num].location = std::to_string(location_);
  }
  void publishPowerStatus()
  {
    uint16_t data[4];
    int location_;
    int num_batteries = this->get_parameter("number_of_batteries").as_int();
    msg.batteryarray.resize(num_batteries);
    bool battery_serial_number_flag = true;
    struct can_frame frame;
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      return;
    }
    switch (frame.can_id) {
       case CanId::battery_id_1:  // Battery 1 Info
	 location_ = 1;
	 conbineBitAndUpdateMessage(msg, location_, frame.data, data);
	 break;
       case CanId::battery_id_2:  // Battery 2 Info
	 location_ = 2;
	 conbineBitAndUpdateMessage(msg, location_, frame.data, data);
	 break;
       case CanId::battery_id_3:  // Battery 3 Info
	 location_ = 3;
	 conbineBitAndUpdateMessage(msg, location_, frame.data, data);
	 break;
       case CanId::battery_id_4:  // Battery 4 Info
	 location_ = 4;
	 conbineBitAndUpdateMessage(msg, location_, frame.data, data);
	 break;
       case CanId::battery_serial_number: // Serial Number
	 location_ = 0;
	 conbineBitAndUpdateMessage(msg, location_, frame.data, data, battery_serial_number_flag);
	 break;
       default:
	 break;
      }
  }

  //enum
  enum CanId : uint8_t{
    battery_id_1 = 0x05,
    battery_id_2 = 0x06,
    battery_id_3 = 0x07,
    battery_id_4 = 0x08,
    battery_serial_number = 0x1d
  };

  // struct
  struct SendCanValueInfo {
    uint8_t id;
    uint8_t data;
  };
  // value
  uint8_t send_can_value;
  uint8_t id;
  uint8_t power_;
  // flag
  bool check_send_data;
  bool temperature_flag_ = true;
  // list
  std::list<SendCanValueInfo> send_can_value_list;
  // can socket
  int can_socket_;
  // define publish msg
  power_controller_msgs::msg::BatteryArray msg;
  // service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_24v_odrive;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_front;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_left;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_right;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_5v_MCU;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_shutdown;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_reboot;
  // subscriber
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_fan_;
  // publisher
  rclcpp::Publisher<power_controller_msgs::msg::BatteryArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr send_can_timer_;
  //mutex
  std::mutex mtx;
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
