// Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// include header file
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cstdint>
#include <regex>
#include <chrono>
#include <list>
#include <mutex>
#include <atomic>

// include ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "sensor_msgs/msg/temperature.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// include custom msg
#include "power_controller_msgs/msg/battery_array.hpp"
#include "power_controller_msgs/srv/fan_controller.hpp"

// font color
#define ANSI_COLOR_CYAN(text)   "\x1b[36m" text "\x1b[0m"

// define value
#define PERIOD 0.001s
#define True_ 0x01
#define False_ 0x00
#define KELVIN 273.1
#define DUTY 2.55
#define MAJOR_CATEGORY_BATTERY_CAN_FILTER 0x100
#define CAN_MAJOR_CATEGORY_MASK 0x380

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
    this->declare_parameter<float>("low_temperature", 25.0);
    this->declare_parameter<float>("high_temperature", 50.0);
    this->declare_parameter<float>("max_fan", 100.0);
    this->declare_parameter<float>("min_fan", 40.0);

    // init battery message
    num_batteries_ = this->get_parameter("number_of_batteries").as_int();
    battery_message_.batteryarray.resize(num_batteries_);

    // service
    service_server_24v_odrive_ = this->create_service<std_srvs::srv::SetBool>(
      "set_24v_power_odrive",
      std::bind(
        &PowerController::set24vPower,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_front_ = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_front",
      std::bind(
        &PowerController::set12vPowerD455Front,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_left_ = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_left",
      std::bind(
        &PowerController::set12vPowerD455Left,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_12v_D455_right_ = this->create_service<std_srvs::srv::SetBool>(
      "set_12v_power_d455_right",
      std::bind(
        &PowerController::set12vPowerD455Right,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_5v_MCU_ = this->create_service<std_srvs::srv::SetBool>(
      "set_5v_power_mcu",
      std::bind(
        &PowerController::set5vPowerMCU,
        this, std::placeholders::_1,
        std::placeholders::_2));
    service_server_shutdown_ = this->create_service<std_srvs::srv::Trigger>(
      "shutdown",
      std::bind(
        &PowerController::shutdownALL,
        this, std::placeholders::_1,
        std::placeholders::_2));
    // fan subscriber
    subscriber_fan_ = this->create_subscription<std_msgs::msg::UInt8>(
      "fan_controller",
      10,
      std::bind(
        &PowerController::fanController,
        this,
        std::placeholders::_1));
    // temperature subscriber
    sub_temper_ = this->create_subscription<sensor_msgs::msg::Temperature>(
      "temperature3",
      10,
      std::bind(
        &PowerController::temperatureCallBack,
        this,
        std::placeholders::_1));
    // publisher
    state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    states_publisher_ = this->create_publisher<power_controller_msgs::msg::BatteryArray>("battery_states", 10);
    read_can_timer_ = this->create_wall_timer(PERIOD, std::bind(&PowerController::readCanMessageIfAvailable, this));
    send_can_timer_ = this->create_wall_timer(PERIOD, std::bind(&PowerController::sendCanMessageIfReceived, this));
    fan_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("fan_controller", 10);
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
    strncpy(ifr.ifr_name, can_interface_.c_str(), sizeof(ifr.ifr_name));
    ioctl(s, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "bind");
      close(s);
      return -1;
    }
    struct can_filter filters[1];
    filters[0].can_id = MAJOR_CATEGORY_BATTERY_CAN_FILTER;
    filters[0].can_mask = CAN_MAJOR_CATEGORY_MASK;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in setsockopt for CAN filter");
      close(s);
      return -1;
    }
    // Set the socket to non-blocking mode
    int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0) {
      perror("fcntl GETFL failed");
      close(s);
      return -1;
    }
    if (fcntl(s, F_SETFL, flags | O_NONBLOCK) < 0) {
      perror("fcntl SETFL failed");
      close(s);
      return -1;
    }
    RCLCPP_INFO(this->get_logger(), "Success open CAN %s (%d)", can_interface_.c_str(), s);
    return s;
  }
  void sendCanFrame(int can_socket, can_frame & frame)
  {
    int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes == sizeof(struct can_frame)) {
      check_send_data_ = true;
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "the number of bytes is not equal to the number of bytes");
    check_send_data_ = false;
  }
  void set24vPower(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx_.lock();
    id_ = CanId::odrive_id;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn on 24V_odrive"));
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn off 24V_odrive"));
      power_ = False_;
    }
    std::memcpy(&send_can_value_, &power_, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    mtx_.unlock();
    res->success = true;
  }
  void set12vPowerD455Front(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx_.lock();
    id_ = CanId::d455_front_id;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn on front D455"));
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn off front D455"));
      power_ = False_;
    }
    std::memcpy(&send_can_value_, &power_, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    mtx_.unlock();
    res->success = true;
  }
  void set12vPowerD455Left(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx_.lock();
    id_ = CanId::d455_left_id;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn on left D455"));
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn off left D455"));
      power_ = False_;
    }
    std::memcpy(&send_can_value_, &power_, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    mtx_.unlock();
    res->success = true;
  }
  void set12vPowerD455Right(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx_.lock();
    id_ = CanId::d455_right_id;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn on right  D455"));
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn off right D455"));
      power_ = False_;
    }
    std::memcpy(&send_can_value_, &power_, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    mtx_.unlock();
    res->success = true;
  }
  void set5vPowerMCU(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    mtx_.lock();
    id_ = CanId::mcu_id;
    if (req->data) {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn on 5V MCU"));
      power_ = True_;
    } else {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("turn off 5V MCU"));
      power_ = False_;
    }
    std::memcpy(&send_can_value_, &power_, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    mtx_.unlock();
    res->success = true;
  }
  void shutdownALL(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("shutdown service is called"));
    mtx_.lock();
    id_ = CanId::power_id;
    uint8_t shutdown_ = False_;
    std::memcpy(&send_can_value_, &shutdown_, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    (void)req;
    res->success = true;
    mtx_.unlock();
  }
  void fanController(std_msgs::msg::UInt8 msg)
  {
    uint8_t data_ = msg.data;
    if (data_ > 100) {
      RCLCPP_ERROR(this->get_logger(), "only values from 0 to 100 can be entered");
      return;
    }
    mtx_.lock();
    id_ = CanId::fan_id;
    double d_pwm = data_ * DUTY;
    uint8_t pwm = static_cast<uint8_t>(std::round(d_pwm));
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("pwm is %d %% (0x%x)"), data_, pwm);
    std::memcpy(&send_can_value_, &pwm, sizeof(bool));
    send_can_value_list_.push_back({id_, send_can_value_});
    mtx_.unlock();
  }
  void temperatureCallBack(sensor_msgs::msg::Temperature temp_msg)
  {
    std_msgs::msg::UInt8 fan_msg;
    double framos_temperature = temp_msg.temperature;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("temperature is %f"), framos_temperature);
    float HIGHTEMPERATURE, LOWTEMPERATURE, MINFAN, MAXFAN;
    this->get_parameter("high_temperature", HIGHTEMPERATURE);
    this->get_parameter("low_temperature", LOWTEMPERATURE);
    this->get_parameter("min_fan", MINFAN);
    this->get_parameter("max_fan", MAXFAN);
    if (framos_temperature >= HIGHTEMPERATURE) {
      fan_msg.data = MAXFAN;
    } else if (framos_temperature <= LOWTEMPERATURE) {
      fan_msg.data = MINFAN;
    } else {
      float slope = (MAXFAN - MINFAN) / (HIGHTEMPERATURE - LOWTEMPERATURE);
      float intercept = MAXFAN - (HIGHTEMPERATURE * slope);
      fan_msg.data = framos_temperature * slope + intercept;
    }
    fan_publisher_->publish(fan_msg);
  }
  void sendCanMessageIfReceived()
  {
    can_frame frame;
    if (send_can_value_list_.empty()) {
      return;
    }
    mtx_.lock();
    SendCanValueInfo send_can_value_temp = send_can_value_list_.front();
    frame.can_id = send_can_value_temp.id;
    frame.can_dlc = 1;
    frame.data[0] = send_can_value_temp.data;
    sendCanFrame(can_socket_, frame);
    if (!check_send_data_) {
      RCLCPP_ERROR(this->get_logger(), "can not send data");
      mtx_.unlock();
      return;
    }
    send_can_value_list_.pop_front();
    mtx_.unlock();
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN("send data"));
  }
  // message & combine bits
  void combineBitAndUpdateMessage(int location, uint8_t * frame_data)
  {
    uint16_t voltage = (frame_data[1] << 8) | frame_data[0];
    int16_t current = (frame_data[3] << 8) | frame_data[2];
    uint16_t percentage = (frame_data[5] << 8) | frame_data[4];
    uint16_t temperature = (frame_data[7] << 8) | frame_data[6];

    int array_num = location - 1;
    battery_message_.batteryarray[array_num].header.stamp = this->get_clock()->now();

    // Converts voltage units from mV to V
    if (voltage == 0xFFFF) {
      battery_message_.batteryarray[array_num].voltage = std::nan("");
    } else {
      battery_message_.batteryarray[array_num].voltage = static_cast<float>(voltage) / 1000.0f;
    }
    // Converts current units from mA to A
    if (current == -1) {
      battery_message_.batteryarray[array_num].current = std::nan("");
    } else {
      battery_message_.batteryarray[array_num].current = static_cast<float>(current) / 1000.0f;  // negative is discharging, positive is charging
    }
    // Convert percentage to a ratio
    if (percentage == 0xFFFF) {
      battery_message_.batteryarray[array_num].percentage = std::nan("");
    } else {
      battery_message_.batteryarray[array_num].percentage = static_cast<float>(percentage) / 100.0f;
    }
    // Convert absolute temperature x 10 to Celsius
    if (temperature == 0xFFFF) {
      battery_message_.batteryarray[array_num].temperature = std::nan("");
    } else {
      battery_message_.batteryarray[array_num].temperature = static_cast<float>(temperature) / 10.0f - KELVIN;
    }
    battery_message_.batteryarray[array_num].location = std::to_string(location);
  }
  void publishBatteryStates(uint8_t * frame_data)
  {
    std::string (* to_hex)(uint16_t) = [](uint16_t data) {
        std::stringstream ss;
        ss << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(data);
        return ss.str();
      };
    battery_message_.batteryarray[0].serial_number = to_hex(frame_data[1] << 8 | frame_data[0]);
    battery_message_.batteryarray[1].serial_number = to_hex(frame_data[3] << 8 | frame_data[2]);
    battery_message_.batteryarray[2].serial_number = to_hex(frame_data[5] << 8 | frame_data[4]);
    battery_message_.batteryarray[3].serial_number = to_hex(frame_data[7] << 8 | frame_data[6]);
    // publish msg
    states_publisher_->publish(battery_message_);
    RCLCPP_INFO(get_logger(), "publish battery_states");

    sensor_msgs::msg::BatteryState average_battery_msg;
    average_battery_msg.header.stamp = this->get_clock()->now();
    average_battery_msg.serial_number = '0';
    average_battery_msg.location = '0';

    // average each property of battery_message_.batteryarray and ignore NaN values
    float percentage_sum = 0, voltage_sum = 0, current_sum = 0, temperature_sum = 0;
    int valid_percentage_count = 0, valid_voltage_count = 0, valid_current_count = 0, valid_temperature_count = 0;

    void (* checkNotANumber)(float, float &, int &) = [](float data, float & sum, int & count) {
        if (!std::isnan(data)) {
          sum += data;
          count++;
        }
      };
    for (int i = 0; i < static_cast<int>(battery_message_.batteryarray.size()); i++) {
      checkNotANumber(battery_message_.batteryarray[i].voltage, voltage_sum, valid_voltage_count);
      checkNotANumber(battery_message_.batteryarray[i].current, current_sum, valid_current_count);
      checkNotANumber(battery_message_.batteryarray[i].percentage, percentage_sum, valid_percentage_count);
      checkNotANumber(battery_message_.batteryarray[i].temperature, temperature_sum, valid_temperature_count);
    }
    average_battery_msg.voltage = valid_voltage_count > 0 ? voltage_sum / valid_voltage_count : std::nan("");
    average_battery_msg.current = valid_current_count > 0 ? current_sum / valid_current_count : std::nan("");
    average_battery_msg.percentage = valid_percentage_count > 0 ? percentage_sum / valid_percentage_count : std::nan("");
    average_battery_msg.temperature = valid_temperature_count > 0 ? temperature_sum / valid_temperature_count : std::nan("");
    state_publisher_->publish(average_battery_msg);
    RCLCPP_INFO(get_logger(), "publish battery_state %f %f %f %f", voltage_sum, current_sum, percentage_sum, temperature_sum);

    // clear old data
    battery_message_.batteryarray.clear();
    battery_message_.batteryarray.resize(num_batteries_);
    return;
  }
  void readCanMessageIfAvailable()
  {
    struct can_frame frame;
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      return;
    }
    RCLCPP_INFO(get_logger(), "received CAN frame %x (%d)", frame.can_id, frame.can_dlc);
    switch (frame.can_id) {
      case CanId::battery_id_1:   // Battery 1 Info
        combineBitAndUpdateMessage(1, frame.data);
        break;
      case CanId::battery_id_2:   // Battery 2 Info
        combineBitAndUpdateMessage(2, frame.data);
        break;
      case CanId::battery_id_3:   // Battery 3 Info
        combineBitAndUpdateMessage(3, frame.data);
        break;
      case CanId::battery_id_4:   // Battery 4 Info
        combineBitAndUpdateMessage(4, frame.data);
        break;
      case CanId::battery_serial_number:  // Serial Number
        publishBatteryStates(frame.data);
        break;
      case CanId::seq_id:
        if (frame.can_dlc == 1 && frame.data[0] == 0) {
          shutdown();
        }
        break;
      default:
        break;
    }
  }
  void shutdown()
  {
    RCLCPP_INFO(get_logger(), "shutting down");
    int ret_code = std::system("sudo systemctl poweroff");
  }

  // enum
  enum CanId : uint16_t
  {
    battery_id_1 = 0x518,
    battery_id_2,
    battery_id_3,
    battery_id_4,
    ems_id = 0x100,
    stat_id,
    ind_id,
    seq_id,
    odrive_id = 0x108,
    power_id,
    d455_front_id,
    d455_left_id,
    d455_right_id,
    mcu_id,
    fan_id,
    battery_serial_number = 0x520
  };

  // struct
  struct SendCanValueInfo
  {
    uint16_t id;
    uint16_t data;
  };
  // value
  uint16_t send_can_value_;
  uint16_t id_;
  uint8_t power_;
  // flag
  bool check_send_data_;
  int num_batteries_;
  // list
  std::list<SendCanValueInfo> send_can_value_list_;
  // can socket
  int can_socket_;
  // define publish msg
  power_controller_msgs::msg::BatteryArray battery_message_;
  // service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_24v_odrive_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_front_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_left_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_right_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_5v_MCU_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server_shutdown_;
  // subscriber
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_fan_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr sub_temper_;
  // publisher
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr state_publisher_;
  rclcpp::Publisher<power_controller_msgs::msg::BatteryArray>::SharedPtr states_publisher_;
  rclcpp::TimerBase::SharedPtr read_can_timer_;
  rclcpp::TimerBase::SharedPtr send_can_timer_;
  // mutex
  std::mutex mtx_;
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
