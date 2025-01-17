//
// Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
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
//

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_srvs/srv/trigger.hpp"

#define CAN_MAJOR_CATEGORY_MASK 0x380  // 0b0 111 0000 000

enum CanId : uint16_t
{
  IMU_LINEAR_CAN_ID = 0x010,
  IMU_ANGULAR_CAN_ID,
  IMU_ORIENTATION_CAN_ID,
  BME_CAN_ID = 0x018,
  TEMPERATURE_1_CAN_ID = 0x021,
  TEMPERATURE_2_CAN_ID,
  TEMPERATURE_3_CAN_ID,
  TEMPERATURE_4_CAN_ID,
  TEMPERATURE_5_CAN_ID,
  TOUCH_CAN_ID = 0x080,
  TACT_CAN_ID = 0x089,
  VIBRATOR_CAN_ID = 0x090,
  SERVO_FREE_CAN_ID = 0x098,
  SERVO_TARGET_CAN_ID,
  SERVO_POS_CAN_ID,
  WIFI_CAN_ID_1 = 0x428,
  WIFI_CAN_ID_2,
  WIFI_CAN_ID_3,
  WIFI_CAN_ID_4,
  WIFI_CAN_ID_5,
  IMU_CALIBRATION_SEND_CAN_ID = 0x430,
  WRITE_IMU_CALIBRATION_ID_1,
  WRITE_IMU_CALIBRATION_ID_2,
  WRITE_IMU_CALIBRATION_ID_3,
  READ_IMU_CALIBRATION_ID_1 = 0x439,
  READ_IMU_CALIBRATION_ID_2,
  READ_IMU_CALIBRATION_ID_3,
  CAPACITIVE_TOUCH_STATUS_ID = 0x481,
  CAPACITIVE_TOUCH_CALIBRATION_ID,
  CAPACITIVE_TOUCH_RECALIBRATION_ID,
  CAPACITIVE_TOUCH_SENSOR_INPUT_ENABLE_ID,
  CAPACITIVE_TOUCH_NULLIFY_NOISE_ID,
  CAPACITIVE_TOUCH_BC_OUT_RECALIBRATION,
};

enum CanDlc : uint8_t
{
  IMU_LINEAR_CAN_DLC = 6,
  IMU_ANGULAR_CAN_DLC = 6,
  IMU_ORIENTATION_CAN_DLC = 8,
  WIFI_CAN_1_DLC = 8,
  WIFI_CAN_2_DLC = 8,
  WIFI_CAN_3_DLC = 8,
  WIFI_CAN_4_DLC = 8,
  WIFI_CAN_5_DLC = 8,
  WIFI_BSSID_CAN_DLC = 6,
  BME_CAN_DLC = 8,
  TOUCH_CAN_DLC = 4,
  TACT_CAN_DLC = 1,
  VIBRATOR_CAN_DLC = 3,
  SERVO_TARGET_CAN_DLC = 4,
  SERVO_POS_CAN_DLC = 4,
  SERVO_FREE_CAN_DLC = 1,
  TEMPERATURE_1_CAN_DLC = 2,
  TEMPERATURE_2_CAN_DLC = 2,
  TEMPERATURE_3_CAN_DLC = 2,
  TEMPERATURE_4_CAN_DLC = 2,
  TEMPERATURE_5_CAN_DLC = 2,
  WRITE_IMU_CALIBRATION_1_DLC = 8,
  WRITE_IMU_CALIBRATION_2_DLC = 8,
  WRITE_IMU_CALIBRATION_3_DLC = 6,
  READ_IMU_CALIBRATION_1_DLC = 8,
  READ_IMU_CALIBRATION_2_DLC = 8,
  READ_IMU_CALIBRATION_3_DLC = 6,
  IMU_CALIBRATION_SEND_CAN_DLC = 1,
  CAPACITIVE_TOUCH_STATUS_CAN_DLC = 3,
  CAPACITIVE_TOUCH_CALIBRATION_CAN_DLC = 1,
  CAPACITIVE_TOUCH_RECALIBRATION_CAN_DLC = 1,
  CAPACITIVE_TOUCH_SENSOR_INPUT_ENABLE_DLC = 1,
  CAPACITIVE_TOUCH_NULLIFY_NOISE_DLC = 1,
  CAPACITIVE_TOUCH_BC_OUT_RECALIBRATION_DLC = 1,
};

enum CanFilter : uint8_t
{
  MAJOR_CATEGORY_SENSOR_CAN_FILTER = 0x000,  // 0b0 000 0000 000
  MAJOR_CATEGORY_HANDLE_CAN_FILTER = 0x080,  // 0b0 001 0000 000
};

class CabotCanNode : public rclcpp::Node
{
public:
  CabotCanNode()
  : touch_mode_("cap"),
    tof_touch_threshold_(25),
    Node("cabot_can_node")
  {
    declare_parameter("touch_mode", touch_mode_);
    declare_parameter("tof_touch_threshold", tof_touch_threshold_);
    declare_parameter("publish_rate", 200);
    declare_parameter("can_interface", "can0");
    declare_parameter("imu_frame_id", "imu_frame");
    declare_parameter("calibration_params", std::vector<int64_t>((CanDlc::WRITE_IMU_CALIBRATION_1_DLC + CanDlc::WRITE_IMU_CALIBRATION_2_DLC + CanDlc::WRITE_IMU_CALIBRATION_3_DLC), 0));
    temperature_1_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature1", 10);
    temperature_2_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature2", 10);
    temperature_3_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature3", 10);
    temperature_4_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature4", 10);
    temperature_5_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature5", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    wifi_pub_ = this->create_publisher<std_msgs::msg::String>("wifi", 10);
    bme_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("bme/temperature", 10);
    bme_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("bme/pressure", 10);
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure", 10);
    calibration_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("calibration", 10);
    tact_pub_ = this->create_publisher<std_msgs::msg::Int8>("pushed", 10);
    capacitive_touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("capacitive/touch", 10);
    tof_touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("tof/touch", 10);
    touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch", 10);
    capacitive_touch_raw_pub_ = this->create_publisher<std_msgs::msg::Int16>("capacitive/touch_raw", 10);
    tof_touch_raw_pub_ = this->create_publisher<std_msgs::msg::Int16>("tof/touch_raw", 10);
    servo_pos_pub_ = this->create_publisher<std_msgs::msg::Int16>("servo_pos", 10);
    vibrator_1_sub_ =
      this->create_subscription<std_msgs::msg::UInt8>(
      "vibrator1", 10,
      static_cast<std::function<void(std_msgs::msg::UInt8::SharedPtr)>>(std::bind(&CabotCanNode::subscribeVibratorData, this, std::placeholders::_1, 1)));
    vibrator_3_sub_ =
      this->create_subscription<std_msgs::msg::UInt8>(
      "vibrator3", 10,
      static_cast<std::function<void(std_msgs::msg::UInt8::SharedPtr)>>(std::bind(&CabotCanNode::subscribeVibratorData, this, std::placeholders::_1, 3)));
    vibrator_4_sub_ =
      this->create_subscription<std_msgs::msg::UInt8>(
      "vibrator4", 10,
      static_cast<std::function<void(std_msgs::msg::UInt8::SharedPtr)>>(std::bind(&CabotCanNode::subscribeVibratorData, this, std::placeholders::_1, 4)));
    servo_target_sub_ = this->create_subscription<std_msgs::msg::Int16>("servo_target", 10, std::bind(&CabotCanNode::subscribeServoTargetData, this, std::placeholders::_1));
    servo_free_sub_ = this->create_subscription<std_msgs::msg::Bool>("servo_free", 10, std::bind(&CabotCanNode::subServoFree, this, std::placeholders::_1));
    capacitive_recalibration_sub_ = this->create_subscription<std_msgs::msg::UInt8>("capacitive/recalibration", 10, std::bind(&CabotCanNode::subscribeCapacitiveRecalibrationData, this, std::placeholders::_1));
    capacitive_bc_out_recalibration_sub_ = this->create_subscription<std_msgs::msg::UInt8>("capacitive/bc_out_recalibration", 10, std::bind(&CabotCanNode::subscribeCapacitiveBcOutRecalibration, this, std::placeholders::_1));
    imu_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>("run_imu_calibration", std::bind(&CabotCanNode::readImuServiceCalibration, this, std::placeholders::_1, std::placeholders::_2));
    capacitive_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>("run_capacitive_calibration", std::bind(&CabotCanNode::capacitiveServiceCalibration, this, std::placeholders::_1, std::placeholders::_2));
    capacitive_noise_nullify_srv_ = this->create_service<std_srvs::srv::Trigger>("nullify_capacitive_noise", std::bind(&CabotCanNode::nullifyCapacitiveNoise, this, std::placeholders::_1, std::placeholders::_2));
    imu_accel_bias_ = this->declare_parameter("imu_accel_bias", std::vector<double>(3, 0.0));  // parameters for adjusting linear acceleration. default value = [0,0, 0.0, 0.0]
    imu_gyro_bias_ = this->declare_parameter("imu_gyro_bias", std::vector<double>(3, 0.0));  // parameters for adjusting angular velocity. default value = [0,0, 0.0, 0.0]
    can_socket_ = openCanSocket();
    writeImuCalibration();
    capacitiveSensorInputEnable();
    // initialize diagonal elements of imu_msg.orientation_covariance
    imu_msg.orientation_covariance[0] = 0.1;
    imu_msg.orientation_covariance[4] = 0.1;
    imu_msg.orientation_covariance[8] = 0.1;
    std::uint8_t publish_rate = this->get_parameter("publish_rate", publish_rate);
    pub_timer_ = this->create_wall_timer(std::chrono::microseconds(publish_rate), std::bind(&CabotCanNode::timerPubCallback, this));
    callback_handler_ =
      this->add_on_set_parameters_callback(std::bind(&CabotCanNode::param_set_callback, this, std::placeholders::_1));

  }

  rcl_interfaces::msg::SetParametersResult param_set_callback(
    const std::vector<rclcpp::Parameter>& params)
  {
    for (auto && param : params) {
      if (!this->has_parameter(param.get_name())) {
        continue;
      }
      if (param.get_name() == "touch_mode") {
        touch_mode_ = param.as_string();
      }
      if (param.get_name() == "tof_touch_threshold") {
        tof_touch_threshold_ = param.as_int();
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

private:
  int openCanSocket()
  {
    std::string can_interface = this->get_parameter("can_interface").as_string();
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error while opening socket");
    }
    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_interface.c_str(), sizeof(ifr.ifr_name));
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in ioctl");
      close(s);
      return -1;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
      close(s);
    }
    struct can_filter filters[2];
    filters[1].can_id = CanFilter::MAJOR_CATEGORY_SENSOR_CAN_FILTER;
    filters[0].can_mask = CAN_MAJOR_CATEGORY_MASK;
    filters[0].can_id = CanFilter::MAJOR_CATEGORY_HANDLE_CAN_FILTER;
    filters[1].can_mask = CAN_MAJOR_CATEGORY_MASK;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in setsockopt for CAN filter");
      close(s);
      return -1;
    }
    return s;
  }

  void writeImuCalibration()
  {
    std::vector<int64_t> calibration_params = get_parameter("calibration_params").as_integer_array();
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::IMU_CALIBRATION_SEND_CAN_ID;
    frame.can_dlc = CanDlc::IMU_CALIBRATION_SEND_CAN_DLC;
    frame.data[0] = 1;
    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send write Imu calibration frame");
    } else {
      RCLCPP_INFO(this->get_logger(), "Write Imu calibration sent successfully");
    }
    // Insert usleep to avoid hanging when writing to CAN in one go.
    usleep(1000);
    frame.can_id = CanId::WRITE_IMU_CALIBRATION_ID_1;
    frame.can_dlc = CanDlc::WRITE_IMU_CALIBRATION_1_DLC;
    for (int i = 0; i < CanDlc::WRITE_IMU_CALIBRATION_1_DLC; ++i) {
      frame.data[i] = (calibration_params[i] >> 0) & 0xFF;
    }
    nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    // Insert usleep to avoid hanging when writing to CAN in one go.
    usleep(1000);
    frame.can_id = CanId::WRITE_IMU_CALIBRATION_ID_2;
    frame.can_dlc = CanDlc::WRITE_IMU_CALIBRATION_2_DLC;
    for (int i = 0; i < CanDlc::WRITE_IMU_CALIBRATION_2_DLC; ++i) {
      frame.data[i] = (calibration_params[CanDlc::WRITE_IMU_CALIBRATION_2_DLC + i] >> 0) & 0xFF;
    }
    nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    // Insert usleep to avoid hanging when writing to CAN in one go.
    usleep(1000);
    frame.can_id = CanId::WRITE_IMU_CALIBRATION_ID_3;
    frame.can_dlc = CanDlc::WRITE_IMU_CALIBRATION_3_DLC;
    for (int i = 0; i < CanDlc::WRITE_IMU_CALIBRATION_3_DLC; ++i) {
      frame.data[i] = (calibration_params[CanDlc::WRITE_IMU_CALIBRATION_1_DLC + CanDlc::WRITE_IMU_CALIBRATION_2_DLC + i] >> 0) & 0xFF;
    }
    nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
  }

  void capacitiveSensorInputEnable()
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::CAPACITIVE_TOUCH_SENSOR_INPUT_ENABLE_ID;
    frame.can_dlc = CanDlc::CAPACITIVE_TOUCH_SENSOR_INPUT_ENABLE_DLC;
    frame.data[0] = 1;
    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    usleep(1000);
    frame.can_id = CanId::CAPACITIVE_TOUCH_CALIBRATION_ID;
    frame.can_dlc = CanDlc::CAPACITIVE_TOUCH_CALIBRATION_CAN_DLC;
    frame.data[0] = 1;
    nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send write capacitive sensor input enable frame");
    } else {
      RCLCPP_INFO(this->get_logger(), "Write capacitive sensor input enable sent successfully");
    }
  }

  void readImuServiceCalibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::IMU_CALIBRATION_SEND_CAN_ID;
    frame.can_dlc = CanDlc::IMU_CALIBRATION_SEND_CAN_DLC;
    frame.data[0] = 0x00;
    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Error sending read Imu calibration frame");
      response->success = false;
      response->message = "Error sending read Imu calibration frame";
    } else {
      RCLCPP_INFO(this->get_logger(), "Read Imu calibration sent successfully");
      response->success = true;
      response->message = "Read Imu calibration sent successfully";
    }
  }

  void capacitiveServiceCalibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::CAPACITIVE_TOUCH_CALIBRATION_ID;
    frame.can_dlc = CanDlc::CAPACITIVE_TOUCH_CALIBRATION_CAN_DLC;
    frame.data[0] = 1;
    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Error sending capacitive calibration frame");
      response->success = false;
      response->message = "Error sending capacitive calibration frame";
    } else {
      RCLCPP_INFO(this->get_logger(), "Capacitive calibration sent successfully");
      response->success = true;
      response->message = "Capacitive calibration sent successfully";
    }
  }

  void nullifyCapacitiveNoise(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::CAPACITIVE_TOUCH_NULLIFY_NOISE_ID;
    frame.can_dlc = CanDlc::CAPACITIVE_TOUCH_NULLIFY_NOISE_DLC;
    frame.data[0] = 10;
    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Error sending nullify capacitive noise frame");
      response->success = false;
      response->message = "Error sending nullify capacitive noise frame";
    } else {
      RCLCPP_INFO(this->get_logger(), "nullify capacitive noise sent successfully");
      response->success = true;
      response->message = "nullify capacitive noise sent successfully";
    }
  }

  void timerPubCallback()
  {
    struct can_frame frame;
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes > 0) {
      switch (frame.can_id) {
        case CanId::BME_CAN_ID:
          publishBmeData(frame);
          break;
        case CanId::TACT_CAN_ID:
          publishTactData(frame);
          break;
        case CanId::TOUCH_CAN_ID:
          publishTouchData(frame);
          break;
        case CanId::SERVO_POS_CAN_ID:
          publishServoPosData(frame);
          break;
        case CanId::CAPACITIVE_TOUCH_STATUS_ID:
          publishCapacitiveTouchStatus(frame);
          break;
        default:
          if (frame.can_id >= CanId::TEMPERATURE_1_CAN_ID && frame.can_id <= CanId::TEMPERATURE_5_CAN_ID) {
            publishTemperatureData(frame);
          } else if (frame.can_id >= CanId::WIFI_CAN_ID_1 && frame.can_id <= CanId::WIFI_CAN_ID_5) {
            publishWifiData(frame);
          } else if (frame.can_id >= CanId::READ_IMU_CALIBRATION_ID_1 && frame.can_id <= CanId::READ_IMU_CALIBRATION_ID_3) {
            readImuCalibration(frame);
          } else if (frame.can_id >= CanId::IMU_LINEAR_CAN_ID && frame.can_id <= CanId::IMU_ORIENTATION_CAN_ID) {
            publishImuData(frame);
          }
          break;
      }
    }
  }

  void readImuCalibration(const struct can_frame & frame)
  {
    static std::vector<int32_t> calibration_data((CanDlc::WRITE_IMU_CALIBRATION_1_DLC + CanDlc::WRITE_IMU_CALIBRATION_2_DLC + CanDlc::WRITE_IMU_CALIBRATION_3_DLC), 0);
    static bool is_read_calibration_1 = false;
    static bool is_read_calibration_2 = false;
    static bool is_read_calibration_3 = false;
    if (frame.can_id == CanId::READ_IMU_CALIBRATION_ID_1 && frame.can_dlc == CanDlc::READ_IMU_CALIBRATION_1_DLC) {
      for (int i = 0; i < CanDlc::READ_IMU_CALIBRATION_1_DLC; i++) {
        calibration_data[i] = static_cast<int32_t>(std::round(frame.data[i]));
      }
      is_read_calibration_1 = true;
    } else if (frame.can_id == CanId::READ_IMU_CALIBRATION_ID_2 && frame.can_dlc == CanDlc::READ_IMU_CALIBRATION_2_DLC) {
      for (int i = 0; i < CanDlc::READ_IMU_CALIBRATION_2_DLC; i++) {
        calibration_data[i + CanDlc::READ_IMU_CALIBRATION_2_DLC] = static_cast<int32_t>(std::round(frame.data[i]));
      }
      is_read_calibration_2 = true;
    } else if (frame.can_id == CanId::READ_IMU_CALIBRATION_ID_3 && frame.can_dlc == CanDlc::READ_IMU_CALIBRATION_3_DLC) {
      for (int i = 0; i < CanDlc::READ_IMU_CALIBRATION_3_DLC; i++) {
        calibration_data[i + CanDlc::READ_IMU_CALIBRATION_1_DLC + CanDlc::READ_IMU_CALIBRATION_2_DLC] = static_cast<int32_t>(std::round(frame.data[i]));
      }
      is_read_calibration_3 = true;
    }
    if (is_read_calibration_1 && is_read_calibration_2 && is_read_calibration_3) {
      std_msgs::msg::Int32MultiArray calibration_msg;
      calibration_msg.data = calibration_data;
      calibration_pub_->publish(calibration_msg);
      is_read_calibration_1 = false;
      is_read_calibration_2 = false;
      is_read_calibration_3 = false;
    }
  }

  void publishImuData(const struct can_frame & frame)
  {
    static bool is_linear = false;
    static bool is_angular = false;
    if (frame.can_id == CanId::IMU_LINEAR_CAN_ID && frame.can_dlc == CanDlc::IMU_LINEAR_CAN_DLC) {
      int16_t linear_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
      int16_t linear_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
      int16_t linear_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
      imu_msg.linear_acceleration.x = linear_x / 100.0;
      imu_msg.linear_acceleration.y = linear_y / 100.0;
      imu_msg.linear_acceleration.z = linear_z / 100.0;
      is_linear = true;
    }
    if (frame.can_id == CanId::IMU_ANGULAR_CAN_ID && frame.can_dlc == CanDlc::IMU_ANGULAR_CAN_DLC && is_linear) {
      int16_t angular_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
      int16_t angular_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
      int16_t angular_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
      imu_msg.angular_velocity.x = (angular_x / 16.0) * (M_PI / 180.0);
      imu_msg.angular_velocity.y = (angular_y / 16.0) * (M_PI / 180.0);
      imu_msg.angular_velocity.z = (angular_z / 16.0) * (M_PI / 180.0);
      is_angular = true;
    }
    if (frame.can_id == CanId::IMU_ORIENTATION_CAN_ID && frame.can_dlc == CanDlc::IMU_ORIENTATION_CAN_DLC && is_linear && is_angular) {
      int16_t orientation_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
      int16_t orientation_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
      int16_t orientation_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
      int16_t orientation_w = (((uint16_t)frame.data[7]) << 8) | ((uint16_t)frame.data[6]);
      imu_msg.orientation.x = orientation_x * (1.0 / (1 << 14));
      imu_msg.orientation.y = orientation_y * (1.0 / (1 << 14));
      imu_msg.orientation.z = orientation_z * (1.0 / (1 << 14));
      imu_msg.orientation.w = orientation_w * (1.0 / (1 << 14));
      imu_msg.header.stamp = this->get_clock()->now();
      std::string frame_id = this->get_parameter("imu_frame_id").as_string();
      imu_msg.header.frame_id = frame_id;
      imu_pub_->publish(imu_msg);
    }
  }

  void publishWifiData(const struct can_frame & frame)
  {
    static std::string ssid;
    static int8_t frame_dlc = 0;
    std::array<uint8_t, 6> mac_address{};
    int8_t channel = 0;
    int8_t rssi = 0;
    if (frame.can_id == CanId::WIFI_CAN_ID_1) {
      ssid.clear();
      frame_dlc = 0;
    }
    if (frame.can_id == CanId::WIFI_CAN_ID_5) {
      mac_address.fill(0);
      channel = 0;
      rssi = 0;
      for (int i = 0; i < CanDlc::WIFI_BSSID_CAN_DLC; ++i) {
        mac_address[i] = frame.data[i];
        frame_dlc++;
      }
      channel = frame.data[6];
      frame_dlc++;
      rssi = frame.data[7];
      frame_dlc++;
    }
    if (frame.can_id >= CanId::WIFI_CAN_ID_1 && frame.can_id <= CanId::WIFI_CAN_ID_4) {
      for (int i = 0; i < frame.can_dlc; ++i) {
        frame_dlc++;
        if (frame.data[i] != '\0') {
          ssid += static_cast<char>(frame.data[i]);
        }
      }
    }
    if (frame_dlc == CanDlc::WIFI_CAN_1_DLC + CanDlc::WIFI_CAN_2_DLC + CanDlc::WIFI_CAN_3_DLC + CanDlc::WIFI_CAN_4_DLC + CanDlc::WIFI_CAN_5_DLC) {
      std::string mac_str;
      for (size_t i = 0; i < mac_address.size(); ++i) {
        if (i != 0) {
          mac_str += ":";
        }
        std::stringstream hex_stream;
        hex_stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac_address[i]);
        mac_str += hex_stream.str();
      }
      std::transform(mac_str.begin(), mac_str.end(), mac_str.begin(), ::toupper);
      rclcpp::Time current_time = rclcpp::Clock().now();
      int seconds = current_time.seconds();
      int64_t nanoseconds_int = current_time.nanoseconds();
      std::string nanoseconds_str = std::to_string(nanoseconds_int);
      std::string nanoseconds = nanoseconds_str.substr(nanoseconds_str.length() - 9);
      std::string wifi_data = mac_str + "," +
        ssid + "," +
        std::to_string(channel) + "," +
        std::to_string(rssi) + "," +
        std::to_string(seconds) + "," +
        nanoseconds;
      std_msgs::msg::String msg;
      msg.data = wifi_data;
      wifi_pub_->publish(msg);
      mac_address.fill(0);
      ssid.clear();
      channel = 0;
      rssi = 0;
      frame_dlc = 0;
    }
  }

  void publishBmeData(const struct can_frame & frame)
  {
    if (frame.can_id == CanId::BME_CAN_ID && frame.can_dlc == CanDlc::BME_CAN_DLC) {
      int16_t temperature_raw = ((uint16_t)(frame.data[1] << 8) | ((uint16_t)frame.data[0]));
      float temperature = temperature_raw / 100.0;
      sensor_msgs::msg::Temperature temp_msg;
      temp_msg.header.stamp = this->get_clock()->now();
      temp_msg.temperature = temperature;
      bme_temperature_pub_->publish(temp_msg);
      int32_t pressure_raw = ((uint32_t)frame.data[7] << 24) |
        ((uint32_t)frame.data[6] << 16) |
        ((uint32_t)frame.data[5] << 8) |
        ((uint32_t)frame.data[4] << 0);
      float pressure = pressure_raw / 100.0;
      sensor_msgs::msg::FluidPressure pressure_msg;
      pressure_msg.header.stamp = this->get_clock()->now();
      pressure_msg.fluid_pressure = pressure;
      bme_pressure_pub_->publish(pressure_msg);
      pressure_pub_->publish(pressure_msg);
    }
  }

  void publishTouchData(const struct can_frame & frame)
  {
    if (frame.can_id == CanId::TOUCH_CAN_ID && frame.can_dlc == CanDlc::TOUCH_CAN_DLC) {
      int16_t capacitive_touch = frame.data[3];
      std_msgs::msg::Int16 capacitive_touch_msg;
      capacitive_touch_msg.data = capacitive_touch;
      capacitive_touch_pub_->publish(capacitive_touch_msg);
      int16_t capacitive_touch_raw = frame.data[2];
      std_msgs::msg::Int16 capacitive_touch_raw_msg;
      capacitive_touch_raw_msg.data = capacitive_touch_raw;
      capacitive_touch_raw_pub_->publish(capacitive_touch_raw_msg);
      int16_t tof_touch_raw = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
      std_msgs::msg::Int16 tof_raw_msg;
      tof_raw_msg.data = tof_touch_raw;
      tof_touch_raw_pub_->publish(tof_raw_msg);
      int16_t tof_touch = 0;
      int16_t touch = frame.data[3];
      if (touch_mode_ == "dual") {
        if (touch == 0) {
          if (tof_touch_raw <= tof_touch_threshold_) {
            touch = 1;
          } else {
            touch = 0;
          }
        }
      } else if (touch_mode_ == "cap") {
        // noop
      } else if (touch_mode_ == "tof") {
        if (tof_touch_raw <= tof_touch_threshold_) {
          touch = 1;
        } else {
          touch = 0;
        }
      }
      if (tof_touch_raw >= 16 && tof_touch_raw <= 25) {
        tof_touch = 1;
      } else {
        tof_touch = 0;
      }
      std_msgs::msg::Int16 tof_touch_msg;
      tof_touch_msg.data = tof_touch;
      tof_touch_pub_->publish(tof_touch_msg);
      std_msgs::msg::Int16 touch_msg;
      touch_msg.data = touch;
      touch_pub_->publish(touch_msg);
    }
  }

  void publishTactData(const struct can_frame & frame)
  {
    if (frame.can_id == CanId::TACT_CAN_ID && frame.can_dlc == CanDlc::TACT_CAN_DLC) {
      std_msgs::msg::Int8 tact_msg;
      uint8_t tact_data = 0;
      if (frame.data[0] == 1) {
        tact_data = 8;
      }
      if (frame.data[0] == 2) {
        tact_data = 4;
      }
      if (frame.data[0] == 4) {
        tact_data = 1;
      }
      if (frame.data[0] == 8) {
        tact_data = 2;
      }
      tact_msg.data = tact_data;
      tact_pub_->publish(tact_msg);
    }
  }

  void publishServoPosData(const struct can_frame & frame)
  {
    if (frame.can_id == CanId::SERVO_POS_CAN_ID && frame.can_dlc == CanDlc::SERVO_POS_CAN_DLC) {
      int16_t servo_pos_raw = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
      // The servo target angle (servo_target_deg) is determined by multiplying 2048
      // by the servo angle (ranging from -179 to +179, 0 ~ 4096)
      float servo_pos = std::round(((servo_pos_raw - 2048) / 2048.0) * 180 * -1);
      std_msgs::msg::Int16 servo_pos_pub_msg;
      servo_pos_pub_msg.data = static_cast<int16_t>(servo_pos);
      servo_pos_pub_->publish(servo_pos_pub_msg);
    }
  }

  void publishTemperatureData(const struct can_frame & frame)
  {
    int16_t temperature_raw = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
    float temperature = temperature_raw * 0.0625;
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = this->get_clock()->now();
    msg.temperature = temperature;
    switch (frame.can_id) {
      case CanId::TEMPERATURE_1_CAN_ID:
        temperature_1_pub_->publish(msg);
        break;
      case CanId::TEMPERATURE_2_CAN_ID:
        temperature_2_pub_->publish(msg);
        break;
      case CanId::TEMPERATURE_3_CAN_ID:
        temperature_3_pub_->publish(msg);
        break;
      case CanId::TEMPERATURE_4_CAN_ID:
        temperature_4_pub_->publish(msg);
        break;
      case CanId::TEMPERATURE_5_CAN_ID:
        temperature_5_pub_->publish(msg);
        break;
    }
  }

  void publishCapacitiveTouchStatus(const struct can_frame & frame)
  {
    if (frame.can_id == CanId::CAPACITIVE_TOUCH_STATUS_ID && frame.can_dlc == CanDlc::CAPACITIVE_TOUCH_STATUS_CAN_DLC) {
      int ACAL_FAIL = 0;
      int BC_OUT = 0;
      int NOISE_FLAG = 0;
      int CALIBRATION = 0;
      if ((frame.data[0] & 32) == 32) {
        ACAL_FAIL = 32;
      }
      if ((frame.data[0] & 64) == 64) {
        BC_OUT = 64;
      }
      if ((frame.data[1] & 1) == 1) {
        NOISE_FLAG = 1;
      }
      if ((frame.data[2] & 1) == 1) {
        CALIBRATION = 1;
      }
      std::string status_string = std::string("CAP1203") + "," +
                                  std::to_string(ACAL_FAIL) + "," +
                                  std::to_string(BC_OUT) + "," +
                                  std::to_string(NOISE_FLAG) + "," +
                                  std::to_string(CALIBRATION);
      RCLCPP_INFO(this->get_logger(), "%s", status_string.c_str());
    }
  }


  void subscribeVibratorData(const std_msgs::msg::UInt8::SharedPtr msg, int vibrator_id)
  {
    uint8_t vibrator1 = 0;
    uint8_t vibrator3 = 0;
    uint8_t vibrator4 = 0;
    if (vibrator_id == 1) {
      vibrator1 = msg->data;
    } else if (vibrator_id == 3) {
      vibrator3 = msg->data;
    } else if (vibrator_id == 4) {
      vibrator4 = msg->data;
    }
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::VIBRATOR_CAN_ID;
    frame.can_dlc = CanDlc::VIBRATOR_CAN_DLC;
    frame.data[0] = vibrator1;
    frame.data[1] = vibrator3;
    frame.data[2] = vibrator4;
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Error sending Vibrator frame");
    }
  }

  void subscribeServoTargetData(const std_msgs::msg::Int16 & msg)
  {
    int16_t servo_target_per = -1 * msg.data;
    // The servo target angle (servo_target_deg) is determined by multiplying 2048 by the servo angle (ranging from -179 to +179, 0 ~ 4096)
    float servo_targer_deg = std::round((servo_target_per / 180.0) * 2048 + 2048);
    int16_t servo_target = static_cast<int16_t>(servo_targer_deg);
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::SERVO_TARGET_CAN_ID;
    frame.can_dlc = CanDlc::SERVO_TARGET_CAN_DLC;
    frame.data[0] = servo_target & 0xFF;
    frame.data[1] = (servo_target >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Error sending servo target frame");
    }
  }

  void subServoFree(const std_msgs::msg::Bool::SharedPtr msg)
  {
    uint8_t servo_free = msg->data ? 0x00 : 0x01;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::SERVO_FREE_CAN_ID;
    frame.can_dlc = CanDlc::SERVO_FREE_CAN_DLC;
    frame.data[0] = servo_free;
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Error sending servo free frame");
    }
  }

  void subscribeCapacitiveRecalibrationData(const std_msgs::msg::UInt8 & msg)
  {
    uint8_t recalibration_data = msg.data;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::CAPACITIVE_TOUCH_RECALIBRATION_ID;
    frame.can_dlc = CanDlc::CAPACITIVE_TOUCH_RECALIBRATION_CAN_DLC;
    frame.data[0] = recalibration_data;
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Error sending capacitive recalibration frame");
    }
  }

  void subscribeCapacitiveBcOutRecalibration(const std_msgs::msg::UInt8 & msg)
  {
    uint8_t configuration2 = msg.data;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = CanId::CAPACITIVE_TOUCH_BC_OUT_RECALIBRATION;
    frame.can_dlc = CanDlc::CAPACITIVE_TOUCH_BC_OUT_RECALIBRATION_DLC;
    frame.data[0] = configuration2;
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Error sending capacitive bc out recalibration frame");
    }
  }

  sensor_msgs::msg::Imu imu_msg;
  std::string touch_mode_;
  int tof_touch_threshold_;
  int can_socket_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_3_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_4_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_5_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr bme_temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr bme_pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr calibration_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr tact_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr capacitive_touch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr capacitive_touch_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr tof_touch_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr tof_touch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_pos_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_1_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_3_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_4_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_free_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr capacitive_recalibration_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr capacitive_bc_out_recalibration_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibration_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capacitive_calibration_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capacitive_noise_nullify_srv_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  std::vector<double> imu_accel_bias_;
  std::vector<double> imu_gyro_bias_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CabotCanNode>());
  rclcpp::shutdown();
  return 0;
}
