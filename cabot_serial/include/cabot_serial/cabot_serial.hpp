/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
 * Copyright (c) 2024  ALPS ALPINE CO., LTD.
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

#ifndef CABOT_SERIAL__CABOT_SERIAL_HPP_
#define CABOT_SERIAL__CABOT_SERIAL_HPP_

#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <tuple>
#include <exception>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>

#include "arduino_serial.hpp"


class CaBotSerialNode;

class TopicCheckTask : public diagnostic_updater::HeaderlessTopicDiagnostic
{
public:
  TopicCheckTask(
    diagnostic_updater::Updater & updater,
    std::shared_ptr<CaBotSerialNode> node,
    const std::string & name, double freq);
  void tick();

private:
  std::shared_ptr<CaBotSerialNode> node_;
  double min;
  double max;
};

class CheckConnectionTask : public diagnostic_updater::DiagnosticTask
{
public:
  CheckConnectionTask(
    std::shared_ptr<CaBotSerialNode> node,
    const std::string & name);
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  std::shared_ptr<CaBotSerialNode> node_;
};

class CheckTouchRawTask : public diagnostic_updater::DiagnosticTask
{
public:
  CheckTouchRawTask(
    rclcpp::Logger logger,
    const std::string & name);
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void set_touch_raw_status(int status, std::string message, rclcpp::Time time);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock ros_clock_;
  int touch_raw_status_;
  std::string touch_raw_diag_message_;
  rclcpp::Time reception_time_;
};

typedef struct Vibration
{
  uint8_t current;
  uint8_t target;
  int count;
} Vibration;

class CaBotSerialNode : public rclcpp::Node, public CaBotArduinoSerialDelegate
{
public:
  explicit CaBotSerialNode(const rclcpp::NodeOptions & options);
  ~CaBotSerialNode() = default;

  void tick();
  bool is_topic_alive();
  bool is_client_ready();
  const char * get_error_msg();
  void prepare();

  // Override and delegate by CaBotArduinoSerialDelegate
  std::tuple<int, int> system_time() override;
  void stopped() override;
  void log(rclcpp::Logger::Level level, const std::string & text) override;
  void log_throttle(
    rclcpp::Logger::Level level, int interval_in_ms, const std::string & text) override;
  void get_param(
    const std::string & name, std::function<void(
      const std::vector<int> &)> callback) override;
  void publish(uint8_t cmd, const std::vector<uint8_t> & data) override;

private:
  static void signalHandler(int signal);
  std::shared_ptr<CaBotSerialNode> shared_from_this()
  {
    return std::static_pointer_cast<CaBotSerialNode>(rclcpp::Node::shared_from_this());
  }
  void vib_loop();
  void vib_callback(const uint8_t cmd, const std_msgs::msg::UInt8::UniquePtr msg);
  void handle_callback(const uint8_t cmd, const std_msgs::msg::UInt8::SharedPtr msg);
  void handle_callback(const uint8_t cmd, const std_msgs::msg::Int16::SharedPtr msg);
  void handle_callback(const uint8_t cmd, const std_msgs::msg::Bool::SharedPtr msg);
  void touch_callback(std_msgs::msg::Int16 & msg);
  void set_touch_speed_active_mode(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  std::shared_ptr<sensor_msgs::msg::Imu> process_imu_data(const std::vector<uint8_t> & data);
  std::shared_ptr<sensor_msgs::msg::Imu> adjust_imu_message(const std::shared_ptr<sensor_msgs::msg::Imu>& msg);

  std::shared_ptr<CaBotArduinoSerial> client_;
  std::shared_ptr<Serial> port_;
  const char * error_msg_;
  diagnostic_updater::Updater updater_;
  rclcpp::Logger client_logger_;
  Vibration vibrations_[4];

  // topic frame
  std::string imu_frame_;
  std::string pressure_frame_;
  // parameters to adjust imu readings without explicit calibration
  bool publish_imu_raw_;
  std::vector<double> imu_accel_bias_;
  std::vector<double> imu_gyro_bias_;

  bool touch_speed_active_mode_;
  double touch_speed_max_speed_;
  double touch_speed_max_speed_inactive_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr touch_speed_switched_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_touch_speed_active_mode_srv;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr button_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr calibration_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib1_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib2_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib3_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib4_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_pos_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_free_sub_;

  rclcpp::TimerBase::SharedPtr vib_timer_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  rclcpp::TimerBase::SharedPtr polling_timer_;

  std::chrono::time_point<std::chrono::system_clock> last_topic_alive_time_{};
  std::shared_ptr<rclcpp::Time> imu_last_topic_time;

  // Diagnostic Updater
  std::shared_ptr<TopicCheckTask> imu_check_task_;
  std::shared_ptr<TopicCheckTask> touch_check_task_;
  std::shared_ptr<TopicCheckTask> button_check_task_;
  std::shared_ptr<TopicCheckTask> pressure_check_task_;
  std::shared_ptr<TopicCheckTask> temp_check_task_;
  std::shared_ptr<CheckConnectionTask> check_connection_task_;
  std::shared_ptr<CheckTouchRawTask> check_touch_raw_task_;
};

#endif  // CABOT_SERIAL__CABOT_SERIAL_HPP_
