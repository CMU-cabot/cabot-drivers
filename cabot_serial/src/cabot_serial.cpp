/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
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

#include <csignal>
#include <string>
#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include <functional>

#include "cabot_serial/cabot_serial.hpp"
#include "cabot_serial/arduino_serial.hpp"

using namespace std::chrono_literals;

TopicCheckTask::TopicCheckTask(
  diagnostic_updater::Updater & updater,
  std::shared_ptr<CaBotSerialNode> node,
  const std::string & name, double freq)
: diagnostic_updater::HeaderlessTopicDiagnostic(
    name, updater, diagnostic_updater::FrequencyStatusParam(&min, &max, 0.1, 2)),
  node_(node), min(freq), max(freq) {}

void TopicCheckTask::tick()
{
  node_->tick();
  diagnostic_updater::HeaderlessTopicDiagnostic::tick();
}

CheckConnectionTask::CheckConnectionTask(
  std::shared_ptr<CaBotSerialNode> node,
  const std::string & name)
: diagnostic_updater::DiagnosticTask(name), node_(node) {}

void CheckConnectionTask::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (node_->is_client_ready()) {
    if (node_->get_error_msg() == nullptr) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "connecting");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, node_->get_error_msg());
    }
  } else {
    if (node_->is_topic_alive()) {
      RCLCPP_ERROR(node_->get_logger(), "connected but no message coming");
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "connected but no message coming");
      node_->stopped();
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "working");
    }
  }
}

CaBotSerialNode::CaBotSerialNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cabot_serial_node", rclcpp::NodeOptions(options).use_intra_process_comms(false)),
  client_(nullptr),
  port_(nullptr),
  error_msg_(nullptr),
  updater_(this),
  client_logger_(rclcpp::get_logger("arduino-serial")),
  vibrations_{},
  touch_speed_active_mode_(false),
  touch_speed_max_speed_(2.0),
  touch_speed_max_speed_inactive_(0.5)
{
  // use_intra_process_comms is currently not supported
  // publishers, diagnostic tasks, and related services
  // will be initialized when corresponding meesage arrives
  RCLCPP_INFO(get_logger(), "CABOT ROS Serial CPP Node");

  signal(SIGINT, CaBotSerialNode::signalHandler);
  std::string port_name_ = declare_parameter("port", "/dev/ttyCABOT");
  int baud_ = declare_parameter("baud", 115200);  // actually it is not used
  auto run_once = [this, port_name_]() {
      if (client_ == nullptr) {
        return;
      }
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000.0, "run_once");
      try {
        client_->run_once();
      } catch (const std::ios_base::failure & e) {  // IOError
        error_msg_ = e.what();
        RCLCPP_ERROR(get_logger(), error_msg_);
        RCLCPP_ERROR(get_logger(), "try to reconnect usb");
        client_ = nullptr;
        if (run_timer_) {
          run_timer_->cancel();
        }
      } catch (const std::system_error & e) {  // OSError
        error_msg_ = e.what();
        RCLCPP_ERROR(get_logger(), error_msg_);
        std::cerr << e.what() << std::endl;
        client_ = nullptr;
        if (run_timer_) {
          run_timer_->cancel();
        }
      } catch (const std::runtime_error & e) {  // termios.error
        error_msg_ = e.what();
        RCLCPP_ERROR(get_logger(), error_msg_);
        RCLCPP_ERROR(get_logger(), "connection disconnected");
        client_ = nullptr;
        if (run_timer_) {
          run_timer_->cancel();
        }
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "error occurred");
        rclcpp::shutdown();
        std::exit(EXIT_FAILURE);
      }
    };
// polling to check if client (arduino) is disconnected and keep trying to reconnect
  auto polling = [this, port_name_, baud_, run_once]() {
      this->prepare();
      RCLCPP_INFO(get_logger(), "polling");
      if (client_ && client_->is_alive_) {
        return;
      }
      client_ = nullptr;
      port_ = nullptr;
      RCLCPP_INFO(get_logger(), "Connecting to %s at %d baud", port_name_.c_str(), baud_);
      try {
        port_ = std::make_shared<Serial>(port_name_, baud_, 5000, 10000);
      } catch (std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(), e.what());
        return;
      }
      client_ = std::make_shared<CaBotArduinoSerial>(port_, baud_);
      // client_->delegate_ = this;
      client_->delegate_ = std::dynamic_pointer_cast<CaBotArduinoSerialDelegate>(shared_from_this());
      updater_.setHardwareID(port_name_);
      last_topic_alive_time_ = std::chrono::time_point<std::chrono::system_clock>();
      client_->start();
      RCLCPP_INFO(get_logger(), "Serial is ready");
      run_timer_ = create_wall_timer(0.001s, run_once);
    };

  polling_timer_ = create_wall_timer(1s, polling);
}

void CaBotSerialNode::tick()
{
  last_topic_alive_time_ = std::chrono::system_clock::now();
}

bool CaBotSerialNode::is_topic_alive()
{
  return last_topic_alive_time_.time_since_epoch().count() > 0 &&
         (std::chrono::system_clock::now() - last_topic_alive_time_) > 1s;
}

bool CaBotSerialNode::is_client_ready()
{
  return client_ != nullptr;
}

const char * CaBotSerialNode::get_error_msg()
{
  return error_msg_;
}

void CaBotSerialNode::prepare()
{
  if (!check_connection_task_) {
    check_connection_task_ =
      std::make_shared<CheckConnectionTask>(shared_from_this(), "Serial Connection");
    updater_.add(*check_connection_task_);
  }
}

// CaBotArduinoSerialDelegate

std::tuple<int, int> CaBotSerialNode::system_time()
{
  rclcpp::Time now = this->get_clock()->now();
  int64_t nnow = now.nanoseconds();
  int sec = static_cast<int>(nnow / 1000000000);
  int nsec = static_cast<int>(nnow % 1000000000);
  // self.client_logger().info("current time={}".format(self.get_clock().now())
  return std::make_pair(sec, nsec);
}

void CaBotSerialNode::stopped()
{
  if (client_ == nullptr) {return;}
  client_->reset_serial();
  client_ = nullptr;
  port_ = nullptr;
  last_topic_alive_time_ = std::chrono::time_point<std::chrono::system_clock>();
  RCLCPP_INFO(get_logger(), "stopped");
}

void CaBotSerialNode::log(rclcpp::Logger::Level level, const std::string & text)
{
  if (level == rclcpp::Logger::Level::Info) {
    RCLCPP_INFO(client_logger_, text.c_str());
  } else if (level == rclcpp::Logger::Level::Warn) {
    RCLCPP_WARN(client_logger_, text.c_str());
  } else if (level == rclcpp::Logger::Level::Error) {
    RCLCPP_ERROR(client_logger_, text.c_str());
  } else if (level == rclcpp::Logger::Level::Debug) {
    RCLCPP_DEBUG(client_logger_, text.c_str());
  }
}

void CaBotSerialNode::log_throttle(
  rclcpp::Logger::Level level, int interval_in_ms, const std::string & text)
{
  if (level == rclcpp::Logger::Level::Info) {
    RCLCPP_INFO_THROTTLE(client_logger_, *this->get_clock(), interval_in_ms, text.c_str());
  } else if (level == rclcpp::Logger::Level::Warn) {
    RCLCPP_WARN_THROTTLE(client_logger_, *this->get_clock(), interval_in_ms, text.c_str());
  } else if (level == rclcpp::Logger::Level::Error) {
    RCLCPP_ERROR_THROTTLE(client_logger_, *this->get_clock(), interval_in_ms, text.c_str());
  } else if (level == rclcpp::Logger::Level::Debug) {
    RCLCPP_DEBUG_THROTTLE(client_logger_, *this->get_clock(), interval_in_ms, text.c_str());
  }
}

void CaBotSerialNode::get_param(
  const std::string & name,
  std::function<void(const std::vector<int> &)> callback)
{
  rcl_interfaces::msg::ParameterDescriptor pd;
  std::vector<int> val;
  try {
    rclcpp::ParameterValue default_value;
    if (name == "run_imu_calibration") {
      pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      default_value = rclcpp::ParameterValue(false);
    } else if (name == "calibration_params") {
      pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
      default_value = rclcpp::ParameterValue(std::vector<int>());
    } else if (name == "touch_params") {
      pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
      default_value = rclcpp::ParameterValue(std::vector<int>());
    } else {
      RCLCPP_INFO(get_logger(), "parameter %s is not defined", name.c_str());
      callback(val);
      return;
    }
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, rclcpp::ParameterValue(default_value), pd);
    }
    rclcpp::Parameter param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      val.push_back(param.as_bool() ? 1 : 0);
    } else {
      // const std::vector<long int> temp = param.as_integer_array();
      auto temp = param.as_integer_array();
      val.assign(temp.begin(), temp.end());
    }
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
    RCLCPP_ERROR(get_logger(), "parameter %s is not defined", name.c_str());
  } catch (const rclcpp::ParameterTypeException &) {
    RCLCPP_ERROR(get_logger(), "invalid parameter type %s", name.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "exception while get parameter %s: %s", name.c_str(), ex.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "unknown error occurred while get parameter %s", name.c_str());
  }
  RCLCPP_INFO(
    get_logger(), "get_param %s=%s", name.c_str(), val.size() > 0 ? std::to_string(
      val[0]).c_str() : "[]");
  callback(val);
}

void CaBotSerialNode::publish(uint8_t cmd, const std::vector<uint8_t> & data)
{
  if (cmd == 0x10) {  // touch
    if (touch_pub_ == nullptr) {
      touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch", rclcpp::QoS(10));
      touch_check_task_ = std::make_shared<TopicCheckTask>(updater_, shared_from_this(), "Touch sensor", 50);

      /* touch speed control
       * touch speed activw mode
       * True:  Touch - go,    Not Touch - no go
       * False: Touch - no go, Not Touch - go
       */
      touch_speed_active_mode_ = true;
      touch_speed_max_speed_ = this->declare_parameter("touch_speed_max_speed", touch_speed_max_speed_);
      touch_speed_max_speed_inactive_ =
        this->declare_parameter("touch_speed_max_speed_inactive", touch_speed_max_speed_inactive_);
      rclcpp::QoS transient_local_qos(1);
      transient_local_qos.transient_local();
      touch_speed_switched_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "touch_speed_switched", transient_local_qos);
      set_touch_speed_active_mode_srv = this->create_service<std_srvs::srv::SetBool>(
        "set_touch_speed_active_mode", std::bind(
          &CaBotSerialNode::set_touch_speed_active_mode, this, std::placeholders::_1,
          std::placeholders::_2));
    }
    std::unique_ptr<std_msgs::msg::Int16> msg = std::make_unique<std_msgs::msg::Int16>();
    msg->data = static_cast<int16_t>((data[1] << 8) | data[0]);
    touch_callback(*msg);
    touch_pub_->publish(std::move(msg));
    touch_check_task_->tick();
  }
  if (cmd == 0x11) {  // touch_raw
    if (touch_raw_pub_ == nullptr) {
      touch_raw_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch_raw", rclcpp::QoS(10));
    }
    std::unique_ptr<std_msgs::msg::Int16> msg = std::make_unique<std_msgs::msg::Int16>();
    msg->data = static_cast<int16_t>((data[1] << 8) | data[0]);
    touch_raw_pub_->publish(std::move(msg));
  }
  if (cmd == 0x12) {  // buttons
    if (button_pub_ == nullptr) {
      button_pub_ = this->create_publisher<std_msgs::msg::Int8>("pushed", rclcpp::QoS(10));
      button_check_task_ = std::make_shared<TopicCheckTask>(updater_, shared_from_this(), "Push Button", 50);

      // assumes vibrators can be used with buttons
      vib1_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "vibrator1", 10, [this](std_msgs::msg::UInt8::UniquePtr msg)
        {this->vib_callback(0x20, std::move(msg));});
      vib2_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "vibrator2", 10, [this](std_msgs::msg::UInt8::UniquePtr msg)
        {this->vib_callback(0x21, std::move(msg));});
      vib3_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "vibrator3", 10, [this](std_msgs::msg::UInt8::UniquePtr msg)
        {this->vib_callback(0x22, std::move(msg));});
      vib4_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "vibrator4", 10, [this](std_msgs::msg::UInt8::UniquePtr msg)
        {this->vib_callback(0x23, std::move(msg));});
      vib_timer_ = this->create_wall_timer(0.01s, std::bind(&CaBotSerialNode::vib_loop, this));
    }
    std::unique_ptr<std_msgs::msg::Int8> msg = std::make_unique<std_msgs::msg::Int8>();
    msg->data = static_cast<int8_t>(data[0]);
    button_pub_->publish(std::move(msg));
    button_check_task_->tick();
  }
  if (cmd == 0x13) {  // imu
    std::shared_ptr<sensor_msgs::msg::Imu> msg = process_imu_data(data);
    if (msg) {
      if (imu_pub_ == nullptr) {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10));
        imu_check_task_ = std::make_shared<TopicCheckTask>(updater_, shared_from_this(), "IMU", 100);
      }
      imu_pub_->publish(*msg);
      imu_check_task_->tick();
    }
  }
  if (cmd == 0x14) {  // calibration
    if (calibration_pub_ == nullptr) {
      calibration_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "calibration", rclcpp::QoS(10));
    }
    std::unique_ptr<std_msgs::msg::UInt8MultiArray> msg =
      std::make_unique<std_msgs::msg::UInt8MultiArray>();
    msg->data = data;
    calibration_pub_->publish(std::move(msg));
  }
  if (cmd == 0x15) {  // pressure
    if (pressure_pub_ == nullptr) {
      pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "pressure", rclcpp::QoS(10));
      pressure_check_task_ = std::make_shared<TopicCheckTask>(updater_, shared_from_this(), "Pressure", 2);
    }
    sensor_msgs::msg::FluidPressure msg;
    float pressure;
    std::memcpy(&pressure, data.data(), sizeof(float));
    msg.fluid_pressure = pressure;
    msg.variance = 0.0;
    msg.header.stamp = this->now();
    msg.header.frame_id = "bmp_frame";
    pressure_pub_->publish(std::move(msg));
    pressure_check_task_->tick();
  }
  if (cmd == 0x16) {  // temperature
    if (temperature_pub_ == nullptr) {
      temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
        "temparature", rclcpp::QoS(10));
      temp_check_task_ = std::make_shared<TopicCheckTask>(updater_, shared_from_this(), "Temperature", 2);
    }
    sensor_msgs::msg::Temperature msg;
    float temperature;
    std::memcpy(&temperature, data.data(), sizeof(float));
    msg.temperature = temperature;
    msg.variance = 0.0;
    msg.header.stamp = this->now();
    msg.header.frame_id = "bmp_frame";
    temperature_pub_->publish(std::move(msg));
    temp_check_task_->tick();
  }
  if (0x20 <= cmd && cmd <= 0x23) {  // vibrator feedback
    vibrations_[cmd - 0x20].current = data[0];
  }
  if (cmd == 0x17) {  // wifi
    if (wifi_pub_ == nullptr) {
      wifi_pub_ = this->create_publisher<std_msgs::msg::String>("wifi", rclcpp::QoS(10));
    }
    std::unique_ptr<std_msgs::msg::String> msg = std::make_unique<std_msgs::msg::String>();
    msg->data = std::string(data.begin(), data.end());
    wifi_pub_->publish(std::move(msg));
  }
}

// Private methods

void CaBotSerialNode::signalHandler(int signal)
{
  (void)signal;  // avoid unused variable warning
  exit(0);
}

void CaBotSerialNode::vib_loop()
{
  if (client_ == nullptr) {
    return;
  }
  for (int i = 0; i < 4; i++) {
    // resend vibration command if the value of vibrator is not updated
    if (vibrations_[i].target > 0) {
      if (vibrations_[i].target != vibrations_[i].current) {
        vibrations_[i].count++;
        std::vector<uint8_t> data;
        data.push_back(vibrations_[i].target);
        client_->send_command(0x20 + i, data);
        RCLCPP_INFO(
          get_logger(), "resend vibrator %d value (%d != %d) [count=%d]",
          i, vibrations_[i].target, vibrations_[i].current, vibrations_[i].count);
      } else {
        vibrations_[i].target = vibrations_[i].current = vibrations_[i].count = 0;
      }
    }
  }
}

void CaBotSerialNode::vib_callback(uint8_t cmd, const std_msgs::msg::UInt8::UniquePtr msg)
{
  if (client_ == nullptr) {return;}
  std::vector<uint8_t> data;
  data.push_back(msg->data);
  vibrations_[cmd - 0x20].target = msg->data;
  client_->send_command(cmd, data);
}

void CaBotSerialNode::touch_callback(std_msgs::msg::Int16 & msg)
{
  std::unique_ptr<std_msgs::msg::Float32> touch_speed_msg =
    std::make_unique<std_msgs::msg::Float32>();
  if (touch_speed_active_mode_) {
    touch_speed_msg->data = msg.data ? touch_speed_max_speed_ : 0.0;
  } else {
    touch_speed_msg->data = msg.data ? 0.0 : touch_speed_max_speed_inactive_;
  }
  touch_speed_switched_pub_->publish(std::move(touch_speed_msg));
}

void CaBotSerialNode::set_touch_speed_active_mode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  touch_speed_active_mode_ = req->data;
  if (touch_speed_active_mode_) {
    res->message = "touch speed active mode = True";
  } else {
    res->message = "touch speed active mode = False";
  }
  res->success = true;
}

std::shared_ptr<sensor_msgs::msg::Imu> CaBotSerialNode::process_imu_data(
  const std::vector<uint8_t> & data)
{
  // Discard possible corrupted data
  int count = 0;
  std::vector<int> data1;
  std::vector<float> data2;
  data2.reserve(12);
  for (int i = 0; i < 12; ++i) {
    if (i < 2) {
      int value;
      std::memcpy(&value, &data[i * 4], sizeof(int));
      data1.push_back(value);
    } else {
      float value;
      std::memcpy(&value, &data[i * 4], sizeof(float));
      data2.push_back(value);
    }
  }
  // check IMU linear acc
  for (int i = 7; i < 10; ++i) {
    if (data2[i] == 0) {
      count++;
    }
  }
  if (count == 3) {
    std::string temp = "Linear acc data could be broken, drop a message: ";
    for (int i = 0; i < 10; i++) {
      if (i > 0) {temp += ",";}
      temp += std::to_string(data2[i]);
    }
    RCLCPP_ERROR(get_logger(), temp.c_str());
    return nullptr;
  }
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.orientation_covariance[0] = 0.1;
  imu_msg.orientation_covariance[4] = 0.1;
  imu_msg.orientation_covariance[8] = 0.1;

  // Convert float(32) to int(32)
  imu_msg.header.stamp.sec = data1[0];
  imu_msg.header.stamp.nanosec = data1[1];
  rclcpp::Time imu_time = rclcpp::Time(
    imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec,
    get_clock()->get_clock_type());

  // Check if the difference of time between the current time and the imu stamp is bigger than 1 sec
  rclcpp::Time now = this->get_clock()->now();
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "time diff = %ld",
    std::abs((now - imu_time).nanoseconds()));
  if (std::abs((now - imu_time).nanoseconds()) > 1e9) {
    RCLCPP_ERROR(
      get_logger(), "IMU timestamp jumps more than 1 second, drop a message\n"
      "imu time: %ld > current time: %ld", imu_time.nanoseconds(), now.nanoseconds());
    return nullptr;
  }
  if (imu_last_topic_time != nullptr) {
    if (*imu_last_topic_time > imu_time) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "IMU timestamp is not consistent, drop a message\n"
        "last imu time: %ld > current imu time: %ld",
        imu_last_topic_time->nanoseconds(), imu_time.nanoseconds());
      return nullptr;
    }
  }
  imu_msg.header.frame_id = "imu_frame";
  imu_last_topic_time = std::make_shared<rclcpp::Time>(imu_time);
  imu_msg.orientation.x = data2[0];
  imu_msg.orientation.y = data2[1];
  imu_msg.orientation.z = data2[2];
  imu_msg.orientation.w = data2[3];
  imu_msg.angular_velocity.x = data2[4];
  imu_msg.angular_velocity.y = data2[5];
  imu_msg.angular_velocity.z = data2[6];
  imu_msg.linear_acceleration.x = data2[7];
  imu_msg.linear_acceleration.y = data2[8];
  imu_msg.linear_acceleration.z = data2[9];
  return std::make_shared<sensor_msgs::msg::Imu>(imu_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSerialNode)
