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

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <utility>
#include <thread>
#include <chrono>
#include <algorithm>

#include "handle_v3.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> Handle::stimuli_names = 
{"unknown", "left_turn", "right_turn", "left_dev", "right_dev", "front",
  "left_about_turn", "right_about_turn", "button_click", "button_holddown",
  "caution", "navigation;event;navigation_start", "navigation_arrived"};
const rclcpp::Duration Handle::double_click_interval_ = rclcpp::Duration(0, 250000000);
const rclcpp::Duration Handle::ignore_interval_ = rclcpp::Duration(0, 50000000);
const rclcpp::Duration Handle::holddown_interval_ = rclcpp::Duration(3, 0);

std::string Handle::get_name(int stimulus)
{
  return stimuli_names[stimulus];
}

Handle::Handle(
  CaBotHandleV3Node * node,
  std::function<void(const std::map<std::string, std::string> &)> eventListener,
  const std::vector<std::string> & buttonKeys, const int & vibratorType)
: node_(node), eventListener_(std::move(eventListener)), buttonKeys_(buttonKeys),
  vibratorType_(vibratorType), logger_(rclcpp::get_logger("Handle_v3"))
{
  vibrator1_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator1", 100);
  vibrator2_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator2", 100);
  vibrator3_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator3", 100);
  vibrator4_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator4", 100);
  servo_free_pub_ = node_->create_publisher<std_msgs::msg::Bool>("servo_free", rclcpp::QoS(1));
  servo_target_pub_ = node_->create_publisher<std_msgs::msg::Int16>("servo_target", rclcpp::QoS(10));
  button_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    "pushed", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Int8::UniquePtr msg) {
      buttonCallback(msg);
    });
  event_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "event", rclcpp::SensorDataQoS(), [this](std_msgs::msg::String::UniquePtr msg) {
      eventCallback(std::move(msg));
    });
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::Twist::UniquePtr msg) {
      cmdVelCallback(msg);
    });
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::UniquePtr msg) {
      handleImuCallback(msg);
    });
  servo_pos_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
    "servo_pos", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Int16::UniquePtr msg) {
      servoPosCallback(msg);
    });
  turn_angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "turn_angle", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Float32::UniquePtr msg) {
      turnAngleCallback(msg);
    });
  turn_type_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "turn_type", rclcpp::SensorDataQoS(), [this](std_msgs::msg::String::UniquePtr msg) {
      turnTypeCallback(msg);
    });
  for (int i = 0; i < 9; ++i) {
    last_up[i] = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_dwn[i] = rclcpp::Time(0, 0, RCL_ROS_TIME);
    up_count[i] = 0;
    btn_dwn[i] = false;
  }
  is_enabled_change_servo_pos_by_yaw_ = false;
  is_navigating_ = false;
  is_servo_free_ = true;
  is_waiting_ = true;
  is_waiting_cnt_ = 0;
  servo_pos_callback_cnt_ = 0;
  last_turn_type_ = turn_type_::NORMAL;
  current_yaw_degrees_ = 0.0;
  start_yaw_degrees_ = 0.0;
  target_yaw_degrees_ = 0.0;
  callbacks_.resize(stimuli_names.size(), nullptr);
  callbacks_[1] = [this]() {
      vibrateLeftTurn();
    };
  callbacks_[2] = [this]() {
      vibrateRightTurn();
    };
  callbacks_[3] = [this]() {
      vibrateLeftDeviation();
    };
  callbacks_[4] = [this]() {
      vibrateRightDeviation();
    };
  callbacks_[5] = [this]() {
      vibrateFront();
    };
  callbacks_[6] = [this]() {
      vibrateAboutLeftTurn();
    };
  callbacks_[7] = [this]() {
      vibrateAboutRightTurn();
    };
  callbacks_[8] = [this]() {
      vibrateButtonClick();
    };
  callbacks_[9] = [this]() {
      vibrateButtonHolddown();
    };
  callbacks_[10] = [this]() {
      vibrateCautionPattern();
    };
  callbacks_[11] = [this]() {
      navigationStart();
    };
  callbacks_[12] = [this]() {
      navigationArrived();
    };
  vibration_timer_ = node_->create_wall_timer(0.01s, std::bind(&Handle::timer_callback, this));
  vib_waiting_timer_ = node_->create_wall_timer(1.0s, std::bind(&Handle::vib_waiting_timer_callback, this));
}

void Handle::timer_callback()
{
  if (vibration_queue_.size() > 0) {
    Vibration & vibration = vibration_queue_.front();
    RCLCPP_DEBUG(rclcpp::get_logger("handle"), "vibration.i = %d", vibration.i);
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "handle"), "vibration.numberVibration = %d", vibration.numberVibrations);
    if (vibration.numberVibrations == 0) {
      vibration_queue_.erase(vibration_queue_.begin());
      RCLCPP_INFO(rclcpp::get_logger("handle"), "Done");
    } else if (vibration.i == 0 && vibration.numberVibrations > 0) {
      std::unique_ptr<std_msgs::msg::UInt8> msg = std::make_unique<std_msgs::msg::UInt8>();
      msg->data = vibration.duration * 0.1;
      vibration.vibratorPub->publish(std::move(msg));
      RCLCPP_INFO(rclcpp::get_logger("handle"), "publish %d", vibration.duration);
      RCLCPP_INFO(rclcpp::get_logger("handle"), "sleep %d ms", vibration.duration);
      vibration.i++;
    } else if (vibration.i == vibration.duration * 0.1 && vibration.numberVibrations == 1) {
      vibration.numberVibrations = 0;
    } else if (vibration.i < (vibration.duration + vibration.sleep) * 0.1) {
      vibration.i++;
    } else {
      vibration.i = 0;
      vibration.numberVibrations--;
    }
  }
}

void Handle::vib_waiting_timer_callback()
{
  if (is_waiting_) {
    if (is_waiting_cnt_ < 2) {  // 1.0sec(vib_waiting_timer_) * 2 = 2.0sec
      is_waiting_cnt_++;
    } else {
      vibrateWaitingPattern();
    }
  } else {
    if (is_waiting_cnt_ > 0) {
      is_waiting_cnt_ = 0;
      vibration_queue_.clear();
    }
  }
}

void Handle::buttonCallback(std_msgs::msg::Int8::UniquePtr & msg)
{
  for (int index = 1; index <= 5; ++index) {
    if (index >= 1 && index <= static_cast<int>(ButtonType::BUTTON_CENTER)) {
      buttonCheck(msg, index);
    }
  }
}

void Handle::buttonCheck(std_msgs::msg::Int8::UniquePtr & msg, int index)
{
  event.clear();
  int bit = 1 << (index - 1);
  bool btn_push = (msg->data & bit) != 0;
  rclcpp::Time now = node_->get_clock()->now();
  rclcpp::Time zerotime(0, 0, RCL_ROS_TIME);
  if (btn_push && !btn_dwn[index] &&
    !(last_up[index] != zerotime && now - last_up[index] < ignore_interval_))
  {
    event.insert(std::pair("button", std::to_string(button_keys(index))));
    event.insert(std::pair("up", "False"));
    btn_dwn[index] = true;
    last_dwn[index] = now;
  }
  if (!btn_push && btn_dwn[index]) {
    event.insert(std::pair("button", std::to_string(button_keys(index))));
    event.insert(std::pair("up", "True"));
    up_count[index]++;
    last_up[index] = now;
    btn_dwn[index] = false;
  }
  if (last_up[index] != zerotime &&
    !btn_dwn[index] &&
    now - last_up[index] > double_click_interval_)
  {
    if (last_dwn[index] != zerotime) {
      event.insert(std::pair("buttons", std::to_string(button_keys(index))));
      event.insert(std::pair("count", std::to_string(up_count[index])));
    }
    last_up[index] = zerotime;
    up_count[index] = 0;
  }
  if (btn_push && btn_dwn[index] &&
    last_dwn[index] != zerotime &&
    now - last_dwn[index] > holddown_interval_)
  {
    event.insert(std::pair("holddown", std::to_string(button_keys(index))));
    last_dwn[index] = zerotime;
  }
  if (!event.empty()) {
    eventListener_(event);
  }
}

void Handle::eventCallback(std_msgs::msg::String::UniquePtr msg)
{
  const std::string name = msg->data;
  auto it = std::find(stimuli_names.begin(), stimuli_names.end(), name.c_str());
  if (it != stimuli_names.end()) {
    int index = std::distance(stimuli_names.begin(), it);
    executeStimulus(index);
  } else {
    RCLCPP_DEBUG(logger_, "Stimulus '%s' not found.", name.c_str());
  }
}

void Handle::executeStimulus(int index)
{
  RCLCPP_INFO(logger_, "execute_stimulus, %d", index);
  const std::size_t size = stimuli_names.size();
  if (index >= 0 && index < static_cast<int>(size) && callbacks_[index]) {
    callbacks_[index]();
    RCLCPP_INFO(logger_, "executed");
  }
}

void Handle::cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr & msg)
{
  std::vector<double> linear = {msg->linear.x, msg->linear.y, msg->linear.z};
  std::vector<double> angular = {msg->angular.x, msg->angular.y, msg->angular.z};
  if ((linear == std::vector<double> {0.0, 0.0, 0.0}) && (angular == std::vector<double> {0.0, 0.0, 0.0})) {
      is_waiting_ = true;
  } else {
      is_waiting_ = false;
  }
}

void Handle::handleImuCallback(sensor_msgs::msg::Imu::UniquePtr & msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  current_yaw_degrees_ = static_cast<float>(yaw * 180 / M_PI);
}

void Handle::servoPosCallback(std_msgs::msg::Int16::UniquePtr & msg)
{
  if (is_waiting_ && !is_navigating_) {
    if (msg->data == 0 && !is_servo_free_) {
      setServoFree(true);
    }
  }
  if (last_turn_type_ == turn_type_::NORMAL) {
    if (is_enabled_change_servo_pos_by_yaw_) {
      float diff_yaw_degrees = current_yaw_degrees_ - start_yaw_degrees_;
      if (diff_yaw_degrees >= 180.0) {
        diff_yaw_degrees -= 360.0;
      } else if (diff_yaw_degrees < -180.0) {
        diff_yaw_degrees += 360.0;
      }
      int16_t target_servo_pos = static_cast<int16_t>(target_yaw_degrees_ - diff_yaw_degrees);
      int16_t current_servo_pos = -1 * (msg->data);
      RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "target_servo_pos: %d", target_servo_pos);
      RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "current_servo_pos: %d", current_servo_pos);
      if (std::abs(current_servo_pos - target_servo_pos) <= 180) {
        changeServoPos(target_servo_pos);
      } 
      if (std::abs(target_servo_pos) < 10) {
        resetServoPosition();
      }
    }
  } else if (last_turn_type_ == turn_type_::AVOIDING) {
    servo_pos_callback_cnt_++;
    if (servo_pos_callback_cnt_ > 100) {  // 20msec(Servo_pub_rate) * 100 = 2000msec
      servo_pos_callback_cnt_ = 0;
      last_turn_type_ = turn_type_::NORMAL;
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "(Avoiding) Reset: servo_pos");
      resetServoPosition();
    }
  }
}

void Handle::turnAngleCallback(std_msgs::msg::Float32::UniquePtr & msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "turn_angle: %f" , msg->data);

  if (std::abs(msg->data) < 60.0) { // thtMinimumTurn = 60
    if (msg->data > 0) {
      target_yaw_degrees_ = 180 / 7;
    } else {
      target_yaw_degrees_ = -180 / 7;
    }
    changeServoPos(static_cast<int16_t>(target_yaw_degrees_));
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "(Avoiding) target_yaw: %d", static_cast<int16_t>(target_yaw_degrees_));
  } else {
    target_yaw_degrees_ = msg->data;
    start_yaw_degrees_ = current_yaw_degrees_;
    changeServoPos(static_cast<int16_t>(target_yaw_degrees_));
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "target_yaw: %d", static_cast<int16_t>(target_yaw_degrees_));
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "start_yaw: %f", start_yaw_degrees_);
    is_enabled_change_servo_pos_by_yaw_ = true;
  }
}

void Handle::turnTypeCallback(std_msgs::msg::String::UniquePtr & msg)
{
  std::string turn_type = msg->data;
  if (turn_type == "Type.Normal") {
    last_turn_type_ = turn_type_::NORMAL;
    RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "Turn_type: Normal");
  } else if (turn_type == "Type.Avoiding") {
    last_turn_type_ = turn_type_::AVOIDING;
    RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "Turn_type: Avoiding");
  }
}

void Handle::changeServoPos(int16_t target_pos)
{
  std::unique_ptr<std_msgs::msg::Int16> msg = std::make_unique<std_msgs::msg::Int16>();
  msg->data = -1 * target_pos;
  RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "change_pos: %d", msg->data);
  servo_target_pub_->publish(std::move(msg));
}

void Handle::setServoFree(bool is_free)
{
  std::unique_ptr<std_msgs::msg::Bool> msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = is_free;
  is_servo_free_ = is_free;
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "servo_free: %d", msg->data);
  servo_free_pub_->publish(std::move(msg));
}

void Handle::navigationArrived()
{
  is_navigating_ = false;
  resetServoPosition();
}

void Handle::navigationStart()
{
  is_navigating_ = true;
  resetServoPosition();
  setServoFree(false);
}

void Handle::resetServoPosition()
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Reset: servo_pos");
  is_enabled_change_servo_pos_by_yaw_ = false;
  changeServoPos(0);
}

void Handle::vibrateLeftTurn()
{
  vibratePattern(vibrator3_pub_, VibConst::NumVibrations::TURN, VibConst::Duration::TURN, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateRightTurn()
{
  vibratePattern(vibrator4_pub_, VibConst::NumVibrations::TURN, VibConst::Duration::TURN, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateLeftDeviation()
{
  vibratePattern(vibrator3_pub_, VibConst::NumVibrations::DEVIATION, VibConst::Duration::DEVIATION, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateRightDeviation()
{
  vibratePattern(vibrator4_pub_, VibConst::NumVibrations::DEVIATION, VibConst::Duration::DEVIATION, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateFront()
{
  vibratePattern(vibrator1_pub_, VibConst::NumVibrations::CONFIRMATION, VibConst::Duration::SINGLE_VIBRATION, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateAboutLeftTurn()
{
  vibratePattern(vibrator3_pub_, VibConst::NumVibrations::ABOUT_TURN, VibConst::Duration::ABOUT_TURN, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateAboutRightTurn()
{
  vibratePattern(vibrator4_pub_, VibConst::NumVibrations::ABOUT_TURN, VibConst::Duration::ABOUT_TURN, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateBack()
{
  vibratePattern(vibrator2_pub_, VibConst::NumVibrations::CONFIRMATION, VibConst::Duration::SINGLE_VIBRATION, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateButtonClick()
{
  vibratePattern(vibrator1_pub_, VibConst::NumVibrations::BUTTON_CLICK, VibConst::Duration::BUTTON_CLICK, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateButtonHolddown()
{
  vibratePattern(vibrator1_pub_, VibConst::NumVibrations::BUTTON_HOLDDOWN, VibConst::Duration::BUTTON_HOLDDOWN, VibConst::Sleep::DEFAULT);
}

void Handle::vibrateCautionPattern()
{
  vibratePattern(vibrator1_pub_, VibConst::NumVibrations::CAUTION, VibConst::Duration::CAUTION, VibConst::Sleep::CAUTION);
}

void Handle::vibrateWaitingPattern()
{
  Vibration vibration;
  vibration.numberVibrations = VibConst::NumVibrations::WAITING;
  vibration.duration = VibConst::Duration::WAITING;
  vibration.sleep = VibConst::Sleep::WAITING;
  vibration.vibratorPub = vibrator1_pub_;
  vibration_queue_.push_back(vibration);
}

void Handle::vibratePattern(
  const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & vibratorPub,
  unsigned int numberVibrations, unsigned int duration, unsigned int sleep)
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Start vibratePattern.");
  Vibration vibration;
  vibration.numberVibrations = numberVibrations;
  vibration.duration = duration;
  vibration.sleep = sleep;
  vibration.vibratorPub = vibratorPub;
  if (is_waiting_) {
    vibration_queue_.insert(vibration_queue_.begin(), vibration);
  } else {
    vibration_queue_.push_back(vibration);
  }
}
