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

#ifndef CABOT__HANDLE_V3_HPP_
#define CABOT__HANDLE_V3_HPP_

#include <time.h>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "button.hpp"
#include "cabot_handle_v3_node.hpp"

class CaBotHandleV3Node;

namespace VibConst
{
  static constexpr uint16_t FREQ = 160;
  namespace Power {
    static constexpr uint8_t DEFAULT = 48;
    static constexpr uint8_t CAUTION = 60;
    static constexpr uint8_t WAITING = 36;
  }
  namespace Duration {
    static constexpr unsigned int TURN = 150;
    static constexpr unsigned int ABOUT_TURN = 400;
    static constexpr unsigned int DEVIATION = 50;
    static constexpr unsigned int SINGLE_VIBRATION = 400;
    static constexpr unsigned int BUTTON_CLICK = 200;
    static constexpr unsigned int BUTTON_HOLDDOWN = 200;
    static constexpr unsigned int CAUTION = 1000;
    static constexpr unsigned int WAITING = 200;
  }
  namespace Sleep
  {
    static constexpr unsigned int DEFAULT = 150;
    static constexpr unsigned int CAUTION = 0;
    static constexpr unsigned int WAITING = 800;
  }
  namespace NumVibrations {
    static constexpr unsigned int TURN = 3;
    static constexpr unsigned int ABOUT_TURN = 2;
    static constexpr unsigned int DEVIATION = 2;
    static constexpr unsigned int CONFIRMATION = 1;
    static constexpr unsigned int BUTTON_CLICK = 1;
    static constexpr unsigned int BUTTON_HOLDDOWN = 1;
    static constexpr unsigned int CAUTION = 1;
    static constexpr unsigned int WAITING = 1;
  }
}

typedef struct vibration
{
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibratorPub;
  unsigned int numberVibrations;
  unsigned int duration;
  unsigned int sleep;
  int i = 0;
} Vibration;

class Handle : public std::enable_shared_from_this<Handle>
{
public:
  enum button_keys
  {
    UNKNOWN = 0, LEFT_TURN = 1, RIGHT_TURN = 2, LEFT_DEV = 3, RIGHT_DEV = 4, FRONT = 5,
    LEFT_ABOUT_TURN = 6, RIGHT_ABOUT_TURN = 7, BUTTON_CLICK = 8, BUTTON_HOLDDOWN = 9,
    CAUTION = 10, NAVIGATION_START = 11, NAVIGATION_ARRIVED = 12, STIMULI_COUNT = 13
  };
  Handle(
    CaBotHandleV3Node * node, std::function<void(const std::map<std::string,
    std::string> &)> eventListener, const std::vector<std::string> & buttonKeys,
    const int & vibratorType);
  void executeStimulus(int index);
  static const std::vector<std::string> stimuli_names;

private:
  enum turn_type_
  {
    NORMAL = 1, AVOIDING = 2
  };
  void timer_callback();
  void vib_waiting_timer_callback();
  void buttonCallback(std_msgs::msg::Int8::UniquePtr & msg);
  void buttonCheck(std_msgs::msg::Int8::UniquePtr & msg, int index);
  void eventCallback(std_msgs::msg::String::UniquePtr msg);
  void cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr & msg);
  void handleImuCallback(sensor_msgs::msg::Imu::UniquePtr & msg);
  void servoPosCallback(std_msgs::msg::Int16::UniquePtr & msg);
  void turnAngleCallback(std_msgs::msg::Float32::UniquePtr & msg);
  void turnTypeCallback(std_msgs::msg::String::UniquePtr & msg);
  void startVibration(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & vibratorPub);
  void stopVibration(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & vibratorPub);
  void changeServoPos(int16_t target_pos);
  void setServoFree(bool is_free);
  void navigationArrived();
  void navigationStart();
  void resetServoPosition();
  void vibrateLeftTurn();
  void vibrateRightTurn();
  void vibrateLeftDeviation();
  void vibrateRightDeviation();
  void vibrateFront();
  void vibrateAboutLeftTurn();
  void vibrateAboutRightTurn();
  void vibrateBack();
  void vibrateButtonClick();
  void vibrateButtonHolddown();
  void vibrateCautionPattern();
  void vibrateWaitingPattern();
  void vibratePattern(
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & vibratorPub,
    unsigned int numberVibrations, unsigned int duration, unsigned int sleep);
  std::shared_ptr<CaBotHandleV3Node> node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator1_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator2_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator3_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator4_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_free_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_target_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr button_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr turn_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr turn_type_sub_;
  rclcpp::Time last_up[9];
  rclcpp::Time last_dwn[9];
  int vibratorType_;
  int up_count[9];
  bool btn_dwn[9];
  bool is_enabled_change_servo_pos_by_yaw_;
  bool is_navigating_;
  bool is_servo_free_;
  bool is_waiting_;
  unsigned char is_waiting_cnt_;
  uint16_t servo_pos_callback_cnt_;
  uint8_t last_turn_type_;
  float current_yaw_degrees_;
  float start_yaw_degrees_;
  float target_yaw_degrees_;
  std::map<std::string, std::string> event;
  std::function<void(const std::map<std::string, std::string> &)> eventListener_;
  std::vector<std::string> buttonKeys_;
  rclcpp::Logger logger_;
  std::vector<std::function<void()>> callbacks_;
  int button[10];
  static const rclcpp::Duration double_click_interval_;
  static const rclcpp::Duration ignore_interval_;
  static const rclcpp::Duration holddown_interval_;
  std::string get_name(int);
  std::vector<Vibration> vibration_queue_;
  rclcpp::TimerBase::SharedPtr vibration_timer_;
  rclcpp::TimerBase::SharedPtr vib_waiting_timer_;
};

#endif  // CABOT__HANDLE_V3_HPP_
