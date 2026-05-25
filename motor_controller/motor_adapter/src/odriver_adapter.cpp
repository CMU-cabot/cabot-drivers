// Copyright (c) 2019, 2022  Carnegie Mellon University
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

/*
 * Odriver motor controller adapter
 *
 * Author: Daisuke Sato <daisukes@cmu.edu>
 */

#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <motor_adapter/odriver_adapter.hpp>
// #include <boost/thread/thread.hpp>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace MotorAdapter
{
const double D2R = M_PI / 180;

ODriverNode::ODriverNode(rclcpp::NodeOptions options)
: rclcpp::Node("odriver_node", options),
  diffDrive_(0),
  cmdVelInput_("/cmd_vel"),
  motorOutput_("/motor"),
  encoderInput_("/encoder"),
  odomOutput_("/odom"),
  pauseControlInput_("/pause_control"),
  PIControl_("/PI_control"),

  lastCmdVelTime_(0, 0, get_clock()->get_clock_type()),
  targetSpdLinear_(0),
  targetSpdTurn_(0),
  currentSpdLinear_(0),
  lastOdomTime_(0, 0, get_clock()->get_clock_type()),

  targetRate_(100),
  maxAcc_(0.5),
  maxDec_(-0.5),

  bias_(0),
  wheel_diameter_(0),
  count_per_rotate_(0),

  measuredSpdLinear_(0),
  measuredSpdTurn_(0),
  integral_linear_(0),
  integral_turn_(0),

  lastImuTime_(0, 0, get_clock()->get_clock_type()),
  lastImuAngularVelocity_(0),
  imuTimeTolerance_(50ms),
  isFreeMode_(false),
  default_motor_control_(true),
  measuredCurrent_(0.0),
  currentLowLidarDist_(0.0),
  currentLidarDist_(0.0)
{
  RCLCPP_INFO(get_logger(), "ODriverNode Constructor");
  PIControl_ = declare_parameter("PI_topic", PIControl_);
  PIPub = create_publisher<odriver_msgs::msg::PIControlData>(PIControl_, 10);

  bias_ = declare_parameter("bias", bias_);
  diffDrive_.set(bias_);

  encoderInput_ = declare_parameter("encoder_topic", encoderInput_);
  encoderSub = create_subscription<odriver_msgs::msg::MotorStatus>(
    encoderInput_, 10, std::bind(&ODriverNode::encoderCallback, this, _1));

  odomOutput_ = declare_parameter("odom_topic", odomOutput_);
  odomPub = create_publisher<nav_msgs::msg::Odometry>(odomOutput_, 10);

  cmdVelInput_ = declare_parameter("cmd_vel_topic", cmdVelInput_);
  cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(cmdVelInput_, 10, std::bind(&ODriverNode::cmdVelCallback, this, _1));

  pauseControlInput_ = declare_parameter("pause_control_topic", pauseControlInput_);
  pauseControlSub = create_subscription<std_msgs::msg::Bool>(
    pauseControlInput_, 10, std::bind(&ODriverNode::pauseControlCallback, this, _1));


  leftVibPub = create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator4", 10);
  rightVibPub = create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator3", 10);


  imuSub = create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(), std::bind(&ODriverNode::imuCallback, this, _1));

// if self.free_mode_detect_lidar_obstacles:
//     self._lidarLimitSub = self._node.create_subscription(std_msgs.msg.Float32, "/cabot/lidar_speed", self._lidar_limit_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())
// if self.free_mode_detect_low_obstacles:
//     self._lowLidarLimitSub = self._node.create_subscription(std_msgs.msg.Float32, "/cabot/low_lidar_speed", self._lidar_limit_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

  lidarCallbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = lidarCallbackGroup;

  lidarLimitSub = create_subscription<std_msgs::msg::Float32>("/cabot/lidar_speed", rclcpp::SensorDataQoS(), std::bind(&ODriverNode::LidarLimitCallback, this, _1), sub_options);
  lowLidarLimitSub = create_subscription<std_msgs::msg::Float32>("/cabot/low_lidar_speed", rclcpp::SensorDataQoS(), std::bind(&ODriverNode::LidarLimitCallback, this, _1), sub_options);

  lidarSub = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&ODriverNode::LidarCallback, this, _1), sub_options);
  lowLidarSub = create_subscription<sensor_msgs::msg::LaserScan>("/livox_scan_expand", rclcpp::SensorDataQoS(), std::bind(&ODriverNode::LowLidarCallback, this, _1), sub_options);


  maxAcc_ = declare_parameter("max_acc", maxAcc_);
  maxDec_ = declare_parameter("max_dec", maxDec_);
  targetRate_ = declare_parameter("target_rate", targetRate_);
  motorOutput_ = declare_parameter("motor_topic", motorOutput_);

  // parameters for linear and angular velocity error feedback
  gain_vel_ = declare_parameter("gain_vel", 0.0);
  gain_omega_ = declare_parameter("gain_omega", 0.0);
  gain_vel_i_ = declare_parameter("gain_vel_i", 0.0);
  gain_omega_i_ = declare_parameter("gain_omega_i", 0.0);
  imuAngularVelocitySign_ = declare_parameter("imu_angular_velocity_sign", 1.0);
  imuAngularVelocityThreshold_ = declare_parameter("imu_angular_velocity_threshold", 0.01);
  feedbackSpdDeadzone_ = declare_parameter("feedback_speed_deadzone", 0.01);
  imuTimeTolerance_ = rclcpp::Duration(std::chrono::duration<double>(declare_parameter("imu_time_tolerance", 0.05)));

  // parameter to set default loop control
  default_motor_control_ = declare_parameter("default_motor_control", default_motor_control_);

  thread = std::make_shared<std::thread>(&ODriverNode::cmdVelLoop, this, targetRate_);

  RCLCPP_INFO(get_logger(), "MotorAdapter ODriverNode - %s", __FUNCTION__);
}

ODriverNode::~ODriverNode()
{
  RCLCPP_INFO(get_logger(), "ODriverNode Destructor");
}

void ODriverNode::cmdVelLoop(int publishRate)
{
  rclcpp::Rate loopRate(publishRate);

  motorPub = create_publisher<odriver_msgs::msg::MotorTarget>(motorOutput_, 10);

  double maxAccStep = maxAcc_ / publishRate;
  double maxDecStep = maxDec_ / publishRate;

  while (rclcpp::ok()) {
    odriver_msgs::msg::MotorTarget target;

    // set loopCtrl
    target.loop_ctrl = true;

    if(isFreeMode_){ //TODO WALK STRAIGHT, TURN ONLY if user moves a lot
      rclcpp::Time now = get_clock()->now();
      double dt = 1.0 / publishRate;
      // double fixedMeasuredSpdTurn = measuredSpdTurn_;
      // if (now - lastImuTime_ < imuTimeTolerance_) {  // assumes imu is received continuously
      //   if (imuAngularVelocityThreshold_ <= fabs(lastImuAngularVelocity_)) {
      //     fixedMeasuredSpdTurn = bias_ / 2.0 * lastImuAngularVelocity_;  // radius * anguler_velocity converts rad/s to m/s dimension.
      //   }
      // }

      // // compute feedback wheel speed
      // double errorSpdLinear = currentSpdLinear_ - measuredSpdLinear_;
      // double errorSpdTurn = targetSpdTurn_ - fixedMeasuredSpdTurn;
      // double feedbackSpdRight = gain_vel_ * errorSpdLinear + gain_omega_ * errorSpdTurn + gain_vel_i_ * integral_linear_ + gain_omega_i_ * integral_turn_;
      // double feedbackSpdLeft = gain_vel_ * errorSpdLinear - gain_omega_ * errorSpdTurn + gain_vel_i_ * integral_linear_ - gain_omega_i_ * integral_turn_;
      // integral_linear_ += errorSpdLinear * dt;
      // integral_turn_ += errorSpdTurn * dt;

      // // ignore small feedback speed to prevent slow rotation
      // if (feedbackSpdDeadzone_ < fabs(feedbackSpdRight) &&
      //   feedbackSpdDeadzone_ < fabs(feedbackSpdLeft))
      // {
      //   target.spd_right += feedbackSpdRight;
      //   target.spd_left += feedbackSpdLeft;
      // }

      // const float transparency = 1.0f; 
      // const float drag = 0.0000f;
      // const float max_speed = 0.2f;


      // float deltaUserSpeed = measuredSpdLinear_ - currentSpdLinear_;
      // currentSpdLinear_ += (deltaUserSpeed * transparency) - dt*(currentSpdLinear_ * drag) - measuredCurrent_;
      // currentSpdLinear_ = std::clamp(static_cast<float>(currentSpdLinear_), -max_speed, max_speed);
      // target.spd_left = currentSpdLinear_;
      // target.spd_right = currentSpdLinear_;

      //float value = 10.0f * std::sin(2.0 * M_PI * (1.0 / 2.0) * now.seconds());

      float value = 0.0f;

      if(measuredSpdLinear_ > 0.5f){
        float delta = measuredSpdLinear_ - 0.5f;
        float deltaSquared = delta * delta;
        value = -70.0f * deltaSquared;
      }

      // if(measuredSpdLinear_ > 0.8f){
      //   value = -10.0f;
      // }

      // if(measuredSpdLinear_ > 1.0f){
      //   value = -50.0f;
      // }

      // if(measuredSpdLinear_ > 1.5f){
      //   value = -200.0f;
      // }

      // if(measuredSpdLinear_ > 2.0f){
      //   value = -1000.0f;
      // }

      target.header.stamp = now;
      target.spd_left = value;
      target.spd_right = value;
      target.torque_ctrl = true;

      if(currentLidarDist_ < 0.3f || currentLowLidarDist_ < 0.3f){
        target.spd_left = -1.0f;
        target.spd_right = -1.0f;
        if(measuredSpdLinear_ > -0.03f){
          target.header.stamp = now;
          target.spd_left = -0.01f;
          target.spd_right = -0.01f;
          target.torque_ctrl = false;
        }
      }
      else if(currentLidarDist_ < 1.0f || currentLowLidarDist_ < 1.0f){
        if(measuredSpdLinear_ > 0.05f){
          float lidarDist = min(currentLidarDist_, currentLowLidarDist_);
          target.spd_left += -50.0f*(measuredSpdLinear_-0.05f)*2.0f*(1.0f-lidarDist);
          target.spd_right += -50.0f*(measuredSpdLinear_-0.05f)*2.0f*(1.0f-lidarDist);
        }
        else{
          target.spd_left = 0.0f;
          target.spd_right = 0.0f;
        }

      }




      motorPub->publish(target);

      /*odriver_msgs::msg::PIControlData control_data_msg;
      control_data_msg.header.stamp = this->get_clock()->now();
      control_data_msg.current_spd_linear = currentSpdLinear_;
      control_data_msg.measured_spd_linear = measuredSpdLinear_;
      control_data_msg.error_spd_linear = errorSpdLinear;
      control_data_msg.integral_linear = integral_linear_;
      control_data_msg.target_spd_turn = targetSpdTurn_;
      control_data_msg.measured_spd_turn = measuredSpdTurn_;
      control_data_msg.error_spd_turn = errorSpdTurn;
      control_data_msg.integral_turn = integral_turn_;
      PIPub->publish(control_data_msg);*/

      //
      // if(isLeftCorridor && leftLidarDist_ > 2.51f){
      //   isLeftCorridor = false;
      //   // small vibration
      //   std_msgs::msg::UInt8 vib_msg;
      //   vib_msg.data = 10;
      //   RCLCPP_INFO(get_logger(), "Left SMALL vibration");
      //   leftVibPub->publish(vib_msg);
      // }
      // else if (!isLeftCorridor && leftLidarDist_ < 2.5f){
      //   isLeftCorridor = true;
      //   // big vibration
      //   std_msgs::msg::UInt8 vib_msg;
      //   vib_msg.data = 50;
      //   RCLCPP_INFO(get_logger(), "Left BIG vibration");
      //   leftVibPub->publish(vib_msg);
      // }

      // if(isRightCorridor && rightLidarDist_ > 2.51f){
      //   isRightCorridor = false;
      //   // small vibration
      //   std_msgs::msg::UInt8 vib_msg;
      //   vib_msg.data = 10;
      //   RCLCPP_INFO(get_logger(), "Right SMALL vibration");
      //   rightVibPub->publish(vib_msg);
      // }
      // else if (!isRightCorridor && rightLidarDist_ < 2.5f){
      //   isRightCorridor = true;
      //   // big vibration
      //   std_msgs::msg::UInt8 vib_msg;
      //   vib_msg.data = 50;
      //   RCLCPP_INFO(get_logger(), "Right BIG vibration");
      //   rightVibPub->publish(vib_msg);
      // }

      // if(isLeftIntersection && !isLeftPreviouslyIntersecting){
      //   // small vibration
      //   std_msgs::msg::UInt8 vib_msg;
      //   vib_msg.data = 10;
      //   RCLCPP_INFO(get_logger(), "Left SMALL vibration");
      //   leftVibPub->publish(vib_msg);
      // }

      // isLeftPreviouslyIntersecting = isLeftIntersection;

      // if(isRightIntersection && !isRightPreviouslyIntersecting){
      //   // small vibration
      //   std_msgs::msg::UInt8 vib_msg;
      //   vib_msg.data = 10;
      //   RCLCPP_INFO(get_logger(), "Right SMALL vibration");
      //   rightVibPub->publish(vib_msg);
      // }
      // isRightPreviouslyIntersecting = isRightIntersection;

      loopRate.sleep();
    }
    else{
      double targetL = targetSpdLinear_;
      double targetT = targetSpdTurn_;

      if (targetL == 0.0) {
        // change linear speed by maximum dec rate, if the control is zero (maybe not specified)
        double lDiff = targetL - currentSpdLinear_;
        if (fabs(lDiff) < fabs(maxDecStep)) {
          currentSpdLinear_ = targetL;
        } else {
          currentSpdLinear_ -= maxDecStep * lDiff / fabs(lDiff);
        }
      } else {
        // change linear speed by maximum acc rate
        double lDiff = targetL - currentSpdLinear_;
        if (fabs(lDiff) < maxAccStep) {
          currentSpdLinear_ = targetL;
        } else {
          currentSpdLinear_ += maxAccStep * lDiff / fabs(lDiff);
        }
      }

      // adjust angular speed
      target.header.stamp = get_clock()->now();
      target.spd_left = currentSpdLinear_ - targetT;
      target.spd_right = currentSpdLinear_ + targetT;
      target.torque_ctrl = false;

      // linear and velocity error feedback
      // apply feedback only when the following conditions are met to prevent integrator error accumulation
      if (lastOdomTime_ > rclcpp::Time(0, 0, get_clock()->get_clock_type()) &&  // after receiving at least one motorStatus message
        get_clock()->now() - lastOdomTime_ < rclcpp::Duration(200ms) &&  // last odometry is almost up-to-date (within 200ms)
        target.loop_ctrl &&  // loop control is on
        !(targetSpdLinear_ == 0.0 && targetSpdTurn_ == 0.0)  // command velocity is non-zero
      )
      {
        rclcpp::Time now = get_clock()->now();
        double dt = 1.0 / publishRate;
        double fixedMeasuredSpdTurn = measuredSpdTurn_;
        if (now - lastImuTime_ < imuTimeTolerance_) {  // assumes imu is received continuously
          if (imuAngularVelocityThreshold_ <= fabs(lastImuAngularVelocity_)) {
            fixedMeasuredSpdTurn = bias_ / 2.0 * lastImuAngularVelocity_;  // radius * anguler_velocity converts rad/s to m/s dimension.
          }
        }

        // compute feedback wheel speed
        double errorSpdLinear = currentSpdLinear_ - measuredSpdLinear_;
        double errorSpdTurn = targetSpdTurn_ - fixedMeasuredSpdTurn;
        double feedbackSpdRight = gain_vel_ * errorSpdLinear + gain_omega_ * errorSpdTurn + gain_vel_i_ * integral_linear_ + gain_omega_i_ * integral_turn_;
        double feedbackSpdLeft = gain_vel_ * errorSpdLinear - gain_omega_ * errorSpdTurn + gain_vel_i_ * integral_linear_ - gain_omega_i_ * integral_turn_;
        integral_linear_ += errorSpdLinear * dt;
        integral_turn_ += errorSpdTurn * dt;

        // ignore small feedback speed to prevent slow rotation
        if (feedbackSpdDeadzone_ < fabs(feedbackSpdRight) &&
          feedbackSpdDeadzone_ < fabs(feedbackSpdLeft))
        {
          target.spd_right += feedbackSpdRight;
          target.spd_left += feedbackSpdLeft;
        }
      } else {
        // reduce current speed to zero at maximum acc rate when feedback is disabled
        double lDiff = 0.0 - currentSpdLinear_;
        if (fabs(lDiff) < fabs(maxDecStep)) {
          currentSpdLinear_ = 0.0;
        } else {
          currentSpdLinear_ -= maxDecStep * lDiff / fabs(lDiff);
        }

        // reset integrator when feedback is disabled
        integral_linear_ = integral_linear_ * 0.9;
        integral_turn_ = integral_turn_ * 0.9;
      }

      motorPub->publish(target);

      double errorSpdLinear = currentSpdLinear_ - measuredSpdLinear_;
      double errorSpdTurn = targetSpdTurn_ - measuredSpdTurn_;

      odriver_msgs::msg::PIControlData control_data_msg;
      control_data_msg.header.stamp = this->get_clock()->now();
      control_data_msg.current_spd_linear = currentSpdLinear_;
      control_data_msg.measured_spd_linear = measuredSpdLinear_;
      control_data_msg.error_spd_linear = errorSpdLinear;
      control_data_msg.integral_linear = integral_linear_;
      control_data_msg.target_spd_turn = targetSpdTurn_;
      control_data_msg.measured_spd_turn = measuredSpdTurn_;
      control_data_msg.error_spd_turn = errorSpdTurn;
      control_data_msg.integral_turn = integral_turn_;
      PIPub->publish(control_data_msg);

      loopRate.sleep();
    }
  }
}

void ODriverNode::LidarLimitCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  currentLidarLimit_ = msg->data;
}

void ODriverNode::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr input)
{
  // Handle lidar data
  //auto it = std::min_element(input->ranges.begin(), input->ranges.end());
  const auto& ranges = input->ranges;
  const int N = ranges.size();

  // Sécurité basique
  if (N == 0) return; 
  if (N == 1) { currentLidarDist_ = ranges[0]; return; }

  const float center = (N - 1) / 2.0f;
  float min_val_weighted = std::numeric_limits<float>::max();

  float left_weighted = 0;
  float left_weight_sum = 0;

  float right_weighted = 0;
  float right_weight_sum = 0;

  for (int i = 0; i < N; ++i) {
      // norm_dist vaut 0 au centre, -1 au tout premier élément, et +1 au dernier
      float norm_dist = (i - center) / center; 
      
      // Parabole stricte : 1.0 + (0)^2 = 1x au centre | 1.0 + (1)^2 = 2x aux bords
      float weight = 1.0f + (norm_dist * norm_dist); 
      
      float left_weight = std::max(0.0f, 1.0f-2.0f*fabs(0.5f+norm_dist));
      float right_weight = std::max(0.0f, 1.0f-2.0f*fabs(0.5f-norm_dist)); // 0 à gauche et au centre, 1x à droite
      
      float weighted_dist = ranges[i] * weight;

      if(ranges[i] > 0.0f && weighted_dist < 10.0f){
        left_weighted += ranges[i] * left_weight;
        left_weight_sum += left_weight;
        right_weighted += ranges[i] * right_weight;
        right_weight_sum += right_weight;
      }

      if (weighted_dist < min_val_weighted) {
          min_val_weighted = weighted_dist;
      }
  }

  currentLidarDist_ = min_val_weighted;


  leftLidarDist_ = (left_weight_sum > 0) ? (left_weighted / left_weight_sum) : std::numeric_limits<float>::max();
  rightLidarDist_ = (right_weight_sum > 0) ? (right_weighted / right_weight_sum) : std::numeric_limits<float>::max();

  float displayedDist = currentLidarDist_;
  float displayedLeftDist = leftLidarDist_;
  float displayedRightDist = rightLidarDist_;

  // // --- Intersection Detection Parameters ---
  // const int WIN_SIZE = 15;                // Reduced window size for sharper edge detection
  // const float STEP_MIN = 1.0f;            // Minimum jump (meters) to be considered an edge
  // const float MIN_CORRIDOR_WIDTH = 0.8f;  // Minimum depth required to validate a walkable corridor
  // const int GAP_CHECK_OFFSET = 15;        // How far into the gap we check for depth validation

  // // Assuming standard LiDAR (Counter-Clockwise): 
  // // Lower indices = Right side, Higher indices = Left side
  // const int SCAN_START = 300;
  // const int SCAN_END = 900;

  // bool leftIntersectionDetected = false;
  // bool rightIntersectionDetected = false;

  // for (int i = SCAN_START + WIN_SIZE; i < SCAN_END - WIN_SIZE; ++i) {
      
  //     float mean_prev = 0.0f; // Average of indices < i (Towards the RIGHT of the robot)
  //     float mean_next = 0.0f; // Average of indices > i (Towards the LEFT of the robot)

  //     // 1. Calculate local averages
  //     for (int j = 1; j <= WIN_SIZE; ++j) {
  //         mean_prev += ranges[i - j];
  //         mean_next += ranges[i + j];
  //     }
  //     mean_prev /= WIN_SIZE;
  //     mean_next /= WIN_SIZE;

  //     // 2. Detect gap on the LEFT side
  //     // If next indices (left) are much further than prev indices (right), the wall ended on the left.
  //     if (mean_next - mean_prev > STEP_MIN) {
          
  //         // Check deeper into the newly found gap (higher index = further left)
  //         int check_idx = i + GAP_CHECK_OFFSET;
          
  //         if (check_idx < N) {
  //             float depth_in_gap = ranges[check_idx];
              
  //             // Validate: Is the gap deep enough to be a corridor?
  //             if (depth_in_gap > (mean_prev + MIN_CORRIDOR_WIDTH)) {
  //                 leftIntersectionDetected = true;
  //                 i += WIN_SIZE * 3; // Debounce to avoid multiple triggers on the same edge
  //                 continue;          // Skip to the next region
  //             }
  //         }
  //     }
      
  //     // 3. Detect gap on the RIGHT side
  //     // If prev indices (right) are much further than next indices (left), the wall ended on the right.
  //     else if (mean_prev - mean_next > STEP_MIN) {
          
  //         // Check deeper into the newly found gap (lower index = further right)
  //         int check_idx = i - GAP_CHECK_OFFSET;
          
  //         if (check_idx >= 0) {
  //             float depth_in_gap = ranges[check_idx];
              
  //             // Validate: Is the gap deep enough to be a corridor?
  //             if (depth_in_gap > (mean_next + MIN_CORRIDOR_WIDTH)) {
  //                 rightIntersectionDetected = true;
  //                 i += WIN_SIZE * 3; // Debounce 
  //                 continue;
  //             }
  //         }
  //     }
  // }
  // isLeftIntersection = leftIntersectionDetected;
  // isRightIntersection = rightIntersectionDetected;

  RCLCPP_INFO(get_logger(), "Min Lidar Range: %f, Left: %f, Right: %f", min_val_weighted, displayedLeftDist, displayedRightDist);
}

void ODriverNode::LowLidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr input)
{
  std::vector<float> ranges;
  ranges.reserve(input->ranges.size());

  for (float r : input->ranges) {
      if (r > input->range_min && r < input->range_max) {
          ranges.push_back(r);
      }
  }

  if (ranges.empty()) return;

  // On définit la limite du 1er quartile (25%)
  size_t q1_end = 9 * ranges.size() / 10;
  if (q1_end == 0) q1_end = 1;

  // On place les Q1 plus petites valeurs au début du vecteur (O(n))
  std::nth_element(ranges.begin(), ranges.begin() + q1_end, ranges.end());

  // Moyenne des points du 1er quartile
  float sum = 0;
  for (size_t i = 0; i < q1_end; ++i) {
      sum += ranges[i];
  }
  
  if (q1_end > 0) {
      currentLowLidarDist_ = currentLowLidarDist_ * 0.9 + ranges[q1_end - 1] * 0.1; 
  } else {
      return;
  }

  float displayedDist = currentLowLidarDist_;

  RCLCPP_INFO(get_logger(), "Min Low Lidar Range: %f", displayedDist);
}

void ODriverNode::encoderCallback(const odriver_msgs::msg::MotorStatus::SharedPtr input)
{
  double time = rclcpp::Time(input->header.stamp).nanoseconds() / 1000000000.0;
  diffDrive_.update(
    input->dist_left,
    input->dist_right,
    time);
  Pose & pose = diffDrive_.pose();

  // update measured velocity
  measuredSpdLinear_ = (input->spd_right + input->spd_left) / 2.0;
  measuredSpdTurn_ = (input->spd_right - input->spd_left) / 2.0;

  measuredCurrent_ = (input->current_measured_left + input->current_measured_right) / 2.0;

  // ROS_INFO("input %d, %d, pose %f, %f", input->dist_left_c, input->dist_right_c, pose.x, pose.y);

  nav_msgs::msg::Odometry odom;

  if (rclcpp::Time(input->header.stamp) - lastOdomTime_ < rclcpp::Duration(7ms)) {
    return;
  }

  lastOdomTime_ = input->header.stamp;
  odom.header.stamp = input->header.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  double linear_covariance = 0.1;
  double angle_covariance = 0.2;

  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.a);
  q.normalize();
  odom.pose.pose.orientation.x = q[0];
  odom.pose.pose.orientation.y = q[1];
  odom.pose.pose.orientation.z = q[2];
  odom.pose.pose.orientation.w = q[3];
  odom.pose.covariance[0] = linear_covariance;
  odom.pose.covariance[7] = linear_covariance;
  odom.pose.covariance[14] = linear_covariance;
  odom.pose.covariance[21] = angle_covariance;
  odom.pose.covariance[28] = angle_covariance;
  odom.pose.covariance[35] = angle_covariance;

  LRdouble & vel = diffDrive_.velocity();
  odom.twist.twist.linear.x = vel.l;
  odom.twist.twist.angular.z = vel.r;
  odom.twist.covariance[0] = linear_covariance;
  odom.twist.covariance[7] = linear_covariance;
  odom.twist.covariance[14] = linear_covariance;
  odom.twist.covariance[21] = angle_covariance;
  odom.twist.covariance[28] = angle_covariance;
  odom.twist.covariance[35] = angle_covariance;
  odomPub->publish(odom);
}

void ODriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input)
{
  rclcpp::Time now = get_clock()->now();
  if (lastCmdVelTime_ > rclcpp::Time(0, 0, get_clock()->get_clock_type()) && now - lastCmdVelTime_ < rclcpp::Duration(200ms)) {
    // return;
  }
  lastCmdVelTime_ = now;
  double l = input->linear.x;
  double w = input->angular.z;

  targetSpdLinear_ = l;
  targetSpdTurn_ = bias_ / 2.0 * w;
}

void ODriverNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr input)
{
  lastImuTime_ = input->header.stamp;
  lastImuAngularVelocity_ = input->angular_velocity.z * imuAngularVelocitySign_;
}

void ODriverNode::pauseControlCallback(const std_msgs::msg::Bool::SharedPtr input)
{
  isFreeMode_ = input->data;
}

}  // namespace MotorAdapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MotorAdapter::ODriverNode);
