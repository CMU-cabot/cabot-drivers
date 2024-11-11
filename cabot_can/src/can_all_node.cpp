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
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <bits/stdc++.h>

enum CanId : uint8_t{
	IMU_LINEAR_CAN_ID = 0x09,
	IMU_ANGULAR_CAN_ID,
	IMU_ORIENTATION_CAN_ID,
	WIFI_CAN_ID_START,
	WIFI_SSID_CAN_ID_END = 0x0F,
	WIFI_CAN_ID_END,
	BME_CAN_ID,
	TOUCH_CAN_ID,
	TACT_CAN_ID,
	VIBRATOR_CAN_ID = 0x1B,
	SERVO_TARGET_CAN_ID,
	SERVO_POS_CAN_ID = 0x1F,
	SERVO_SWITCH_CAN_ID,
	TEMPERATURE_1_CAN_ID,
	TEMPERATURE_2_CAN_ID,
	TEMPERATURE_3_CAN_ID,
	TEMPERATURE_4_CAN_ID,
	TEMPERATURE_5_CAN_ID,
	WRITE_IMU_CALIBRATION_ID_1 = 0x31,
	WRITE_IMU_CALIBRATION_ID_2,
	WRITE_IMU_CALIBRATION_ID_3,
	READ_IMU_CALIBRATION_ID_1,
	READ_IMU_CALIBRATION_ID_2,
	READ_IMU_CALIBRATION_ID_3,
	IMU_CALIBRATION_SEND_CAN_ID = 0x38,
};

class  CanAllNode: public rclcpp::Node {
public:
	CanAllNode()
		: Node("can_all_node")
		{
		std::vector<int64_t> calibration_params(22, 0);
		declare_parameter("calibration_params", calibration_params);
		std::string can_interface = "can0";
		declare_parameter("can_interface", can_interface);
		temperature_1_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature1", 2);
		temperature_2_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature2", 2);
		temperature_3_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature3", 2);
		temperature_4_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature4", 2);
		temperature_5_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature5", 2);
		imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
		wifi_pub_ = this->create_publisher<std_msgs::msg::String>("wifi", 1);
		BME_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("bme/temperature", 2);
		BME_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("bme/pressure", 2);
		calibration_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("calibration", 50);
		tact_pub_ = this->create_publisher<std_msgs::msg::Int8>("pushed", 50);
		capacitive_touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("capacitive/touch", 50);
		capacitive_touch_raw_pub_ = this->create_publisher<std_msgs::msg::Int16>("capacitive/touch_raw", 50);
		tof_touch_raw_pub_ = this->create_publisher<std_msgs::msg::Int16>("tof/touch_raw", 50);
		servo_pos_pub_ = this->create_publisher<std_msgs::msg::Int16>("servo_pos", 50);
		touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch", 50);
		vibrator_1_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator1", 10,[this](const std_msgs::msg::UInt8::SharedPtr msg) {this->subscribeVibratorData(msg, 1);});
		vibrator_3_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator3", 10,[this](const std_msgs::msg::UInt8::SharedPtr msg) {this->subscribeVibratorData(msg, 3);});
		vibrator_4_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator4", 10,[this](const std_msgs::msg::UInt8::SharedPtr msg) {this->subscribeVibratorData(msg, 4);});
		servo_target_sub_ = this->create_subscription<std_msgs::msg::Int16>("servo_target", 10, std::bind(&CanAllNode::subscribeServoTargetData, this, std::placeholders::_1));
		servo_free_sub_ = this->create_subscription<std_msgs::msg::Bool>("servo_free",10,std::bind(&CanAllNode::subServoFree, this, std::placeholders::_1));
		imu_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>("run_imu_calibration",std::bind(&CanAllNode::readImuServiceCalibration, this, std::placeholders::_1, std::placeholders::_2));
		imu_accel_bias_ = this->declare_parameter("imu_accel_bias", std::vector<double>(3, 0.0)); // parameters for adjusting linear acceleration. default value = [0,0, 0.0, 0.0]
		imu_gyro_bias_ = this->declare_parameter("imu_gyro_bias", std::vector<double>(3, 0.0)); // parameters for adjusting angular velocity. default value = [0,0, 0.0, 0.0]
		can_socket_ = openCanSocket();
		writeImuCalibration();
		// initialize diagonal elements of imu_msg.orientation_covariance
		imu_msg.orientation_covariance[0] = 0.1;
		imu_msg.orientation_covariance[4] = 0.1;
		imu_msg.orientation_covariance[8] = 0.1;
		pub_timer_ = this->create_wall_timer(
			std::chrono::microseconds(100),
			std::bind(&CanAllNode::timerPubCallback, this));
	}

private:
	int openCanSocket() {
		std::string can_interface = this->get_parameter("can_interface").as_string();
		int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error while opening socket");
			return -1;
		}
		struct ifreq ifr;
		strcpy(ifr.ifr_name, can_interface.c_str());
		ioctl(s, SIOCGIFINDEX, &ifr);
		struct sockaddr_can addr;
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error in socket bind");
			close(s);
			return -1;
		}
	return s;
	}

	void writeImuCalibration() {
		auto param = get_parameter("calibration_params").as_integer_array();
		struct can_frame frame;
		std::memset(&frame, 0, sizeof(struct can_frame));
		frame.can_id = CanId::IMU_CALIBRATION_SEND_CAN_ID;
		frame.can_dlc = 1;
		frame.data[0] = 0x01;
		int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
		if (nbytes <= 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send write Imu calibration frame");
		} else {
			RCLCPP_INFO(this->get_logger(), "write Imu calibration sent successfully");
		}
		usleep(1000);
		frame.can_id = CanId::WRITE_IMU_CALIBRATION_ID_1;
		frame.can_dlc = 8;
		for (int i = 0; i < 8; ++i) {
			frame.data[i] = (param[i] >> 0) & 0xFF;
		}
		nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
		usleep(1000);
		frame.can_id = CanId::WRITE_IMU_CALIBRATION_ID_2;
		frame.can_dlc = 8;
		for (int i = 0; i < 8; ++i) {
			frame.data[i] = (param[8 + i] >> 0) & 0xFF;
		}
		nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
		usleep(1000);
		frame.can_id = CanId::WRITE_IMU_CALIBRATION_ID_3;
		frame.can_dlc = 6;
		for (int i = 0; i < 6; ++i) {
			frame.data[i] = (param[16 + i] >> 0) & 0xFF;
		}
		nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
	}

	void timerPubCallback() {
		struct can_frame frame;
		int nbytes = read(can_socket_ , &frame, sizeof(struct can_frame));
		if (nbytes > 0) {
			if (frame.can_id >= CanId::TEMPERATURE_1_CAN_ID && frame.can_id <= CanId::TEMPERATURE_5_CAN_ID) {
				publishTemperatureData(frame);
			} else if (frame.can_id >= CanId::WIFI_CAN_ID_START && frame.can_id <= CanId::WIFI_CAN_ID_END) {
				publishWifiData(frame);
			} else if (frame.can_id == CanId::BME_CAN_ID) {
				publishBmeData(frame);
			} else if (frame.can_id >= CanId::READ_IMU_CALIBRATION_ID_1 && frame.can_id <= CanId::READ_IMU_CALIBRATION_ID_3) {
				readImuCalibration(frame);
			} else if (frame.can_id == CanId::TACT_CAN_ID) {
				publishTactData(frame);
			} else if (frame.can_id == CanId::TOUCH_CAN_ID){
				publishTouchData(frame);
			} else if (frame.can_id >= CanId::IMU_LINEAR_CAN_ID && frame.can_id <= CanId::IMU_ORIENTATION_CAN_ID){
				publishImuData(frame);
			} else if (frame.can_id == CanId::SERVO_POS_CAN_ID){
				publishServoPosData(frame);
			}
		}
	}

	void readImuServiceCalibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
																				std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
		struct can_frame frame;
		std::memset(&frame, 0, sizeof(struct can_frame));
		frame.can_id = CanId::IMU_CALIBRATION_SEND_CAN_ID;
		frame.can_dlc = 1;
		frame.data[0] = 0x00;
		int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
		if (nbytes <= 0) {
			RCLCPP_ERROR(this->get_logger(), "Error sending read Imu calibration frame");
			response->success = false;
			response->message = "Error sending read Imu calibration frame";
		} else {
			RCLCPP_INFO(this->get_logger(), "read Imu calibration sent successfully");
			response->success = true;
			response->message = "read Imu calibration sent successfully";
		}
	}

	void readImuCalibration(const struct can_frame &frame) {
		static std::vector<int32_t> calibration_data(22);
		static int count = 0;
		if (count == 0) {
			std::fill(calibration_data.begin(), calibration_data.end(), 0);
		}
		if (frame.can_id == CanId::READ_IMU_CALIBRATION_ID_1) {
			if (frame.can_dlc >= 8) {
				for (int i = 0; i < 8; i++) {
					calibration_data[i] = static_cast<int32_t>(frame.data[i]);
				}
				count++;
			}
		} else if (frame.can_id == CanId::READ_IMU_CALIBRATION_ID_2) {
			if (frame.can_dlc >= 8) {
				for (int i = 0; i < 8; i++) {
					calibration_data[i + 8] = static_cast<int32_t>(frame.data[i]);
				}
				count++;
			}
		} else if (frame.can_id == CanId::READ_IMU_CALIBRATION_ID_3) {
			if (frame.can_dlc >= 6) {
				for (int i = 0; i < 6; i++) {
					calibration_data[i + 16] = static_cast<int32_t>(frame.data[i]);
				}
				count++;
			}
		}
		if (count == 3) {
			std_msgs::msg::Int32MultiArray calibration_msg;
			calibration_msg.data = calibration_data;
			calibration_pub_->publish(calibration_msg);
			count = 0;
		}
		count = 0;
	}

	void publishImuData(const struct can_frame &frame) {
		static bool linear_data_received = false;
		static bool angular_data_received = false;
		if (frame.can_id == CanId::IMU_LINEAR_CAN_ID) {
			if (frame.can_dlc >= 6) {
				int16_t linear_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
				int16_t linear_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
				int16_t linear_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
				imu_msg.linear_acceleration.x = linear_x / 100.0;
				imu_msg.linear_acceleration.y = linear_y / 100.0;
				imu_msg.linear_acceleration.z = linear_z / 100.0;
				imu_msg.linear_acceleration.x -= this->imu_accel_bias_.at(0);
				imu_msg.linear_acceleration.y -= this->imu_accel_bias_.at(1);
				imu_msg.linear_acceleration.z -= this->imu_accel_bias_.at(2);

				linear_data_received = true;
				angular_data_received = false;
			}
		}
		if (frame.can_id == CanId::IMU_ANGULAR_CAN_ID) {
			if (angular_data_received) {
				linear_data_received = false;
				angular_data_received = false;
			} else if (frame.can_dlc >= 6) {
				int16_t angular_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
				int16_t angular_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
				int16_t angular_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
				imu_msg.angular_velocity.x = (angular_x / 16.0) * (M_PI / 180.0);
				imu_msg.angular_velocity.y = (angular_y / 16.0) * (M_PI / 180.0);
				imu_msg.angular_velocity.z = (angular_z / 16.0) * (M_PI / 180.0);
				imu_msg.angular_velocity.x -= this->imu_gyro_bias_.at(0);
				imu_msg.angular_velocity.y -= this->imu_gyro_bias_.at(1);
				imu_msg.angular_velocity.z -= this->imu_gyro_bias_.at(2);
				angular_data_received = true;
			}
		}
		if (frame.can_id == CanId::IMU_ORIENTATION_CAN_ID) {
			if (frame.can_dlc >= 8) {
				int16_t orientation_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
				int16_t orientation_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
				int16_t orientation_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
				int16_t orientation_w = (((uint16_t)frame.data[7]) << 8) | ((uint16_t)frame.data[6]);
				imu_msg.orientation.x = orientation_x * (1.0 / (1 << 14));
				imu_msg.orientation.y = orientation_y * (1.0 / (1 << 14));
				imu_msg.orientation.z = orientation_z * (1.0 / (1 << 14));
				imu_msg.orientation.w = orientation_w * (1.0 / (1 << 14));
				if (linear_data_received && angular_data_received) {
					imu_msg.header.stamp = this->get_clock()->now();
					imu_msg.header.frame_id = "imu_frame";
					imu_pub_->publish(imu_msg);
				}
				linear_data_received = false;
				angular_data_received = false;
			}
		}
	}

	void publishWifiData(const struct can_frame &frame) {
		static std::array<uint8_t, 6> mac_address{};
		static std::string ssid;
		static int8_t channel = 0;
		static int8_t rssi = 0;
		static int8_t frame_count = 0;
		if (frame.can_id == CanId::WIFI_CAN_ID_START) {
			ssid.clear();
			frame_count = 0;
		}
		if (frame.can_id == CanId::WIFI_CAN_ID_END) {
			for (int i = 0; i < 6; ++i) {
				mac_address[i] = frame.data[i];
			}
			channel = frame.data[6];
			rssi = frame.data[7];
			frame_count++;
		}
		else if (frame.can_id >= CanId::WIFI_CAN_ID_START && frame.can_id <= CanId::WIFI_SSID_CAN_ID_END) {
			for (int i = 0; i < frame.can_dlc; ++i) {
				if (frame.data[i] != '\0') {
					ssid += static_cast<char>(frame.data[i]);
				}
			}
			frame_count++;
		}
		if (frame_count >= 5 && !ssid.empty() && mac_address[0] != 0 && channel != 0 && rssi != 0) {
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
			std::string message = mac_str + "," + ssid + "," + std::to_string(channel) + "," + std::to_string(rssi) + "," + std::to_string(seconds) + "," + nanoseconds;
			std_msgs::msg::String msg;
			msg.data = message;
			wifi_pub_->publish(msg);
			mac_address.fill(0);
			ssid.clear();
			channel = 0;
			rssi = 0;
			frame_count = 0;
		}
	}

	void publishBmeData(const struct can_frame &frame) {
		if (frame.can_id == CanId::BME_CAN_ID) {
			if (frame.can_dlc >= 8) {
				int16_t temperature_raw = ((uint16_t)(frame.data[1] << 8) | ((uint16_t)frame.data[0]));
				float temperature = temperature_raw / 100.0;
				sensor_msgs::msg::Temperature temp_msg;
				temp_msg.header.stamp = this->get_clock()->now();
				temp_msg.temperature = temperature;
				BME_temperature_pub_->publish(temp_msg);
				int32_t pressure_raw = ((uint32_t)frame.data[7] << 24) | ((uint32_t)frame.data[6] << 16) | ((uint32_t)frame.data[5] << 8) | ((uint32_t)frame.data[4] << 0);
				float pressure = pressure_raw / 100.0;
				sensor_msgs::msg::FluidPressure pressure_msg;
				pressure_msg.header.stamp = this->get_clock()->now();
				pressure_msg.fluid_pressure = pressure;
				BME_pressure_pub_->publish(pressure_msg);
			}
		}
	}

	void publishTouchData(const struct can_frame &frame) {
		if (frame.can_id == CanId::TOUCH_CAN_ID && frame.can_dlc >= 4) {
			int16_t capacitive_touch = frame.data[3];
			std_msgs::msg::Int16 touch_msg;
			touch_msg.data = capacitive_touch;
			capacitive_touch_pub_->publish(touch_msg);
			touch_pub_->publish(touch_msg);
			int16_t capacitive_touch_raw = frame.data[2];
			std_msgs::msg::Int16 touch_raw_msg;
			touch_raw_msg.data = capacitive_touch_raw;
			capacitive_touch_raw_pub_->publish(touch_raw_msg);
			int16_t tof_raw = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
			std_msgs::msg::Int16 tof_raw_msg;
			tof_raw_msg.data = tof_raw;
			tof_touch_raw_pub_->publish(tof_raw_msg);
		}
	}

	void publishTactData(const struct can_frame &frame) {
		if (frame.can_id == CanId::TACT_CAN_ID && frame.can_dlc >= 1) {
			std_msgs::msg::Int8 tact_msg;
			uint8_t tact_data = 0;
			if (frame.data[0] == 1){
				tact_data = 8;
			}
			if (frame.data[0] == 2){
				tact_data = 4;
			}
			if (frame.data[0] == 4){
				tact_data = 1;
			}
			if (frame.data[0] == 8){
				tact_data = 2;
			}
			tact_msg.data = tact_data;
			tact_pub_->publish(tact_msg);
		}
	}

	void publishServoPosData(const struct can_frame &frame) {
		if (frame.can_id == CanId::SERVO_POS_CAN_ID && frame.can_dlc >= 2) {
			int16_t servo_pos_raw = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
			float servo_pos = ((servo_pos_raw - 2048) / 1024.0) * 90;
			std_msgs::msg::Int16 servo_pos_pub_msg;
			servo_pos_pub_msg.data = static_cast<int16_t>(servo_pos);
			servo_pos_pub_->publish(servo_pos_pub_msg);
		}
	}

	void publishTemperatureData(const struct can_frame &frame) {
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
			default:
				break;
		}
	}

	void subscribeVibratorData(const std_msgs::msg::UInt8::SharedPtr msg, int vibrator_id) {
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
		frame.can_dlc = 3;
		frame.data[0] = vibrator1;
		frame.data[1] = vibrator3;
		frame.data[2] = vibrator4;
		if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			RCLCPP_ERROR(this->get_logger(), "Error sending Vibrator frame");
		}
	}

	void subscribeServoTargetData(const std_msgs::msg::Int16& msg) {
		int16_t servo_target_per = msg.data;
		float servo_targer_deg = (servo_target_per / 90.0) * 1024 + 2048;
		int16_t servo_target = static_cast<int16_t>(servo_targer_deg);
		struct can_frame frame;
		std::memset(&frame, 0, sizeof(struct can_frame));
		frame.can_id = CanId::SERVO_TARGET_CAN_ID;
		frame.can_dlc = 4;
		frame.data[0] = servo_target & 0xFF;
		frame.data[1] = (servo_target >> 8) & 0xFF;
		if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			RCLCPP_ERROR(this->get_logger(), "Error sending servo target frame");
		}
	}

	void subServoFree(const std_msgs::msg::Bool::SharedPtr msg) {
		uint8_t servo_free = msg->data ? 0x00 : 0x01;
		struct can_frame frame;
		std::memset(&frame, 0, sizeof(struct can_frame));
		frame.can_id = CanId::SERVO_SWITCH_CAN_ID;
		frame.can_dlc = 1;
		frame.data[0] = servo_free;
		if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			RCLCPP_ERROR(this->get_logger(), "Error sending servo free frame");
		}
	}

	sensor_msgs::msg::Imu imu_msg;
	int can_socket_;
	std::vector<double> imu_accel_bias_;
	std::vector<double> imu_gyro_bias_;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_1_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_2_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_3_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_4_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_5_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr BME_temperature_pub_;
	rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr BME_pressure_pub_;
	rclcpp::Publisher< std_msgs::msg::Int32MultiArray>::SharedPtr calibration_pub_;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr tact_pub_;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr capacitive_touch_pub_;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr capacitive_touch_raw_pub_;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr tof_touch_raw_pub_;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_pub_;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_pos_pub_;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_1_sub_;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_3_sub_;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_4_sub_;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo_target_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_free_sub_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibration_srv_;
	rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CanAllNode>());
	rclcpp::shutdown();
	return 0;
}
