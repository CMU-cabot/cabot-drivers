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
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_srvs/srv/trigger.hpp" 
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
# include <bits/stdc++.h>

const int IMU_CALIBRATION_SEND_CAN_ID = 0x04;
const int IMU_LINEAR_CAN_ID = 0x09;
const int IMU_ANGULAR_CAN_ID = 0x0A;
const int IMU_ORIENTATION_CAN_ID = 0x0B;
const int WIFI_SSID_CAN_ID_START = 0x0C;
const int WIFI_SSID_CAN_ID_END = 0x0F;
const int BME_CAN_ID = 0x11;
const int TOUCH_CAN_ID = 0x12;
const int TACT_CAN_ID = 0x13;
const int VIBRATOR_CAN_ID = 0x1B;
const int SERVO_TARGET_CAN_ID = 0x1C;
const int SERVO_POS_CAN_ID = 0x1F;
const int SERVO_SWITCH_CAN_ID = 0x20;
const int TEMPERATURE_CAN_ID_1 = 0x21;
const int TEMPERATURE_CAN_ID_2 = 0x22;
const int TEMPERATURE_CAN_ID_3 = 0x23;
const int TEMPERATURE_CAN_ID_4 = 0x24;
const int TEMPERATURE_CAN_ID_5 = 0x25;
const int WRITE_IMU_CALIBRATION_ID_START = 0x31;
const int WRITE_IMU_CALIBRATION_ID_END = 0x33;
const int READ_IMU_CALIBRATION_ID_START = 0x34;

const int READ_IMU_CALIBRATION_ID_END = 0x36;

class  CanToRos2Node: public rclcpp::Node {
public:
    CanToRos2Node()
        : Node("can_all_node")
        {
        std::vector<int64_t> calibration_params(22, 0);
        declare_parameter("calibration_params", calibration_params);
        auto param = get_parameter("calibration_params").as_integer_array();
        std::cout << "calibration_params: ";
        for(const auto& val : param) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
        linear_data_received_ = false;
        angular_data_received_ = false;
        orientation_data_received_ = false;    
        temperature_pub_1_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature1", 2);
        temperature_pub_2_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature2", 2);
        temperature_pub_3_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature3", 2);
        temperature_pub_4_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature4", 2);
        temperature_pub_5_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature5", 2);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 100);
        wifi_pub_ = this->create_publisher<std_msgs::msg::String>("wifi", 10); 
        BME_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("bme/temperature", 2);
        BME_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("bme/pressure", 2); 
        calibration_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("calibration", 50);
        tact_pub_ = this->create_publisher<std_msgs::msg::Int8>("pushed", 50);
        capacitive_touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("capacitive/touch", 50);
        servo_pos_pub_ = this->create_publisher<std_msgs::msg::Int16>("servo_pos", 50);
        touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch", 50);

        vibrator_1_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator1", 10, std::bind(&CanToRos2Node::vibrator1Callback, this, std::placeholders::_1));
        vibrator_2_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator2", 10, std::bind(&CanToRos2Node::vibrator2Callback, this, std::placeholders::_1));
        vibrator_3_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator3", 10, std::bind(&CanToRos2Node::vibrator3Callback, this, std::placeholders::_1));
        servo_target_sub_ = this->create_subscription<std_msgs::msg::Int16>("servo_target", 10,std::bind(&CanToRos2Node::servoTargetCallback, this, std::placeholders::_1));
        
        imu_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "run_imu_calibration",
            std::bind(&CanToRos2Node::readImuCalibration, this, std::placeholders::_1, std::placeholders::_2)
        );
        can_socket_ = openCanSocket();
        writeImuCalibration();
        
        pub_timer_ = this->create_wall_timer(
            std::chrono::microseconds(100),
            std::bind(&CanToRos2Node::timerPubCallback, this));

        // sub_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(1),
        //     std::bind(&CanToRos2Node::timerSubCallback, this));
    }

private:
    int openCanSocket() {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error while opening socket");
            return -1;
        }
        struct ifreq ifr;
        strcpy(ifr.ifr_name,"can0");
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

    void readImuCalibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(struct can_frame));
        frame.can_id = IMU_CALIBRATION_SEND_CAN_ID;
        frame.can_dlc = 1;
        frame.data[0] = 0x00;
        int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame. nbytes: %d", nbytes);
            response->success = false;
            response->message = "Failed to send CAN frame";
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN frame sent successfully. nbytes: %d", nbytes);
            response->success = true;
            response->message = "CAN frame sent successfully";
        }
    }

    void timerPubCallback() {
        struct can_frame frame;
        int nbytes = read(can_socket_ , &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            if (frame.can_id >= TEMPERATURE_CAN_ID_1 && frame.can_id <= TEMPERATURE_CAN_ID_5) {
                publishTemperatureData(frame);
            } else if (frame.can_id >= 0x0c && frame.can_id <= 0x10) {
                publishWifiData(frame); 
            } else if (frame.can_id == BME_CAN_ID) {
                publishBmeData(frame);
            } else if (frame.can_id >= READ_IMU_CALIBRATION_ID_START && frame.can_id <= READ_IMU_CALIBRATION_ID_END) {
                 readImuCalibration2(frame);
            } else if (frame.can_id == TACT_CAN_ID) {
                publishTactData(frame);   
            }else if (frame.can_id == TOUCH_CAN_ID){
                publishTouchData(frame);
            } else if (frame.can_id >= IMU_LINEAR_CAN_ID && frame.can_id <= IMU_ORIENTATION_CAN_ID){
                publishImuData(frame);
            }else if (frame.can_id == SERVO_POS_CAN_ID){
                publishServoPosData(frame);
            }
        }
    }
    // void timerSubCallback() {
    //     struct can_frame frame;
    //     int nbytes = read(can_socket_ , &frame, sizeof(struct can_frame));
    //     if (nbytes > 0) {
    //         if (frame.can_id == VIBRATOR_CAN_ID) {
    //             sabscribeVibratorData();    
    //         }else if (frame.can_id == SERVO_TARGET_CAN_ID) {
    //             subscribeServoTargetData();    
    //         }
    //     }
    // }


    void publishTemperatureData(const struct can_frame &frame) {
        int16_t temperature_raw = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
        float temperature = temperature_raw * 0.0625;
        sensor_msgs::msg::Temperature msg;
        msg.header.stamp = this->get_clock()->now();
        msg.temperature = temperature;

        switch (frame.can_id) {
            case TEMPERATURE_CAN_ID_1:
                temperature_pub_1_->publish(msg);
                break;
            case TEMPERATURE_CAN_ID_2:
                temperature_pub_2_->publish(msg);
                break;
            case TEMPERATURE_CAN_ID_3:
                temperature_pub_3_->publish(msg);
                break;
            case TEMPERATURE_CAN_ID_4:
                temperature_pub_4_->publish(msg);
                break;
            case TEMPERATURE_CAN_ID_5:
                temperature_pub_5_->publish(msg);
                break;
            default:
                break;
        }
    }

    void publishImuData(const struct can_frame &frame) {
        if (frame.can_id == IMU_LINEAR_CAN_ID) {
            if (frame.can_dlc >= 6) {
                int16_t linear_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
                int16_t linear_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
                int16_t linear_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
                imu_msg.linear_acceleration.x = linear_x / 100.0;
                imu_msg.linear_acceleration.y = linear_y / 100.0;
                imu_msg.linear_acceleration.z = linear_z / 100.0;
                linear_data_received_ = true;
            }
        }
        if (frame.can_id == IMU_ANGULAR_CAN_ID) {
            if (frame.can_dlc >= 6) {
                int16_t angular_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
                int16_t angular_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
                int16_t angular_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
                imu_msg.angular_velocity.x = (angular_x / 16.0) * (M_PI / 180.0); // 16deg/s -> rad/s
                imu_msg.angular_velocity.y = (angular_y / 16.0) * (M_PI / 180.0); // 16deg/s -> rad/s
                imu_msg.angular_velocity.z = (angular_z / 16.0) * (M_PI / 180.0); // 16deg/s -> rad/s
                angular_data_received_ = true;
            }
        }
        if (frame.can_id == IMU_ORIENTATION_CAN_ID) {
            if (frame.can_dlc >= 8) {
                int16_t orientation_x = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
                int16_t orientation_y = (((uint16_t)frame.data[3]) << 8) | ((uint16_t)frame.data[2]);
                int16_t orientation_z = (((uint16_t)frame.data[5]) << 8) | ((uint16_t)frame.data[4]);
                int16_t orientation_w = (((uint16_t)frame.data[7]) << 8) | ((uint16_t)frame.data[6]);
                imu_msg.orientation.x = orientation_x / 32767.0;
                imu_msg.orientation.y = orientation_y / 32767.0;
                imu_msg.orientation.z = orientation_z / 32767.0;
                imu_msg.orientation.w = orientation_w / 32767.0;
                orientation_data_received_ = true;
            }
        }
        if (linear_data_received_ && angular_data_received_ && orientation_data_received_) {
            imu_msg.header.stamp = this->get_clock()->now();
            imu_pub_->publish(imu_msg);
            linear_data_received_ = false;
            angular_data_received_ = false;
            orientation_data_received_ = false;
        }
    }

    void publishWifiData(const struct can_frame &frame) {
        static std::array<uint8_t, 6> mac_address{};
        static std::string ssid;
        static int8_t channel = 0;
        static int8_t rssi = 0;

        if (frame.can_id == 0x10) {
            for (int i = 0; i < 6; ++i) {
                mac_address[i] = frame.data[i];
            }
            channel = frame.data[6];
            rssi = frame.data[7];
        } 
        else if (frame.can_id >= 0x0C && frame.can_id <= 0x0F) {
            for (int i = 0; i < frame.can_dlc; ++i) {
                if (frame.data[i] != '\0') {
                    ssid += static_cast<char>(frame.data[i]);
                }
            }
        }

        if (!ssid.empty() && mac_address[0] != 0 && channel != 0 && rssi != 0) {
            std::string mac_str;
            for (size_t i = 0; i < mac_address.size(); ++i) {
                if (i != 0) {
                    mac_str += ":";
                }
                std::stringstream hex_stream;
                hex_stream << std::hex << static_cast<int>(mac_address[i]);
                mac_str += hex_stream.str();
            }
            rclcpp::Time current_time = rclcpp::Clock().now();
            int seconds = current_time.seconds();
            int64_t nanoseconds = current_time.nanoseconds();
            std::string num_str = std::to_string(nanoseconds);
            std::string last_9_digits = num_str.substr(num_str.length() - 9);
            std::string message = mac_str + "," + ssid + "," + std::to_string(channel) + "," + std::to_string(rssi) + "," + std::to_string(seconds) + "," + last_9_digits;
            std_msgs::msg::String msg;
            msg.data = message;
            wifi_pub_->publish(msg);

            mac_address.fill(0);
            ssid.clear();
            channel = 0;
            rssi = 0;
        }
    }

    void publishBmeData(const struct can_frame &frame) {
        if (frame.can_id == BME_CAN_ID) {
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

    void writeImuCalibration() {
        auto param = get_parameter("calibration_params").as_integer_array();
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(struct can_frame));
        frame.can_id = IMU_CALIBRATION_SEND_CAN_ID;
        frame.can_dlc = 1;
        frame.data[0] = 0x01;

        int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to CAN ID: 0x04");
        } else {
            RCLCPP_INFO(this->get_logger(), "Written 0x01 to CAN ID: 0x04");
        }

        frame.can_id = 0x31;
        frame.can_dlc = 8;
        for (int i = 0; i < 8; ++i) {
            frame.data[i] = (param[i] >> 0) & 0xFF;
        }
        nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
        frame.can_id = 0x32;
        frame.can_dlc = 8;
        for (int i = 0; i < 8; ++i) {
            frame.data[i] = (param[8 + i] >> 0) & 0xFF;
        }
        nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
        frame.can_id = 0x33;
        frame.can_dlc = 6;
        for (int i = 0; i < 6; ++i) {
            frame.data[i] = (param[16 + i] >> 0) & 0xFF;
        }
        nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    }

    void readImuCalibration2(const struct can_frame &frame) {
        static std::vector<int32_t> calibration_data(22); 
        static int received_count = 0; 
        if (frame.can_id == 0x34) {
            if (frame.can_dlc >= 8) {
                for (int i = 0; i < 8; i++) {
                    calibration_data[i] = static_cast<int32_t>(frame.data[i]);
                }
                received_count++;
            }
        } else if (frame.can_id == 0x35) {
            if (frame.can_dlc >= 8) {
                for (int i = 0; i < 8; i++) {
                    calibration_data[i + 8] = static_cast<int32_t>(frame.data[i]);
                }
                received_count++;
            }
        } else if (frame.can_id == 0x36) {
            if (frame.can_dlc >= 6) {
                for (int i = 0; i < 6; i++) {
                    calibration_data[i + 16] = static_cast<int32_t>(frame.data[i]);
                }
                received_count++;
            }
        }
        if (received_count == 3) {
            std_msgs::msg::Int32MultiArray calibration_msg;
            calibration_msg.data = calibration_data;
            calibration_publisher_->publish(calibration_msg);
            received_count = 0;
        }
    }

    void publishTactData(const struct can_frame &frame) {
        if (frame.can_id == TACT_CAN_ID && frame.can_dlc >= 1) {
            std_msgs::msg::Int8 tact_msg;
            tact_msg.data = frame.data[0];
            tact_pub_->publish(tact_msg);
        }
    }

    // void vibrator1Callback(const std_msgs::msg::UInt8::SharedPtr msg)
    // {
    //     data1_ = msg->data;
    // }

    // void vibrator2Callback(const std_msgs::msg::UInt8::SharedPtr msg)
    // {
    //     data2_ = msg->data;
    // }

    // void vibrator3Callback(const std_msgs::msg::UInt8::SharedPtr msg)
    // {
    //     data3_ = msg->data;
    // }

    // void sabscribeVibratorData() {
    //     struct can_frame vibrator_frame;
    //     std::memset(&vibrator_frame, 0, sizeof(struct can_frame));
    //     vibrator_frame.can_id = VIBRATOR_CAN_ID;
    //     vibrator_frame.can_dlc = 3;
    //     vibrator_frame.data[0] = data1_;
    //     vibrator_frame.data[1] = data2_;
    //     vibrator_frame.data[2] = data3_;
    //     if (write(can_socket_, &vibrator_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message");
    //     }
    // }

    void publishTouchData(const struct can_frame &frame) {
        if (frame.can_id == TOUCH_CAN_ID && frame.can_dlc >= 4) {
            int16_t capacitive_touch = frame.data[3];
            std_msgs::msg::Int16 touch_msg;
            touch_msg.data = capacitive_touch;
            capacitive_touch_pub_->publish(touch_msg);
            touch_pub_->publish(touch_msg);
        }
    }

    void publishServoPosData(const struct can_frame &frame) {
        if (frame.can_id == SERVO_POS_CAN_ID && frame.can_dlc >= 2) {
            int16_t servo_pos2 = (((uint16_t)frame.data[1]) << 8) | ((uint16_t)frame.data[0]);
            float servo_pos = ((servo_pos2 - 2048) / 1024.0) * 90;
            std_msgs::msg::Int16 servo_pos_pub_msg;
            servo_pos_pub_msg.data = static_cast<int16_t>(servo_pos);
            servo_pos_pub_->publish(servo_pos_pub_msg);
        }
    }
    // void servoTargetCallback(const std_msgs::msg::Int16::SharedPtr msg)
    // {
    //     data4_ = msg->data;
    // }

    // void subscribeServoTargetData() {
    //     struct can_frame vibrator_frame;
    //     std::memset(&vibrator_frame, 0, sizeof(struct can_frame));
    //     vibrator_frame.can_id = 0x1c; 
    //     vibrator_frame.can_dlc = 2;
    //     vibrator_frame.data[0] = static_cast<uint8_t>(data4_ & 0xFF);  
    //     vibrator_frame.data[1] = static_cast<uint8_t>((data4_ >> 8) & 0xFF);
    //     int nbytes = write(can_socket_, &vibrator_frame, sizeof(struct can_frame));
    //     if (nbytes <= 0) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to write to CAN ID: 0x1c");
    //     } else {
    //         RCLCPP_INFO(this->get_logger(), "Written data to CAN ID: 0x1c");
    //     }
    // } 

    sensor_msgs::msg::Imu imu_msg;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_1_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_2_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_3_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_4_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_5_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr BME_temperature_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr BME_pressure_pub_; 
    rclcpp::Publisher< std_msgs::msg::Int32MultiArray>::SharedPtr calibration_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr tact_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr capacitive_touch_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_pos_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_1_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_2_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_3_;   
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo_target_sub_;   
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibration_srv_;
    bool linear_data_received_ = false;
    bool angular_data_received_ = false;
    bool orientation_data_received_ = false;
    bool received_34_ = false;
    bool received_35_ = false;
    bool received_36_ = false;
    int can_socket_;
    bool wifi_received_[4] = {false, false, false, false};
    uint8_t data1_ = 0, data2_ = 0, data3_ = 0;
    int16_t data4_= 0;
    std::string wifi_data_[4];   
    
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr sub_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanToRos2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
