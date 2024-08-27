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

    
int openCanSocket(const std::string &can_interface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error while opening socket");
        return -1;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface.c_str());
    ioctl(sock, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error in socket bind");
        return -2;
    }

    return sock;
}

class  CanToRos2Node: public rclcpp::Node {
public:
    CanToRos2Node()
        : Node("can_all_node"),
          can_sock_(openCanSocket("can0")),
          linear_data_received_(false),
          angular_data_received_(false),
          orientation_data_received_(false) {

        std::vector<int64_t> calibration_params(22, 0);
        declare_parameter("calibration_params", calibration_params);
        auto param = get_parameter("calibration_params").as_integer_array();
        std::cout << "calibration_params: ";
        for(const auto& val : param) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        temperature_pub_1_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature1", 2);
        temperature_pub_2_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature2", 2);
        temperature_pub_3_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature3", 2);
        temperature_pub_4_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature4", 2);
        temperature_pub_5_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature5", 2);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 100);
        wifi_pub_ = this->create_publisher<std_msgs::msg::String>("/wifi", 10); 
        BME_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/bme/temperature", 2);
        BME_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/bme/pressure", 2); 
        calibration_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/calibration", 50);
        tact_1_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pushed_1", 50);
        tact_2_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pushed_2", 50);
        tact_3_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pushed_3", 50);
        tact_4_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pushed_4", 50);
        capacitive_touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("/capacitive/touch", 50);

        vibrator_1_ = this->create_subscription<std_msgs::msg::UInt8>("/vibrator1", 10, std::bind(&CanToRos2Node::vibrator1Callback, this, std::placeholders::_1));
        vibrator_2_ = this->create_subscription<std_msgs::msg::UInt8>("/vibrator2", 10, std::bind(&CanToRos2Node::vibrator2Callback, this, std::placeholders::_1));
        vibrator_3_ = this->create_subscription<std_msgs::msg::UInt8>("/vibrator3", 10, std::bind(&CanToRos2Node::vibrator3Callback, this, std::placeholders::_1));

        imu_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/run_imu_calibration",
            std::bind(&CanToRos2Node::readImuCalibration, this, std::placeholders::_1, std::placeholders::_2)
        );

        // writeImuCalibration();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&CanToRos2Node::timerCallback, this));
    }
private:
    void readImuCalibration(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(struct can_frame));
        frame.can_id = IMU_CALIBRATION_SEND_CAN_ID;
        frame.can_dlc = 1;
        frame.data[0] = 0x00;
        int nbytes = write(can_sock_, &frame, sizeof(struct can_frame));
        if (nbytes <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame. nbytes: %d", nbytes);
            response->success = false;
            response->message = "Failed to send CAN frame";
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN frame sent successfully. nbytes: %d", nbytes);
            response->success = true;
            response->message = "CAN frame sent successfully";
            timerCallback();
        }
    }

    void timerCallback() {
        struct can_frame frame;
        int nbytes = read(can_sock_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            if (frame.can_id >= TEMPERATURE_CAN_ID_1 && frame.can_id <= TEMPERATURE_CAN_ID_5) {
                publishTemperatureData(frame);
            } else if (frame.can_id >= WIFI_SSID_CAN_ID_START && frame.can_id <= WIFI_SSID_CAN_ID_END) {
                publishWifiData(frame); 
            } else if (frame.can_id == BME_CAN_ID) {
                publishBmeData(frame);
            } else if (frame.can_id >= READ_IMU_CALIBRATION_ID_START && frame.can_id <= READ_IMU_CALIBRATION_ID_END) {
                 readImuCalibration2(frame);
            } else if (frame.can_id == TACT_CAN_ID) {
                publishTactData(frame);   
            } else if (frame.can_id == VIBRATOR_CAN_ID) {
                sabscribeVibratorData();    
            } else if (frame.can_id == TOUCH_CAN_ID){
                publishTouchData(frame);
            } else {
                publishImuData(frame);
            }
        }
    }

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
            imu_pub_->publish(imu_msg);
            linear_data_received_ = false;
            angular_data_received_ = false;
            orientation_data_received_ = false;
        }
    }

    void publishWifiData(const struct can_frame &frame) {
        if (frame.can_id >= WIFI_SSID_CAN_ID_START && frame.can_id <= WIFI_SSID_CAN_ID_END) {
            wifi_data_[frame.can_id - WIFI_SSID_CAN_ID_START] = std::string((const char*)frame.data, frame.can_dlc);
            wifi_received_[frame.can_id - WIFI_SSID_CAN_ID_START] = true;
            if (wifi_received_[0] && wifi_received_[1] && wifi_received_[2] && wifi_received_[3]) {
                std_msgs::msg::String wifi_msg;
                wifi_msg.data = wifi_data_[0] + wifi_data_[1] + wifi_data_[2] + wifi_data_[3];  
                wifi_pub_->publish(wifi_msg);
                std::fill(std::begin(wifi_received_), std::end(wifi_received_), false);
            }
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

        int nbytes = write(can_sock_, &frame, sizeof(struct can_frame));
        if (nbytes <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to CAN ID: 0x04");
        } else {
            RCLCPP_INFO(this->get_logger(), "Written 0x01 to CAN ID: 0x04");
        }
        for (int i = 0; i < 3; ++i) {
            frame.can_id = WRITE_IMU_CALIBRATION_ID_START + i;
            frame.can_dlc = 8;
            for (int j = 0; j < 2; ++j) {
                int32_t data = param[i * 2 + j];
                frame.data[j * 4 + 0] = (data >> 0) & 0xFF;
                frame.data[j * 4 + 1] = (data >> 8) & 0xFF;
                frame.data[j * 4 + 2] = (data >> 16) & 0xFF;
                frame.data[j * 4 + 3] = (data >> 24) & 0xFF;
            }
            nbytes = write(can_sock_, &frame, sizeof(struct can_frame));
            if (nbytes <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to CAN ID: 0x%02X", frame.can_id);
            } else {
                RCLCPP_INFO(this->get_logger(), "Written calibration data to CAN ID: 0x%02X", frame.can_id);
            }
        }
    }

    void readImuCalibration2(const struct can_frame &frame) {
        static std::vector<int32_t> calibration_data;
        static bool received_33 = false;
        static bool received_34 = false;
        static bool received_35 = false;

        if (frame.can_id == 0x34 && !received_33) {
            for (uint8_t byte : frame.data) {
                calibration_data.push_back(static_cast<int32_t>(byte));
            }
            received_33 = true;
        } else if (frame.can_id == 0x35 && !received_34) {
            for (uint8_t byte : frame.data) {
                calibration_data.push_back(static_cast<int32_t>(byte));
            }
            received_34 = true;
        } else if (frame.can_id == 0x36 && !received_35) {
            for (uint8_t byte : frame.data) {
                calibration_data.push_back(static_cast<int32_t>(byte));
            }
            received_35 = true;
        }

        if (received_33 && received_34 && received_35) {
            std::cout << "Received IMU calibration data:" << std::endl;
            for (const auto &data : calibration_data) {
                std::cout << data << " ";
            }
            std::cout << std::endl;

            std_msgs::msg::Int32MultiArray calibration_msg;
            calibration_msg.data = calibration_data;
            calibration_publisher_->publish(calibration_msg);

            calibration_data.clear();
            received_33 = false;
            received_34 = false;
            received_35 = false;
        }
    }

    void publishTactData(const struct can_frame &frame) {
            if (frame.can_id == TACT_CAN_ID) {
                std_msgs::msg::Bool bool_msg_1;
                bool_msg_1.data = (frame.data[0] & 0x0F) == 0x01;
                tact_1_pub_->publish(bool_msg_1);

                std_msgs::msg::Bool bool_msg_2;
                bool_msg_2.data = (frame.data[0] & 0x0F) == 0x02;
                tact_2_pub_->publish(bool_msg_2);

                std_msgs::msg::Bool bool_msg_3;
                bool_msg_3.data = (frame.data[0] & 0x0F) == 0x04;
                tact_3_pub_->publish(bool_msg_3);

                std_msgs::msg::Bool bool_msg_4;
                bool_msg_4.data = (frame.data[0] & 0x0F) == 0x08;
                tact_4_pub_->publish(bool_msg_4);
            }
    }
    void vibrator1Callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        data1_ = msg->data;
    }

    void vibrator2Callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        data2_ = msg->data;
    }

    void vibrator3Callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        data3_ = msg->data;
    }

    void sabscribeVibratorData() {
        struct can_frame vibrator_frame;
        std::memset(&vibrator_frame, 0, sizeof(struct can_frame));
        vibrator_frame.can_id = VIBRATOR_CAN_ID;
        vibrator_frame.can_dlc = 3;
        vibrator_frame.data[0] = data1_;
        vibrator_frame.data[1] = data2_;
        vibrator_frame.data[2] = data3_;
        int nbytes = write(can_sock_, &vibrator_frame, sizeof(struct can_frame));
        if (nbytes <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to CAN ID: 0x04");
        } else {
            RCLCPP_INFO(this->get_logger(), "Written data to CAN ID: 0x04");
        }
    }

    void publishTouchData(const struct can_frame &frame) {
        if (frame.can_id == TOUCH_CAN_ID && frame.can_dlc >= 4) {
            int16_t capacitive_touch = frame.data[3];
            std_msgs::msg::Int16 touch_msg;
            touch_msg.data = capacitive_touch;
            capacitive_touch_pub_->publish(touch_msg);
        }
    }




    uint8_t data1_ = 0, data2_ = 0, data3_ = 0;
    std::string wifi_data_[4];
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tact_1_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tact_2_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tact_3_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tact_4_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr capacitive_touch_pub_;
    
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_1_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_2_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vibrator_3_;   

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibration_srv_;
    int can_sock_;
    bool linear_data_received_;
    bool angular_data_received_;
    bool orientation_data_received_;
    bool wifi_received_[4] = {false, false, false, false};
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanToRos2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}