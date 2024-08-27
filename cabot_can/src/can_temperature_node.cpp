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
#include "sensor_msgs/msg/temperature.hpp"
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>

const int TEMPERATURE_CAN_ID_1 = 0x21;
const int TEMPERATURE_CAN_ID_2 = 0x22;
const int TEMPERATURE_CAN_ID_3 = 0x23;
const int TEMPERATURE_CAN_ID_4 = 0x24;
const int TEMPERATURE_CAN_ID_5 = 0x25;

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

bool receiveCanData(int sock, struct can_frame &frame) {
    int nbytes = read(sock, &frame, sizeof(struct can_frame));
    return nbytes > 0;
}

class CanTemperaturePublisher : public rclcpp::Node {
public:
    CanTemperaturePublisher()
        : Node("can_temperature_publisher"),
          can_sock_(openCanSocket("can1"))    {
        temperature_pub_1_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature1", 2);
        temperature_pub_2_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature2", 2);
        temperature_pub_3_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature3", 2);
        temperature_pub_4_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature4", 2);
        temperature_pub_5_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature5", 2);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&CanTemperaturePublisher::timerCallback, this));
    }

private:
    void timerCallback() {
        struct can_frame frame;
        if (receiveCanData(can_sock_, frame)) {
            int16_t temperature_raw = (frame.data[1] << 8) | frame.data[0];
            float temperature = static_cast<float>(temperature_raw) / 16.0;

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
    }

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_1_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_2_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_3_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_4_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_5_;

    int can_sock_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanTemperaturePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
