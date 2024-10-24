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

#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>

int openCanSocket() {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket");
        return -1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        close(s);
        return -1;
    }

    return s;
}

void sendCanFrame(int can_socket, can_frame &frame) {
    int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        perror("Send");
    }
}

class CanSenderNode : public rclcpp::Node {
public:
    CanSenderNode() : Node("can_sender_node") {
        can_socket_ = openCanSocket();

        std::string motor_pi_gain_param_name = "motor_pi_gain";
        this->declare_parameter(motor_pi_gain_param_name, std::vector<double>({10.0,40.0}));
        motor_pi_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto motor_pi_gain_cb = [this](const rclcpp::Parameter & p) {
            std::vector<double> motor_pi_gain = p.as_double_array();
            this->vel_gain_data_            = motor_pi_gain[0];
            this->vel_integrator_gain_data_ = motor_pi_gain[1];
            sendCanMessage(0x1b);
        };
        motor_pi_gain_cb_handle_ =
            motor_pi_gain_subscriber_->add_parameter_callback(
                motor_pi_gain_param_name,
                motor_pi_gain_cb
        );
    }

private:
    std::shared_ptr<rclcpp::ParameterEventHandler> motor_pi_gain_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> motor_pi_gain_cb_handle_;
    void sendCanMessage(uint8_t can_id) {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;

        std::memcpy(&frame.data[0], &vel_gain_data_, 4);
        std::memcpy(&frame.data[4], &vel_integrator_gain_data_, 4);

        sendCanFrame(can_socket_, frame);
    }

    int can_socket_;
    float vel_gain_data_;
    float vel_integrator_gain_data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
