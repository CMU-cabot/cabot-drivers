#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <vector>
#include <iostream>

class CanSenderNode : public rclcpp::Node {
public:
    CanSenderNode() : Node("set_vel_gains_pro_node"), vel_gain_data_(0.16), vel_integrator_gain_data_(0.33) {
        this->declare_parameter<float>("vel_gain", vel_gain_data_);
        this->declare_parameter<float>("vel_integrator_gain", vel_integrator_gain_data_);

        can_socket_ = openCanSocket();

        // 初期値設定
        sendCanMessage(0x1b);
        sendCanMessageVelGain(0x004, {0x00, 0x91, 0x01, 0x00});
        sendCanMessageVelIntegratorGain(0x004, {0x00, 0x92, 0x01, 0x00});
        this->get_parameter("vel_gain", vel_gain_data_);
        this->get_parameter("vel_integrator_gain", vel_integrator_gain_data_);

        // パラメーターセット
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
                auto result = rcl_interfaces::msg::SetParametersResult();
                result.successful = true;

                for (const auto &param : params) {
                    if (param.get_name() == "vel_gain") {
                        vel_gain_data_ = param.as_double();
                        sendCanMessage(0x1b);
                        sendCanMessageVelGain(0x004, {0x00, 0x91, 0x01, 0x00});
                    } else if (param.get_name() == "vel_integrator_gain") {
                        vel_integrator_gain_data_ = param.as_double();
                        sendCanMessage(0x1b);
                        sendCanMessageVelIntegratorGain(0x004, {0x00, 0x92, 0x01, 0x00});
                    }
                }
                return result;
            }
        );
    }

private:
    int openCanSocket() {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) return -1;

        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) return -1;

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(s);
            return -1;
        }
        return s;
    }

    void sendCanMessage(uint8_t can_id) {
        can_frame frame = {};
        frame.can_id = can_id;
        frame.can_dlc = 8;
        std::memcpy(&frame.data[0], &vel_gain_data_, 4);
        std::memcpy(&frame.data[4], &vel_integrator_gain_data_, 4);
        // すべてのデータ出力
        std::cout << std::hex << "all data: ";
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::dec << std::endl;
        write(can_socket_, &frame, sizeof(frame));
    }

    void sendCanMessageVelGain(uint8_t can_id, const std::vector<uint8_t>& data) {
        can_frame frame = {};
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        std::memcpy(&frame.data[0], data.data(), data.size());
        write(can_socket_, &frame, sizeof(frame));
        receiveCanMessageVelGain(0x05);
    }

    void sendCanMessageVelIntegratorGain(uint8_t can_id, const std::vector<uint8_t>& data) {
        can_frame frame = {};
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        std::memcpy(&frame.data[0], data.data(), data.size());
        write(can_socket_, &frame, sizeof(frame));
        receiveCanMessageVelIntegratorGain(0x05);
    }

    void receiveCanMessageVelGain(uint8_t can_id) {
        can_frame frame = {};
        bool match = true;

        while (true) {
            int nbytes = read(can_socket_, &frame, sizeof(frame));
            if (nbytes > 0 && frame.can_id == can_id) {
                uint8_t sent_data[8];
                std::memcpy(&sent_data[0], &vel_gain_data_, 4);
                std::memcpy(&sent_data[4], &vel_integrator_gain_data_, 4);
                for (int i = 0; i < 4; ++i) {
                    if (frame.data[i + 4] != sent_data[i]) {
                        match = false;
                        break;
                    }
                }
                std::cout << std::hex << "data1: ";
                for (int i = 4; i < 8; ++i) {
                    std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
                }
                std::cout << std::dec << std::endl;
                std::cout << (match ? "true" : "false") << std::endl;

                if (!match) {
                    sendCanMessage(0x1b);
                    sendCanMessageVelGain(0x004, {0x00, 0x91, 0x01, 0x00});
                }
                break;
            }
        }
    }

    void receiveCanMessageVelIntegratorGain(uint8_t can_id) {
        can_frame frame = {};
        bool match = true;

        while (true) {
            int nbytes = read(can_socket_, &frame, sizeof(frame));
            if (nbytes > 0 && frame.can_id == can_id) {
                uint8_t sent_data[8];
                std::memcpy(&sent_data[0], &vel_gain_data_, 4);
                std::memcpy(&sent_data[4], &vel_integrator_gain_data_, 4);
                std::cout << std::hex << "data2: ";
                for (int i = 4; i < 8; ++i) {
                    std::cout << static_cast<int>(frame.data[i]) << " ";
                    if (frame.data[i] != sent_data[i]) {
                        match = false;
                    }
                }
                std::cout << std::dec << std::endl;
                std::cout << (match ? "true" : "false") << std::endl;

                if (!match) {
                    sendCanMessage(0x1b);
                    sendCanMessageVelIntegratorGain(0x004, {0x00, 0x92, 0x01, 0x00});
                }
                break;
            }
        }
    }

    int can_socket_;
    float vel_gain_data_;
    float vel_integrator_gain_data_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanSenderNode>());
    rclcpp::shutdown();
    return 0;
}
