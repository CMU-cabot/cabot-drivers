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
    CanSenderNode() : Node("can_sender_node"), vel_gain_data_(10.0), vel_integrator_gain_data_(40.0) {
        can_socket_ = openCanSocket();

        subscription_vel_gain_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vel_gain", 10,
            std::bind(&CanSenderNode::velGainCallback, this, std::placeholders::_1));

        subscription_vel_integrator_gain_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vel_integrator_gain", 10,
            std::bind(&CanSenderNode::velIntegratorGainCallback, this, std::placeholders::_1));
    }

private:
    void velGainCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::memcpy(&vel_gain_data_, &msg->data, sizeof(float));
        vel_gain_received_ = true;
        sendCanMessageIfReceived(0x1b); // vel_gainのCAN ID設定
    }

    void velIntegratorGainCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::memcpy(&vel_integrator_gain_data_, &msg->data, sizeof(float));
        vel_integrator_gain_received_ = true;
        sendCanMessageIfReceived(0x1b); //vel_integrator_gainのCAN ID設定
    }

    void sendCanMessageIfReceived(uint8_t can_id) {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;

        std::memcpy(&frame.data[0], &vel_gain_data_, 4);
        std::memcpy(&frame.data[4], &vel_integrator_gain_data_, 4);

        sendCanFrame(can_socket_, frame);

        vel_gain_received_ = false;
        vel_integrator_gain_received_ = false;
    }

    int can_socket_;
    bool vel_gain_received_;
    bool vel_integrator_gain_received_;
    float vel_gain_data_;
    float vel_integrator_gain_data_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_vel_gain_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_vel_integrator_gain_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}