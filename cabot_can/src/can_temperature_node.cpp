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

int open_can_socket(const std::string &can_interface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        std::cerr << "Error while opening socket" << std::endl;
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
        std::cerr << "Error in socket bind" << std::endl;
        return -2;
    }

    return sock;
}

bool receive_can_data(int sock, struct can_frame &frame) {
    int nbytes = read(sock, &frame, sizeof(struct can_frame));
    return nbytes > 0;
}

class CanTemperaturePublisher : public rclcpp::Node {
public:
    CanTemperaturePublisher()
        : Node("can_temperature_publisher"),
          can_sock_(open_can_socket("can1"))    {
        temperature_pub_1_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature1", 2);
        temperature_pub_2_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature2", 2);
        temperature_pub_3_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature3", 2);
        temperature_pub_4_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature4", 2);
        temperature_pub_5_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature5", 2);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CanTemperaturePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        struct can_frame frame;
        if (receive_can_data(can_sock_, frame)) {
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