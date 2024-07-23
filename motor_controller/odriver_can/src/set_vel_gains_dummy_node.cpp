#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

class CanSenderDummyNode : public rclcpp::Node {
public:
    CanSenderDummyNode() : Node("can_sender_node") {
        std::string vel_gain_param_name = "vel_gain";
        this->declare_parameter(vel_gain_param_name,10.0);
        vel_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto vel_gain_cb_ = [this](const rclcpp::Parameter & p) {
            // do nothing
        };
        vel_gain_cb_handle_ = 
            vel_gain_subscriber_->add_parameter_callback(
                vel_gain_param_name,
                vel_gain_cb_
        );

        std::string vel_integrator_gain_param_name = "vel_integrator_gain";
        this->declare_parameter(vel_integrator_gain_param_name,40.0);
        vel_integrator_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto vel_integrator_gain_cb_ = [this](const rclcpp::Parameter & p) {
            // do nothing
        };
        vel_integrator_gain_cb_handle_ =
            vel_integrator_gain_subscriber_->add_parameter_callback(
                vel_integrator_gain_param_name,
                vel_integrator_gain_cb_
        );
    }

private:
    std::shared_ptr<rclcpp::ParameterEventHandler> vel_gain_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> vel_gain_cb_handle_;
    std::shared_ptr<rclcpp::ParameterEventHandler> vel_integrator_gain_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> vel_integrator_gain_cb_handle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanSenderDummyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
