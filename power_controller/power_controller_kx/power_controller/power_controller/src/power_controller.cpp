//include header file
#include <cstdio>
#include <regex>
#include <chrono>

// include ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

// include custom msg
#include "power_controller_msgs/msg/battery_array.hpp"
#include "power_controller_msgs/srv/fan_controller.hpp"

class PowerController : public rclcpp::Node{
public:
  PowerController()
  : Node("power_controller_node")
  {
    using namespace std::chrono_literals;
    this->declare_parameter<int>("number_of_batterys", 4);
    service_server_24v_Odrive =  this->create_service<std_srvs::srv::SetBool>("set_24v_power_Odrive",
      std::bind(&PowerController::Set24vPower,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_12v_PC =  this->create_service<std_srvs::srv::SetBool>("set_12v_power_PC",
      std::bind(&PowerController::Set12vPowerPC,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_12v_D455_1 =  this->create_service<std_srvs::srv::SetBool>("set_12v_power_D455_1",
      std::bind(&PowerController::Set12vPowerD4551,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_12v_D455_2 =  this->create_service<std_srvs::srv::SetBool>("set_12v_power_D455_2",
      std::bind(&PowerController::Set12vPowerD4552,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_12v_D455_3 =  this->create_service<std_srvs::srv::SetBool>("set_12v_power_D455_3",
      std::bind(&PowerController::Set12vPowerD4553,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_5v_MCU =  this->create_service<std_srvs::srv::SetBool>("set_5v_power_MCU",
      std::bind(&PowerController::Set5vPower,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_shutdown =  this->create_service<std_srvs::srv::Empty>("shutdown",
      std::bind(&PowerController::Shutdown,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_fan_pwm =  this->create_service<power_controller_msgs::srv::FanController>("fan_control",
      std::bind(&PowerController::FanControl,
      this, std::placeholders::_1,
      std::placeholders::_2));
    // publisher
    publisher_ = this->create_publisher<power_controller_msgs::msg::BatteryArray>("battery_state",10);
    timer_ = this->create_wall_timer(1s, std::bind(&PowerController::PublisherPowerStatus, this));
  }

private:
  void Set24vPower(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "turn on 24V_Odrive");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "turn off 24V_Odrive");
    }
    res->success = true;
  }
  
  void Set12vPowerPC(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "turn on 12V PC");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "turn off 12V PC");
    }
    res->success = true;
  }
  void Set12vPowerD4551(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "turn on 12V D455_1");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "turn off 12V D455_1");
    }
    res->success = true;
  }
  void Set12vPowerD4552(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "turn on 12V D455_2");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "turn off 12V D455_2");
    }
    res->success = true;
  }
  void Set12vPowerD4553(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "turn on 12V D455_3");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "turn off 12V D455_3");
    }
    res->success = true;
  }
  void Set5vPower(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		  const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "turn on 5V MCU");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "turn off 5V MCU");
    }
    res->success = true;
  }
  void Shutdown(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
		const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void)req;
    (void)res;
    RCLCPP_INFO(this->get_logger(), "Shutdown");
  }
  void FanControl(const std::shared_ptr<power_controller_msgs::srv::FanController::Request> req,
		  const std::shared_ptr<power_controller_msgs::srv::FanController::Response> res)
  {
    int pwm = req->data;
    if (pwm < 101)
    {
      RCLCPP_INFO(this->get_logger(), "pwm is %d", pwm);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Only values from 0 to 100 can be entered");
    }
    res->response = true;
  }

  void PublisherPowerStatus()
  {
    int num_batterys = this->get_parameter("number_of_batterys").as_int();
    // setting loop rate
    rclcpp::WallRate loop(1);
    // define publish msg
    power_controller_msgs::msg::BatteryArray msg;
    // define msg's value
    msg.batteryarray.resize(num_batterys);
    for (int i = 0; i < num_batterys; i++)
    {
      msg.batteryarray[i].header.stamp = this->get_clock()->now();
      msg.batteryarray[i].voltage = 24.0 - float(i)/100.0;
      msg.batteryarray[i].current = i + 0.2;
      msg.batteryarray[i].percentage = float(i) / 10;
      msg.batteryarray[i].serial_number = std::to_string(i + 1);
      msg.batteryarray[i].temperature = i + 20.5;
    }
    // publsih msg
    publisher_->publish(msg);
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_24v_Odrive;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_PC;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_1;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_2;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_3;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_5v_MCU;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_shutdown;
  rclcpp::Service<power_controller_msgs::srv::FanController>::SharedPtr service_server_fan_pwm;
  rclcpp::Publisher<power_controller_msgs::msg::BatteryArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PowerController>());
  rclcpp::shutdown();
  return 0;
}

/*output
---
batteryarray:
- battery:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    voltage: 10.0
    temperature: 0.0
    current: 0.20000000298023224
    charge: 0.0
    capacity: 0.0
    design_capacity: 0.0
    percentage: 0.0
    power_supply_status: 0
    power_supply_health: 0
    power_supply_technology: 0
    present: false
    cell_voltage: []
    cell_temperature: []
    location: ''
    serial_number: '1'
  temperature:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    temperature: 20.5
    variance: 0.0
- battery:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    voltage: 11.0
    temperature: 0.0
    current: 1.2000000476837158
    charge: 0.0
    capacity: 0.0
    design_capacity: 0.0
    percentage: 0.10000000149011612
    power_supply_status: 0
    power_supply_health: 0
    power_supply_technology: 0
    present: false
    cell_voltage: []
    cell_temperature: []
    location: ''
    serial_number: '2'
  temperature:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    temperature: 21.5
    variance: 0.0
- battery:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    voltage: 12.0
    temperature: 0.0
    current: 2.200000047683716
    charge: 0.0
    capacity: 0.0
    design_capacity: 0.0
    percentage: 0.20000000298023224
    power_supply_status: 0
    power_supply_health: 0
    power_supply_technology: 0
    present: false
    cell_voltage: []
    cell_temperature: []
    location: ''
    serial_number: '3'
  temperature:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    temperature: 22.5
    variance: 0.0
---*/
