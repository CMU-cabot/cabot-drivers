//include header file
#include <cstdio>
#include <regex>
#include <chrono>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>

// include ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

// include custom msg
#include "power_controller_msgs/msg/battery_array.hpp"
#include "power_controller_msgs/srv/fan_controller.hpp"

// font color
#define ANSI_COLOR_CYAN   "\x1b[36m"

class PowerController : public rclcpp::Node{
public:
  PowerController()
    : Node("power_controller_node"), check_power_(true)
  {
    using namespace std::chrono_literals;
    // number of batterys
    this->declare_parameter<int>("number_of_batterys", 4);
    // service
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
    service_server_reboot =  this->create_service<std_srvs::srv::Empty>("reboot",
      std::bind(&PowerController::Reboot,
      this, std::placeholders::_1,
      std::placeholders::_2));
    service_server_fan_pwm =  this->create_service<power_controller_msgs::srv::FanController>("fan_control",
      std::bind(&PowerController::FanControl,
      this, std::placeholders::_1,
      std::placeholders::_2));
    // publisher
    publisher_ = this->create_publisher<power_controller_msgs::msg::BatteryArray>("battery_state",10);
    timer_ = this->create_wall_timer(1s, std::bind(&PowerController::PublisherPowerStatus, this));
    // open can socket
    can_socket_ = openCanSocket();
  }

private:
  // can
  int openCanSocket()
  {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
      {
	perror("Socket");
	return -1;
      }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      {
	perror("Bind");
	close(s);
	return -1;
      }
    return s;
  }

  void sendCanFrame(int can_socket, can_frame &frame)
  {
    int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame))
      {
	perror("Send");
      }
  }
  
  void Set24vPower(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    uint8_t can_id = 0x14;
    if (req->data == 1)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 24V_Odrive" );
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 24V_Odrive" );
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    res->success = true;
  }
  
  void Set12vPowerPC(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    uint8_t can_id = 0x15;
    if (req->data == 1)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on" );
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off" );
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    res->success = true;
  }
  void Set12vPowerD4551(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    uint8_t can_id = 0x16;
    if (req->data == 1)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_1" );
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_1" );
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    res->success = true;
  }
  void Set12vPowerD4552(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    uint8_t can_id = 0x17;
    if (req->data == 1)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_2" );
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_2" );
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    res->success = true;
  }
  void Set12vPowerD4553(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		   const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    uint8_t can_id = 0x18;
    if (req->data == 1)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 12V D455_3" );
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 12V D455_3" );
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    res->success = true;
  }
  void Set5vPower(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		  const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    uint8_t can_id = 0x19;
    if (req->data == 1)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn on 5V MCU" );
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "turn off 5V MCU" );
    }
    std::memcpy(&check_power_, &req->data, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    res->success = true;
  }
  void Shutdown(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
		const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    uint8_t can_id = 0x15;
    bool shutdown_ = false;
    std::memcpy(&check_power_, &shutdown_, sizeof(bool));
    sendCanMessageIfReceived(can_id);
    (void)req;
    (void)res;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "Shutdown" );
  }
  void Reboot(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
		const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    uint8_t can_id = 0x15;
    int reboot_ = 255;
    std::memcpy(&check_power_, &reboot_, sizeof(int));
    sendCanMessageIfReceived(can_id);
    (void)req;
    (void)res;
    RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "reboot" );
  }
  void FanControl(const std::shared_ptr<power_controller_msgs::srv::FanController::Request> req,
		  const std::shared_ptr<power_controller_msgs::srv::FanController::Response> res)
  {
    int pwm = req->data;
    uint8_t can_id = 0x1d;
    if (pwm < 101)
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "pwm is %d", pwm);
      std::memcpy(&check_power_, &req->data, sizeof(bool));
      sendCanMessageIfReceived(can_id);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), ANSI_COLOR_CYAN "Only values from 0 to 100 can be entered");
    }
    res->response = true;
  }

  void sendCanMessageIfReceived(uint8_t can_id)
  {
    can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 1;
    std::memcpy(&frame.data[0], &check_power_, 1);
    sendCanFrame(can_socket_, frame);
  }

  void PublisherPowerStatus()
  {
    bool confirm_publish = false;
    // setting loop rate
    rclcpp::WallRate loop(1);
    unsigned short data1;
    unsigned short data2;
    unsigned short data3;
    unsigned short data4;
    int num_batterys = this->get_parameter("number_of_batterys").as_int();
    struct can_frame frame;
    while (!confirm_publish)
    {
      int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
      if (nbytes > 0)
      {
	switch (frame.can_id)
	{
          case 0x05:  //Battery 1 Info
	    RCLCPP_INFO(this->get_logger(), "Get data of Battery 1");
	    data1 = (frame.data[1] << 8) | frame.data[0];
	    data2 = (frame.data[3] << 8) | frame.data[2];
	    data3 = (frame.data[5] << 8) | frame.data[4];
	    data4 = (frame.data[7] << 8) | frame.data[6];
	    // define msg's value
	    msg.batteryarray.resize(num_batterys);
	    msg.batteryarray[0].header.stamp = this->get_clock()->now();
	    msg.batteryarray[0].voltage = data1;
	    msg.batteryarray[0].current = data2;
	    msg.batteryarray[0].percentage = data3;
	    msg.batteryarray[0].temperature = data4;
	    msg.batteryarray[0].location = std::to_string(1);
	    break;
	  case 0x06:  //Battery 2 Info
	    RCLCPP_INFO(this->get_logger(), "Get data of Battery 2");
	    data1 = (frame.data[0] << 8) | frame.data[1];
	    data2 = (frame.data[2] << 8) | frame.data[3];
	    data3 = (frame.data[4] << 8) | frame.data[5];
	    data4 = (frame.data[6] << 8) | frame.data[7];
	    // define msg's value
	    msg.batteryarray.resize(num_batterys);
	    msg.batteryarray[1].header.stamp = this->get_clock()->now();
	    msg.batteryarray[1].voltage = data1;
	    msg.batteryarray[1].current = data2;
	    msg.batteryarray[1].percentage = data3;
	    msg.batteryarray[1].temperature = data4;
	    msg.batteryarray[1].location = std::to_string(2);
	    break;
	  case 0x07:  //Battery 3 Info
	    RCLCPP_INFO(this->get_logger(), "Get data of Battery 3");
	    data1 = (frame.data[0] << 8) | frame.data[1];
	    data2 = (frame.data[2] << 8) | frame.data[3];
	    data3 = (frame.data[4] << 8) | frame.data[5];
	    data4 = (frame.data[6] << 8) | frame.data[7];
	    // define msg's value
	    msg.batteryarray.resize(num_batterys);
	    msg.batteryarray[2].header.stamp = this->get_clock()->now();
	    msg.batteryarray[2].voltage = data1;
	    msg.batteryarray[2].current = data2;
	    msg.batteryarray[2].percentage = data3;
	    msg.batteryarray[2].temperature = data4;
	    msg.batteryarray[2].location = std::to_string(3);
	    break;
	  case 0x08:  //Battery 4 Info
	    RCLCPP_INFO(this->get_logger(), "Get data of Battery 4");
	    data1 = (frame.data[0] << 8) | frame.data[1];
	    data2 = (frame.data[2] << 8) | frame.data[3];
	    data3 = (frame.data[4] << 8) | frame.data[5];
	    data4 = (frame.data[6] << 8) | frame.data[7];
	    // define msg's value
	    msg.batteryarray.resize(num_batterys);
	    msg.batteryarray[3].header.stamp = this->get_clock()->now();
	    msg.batteryarray[3].voltage = data1;
	    msg.batteryarray[3].current = data2;
	    msg.batteryarray[3].percentage = data3;
	    msg.batteryarray[3].temperature = data4;
	    msg.batteryarray[3].location = std::to_string(4);
	    break;
	  case 0x1c: //0x1d
	    RCLCPP_INFO(this->get_logger(), "Battery serial number");
	    data1 = (frame.data[0] << 8) | frame.data[1];
	    data2 = (frame.data[2] << 8) | frame.data[3];
	    data3 = (frame.data[4] << 8) | frame.data[5];
	    data4 = (frame.data[6] << 8) | frame.data[7];
	    // define msg's value
	    msg.batteryarray[0].serial_number = std::to_string(data1);
	    msg.batteryarray[1].serial_number = std::to_string(data2);
	    msg.batteryarray[2].serial_number = std::to_string(data3);
	    msg.batteryarray[3].serial_number = std::to_string(data4);
	    // publsih msg
	    publisher_->publish(msg);
	    confirm_publish = true;
	    break;
	}
      }
    }
  }

  // can socket
  int can_socket_;
  bool check_power_;
  
  // define publish msg
  power_controller_msgs::msg::BatteryArray msg;

  // service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_24v_Odrive;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_PC;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_1;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_2;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_12v_D455_3;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_5v_MCU;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_shutdown;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server_reboot;
  rclcpp::Service<power_controller_msgs::srv::FanController>::SharedPtr service_server_fan_pwm;
  // publisher
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
