# powe_controller-kx

manage each power supply and output battery information via topic communication

## Node
  - power_controller

## Services
- **set_24v_power_odrive** (std_srvs::srv::SetBool)
    - managing odrive's power supply
- **set_12v_power_d455_front** (std_srvs::srv::SetBool)
    - managing framos front power supply
- **set_12v_power_d455_right** (std_srvs::srv::SetBool)
    - managing framos right power supply
- **set_12v_power_d455_left** (std_srvs::srv::SetBool)
    - managing framos left power supply
- **set_5v_power_mcu** (std_srvs::srv::SetBool)
    - managing mcu power supply
- **reboot** (std_srvs::srv::Empty)
    - managing power supply
- **shutdown** (std_srvs::srv::Empty)
    - managing power supply



    |power|`true`|`false`|behavior|
    |---|---|---|---|
    |set_24v_power_odrive|turn on odrive|turn off odrive|odrive power management|
    |set_12v_power_d455_front|turn on front framos|turn off front framos|front framos power management|
    |set_12v_power_d455_right|turn on right framos|turn off right framos|right framos power management|
    |set_12v_power_d455_left|turn on left framos|turn off left framos|left framos power management|
    |set_5v_power_mcu|turn on mcu|turn off mcu|mcu  power management|
    |reboot|----|----|restart begins when called by client|
    |shutdown|----|----|performs shutdown when called by the client|

# Publisher
 - **/battery_state** (power_controller_msgs::msg::BatteryArray): publish information on multiple batteries

# Subscriber
 - **/fan_controller** (std_msgs::msg::UInt8): fan is regulated using PWM control(0~100)

# Parameters
- **can_interface** (std::string): the interface of CAN can be changed
- **number_of_batteries** (int): number of batteries can be changed