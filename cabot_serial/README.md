# cabot_serial

- python/cpp implementation to communicate with CaBot's microcontroller
    - [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino)
        - Single microcontroller configuration
    - [cabot-arduino-di](https://github.com/CMU-cabot/cabot-arduino-di)
        - Two microcontrollers configuration (Sensor + Handle)

## Node

- cabot_serial_node.py
- cabot_serial_node (cpp implementation)

### Services

- **set_touch_speed_active_mode** (std_srvs::srv::SetBool)
    - change touch speed active mode (default=true) and what **/touch_speed_switched** publishes
    
    |Touch speed active mode|`/touch`=true|`/touch`=false|behavior|
    |---|---|---|---|
    |`true`|`touch_speed_max`|`0`|CaBot can proceed when someone is holding handle|
    |`false`|`0`|`touch_speed_max_inactive`|CaBot can proceed when someone is not holding handle (used in summon mode - CaBot runs by itself)|

### Parameters

- **touch_speed_max** (float) - default=`2.0`
    - `/touch_speed_switched` value when the touch sensor is active
- **touch_speed_max_inactive** (float) - default=`0.5`
    - `/touch_speed_switched` value when the touch sensor is inactive
- **port** (string) - default=`/dev/ttyCABOT`
    - serial port
- **baud** (int) - default=`119200`
    - baud rate
- **run_imu_calibration** (bool) - default=`false`
    - see [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino)
- **calibration_params** (int[22]) - default=`None`
    - see [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino)
- **touch_params** (int[3]) - default=`None`
    - see [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino)

### Publishers

- **/calibration** (int[22]): only publishes values when `run_imu_calibration` is true
- **/diagnostics** (diagnostic_msgs/msg/DiagnosticArray): diagnostics
- **/imu** (sensor_msgs/msg/Imu): IMU data - 100Hz
- **/pressure** (sensor_msgs/msg/FluidPressure): Pressure data - 2Hz
- **/pushed** (std_msgs/msg/Int8): button status (bit0-4 represents each button) - 50Hz
- **/pushed_1**, **/pushed_2**, **/pushed_3**, **/pushed_4**, **/pushed_5** (std_msg/msg/Bool): publishes each button status - 50Hz
- **/temparature** (sensor_msgs/msg/Temperature): Temperature data - 2Hz
- **/touch** (std_msgs/msg/Int16): `1` if touched else `0` - 50Hz
- **/touch_raw** (std_msgs/msg/Int16): touch sensor's raw value (for debug) - 50Hz
- **/touch_speed_switched** (float): speed control by handle - see above - 50Hz
- **/wifi** (std_msgs/msg/String): wifi scan string - depends on how often WiFi signal is scanned (only with ESP32 microcontroller)
- **/servo_pos** (std_msgs/msg/Int16): current servo position - 50Hz

### Subscribers

- **/servo_target** (std_msgs/msg/Int16): servo position (-90 ~ +90)
- **/servo_free** (std_msgs/msg/Bool): `true` -> set free, `false` -> no operation
- **/vibrator1** (std_msgs/msg/UInt8): front
- **/vibrator2** (std_msgs/msg/UInt8): back (not used)
- **/vibrator3** (std_msgs/msg/UInt8): left
- **/vibrator4** (std_msgs/msg/UInt8): right
    - if you publish a `value`, it vibrates corresponding vibrator for `value` x 10 milliseconds (0 - 2550 milliseconds)

## Test

- launch cabot_serial_node
```
ros2 run cabot_serial cabot_serial_node --ros-args -p port:=<port>
```

- check if all diagnostic messages are OK (Serial Connection, IMU, Push Button, Touch sensor, Pressure, Temperature)
```
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

- check if the handle vibrates
```
ros2 topic pub -1 /vibrator1 std_msgs/msg/UInt8 "data: 10"
ros2 topic pub -1 /vibrator3 std_msgs/msg/UInt8 "data: 10"
ros2 topic pub -1 /vibrator4 std_msgs/msg/UInt8 "data: 10"
```

- check if the servo (direction indicator) rotates
```
ros2 topic pub -1 /servo_target std_msgs/msg/Int16 "data: 90"
ros2 topic pub -1 /servo_target std_msgs/msg/Int16 "data: -90"
ros2 topic pub -1 /servo_target std_msgs/msg/Int16 "data: 0"
```
