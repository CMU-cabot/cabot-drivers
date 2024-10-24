# odriver_can_adapter
## Node

- odriver_can_adapter_node
- depends on [odrive_can](https://github.com/CMU-cabot/ros_odrive) and [odriver_adapter_node](https://github.com/CMU-cabot/cabot-drivers/tree/main/motor_controller/motor_adapter)

### Parameters

- **wheel_diameter_m** (double) - default=`0.17`
    - wheel diameter
- **is_clockwise** (bool) - default=`true`
    - true: the right wheel rotates forward and the left wheel rotates backward
- **hz** (double) - default=`20.0`
    - update frequency of the timer callback (related to update frequency of `/motor_status`)
- **service_timeout_ms** (int) - default=`2000`
    - timeout for the return of the `/request_axis_state` call

### Publishers

- **/control_message_left** (odrive_can/msg/ControlMessage): left wheel control message
- **/control_message_right** (odrive_can/msg/ControlMessage): right wheel control message
- **/motor_status** (odriver_msgs/msg/MotorStatus): motor status for odriver_adapter_node

### Subscribers
- **/controller_status_left** (odrive_can/msg/ControllerStatus): left wheel controller status
- **/controller_status_right** (odrive_can/msg/ControllerStatus): right wheel controller status
- **/motor_target** (odriver_msgs/msg/MotorTarget): motor target from odriver_adapter_node

## Test

- launch test
```
ros2 run odriver_can_adapter odriver_can_adapter_node
```
> [!NOTE]
> To confirm that ODrive can be used with a CAN connection <br>
> support page: https://docs.odriverobotics.com/v/latest/guides/can-guide.html

- check wheel rotation
```
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
> [!TIP]
> when using the botwheel, it rotates once per second.