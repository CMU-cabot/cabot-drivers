# CaBot Drivers

Hardware interface packages for CaBot

## Packges

|Package|Description|
|---|---|
|[cabot_base](./cabot_base)|launch configurations to bringup cabot hardware drivers|
|[cabot_serial](./cabot_serial)|node to communicate with Arduino/ESP32 microcontroller|
|[motor_controller](./motor_controller)|motor driver and adapter|
|[wireless_scanner_ros](./wireless_scanner_ros)|WiFi/BLE scanner for ROS|

## Docker environment for test

```
docker compose build
docker compose run --rm ros2 bash
# inside docker
colcon build
source install/setup.bash
# run node for test
```

# License

[MIT License](LICENSE)
