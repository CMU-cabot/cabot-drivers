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

- run the script to download dependencies

```
./setup-dependency.sh
```

- if you pull the latest docker images from docker hub, run the following command

```
docker compose --profile build pull
```

- if you build docker image, run the script to build image

```
./bake-docker.sh -i         # run docker image build for your platform
```

- if you run in development mode, run the script to build workspaces

```
./build-workspace.sh        # run workspace build
./build-workspace.sh -d     # run workspace debug build (symlink-install)
```

- run tests by following steps

```
# if you run docker container in production mode
docker compose run --rm driver-prod bash
# if you run docker container in development mode
docker compose run --rm driver-dev bash

# inside docker
source install/setup.bash
# run node for test
```

## Topic/Service requirements works with cabot-navigation

### Required services
- /cabot/set_touch_speed_active_mode  (std_srvs/srv/SetBool, to control touch active mode)

Touch speed active mode | Touching | Not Touching
--- | --- | ---
True  |  robot can move | robot cannot move
False | robot cannot move | robot can move


### Required topics
- /cabot/event                 (std_msgs/msg/String, button events, needs to control cabot_ui)
- /cabot/odom_raw              (nav_msgs/msg/Odometry, raw odometry)
- /cabot/imu/data              (sensor_msgs/msg/Imu, IMU)
- /cabot/touch_speed_switched  (std_msgs/msg/Float32, touch speed control)
- /velodyne_points             (sensor_msgs/msg/Point2, expects this name even if you use different LiDAR from velodyne)
- /joint_states                (sensor_msgs/msg/JointState, control shift)
- /tf                          (tf2_msgs/msg/TFMessage)
- /tf_static                   (tf2_msgs/msg/TFMessage)

### Optional topics
- /cabot/pressure              (sensor_msgs/msg/FluidPressure, better localization for multi story building)

### Subscribers

- /cabot/cmd_vel               (geometry_msgs/msg/Twist, motor control)
- /cabot/notification          (std_msgs/msg/Int8, handle vibration)
- /cabot/pause_control         (std_msgs/msg/Bool, optional, turn off motor loop contro)

# License

[MIT License](LICENSE)
