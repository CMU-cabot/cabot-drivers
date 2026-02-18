# cabot-shared-control

ODrive S1 x2 + BotWheels 2輪差動向けの shared control 実装です。  
`ros_odrive` (`odrive_can`) のCANトピックを使い、以下を行います。

- モーター速度/電流(または推定トルク)から外力を推定
- IMU姿勢から坂道重力成分を補償
- 推定外力をコンプライアンス制御で速度指令へ変換
- 必要に応じて自律側 `TwistStamped` と合成（shared control）
- LiDAR点群 + footprintから障害物接近時の前後進ブロック（安全ガード）

実装言語は C++ (`rclcpp`) です。

ベースとなる制御構造は以下の論文です（4輪オムニの式を2輪差動へ縮約して実装）:

- `A_new_compliant_motion_control_design_of_a_walking-help_robot_based_on_motor_current_and_speed_measurement.pdf`

## 依存

- ROS 2 Humble
- `odrive_can` パッケージ（`ros_odrive` リポジトリ）
- `sensor_msgs/Imu`

## ビルド

```bash
colcon build --symlink-install --packages-select cabot_shared_control
source install/setup.bash
```

## 起動

```bash
ros2 launch cabot_shared_control shared_control.launch.py
```

`shared_control.launch.py` は以下を起動します（`cabot3-k4` 前提）:

- `robot_state_publisher`
- `local_robot_state_publisher`
- `hesai_lidar`（または `hesai_ros_driver`）
- `filter_crop_box_node`（`/velodyne_points` -> `/velodyne_points_cropped`）
- `odrive_can_node_left` (`node_id=0`)
- `odrive_can_node_right` (`node_id=1`)
- `shared_control_node`

外部の速度指令（`/autonomy/cmd_vel`）は不要です。デフォルトで `autonomy_force_weight=0.0` を適用します。

必要に応じて以下を指定できます:

```bash
ros2 launch cabot_shared_control shared_control.launch.py \
  model:=cabot3-k4 \
  use_robot_state_publisher:=true \
  use_hesai_lidar:=true \
  use_crop_box:=true \
  can_interface:=can1 \
  hesai_ros_2_0:=false \
  imu_topic:=/cabot/imu/data \
  pointcloud_topic:=/velodyne_points_cropped \
  footprint_topic:=/footprint
```

`odrive_model` と `odrive_firmware_version` は `ODRIVE_MODEL` / `ODRIVE_FIRMWARE_VERSION` 環境変数から参照します。
IMUの使用有無は `CABOT_SHARED_CONTROL_USE_IMU` で切り替えできます（`false` でIMU無効、水平面前提）。

## 想定トピック

- 入力:
  - `/cabot/controller_status_left` (`odrive_can/msg/ControllerStatus`)
  - `/cabot/controller_status_right` (`odrive_can/msg/ControllerStatus`)
  - `/cabot/imu/data` (`sensor_msgs/msg/Imu`)
  - `/autonomy/cmd_vel` (`geometry_msgs/msg/TwistStamped`, 任意)
  - `/velodyne_points_cropped` (`sensor_msgs/msg/PointCloud2`)
  - `/footprint` (`geometry_msgs/msg/PolygonStamped`)
- 出力:
  - `/cabot/control_message_left` (`odrive_can/msg/ControlMessage`)
  - `/cabot/control_message_right` (`odrive_can/msg/ControlMessage`)
  - `/shared_control/external_wrench` (`geometry_msgs/msg/WrenchStamped`)
  - `/shared_control/cmd_vel` (`geometry_msgs/msg/TwistStamped`)

## 重要パラメータ（`config/shared_control.yaml`）

- 幾何:
  - `wheel_radius_m`
  - `wheel_separation_m`
  - `left_wheel_sign`, `right_wheel_sign`
- オブザーバ:
  - `observer_gain_x`, `observer_gain_z`
  - `robot_mass_x`, `robot_inertia_z`
  - `robot_damping_x`, `robot_damping_z`
  - `coulomb_friction_x`, `coulomb_friction_z`
- コンプライアンス:
  - `desired_mass_x`, `desired_inertia_z`
  - `desired_damping_x`, `desired_damping_z`
- shared control:
  - `human_force_weight`
  - `autonomy_force_weight`
  - `autonomy_virtual_stiffness_x`, `autonomy_virtual_stiffness_z`
- 障害物ガード:
  - `obstacle_guard_enabled`
  - `obstacle_stop_distance_m` (初期値 0.5)
  - `pointcloud_topic`, `footprint_topic`
  - `obstacle_point_min_z`, `obstacle_point_max_z`
  - `strict_frame_match`

## セットアップ注意

- `odrive_can` 側で cyclic message を有効化してください（特に `heartbeat`, `encoder`, `iq`, `torques`）。
- 軸状態は `CLOSED_LOOP_CONTROL` である必要があります。本ノードは起動時に `/request_axis_state` を呼び出す設定が可能です。
- `wheel_radius_m`, `wheel_separation_m`, 符号パラメータは実機に合わせて必ず調整してください。
- 障害物ガードは `PointCloud2` と `footprint` が同一フレームであることを前提にしています。
