# cabot-shared-control

ODrive S1 x2 + BotWheels 2輪差動向けの shared control 実装です。  
`ros_odrive` (`odrive_can`) のCANトピックを使い、以下を行います。

- モーター速度/電流(または推定トルク)から外力を推定
- IMU姿勢から坂道重力成分を補償
- 推定外力をコンプライアンス制御で速度指令へ変換
- 必要に応じて自律側 `TwistStamped` と合成（shared control）

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

## 想定トピック

- 入力:
  - `/odrive_axis0/controller_status` (`odrive_can/msg/ControllerStatus`)
  - `/odrive_axis1/controller_status` (`odrive_can/msg/ControllerStatus`)
  - `/imu/data` (`sensor_msgs/msg/Imu`)
  - `/autonomy/cmd_vel` (`geometry_msgs/msg/TwistStamped`, 任意)
- 出力:
  - `/odrive_axis0/control_message` (`odrive_can/msg/ControlMessage`)
  - `/odrive_axis1/control_message` (`odrive_can/msg/ControlMessage`)
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

## セットアップ注意

- `odrive_can` 側で cyclic message を有効化してください（特に `heartbeat`, `encoder`, `iq`, `torques`）。
- 軸状態は `CLOSED_LOOP_CONTROL` である必要があります。本ノードは起動時に `/request_axis_state` を呼び出す設定が可能です。
- `wheel_radius_m`, `wheel_separation_m`, 符号パラメータは実機に合わせて必ず調整してください。
