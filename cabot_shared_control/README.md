# cabot-shared-control

ODrive S1 x2 + BotWheels 2輪差動向けの shared control 実装です。  
`ros_odrive` (`odrive_can`) のCANトピックを使い、以下を行います。

- モーター速度/電流(または推定トルク)から外力を推定
- IMU姿勢から坂道重力成分を補償
- 推定外力をコンプライアンス制御で速度指令へ変換
- 必要に応じて自律側 `TwistStamped` と合成（shared control）
- LiDAR (`LaserScan`) + footprintから障害物接近時の前後進ブロック（安全ガード）

実装言語は C++ (`rclcpp`) です。

ベースとなる制御構造は以下の論文です（4輪オムニの式を2輪差動へ縮約して実装）:

- `A_new_compliant_motion_control_design_of_a_walking-help_robot_based_on_motor_current_and_speed_measurement.pdf`

## shared control モードに関する注意点など (as of 2026-03-05)

- 触らなければ動かない（はず）　ー　状況によって動く可能性があるので注意（床の状態、傾斜等）
  - 傾斜に対するIMUによる補正は入れていますが、テストが出来ていない状態です。可能な限り平らな床でテストしてください。平らでない場所でテストする場合には十分に注意してください。
- 少し押す
  - 速度ゼロから押すことで速度が少しあがります。
  - 上がった速度は維持しようとするので、押し続けなくても動きます。
  - 押し続けると最高速度まで加速します。
- 少し引く
  - 現在の速度から少しずつ減速します。
  - 止まっても引くと後退します。
  - 後退は前進よりも速度が抑えられていますが、注意してください。
  - 後退しているのを止めるときは前に押してください。
- 障害物
  - 障害物に近づくと減速し、一定距離以上は近づかないように制御します


## 依存

- ROS 2 Humble
- `odrive_can` パッケージ（`ros_odrive` リポジトリ）
- `sensor_msgs/Imu`

## ビルド

```bash
colcon build --symlink-install --packages-select cabot_shared_control
source install/setup.bash
```

## odriver_adapter 互換性テスト

`odriver_adapter` 構成で収録した実機 bag から、`shared_control_node` の比較に必要な最小トピックを抜き出した回帰テスト用 bag を `test/bags/odriver_adapter_reference` に配置しています。

- 入力トピック:
  - `/cabot/cmd_vel`
  - `/cabot/controller_status_left`
  - `/cabot/controller_status_right`
- 参照出力トピック:
  - `/cabot/control_message_left`
  - `/cabot/control_message_right`
  - `/cabot/odom_raw`

抽出済み bag の再生成は `test/extract_odriver_adapter_test_bag.py` を使います。

```bash
python3 cabot_shared_control/test/extract_odriver_adapter_test_bag.py \
  --source-bag /path/to/ros2_topics \
  --output-dir /path/to/cabot_shared_control/test/bags/odriver_adapter_reference \
  --start-ns 1770600630000000000 \
  --duration-sec 120
```

テストは `shared_control_node` を normal モードで起動し、bag 入力に対する
`/cabot/control_message_{left,right}` と `/cabot/odom_raw` を参照 bag と比較します。

## 起動

```bash
ros2 launch cabot_shared_control shared_control.launch.py
```

リポジトリのトップディレクトリから docker 内で起動する場合:

```bash
./shared-control-launch.sh        # driver サービス
./shared-control-launch.sh -d     # driver-dev サービス
./shared-control-launch.sh -r     # ODrive 24V を再投入してから起動
```

`shared_control.launch.py` は以下を起動します（`cabot3-k4` 前提）:

- `robot_state_publisher`
- `local_robot_state_publisher`
- `hesai_lidar`（または `hesai_ros_driver`）
- `filter_crop_box_node`（`/velodyne_points` -> `/velodyne_points_cropped`）
- `odrive_can_node_left` (`node_id=0`)
- `odrive_can_node_right` (`node_id=1`)
- `shared_control_node`

外部の速度指令（`/autonomy/cmd_vel`）は必須ではありません。`shared_control.launch.py` では `autonomy_force_weight=0.0` を固定で渡すため、デフォルトでは人の操作のみで shared control を行います。

必要に応じて以下を指定できます:

```bash
ros2 launch cabot_shared_control shared_control.launch.py \
  model:=cabot3-k4 \
  use_robot_state_publisher:=true \
  use_hesai_lidar:=true \
  use_crop_box:=true \
  can_interface:=can1 \
  hesai_ros_2_0:=false \
  use_imu:=true \
  imu_topic:=/cabot/imu/data \
  shared_control_mode_topic:=/shared_control_mode \
  scan_topic:=/scan \
  footprint_topic:=/footprint
```

`odrive_model` と `odrive_firmware_version` は `ODRIVE_MODEL` / `ODRIVE_FIRMWARE_VERSION` 環境変数から参照します。
IMUの使用有無は `CABOT_SHARED_CONTROL_USE_IMU` でも切り替えできます（`false` でIMU無効、水平面前提）。
`shared-control-launch.sh` は `ROS_LOG_DIR` を作成し、必要トピックの rosbag を同時記録します（`docker/home/.ros/log/latest_shared_control`）。
`shared-control-launch.sh` は起動時に `/cabot/odrive_status_left` と `/cabot/odrive_status_right` を確認し、受信できない場合は ODrive 電源OFFとみなしてエラー終了します。

## パラメーターの優先順位

`shared_control_node` の実行時パラメーターは次の優先順位で決まります。

1. `shared_control.launch.py` が `Node(parameters=[...])` で直接渡す値
2. `config/shared_control.yaml`
3. `shared_control_node.cpp` の `declare_parameter()` に書かれた C++ デフォルト値

つまり、README に記載する C++ デフォルト値は「パラメーター未指定時の最終フォールバック」です。`shared_control.launch.py` で起動する場合は、`shared_control.yaml` と launch 引数の値が優先されます。

## `shared_control.launch.py` の引数

| 引数 | デフォルト値 | 説明 |
| --- | --- | --- |
| `model` | `CABOT_MODEL` 環境変数。未設定時は `cabot3-k4` | 読み込む CaBot モデル名。 |
| `use_sim_time` | `false` | シミュレーション時刻を使うかどうか。 |
| `can_interface` | `can1` | ODrive S1 通信に使う CAN インターフェース。 |
| `use_robot_state_publisher` | `true` | `robot_state_publisher` と `local_robot_state_publisher` を起動するかどうか。 |
| `use_hesai_lidar` | `true` | Hesai LiDAR 用 launch を起動するかどうか。 |
| `use_crop_box` | `true` | `CropBox` と `pointcloud_to_laserscan` を使って `/scan` を生成するかどうか。 |
| `odrive_model` | `ODRIVE_MODEL` 環境変数 | 読み込む ODrive endpoint JSON のモデル名。 |
| `odrive_firmware_version` | `ODRIVE_FIRMWARE_VERSION` 環境変数 | 読み込む ODrive endpoint JSON のファームウェア版。 |
| `hesai_ros_2_0` | `HESAI_ROS_2_0` 環境変数。未設定時は `false` | `true` のとき `HesaiLidar_ROS_2.0` 構成を使います。 |
| `use_imu` | `CABOT_SHARED_CONTROL_USE_IMU` 環境変数。未設定時は `true` | `shared_control_node` の IMU 入力を有効化します。 |
| `shared_control_mode_topic` | `CABOT_SHARED_CONTROL_MODE_TOPIC` 環境変数。未設定時は `/shared_control_mode` | `shared_control_node` の `/shared_control_mode` 入力を remap するためのトピック名です。 |
| `imu_topic` | `/cabot/imu/data` | `shared_control_node` の `/imu/data` 入力を remap するための IMU トピック名です。 |
| `scan_topic` | `/scan` | `shared_control_node` の `/scan` 入力を remap するための `sensor_msgs/msg/LaserScan` トピック名です。 |
| `footprint_topic` | `/footprint` | `shared_control_node` の `/footprint` 入力を remap するための `geometry_msgs/msg/Polygon` トピック名です。 |

補足:

- `shared_control.launch.py` は `shared_control_node` に `shared_control_mode:=1` を固定で渡します。
- 同じく `autonomy_force_weight:=0.0` を固定で渡すため、自律指令はデフォルトでは shared 力へ反映されません。

## `shared_control_node` のパラメーター一覧

表の見方:

- `C++ default` は `shared_control_node.cpp` に書かれている `declare_parameter()` の既定値です。
- 実行時の実効値は `config/shared_control.yaml` と `shared_control.launch.py` の引数で上書きされることがあります。詳細は上の「パラメーターの優先順位」と「`shared_control.launch.py` の引数」を参照してください。

### モード・基本設定

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `shared_control_mode` | `0` (`normal`) | 起動時モード。`0=normal`, `1=shared`, `2=free` です。`shared_control.launch.py` では `1` (`shared`) を固定で渡します。 |
| `use_imu` | `true` | `false` のとき IMU 購読を止め、重力補償と IMU 加速度利用も自動で無効化します。launch では `CABOT_SHARED_CONTROL_USE_IMU` または `true` が使われます。 |

### 車体・ODrive 設定

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `wheel_radius_m` | `0.0855` | 車輪半径 [m]。速度換算とオドメトリ計算に使います。 |
| `wheel_separation_m` | `0.139` | 左右車輪間距離 [m]。差動二輪の速度変換に使います。 |
| `left_wheel_sign` | `-1.0` | 左車輪速度の符号補正です。 |
| `right_wheel_sign` | `1.0` | 右車輪速度の符号補正です。 |
| `odrive_velocity_is_turns_per_sec` | `true` | ODrive の速度値を `turns/s` とみなして `rad/s` に変換するかどうか。 |
| `control_mode` | `2` | `odrive_can/msg/ControlMessage.control_mode` に入れる値です。既定値 `2` は `VELOCITY_CONTROL`。 |
| `input_mode` | `1` | `odrive_can/msg/ControlMessage.input_mode` に入れる値です。既定値 `1` は `PASSTHROUGH`。 |
| `request_closed_loop_on_startup` | `true` | 起動時に `/request_axis_state` を呼んで閉ループ制御へ遷移させるかどうか。 |

### 外力推定オブザーバ

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `observer_gain_x` | `10.0` | 前後方向外力オブザーバの収束ゲインです。 |
| `observer_gain_z` | `10.0` | ヨー軸外力オブザーバの収束ゲインです。 |
| `robot_mass_x` | `24.0` | 前後方向の等価質量 [kg]。 |
| `robot_inertia_z` | `2.2` | ヨー軸まわりの等価慣性 [kg m^2]。 |
| `robot_damping_x` | `40.0` | 前後方向の粘性減衰係数です。 |
| `robot_damping_z` | `5.0` | ヨー方向の粘性減衰係数です。 |
| `coulomb_friction_x` | `8.0` | 前後方向のクーロン摩擦補償量です。 |
| `coulomb_friction_z` | `0.8` | ヨー方向のクーロン摩擦補償量です。 |
| `friction_smoothing_linear` | `0.05` | 前後方向摩擦の `tanh` 平滑化幅です。 |
| `friction_smoothing_angular` | `0.05` | 回転方向摩擦の `tanh` 平滑化幅です。 |
| `use_torque_estimate` | `true` | `ControllerStatus.torque_estimate` を使って車輪トルクを推定するかどうか。 |
| `motor_torque_constant` | `0.06` | `use_torque_estimate=false` のときに `iq_measured` からトルクへ換算する係数です。 |

### IMU 補償とコンプライアンス

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `use_gravity_compensation` | `true` | ピッチ角から坂道重力成分を補償するかどうか。 |
| `gravity_comp_gain` | `1.0` | 重力補償量のゲインです。 |
| `use_imu_linear_accel` | `false` | IMU の線形加速度を速度変化推定へ混ぜるかどうか。 |
| `imu_accel_blend` | `0.2` | `use_imu_linear_accel=true` のときの IMU 加速度ブレンド率です。 |
| `desired_mass_x` | `11.0` | shared 制御で実現したい前後方向の仮想質量です。 |
| `desired_inertia_z` | `0.9` | shared 制御で実現したいヨー方向の仮想慣性です。 |
| `desired_damping_x` | `24.0` | shared 制御で実現したい前後方向の仮想減衰です。 |
| `desired_damping_z` | `2.8` | shared 制御で実現したいヨー方向の仮想減衰です。 |

### Shared control 合成

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `human_force_weight` | `1.0` | 推定した人の操作力に掛ける重みです。 |
| `autonomy_force_weight` | `0.0` | 自律側仮想力に掛ける重みです。`shared_control.launch.py` では常に `0.0` を渡します。 |
| `autonomy_virtual_stiffness_x` | `45.0` | 自律側の前後速度偏差を仮想力へ変換する剛性です。 |
| `autonomy_virtual_stiffness_z` | `12.0` | 自律側の角速度偏差を仮想トルクへ変換する剛性です。 |
| `autonomy_timeout_sec` | `0.3` | この時間以内の自律指令だけを shared 制御に使います。 |
| `human_force_x_sign` | `-1.0` | 推定した前後方向外力の符号を実機都合で反転するための係数です。 |
| `human_torque_z_sign` | `-1.0` | 推定したヨー方向トルクの符号補正です。 |
| `force_deadband_x` | `4.0` | 人の前後方向入力として無視するデッドバンド [N] です。 |
| `force_deadband_z` | `1.2` | 人の回転入力として無視するデッドバンド [Nm] です。 |

### 制御周期と速度制限

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `loop_rate_hz` | `100.0` | 制御周期 [Hz] です。 |
| `status_timeout_sec` | `0.2` | ODrive ステータスを有効とみなすタイムアウトです。 |
| `cmd_vel_timeout_sec` | `0.2` | normal モードで `cmd_vel` を有効とみなすタイムアウトです。 |
| `axis_state_request_interval_sec` | `0.5` | `/request_axis_state` を再送する最短間隔です。 |
| `max_acc` | `0.5` | 旧互換の前進加速度既定値です。`max_linear_accel*` 未指定時の元値として使われます。 |
| `max_dec` | `-0.5` | 旧互換の減速度既定値です。`max_linear_accel_reverse` の初期値計算に使われます。 |
| `max_linear_velocity` | `0.32` | 旧互換の線速度既定値です。前進・後退別設定の元値です。 |
| `max_linear_velocity_forward` | `1.0` | 前進時の最大線速度 [m/s] です。 |
| `max_linear_velocity_reverse` | `0.3` | 後退時の最大線速度 [m/s] です。 |
| `max_angular_velocity` | `1.0` | 最大角速度 [rad/s] です。 |
| `max_linear_accel` | `0.48` | 旧互換の線加速度既定値です。前進・後退別加速度の元値です。 |
| `max_linear_accel_forward` | `0.5` | 前進時の最大線加速度 [m/s^2] です。 |
| `max_linear_accel_reverse` | `0.5` | 後退時の最大線加速度 [m/s^2] です。 |
| `max_angular_accel` | `1.5` | 最大角加速度 [rad/s^2] です。 |

### 障害物ガード

| パラメーター | C++ default | 説明 |
| --- | --- | --- |
| `obstacle_guard_enabled` | `true` | 障害物ガード全体を有効にします。 |
| `obstacle_guard_reverse_enabled` | `false` | 後退側にも同じ障害物ガードを適用するかどうか。 |
| `obstacle_stop_distance_m` | `0.0` | 障害物までこの距離以下になったら前後進を停止させる閾値です。 |
| `obstacle_slowdown_margin_m` | `0.6` | `obstacle_stop_distance_m` の外側で速度を滑らかに落とすマージン幅です。 |
| `obstacle_min_speed_scale` | `0.2` | 減速マージン内で許容する最小速度スケールです。 |
| `obstacle_pushback_enabled` | `true` | 停止距離に侵入したときに仮想ばねの押し返し力を与えるかどうか。 |
| `obstacle_pushback_stiffness` | `60.0` | 押し返し力のばね定数です。 |
| `obstacle_pushback_max_force` | `30.0` | 押し返し力の最大値です。 |
| `obstacle_timeout_sec` | `0.3` | この時間以内の `scan_topic` データだけを有効とみなします。 |
| `obstacle_point_min_z` | `-0.3` | 現在の `LaserScan` 実装では未使用です。旧 PointCloud 系設定との互換のため残っています。 |
| `obstacle_point_max_z` | `1.2` | 現在の `LaserScan` 実装では未使用です。 |
| `strict_frame_match` | `true` | 互換のため残しているフラグです。現在は `scan` 点を `base_footprint` へ tf 変換してから計算するため、実質的には未使用です。 |
| `sensor_guard_enabled` | `true` | footprint がなくても、センサ前方帯域だけで簡易ガードを行うかどうか。 |
| `sensor_guard_half_width_m` | `0.35` | 簡易ガードで使う左右半幅 [m] です。`|y| <= half_width` の点を監視します。 |

## 想定トピック

必要に応じた入出力トピックの差し替えは、ノードパラメーターではなく `shared_control.launch.py` の remapping で行います。

- 入力:
  - `/cabot/controller_status_left` (`odrive_can/msg/ControllerStatus`)
  - `/cabot/controller_status_right` (`odrive_can/msg/ControllerStatus`)
  - `/cabot/imu/data` (`sensor_msgs/msg/Imu`)
  - `/shared_control_mode` (`std_msgs/msg/Int8`, `0=normal`, `1=shared`, `2=free`)
  - `/cabot/pause_control` (`std_msgs/msg/Bool`)
  - `/cabot/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `/autonomy/cmd_vel` (`geometry_msgs/msg/TwistStamped`, 任意)
  - `/scan` (`sensor_msgs/msg/LaserScan`)
  - `/footprint` (`geometry_msgs/msg/Polygon`)
- 出力:
  - `/cabot/control_message_left` (`odrive_can/msg/ControlMessage`)
  - `/cabot/control_message_right` (`odrive_can/msg/ControlMessage`)
  - `/shared_control/external_wrench` (`geometry_msgs/msg/WrenchStamped`)
  - `/shared_control/cmd_vel` (`geometry_msgs/msg/TwistStamped`)
  - `/cabot/odom_raw` (`nav_msgs/msg/Odometry`)

## セットアップ注意

- `odrive_can` 側で cyclic message を有効化してください（特に `heartbeat`, `encoder`, `iq`, `torques`）。
- 軸状態は `CLOSED_LOOP_CONTROL` である必要があります。本ノードは起動時に `/request_axis_state` を呼び出す設定が可能です。
- `wheel_radius_m`, `wheel_separation_m`, 符号パラメータは実機に合わせて必ず調整してください。
- 障害物ガードは `sensor_msgs/msg/LaserScan` と `geometry_msgs/msg/Polygon` を使います。`PointCloud2` を直接は入力しません。
- `footprint` は `base_footprint` 座標系のポリゴンを前提にし、`scan` 点は tf で `base_footprint` に変換してから距離計算します。
