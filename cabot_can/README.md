# cabot_can
## Node
- cabot_can_node
### Services
- **run_imu_calibration** (std_srvs::srv::Trigger)
    - This service will publish data from '/calibration' by sending the service if all 4 bits of the data received from CAN ID: 438 are equal to 3.
- **run_capacitive_calibration** (std_srvs::srv::Trigger)    
    - During calibration, the 2nd bit of CAN ID: 482 will be set to 1
- **nullify_capacitive_noise** (std_srvs::srv::Trigger)
    - Determines whether the analog noise filter is enabled. 
### Parameters
- **publish_rate** (int) - default=`200`
    - publish_rate
- **can_interface** (string) - default=`can0`
    - can interface numberinterface
- **imu_frame_id** (string) - default=`imu_frame`
    - imu frame id
- **calibration_params** (int[22]) - default=`(22,0)`
    - Can be set by setting parameters in calibration_params of params.yaml
- **touch_mode** (string) - default=`touch_mode_`
    - Selecting Sensors for /touch (ToF Sensors, Capacitive Sensors, or Both)
- **tof_touch_threshold** (int) - default=`25`
    - Determine the maximum distance threshold for the ToF sensor
### Publishers
- **/calibration** (int[22]): only publishes values when `/run_imu_calibration` is true
- **/imu** (sensor_msgs/msg/Imu): IMU data - 100Hz
- **/pressure**,**/bme/pressure** (sensor_msgs/msg/FluidPressure): Pressure data - 2Hz
- **/pushed** (std_msgs/msg/Int8): button status (bit0-4 represents each button) - 50Hz
- **/temparature_1**,**/temparature_2**,**/temparature_3**,**/temparature_4**,**/temparature_5**,**/bme/temperature**(sensor_msgs/msg/Temperature): Temperature data - 2Hz
- **/touch**, (std_msgs/msg/Int16): `1` if touched else `0`,
TOF Sensor and Capacitive Sensor Integration - 50Hz
- **/capacitive/touch**(std_msgs/msg/Int16): `1` if touched else `0`, Capacitive sensor's - 50Hz
- **/tof/touch**(std_msgs/msg/Int16): `1` if touched else `0`, Tof sensor's - 50Hz
- **/capacitive/touch_raw** (std_msgs/msg/Int16): capacitive sensor's raw value (for debug) - 50Hz
- **/tof/touch_raw** (std_msgs/msg/Int16): tof sensor's raw value (for debug) - 50Hz
- **/wifi** (std_msgs/msg/String): wifi scan string - depends on how often WiFi signal is scanned
- **/servo_pos** (std_msgs/msg/Int16): current position of directional indicator - 50Hz
### Subscribers
- **/vibrator1** (std_msgs/msg/UInt8): front
- **/vibrator3** (std_msgs/msg/UInt8): left
- **/vibrator4** (std_msgs/msg/UInt8): right
    - if you publish a `value`, it vibrates corresponding vibrator for `value` x 10 milliseconds (0 - 2550 milliseconds)
- **/servo_target** (std_msgs/msg/Int16): if you publish a `value`, it rotates directional indicator (value: -179 ~ +179)
- **/servo_free** (std_msgs/msg/Bool): `false` -> set free, `true` -> no operation
- **/capacitive/recalibration** (std_msgs/msg/UInt8): 
    - Bits 4 - 3 - NEG_DELTA_CNT[1:0] - Determines the number of negative delta counts necessary to trigger a digital calibration
- **capacitive/bc_out_recalibration** (std_msgs/msg/UInt8): 
    - Bit 2 - DIS_RF_NOISE - Determines whether the RF noise filter is enabled. Setting this bit disables the feature.
        - ‘0’ (default) - If RF noise is detected by the analog block, the delta count on the corresponding channel is set to 0. Note that this does not require that Noise Status bits be set.
        - ‘1’ - A touch is not blocked even if RF noise is detected.
    - Bit 6 - BC_OUT_RECAL - Controls whether to retry analog calibration when the base count is out of limit for one or more sensor inputs.
        - ‘0’ - When the BC_OUTx bit is set for a sensor input, the out of limit base count will be used for the sensor input.
        - ‘1’ (default) - When the BC_OUTx bit is set for a sensor input , analog calibration will be repeated on the sensor input.