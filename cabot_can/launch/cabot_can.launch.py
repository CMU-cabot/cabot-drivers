# Copyright (c) 2025  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Launch file for cabot_can
"""
from launch.logging import launch_config

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
# from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = 'both'

    use_sim_time = LaunchConfiguration('use_sim_time')
    touch_params = LaunchConfiguration('touch_params')  # TODO no default value
    # touch_enabled = LaunchConfiguration('touch_enabled')  # TODO not implemented
    imu_accel_bias = LaunchConfiguration('imu_accel_bias')
    imu_gyro_bias = LaunchConfiguration('imu_gyro_bias')
    tof_touch_threshold_low = LaunchConfiguration('tof_touch_threshold_low')
    tof_touch_threshold_high = LaunchConfiguration('tof_touch_threshold_high')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        # RegisterEventHandler(
        #     OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_can")]),
        #     condition=UnlessCondition(use_sim_time)
        # ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether the simulated time is used or not'
        ),
        DeclareLaunchArgument(
            'touch_params',
            default_value=EnvironmentVariable('CABOT_TOUCH_PARAMS'),
            description='An array of three values for touch detection, like [128, 48, 24]'
        ),
        DeclareLaunchArgument(
            'touch_enabled',
            default_value=EnvironmentVariable('CABOT_TOUCH_ENABLED', default_value='true'),
            description='If true, the touch sensor on the handle is used to control speed'
        ),
        DeclareLaunchArgument(
            'imu_accel_bias',
            default_value=EnvironmentVariable('CABOT_IMU_ACCEL_BIAS', default_value='[0.0, 0.0, 0.0]'),
            description='An array of three values for adjusting imu acceleration'
        ),
        DeclareLaunchArgument(
            'imu_gyro_bias',
            default_value=EnvironmentVariable('CABOT_IMU_GYRO_BIAS', default_value='[0.0, 0.0, 0.0]'),
            description='An array of three values for adjusting imu angular velocity'
        ),
        DeclareLaunchArgument(
            'tof_touch_threshold_low',
            default_value=EnvironmentVariable('CABOT_TOF_TOUCH_THRESHOLD_LOW', default_value='25'),
            description='The distance threshold to enable tof detection'
        ),
        DeclareLaunchArgument(
            'tof_touch_threshold_high',
            default_value=EnvironmentVariable('CABOT_TOF_TOUCH_THRESHOLD_HIGH', default_value='35'),
            description='The distance threshold to disable tof detection (after enabling)'
        ),
        Node(
            package='cabot_can',
            executable='cabot_can_node',
            namespace='/cabot',
            name='cabot_can',
            output=output,
            parameters=[
                {
                    'can_interface': 'can0',
                    'use_sim_time': use_sim_time,
                    'touch_params': touch_params,
                    'imu_accel_bias': imu_accel_bias,
                    'imu_gyro_bias': imu_gyro_bias,
                    'tof_touch_threshold_low': tof_touch_threshold_low,
                    'tof_touch_threshold_high': tof_touch_threshold_high,
                }
            ],
            remappings=[
                ('/cabot/imu', '/cabot/imu/data'),
                ('/cabot/touch_speed', '/cabot/touch_speed_raw'),
            ],
        ),
        # wifi scan converter for cabot_can
        Node(
            package='wireless_scanner_ros',
            executable='esp32_wifi_scan_converter.py',
            namespace='/cabot',
            name='esp32_wifi_scan_converter',
            output=output,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                }
            ],
            remappings=[
                ('/esp32/wifi_scan_str', '/cabot/wifi'),
            ],
        )
    ])
