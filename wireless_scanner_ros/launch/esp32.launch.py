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

from launch.logging import launch_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    pkg_dir = get_package_share_directory('wireless_scanner_ros')
    port = LaunchConfiguration('port')
    wifi_scan_str = LaunchConfiguration('wifi_scan_str')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        # Append prefix name to the log directory for convenience
        RegisterEventHandler(
            OnShutdown(on_shutdown=[AppendLogDirPrefix("wifi-scan")])
        ),

        # Declare arguments
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyESP32',
            description='Serial port for ESP32'
        ),
        DeclareLaunchArgument(
            'wifi_scan_str',
            default_value='esp32/wifi_scan_str',
            description='Topic name for WiFi scan string'
        ),

        # Log information
        LogInfo(msg=['Launching ESP32 node with port: ', port]),

        # ESP32 Node
        Node(
            package='cabot_serial',
            executable='cabot_serial_node',
            name='rosserial_esp32',
            output='screen',
            parameters=[{
                'baud': 115200,
                'port': port,
                'verbose': 0
            }],
            remappings=[
                ('wifi', wifi_scan_str)
            ]
        ),

        # Include ESP32 Receiver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_dir, 'launch', 'esp32_receiver.launch.py'])
            )
        ),
    ])
