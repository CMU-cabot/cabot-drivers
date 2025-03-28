# Copyright (c) 2022, 2023  Carnegie Mellon University
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
Launch file for all CaBot2

change from ROS1: each model had own launch file in ROS1, but ROS2 launch will handle all models.
  differences are managed by parameter file `<model_name>.yaml`

- Known Model
  - cabot2-gt1   (AIS-2020)
  - cabot2-gtm   (AIS-2021, Miraikan)
  - cabot2-ace   (AIS-2022, Consortium)
  - cabot2-gtmx  (AIS-2021 + Outside, Miraikan)
"""
from launch.logging import launch_config

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterFile

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_base')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_name = LaunchConfiguration('model')  # need to be set
    touch_params = LaunchConfiguration('touch_params')  # TODO no default value
    use_standalone_wifi_scanner = LaunchConfiguration('use_standalone_wifi_scanner')

    param_files = [
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'config',
            'cabot2-common.yaml'
        ]),
            allow_substs=True,
        ),
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'config',
            PythonExpression(['"', model_name, '.yaml"'])
        ]),
            allow_substs=True,
        ),
    ]

    # deprecated parameters
    # - offset
    # - no_vibration
    # - output
    # - use_velodyne
    # - use_tf_static

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether the simulated time is used or not'
        ),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(
            OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_base")]),
            condition=UnlessCondition(use_sim_time)
        ),
        DeclareLaunchArgument(
            'model',
            default_value=EnvironmentVariable('CABOT_MODEL'),
            description='CaBot model'
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
            'use_standalone_wifi_scanner',
            default_value=EnvironmentVariable('CABOT_STANDALONE_WIFI_SCANNER', default_value='false'),
            description='If true, launch stand alone wifi scanner with ESP32'
            ' (only for GT/GTM with Arduino), ace can scan wifi by builtin ESP32'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value=EnvironmentVariable('CABOT_MAX_SPEED', default_value='1.0'),
            description='Set maximum speed of the robot'
        ),

        # Kind error message
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', ''),
        ),

        GroupAction([
            ComposableNodeContainer(
                name='cabot_nodes_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[],
            ),
            LoadComposableNodes(
                target_container='/cabot_nodes_container',
                composable_node_descriptions=[
                    # CaBot related
                    ComposableNode(
                        package='cabot_base',
                        plugin='CaBotHandleV2Node',
                        namespace='/cabot',
                        name='cabot_handle_v2_node',
                        parameters=[*param_files, {'use_sim_time': use_sim_time}],
                    ),
                ]
            ),
            LoadComposableNodes(
                target_container='/cabot_nodes_container',
                condition=IfCondition(use_sim_time),
                composable_node_descriptions=[
                    # Microcontroller (Arduino - gt1/gtm or ESP32 - ace)
                    ComposableNode(
                        package='cabot_serial',
                        plugin='CaBotSerialNode',
                        namespace='/cabot',
                        name='cabot_serial',
                        parameters=[
                            *param_files,
                            {'use_sim_time': False, 'touch_params': touch_params}
                        ],
                        remappings=[
                            # ('/cabot/imu', '/cabot/imu/data'),
                            ('/cabot/touch_speed', '/cabot/touch_speed_raw')
                        ],
                    ),
                ]
            ),
            LoadComposableNodes(
                target_container='/cabot_nodes_container',
                condition=UnlessCondition(use_sim_time),
                composable_node_descriptions=[
                    ComposableNode(
                        package='cabot_serial',
                        plugin='CaBotSerialNode',
                        namespace='/cabot',
                        name='cabot_serial',
                        parameters=[
                            *param_files,
                            {'use_sim_time': False, 'touch_params': touch_params}
                        ],
                        remappings=[
                            ('/cabot/imu', '/cabot/imu/data'),
                            ('/cabot/touch_speed', '/cabot/touch_speed_raw')
                        ],
                    ),
                ]
            ),
            LoadComposableNodes(
                target_container='/cabot_nodes_container',
                condition=IfCondition(use_standalone_wifi_scanner),
                composable_node_descriptions=[
                    # optional wifi scanner with ESP32
                    ComposableNode(
                        package='cabot_serial',
                        plugin='CaBotSerialNode',
                        namespace='/cabot',
                        name='serial_esp32_wifi_scanner',
                        parameters=[*param_files, {'use_sim_time': use_sim_time}],
                        remappings=[('wifi_scan_str', '/esp32/wifi_scan_str')],
                    ),
                ]
            ),
        ],
            condition=LaunchConfigurationNotEquals('model', '')
        ),
    ])
