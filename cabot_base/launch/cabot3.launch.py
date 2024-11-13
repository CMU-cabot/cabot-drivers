# Copyright (c) 2022, 2023  Carnegie Mellon University
# Copyright (c) 2024  ALPS ALPINE CO., LTD.
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
Launch file for all CaBot3

change from ROS1: each model had own launch file in ROS1, but ROS2 launch will handle all models.
  differences are managed by parameter file `<model_name>.yaml`

- Known Model
  - cabot3-s1    (AIS-2023, Consortium)
  - cabot3-ace2  (AIS-2023, Miraikan)
  - cabot3-i1    (AIS-2024, Consortium)
  - cabot3-m1    (AIS-2024, Miraikan)
  - cabot3-m2    (AIS-2024, Miraikan)
"""
from launch.logging import launch_config

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnShutdown
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import AndSubstitution
from launch.substitutions import NotSubstitution
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterValue
from launch_ros.descriptions import ParameterFile

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot_base')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_name = LaunchConfiguration('model')  # need to be set
    touch_params = LaunchConfiguration('touch_params')  # TODO no default value
    use_standalone_wifi_scanner = LaunchConfiguration('use_standalone_wifi_scanner')
    odrive_left_serial_number = LaunchConfiguration('odrive_left_serial_number')
    odrive_right_serial_number = LaunchConfiguration('odrive_right_serial_number')
    imu_accel_bias = LaunchConfiguration('imu_accel_bias')
    imu_gyro_bias = LaunchConfiguration('imu_gyro_bias')
    use_directional_indicator = LaunchConfiguration('use_directional_indicator')
    vibrator_type = LaunchConfiguration('vibrator_type')

    # switch lidar node based on model_name
    use_hesai = PythonExpression(['"', model_name, '" in ["cabot3-ace2", "cabot3-i1", "cabot3-m1", "cabot3-m2", "cabot3-k1"]'])
    use_velodyne = NotSubstitution(use_hesai)
    use_livox = PythonExpression(['"', model_name, '" in ["cabot3-i1", "cabot3-m1", "cabot3-m2", "cabot3-k1"]'])

    xacro_for_cabot_model = PathJoinSubstitution([
        get_package_share_directory('cabot_description'),
        'robots',
        PythonExpression(['"', model_name, '.urdf.xacro.xml', '"'])
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_for_cabot_model, ' offset:=0.25', ' sim:=', use_sim_time]),
        value_type=str
    )

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
        DeclareLaunchArgument(
            'odrive_left_serial_number',
            default_value=EnvironmentVariable('CABOT_ODRIVER_SERIAL_0', default_value='xxx'),
            description='Set odrive serial number (left wheel)'
        ),
        DeclareLaunchArgument(
            'odrive_right_serial_number',
            default_value=EnvironmentVariable('CABOT_ODRIVER_SERIAL_1', default_value='yyyy'),
            description='Set odrive serial number (right wheel)'
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
            'use_directional_indicator',
            default_value=EnvironmentVariable('CABOT_USE_DIRECTIONAL_INDICATOR', default_value='false'),
            description='If true, the directional indicator on the handle is enabled'
        ),
        DeclareLaunchArgument(
            'vibrator_type',
            default_value=EnvironmentVariable('CABOT_VIBRATOR_TYPE', default_value='1'),
            description='1: ERM (Eccentric Rotating Mass), 2: LRA (Linear Resonant Actuator)'
        ),

        # Kind error message
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', ''),
        ),

        GroupAction([
            # publish robot state
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output=output,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
                    'robot_description': robot_description
                }]
            ),
            # publish **local** robot state for local map navigation (getting off elevators)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='local_robot_state_publisher',
                output=output,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
                    'frame_prefix': 'local/',
                    'robot_description': robot_description
                }]
            ),

            # launch velodyne lider related nodes
            ComposableNodeContainer(
                name='laser_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_dir, 'launch', 'include', 'vlp16.launch.py'
                    ])
                ]),
                launch_arguments={
                    'target_container': 'laser_container'
                }.items(),
                condition=IfCondition(AndSubstitution(use_velodyne, NotSubstitution(use_sim_time)))  # if (use_velodyne and (not use_simtime))
            ),

            LoadComposableNodes(
                target_container='/laser_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='pointcloud_to_laserscan',
                        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                        namespace='',
                        name='pointcloud_to_laserscan_node',
                        parameters=[*param_files, {'use_sim_time': use_sim_time}],
                        remappings=[
                            ('/cloud_in', '/velodyne_points_cropped')
                        ]
                    ),
                    ComposableNode(
                        package='pcl_ros',
                        plugin='pcl_ros::CropBox',
                        namespace='',
                        name='filter_crop_box_node',
                        parameters=[*param_files, {'use_sim_time': use_sim_time}],
                        remappings=[
                            ('/input',  '/velodyne_points'),
                            ('/output', '/velodyne_points_cropped')
                        ]
                    ),
                ]
            ),

            # launch hesai lidar node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_dir, 'launch', 'include', 'hesai_lidar.launch.py'
                    ])
                ]),
                launch_arguments={
                    'model': model_name,
                    'output': output,
                    'pandar': '/velodyne_points'
                }.items(),
                condition=IfCondition(AndSubstitution(use_hesai, NotSubstitution(use_sim_time)))  # if (use_hesai and (not use_simtime))
            ),

            # launch livox node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_dir, 'launch', 'include', 'livox_lidar_msg.launch.py'
                    ])
                ]),
                launch_arguments={
                    'frame_id': 'livox_link',
                    'xfer_format': '0',
                    'output_topic': '/livox/points',
                    'output': output
                }.items(),
                condition=IfCondition(AndSubstitution(use_livox, NotSubstitution(use_sim_time)))
            ),

            # CaBot related
            Node(
                package='cabot_base',
                executable='cabot_handle_v3_node',
                namespace='/cabot',
                name='cabot_handle_v3_node',
                output=output,
                parameters=[
                    *param_files,
                    {
                        'use_sim_time': use_sim_time,
                        'vibrator_type': vibrator_type
                    }
                ],
                condition=IfCondition(use_directional_indicator),
            ),
            Node(
                package='cabot_base',
                executable='cabot_handle_v2_node',
                namespace='/cabot',
                name='cabot_handle_v2_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
                condition=UnlessCondition(use_directional_indicator),
            ),

            # Microcontroller (Arduino - gt1/gtm or ESP32 - ace)
            Node(
                package='cabot_serial',
                executable='cabot_serial_node',
                namespace='/cabot',
                name='cabot_serial',
                output=output,
                parameters=[
                    *param_files,
                    {'use_sim_time': use_sim_time, 'touch_params': touch_params}
                ],
                remappings=[
                    # ('/cabot/imu', '/cabot/imu/data'), # /cabot/imu/data topic is published by the simulator
                    ('/cabot/touch_speed', '/cabot/touch_speed_raw')
                ],
                condition=IfCondition(use_sim_time)
            ),
            #Node(
            #    package='cabot_serial',
            #    executable='cabot_serial_node',
            #    namespace='/cabot',
            #    name='cabot_serial',
            #    output=output,
            #    parameters=[
            #        *param_files,
            #        {
            #            'use_sim_time': use_sim_time,
            #            'touch_params': touch_params,
            #            'imu_accel_bias': imu_accel_bias,
            #            'imu_gyro_bias': imu_gyro_bias
            #        }
            #    ],
            #    remappings=[
            #        ('/cabot/imu', '/cabot/imu/data'),
            #        ('/cabot/imu_raw', '/cabot/imu_raw/data'),
            #        ('/cabot/touch_speed', '/cabot/touch_speed_raw')
            #    ],
            #    condition=UnlessCondition(use_sim_time)
            #),
            Node(
                package='cabot_can',
                executable='can_all_node',
                namespace='/cabot',
                name='cabot_can',
                output=output,
                parameters=[
                    *param_files,
                    {
                        'use_sim_time': use_sim_time,
                        'touch_params': touch_params,
                        'imu_accel_bias': imu_accel_bias,
                        'imu_gyro_bias': imu_gyro_bias
                    }
                ],
                remappings=[
                    ('/cabot/imu', '/cabot/imu/data'),
                    ('/cabot/touch_speed', '/cabot/touch_speed_raw'),
                    ('/cabot/bme/pressure', '/cabot/pressure')
                ],
                condition=UnlessCondition(use_sim_time)
            ),
            Node(
                package='power_controller',
                executable='power_controller',
                namespace='/cabot',
                name='power_controller',
                output=output,
                parameters=[
                    *param_files,
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
                condition=UnlessCondition(use_sim_time)
            ),
            # optional wifi scanner with ESP32
            Node(
                package='cabot_serial',
                executable='cabot_serial_node',
                namespace='/cabot',
                name='serial_esp32_wifi_scanner',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
                remappings=[('wifi_scan_str', '/esp32/wifi_scan_str')],
                condition=IfCondition(use_standalone_wifi_scanner),
            ),

            # Motor Controller Adapter
            # Convert cmd_vel (linear, rotate) speed to motor target (left, right) speed.
            Node(
                package='motor_adapter',
                executable='odriver_adapter_node',
                namespace='/cabot',
                name='odriver_adapter_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/imu', '/cabot/imu/data')
                ],
                condition=UnlessCondition(use_sim_time),
            ),

            # Motor Controller (ODrive)
            #Node(
            #    package='odriver',
            #    executable='odriver_s1_node.py',
            #    namespace='/cabot',
            #    name='odriver_s1_node',
            #    output=output,
            #    parameters=[
            #        *param_files,
            #        {
            #            'use_sim_time': use_sim_time,
            #            'odrive_left_serial_number': odrive_left_serial_number,
            #            'odrive_right_serial_number': odrive_right_serial_number
            #        }
            #    ],
            #    remappings=[
            #        ('/motorTarget', '/cabot/motorTarget'),
            #        ('/motorStatus', '/cabot/motorStatus'),
            #    ],
            #    condition=UnlessCondition(use_sim_time),
            #),
            Node(
                package='odriver_can_adapter',
                executable='odriver_can_adapter_node',
                namespace='/cabot',
                name='odriver_can_adapter_node',
                output='screen',
                parameters=[
                    {
                        'is_clockwise' : True,
                    }
                ],
                remappings=[
                    ('/control_message_left', '/cabot/control_message_left'),
                    ('/control_message_right', '/cabot/control_message_right'),
                    ('/controller_status_left', '/cabot/controller_status_left'),
                    ('/controller_status_right', '/cabot/controller_status_right'),
                    ('/motor_status', '/cabot/motorStatus'),
                    ('/motor_target', '/cabot/motorTarget'),
                    ('/request_axis_state_left', '/cabot/request_axis_state_left'),
                    ('/request_axis_state_right', '/cabot/request_axis_state_right'),
                ],
            ),
            Node(
                package='odrive_can',
                executable='odrive_can_node',
                namespace='/cabot',
                name='odrive_can_node_left',
                output='screen',
                parameters=[
                    {
                        'node_id' : 0,
                        'interface' : 'can1',
                        'axis_idle_on_shutdown' : True,
                    }
                ],
                remappings=[
                    ('/cabot/control_message', '/cabot/control_message_left'),
                    ('/cabot/controller_status', '/cabot/controller_status_left'),
                    ('/cabot/request_axis_state', '/cabot/request_axis_state_left')
                ],
            ),
            Node(
                package='odrive_can',
                executable='odrive_can_node',
                namespace='/cabot',
                name='odrive_can_node_right',
                output='screen',
                parameters=[
                    {
                        'node_id' : 1,
                        'interface' : 'can1',
                        'axis_idle_on_shutdown' : True,
                    }
                ],
                remappings=[
                    ('/cabot/control_message', '/cabot/control_message_right'),
                    ('/cabot/controller_status', '/cabot/controller_status_right'),
                    ('/cabot/request_axis_state', '/cabot/request_axis_state_right'),
                ],
            ),
        ],
            condition=LaunchConfigurationNotEquals('model', '')
        ),
    ])
