# Copyright (c) 2026  Carnegie Mellon University
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
Launch file for shared control on CaBot3-k4 with ODrive CAN.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    output = {'stderr': {'log'}}
    shared_control_pkg_dir = get_package_share_directory('cabot_shared_control')
    cabot_base_pkg_dir = get_package_share_directory('cabot_base')
    cabot_description_pkg_dir = get_package_share_directory('cabot_description')
    odrive_can_pkg_dir = get_package_share_directory('odrive_can')

    model_name = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    can_interface = LaunchConfiguration('can_interface')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')
    use_hesai_lidar = LaunchConfiguration('use_hesai_lidar')
    use_crop_box = LaunchConfiguration('use_crop_box')
    hesai_ros_2_0 = LaunchConfiguration('hesai_ros_2_0')
    use_imu = LaunchConfiguration('use_imu')
    shared_control_mode_topic = LaunchConfiguration('shared_control_mode_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    footprint_topic = LaunchConfiguration('footprint_topic')
    odrive_model = LaunchConfiguration('odrive_model')
    odrive_firmware_version = LaunchConfiguration('odrive_firmware_version')

    xacro_for_cabot_model = PathJoinSubstitution([
        cabot_description_pkg_dir,
        'robots',
        PythonExpression(['"', model_name, '.urdf.xacro.xml', '"'])
    ])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_for_cabot_model, ' offset:=0.25', ' sim:=', use_sim_time]),
        value_type=str
    )

    param_files = [
        ParameterFile(PathJoinSubstitution([
            cabot_base_pkg_dir,
            'config',
            'cabot3-common.yaml'
        ]),
            allow_substs=True,
        ),
        ParameterFile(PathJoinSubstitution([
            cabot_base_pkg_dir,
            'config',
            PythonExpression(['"', model_name, '.yaml"'])
        ]),
            allow_substs=True,
        ),
    ]
    shared_control_param_file = PathJoinSubstitution([
        shared_control_pkg_dir,
        'config',
        'shared_control.yaml'
    ])
    flat_endpoints_json_path = PathJoinSubstitution([
        odrive_can_pkg_dir,
        'json',
        odrive_firmware_version,
        PythonExpression(['"flat_endpoints_', odrive_model, '.json"'])
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=EnvironmentVariable('CABOT_MODEL', default_value='cabot3-k4'),
            description='CaBot model'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether the simulated time is used or not'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can1',
            description='CAN interface for ODrive S1 bus'
        ),
        DeclareLaunchArgument(
            'use_robot_state_publisher',
            default_value='true',
            description='Launch robot_state_publisher and local_robot_state_publisher'
        ),
        DeclareLaunchArgument(
            'use_hesai_lidar',
            default_value='true',
            description='Launch Hesai lidar include launch'
        ),
        DeclareLaunchArgument(
            'use_crop_box',
            default_value='true',
            description='Create /velodyne_points_cropped from /velodyne_points'
        ),
        DeclareLaunchArgument(
            'odrive_model',
            default_value=EnvironmentVariable('ODRIVE_MODEL'),
            description='odrive model'
        ),
        DeclareLaunchArgument(
            'odrive_firmware_version',
            default_value=EnvironmentVariable('ODRIVE_FIRMWARE_VERSION'),
            description='odrive firmware version'
        ),
        DeclareLaunchArgument(
            'hesai_ros_2_0',
            default_value=EnvironmentVariable('HESAI_ROS_2_0', default_value='false'),
            description='if true, use HesaiLidar_ROS_2.0'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value=EnvironmentVariable('CABOT_SHARED_CONTROL_USE_IMU', default_value='true'),
            description='If false, disable IMU input and assume horizontal terrain'
        ),
        DeclareLaunchArgument(
            'shared_control_mode_topic',
            default_value=EnvironmentVariable('CABOT_SHARED_CONTROL_MODE_TOPIC', default_value='/shared_control_mode'),
            description='Mode topic for shared_control_node (std_msgs/Int8: 0=normal, 1=shared, 2=free)'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/cabot/imu/data',
            description='IMU topic for gravity compensation'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Obstacle guard LaserScan topic'
        ),
        DeclareLaunchArgument(
            'footprint_topic',
            default_value='/footprint',
            description='Robot footprint polygon topic'
        ),
        LogInfo(msg=PythonExpression(['"Launching shared control for model: ', model_name, '"'])),
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', ''),
        ),

        GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output=output,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
                    'robot_description': robot_description
                }],
                condition=IfCondition(use_robot_state_publisher)
            ),
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
                }],
                condition=IfCondition(use_robot_state_publisher)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        cabot_base_pkg_dir, 'launch', 'include', 'hesai_lidar.launch.py'
                    ])
                ]),
                launch_arguments={
                    'model': model_name,
                    'pandar': '/velodyne_points',
                    'hesai_ros_2_0': hesai_ros_2_0
                }.items(),
                condition=IfCondition(use_hesai_lidar)
            ),
            ComposableNodeContainer(
                name='laser_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[],
                condition=IfCondition(use_crop_box)
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
                            ('/input', '/velodyne_points'),
                            ('/output', '/velodyne_points_cropped')
                        ]
                    ),
                ],
                condition=IfCondition(use_crop_box)
            ),
            Node(
                package='cabot_common',
                executable='footprint_publisher',
                name='footprint_publisher',
                output=output,
                parameters=[
                    *param_files,
                    {
                        'use_sim_time': use_sim_time,
                        # Keep this as a concrete string array. If LaunchConfiguration is
                        # embedded directly here, it can collapse to a plain string and
                        # crash footprint_publisher (expects string_array).
                        'footprint_topics': ['/footprint']
                    }
                ],
                remappings=[
                    ('/footprint', footprint_topic),
                ]
            ),
            Node(
                package='odrive_can',
                executable='odrive_can_node',
                namespace='/cabot',
                name='odrive_can_node_left',
                output='screen',
                parameters=[{
                    'node_id': 0,
                    'interface': can_interface,
                    'axis_idle_on_shutdown': True,
                    'json_file_path': flat_endpoints_json_path
                }],
                remappings=[
                    ('/cabot/control_message', '/cabot/control_message_left'),
                    ('/cabot/controller_status', '/cabot/controller_status_left'),
                    ('/cabot/odrive_status', '/cabot/odrive_status_left'),
                    ('/cabot/request_axis_state', '/cabot/request_axis_state_left')
                ]
            ),
            Node(
                package='odrive_can',
                executable='odrive_can_node',
                namespace='/cabot',
                name='odrive_can_node_right',
                output='screen',
                parameters=[{
                    'node_id': 1,
                    'interface': can_interface,
                    'axis_idle_on_shutdown': True,
                    'json_file_path': flat_endpoints_json_path
                }],
                remappings=[
                    ('/cabot/control_message', '/cabot/control_message_right'),
                    ('/cabot/controller_status', '/cabot/controller_status_right'),
                    ('/cabot/odrive_status', '/cabot/odrive_status_right'),
                    ('/cabot/request_axis_state', '/cabot/request_axis_state_right')
                ]
            ),
            Node(
                package='cabot_shared_control',
                executable='shared_control_node',
                name='shared_control_node',
                output='screen',
                parameters=[
                    shared_control_param_file,
                    {
                        'use_imu': ParameterValue(use_imu, value_type=bool),
                        'shared_control_mode_topic': shared_control_mode_topic,
                        'imu_topic': imu_topic,
                        'scan_topic': scan_topic,
                        'footprint_topic': footprint_topic,
                        'autonomy_force_weight': 0.0,
                        'use_sim_time': use_sim_time
                    }
                ],
                remappings=[
                    ('/odrive_axis0/control_message', '/cabot/control_message_left'),
                    ('/odrive_axis1/control_message', '/cabot/control_message_right'),
                    ('/odrive_axis0/controller_status', '/cabot/controller_status_left'),
                    ('/odrive_axis1/controller_status', '/cabot/controller_status_right'),
                    ('/odrive_axis0/request_axis_state', '/cabot/request_axis_state_left'),
                    ('/odrive_axis1/request_axis_state', '/cabot/request_axis_state_right')
                ]
            ),
        ],
            condition=LaunchConfigurationNotEquals('model', '')
        ),
    ])
