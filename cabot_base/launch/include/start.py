#!/usr/bin/env python3

"""
Launch file for hesai lidar
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_base')

    model_name = LaunchConfiguration('model')
    lidar_points = LaunchConfiguration('lidar_points')
    lidar_packets = LaunchConfiguration('lidar_packets')
    output = LaunchConfiguration('output')

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=EnvironmentVariable('CABOT_MODEL'),
            description='CaBot model'
        ),
        DeclareLaunchArgument(
            'lidar_points',
            default_value='lidar_points',
            description='Published PointCloud2 topic'
        ),
        DeclareLaunchArgument(
            'lidar_packets',
            default_value='lidar_packets',
            description='Published PandarScan topic'
        ),
        DeclareLaunchArgument(
            'output',
            default_value='both',
            description='hesai_lidar node output'
        ),

        Node(
            package='hesai_ros_driver',
            namespace='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver',
            output=output,
            remappings=[
                ('/lidar_points', '/velodyne_points'),
                ('/lidar_packets', lidar_packets)
            ]
        )
    ])

