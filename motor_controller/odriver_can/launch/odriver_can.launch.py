from launch.logging import launch_config

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        Node(
            package='odriver_can',
            namespace='/',
            executable='set_vel_gains_node',
            name='set_vel_gains_node',
        ),
    ])
