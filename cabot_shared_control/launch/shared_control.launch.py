from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("cabot_shared_control")
    param_file = os.path.join(pkg_share, "config", "shared_control.yaml")

    node = Node(
        package="cabot_shared_control",
        executable="shared_control_node",
        name="shared_control_node",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription([node])
