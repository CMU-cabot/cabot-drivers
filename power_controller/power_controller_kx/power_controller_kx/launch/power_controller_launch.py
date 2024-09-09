from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # launchの構成を示すLaunchDescription型の変数の定義
    ld = LaunchDescription()

    # publisher nodeを、"talker_renamed1"という名前で定義
    power_controller_node = Node(
        package='power_controller',
        executable='power_controller',
        name='power_controller',
        respawn="true"
    )

    ld.add_action(power_controller_node)
    return ld
