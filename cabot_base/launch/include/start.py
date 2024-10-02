from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_base')

    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'

    config_file = '/home/developer/driver_ws/install/cabot_base/share/cabot_base/config/helios/helios.yaml'
    
    return LaunchDescription([
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}, {'rslidar_points': 'pandar'}]),
        #Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
