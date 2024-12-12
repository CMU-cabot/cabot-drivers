# Copyright (c) 2024  Carnegie Mellon University, IBM Corporation, and others
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_base')

    xfer_format = LaunchConfiguration('xfer_format')
    multi_topic = LaunchConfiguration('multi_topic')
    data_src = LaunchConfiguration('data_src')
    publish_freq = LaunchConfiguration('publish_freq')
    output_type = LaunchConfiguration('output_type')
    frame_id = LaunchConfiguration('frame_id')
    cmdline_bd_code = LaunchConfiguration('cmdline_bd_code')
    output_topic = LaunchConfiguration('output_topic')
    output = LaunchConfiguration('output')

    user_config_path = PathJoinSubstitution([pkg_dir, 'config', 'livox', 'livox_lidar_return_dual.json'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'xfer_format',
            default_value='1',
            description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format'
        ),
        DeclareLaunchArgument(
            'multi_topic',
            default_value='0',
            description='0-All LiDARs share the same topic, 1-One LiDAR one topic'
        ),
        DeclareLaunchArgument(
            'data_src',
            default_value='0',
            description='0-lidar,1-hub'
        ),
        DeclareLaunchArgument(
            'publish_freq',
            default_value='10.0',
            description='freqency of publish,1.0,2.0,5.0,10.0,etc'
        ),
        DeclareLaunchArgument(
            'output_type',
            default_value='0',
            description='0-Output to ROS subscriber, 1-Output to rosbag file'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='livox_frame',
            description='output topic frame ID'
        ),
        DeclareLaunchArgument(
            'cmdline_bd_code',
            default_value='livox0000000001',
            description='LiDAR broadcast code'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/livox/points',
            description='output topic name'
        ),
        DeclareLaunchArgument(
            'output',
            default_value='both',
            description='livox_lidar_msg node output'
        ),

        Node(
            package='livox_ros2_driver',
            executable='livox_ros2_driver_node',
            name='livox_lidar_publisher',
            output=output,
            parameters=[
                {"xfer_format": xfer_format},
                {"multi_topic": multi_topic},
                {"data_src": data_src},
                {"publish_freq": publish_freq},
                {"output_data_type": output_type},
                {"frame_id": frame_id},
                {"user_config_path": user_config_path},
                {"cmdline_input_bd_code": cmdline_bd_code}
            ],
            remappings=[('/livox/lidar', output_topic)],
        )
    ])
