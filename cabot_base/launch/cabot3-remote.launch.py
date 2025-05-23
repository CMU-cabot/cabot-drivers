# Copyright (c) 2022, 2023 Carnegie Mellon University
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
Launch file for all CaBot3 Remote Control Test

change from ROS1: each model had own launch file in ROS1, but ROS2 launch will handle all models.
  differences are managed by parameter file `<model_name>.yaml`, which is common to cabot2.launch

- Known Model
  - cabot3-s1    (AIS-2023, Consortium)
  - cabot3-ace2  (AIS-2023, Miraikan)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import OrSubstitution
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot_base')

    model_name = LaunchConfiguration('model')  # need to be set
    touch_params = LaunchConfiguration('touch_params')
    gamepad = LaunchConfiguration('gamepad')
    use_joy_linux = LaunchConfiguration('use_joy_linux')
    use_keyboard = LaunchConfiguration('use_keyboard')
    is_model_ace = PythonExpression(['"', model_name, '"=="cabot3-s1"'])
    use_imu = OrSubstitution(is_model_ace, LaunchConfiguration('use_imu'))
    odrive_left_serial_number = LaunchConfiguration('odrive_left_serial_number')
    odrive_right_serial_number = LaunchConfiguration('odrive_right_serial_number')
    imu_accel_bias = LaunchConfiguration('imu_accel_bias')
    imu_gyro_bias = LaunchConfiguration('imu_gyro_bias')

    param_files = [
        ParameterFile(PathJoinSubstitution([
                pkg_dir,
                'config',
                'cabot3-common.yaml'
            ]), allow_substs=True
        ),
        ParameterFile(PathJoinSubstitution([
                pkg_dir,
                'config',
                'cabot-common-remote.yaml'
            ]), allow_substs=True
        ),
        ParameterFile(PathJoinSubstitution([
                pkg_dir,
                'config',
                PythonExpression(['"', model_name, '.yaml"'])
            ]), allow_substs=True
        ),
    ]

    return LaunchDescription([
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
            'gamepad',
            default_value='ps4',
            description='Gamepad name ps4 or pro (Switch Pro Con)'
        ),
        DeclareLaunchArgument(
            'use_joy_linux',
            default_value='False',
            description='If true use joy_linux_node instead of joy_node'
        ),
        DeclareLaunchArgument(
            'use_keyboard',
            default_value='False',
            description='If true you can control the robot via the keyboard'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='False',
            description='If true use IMU for rotation adjustment'
        ),
        DeclareLaunchArgument(
            'odrive_left_serial_number',
            default_value=EnvironmentVariable('CABOT_ODRIVER_SERIAL_0', default_value=''),
            description='Set odrive serial number (left wheel)'
        ),
        DeclareLaunchArgument(
            'odrive_right_serial_number',
            default_value=EnvironmentVariable('CABOT_ODRIVER_SERIAL_1', default_value=''),
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

        # Motor Controller Adapter
        # Convert cmd_vel (linear, rotate) speed to motor target (left, right) speed.
        Node(
            package='motor_adapter',
            executable='odriver_adapter_node',
            namespace='cabot',
            name='odriver_adapter_node',
            output=output,
            parameters=[*param_files],
            remappings=[
                ('/imu', '/cabot/imu/data'),
                ('/cmd_vel', '/cabot/cmd_vel'),  # /cabot/cmd_vel is directly input to motor_adapter in remote mode
            ],
        ),

        # Motor Controller (ODrive)
        Node(
            package='odriver',
            executable='odriver_s1_node.py',
            namespace='cabot',
            name='odriver_s1_node',
            output='log',
            parameters=[
                *param_files,
                {
                    'odrive_left_serial_number': odrive_left_serial_number,
                    'odrive_right_serial_number': odrive_right_serial_number
                }
            ],
            remappings=[
                ('/motorTarget', '/cabot/motorTarget'),
                ('/motorStatus', '/cabot/motorStatus'),
            ],
        ),

        # Microcontroller (Arduino or ESP32)
        Node(
            package='cabot_serial',
            executable='cabot_serial_node',
            namespace='cabot',
            name='cabot_serial',
            output='log',
            parameters=[
                *param_files,
                {
                    'touch_params': touch_params,
                    'imu_accel_bias': imu_accel_bias,
                    'imu_gyro_bias': imu_gyro_bias
                }
            ],
            remappings=[
                ('/cabot/imu', '/cabot/imu/data'),
                ('/cabot/imu_raw', '/cabot/imu_raw/data'),
            ],
            condition=IfCondition(use_imu),
        ),

        Node(
            package='joy',
            executable='joy_node',
            namespace='cabot',
            name='joy_node',
            parameters=[*param_files],
            condition=UnlessCondition(use_joy_linux),
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            namespace='cabot',
            name='joy_node',  # keep the node name joy_node to keep using the same parameters
            parameters=[*param_files],
            condition=IfCondition(use_joy_linux),
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            namespace='cabot',
            name=PythonExpression(['"teleop_twist_joy_', gamepad, '"']),
            parameters=[*param_files],
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            namespace='cabot',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[*param_files],
            condition=IfCondition(use_keyboard)
        ),
    ])
