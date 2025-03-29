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
Launch file for power controller

- Known Model
  - cabot2-ace   (AIS-2022, Miraikan)
  - cabot3-ace2  (AIS-2023, Miraikan)
  - cabot3-k1    (AIS-2024, Kufusha)
  - cabot3-k2    (AIS-2024, Kufusha)
  - cabot3-k3    (AIS-2024, Kufusha)
  - cabot3-k4    (AIS-2024, Kufusha)
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
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot_base')

    model_name = LaunchConfiguration('model')  # need to be set

    # Define models with their associated flags (without the "use_" prefix)
    model_flags = {
        "cabot2-ace": ["ace"],
        "cabot3-ace2": ["ace"],
        "cabot3-k1": ["kx"],
        "cabot3-k2": ["kx"],
        "cabot3-k3": ["kx"],
        "cabot3-k4": ["kx"],
    }

    # Helper function to check if a flag applies to the given model
    def has_flag(flag_name):
        return PythonExpression(['"', model_name, '" in ', str([k for k, v in model_flags.items() if flag_name in v])])

    # Flags derived from the model's features (using the "use_" prefix for variable names)
    use_ace = has_flag("ace")
    use_kx = has_flag("kx")

    param_files = [
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'config',
            PythonExpression(['"', model_name, '.yaml"'])
        ]),
            allow_substs=True,
        ),
    ]

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        DeclareLaunchArgument(
            'model',
            default_value=EnvironmentVariable('CABOT_MODEL'),
            description='CaBot model'
        ),
        LogInfo(msg=PythonExpression(["\"Launching for model: ", model_name, "\""])),
        LogInfo(msg=PythonExpression(["\"            use_ace: ", use_ace, "\""])),
        LogInfo(msg=PythonExpression(["\"             use_kx: ", use_kx, "\""])),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(
            OnShutdown(on_shutdown=[AppendLogDirPrefix("power_controller")])
        ),

        # Kind error message
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', ''),
        ),

        GroupAction([
            Node(
                package='power_controller_ace',
                executable='power_controller_node.py',
                name='power_controller',
                output=output,
                parameters=[
                    *param_files
                ],
                condition=IfCondition(use_ace)
            ),
            Node(
                package='power_controller_kx',
                executable='power_controller',
                name='power_controller',
                output=output,
                parameters=[
                    *param_files
                ],
                remappings=[("/temperature3", "/cabot/temperature3")],
                condition=IfCondition(use_kx)
            ),
        ],
            condition=LaunchConfigurationNotEquals('model', '')
        ),
    ])
