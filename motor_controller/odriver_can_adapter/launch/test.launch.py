# ******************************************************************************
#  Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_adapter',
            executable='odriver_adapter_node',
            namespace='/cabot',
            name='motor_adapter_node',
            output='screen',
            parameters=[
                {
                    'max_acc': 1.2,
                    'target_rage': 20,
                    'bias': 1.0,
                    'gain_omega': 1.0,
                    'gain_omega_i': 0.0,
                }
            ],
            remappings=[
                ('/imu', '/cabot/imu/data'),
                ('/motor', '/cabot/motor_target'),
                ('/encoder', '/cabot/motor_status'),
                ('/cmd_vel', '/cabot/cmd_vel'),
                ('/odom', '/cabot/odom'),
                ('/pause_control', '/cabot/pause_control'),
            ],
        ),
        Node(
            package='odriver_can_adapter',
            executable='odriver_can_adapter_node',
            namespace='/cabot',
            name='odriver_can_adapter_node',
            output='screen',
            parameters=[
                {
                    'is_clockwise': True,
                }
            ],
            remappings=[
                ('/control_message_left', '/cabot/control_message_left'),
                ('/control_message_right', '/cabot/control_message_right'),
                ('/controller_status_left', '/cabot/controller_status_left'),
                ('/controller_status_right', '/cabot/controller_status_right'),
                ('/motor_status', '/cabot/motor_status'),
                ('/motor_target', '/cabot/motor_target'),
                ('/request_axis_state_left', '/cabot/request_axis_state_left'),
                ('/request_axis_state_right', '/cabot/request_axis_state_right'),
                ('/odrive_status_left', '/cabot/odrive_status_left'),
                ('/odrive_status_right', '/cabot/odrive_status_right'),
                ('/set_odrive_power', '/set_24v_power_odrive'),
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
                    'interface': 'can1',
                    'node_id': 0,
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
                    'interface': 'can1',
                    'node_id': 1,
                }
            ],
            remappings=[
                ('/cabot/control_message', '/cabot/control_message_right'),
                ('/cabot/controller_status', '/cabot/controller_status_right'),
                ('/cabot/request_axis_state', '/cabot/request_axis_state_right'),
            ],
        ),
    ])
