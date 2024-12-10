#!/usr/bin/env python3

# Copyright (c) 2024  Carnegie Mellon University
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
#

import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

if __name__ == "__main__":
    rclpy.init()
    node = Node("features")

    qos_profile = QoSProfile(depth=10)
    qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

    # "left" and/or "right"
    handleside_descriptor = ParameterDescriptor(read_only=True)
    value = node.declare_parameter('handleside', 'left', handleside_descriptor).value
    handleside_pub = node.create_publisher(String, "features/handleside", qos_profile)
    msg = String()
    msg.data = value
    handleside_pub.publish(msg)

    # "cap", "tof", and/or "dual"
    touchmode_descriptor = ParameterDescriptor(read_only=True)
    value = node.declare_parameter('touchmode', 'cap', touchmode_descriptor).value
    touchmode_pub = node.create_publisher(String, "features/touchmode", qos_profile)
    msg = String()
    msg.data = value
    touchmode_pub.publish(msg)

    rclpy.spin(node)
