#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
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
# memo: want to separate hardware spec and UI
#

import rclpy
import rclpy.node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import std_msgs.msg
import cabot_common.event
from cabot_common import vibration

from cabot_base.handle_v2 import Handle


def notification_callback(msg):
    node.get_logger().info(f"{msg}")
    handle.execute_stimulus(msg.data)


def event_listener(msg):
    node.get_logger().info(f"{msg}")
    event = None
    if "button" in msg:
        event = cabot_common.event.ButtonEvent(**msg)

        # button down confirmation
        if not event.up:
            handle.execute_stimulus(vibration.BUTTON_CLICK)
    if "buttons" in msg:
        event = cabot_common.event.ClickEvent(**msg)

    if "holddown" in msg:
        event = cabot_common.event.HoldDownEvent(**msg)
        # button hold down confirmation
        handle.execute_stimulus(vibration.BUTTON_HOLDDOWN)

    if event is not None:
        node.get_logger().info(f"{event}")
        msg = std_msgs.msg.String()
        msg.data = str(event)
        event_pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.node.Node("cabot_handle_v2_node")

    event_pub = node.create_publisher(std_msgs.msg.String, '/cabot/event', 10)
    button_keys = node.declare_parameter("buttons", ['']).value
    handle = Handle(node, event_listener=event_listener, button_keys=button_keys)

    node.get_logger().info(f"buttons: {button_keys}")

    no_vibration = node.declare_parameter("no_vibration", False).value
    node.get_logger().info(f"no_vibration = {no_vibration}")

    if not no_vibration:
        node.create_subscription(std_msgs.msg.Int8, "/cabot/notification", notification_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
