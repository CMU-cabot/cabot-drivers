#!/usr/bin/env python3

# Copyright (c) 2026  Carnegie Mellon University
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

from __future__ import annotations

import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import List, Sequence

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message

from odrive_can.msg import ControlMessage


INPUT_TOPICS = [
    "/cabot/cmd_vel",
    "/cabot/controller_status_left",
    "/cabot/controller_status_right",
]
EXPECTED_LEFT_TOPIC = "/cabot/control_message_left"
EXPECTED_RIGHT_TOPIC = "/cabot/control_message_right"
EXPECTED_ODOM_TOPIC = "/cabot/odom_raw"


def _bag_path() -> Path:
    return (
        Path(__file__).resolve().parent
        / "bags"
        / "odriver_adapter_reference"
    )


def _read_control_messages(
    bag_path: Path,
    topics: Sequence[str],
) -> dict[str, List[object]]:
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    selected_topic_types = {
        topic_name: get_message(topic_types[topic_name])
        for topic_name in topics
    }
    results = {topic: [] for topic in topics}

    while reader.has_next():
        topic_name, raw_data, _timestamp = reader.read_next()
        if topic_name not in results:
            continue
        msg_type = selected_topic_types[topic_name]
        results[topic_name].append(deserialize_message(raw_data, msg_type))
    return results


def _terminate_process(proc: subprocess.Popen, name: str) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
    if proc.returncode not in (0, -signal.SIGINT, -signal.SIGTERM):
        raise RuntimeError(f"{name} exited with unexpected status {proc.returncode}")


def _resample(values: Sequence[float], n: int) -> List[float]:
    if n <= 0:
        return []
    if len(values) == n:
        return list(values)
    if len(values) == 1:
        return [values[0]] * n
    output = []
    src_last = len(values) - 1
    dst_last = n - 1
    for i in range(n):
        pos = 0.0 if dst_last == 0 else (i * src_last / dst_last)
        lo = int(math.floor(pos))
        hi = min(lo + 1, src_last)
        t = pos - lo
        output.append(values[lo] * (1.0 - t) + values[hi] * t)
    return output


def _mae(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(abs(x - y) for x, y in zip(a, b)) / max(len(a), 1)


def _corr(a: Sequence[float], b: Sequence[float]) -> float:
    n = len(a)
    if n == 0:
        return 0.0
    mean_a = sum(a) / n
    mean_b = sum(b) / n
    num = sum((x - mean_a) * (y - mean_b) for x, y in zip(a, b))
    den_a = sum((x - mean_a) ** 2 for x in a)
    den_b = sum((y - mean_b) ** 2 for y in b)
    den = math.sqrt(den_a * den_b)
    if den < 1.0e-12:
        return 0.0
    return num / den


def _stddev(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


class _Collector(Node):
    def __init__(self) -> None:
        super().__init__("compatibility_collector")
        self.left: List[ControlMessage] = []
        self.right: List[ControlMessage] = []
        self.odom: List[Odometry] = []
        self.create_subscription(
            ControlMessage,
            EXPECTED_LEFT_TOPIC,
            self._on_left,
            200,
        )
        self.create_subscription(
            ControlMessage,
            EXPECTED_RIGHT_TOPIC,
            self._on_right,
            200,
        )
        self.create_subscription(
            Odometry,
            EXPECTED_ODOM_TOPIC,
            self._on_odom,
            200,
        )

    def _on_left(self, msg: ControlMessage) -> None:
        self.left.append(msg)

    def _on_right(self, msg: ControlMessage) -> None:
        self.right.append(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom.append(msg)


def test_odriver_adapter_compatibility_from_bag():
    bag_path = _bag_path()
    assert bag_path.exists(), f"missing bag directory: {bag_path}"
    assert (bag_path / "metadata.yaml").exists(), f"missing metadata.yaml: {bag_path}"

    expected = _read_control_messages(
        bag_path,
        [EXPECTED_LEFT_TOPIC, EXPECTED_RIGHT_TOPIC, EXPECTED_ODOM_TOPIC],
    )
    assert len(expected[EXPECTED_LEFT_TOPIC]) > 500
    assert len(expected[EXPECTED_RIGHT_TOPIC]) > 500
    assert len(expected[EXPECTED_ODOM_TOPIC]) > 500

    env = os.environ.copy()
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    qos_override = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    qos_override.write(
        "/cabot/controller_status_left:\n"
        "  history: keep_last\n"
        "  depth: 50\n"
        "  reliability: reliable\n"
        "  durability: volatile\n"
        "/cabot/controller_status_right:\n"
        "  history: keep_last\n"
        "  depth: 50\n"
        "  reliability: reliable\n"
        "  durability: volatile\n"
        "/cabot/cmd_vel:\n"
        "  history: keep_last\n"
        "  depth: 50\n"
        "  reliability: reliable\n"
        "  durability: volatile\n"
    )
    qos_override.flush()
    qos_override_path = qos_override.name
    qos_override.close()

    node_cmd = [
        "ros2",
        "run",
        "cabot_shared_control",
        "shared_control_node",
        "--ros-args",
        "-p", "shared_control_mode:=0",
        "-p", "use_imu:=false",
        "-p", "request_closed_loop_on_startup:=false",
        "-p", "obstacle_guard_enabled:=false",
        "-p", "sensor_guard_enabled:=false",
        "-p", "loop_rate_hz:=40.0",
        "-r", "/odrive_axis0/control_message:=/cabot/control_message_left",
        "-r", "/odrive_axis1/control_message:=/cabot/control_message_right",
        "-r", "/odrive_axis0/controller_status:=/cabot/controller_status_left",
        "-r", "/odrive_axis1/controller_status:=/cabot/controller_status_right",
        "-r", "/odrive_axis0/request_axis_state:=/cabot/request_axis_state_left",
        "-r", "/odrive_axis1/request_axis_state:=/cabot/request_axis_state_right",
    ]
    play_cmd = [
        "ros2", "bag", "play", str(bag_path),
        "--qos-profile-overrides-path", qos_override_path,
        "--topics", *INPUT_TOPICS
    ]

    node_proc = subprocess.Popen(
        node_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
    )
    play_proc = None
    collector = None
    try:
        rclpy.init()
        collector = _Collector()
        time.sleep(1.0)

        play_proc = subprocess.Popen(
            play_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
        )

        finished_at = None
        deadline = time.monotonic() + 200.0
        while time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.05)
            if play_proc.poll() is not None and finished_at is None:
                finished_at = time.monotonic()
            if finished_at is not None and time.monotonic() - finished_at > 1.5:
                break

        assert play_proc.poll() == 0, "ros2 bag play did not exit cleanly"
    finally:
        if play_proc is not None:
            _terminate_process(play_proc, "ros2 bag play")
        if collector is not None:
            collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _terminate_process(node_proc, "shared_control_node")
        if os.path.exists(qos_override_path):
            os.unlink(qos_override_path)

    actual_left = collector.left if collector else []
    actual_right = collector.right if collector else []
    actual_odom = collector.odom if collector else []
    assert len(actual_left) > 500
    assert len(actual_right) > 500
    assert len(actual_odom) > 500

    expected_left_vel = [msg.input_vel for msg in expected[EXPECTED_LEFT_TOPIC]]
    expected_right_vel = [msg.input_vel for msg in expected[EXPECTED_RIGHT_TOPIC]]
    actual_left_vel = [msg.input_vel for msg in actual_left]
    actual_right_vel = [msg.input_vel for msg in actual_right]

    compare_count_left = min(len(expected_left_vel), len(actual_left_vel), 2000)
    compare_count_right = min(len(expected_right_vel), len(actual_right_vel), 2000)
    assert compare_count_left >= 500
    assert compare_count_right >= 500

    expected_left_resampled = _resample(expected_left_vel, compare_count_left)
    actual_left_resampled = _resample(actual_left_vel, compare_count_left)
    expected_right_resampled = _resample(expected_right_vel, compare_count_right)
    actual_right_resampled = _resample(actual_right_vel, compare_count_right)

    left_mae = _mae(expected_left_resampled, actual_left_resampled)
    right_mae = _mae(expected_right_resampled, actual_right_resampled)
    left_corr = _corr(expected_left_resampled, actual_left_resampled)
    right_corr = _corr(expected_right_resampled, actual_right_resampled)
    left_expected_std = _stddev(expected_left_resampled)
    right_expected_std = _stddev(expected_right_resampled)

    assert left_mae < 0.20, f"left input_vel MAE too high: {left_mae:.4f}"
    assert right_mae < 0.20, f"right input_vel MAE too high: {right_mae:.4f}"
    if left_expected_std > 1.0e-3:
        assert left_corr > 0.90, f"left input_vel correlation too low: {left_corr:.4f}"
    if right_expected_std > 1.0e-3:
        assert right_corr > 0.90, f"right input_vel correlation too low: {right_corr:.4f}"

    expected_odom = expected[EXPECTED_ODOM_TOPIC]
    compare_count_odom = min(len(expected_odom), len(actual_odom), 1500)
    assert compare_count_odom >= 500

    expected_x = _resample([msg.pose.pose.position.x for msg in expected_odom], compare_count_odom)
    actual_x = _resample([msg.pose.pose.position.x for msg in actual_odom], compare_count_odom)
    expected_y = _resample([msg.pose.pose.position.y for msg in expected_odom], compare_count_odom)
    actual_y = _resample([msg.pose.pose.position.y for msg in actual_odom], compare_count_odom)
    expected_yaw = _resample(
        [2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) for msg in expected_odom],
        compare_count_odom,
    )
    actual_yaw = _resample(
        [2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) for msg in actual_odom],
        compare_count_odom,
    )
    expected_vx = _resample([msg.twist.twist.linear.x for msg in expected_odom], compare_count_odom)
    actual_vx = _resample([msg.twist.twist.linear.x for msg in actual_odom], compare_count_odom)
    expected_wz = _resample([msg.twist.twist.angular.z for msg in expected_odom], compare_count_odom)
    actual_wz = _resample([msg.twist.twist.angular.z for msg in actual_odom], compare_count_odom)

    assert _mae(expected_x, actual_x) < 0.03, "odom x MAE too high"
    assert _mae(expected_y, actual_y) < 0.03, "odom y MAE too high"
    assert _mae(expected_yaw, actual_yaw) < 0.08, "odom yaw MAE too high"
    assert _mae(expected_vx, actual_vx) < 0.08, "odom linear velocity MAE too high"
    assert _mae(expected_wz, actual_wz) < 0.15, "odom angular velocity MAE too high"
