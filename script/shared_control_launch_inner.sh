#!/usr/bin/env bash

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

set -euo pipefail

repower_odrive=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repower-odrive)
            repower_odrive=1
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done
launch_args=("$@")

# setup scripts may reference optional vars (e.g., COLCON_TRACE).
set +u
source /home/developer/driver_ws/install/setup.bash
set -u

if [[ -z "${ROS_LOG_DIR:-}" ]]; then
    ROS_LOG_DIR="/home/developer/.ros/log/shared_control_$(date +%Y-%m-%d-%H-%M-%S)"
    export ROS_LOG_DIR
fi

mkdir -p "$ROS_LOG_DIR"
mkdir -p "$ROS_LOG_DIR/bag"

bag_topics=(
    /shared_control/cmd_vel
    /shared_control/external_wrench
    /cabot/control_message_left
    /cabot/control_message_right
    /cabot/controller_status_left
    /cabot/controller_status_right
    /cabot/odrive_status_left
    /cabot/odrive_status_right
    /cabot/imu/data
    /velodyne_points_cropped
    /footprint
    /tf
    /tf_static
)

bag_output="$ROS_LOG_DIR/bag/shared_control"
launch_pid=""
bag_pid=""

cleanup()
{
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        echo "[INFO] stopping shared control launch (pid=$launch_pid)"
        kill -INT "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
        echo "[INFO] stopping rosbag record (pid=$bag_pid)"
        kill -INT "$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
    fi
}

wait_for_power_service()
{
    local timeout_sec="${1:-8}"
    local end_time=$((SECONDS + timeout_sec))
    while (( SECONDS < end_time )); do
        if ros2 service list 2>/dev/null | grep -Fxq "/set_24v_power_odrive"; then
            echo "/set_24v_power_odrive"
            return 0
        fi
        if ros2 service list 2>/dev/null | grep -Fxq "/set_odrive_power"; then
            echo "/set_odrive_power"
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_topic_once()
{
    local topic_name="$1"
    local timeout_sec="$2"
    timeout "$timeout_sec" ros2 topic echo --once "$topic_name" >/dev/null 2>&1
}

trap cleanup EXIT INT QUIT TERM

ros2 daemon start >/dev/null 2>&1 || true

if [[ "$repower_odrive" -eq 1 ]]; then
    echo "[INFO] re-power option enabled: searching ODrive power service"
    if ! power_service_name=$(wait_for_power_service 10); then
        echo "[ERROR] ODrive power service not found (/set_24v_power_odrive or /set_odrive_power)." >&2
        exit 1
    fi
    echo "[INFO] power service: $power_service_name"
    timeout 8 ros2 service call "$power_service_name" std_srvs/srv/SetBool "{data: false}" >/dev/null
    sleep 1.0
    timeout 8 ros2 service call "$power_service_name" std_srvs/srv/SetBool "{data: true}" >/dev/null
    sleep 1.0
fi

echo "[INFO] recording bag : $bag_output"
ros2 bag record -o "$bag_output" "${bag_topics[@]}" > "$ROS_LOG_DIR/rosbag_record.log" 2>&1 &
bag_pid=$!

echo "[INFO] launching shared control"
ros2 launch cabot_shared_control shared_control.launch.py "${launch_args[@]}" &
launch_pid=$!

odrive_status_timeout_sec="${ODRIVE_STATUS_TIMEOUT_SEC:-12}"
if ! wait_for_topic_once /cabot/odrive_status_left "$odrive_status_timeout_sec"; then
    echo "[ERROR] No /cabot/odrive_status_left received within ${odrive_status_timeout_sec}s." >&2
    echo "[ERROR] ODrive power may still be OFF (e.g., emergency stop active)." >&2
    if [[ "$repower_odrive" -eq 0 ]]; then
        echo "[ERROR] Retry with -r to re-power ODrive only." >&2
    fi
    exit 1
fi
if ! wait_for_topic_once /cabot/odrive_status_right "$odrive_status_timeout_sec"; then
    echo "[ERROR] No /cabot/odrive_status_right received within ${odrive_status_timeout_sec}s." >&2
    echo "[ERROR] ODrive power may still be OFF (e.g., emergency stop active)." >&2
    if [[ "$repower_odrive" -eq 0 ]]; then
        echo "[ERROR] Retry with -r to re-power ODrive only." >&2
    fi
    exit 1
fi

echo "[INFO] ODrive status received on both axes"
wait "$launch_pid"
