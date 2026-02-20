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

has_use_pause_control_arg=0
for arg in "${launch_args[@]}"; do
    case "$arg" in
        use_pause_control:=*)
            has_use_pause_control_arg=1
            break
            ;;
    esac
done
if [[ "$has_use_pause_control_arg" -eq 0 ]]; then
    launch_args+=(use_pause_control:=false)
fi

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
cleanup_done=0
power_service_name=""
odrive_power_forced_off=0

stop_process_gracefully()
{
    local process_name="$1"
    local process_pid="$2"
    local timeout_sec="${3:-10}"

    if [[ -z "$process_pid" ]] || ! kill -0 "$process_pid" 2>/dev/null; then
        return
    fi

    echo "[INFO] stopping ${process_name} (pid=${process_pid})"
    kill -INT "$process_pid" 2>/dev/null || true

    local end_time=$((SECONDS + timeout_sec))
    while kill -0 "$process_pid" 2>/dev/null; do
        if (( SECONDS >= end_time )); then
            echo "[WARN] ${process_name} did not exit after ${timeout_sec}s, sending SIGTERM"
            kill -TERM "$process_pid" 2>/dev/null || true
            break
        fi
        sleep 0.2
    done

    wait "$process_pid" 2>/dev/null || true
}

cleanup()
{
    if [[ "$cleanup_done" -eq 1 ]]; then
        return
    fi
    cleanup_done=1

    # Stop rosbag first so it can flush metadata.yaml before launch/container shutdown progresses.
    stop_process_gracefully "rosbag record" "$bag_pid" 15
    bag_pid=""

    stop_process_gracefully "shared control launch" "$launch_pid" 8
    launch_pid=""

    if [[ "$odrive_power_forced_off" -eq 1 ]] && [[ -n "$power_service_name" ]]; then
        echo "[WARN] ODrive power cycle interrupted; trying to restore 24V power ON"
        timeout 8 ros2 service call "$power_service_name" std_srvs/srv/SetBool "{data: true}" \
            >/dev/null 2>&1 || echo "[ERROR] failed to restore ODrive power ON" >&2
        odrive_power_forced_off=0
    fi
}

on_signal()
{
    local sig_name="$1"
    echo "[INFO] caught signal $sig_name"
    cleanup
    exit 130
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

trap cleanup EXIT
trap 'on_signal INT' INT
trap 'on_signal QUIT' QUIT
trap 'on_signal TERM' TERM

ros2 daemon start >/dev/null 2>&1 || true

if [[ "$repower_odrive" -eq 1 ]]; then
    echo "[INFO] re-power option enabled: searching ODrive power service"
    if ! power_service_name="$(wait_for_power_service 10)"; then
        echo "[ERROR] ODrive power service not found (/set_24v_power_odrive or /set_odrive_power)." >&2
        exit 1
    fi
    echo "[INFO] power service: $power_service_name"

    timeout 8 ros2 service call "$power_service_name" std_srvs/srv/SetBool "{data: false}" >/dev/null
    odrive_power_forced_off=1
    sleep 1.0
    timeout 8 ros2 service call "$power_service_name" std_srvs/srv/SetBool "{data: true}" >/dev/null
    odrive_power_forced_off=0
    sleep 1.0
fi

echo "[INFO] recording bag : $bag_output"
ros2 bag record -o "$bag_output" "${bag_topics[@]}" > "$ROS_LOG_DIR/rosbag_record.log" 2>&1 &
bag_pid=$!

echo "[INFO] launching shared control"
ros2 launch cabot_shared_control shared_control.launch.py "${launch_args[@]}" &
launch_pid=$!
set +e
wait "$launch_pid"
launch_rc=$?
set -e
launch_pid=""
exit "$launch_rc"
