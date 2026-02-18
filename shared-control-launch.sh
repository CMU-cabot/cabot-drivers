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

scriptdir=$(cd "$(dirname "$0")" && pwd)
cd "$scriptdir"

service_name="driver"

function help()
{
    cat <<USAGE
Usage: ./shared-control-launch.sh [-d] [-- <ros2 launch args>]
  -d    Use docker compose service 'driver-dev' (default is 'driver')
  -h    Show this help
USAGE
}

while getopts "dh" arg; do
    case $arg in
        d)
            service_name="driver-dev"
            ;;
        h)
            help
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

if [ -f "$scriptdir/.env" ]; then
    set -a
    # shellcheck source=/dev/null
    source "$scriptdir/.env"
    set +a
fi

export CABOT_SHARED_CONTROL_USE_IMU="${CABOT_SHARED_CONTROL_USE_IMU:-true}"

log_name="shared_control_$(date +%Y-%m-%d-%H-%M-%S)"
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
host_ros_log_root="$scriptdir/docker/home/.ros/log"
host_ros_log_dir="$host_ros_log_root/$log_name"
mkdir -p "$host_ros_log_dir"
ln -snf "$host_ros_log_dir" "$host_ros_log_root/latest_shared_control"
if [ -f "$scriptdir/.env" ]; then
    cp "$scriptdir/.env" "$host_ros_log_dir/env-file"
fi

echo "[INFO] service       : $service_name"
echo "[INFO] ROS_LOG_DIR   : $ROS_LOG_DIR"
echo "[INFO] host log dir  : $host_ros_log_dir"
echo "[INFO] use imu       : $CABOT_SHARED_CONTROL_USE_IMU"
echo "[INFO] launch args   : $*"

docker compose run --rm -T "$service_name" bash -s -- "$@" <<'INNER_SCRIPT'
set -euo pipefail

# setup scripts may reference optional vars (e.g., COLCON_TRACE).
# Temporarily disable nounset while sourcing.
set +u
source /home/developer/driver_ws/install/setup.bash
set -u

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

echo "[INFO] recording bag : $bag_output"
ros2 bag record -o "$bag_output" "${bag_topics[@]}" > "$ROS_LOG_DIR/rosbag_record.log" 2>&1 &
bag_pid=$!

cleanup()
{
    if kill -0 "$bag_pid" 2>/dev/null; then
        echo "[INFO] stopping rosbag record (pid=$bag_pid)"
        kill -INT "$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT QUIT TERM

echo "[INFO] launching shared control"
ros2 launch cabot_shared_control shared_control.launch.py "$@"
INNER_SCRIPT
