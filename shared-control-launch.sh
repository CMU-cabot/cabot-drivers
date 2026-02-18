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
repower_odrive=0

function help()
{
    cat <<USAGE
Usage: ./shared-control-launch.sh [-d] [-r] [-- <ros2 launch args>]
  -d    Use docker compose service 'driver-dev' (default is 'driver')
  -r    Re-power on ODrive 24V line before launch (ODrive only)
  -h    Show this help
USAGE
}

while getopts "drh" arg; do
    case $arg in
        d)
            service_name="driver-dev"
            ;;
        r)
            repower_odrive=1
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
echo "[INFO] repower       : $repower_odrive"
echo "[INFO] launch args   : $*"

container_args=()
if [ "$repower_odrive" -eq 1 ]; then
    container_args+=(--repower-odrive)
fi

docker_run_opts=(--rm)
if [[ ! -t 0 ]]; then
    docker_run_opts+=(-T)
fi

exec docker compose run "${docker_run_opts[@]}" "$service_name" \
    /home/developer/driver_ws/script/shared_control_launch_inner.sh \
    "${container_args[@]}" "$@"
