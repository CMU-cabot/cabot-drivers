#!/bin/bash

# Copyright (c) 2020, 2023  Carnegie Mellon University
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

set -m

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

pids=()
checks=()

## debug
debug=0
command_prefix=''
command_postfix='&'

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function wait_for_dds_interface_ipv4() {
    local interface_name="$1"
    local timeout_sec="${2:-30}"
    local start
    start=$(date +%s)

    if [[ -z "$interface_name" ]]; then
        return 0
    fi

    blue "waiting for global IPv4 on $interface_name (timeout=${timeout_sec}s)"

    while true; do
        if ip -4 -o addr show dev "$interface_name" scope global | grep -q 'inet '; then
            blue "$interface_name has a global IPv4 address"
            return 0
        fi

        if [[ $(( $(date +%s) - start )) -ge "$timeout_sec" ]]; then
            err "$interface_name did not get IPv4 within ${timeout_sec}s"
            return 1
        fi
        snore 1
    done
}

function signal() {
    blue "trap launch_driver.sh "

    # ps -Af
    for pid in ${pids[@]}; do
        echo "send SIGINT to $pid"
        com="kill -INT $pid"
        eval $com
    done
    for pid in ${pids[@]}; do
	count=0
        while kill -0 $pid 2> /dev/null; do
	    if [[ $count -eq 15 ]]; then
		blue "escalate to SIGTERM $pid"
		com="kill -TERM $pid"
		eval $com
	    fi
	    if [[ $count -eq 30 ]]; then
		blue "escalate to SIGKILL $pid"
		com="kill -KILL $pid"
		eval $com
	    fi
            echo "waiting $0 $pid"
	    # ps -Af
            snore 1
	    count=$((count+1))
        done
    done
    
    exit
}

# initialize environment variables
# required variables
: ${CABOT_SIDE:=left}
: ${CABOT_MODEL:=}
: ${CABOT_TOUCH_PARAMS:=}


# check required environment variables
error_flag=0
if [[ -z $CABOT_MODEL ]]; then
    err "CABOT_MODEL should be configured"
    error_flag=1
fi
if [[ -z $CABOT_TOUCH_PARAMS ]]; then
    err "CABOT_TOUCH_PARAMS should be configured"
    error_flag=1
fi
if [[ $error_flag -ne 0 ]]; then
    exit
fi

source $scriptdir/../install/setup.bash

# check cabot major version to switch venv and launch file
cabot_major=${CABOT_MODEL:0:6} # cabotN
venv_path=/opt/venv/$cabot_major/bin/activate
cabot_launch_py=$cabot_major.launch.py

source $venv_path

if [[ -n "$CYCLONEDDS_NETWORK_INTERFACE_NAME" ]]; then
    wait_for_dds_interface_ipv4 "$CYCLONEDDS_NETWORK_INTERFACE_NAME" "${CYCLONEDDS_NETWORK_INTERFACE_WAIT_TIMEOUT_SEC:-30}" || exit 1
fi

blue "bringup $CABOT_MODEL base"
com="$command_prefix ros2 launch -n cabot_base $cabot_launch_py $command_postfix"
echo $com
eval $com
checks+=($!)
pids+=($!)

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    for pid in ${checks[@]}; do
        kill -0 $pid 2> /dev/null
	if [[ $? -ne 0 ]]; then
	    red "process (pid=$pid) is not running, please check logs"
	    exit
	fi
    done
    snore 1
done
