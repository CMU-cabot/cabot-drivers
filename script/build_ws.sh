#!/bin/bash

# Copyright (c) 2023  Carnegie Mellon University
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

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

debug=0
sequential=0

while getopts "ds" arg; do
    case $arg in
	d)
	    debug=1
	    ;;
	s)
	    sequential=1
	    ;;
    esac
done
shift $((OPTIND-1))

pushd $scriptdir/../
build_option=
if [[ $debug -eq 1 ]]; then
    build_option+=" --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install"
else
    build_option+=" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
fi
if [[ $sequential -eq 1 ]]; then
    build_option+=" --executor sequential"
fi
colcon build $build_option $@
