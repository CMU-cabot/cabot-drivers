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

ARG ROS_DISTRO
ARG PREFIX=cabot
ARG FROM_IMAGE=${PREFIX}__jammy-humble-custom-mesa

FROM ${FROM_IMAGE}
ENV ROS_DISTRO=humble

RUN apt update && apt install -y \
	libusb-1.0-0 \
	python3-pip \
	python3-venv \
    python3-gi \
    python3-dbus \
	ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-xacro

RUN pip3 install --no-cache-dir \
    odrive==0.6.5 \
    pyserial

# set up venv for keeping compatibility
RUN mkdir -p /opt/venv
# cabot2 venv (odrive==0.5.2.post0)
RUN python3 -m venv --system-site-packages /opt/venv/cabot2 && \
    . /opt/venv/cabot2/bin/activate && \
    pip3 install --no-cache-dir odrive==0.5.2.post0 && \
    deactivate
# cabot3 venv (odrive==0.6.5)
RUN python3 -m venv --system-site-packages /opt/venv/cabot3

COPY ./launch.sh /

ENV USERNAME developer
# Replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
	usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/driver_ws

RUN mkdir -p src && git clone http://github.com/wg-perception/people.git -b ros2 src/people
