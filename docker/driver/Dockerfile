# Copyright (c) 2023, 2024  Carnegie Mellon University and Miraikan
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
ARG BASE_IMAGE=cabot-base
ARG FROM_IMAGE=cmucal/${BASE_IMAGE}:humble-base-custom-mesa

# build underlay
FROM ${FROM_IMAGE} as cache

ENV UNDERLAY_WS=/opt/underlay_ws

RUN mkdir -p $UNDERLAY_WS/src
COPY ./ros_odrive $UNDERLAY_WS/src/ros_odrive

RUN apt update && apt install -y \
    nlohmann-json3-dev \
    && \
    rm -rf /var/lib/apt/lists/*

WORKDIR $UNDERLAY_WS

# copy package.xml files with parent directories for cache
RUN find . -name "src" -type d \
      -mindepth 1 -maxdepth 2 -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/opt/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/opt || true

RUN apt update && rosdep update && rosdep install -q -y \
	--from-paths src \
	--ignore-src

RUN . /opt/custom_ws/install/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


FROM ${FROM_IMAGE} as build
ENV ROS_DISTRO=humble

ARG TZ=UTC
ENV TZ=$TZ
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone
ENV UNDERLAY_WS=/opt/underlay_ws

WORKDIR $UNDERLAY_WS

COPY --from=cache $UNDERLAY_WS/install $UNDERLAY_WS/install
# copy package.xml files from cache
COPY --from=cache /tmp/opt/src $UNDERLAY_WS/src

RUN apt update && apt install -y \
    gosu \
    libusb-1.0-0 \
    python3-pip \
    python3-venv \
    python3-gi \
    python3-dbus \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    # patch build errors for odrive_ros2_control
    ros-humble-hardware-interface \
    nlohmann-json3-dev \
    && \
    rm -rf /var/lib/apt/lists/*

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

# change location of ros base
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh

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

ENV HOME /home/$USERNAME

WORKDIR $HOME/driver_ws
RUN chown $USERNAME:$USERNAME $HOME/driver_ws

FROM build as cache_ws

USER $USERNAME

COPY --from=src ./cabot-common/cabot_common $HOME/driver_ws/src/cabot_common
COPY --from=src ./cabot-common/cabot_msgs $HOME/driver_ws/src/cabot_msgs
COPY --from=src ./cabot-common/docker/humble-custom/people $HOME/driver_ws/src/people
COPY --from=src ./cabot-description/cabot_description $HOME/driver_ws/src/cabot_description
COPY --from=src ./cabot_base $HOME/driver_ws/src/cabot_base
COPY --from=src ./cabot_serial $HOME/driver_ws/src/cabot_serial
COPY --from=src ./motor_controller $HOME/driver_ws/src/motor_controller
COPY --from=src ./wireless_scanner_ros $HOME/driver_ws/src/wireless_scanner_ros
COPY --from=src ./script $HOME/driver_ws/script

RUN /ros_entrypoint.sh /launch.sh build

FROM build as final

# change location of ros base
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed -i 's:custom_ws:underlay_ws:' /ros_entrypoint.sh

COPY --chown=$USERNAME:$USERNAME --from=cache_ws $HOME/driver_ws/install $HOME/driver_ws/install
COPY --chown=$USERNAME:$USERNAME --from=cache_ws $HOME/driver_ws/script $HOME/driver_ws/script
