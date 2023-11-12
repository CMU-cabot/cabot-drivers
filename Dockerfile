FROM ros:humble-ros-base-jammy

RUN apt update && apt install -y \
	libusb-1.0-0 \
	python3-pip \
	python3-venv \
	ros-humble-diagnostic-updater


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

WORKDIR $HOME/ros2_ws

