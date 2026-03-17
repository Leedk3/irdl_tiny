ARG TAG

# Ubuntu 22.04 ARM64 base — no CUDA for Apple Silicon
FROM ubuntu:22.04

ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

## Install base apt packages
## --ignore-missing handles packages unavailable on ARM64 (e.g. icecc, isag)
COPY docker/dockerfiles/apt-packages /tmp/
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --ignore-missing \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

# Python setup
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3.10 python3-pip python3.10-dev python3.10-venv git wget curl ca-certificates unzip && \
    ln -sf python3.10 /usr/bin/python3 && \
    pip3 install --upgrade pip

# PyTorch — CPU only (no CUDA on Mac)
RUN pip install torch==2.1.0 torchvision torchaudio

## Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# OpenCV — system apt package (no CUDA compilation needed)
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libopencv-dev python3-opencv \
    && rm -rf /var/lib/apt/lists/*

## Install ROS2 packages
COPY docker/dockerfiles/ros2-packages /tmp/
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --ignore-missing \
        $(cat /tmp/ros2-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/ros2-packages \
    && apt-get clean

# ZeroMQ
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        libzmq3-dev libczmq-dev \
    && rm -rf /var/lib/apt/lists/*

# pip install
COPY requirements.txt /tmp
RUN cd /tmp && pip install --ignore-installed --no-cache-dir -r requirements.txt

ARG UID=1000
ARG USER
ENV USER=$USER
RUN useradd -u $UID -ms /bin/bash $USER | echo $USER
ENV CMAKE_MODULE_PATH=/usr/local/share/cmake-3.22/Modules

COPY . /ros_ws/src/irdl_tiny
RUN mkdir -p /ros_ws/build /ros_ws/install /ros_ws/map

RUN chown -R ${USER}:${USER} /ros_ws
RUN chmod 755 /ros_ws

# Passwordless sudo
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# bashrc
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USER/.bashrc
RUN echo 'source "/ros_ws/install/setup.bash"' >> /home/$USER/.bashrc

WORKDIR /ros_ws

COPY docker/env.sh /env.sh
RUN chmod +x /env.sh
ENTRYPOINT ["/env.sh"]
