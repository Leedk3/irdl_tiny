ARG TAG

FROM nvidia/cuda:12.1.1-cudnn8-runtime-ubuntu22.04

ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

## Install from packages list
COPY docker/dockerfiles/apt-packages /tmp/
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

# 기본 설정
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3.10 python3-pip python3.10-dev python3.10-venv git wget curl ca-certificates unzip && \
    ln -sf python3.10 /usr/bin/python3 && \
    pip3 install --upgrade pip

# PyTorch 설치 (CUDA 12.1용)
RUN pip install torch==2.1.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

## Install ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 
# install OpenCV (with CUDA)
#
COPY docker/opencv_install.sh /tmp/opencv_install.sh
RUN cd /tmp && ./opencv_install.sh 

# 
# install development packages
#
COPY docker/dockerfiles/ros2-packages /tmp/
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/ros2-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/ros2-packages \
    && apt-get clean


# Additional layer
#######################################################################
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        libzmq3-dev libczmq-dev \
 && rm -rf /var/lib/apt/lists/*

# pip install
COPY requirements.txt /tmp
RUN pip install --no-cache-dir "setuptools==65.5.1" "wheel<0.40.0" "pip<24.1"
RUN  cd /tmp && pip install --ignore-installed --no-cache-dir -r requirements.txt

ARG UID=1000
ARG USER
ENV USER=$USER
RUN useradd -u $UID -ms /bin/bash $USER | echo $USER
ENV CMAKE_MODULE_PATH=/usr/local/share/cmake-3.22/Modules

# RUN useradd -ms /bin/bash usrg
COPY . /ros_ws/src/irdl_tiny
RUN mkdir /ros_ws/build
RUN mkdir /ros_ws/install
RUN mkdir /ros_ws/map

# Install f1tenth_gym from bundled source
RUN pip install --no-cache-dir /ros_ws/src/irdl_tiny/f1tenth_gym_ros/f1tenth_gym

# Upgrade colcon-python-setup-py: apt version has bug with --symlink-install calling
# 'python setup.py --editable' which is not recognized. Newer pip version fixes this.
# Pin coverage<7.0: coverage 7.x removed coverage.types which numba depends on.
RUN pip install --no-cache-dir "colcon-python-setup-py>=0.2.7" "setuptools==65.5.1" "coverage<7.0"

RUN chown -R ${USER}:${USER} /ros_ws
RUN chmod 755 /ros_ws

# SUDOERS settings
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# bashrc setting finalize
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USER/.bashrc
RUN echo 'source "/ros_ws/install/setup.bash"' >> /home/$USER/.bashrc

WORKDIR /ros_ws

COPY docker/env.sh /env.sh
RUN chmod +x /env.sh
ENTRYPOINT ["/env.sh"]
