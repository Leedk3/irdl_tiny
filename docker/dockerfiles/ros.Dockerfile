ARG TAG
ARG L4TPYTORCH_VER=r36.4.0

FROM dustynv/l4t-pytorch:$L4TPYTORCH_VER

ENV ROS_DISTRO=jazzy
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

# jetson-utils 
# WORKDIR /tmp
# RUN git clone https://github.com/dusty-nv/jetson-utils.git && \
#     cd jetson-utils && \
#     mkdir build && cd build && \
#     cmake ../ && \
#     make -j$(nproc) && \
#     make install && \
#     ldconfig

# # make a copy of this cause it gets purged...
# RUN mkdir -p /usr/local/include/gstreamer-1.0/gst && \
#     cp -r /usr/include/gstreamer-1.0/gst/webrtc /usr/local/include/gstreamer-1.0/gst && \
#     ls -ll /usr/local/include/ && \
#     ls -ll /usr/local/include/gstreamer-1.0/gst/webrtc


# ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/local/cuda/compat:$LD_LIBRARY_PATH

# WORKDIR /tmp
# RUN git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference.git && \
#     cd jetson-inference && \
#     mkdir build && cd build && \
#     cmake ../ \
#     -DENABLE_NVMM=ON \
#     -DBUILD_EXAMPLES=OFF \
#     -DCMAKE_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu \
#     -DCMAKE_CUDA_FLAGS="-L/usr/local/cuda/lib64" && \
#     make -j$(nproc) && \
#     make install && \
#     ldconfig



# RUN add-apt-repository ppa:borglab/gtsam-release-4.1
# RUN apt-get update
# RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
#     libgtsam-dev \
#     libgtsam-unstable-dev

# RUN apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y geographiclib-tools \ 
#     && rm -rf /var/lib/apt/lists/*

# COPY docker/dockerfiles/ros2-packages /tmp/
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y \
#         $(cat /tmp/ros2-packages | cut -d# -f1 | envsubst) \
#     && rm -rf /var/lib/apt/lists/* /tmp/ros2-packages \
#     && apt-get clean

# # g2o install
# #######################################################################
# RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
#     apt-get install -y --no-install-recommends \
#         build-essential cmake git \
#         libeigen3-dev libspdlog-dev libsuitesparse-dev \
#         qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 \
#  && rm -rf /var/lib/apt/lists/*

# WORKDIR /opt
# RUN git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git
# WORKDIR /opt/g2o
# RUN mkdir build && cd build \
#  && cmake .. -DCMAKE_BUILD_TYPE=Release \
#  && make -j$(nproc) \
#  && make install \
#  && ldconfig          
# #######################################################################

# # pip install
# COPY requirements_for_mac.txt /tmp
# RUN  cd /tmp && pip install --ignore-installed --no-cache-dir -r requirements_for_mac.txt 

ARG UID=1000
ARG USER
ENV USER=$USER
RUN useradd -u $UID -ms /bin/bash $USER | echo $USER
ENV CMAKE_MODULE_PATH=/usr/local/share/cmake-3.22/Modules

# RUN useradd -ms /bin/bash usrg
COPY . /irdl_ws/src/irdl_tutorial
RUN mkdir /irdl_ws/build
RUN mkdir /irdl_ws/install
RUN mkdir /irdl_ws/map

RUN chown -R ${USER}:${USER} /irdl_ws
RUN chmod 755 /irdl_ws

# SUDOERS settings
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# bashrc setting finalize
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USER/.bashrc
RUN echo 'source "/irdl_ws/install/setup.bash"' >> /home/$USER/.bashrc

WORKDIR /irdl_ws

COPY docker/env.sh /env.sh
RUN chmod +x /env.sh
ENTRYPOINT ["/env.sh"]
