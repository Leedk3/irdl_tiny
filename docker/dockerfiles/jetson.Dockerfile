ARG TAG
ARG ROS_DISTRO=humble
ARG USER_ID
ARG GROUP_ID

FROM ros:$ROS_DISTRO

ARG GH_TOKEN=${GH_TOKEN}

## Install from packages list
COPY docker/dockerfiles/apt-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y geographiclib-tools libgeographic-dev \ 
    && rm -rf /var/lib/apt/lists/*

# RUN add-apt-repository universe
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

COPY docker/dockerfiles/ros2-packages-jetson /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/ros2-packages-jetson | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/ros2-packages-jetson \
    && apt-get clean

#geographic lib
RUN cd /opt && \
    wget http://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.52.tar.gz --no-check-certificate && \
    tar xvzf GeographicLib-1.52.tar.gz && cd GeographicLib-1.52 && \
    mkdir build && cd build && \
    cmake ..&& \ 
    make install -j8



# pip install
COPY requirements_for_jetson.txt /tmp
# RUN . /opt/conda/etc/profile.d/conda.sh && conda activate base && \
#     export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH && \
RUN cd /tmp && pip install --ignore-installed --no-cache-dir -r requirements_for_jetson.txt 

# RUN . /opt/conda/etc/profile.d/conda.sh && conda activate base && \
#     export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH && \
#     conda install mamba -c conda-forge && \
#     conda config --env --add channels robostack-staging && \
#     # Install ros-humble into the environment (ROS2)
#     mamba install ros-humble-desktop 

ARG HOST_USERNAME


# COPY third_party/ACADOtoolkit /opt/ACADOtoolkit
# RUN cd /opt/ACADOtoolkit && \
#     mkdir build && cd build && \
#     cmake ..&& \ 
#     make install -j8


ARG UID=1000
ENV USER=${HOST_USERNAME}
RUN useradd -u $UID -ms /bin/bash $USER | echo $USER

# RUN useradd -ms /bin/bash usrg
COPY . /etri_ws/src/etri_uam
RUN mkdir /etri_ws/build
#RUN mkdir /etri_ws/devel
RUN mkdir /etri_ws/install
RUN mkdir /etri_ws/map

RUN chown -R ${USER}:${USER} /etri_ws
RUN chmod 755 /etri_ws

#entri env
COPY docker/env.sh /tmp/etri_env.sh
RUN ["chmod", "+x", "/tmp/etri_env.sh"]
CMD . /tmp/etri_env.sh

# SUDOERS settings
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# bashrc setting finalize
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USER/.bashrc
RUN echo 'source "/etri_ws/install/setup.bash"' >> /home/$USER/.bashrc

ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}


#GIT config
RUN cd /etri_ws && echo $GH_TOKEN > .secret.github

WORKDIR /etri_ws

COPY docker/env.sh /env.sh
RUN chmod +x /env.sh
ENTRYPOINT ["/env.sh"]
