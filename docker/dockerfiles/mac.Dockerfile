ARG TAG
ARG ROS_DISTRO=humble
ARG USER_ID
ARG GROUP_ID

FROM ros:$ROS_DISTRO

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

#g2o
RUN cd /opt && git clone https://github.com/Leedk3/g2o.git
RUN cd /opt/g2o && git checkout hdl_graph_slam && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE && \
    make -j8 && \
    make install

#geographic lib
RUN cd /opt && \
    wget http://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.52.tar.gz --no-check-certificate && \
    tar xvzf GeographicLib-1.52.tar.gz && cd GeographicLib-1.52 && \
    mkdir build && cd build && \
    cmake ..&& \ 
    make install -j8

#blasfeo lib

RUN add-apt-repository ppa:borglab/gtsam-release-4.1
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libgtsam-dev \
    libgtsam-unstable-dev \
    gedit

#ceres solver
# RUN cd /opt && \
#     git clone -b 1.14.x https://github.com/ceres-solver/ceres-solver.git && \
#     mkdir ceres-bin && \
#     cd ceres-bin && \
#     cmake ../ceres-solver && \
#     make -j3 && \
#     make test && \
#     make install

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y geographiclib-tools \ 
    && rm -rf /var/lib/apt/lists/*

# RUN add-apt-repository universe
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

COPY docker/dockerfiles/ros2-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/ros2-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/ros2-packages \
    && apt-get clean

# RUN apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y iputils-ping \ 
#     && rm -rf /var/lib/apt/lists/*

# Copy the requirements.txt file into the container
# RUN cd /opt && \
#     curl --output anaconda.sh https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-aarch64.sh && \
#     sha256sum anaconda.sh && /bin/bash ./anaconda.sh -b -p /opt/conda && \
#     export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH

# RUN /opt/conda/bin/conda init bash

# # pip install
# COPY requirements_for_mac.txt /tmp
# RUN . /opt/conda/etc/profile.d/conda.sh && conda activate base && \
#     export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH && \
#     cd /tmp && pip install --ignore-installed --no-cache-dir -r requirements_for_mac.txt 

ARG HOST_USERNAME


COPY third_party/ACADOtoolkit /opt/ACADOtoolkit
RUN cd /opt/ACADOtoolkit && \
    mkdir build && cd build && \
    cmake ..&& \ 
    make install -j8


ARG UID=1000
ENV USER="etri"
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
# RUN echo 'alias start_system="tmuxp load -d /etri_ws/src/etri_uam/operations/tmuxp_config/sim_2023_final.yaml"' >> /home/$USER/.bashrc
# RUN echo 'alias start_qualify="tmuxp load -d /etri_ws/src/etri_uam/operations/tmuxp_config/qualify_2023_final.yaml"' >> /home/$USER/.bashrc
# RUN echo 'alias start_solo="tmuxp load -d /etri_ws/src/etri_uam/operations/tmuxp_config/solo_2023_final.yaml"' >> /home/$USER/.bashrc

# RUN echo 'function grid() {' >> /home/$USER/.bashrc
# RUN echo '     local grid_num="$1"' >> /home/$USER/.bashrc
# RUN echo '     export GRID_NUM=$grid_num' >> /home/$USER/.bashrc
# RUN echo '     rosservice call /StartGrid "data: ' >> /home/$USER/.bashrc
# RUN echo '     data: $GRID_NUM" ' >> /home/$USER/.bashrc
# RUN echo '     # ...' >> /home/$USER/.bashrc
# RUN echo '}' >> /home/$USER/.bashrc

# RUN echo 'function force() {' >> /home/$USER/.bashrc
# RUN echo '     local x="$1"' >> /home/$USER/.bashrc
# RUN echo '     local y="$2"' >> /home/$USER/.bashrc
# RUN echo '     local thres="$3"' >> /home/$USER/.bashrc
# RUN echo '     rosservice call /ForcedInit "x: $x' >> /home/$USER/.bashrc
# RUN echo 'y: $y' >> /home/$USER/.bashrc
# RUN echo 'score_thres: $thres"' >> /home/$USER/.bashrc
# RUN echo '     # ...' >> /home/$USER/.bashrc
# RUN echo '}' >> /home/$USER/.bashrc


# RUN echo 'alias auto_enable="rosservice call /autonomous_trigger \"data: true\""' >> /home/$USER/.bashrc
# RUN echo 'alias auto_disable="rosservice call /autonomous_trigger \"data: false\""' >> /home/$USER/.bashrc
# RUN echo "alias init_sim='bash /etri_ws/src/etri_uam/operations/script/init_sim.sh'" >> /home/$USER/.bashrc
# RUN echo "alias launch_bag='bash /etri_ws/src/etri_uam/launch_bagging.sh'" >> /home/$USER/.bashrc
# RUN echo "alias set_speed='rosparam set /longitudinal_control_mux_node/max_speed'" >> /home/$USER/.bashrc
# RUN echo "alias set_ratio='rosparam set /longitudinal_control_mux_node/ratio'" >> /home/$USER/.bashrc

RUN echo 'source "/opt/ACADOtoolkit/build/acado_env.sh"' >> /home/$USER/.bashrc

# COPY docker/entrypoint.sh /var/tmp
# CMD bash -E /var/tmp/entrypoint.sh && /bin/bash

#ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}


WORKDIR /etri_ws

