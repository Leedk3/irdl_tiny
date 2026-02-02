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

#g2o
RUN cd /opt && git clone https://github.com/Leedk3/g2o.git
RUN cd /opt/g2o && git checkout hdl_graph_slam && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE && \
    make -j8 && \
    make install

#geographic lib
# RUN cd /opt && \
    # wget http://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.51.tar.gz --no-check-certificate && \
    # tar xvzf GeographicLib-1.51.tar.gz && cd GeographicLib-1.51 && \
    # mkdir build && cd build && \
    # cmake ..&& \ 
    # make install -j8

#blasfeo lib
RUN cd /opt && git clone https://github.com/giaf/blasfeo.git
RUN cd /opt/blasfeo && \
    mkdir -p build && mkdir -p lib && \ 
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../lib) && \
    make -j8 && \
    make install

#hpipm lib
RUN cd /opt && git clone https://github.com/giaf/hpipm.git
RUN cd /opt/hpipm && \
    mkdir -p build && mkdir -p lib && \ 
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../lib) -DBLASFEO_PATH=$(realpath ../../blasfeo/lib) && \
    make -j8 && \
    make install

# #Sophus for kiss-gicp
# RUN cd /opt && \
#     git clone -b main-1.x https://github.com/strasdat/Sophus.git && \
#     cd Sophus && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

# #tsl for kiss-gicp
# RUN cd /opt && \
#     git clone https://github.com/Tessil/robin-map.git && \
#     cd robin-map && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

# RUN cd /opt && \
#     git clone -b release https://github.com/Kitware/CMake.git && \
#     cd CMake && \
#     git checkout tags/v3.18.3 && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

# RUN cd /opt && \
#     git clone --recursive https://github.com/Seongwoo-Moon/osqp.git && \
#     cd osqp && \
#     git checkout moon && \
#     git submodule update --init --recursive && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

# RUN cd /opt && \
#     git clone --recursive https://github.com/osqp/osqp.git && \
#     cd osqp && \
#     git checkout tags/v1.0.0.beta0 && \
#     git submodule update --init --recursive && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

# RUN cd /opt && \
#     git clone https://github.com/robotology/osqp-eigen.git && \
#     cd osqp-eigen && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

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
    DEBIAN_FRONTEND=noninteractive apt-get install -y geographiclib-tools libgeographic-dev \ 
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

# Install VSCode
# RUN apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common apt-transport-https wget iputils-ping && \
#     wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - && \
#     add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" && \
#     apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y code

#geographic lib
# RUN cd /opt && \
#     wget http://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.52.tar.gz --no-check-certificate && \
#     tar xvzf GeographicLib-1.52.tar.gz && cd GeographicLib-1.52 && \
#     mkdir build && cd build && \
#     cmake ..&& \ 
#     make install -j8


# Copy the requirements.txt file into the container
# RUN cd /opt && \
#     curl --output anaconda.sh https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh && \
#     sha256sum anaconda.sh && /bin/bash ./anaconda.sh -b -p /opt/conda && \
#     export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH

# # RUN /opt/conda/bin/conda init bash

# # conda install
# RUN export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH && \
#     conda install --yes -c conda-forge \
#         faiss-gpu 

# pip install
#COPY requirements.txt /tmp
# RUN . /opt/conda/etc/profile.d/conda.sh && conda activate base && \
#     export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH && \
#RUN cd /tmp && pip install --ignore-installed --no-cache-dir -r requirements.txt 

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
#COPY . /etri_ws/src/AI3CT
#RUN mkdir /etri_ws/build
#RUN mkdir /etri_ws/devel
#RUN mkdir /etri_ws/install
#RUN mkdir /etri_ws/map

#RUN chown -R ${USER}:${USER} /etri_ws
#RUN chmod 755 /etri_ws

#entri env
#COPY docker/env.sh /tmp/etri_env.sh
#RUN ["chmod", "+x", "/tmp/etri_env.sh"]
#CMD . /tmp/etri_env.sh

# SUDOERS settings
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# bashrc setting finalize
#RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USER/.bashrc
#RUN echo 'source "/etri_ws/install/setup.bash"' >> /home/$USER/.bashrc


# RUN echo 'alias auto_enable="rosservice call /autonomous_trigger \"data: true\""' >> /home/$USER/.bashrc
# RUN echo 'alias auto_disable="rosservice call /autonomous_trigger \"data: false\""' >> /home/$USER/.bashrc
# RUN echo "alias init_sim='bash /etri_ws/src/etri_uam/operations/script/init_sim.sh'" >> /home/$USER/.bashrc
# RUN echo "alias launch_bag='bash /etri_ws/src/etri_uam/launch_bagging.sh'" >> /home/$USER/.bashrc
# RUN echo "alias set_speed='rosparam set /longitudinal_control_mux_node/max_speed'" >> /home/$USER/.bashrc
# RUN echo "alias set_ratio='rosparam set /longitudinal_control_mux_node/ratio'" >> /home/$USER/.bashrc

# RUN echo 'source "/opt/ACADOtoolkit/build/acado_env.sh"' >> /home/$USER/.bashrc

# COPY docker/entrypoint.sh /var/tmp
# CMD bash -E /var/tmp/entrypoint.sh && /bin/bash

#ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}


#GIT config
#RUN cd /etri_ws && echo $GH_TOKEN > .secret.github

#WORKDIR /etri_ws

# COPY docker/env.sh /env.sh
# RUN chmod +x /env.sh
# ENTRYPOINT ["/env.sh"]
