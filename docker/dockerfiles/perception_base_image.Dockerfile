FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH /opt/conda/bin:$PATH

# Install Dependencies of conda
RUN apt-get update --fix-missing && \
    apt-get -y upgrade && \ 
    apt-get install -y wget bzip2 curl git libgl1-mesa-glx && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Anaconda 
RUN cd /opt && \
    curl --output anaconda.sh https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh && \
    sha256sum anaconda.sh && /bin/bash ./anaconda.sh -b -p /opt/conda && \
    export PATH=/opt/conda/bin:/opt/conda/condabin:$PATH


RUN cd /opt && \
    . /opt/conda/etc/profile.d/conda.sh && conda activate base && \
    conda config --env --add channels robostack-staging && \
    conda install --yes -c conda-forge micromamba

RUN micromamba create --yes -n ros_env -c conda-forge -c robostack-staging ros-humble-desktop

# Make RUN commands use the new environment:
# SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
SHELL ["conda", "init", "&&", "source", "/root/.bashrc"]
EXPOSE 5003
# ENTRYPOINT ["conda", "activate", "base"]
# ENTRYPOINT ["conda", "run", "--no-capture-output", "-n", "base", "python3", "src/server.py"]

CMD [ "/bin/bash" ]



