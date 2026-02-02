#!/bin/bash

# For Docker build: OpenCV install script
# Modified from NVIDIA's original version

set -e

version="4.10.0"
folder="OpenCV"

echo "** Remove default OpenCV if present"
apt-get update
apt-get -y purge '*libopencv*' || true

echo "------------------------------------"
echo "** Install requirement (1/4)"
echo "------------------------------------"
apt-get update && \
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  python3.8-dev python3-numpy \
  libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev \
  libv4l-dev v4l-utils qv4l2 v4l2ucp curl unzip \
  rm -rf /var/lib/apt/lists/* && \
  apt-get clean

echo "------------------------------------"
echo "** Download opencv ${version} (2/4)"
echo "------------------------------------"
mkdir -p ${folder}
cd ${folder}
curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
unzip opencv-${version}.zip
unzip opencv_contrib-${version}.zip
rm opencv-${version}.zip opencv_contrib-${version}.zip
cd opencv-${version}/

echo "------------------------------------"
echo "** Build opencv ${version} (3/4)"
echo "------------------------------------"
mkdir release
cd release/
cmake -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D CUDA_ARCH_BIN="8.7" \
	    -D CMAKE_CUDA_FLAGS="-DCV_CUDA_DISABLER=double" \
      -D CUDA_ARCH_PTX="" \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules \
      -D WITH_GSTREAMER=ON \
      -D WITH_LIBV4L=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j"$(nproc)"

echo "------------------------------------"
echo "** Install opencv ${version} (4/4)"
echo "------------------------------------"
make install
ldconfig

# Optional environment (could be moved to Dockerfile ENV)
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PYTHONPATH=/usr/local/lib/python3.10/site-packages/:$PYTHONPATH' >> ~/.bashrc

echo "** Installed opencv ${version} successfully"
