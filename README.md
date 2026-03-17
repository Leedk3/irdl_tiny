# irdl_tiny: Autonomous System Development Environment

A Docker-based development environment built on Ubuntu 22.04 + ROS2 Humble for building and running autonomous system software.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Create Workspace & Clone Repository](#2-create-workspace--clone-repository)
3. [Build Docker Image](#3-build-docker-image)
4. [Shell Environment Setup](#4-shell-environment-setup)
5. [Run the Container](#5-run-the-container)
6. [Build Packages](#6-build-packages)
7. [GUI on Mac (RViz2 / X11)](#7-gui-on-mac-rviz2--x11)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Prerequisites

### 1-1. Install Docker

```bash
# Remove old versions (if any)
sudo apt-get remove docker docker-engine docker.io containerd runc

# Install using the official script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Allow running docker without sudo (re-login required)
sudo usermod -aG docker $USER
```

Verify installation:

```bash
docker --version
```

### 1-2. Install NVIDIA Container Toolkit (required for GPU support)

```bash
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

sudo systemctl restart docker
```

Verify installation:

```bash
docker run --rm --gpus all nvidia/cuda:12.1.1-base-ubuntu22.04 nvidia-smi
```

> Reference: [NVIDIA Docker Setup Guide](https://velog.io/@boom109/Nvidia-docker)

---

## 2. Create Workspace & Clone Repository

```bash
mkdir -p /home/${USER}/ros_ws/src
cd /home/${USER}/ros_ws/src
git clone --recursive https://github.com/Leedk3/irdl_tutorial.git
cd irdl_tutorial
git checkout main
```

> **Note**: If you do not have an SSH key configured, submodule cloning may fail.
> Set up GitHub SSH keys here: [github.com/settings/keys](https://github.com/settings/keys)

---

## 3. Build Docker Image

```bash
cd /home/${USER}/ros_ws/src/irdl_tutorial/docker
```

### For x86 Desktop (PC / Workstation)

Includes: Ubuntu 22.04, ROS2 Humble, CUDA 12.1 + cuDNN 8, PyTorch 2.1.0 (CUDA), OpenCV (CUDA), ZeroMQ

```bash
./build_docker.sh x86
```

Resulting image: `irdl-tiny-image:x86`

> **Note**: The first build may take 30 minutes to over an hour due to OpenCV CUDA compilation.

### For Mac (Apple Silicon — M1/M2/M3/M4/M5)

Includes: Ubuntu 22.04 ARM64, ROS2 Humble, PyTorch 2.1.0 (CPU), OpenCV (apt), ZeroMQ

```bash
./build_docker.sh mac
```

Resulting image: `irdl-tiny-image:mac`

> **Note**: No CUDA support. GPU-accelerated features (CUDA inference, CUDA OpenCV) will not be available.

### For Jetson (Edge Computing Board)

```bash
./build_docker.sh ros2
```

Resulting image: `irdl-tiny-image:jetson`

---

## 4. Shell Environment Setup

Add the following to `~/.bashrc` to configure ROS2 environment variables and container launch aliases.

```bash
# ROS2 DDS middleware
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export RMW_CYCLONEDDS_URI=file:///home/${USER}/.cyclonedds.xml" >> ~/.bashrc

# ROS Domain ID (used to isolate namespaces in multi-robot setups)
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc

# Network interface (replace with your actual interface, e.g., eth0, wlan0)
echo "export DDS_INTERFACE=eth0" >> ~/.bashrc
```

### Container launch alias — x86 Desktop

```bash
echo "alias irdl_enter='cd /home/${USER}/ros_ws/src/irdl_tutorial/docker && bash run_x86.sh'" >> ~/.bashrc
```

### Container launch alias — Mac (Apple Silicon)

```bash
echo "alias irdl_enter='cd /home/${USER}/ros_ws/src/irdl_tutorial/docker && bash run_mac.sh'" >> ~/.bashrc
```

> **GUI on Mac (RViz2)**: See the setup steps below. Mac Docker Desktop runs containers inside a Linux VM, so the standard `/tmp/.X11-unix` socket mount does not work. TCP-based X11 forwarding via XQuartz is required.

### Container launch alias — Jetson

```bash
echo "alias irdl_enter='cd /home/${USER}/ros_ws/src/irdl_tutorial/docker && bash run_jetson.sh'" >> ~/.bashrc
```

Apply changes:

```bash
source ~/.bashrc
```

> **How to check your network interface name**:
>
> ```bash
> ip link show
> ```
>
> Look for the active interface (e.g., `eth0`, `ens3`, `wlan0`) and update `DDS_INTERFACE` accordingly.

---

## 5. Run the Container

```bash
irdl_enter
```

The script handles all container states automatically:

| State | Behavior |
| --- | --- |
| Container does not exist | Creates and starts a new container |
| Container exists but is stopped | Restarts and attaches |
| Container is already running | Opens a new shell in the running container |

The working directory inside the container is `/ros_ws`, which is mounted from `/home/${USER}/ros_ws` on the host.

---

## 6. Build Packages

After entering the container, build all packages:

```bash
irdl_enter
cd /ros_ws/src/irdl_tutorial
./build.sh
```

To build a specific package only:

```bash
irdl_enter
cd /ros_ws
colcon build --packages-up-to example_cpp_node
```

---

## 7. GUI on Mac (RViz2 / X11)

Mac Docker Desktop runs containers inside a Linux VM, so mounting `/tmp/.X11-unix` does not work.
Instead, `run_mac.sh` uses TCP-based X11 via XQuartz (`DISPLAY=host.docker.internal:0`).

### Setup (one-time)

**Step 1** — Install XQuartz:

```bash
brew install --cask xquartz
```

**Step 2** — Enable network connections in XQuartz:

Open XQuartz → **Preferences** → **Security** tab → check **"Allow connections from network clients"**

Then **quit and relaunch XQuartz** for the setting to take effect.

**Step 3** — Allow the container to connect (run this every time after XQuartz starts):

```bash
xhost +localhost
```

**Step 4** — Start the container and launch RViz2:

```bash
irdl_enter
rviz2
```

> **Note**: `xhost +localhost` must be re-run each time XQuartz restarts. You can add it to your Mac login items or shell profile to automate this.

---

## 8. Troubleshooting

### `permission denied` when running Docker

```bash
sudo usermod -aG docker $USER
# Log out and log back in
```

### `Cannot connect to the X server` (GUI applications)

```bash
xhost +local:docker
```

### Container name conflict

```bash
docker rm irdl-container-all
```

### Check running containers

```bash
docker ps -a
```

### Check available images

```bash
docker images
```
