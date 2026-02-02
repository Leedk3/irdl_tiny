# 🛡️ irdl_tutorial: Let's learn how to build autonomy system

## 📦 Prerequisites

1. **Install Docker**
2. **Install NVIDIA Container Toolkit**  
   - Guide: [https://velog.io/@boom109/Nvidia-docker](https://velog.io/@boom109/Nvidia-docker)
3. **Create Workspace & Clone Repository**

   ``` $ bash
   $ mkdir -p /home/${USER}/irdl_ws/src
   $ cd /home/${USER}/irdl_ws/src
   $ git clone --recursive https://github.com/Leedk3/irdl_tutorial.git
   $ cd irdl_tutorial
   $ git checkout main
   ```

   *Note*: If you have no SSH keys, you might see the error during recursive updating. More details can be found here : [github SSH keys](https://github.com/settings/keys).


## 🐳 Docker Installation
Docker images are based on Ubuntu 22.04 + ROS2 Humble, preconfigured with necessary libraries and dependencies.

cd /home/${USER}/irdl_ws/src/irdl_tutorial/docker
### For x86 Desktop
```
$ ./build_docker.sh x86
```

### For Jetson
```
$ ./build_docker.sh ros2
```



## ⚙️ Shell Environment Setup
```
# System stop script
$ echo "alias kill_sys='cd /home/${USER}/irdl_ws/src/irdl_tutorial && bash kill_sys.sh'" >> ~/.bashrc

# ROS2 configuration
$ echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
$ echo "export RMW_CYCLONEDDS_URI=file:///home/${USER}/.cyclonedds.xml" >> ~/.bashrc
$ echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
$ echo "export DDS_INTERFACE={your_network_interface, e.g., eth0}" >> ~/.bashrc
```
#### Easy Container run for x86 Desktop 
```
# Docker run aliases
$ echo "alias irdl_enter='cd /home/${USER}/irdl_ws/src/irdl_tutorial/docker && bash run_x86.sh'" >> ~/.bashrc  # x86
```
#### Easy Container run for Jetson
```
# Docker run aliases
$ echo "alias irdl_enter='cd /home/${USER}/irdl_ws/src/irdl_tutorial/docker && bash run_jetson.sh'" >> ~/.bashrc  # Jetson
```

#### 📌 Notes
- Be sure to replace {your_network_interface} with your actual network interface (e.g., eth0, wlan0, etc.).
- You can freely change ROS_DOMAIN_ID based on your multi-robot configuration or testing environment.


## 🔨 Build Instructions
```
$ irdl_enter
$ cd /irdl_ws/src/irdl_tutorial
$ ./build.sh
```

### (Optional) Build Specific Package with colcon
```
$ irdl_enter
$ cd /irdl_ws
$ colcon build --packages-up-to example_cpp_node
```



