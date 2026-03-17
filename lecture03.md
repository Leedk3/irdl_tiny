---
marp: true
theme: default
paginate: true
footer: "방산창의공학설계 | Dept. of Advanced Defense Technology and Industry | 이대규 교수"
style: |
  section {
    justify-content: flex-start;
    padding: 10px 10px 10px 10px;
  }

  footer {
    position: absolute;
    bottom: 20px;
    left: 0;
    right: 0;
    text-align: center;
    font-size: 10pt;
    color: #666666;
  }

  h1 {
    font-size: 28pt;
    color: #000000;
    margin-top: 0;
    margin-bottom: 20px;
    text-align: left;
    width: 100%;
  }

  h2 {
    color: #4f81bd;
    font-size: 20pt;
    margin-top: 0;
    margin-bottom: 10px;
  }

  p, li {
    font-size: 18pt;
    line-height: 1.4;
  }

  pre, code {
    font-size: 14pt;
  }

  table {
    font-size: 15pt;
    width: 100%;
  }
---

# `Week 3: Dev Environment & ROS2 Communication`

## Docker Setup · Publisher · Subscriber · RViz2 Visualization

**Department of Advanced Defense Technology and Industry**

---

# `1. Course Objectives for Today`

**From zero environment to a running ROS2 visualization.**

- **Docker:** Understand why we use containers and how to build/run one.
- **Publisher (talker_node):** Write a node that broadcasts messages on a topic.
- **Subscriber (listener_node):** Write a node that receives and reacts to messages.
- **RViz2 (rviz_marker_node):** Visualize objects in 3D using markers.
- **AD Visualization (ad_viz_node):** Simulate a self-driving vehicle with obstacle awareness.

> **Goal:** By the end of class, you will see a vehicle navigating a circular path with color-coded obstacle alerts — driven entirely by ROS2 nodes you understand.

---

# `2. Why Docker?`

## The "Works on My Machine" Problem

Without a shared environment, software behaves differently on every PC.

| Without Docker | With Docker |
| --- | --- |
| Each student installs dependencies manually | One Dockerfile defines the entire environment |
| Version conflicts between OS / ROS / CUDA | All students run the identical image |
| "It works on my machine" | Identical behavior on x86, Mac, Jetson |
| Hours of setup per student | `./build_docker.sh` once, then done |

> Docker packages the OS, libraries, and your code into a **portable container image**.

---

# `3. Docker Core Concepts`

## Image vs. Container

```text
   Dockerfile  ──build──►  Image  ──run──►  Container
   (recipe)                (snapshot)        (running process)
```

- **Dockerfile:** A script that describes how to build an image (base OS, packages, env vars).
- **Image:** A read-only snapshot created from the Dockerfile. Shared via registry.
- **Container:** A live, running instance of an image. Multiple containers can run from one image.
- **Volume (`-v`):** Mounts a host directory into the container so files persist after the container stops.

> Think of an Image as a **class definition** and a Container as an **object instance**.

---

# `4. Our Docker Image Contents`

## What is pre-installed in `irdl-tiny-image`

| Component | Version | Purpose |
| --- | --- | --- |
| Ubuntu | 22.04 | Base OS |
| ROS2 | Humble | Robot middleware |
| CUDA + cuDNN | 12.1 + 8 | GPU acceleration (x86) |
| PyTorch | 2.1.0 | Deep learning |
| OpenCV | 4.x | Image processing |
| CycloneDDS | latest | ROS2 communication layer |
| ZeroMQ | latest | Inter-process messaging |

> Apple Silicon (Mac) image uses **CPU-only PyTorch** and **apt OpenCV** — no CUDA.

---

# `5. Step 1 — Install Docker`

## Setting up the host machine

```bash
# Remove old Docker versions (if any)
sudo apt-get remove docker docker-engine docker.io containerd runc

# Install via official script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Allow current user to run Docker without sudo
sudo usermod -aG docker $USER
# ⚠ Log out and log back in for this to take effect
```

Verify:

```bash
docker --version
# Docker version 24.x.x, build ...
```

> **Mac users:** Install Docker Desktop for Mac — no command-line install needed.

---

# `6. Step 2 — NVIDIA Container Toolkit (Linux/Jetson only)`

## Enabling GPU pass-through into the container

```bash
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Verify:

```bash
docker run --rm --gpus all nvidia/cuda:12.1.1-base-ubuntu22.04 nvidia-smi
```

> **Mac users:** Skip this step entirely. Apple Silicon has no NVIDIA GPU.

---

# `7. Step 3 — Clone & Build the Image`

## Getting the source and building the Docker image

```bash
# Create workspace and clone
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone --recursive https://github.com/Leedk3/irdl_tiny.git
cd irdl_tiny && git checkout main

# Build image (choose your platform)
cd docker
./build_docker.sh x86     # Intel/AMD desktop with NVIDIA GPU
./build_docker.sh mac     # Apple Silicon (M1/M2/M3/M4/M5)
./build_docker.sh ros2    # NVIDIA Jetson
```

> First build installs everything from scratch.
> **x86:** ~30–60 min (OpenCV CUDA compilation)
> **Mac:** ~10–20 min (OpenCV via apt, no compilation)

---

# `8. Step 4 — Shell Setup & Running the Container`

## Aliases and environment variables

```bash
# ROS2 DDS middleware
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
echo "export DDS_INTERFACE=eth0" >> ~/.bashrc   # ← change to your NIC

# Container launch alias (choose one)
# x86:
echo "alias irdl_enter='cd ~/ros_ws/src/irdl_tiny/docker && bash run_x86.sh'" >> ~/.bashrc
# Mac:
echo "alias irdl_enter='cd ~/ros_ws/src/irdl_tiny/docker && bash run_mac.sh'" >> ~/.bashrc

source ~/.bashrc
irdl_enter   # enter the container
```

Inside the container, your workspace is `/ros_ws`.
The host directory `~/ros_ws` is **mounted** — edits on either side are instantly visible.

---

# `8-A. Mac-Specific Docker Notes`

## Known issues and fixes on Apple Silicon

| Issue | Cause | Fix |
| --- | --- | --- |
| `mkdir /home/user: not supported` | Mac home is `/Users/`, not `/home/` | `run_mac.sh` uses `${HOME}` |
| `Mounts denied: /home/...` | Docker Desktop only shares `/Users` | Fixed by using `${HOME}/ros_ws/bag` |
| `/bin/zsh: not found` | Container has no zsh | `run_mac.sh` uses `/bin/bash` |
| `I have no name!` | Mac `/etc/passwd` has no container user | Removed `/etc/passwd` bind-mount |

**Build the image on your Mac** so the container user matches your username:

```bash
cd ~/ros_ws/src/irdl_tiny/docker
./build_docker.sh mac
```

> Always rebuild the image after pulling updates to `run_mac.sh` or Dockerfiles.

---

# `9. What is ROS2?`

## A quick recap before we write code

**ROS2 (Robot Operating System 2)** is a middleware framework that lets independent programs (nodes) communicate through a standardized message-passing system.

```text
  [talker_node]  ──/chatter──►  [listener_node]
  publishes msg                  receives msg
```

Key concepts:

- **Node:** A single executable process (one class, one purpose).
- **Topic:** A named communication channel. Nodes publish to or subscribe from topics.
- **Message:** A typed data structure sent over a topic (e.g., `std_msgs/String`).
- **QoS:** Quality of Service — controls reliability, history depth, durability.

---

# `10. ROS2 Node Anatomy (C++)`

## The skeleton every node follows

```cpp
// 1. Include the node base class
#include <rclcpp/rclcpp.hpp>

// 2. Inherit from rclcpp::Node
class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    // 3. Create publishers, subscribers, timers here
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MyNode::timerCallback, this));
  }
private:
  void timerCallback() { /* runs every 100 ms */ }
  rclcpp::TimerBase::SharedPtr timer_;
};

// 4. Spin the node
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
}
```

---

# `11. talker_node — The Publisher`

## Broadcasting a message on a topic at a fixed rate

**What it does:** Publishes a `std_msgs/String` on `/chatter` every second.

```cpp
// Create publisher: topic name, queue depth
pub_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

// Create 1 Hz timer
timer_ = this->create_wall_timer(
  std::chrono::milliseconds(1000),
  std::bind(&Talker::timerCallback, this));
```

Each timer tick:

```cpp
void Talker::timerCallback() {
  std_msgs::msg::String msg;
  msg.data = m_message + " [" + std::to_string(m_count++) + "]";
  pubChatter->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
}
```

> `RCLCPP_INFO` prints to the terminal — equivalent to `printf` but timestamped and filterable.

---

# `12. talker_node — Parameters & Launch`

## Making behavior configurable without recompiling

`config/config.yaml`:

```yaml
/**:
  ros__parameters:
    message: "Hello, ROS2!"   # change this string without recompiling
```

Declaring and reading in C++:

```cpp
this->declare_parameter<std::string>("message", "Hello, ROS2!");
m_message = this->get_parameter("message").as_string();
```

Running:

```bash
# Via launch file (loads config.yaml automatically)
ros2 launch talker_node deploy.launch.py
```

---

# `13. listener_node — The Subscriber`

## Reacting to every incoming message

**What it does:** Subscribes to `/chatter` and prints each message.

```cpp
// Create subscription: topic, queue depth, callback
sub_ = this->create_subscription<std_msgs::msg::String>(
  "/chatter", 10,
  std::bind(&Listener::chatterCallback, this, std::placeholders::_1));
```

Callback — called automatically when a message arrives:

```cpp
void Listener::chatterCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg->data.c_str());
}
```

> `SharedPtr msg` is a shared pointer. Access fields with `msg->data`, not `msg.data`.

---

# `14. Running Publisher + Subscriber Together`

## Open two terminals inside the container

**Terminal 1 — start the publisher:**

```bash
irdl_enter
ros2 launch talker_node deploy.launch.py
```

**Terminal 2 — start the subscriber:**

```bash
irdl_enter
ros2 launch listener_node deploy.launch.py
```

Expected output in Terminal 2:

```text
[listener_node]: Heard: 'Hello, ROS2! [0]'
[listener_node]: Heard: 'Hello, ROS2! [1]'
[listener_node]: Heard: 'Hello, ROS2! [2]'
```

**Inspect the topic (Terminal 3):**

```bash
ros2 topic list           # shows /chatter
ros2 topic echo /chatter  # prints every message live
ros2 topic hz   /chatter  # measures actual publish rate
```

---

# `15. Build Workflow`

## Compile and activate packages

```bash
# Inside the container
cd /ros_ws/src/irdl_tiny
./build.sh
```

`build.sh` runs `colcon build` with the correct flags. After every build, **source the install directory** in your current shell:

```bash
source /ros_ws/install/setup.bash
```

> ⚠ Without sourcing, `ros2 run <package>` tab-completion won't find new packages.
> The build script runs in a subshell — its `source` does not affect your shell.

Convenience alias (add to `~/.bashrc`):

```bash
alias cb='cd /ros_ws/src/irdl_tiny && ./build.sh && source /ros_ws/install/setup.bash'
```

---

# `16. What is RViz2?`

## The standard 3D visualization tool for ROS2

**RViz2** is a real-time 3D display that renders ROS2 topics as visual objects.

| Display Type | Message Type | Use |
| --- | --- | --- |
| Marker / MarkerArray | `visualization_msgs` | Custom 3D shapes |
| PointCloud2 | `sensor_msgs` | LiDAR point clouds |
| Path | `nav_msgs` | Planned route |
| TF | — | Coordinate frame tree |

**How it works:**

1. A node **publishes** a `Marker` or `MarkerArray` message.
2. RViz2 **subscribes** to that topic and draws the shapes in 3D space.
3. You choose a **Fixed Frame** — all positions are relative to this frame.

---

# `17. rviz_marker_node — Basic Marker`

## Publishing a single 3D cube at the origin

**What it does:** Publishes a `visualization_msgs/Marker` (blue cube) on `/visualization_marker` at 10 Hz.

```cpp
visualization_msgs::msg::Marker marker;
marker.header.frame_id = m_frame_id;   // "map"
marker.type   = visualization_msgs::msg::Marker::CUBE;
marker.action = visualization_msgs::msg::Marker::ADD;
marker.scale.x = m_scale;   // 1.0 m
marker.scale.y = m_scale;
marker.scale.z = m_scale;
marker.color.r = 0.0;  marker.color.g = 0.5;
marker.color.b = 1.0;  marker.color.a = 1.0;  // blue
```

`config/config.yaml`:

```yaml
/**:
  ros__parameters:
    frame_id: "map"
    scale: 1.0
```

---

# `18. Running rviz_marker_node`

## Step-by-step demo

**Step 1 — Launch the marker publisher:**

```bash
irdl_enter
ros2 launch rviz_marker_node deploy.launch.py
```

**Step 2 — Open RViz2:**

```bash
rviz2   # in another terminal inside the container
```

**Step 3 — Configure RViz2:**

1. **Global Options → Fixed Frame:** type `map`
2. Click **Add** → **By topic** → `/visualization_marker` → **Marker** → OK

You should see a blue cube at the origin.

> **Mac users:** Run `xhost +localhost` on the Mac host before starting RViz2.
> If RViz2 fails to open, set `LIBGL_ALWAYS_SOFTWARE=1` inside the container.

---

# `19. ad_viz_node — Autonomous Driving Visualization`

## Simulating a vehicle navigating with obstacle awareness

**Published topics:**

| Topic | Type | Content |
| --- | --- | --- |
| `/ad/vehicle` | `Marker` | Yellow arrow showing vehicle pose |
| `/ad/global_path` | `Path` | Pre-planned circular route |
| `/ad/safety_circle` | `Marker` | Transparent caution radius |
| `/ad/obstacles` | `MarkerArray` | Obstacles + distance labels |
| TF `map→base_link` | — | Live coordinate frame |

**Obstacle color logic (distance to vehicle):**

| Color | Distance | Meaning |
| --- | --- | --- |
| 🟢 Green | > 6 m | Safe |
| 🟡 Yellow | 3 – 6 m | Caution |
| 🔴 Red | < 3 m | Danger |

---

# `20. ad_viz_node — Key Code`

## Vehicle motion and obstacle coloring

```cpp
void AdViz::timerCallback()
{
  m_t += m_speed;                          // advance angle each tick
  double vx  = m_radius * std::cos(m_t);  // x = r·cos(θ)
  double vy  = m_radius * std::sin(m_t);  // y = r·sin(θ)
  double yaw = m_t + M_PI / 2.0;          // tangent = heading direction

  pubVehicle->publish(makeVehicleMarker(vx, vy, yaw));
  pubObstacles->publish(makeObstacleMarkers(vx, vy));
  broadcastTF(vx, vy, yaw);
}
```

Obstacle coloring:

```cpp
double dist = std::sqrt(std::pow(vx-ox,2) + std::pow(vy-oy,2));
if      (dist < m_danger_dist)  { r=1; g=0; b=0; }  // RED
else if (dist < m_caution_dist) { r=1; g=1; b=0; }  // YELLOW
else                            { r=0; g=1; b=0; }  // GREEN
```

---

# `21. Running ad_viz_node`

## Full autonomous driving visualization

**Step 1 — Launch the node:**

```bash
irdl_enter
ros2 launch ad_viz_node deploy.launch.py
```

**Step 2 — Open RViz2 and add displays:**

```bash
rviz2   # another terminal
```

| Add | Type | Topic |
| --- | --- | --- |
| Fixed Frame | — | `map` |
| Path | Path | `/ad/global_path` |
| Marker | Marker | `/ad/vehicle` |
| Marker | Marker | `/ad/safety_circle` |
| MarkerArray | MarkerArray | `/ad/obstacles` |
| TF | — | — |

> Watch obstacle colors change from green → yellow → red as the vehicle approaches.

---

# `22. ad_viz_node — Parameters`

## Tuning the simulation without recompiling

`config/config.yaml`:

```yaml
/**:
  ros__parameters:
    radius: 8.0        # [m]   circular path radius
    speed: 0.01        # [rad/step] angular speed (step = 50 ms)
    danger_dist: 3.0   # [m]   distance threshold → RED
    caution_dist: 6.0  # [m]   distance threshold → YELLOW
```

Try changing these values and observe the effect:

| Parameter | Effect |
| --- | --- |
| `radius` | Larger path circle — some obstacles may never turn red |
| `speed` | Faster or slower vehicle |
| `danger_dist` | Widens/narrows the red alert zone |
| `caution_dist` | Widens/narrows the yellow caution zone |

---

# `23. Putting It All Together`

## Full system: all nodes running simultaneously

```bash
# Terminal 1 — talker
irdl_enter && ros2 launch talker_node deploy.launch.py

# Terminal 2 — listener
irdl_enter && ros2 launch listener_node deploy.launch.py

# Terminal 3 — AD visualization
irdl_enter && ros2 launch ad_viz_node deploy.launch.py

# Terminal 4 — RViz2
irdl_enter && rviz2
```

Inspect all active topics:

```bash
ros2 topic list
# /chatter  /ad/vehicle  /ad/global_path
# /ad/safety_circle  /ad/obstacles
```

> Each node is fully independent. Killing one does not affect the others.
> This is the fundamental design principle of ROS2.

---

# `24. Lab Exercise`

## Modify and extend the examples

**Exercise 1 — Change the message:**
Edit `talker_node/config/config.yaml`, change `message` to your name. Rebuild and confirm the listener receives the new string.

**Exercise 2 — Add a new obstacle:**
In `ad_viz.cpp`, add a new entry to `m_obstacles` in the constructor. Place it at `{3.0, 3.0}` — directly on the path. Observe it turning red.

**Exercise 3 — Change marker type:**
In `rviz_marker.cpp`, change `CUBE` to `SPHERE` and set color to red (`r=1, g=0, b=0`). Rebuild and verify in RViz2.

```cpp
// Hint: change this line in makeMarker()
marker.type = visualization_msgs::msg::Marker::SPHERE;
marker.color.r = 1.0;  marker.color.g = 0.0;  marker.color.b = 0.0;
```

---

# `25. Assignment #3`

## Build and demonstrate your ROS2 environment

Submit a short report (screenshots + brief explanation) showing:

1. **Docker image built** — screenshot of `docker images` showing `irdl-tiny-image`.
2. **talker + listener running** — screenshot of listener terminal receiving messages.
3. **rviz_marker_node** — screenshot showing the blue cube in RViz2.
4. **ad_viz_node** — screenshot showing the vehicle, path, and color-coded obstacles.
5. **Lab Exercise 2 completed** — screenshot showing new obstacle turning red on the path.

**Due:** Next class session.

> **Hint:** Include a `rqt_graph` screenshot to show the node connections in your report.

---

# `26. Summary`

## What we covered today

| Topic | Key Takeaway |
| --- | --- |
| Docker | Containers eliminate environment inconsistency |
| `build_docker.sh` | One script builds for x86, Mac, or Jetson |
| Build workflow | `./build.sh` → `source install/setup.bash` |
| ROS2 Node | Inherit `rclcpp::Node`, create pub/sub/timer |
| Publisher | `create_publisher<T>()` + timer callback + `publish()` |
| Subscriber | `create_subscription<T>()` + message callback |
| Parameters | Declare in C++, set in `config.yaml`, load via launch |
| RViz2 Markers | Publish `Marker/MarkerArray` → Fixed Frame → Add display |
| ad_viz_node | Vehicle + path + distance-based obstacle coloring |

**Next week:** Coordinate frames, TF2, and transforming between sensor frames.
