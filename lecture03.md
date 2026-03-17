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

## Docker Setup · Publisher · Subscriber · RViz2

**Department of Advanced Defense Technology and Industry**

---

# `1. Course Objectives for Today`

**From zero environment to a running ROS2 visualization.**

- **Docker:** Understand why we use containers and how to build/run one.
- **Publisher (talker_node):** Write a node that broadcasts messages on a topic.
- **Subscriber (listener_node):** Write a node that receives and reacts to messages.
- **RViz2 (rviz_marker_node):** Visualize robot world objects in 3D using markers.

> **Goal:** By the end of class, you will have a blue sphere orbiting an orange cylinder on your screen — driven entirely by code you understand.

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
mkdir -p /home/${USER}/ros_ws/src
cd /home/${USER}/ros_ws/src
git clone --recursive https://github.com/Leedk3/irdl_tutorial.git
cd irdl_tutorial && git checkout main

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
echo "alias irdl_enter='cd ~/ros_ws/src/irdl_tutorial/docker && bash run_x86.sh'" >> ~/.bashrc
# Mac:
echo "alias irdl_enter='cd ~/ros_ws/src/irdl_tutorial/docker && bash run_mac.sh'" >> ~/.bashrc

source ~/.bashrc
irdl_enter   # enter the container
```

Inside the container, your workspace is `/ros_ws`.
The host directory `~/ros_ws` is **mounted** — edits on either side are instantly visible.

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
  msg.data = "Hello, ROS2! count = " + std::to_string(count_++);
  pub_->publish(msg);
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
    publish_rate: 1.0   # Hz — change this to speed up or slow down
```

Declaring and reading in C++:

```cpp
this->declare_parameter<double>("publish_rate", 1.0);
publish_rate_ = this->get_parameter("publish_rate").as_double();
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
  std::bind(&Listener::msgCallback, this, std::placeholders::_1));
```

Callback — called automatically when a message arrives:

```cpp
void Listener::msgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
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
[listener_node]: Received: 'Hello, ROS2! count = 0'
[listener_node]: Received: 'Hello, ROS2! count = 1'
[listener_node]: Received: 'Hello, ROS2! count = 2'
```

**Inspect the topic (Terminal 3):**

```bash
ros2 topic list           # shows /chatter
ros2 topic echo /chatter  # prints every message live
ros2 topic hz   /chatter  # measures actual publish rate
```

---

# `15. Visualizing the Node Graph`

## rqt_graph — seeing who talks to whom

```bash
rqt_graph
```

You will see:

```text
  [/talker_node]  ──/chatter──►  [/listener_node]
```

This graph updates live as nodes start and stop. It is the fastest way to debug:

- Is my publisher actually running?
- Is the subscriber on the right topic name?
- Are there unexpected extra publishers?

> **Common bug:** Topic name typo — publisher on `/Chatter`, subscriber on `/chatter`. `rqt_graph` reveals this instantly.

---

# `16. What is RViz2?`

## The standard 3D visualization tool for ROS2

**RViz2** is a real-time 3D display that renders ROS2 topics as visual objects.

| Display Type | Message Type | Use |
| --- | --- | --- |
| Marker / MarkerArray | `visualization_msgs` | Custom 3D shapes |
| PointCloud2 | `sensor_msgs` | LiDAR point clouds |
| Odometry | `nav_msgs` | Robot pose + path |
| Image | `sensor_msgs` | Camera feed |
| TF | — | Coordinate frame tree |

**How it works:**

1. A node **publishes** a `MarkerArray` message.
2. RViz2 **subscribes** to that topic and draws the markers in 3D space.
3. You choose a **Fixed Frame** — all positions are relative to this frame.

---

# `17. rviz_marker_node — What it Publishes`

## Two objects: an orbiting sphere and a static cylinder

```text
           ↑ z
           |
           |   ● (blue sphere, orbits at radius 2 m)
           |  /
           | /  angle increases over time
           |/___________► x
        🔶 (orange cylinder at origin)
```

- Published on topic: `/visualization_marker_array`
- Frame: `map`
- Rate: 20 Hz

**Objects:**

- **Sphere** (`SPHERE` type) — position updates every tick based on `angle_`
- **Cylinder** (`CYLINDER` type) — position fixed at `(0, 0, 0.25)`

---

# `18. rviz_marker_node — Key Code`

## How the orbit position is computed each tick

```cpp
void RvizMarker::timerCallback()
{
  // Advance angle: full circle (2π) divided over orbit_period seconds
  angle_ += (2.0 * M_PI / orbit_period_) / publish_rate_;
  if (angle_ >= 2.0 * M_PI) angle_ -= 2.0 * M_PI;

  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(makeOrbitSphere(angle_));
  array.markers.push_back(makeOriginCylinder());
  pub_->publish(array);
}

visualization_msgs::msg::Marker RvizMarker::makeOrbitSphere(double angle)
{
  // Parametric circle: x = r·cos(θ),  y = r·sin(θ)
  marker.pose.position.x = orbit_radius_ * std::cos(angle);
  marker.pose.position.y = orbit_radius_ * std::sin(angle);
  marker.pose.position.z = 0.5;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  // color: blue  (r=0.2, g=0.6, b=1.0, a=1.0)
}
```

---

# `19. Running rviz_marker_node`

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
2. Click **Add** (bottom-left) → **By topic** → `/visualization_marker_array` → **MarkerArray** → OK

You should now see the orange cylinder at the origin and the blue sphere orbiting around it.

> **Mac users:** Run `xhost +localhost` in a Mac terminal before starting the container.

---

# `20. rviz_marker_node — Parameters`

## Tuning the visualization without changing code

`config/config.yaml`:

```yaml
/**:
  ros__parameters:
    publish_rate: 20.0   # [Hz]  how often the marker is updated
    orbit_radius: 2.0    # [m]   how far the sphere orbits from center
    orbit_period: 5.0    # [s]   time for one full revolution
```

Try changing these values and rebuilding to see the effect:

| Parameter | Small value | Large value |
| --- | --- | --- |
| `orbit_radius` | Tight orbit near origin | Wide orbit |
| `orbit_period` | Fast spinning | Slow spinning |
| `publish_rate` | Choppy animation | Smooth animation |

```bash
cd /ros_ws/src/irdl_tutorial && ./build.sh
ros2 launch rviz_marker_node deploy.launch.py
```

---

# `21. Putting It All Together`

## Full system: talker → listener + marker visualization

Run all three nodes simultaneously (three terminals):

```bash
# Terminal 1
irdl_enter && ros2 launch talker_node deploy.launch.py

# Terminal 2
irdl_enter && ros2 launch listener_node deploy.launch.py

# Terminal 3
irdl_enter && ros2 launch rviz_marker_node deploy.launch.py

# Terminal 4 — visualize everything
irdl_enter && rviz2
```

Inspect the full topic map:

```bash
ros2 topic list
# /chatter
# /visualization_marker_array
```

> Each node is fully independent. Killing one does not affect the others.
> This is the fundamental design principle of ROS2.

---

# `22. Lab Exercise`

## Modify and extend the examples

**Exercise 1 — Change publish rate:**
Edit `talker_node/config/config.yaml`, change `publish_rate` to `5.0`. Rebuild and confirm the listener receives 5 messages per second using `ros2 topic hz /chatter`.

**Exercise 2 — Change the orbit:**
Edit `rviz_marker_node/config/config.yaml`. Set `orbit_radius: 4.0` and `orbit_period: 2.0`. Observe the result in RViz2.

**Exercise 3 — Add a second marker:**
In `rviz_marker.cpp`, add a third marker — a red `CUBE` fixed at `(3.0, 0.0, 0.5)` and push it to `marker_array.markers`.

```cpp
// Hint: copy makeOriginCylinder(), change type and color
marker.type    = visualization_msgs::msg::Marker::CUBE;
marker.color.r = 1.0f;
marker.color.g = 0.0f;
marker.color.b = 0.0f;
```

---

# `23. Assignment #3`

## Build and demonstrate your ROS2 environment

Submit a short report (screenshots + brief explanation) showing:

1. **Docker image built** — screenshot of `docker images` showing `irdl-tiny-image`.
2. **talker + listener running** — screenshot of listener terminal receiving messages.
3. **RViz2 with rviz_marker_node** — screenshot of the orbiting sphere and cylinder.
4. **Lab Exercise 3 completed** — screenshot showing the red cube at `(3, 0, 0.5)`.

**Due:** Next class session.

> **Hint:** Include a `rqt_graph` screenshot to show the node connections in your report.

---

# `24. Summary`

## What we covered today

| Topic | Key Takeaway |
| --- | --- |
| Docker | Containers eliminate environment inconsistency |
| `build_docker.sh` | One script builds for x86, Mac, or Jetson |
| ROS2 Node | Inherit `rclcpp::Node`, create pub/sub/timer |
| Publisher | `create_publisher<T>()` + timer callback + `publish()` |
| Subscriber | `create_subscription<T>()` + message callback |
| Parameters | Declare in C++, set in `config.yaml`, load via launch |
| RViz2 Markers | Publish `MarkerArray` → Fixed Frame → Add display |

**Next week:** Coordinate frames, TF2, and transforming between sensor frames.
