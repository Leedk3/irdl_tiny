---
marp: true
theme: default
paginate: true
footer: "방산창의공학설계 | Dept. of Advanced Defense Technology and Industry | 이대규 교수"
style: |
  section {
    justify-content: flex-start;
    padding: 40px 50px 40px 50px;
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

# `Week 4: LiDAR Characteristics · TF2 · GICP & Clustering`

## PointCloud2 · LiDAR Perception Pipeline · KNN · Clustering

**Department of Advanced Defense Technology and Industry**

---

# `1. Course Objectives for Today`

**From raw LiDAR pulses to structured environment understanding.**

- **LiDAR Characteristics:** Understand the physical properties and data structure of LiDAR.
- **Perception Pipeline:** Learn how 3D data is processed step-by-step.
- **PointCloud2 & open3d:** Handle binary 3D messages in ROS2 and Python.
- **TF2 & Fast-GICP:** Understand coordinate frames and align point clouds.
- **Clustering & KNN:** Group distinct objects using K-Nearest Neighbors.

> **Goal:** By the end of class, you will understand how to acquire raw LiDAR data, downsample it, align it, and extract meaningful object clusters.

---

# `2. What is a Point Cloud?`

## Representing 3D geometry as a set of points

A **point cloud** is a collection of 3D coordinates — each point is a measurement of a surface in space.

```text
  LiDAR sensor fires laser pulses in all directions
  Each pulse returns a (x, y, z) coordinate where it hit a surface
  Thousands to millions of such points per second → point cloud
```

| Sensor | Output | Use |
| :--- | :--- | :--- |
| Velodyne / Ouster / Livox | 3D point cloud | Autonomous driving, mapping |
| Intel RealSense / ZED | RGBD point cloud | Manipulation, indoor nav |
| Simulation (Gazebo) | Synthetic cloud | Algorithm testing |
| .ply / .pcd file | Offline format | Dataset, 3D model |

---

# `3. LiDAR Data Characteristics`

## Why is LiDAR data unique and challenging?

Unlike 2D camera images (which are dense, ordered grids of pixels), LiDAR point clouds have distinct properties:

* **High Precision 3D Depth:** Direct distance measurement. Immune to lighting conditions (works perfectly in complete darkness).
* **Sparsity (희소성):** Point density drops significantly as distance increases. An object at 50m has far fewer points than the same object at 5m.
* **Unordered & Unstructured:** Just a 1D array of $(x, y, z)$ points. There is no concept of "adjacent pixel" without mathematical search algorithms.
* **Intensity (반사도):** Besides $(x,y,z)$, LiDAR returns 'intensity'—how strongly the laser bounced back. Highly reflective materials (like lane markings or street signs) appear brighter.

---

# `4. How to Utilize LiDAR: The Perception Pipeline`

## The standard workflow in Autonomous Systems

Because raw LiDAR data is heavy and unstructured, we process it in stages:

1.  **Acquisition:** Read `sensor_msgs/PointCloud2` from the hardware driver.
2.  **Pre-processing (Filtering):**
    * *Crop Box:* Remove points outside the Region of Interest (ROI).
    * *Voxel Grid:* Downsample the cloud to reduce computation time.
3.  **Ground Segmentation:** Separate the drivable surface (ground) from obstacles using algorithms like RANSAC.
4.  **Clustering:** Group the remaining obstacle points into individual objects (Cars, Pedestrians) using KD-Trees and Euclidean distance.
5.  **Tracking & Alignment:** Track objects over time or use GICP to align current scans with a global map.

---

# `5. Pre-processing: Voxel Grid Filter`

## Downsampling to make algorithms run in real-time

Raw LiDAR can produce 300,000+ points per scan. Running $O(N \log N)$ algorithms on this is too slow. A voxel filter keeps the shape but reduces $N$.

```python
def voxel_filter(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep one representative point per 3D voxel cell."""
    # Compute integer grid index for each point
    keys = np.floor(points / voxel_size).astype(np.int32)

    # Use a dict: key = (ix, iy, iz), value = point index
    voxel_map = {}
    for i, k in enumerate(map(tuple, keys)):
        voxel_map[k] = i  # Overwrites duplicates in the same cell

    return points[list(voxel_map.values())]
```

| VOXEL_SIZE | Point Count | Computation Speed | Shape Preservation |
| :--- | :--- | :--- | :--- |
| 0.05 m | High | Slow | Excellent |
| 0.20 m | Medium | Fast ✓ | Good |
| 0.50 m | Low | Very Fast | Poor (Objects lose shape) |

---

# `6. sensor_msgs/PointCloud2 — The Message Format`

## Binary-packed, schema-flexible 3D data

In ROS2, `PointCloud2` stores all points as a raw byte array. The `fields` list tells you how to interpret each byte.

```yaml
  fields:  [ {name:"x", offset:0, type:FLOAT32},
             {name:"y", offset:4, type:FLOAT32},
             {name:"z", offset:8, type:FLOAT32},
             {name:"intensity", offset:16, type:FLOAT32} ]
  point_step: 32   # bytes per point
  row_step:   N×32 # bytes per row
```

| Field | Meaning |
| :--- | :--- |
| `header.frame_id` | Which coordinate frame the points are expressed in |
| `header.stamp` | Timestamp — critical for TF lookup and synchronisation |
| `is_dense` | True if no NaN values in the cloud |

---

# `7. open3d — Working with 3D Files in Python`

## The go-to library for 3D geometry

`open3d` provides Python APIs for loading, processing, and visualising point clouds.

```python
import open3d as o3d
import numpy as np

# Load a .ply or .pcd file
pcd = o3d.io.read_point_cloud("teapot.ply")

# Convert open3d format to numpy array (N × 3)
pts = np.asarray(pcd.points)
print(pts.shape)   # e.g. (3000, 3)

# Centre and normalise to unit sphere
pts -= pts.mean(axis=0)                        # translate to origin
pts /= np.linalg.norm(pts, axis=1).max()       # scale to radius 1
```

> `open3d` also natively supports the Voxel Downsampling and DBSCAN clustering we discussed in the pipeline.

---

# `8. Coordinate Frames & TF2`

## "In which frame are these coordinates?"

Every measurement in robotics has meaning only relative to a frame of reference.

```text
  world / map
  └── base_link           (vehicle body centre)
      └── lidar_link      (sensor mounted on vehicle roof)
```

**TF2 is the ROS2 library that tracks transforms:**
* **Broadcast:** Publish the physical mounting position (`base_link` $\rightarrow$ `lidar_link`).
* **Lookup:** Query the transform to align sensor data with the global coordinate system.

```python
from tf2_ros import Buffer, TransformListener

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# Where is base_link in the map frame right now?
tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
```

---

# `9. Mathematical Representation of Transforms`

## Rigid Body Transformation (Translation + Rotation)

To transform a point from a source frame to a target frame, we use a Transformation Matrix $T$. This matrix combines a $3 \times 3$ Rotation matrix $R$ and a $3 \times 1$ Translation vector $t$.

Using Homogeneous Coordinates, this is represented as a $4 \times 4$ matrix:

$$
\begin{bmatrix} x_{target} \\ y_{target} \\ z_{target} \\ 1 \end{bmatrix} = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} x_{source} \\ y_{source} \\ z_{source} \\ 1 \end{bmatrix}
$$

In Python (Numpy), applying this to an array of points $P_{source}$ (shape $N \times 3$):

```python
# p_target = R * p_source + t
pts_target = (R @ pts_source.T).T + t
```

> **Why Homogeneous?** It allows us to apply both rotation and translation using a single matrix multiplication, which is highly efficient for millions of points.

---

# `10. Generalized Iterative Closest Point (GICP) 상세 분석 1`

## 1. 표준 ICP 및 거리 측정(Metric) 방식

ICP 알고리즘은 두 점군 Source($A$)와 Target($B$) 사이의 최적의 회전 변환($R$)과 병진 변환($t$)을 찾아냅니다.

**A. Point-to-Point ICP**
대응되는 두 점 사이의 유클리디안 거리(Euclidean Distance)를 최소화합니다.
* **목적 함수:** $E = \sum_{i} || R a_i + t - b_i ||^2$
* **특징:** 구현이 간단하지만, 실제 환경에서는 수렴 속도가 느리고 지역 최적점(Local Minima)에 빠지기 쉽습니다.

**B. Point-to-Line / Point-to-Plane ICP**
Target 점 $b_i$에서의 법선 벡터 $n_i$를 구하고, 선이나 평면까지의 '수직 거리'를 최소화합니다.
* **목적 함수:** $E = \sum_{i} || (R a_i + t - b_i) \cdot n_i ||^2$
* **특징:** 표면을 따라 미끄러지는(Sliding) 움직임을 허용하여 적은 반복만으로도 빠르게 수렴합니다.

---

# `11. GICP 상세 분석 2`

## 2. GICP (Generalized ICP)의 핵심 개념

GICP는 기존 방식들을 포괄하는 **확률적 모델(Probabilistic Model)**입니다. 점들을 센서 노이즈나 표면 특성이 반영된 **가우시안 분포(Gaussian Distribution)**로 취급합니다.

* Source 점: $a_i \sim \mathcal{N}(\hat{a}_i, C_i^A)$
* Target 점: $b_i \sim \mathcal{N}(\hat{b}_i, C_i^B)$

오차 $d_i = b_i - (R a_i + t)$ 역시 가우시안 분포를 따릅니다.
$$d_i \sim \mathcal{N}(0, C_i^B + R C_i^A R^T)$$

**일반화(Generalization)의 원리:**
* **Point-to-Point:** $C_i^A = 0$, $C_i^B = I$ 로 설정.
* **Point-to-Plane:** $C_i^A = 0$, $C_i^B$를 법선 방향으로 작게 설정.
* **Plane-to-Plane (GICP 고유):** 양쪽 모두의 공분산을 표면 기하학에 맞게 설정하여 노이즈에 강인하고 높은 정합 정확도를 보여줍니다.

---

# `12. GICP 상세 분석 3`

## 3. GICP 알고리즘 수행 과정 (Step-by-Step)

1.  **지역 공분산 계산:** 점군 $A, B$의 이웃 점들에 대해 PCA를 수행하여 공분산 행렬 $C_i^A, C_i^B$를 도출 (표면의 평평함과 법선 방향).
2.  **대응점 탐색:** KD-Tree를 사용하여 변환된 $A$의 각 점에 대해 $B$에서 가장 가까운 대응점(Match)을 탐색.
3.  **오차 함수 구성:** 마할라노비스 거리를 최소화하는 목적 함수 구성.
    $$E(R, t) = \sum_{i} d_i^T (C_i^B + R C_i^A R^T)^{-1} d_i$$
4.  **최적화:** 비선형 최적화 기법(BFGS, Gauss-Newton 등)을 사용하여 $R, t$ 계산.
5.  **수렴 판정:** 오차나 변환 행렬의 변화량이 임계값보다 작아지면 종료, 아니면 업데이트된 $R, t$로 Step 2 반복.

---

# `13. Point Cloud Registration — What is GICP?`

## Finding the transform that aligns two point clouds

**Goal:** Given a source cloud and a target cloud, find the exact matrix $T$ that correctly maps the source onto the target. Useful for LiDAR Odometry and mapping.

* **ICP (Iterative Closest Point):**
    * Find the nearest target point for each source point.
    * Estimate the $T$ matrix that minimises the total distance error.
    * Apply the transform to the source, and repeat until it converges.

* **GICP (Generalised ICP):**
    * Extends ICP by using surface covariances. Instead of treating points as isolated dots, it treats them as small planar surfaces, making alignment much more robust against noise and sparsity.

---

# `14. GICP Registration Demo`

## Aligning a rotating cloud to a static reference in ROS2

```python
import small_gicp
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Synchronise two topics (within 0.1 s of each other)
self.sync = ApproximateTimeSynchronizer(
    [Subscriber(self, PointCloud2, 'pcd_rotating'),
     Subscriber(self, PointCloud2, 'pcd_static')], queue_size=10, slop=0.1)
self.sync.registerCallback(self.callback)

def callback(self, rotating_msg, static_msg):
    source = pointcloud2_to_numpy(rotating_msg)
    target = pointcloud2_to_numpy(static_msg)

    # 1. Calculate the rigid transformation matrix T
    result = small_gicp.align(target, source)
    T = result.T_target_source  # The 4x4 matrix discussed earlier!

    # 2. Apply transform and publish
    aligned = apply_transform(T, source)
    self.pub.publish(numpy_to_pointcloud2(aligned, 'map'))
```

---

# `15. Understanding the Scene: Clustering`

## Separating the environment into individual objects

Once we have filtered our LiDAR scan and removed the ground, we are left with floating points. How does a robot know which points belong to a car vs. a pedestrian?

**Point Cloud Clustering** solves this by grouping nearby points together into distinct objects.

**Core approaches:**
* **Euclidean Clustering:** Group points based strictly on physical distance.
* **Density-Based (DBSCAN):** Group points based on local density.

> **Requirement:** To cluster points efficiently, we must be able to rapidly query *"Which points are close to me?"* $\rightarrow$ This requires KNN.

---

# `16. K-Nearest Neighbors (KNN) & KD-Trees`

## The backbone of spatial search algorithms

KNN finds the $K$ closest data points to a specific query point.

If we have $N$ points, checking the distance to every other point takes $O(N)$ time. For a point cloud with 100,000 points, doing this for every point takes $O(N^2)$ — far too slow for real-time robotics.

**Solution: KD-Tree (K-Dimensional Tree)**
* A spatial partitioning data structure that divides space into binary trees.
* Reduces search time from $O(N)$ to $O(\log N)$.
* Essential for ICP point matching, radius searches, and clustering.

---

# `17. Euclidean Clustering via KNN`

## Region Growing for Object Extraction

How Euclidean Clustering uses KNN/KD-Trees to isolate objects:

1.  Create an empty list of clusters.
2.  Pick an unassigned point $P$. Create a new cluster $C$.
3.  Use a KD-Tree to find all points within a certain radius $r$ of $P$.
4.  Add these points to $C$.
5.  For every new point added to $C$, repeat the radius search until no new points are found.
6.  Save cluster $C$, and pick the next unassigned point.

**Trade-offs of Search Radius $r$:**
* **Too small:** A single car splits into multiple small clusters.
* **Too large:** A car and a nearby wall merge into a single massive cluster.

---

# `18. System Architecture (Perception Node)`

## How the data flows in our ROS2 system

```text
 ┌──────────────────────────────────────────────────────────┐
 │                       ROS2 Topics                        │
 │                                                          │
 │  sensor_driver ─────► /lidar_raw [lidar_link]            │
 │                                                          │
 │  perception_node ◄─── /lidar_raw                         │
 │                   ──► 1. Voxel Filter (Downsample)       │
 │                   ──► 2. Ground Removal                  │
 │                   ──► 3. KD-Tree + Euclidean Clustering  │
 │                   ──► /clusters [MarkerArray]            │
 │                                                          │
 │  RViz2 ◄───────────── all topics above                   │
 └──────────────────────────────────────────────────────────┘
```

By decoupling the hardware driver from the perception node via ROS2 topics, we can easily test our algorithms on simulated data or real hardware without changing the code.

---

# `19. Key Concepts — Quick Reference`

## Cheat sheet for today's topics

| Concept | One-line summary |
| :--- | :--- |
| **LiDAR Sparsity** | Point density decreases rapidly with distance |
| **PointCloud2** | Binary-packed 3D points; interpret via `fields` metadata |
| **Voxel Filter** | Pre-processing step to downsample points and speed up algorithms |
| **Transform Matrix** | $4 \times 4$ homogeneous matrix combining Rotation & Translation |
| **TF2** | Tracks transforms (`lookup_transform`) between coordinate frames |
| **GICP** | Iterative alignment of two point clouds yielding a $T$ matrix |
| **KD-Tree** | $O(\log N)$ spatial data structure for rapid point search |
| **Clustering** | Grouping points into distinct objects based on proximity radius |
| **ApproximateTimeSync** | Pair ROS2 messages from different topics by timestamp |

---

# `20. Lab Exercise`

## Modify and test the perception algorithms

**Exercise 1 — Observe GICP Alignment:**
Run `gicp_demo.launch.py`. Add all three topics to RViz2. Pause and resume the simulation — observe how the aligned (blue) cloud snaps back to the static reference (green) via transformation matrix updates.

**Exercise 2 — Tune the Voxel Filter:**
Apply the voxel filter to a raw `.ply` file. Change `VOXEL_SIZE` from `0.05` to `0.5`. Print the shape of the numpy array before and after. How much data is discarded?

**Exercise 3 — Clustering Radius:**
Conceptually, if you set the clustering radius $r = 0.1\text{m}$ for a LiDAR scan of a vehicle 30 meters away, what will happen to the resulting cluster? (Hint: Remember LiDAR sparsity characteristics).
