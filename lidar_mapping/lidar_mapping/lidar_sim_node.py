"""
lidar_sim_node.py
=================
Simulates a LiDAR sensor mounted on a moving vehicle.

Flow:
  1. Load a 3D scene (PLY file) and place it at the map origin.
  2. Read the vehicle pose from TF  (map → base_link, broadcast by ad_viz_node).
  3. Every 50 ms, find PLY points that are within LiDAR range from the vehicle.
  4. Express those points in the vehicle-local 'base_link' frame.
  5. Publish them as sensor_msgs/PointCloud2  →  /lidar_scan

Students can visualise /lidar_scan in RViz to see what the sensor "sees" at
each moment, then watch /pcd_map (map_builder_node) grow over time.
"""

import sys
import math

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from tf2_ros import Buffer, TransformListener

# ── tuneable constants ────────────────────────────────────────────────────────
LIDAR_RANGE   = 12.0   # [m]  maximum sensor range
SCENE_RADIUS  = 3.0    # [m]  PLY scene is normalised to fit inside this sphere
PUBLISH_HZ    = 20.0   # [Hz] sensor update rate
# ─────────────────────────────────────────────────────────────────────────────


def load_and_normalise(ply_path: str) -> np.ndarray:
    """Load PLY, centre at origin, scale to SCENE_RADIUS."""
    pcd = o3d.io.read_point_cloud(ply_path)
    pts = np.asarray(pcd.points, dtype=np.float64)

    centroid = pts.mean(axis=0)
    pts -= centroid

    max_dist = np.linalg.norm(pts, axis=1).max()
    if max_dist > 0:
        pts *= SCENE_RADIUS / max_dist

    return pts


def quat_to_R(q) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])


def numpy_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> sensor_msgs.PointCloud2:
    itemsize = 4  # float32
    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize,
        datatype=sensor_msgs.PointField.FLOAT32, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=frame_id)
    header.stamp = stamp

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=itemsize * 3,
        row_step=itemsize * 3 * points.shape[0],
        data=points.astype(np.float32).tobytes(),
    )


class LidarSimNode(Node):

    def __init__(self):
        super().__init__('lidar_sim_node')

        # ── load scene ────────────────────────────────────────────────────────
        assert len(sys.argv) > 1, 'Usage: lidar_sim_node <path/to/scene.ply>'
        ply_path = sys.argv[1]
        self.scene_pts = load_and_normalise(ply_path)           # (N, 3) float64
        self.get_logger().info(
            f'Scene loaded: {self.scene_pts.shape[0]} points  '
            f'(normalised to r={SCENE_RADIUS} m)')

        # ── TF ───────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── publisher ────────────────────────────────────────────────────────
        self.pub = self.create_publisher(sensor_msgs.PointCloud2, 'lidar_scan', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)

        self.get_logger().info(
            f'LiDAR sim started  range={LIDAR_RANGE} m  '
            f'→ publishing /lidar_scan in [base_link]')

    # ─────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        # 1. Get vehicle pose (map → base_link)
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception:
            return   # TF not yet available

        t = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])
        R = quat_to_R(tf.transform.rotation)   # R_map_baselink

        # 2. Distance filter: keep scene points within LiDAR range
        diff    = self.scene_pts - t            # (N, 3)
        dists   = np.linalg.norm(diff, axis=1)
        mask    = dists < LIDAR_RANGE
        visible = diff[mask]                    # still in map frame relative to vehicle

        if visible.shape[0] == 0:
            return

        # 3. Rotate into base_link frame  (R^T rotates map→base_link)
        pts_bl = (R.T @ visible.T).T            # (M, 3)

        # 4. Publish
        msg = numpy_to_pointcloud2(
            pts_bl.astype(np.float32), 'base_link',
            self.get_clock().now().to_msg())
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
