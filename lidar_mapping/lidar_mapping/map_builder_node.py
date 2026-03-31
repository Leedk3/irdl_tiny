"""
map_builder_node.py
===================
Incrementally builds a global point cloud map from LiDAR scans.

Flow:
  1. Subscribe to /lidar_scan  (PointCloud2 in 'base_link' frame).
  2. For each scan, look up TF  (map ← base_link)  to get vehicle pose.
  3. Rotate + translate scan points into the 'map' frame.
  4. Append to the accumulated map, then apply a voxel filter to keep
     the map size manageable.
  5. Publish the growing map as PointCloud2  →  /pcd_map  at 5 Hz.

Watching /pcd_map in RViz, students can see the 3D scene being
reconstructed point by point as the vehicle drives around it.
"""

import math
import struct

import numpy as np

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from tf2_ros import Buffer, TransformListener

# ── tuneable constants ────────────────────────────────────────────────────────
VOXEL_SIZE    = 0.05   # [m]  map resolution (one point kept per voxel)
MAX_POINTS    = 200_000
PUBLISH_HZ    = 5.0
# ─────────────────────────────────────────────────────────────────────────────

_DATATYPES = {
    sensor_msgs.PointField.FLOAT32: ('f', 4),
    sensor_msgs.PointField.FLOAT64: ('d', 8),
    sensor_msgs.PointField.INT8:    ('b', 1),
    sensor_msgs.PointField.UINT8:   ('B', 1),
    sensor_msgs.PointField.INT16:   ('h', 2),
    sensor_msgs.PointField.UINT16:  ('H', 2),
    sensor_msgs.PointField.INT32:   ('i', 4),
    sensor_msgs.PointField.UINT32:  ('I', 4),
}


def pointcloud2_to_numpy(msg: sensor_msgs.PointCloud2) -> np.ndarray:
    """Convert PointCloud2 → Nx3 float32 numpy array."""
    fmt = ('>' if msg.is_bigendian else '<') + ''.join(
        _DATATYPES[f.datatype][0]
        for f in sorted(msg.fields, key=lambda f: f.offset)
        if f.datatype in _DATATYPES
    )
    unpack = struct.Struct(fmt).unpack_from
    raw = bytes(msg.data)
    points = []
    offset = 0
    for _ in range(msg.height * msg.width):
        p = unpack(raw, offset)
        if not any(math.isnan(v) for v in p[:3]):
            points.append(p[:3])
        offset += msg.point_step
    return np.array(points, dtype=np.float32) if points else np.empty((0, 3), np.float32)


def quat_to_R(q) -> np.ndarray:
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])


def voxel_filter(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep one representative point per voxel cell."""
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int32)
    # Use a dict to deduplicate (keeps last point per voxel)
    voxel_map = {}
    for i, k in enumerate(map(tuple, keys)):
        voxel_map[k] = i
    return points[list(voxel_map.values())]


def numpy_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> sensor_msgs.PointCloud2:
    itemsize = 4
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


class MapBuilderNode(Node):

    def __init__(self):
        super().__init__('map_builder_node')

        self.map_points = np.empty((0, 3), dtype=np.float32)

        # ── TF ───────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── subscriber / publisher ────────────────────────────────────────────
        self.sub = self.create_subscription(
            sensor_msgs.PointCloud2, 'lidar_scan', self.scan_callback, 10)
        self.pub = self.create_publisher(
            sensor_msgs.PointCloud2, 'pcd_map', 10)

        self.create_timer(1.0 / PUBLISH_HZ, self.publish_map)

        self.get_logger().info(
            f'Map builder started  voxel={VOXEL_SIZE} m  '
            f'→ publishing /pcd_map in [map]')

    # ─────────────────────────────────────────────────────────────────────────
    def scan_callback(self, msg: sensor_msgs.PointCloud2):
        # 1. Get vehicle pose at scan time (map ← base_link)
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return

        pts_bl = pointcloud2_to_numpy(msg)      # (M, 3) in base_link
        if pts_bl.shape[0] == 0:
            return

        # 2. Transform to map frame:  p_map = R * p_bl + t
        R = quat_to_R(tf.transform.rotation)
        t = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float32)

        pts_map = (R @ pts_bl.T).T + t          # (M, 3)

        # 3. Accumulate
        self.map_points = np.vstack([self.map_points, pts_map.astype(np.float32)])

        # 4. Voxel filter to control size
        if self.map_points.shape[0] > MAX_POINTS:
            self.map_points = voxel_filter(self.map_points, VOXEL_SIZE)
            self.get_logger().info(
                f'Map: {self.map_points.shape[0]} points after voxel filter')

    def publish_map(self):
        if self.map_points.shape[0] == 0:
            return
        msg = numpy_to_pointcloud2(
            self.map_points, 'map',
            self.get_clock().now().to_msg())
        self.pub.publish(msg)
        self.get_logger().debug(f'Map published: {self.map_points.shape[0]} pts')


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
