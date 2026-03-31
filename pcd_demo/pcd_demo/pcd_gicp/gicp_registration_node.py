import struct
import math

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import small_gicp


class GICPRegistrationNode(Node):
    """
    Subscribes to /pcd_rotating (source) and /pcd_static (target).
    Runs Fast-GICP to align the rotating cloud onto the static cloud.
    Publishes the aligned result to /pcd_aligned.

    Topic layout:
        /pcd_rotating  -- source: the spinning point cloud
        /pcd_static    -- target: fixed reference point cloud
        /pcd_aligned   -- result: source after GICP alignment
    """

    def __init__(self):
        super().__init__('gicp_registration_node')

        self.rotating_sub = Subscriber(self, sensor_msgs.PointCloud2, 'pcd_rotating')
        self.static_sub   = Subscriber(self, sensor_msgs.PointCloud2, 'pcd_static')

        # Synchronize the two topics within 0.1 s
        self.sync = ApproximateTimeSynchronizer(
            [self.rotating_sub, self.static_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.callback)

        self.aligned_pub = self.create_publisher(sensor_msgs.PointCloud2, 'pcd_aligned', 10)
        self.get_logger().info('GICP Registration Node started')
        self.get_logger().info('  source: /pcd_rotating')
        self.get_logger().info('  target: /pcd_static')
        self.get_logger().info('  result: /pcd_aligned')

    def callback(self, rotating_msg: sensor_msgs.PointCloud2,
                 static_msg: sensor_msgs.PointCloud2):

        source = pointcloud2_to_numpy(rotating_msg)  # the rotating cloud
        target = pointcloud2_to_numpy(static_msg)    # the fixed reference

        # --- Fast-GICP alignment ---
        # small_gicp.align(target, source) returns the transform T such that
        #   aligned = T @ source_point  (in homogeneous coords)
        result = small_gicp.align(target, source)
        T = result.T_target_source  # 4x4 numpy array

        # Apply the estimated transform to the source points
        N = source.shape[0]
        pts_h = np.hstack([source, np.ones((N, 1), dtype=np.float64)])
        aligned = (T @ pts_h.T).T[:, :3].astype(np.float32)

        msg = numpy_to_pointcloud2(aligned, rotating_msg.header.frame_id,
                                   rotating_msg.header.stamp)
        self.aligned_pub.publish(msg)

        self.get_logger().info(
            f'GICP: inliers={result.num_inliers}  '
            f'error={result.error:.4f}'
        )


# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------

_DATATYPES = {
    sensor_msgs.PointField.INT8:    ('b', 1),
    sensor_msgs.PointField.UINT8:   ('B', 1),
    sensor_msgs.PointField.INT16:   ('h', 2),
    sensor_msgs.PointField.UINT16:  ('H', 2),
    sensor_msgs.PointField.INT32:   ('i', 4),
    sensor_msgs.PointField.UINT32:  ('I', 4),
    sensor_msgs.PointField.FLOAT32: ('f', 4),
    sensor_msgs.PointField.FLOAT64: ('d', 8),
}


def pointcloud2_to_numpy(msg: sensor_msgs.PointCloud2) -> np.ndarray:
    """Convert PointCloud2 message to Nx3 float64 numpy array."""
    fmt = ('>' if msg.is_bigendian else '<') + ''.join(
        _DATATYPES[f.datatype][0]
        for f in sorted(msg.fields, key=lambda f: f.offset)
        if f.datatype in _DATATYPES
    )
    unpack = struct.Struct(fmt).unpack_from
    points = []
    for v in range(msg.height):
        offset = msg.row_step * v
        for u in range(msg.width):
            p = unpack(bytes(msg.data), offset)
            if not any(math.isnan(x) for x in p):
                points.append(p[:3])
            offset += msg.point_step
    return np.array(points, dtype=np.float64)


def numpy_to_pointcloud2(points: np.ndarray, frame_id: str, stamp=None) -> sensor_msgs.PointCloud2:
    """Convert Nx3 numpy array to PointCloud2 message."""
    ros_dtype = sensor_msgs.PointField.FLOAT32
    itemsize = 4  # float32

    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=frame_id)
    if stamp is not None:
        header.stamp = stamp

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=itemsize * 3,
        row_step=itemsize * 3 * points.shape[0],
        data=points.astype(np.float32).tobytes(),
    )


def main(args=None):
    rclpy.init(args=args)
    node = GICPRegistrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
