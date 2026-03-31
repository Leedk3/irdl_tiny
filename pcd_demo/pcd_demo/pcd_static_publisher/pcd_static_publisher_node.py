import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d


class PCDStaticPublisher(Node):
    """
    Publishes a fixed (non-rotating) point cloud to /pcd_static.
    This serves as the 'target' for GICP registration.
    """

    def __init__(self):
        super().__init__('pcd_static_publisher_node')

        assert len(sys.argv) > 1, "No ply file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."

        pcd = o3d.io.read_point_cloud(sys.argv[1])
        self.points = np.asarray(pcd.points, dtype=np.float32)
        self.get_logger().info(f'Loaded static point cloud: {self.points.shape[0]} points')

        self.publisher = self.create_publisher(sensor_msgs.PointCloud2, 'pcd_static', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        msg = point_cloud(self.points, 'map', self.get_clock().now().to_msg())
        self.publisher.publish(msg)


def point_cloud(points, parent_frame, stamp=None):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=parent_frame)
    if stamp is not None:
        header.stamp = stamp

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def main(args=None):
    rclpy.init(args=args)
    node = PCDStaticPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
