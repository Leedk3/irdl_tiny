import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('pcd_demo'), 'config', 'config.rviz')

    ply_path = os.path.join(
        get_package_share_directory('pcd_demo'), 'resource', 'teapot.ply')

    assert os.path.exists(ply_path), f'PLY not found: {ply_path}'

    return LaunchDescription([
        # Visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
        # Source: rotating point cloud  →  /pcd_rotating
        Node(
            package='pcd_demo',
            executable='pcd_publisher_node',
            name='pcd_publisher_node',
            output='screen',
            arguments=[ply_path],
        ),
        # Target: static (fixed) point cloud  →  /pcd_static
        Node(
            package='pcd_demo',
            executable='pcd_static_publisher_node',
            name='pcd_static_publisher_node',
            output='screen',
            arguments=[ply_path],
        ),
        # Fast-GICP registration  →  /pcd_aligned
        Node(
            package='pcd_demo',
            executable='gicp_registration_node',
            name='gicp_registration_node',
            output='screen',
        ),
    ])
