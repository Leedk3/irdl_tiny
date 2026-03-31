"""
mapping_demo.launch.py
======================
Launches a LiDAR mapping demo that combines:
  - ad_viz_node  : vehicle driving in a circle  (TF: map → base_link)
  - lidar_sim_node   : simulates LiDAR on the vehicle  → /lidar_scan
  - map_builder_node : accumulates scans into a map     → /pcd_map

RViz topics to add:
  /ad/vehicle        Marker      - yellow vehicle arrow
  /ad/global_path    Path        - circular reference path
  /ad/obstacles      MarkerArray - coloured obstacle cubes
  /lidar_scan        PointCloud2 - current sensor view  (frame: base_link)
  /pcd_map           PointCloud2 - growing map          (frame: map)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # PLY scene file from pcd_demo (teapot placed at map origin)
    ply_path = os.path.join(
        get_package_share_directory('pcd_demo'), 'resource', 'teapot.ply')
    assert os.path.exists(ply_path), f'PLY not found: {ply_path}'

    # ad_viz_node config
    ad_viz_config = os.path.join(
        get_package_share_directory('ad_viz_node'), 'config', 'config.yaml')

    return LaunchDescription([
        # ── vehicle simulation (C++) ──────────────────────────────────────────
        Node(
            package='ad_viz_node',
            executable='ad_viz_exe',
            name='ad_viz_node',
            output='screen',
            parameters=[ad_viz_config],
        ),

        # ── LiDAR simulator (Python) ─────────────────────────────────────────
        # Reads PLY scene, publishes /lidar_scan in [base_link] frame
        Node(
            package='lidar_mapping',
            executable='lidar_sim_node',
            name='lidar_sim_node',
            output='screen',
            arguments=[ply_path],
        ),

        # ── Map builder (Python) ─────────────────────────────────────────────
        # Accumulates /lidar_scan → /pcd_map in [map] frame
        Node(
            package='lidar_mapping',
            executable='map_builder_node',
            name='map_builder_node',
            output='screen',
        ),

        # ── RViz2 ────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
