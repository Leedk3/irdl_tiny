#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# this is the function launch  system will look for
def generate_launch_description():

    pkg_name = '4fn_basic_viewer'
    config_file = 'front_center.yaml'
    rviz_file = 'srs_rviz.rviz'
    cfg_fc = LaunchConfiguration(
        'cfg_fc',
        default = os.path.join(get_package_share_directory(pkg_name), 'config', config_file))
    rviz = os.path.join(get_package_share_directory(pkg_name), 'rviz', rviz_file)

    radar_node = Node(
        package='4fn_basic_viewer',
        executable='radar_node',
        parameters=[cfg_fc],
        arguments=[],
        output="screen",
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[],
        arguments=['-d', rviz],
        output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
        [
            radar_node,
            rviz2,
        ]
    )
