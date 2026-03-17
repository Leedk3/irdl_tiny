import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("rviz_marker_node"),
        "config",
        "config.yaml"
    )

    param_file_launch_arg = DeclareLaunchArgument(
        'rviz_marker_node_param_file',
        default_value=params_file,
        description='rviz_marker_node parameters'
    )

    rviz_marker_node = Node(
        package="rviz_marker_node",
        executable="rviz_marker_exe",
        output={"stderr": "screen", "stdout": "screen"},
        emulate_tty=True,
        parameters=[LaunchConfiguration('rviz_marker_node_param_file')],
    )

    return LaunchDescription([
        param_file_launch_arg,
        rviz_marker_node,
    ])
