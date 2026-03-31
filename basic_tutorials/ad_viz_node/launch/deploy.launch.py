import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("ad_viz_node"),
        "config",
        "config.yaml"
    )

    param_file_launch_arg = DeclareLaunchArgument(
        'ad_viz_node_param_file',
        default_value=params_file,
        description='ad_viz_node parameters'
    )

    ad_viz_node = Node(
        package="ad_viz_node",
        executable="ad_viz_exe",
        output={"stderr": "screen", "stdout": "screen"},
        emulate_tty=True,
        parameters=[LaunchConfiguration('ad_viz_node_param_file')],
    )

    return LaunchDescription([
        param_file_launch_arg,
        ad_viz_node,
    ])
