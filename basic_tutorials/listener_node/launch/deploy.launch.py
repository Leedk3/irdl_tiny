import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("listener_node"),
        "config",
        "config.yaml"
    )

    param_file_launch_arg = DeclareLaunchArgument(
        'listener_node_param_file',
        default_value=params_file,
        description='listener_node parameters'
    )

    listener_node = Node(
        package="listener_node",
        executable="listener_exe",
        output={"stderr": "screen", "stdout": "screen"},
        emulate_tty=True,
        parameters=[LaunchConfiguration('listener_node_param_file')],
    )

    return LaunchDescription([
        param_file_launch_arg,
        listener_node,
    ])
