import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
import math

# get test site
origin_lat_param = float(os.environ.get('TESTSITE_LAT_ORIGIN'))
origin_lon_param = float(os.environ.get('TESTSITE_LON_ORIGIN'))


def generate_launch_description():
    config_file = 'config.yaml'
    params_file = os.path.join(
            get_package_share_directory("mjpeg_to_ros_node"),
            "config",
            config_file
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'mjpeg_to_ros_node_param_file',
        default_value=params_file,
        description='mjpeg_to_ros_node_param'
    )

    mjpeg_to_ros_node =  Node(
                package="mjpeg_to_ros_node",
                executable="mjpeg_to_ros_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('mjpeg_to_ros_node_param_file'),
                    # {
                    #     "bestvel_heading_update_velocity_thres" : 3.0,
                    # }
                ],
                remappings=[
                    # ("in_inspva", "novatel_bottom/inspva"), # HEADING PRIORITY 1
                    # ("out_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
                ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            mjpeg_to_ros_node,
        ])
    

