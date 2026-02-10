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
            get_package_share_directory("udp_communication_node"),
            "config",
            config_file
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'udp_communication_param_file',
        default_value=params_file,
        description='udp_communication_param'
    )

    udp_communication_node =  Node(
                package="udp_communication_node",
                executable="udp_communication_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('udp_communication_param_file'),
                    {
                        "origin_lat" : origin_lat_param,
                        "origin_lon" : origin_lon_param,
                    }
                ],
                remappings=[
                    # ("in_inspva", "novatel_bottom/inspva"), # HEADING PRIORITY 1
                    # ("out_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
                ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            udp_communication_node,
        ])
    

