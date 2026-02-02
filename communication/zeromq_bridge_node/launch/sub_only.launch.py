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
            get_package_share_directory("zeromq_bridge_node"),
            "config",
            config_file
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'zeromq_bridge_node_param_file',
        default_value=params_file,
        description='zeromq_bridge_node_param'
    )

    zeromq_bridge_pub_node =  Node(
                package="zeromq_bridge_node",
                executable="zeromq_bridge_pub_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('zeromq_bridge_node_param_file'),
                    # {
                    #     "bestvel_heading_update_velocity_thres" : 3.0,
                    # }
                ],
                remappings=[
                    # ("in_inspva", "novatel_bottom/inspva"), # HEADING PRIORITY 1
                    # ("out_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
                ]
    )

    zeromq_bridge_sub_node =  Node(
                package="zeromq_bridge_node",
                executable="zeromq_bridge_sub_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('zeromq_bridge_node_param_file'),
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
#            zeromq_bridge_pub_node,
            zeromq_bridge_sub_node,
        ])
    

