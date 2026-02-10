import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

# get test site
origin_lat_param = float(os.environ.get('TESTSITE_LAT_ORIGIN'))
origin_lon_param = float(os.environ.get('TESTSITE_LON_ORIGIN'))

def generate_launch_description():
    config_file = 'config.yaml'
    params_file = os.path.join(
            get_package_share_directory("uam_route_planner"),
            "config",
            config_file
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'uam_route_planner_param_file',
        default_value=params_file,
        description='uam_route_planner_param'
    )

    uam_route_planner =  Node(
                package="uam_route_planner",
                executable="uam_route_planner_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('uam_route_planner_param_file'),
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
            uam_route_planner    
        ])