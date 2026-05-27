"""Nav2 mot live /map fra RTAB-Map. Uten map_server, AMCL eller egen map-til-odom-TF."""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
import _nav2_rtabmap_common as _common 


def generate_launch_description() -> LaunchDescription:
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_pkg = get_package_share_directory("moonmapper_nav2")
    default_params = os.path.join(nav2_pkg, "config", "nav2_params_rtabmap_sim.yaml")
    default_rviz = os.path.join(nav2_pkg, "rviz", "moonmapper_nav2.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    safety_stop = LaunchConfiguration("safety_stop_distance")
    safety_angle = LaunchConfiguration("safety_front_angle_deg")
    safety_to = LaunchConfiguration("safety_scan_timeout_sec")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/camera/depth/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.20"),
            DeclareLaunchArgument("safety_front_angle_deg", default_value="35.0"),
            DeclareLaunchArgument("safety_scan_timeout_sec", default_value="0.6"),
            LogInfo(
                msg=(
                    "[moonmapper_nav2] RTAB Nav2: navigation_launch only "
                    "(uten map_server/AMCL/identity map-til-odom). Forvent /map fra relay + RTAB map-til-odom TF."
                )
            ),
            _common.depth_to_scan_real(use_sim_time, LaunchConfiguration("use_sim_time")),
            navigation,
            _common.safety_node_rtabmap(use_sim_time, safety_stop, safety_angle, safety_to),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav2_rtabmap",
                output="screen",
                arguments=["-d", default_rviz],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                condition=IfCondition(rviz),
            ),
        ]
    )
