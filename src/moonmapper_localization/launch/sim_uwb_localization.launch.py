"""Sim-only UWB localization: sim UWB node + robot_localization EKF.

Dette er sim-only UWB. Fysisk BU04-integrasjon kommer senere.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    loc_share = get_package_share_directory("moonmapper_localization")
    default_sim_cfg = os.path.join(loc_share, "config", "uwb_anchors.yaml")
    default_ekf_cfg = os.path.join(loc_share, "config", "ekf_sim_uwb.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_cfg = LaunchConfiguration("sim_uwb_config")
    ekf_cfg = LaunchConfiguration("ekf_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("sim_uwb_config", default_value=default_sim_cfg),
            DeclareLaunchArgument("ekf_config", default_value=default_ekf_cfg),
            Node(
                package="moonmapper_localization",
                executable="sim_uwb_node",
                name="sim_uwb_node",
                output="screen",
                parameters=[sim_cfg, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_cfg, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
        ]
    )

