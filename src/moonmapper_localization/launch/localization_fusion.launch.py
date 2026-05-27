"""Sim: fake UWB + robot_localization EKF for fused odom."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    loc_share = get_package_share_directory('moonmapper_localization')
    ekf_cfg = os.path.join(loc_share, 'config', 'ekf_sim_uwb.yaml')
    uwb_cfg = os.path.join(loc_share, 'config', 'uwb_anchors.yaml')

    use_sim = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    sim_uwb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'sim_uwb.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim,
            'config_file': config_file,
        }.items(),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_sim_uwb',
        output='screen',
        parameters=[ekf_cfg, {'use_sim_time': use_sim}],
    )

    preflight = Node(
        package='moonmapper_localization',
        executable='localization_preflight_node',
        name='localization_preflight',
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=uwb_cfg),
        LogInfo(
            msg=(
                '[localization_fusion] SIM: UWB -> /uwb/pose + /uwb/ranges; '
                'ekf_node -> /odometry/filtered + TF odom->base_footprint. RTAB-Map kan fortsatt eie map->odom.'
            )
        ),
        sim_uwb,
        ekf_node,
        preflight,
    ])
