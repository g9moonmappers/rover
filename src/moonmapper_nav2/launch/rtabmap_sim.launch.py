"""RTAB-Map for Gazebo-sim.

Starter RTAB-Map SLAM og (valgfritt) rtabmap_viz. Vi kjører rtabmap_viz direkte her
med `subscribe_odom:=true` for å unngå at GUI mister robotpose når TF-bufferen må
clear'es (typisk ved sim-tid hopp).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rtabmap_share = get_package_share_directory("rtabmap_launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_id = LaunchConfiguration("frame_id")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    viz = LaunchConfiguration("rtabmap_rviz")
    database_path = LaunchConfiguration("database_path")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_tf_map = LaunchConfiguration("publish_tf_map")

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_share, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "frame_id": frame_id,
            "odom_frame_id": "odom",
            "odom_topic": odom_topic,
            "approx_sync": "true",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "use_sim_time": use_sim_time,
            "map_always_update": "true",
            "publish_tf_map": publish_tf_map,
            # Ikke start rtabmap_viz fra rtabmap.launch.py; vi starter den selv nedenfor.
            "rviz": "false",
            "rtabmap_viz": "false",
            "database_path": database_path,
            "wait_for_transform": "0.5",
            "rtabmap_args": (
                "Odom/MinInliers:=3 Vis/MinInliers:=3 Odom/ResetCountdown:=1 "
                "Rtabmap/LoopThr:=0.5 Mem/NotLinkedNodesKept:=false"
            ),
        }.items(),
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace="/rtabmap",
        output="screen",
        condition=IfCondition(viz),
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {
                "frame_id": frame_id,
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_scan": True,
                # Viktig: bruk odom-topic direkte, ikke TF, for robust GUI.
                "subscribe_odom": True,
                "wait_for_transform": 0.5,
                "approx_sync": True,
            },
        ],
        remappings=[
            ("rgb/image", rgb_topic),
            ("depth/image", depth_topic),
            ("rgb/camera_info", camera_info_topic),
            ("scan", "/scan"),
            ("odom", odom_topic),
        ],
        arguments=["--ros-args", "--log-level", "rtabmap_viz:=info"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="base_footprint"),
            DeclareLaunchArgument("rgb_topic", default_value="/depth_camera/image"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/depth_camera/depth_image"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/depth_camera/camera_info"
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="",
                description="Sti til RTAB-Map SQLite-database (tom = pakke-standard).",
            ),
            DeclareLaunchArgument("rtabmap_rviz", default_value="false"),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/diff_drive_controller/odom",
                description=(
                    "Odometri topic brukt av RTAB-Map. "
                    "Sett til /odometry/filtered når moonmapper_localization (EKF) brukes."
                ),
            ),
            DeclareLaunchArgument(
                "publish_tf_map",
                default_value="true",
                description="Når true: RTAB-Map publiserer TF map->odom.",
            ),
            LogInfo(
                msg=(
                    "[moonmapper_nav2] RTAB-Map SIM: depth_camera topics, "
                    "frame_id=base_footprint, map-til-odom fra RTAB"
                )
            ),
            rtabmap,
            rtabmap_viz,
        ]
    )
