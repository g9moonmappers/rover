"""Felles noder for RTAB-Map og Nav2.
"""

from __future__ import annotations

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def depth_to_scan_real(
    use_sim_time: LaunchConfiguration, _unused: LaunchConfiguration
) -> Node:
    """depth_to_scan from launch-configured depth/camera_info topics."""
    return Node(
        package="moonmapper_autonomy",
        executable="depth_to_scan_node",
        name="depth_to_scan_rtabmap",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {
                "depth_image_topic": ParameterValue(
                    LaunchConfiguration("depth_image_topic"), value_type=str
                ),
                "camera_info_topic": ParameterValue(
                    LaunchConfiguration("camera_info_topic"), value_type=str
                ),
                "scan_topic": "/scan",
                "scan_height_mode": "roi_percentile",
                "roi_top_ratio": 0.42,
                "roi_bottom_ratio": 0.58,
                "center_crop_ratio": 0.88,
                "min_valid_points_per_column": 10,
                "ground_filter_enabled": True,
                "ground_filter_bottom_roi_ratio": 0.28,
                "roi_percentile": 0.18,
                "front_percentile": 0.18,
                "depth_min_valid_m": 0.32,
                "depth_max_valid_m": 4.0,
                "range_min": 0.22,
                "range_max": 3.5,
                "debug_log_period_sec": 1.0,
                "scan_time": 0.1,
                # Bruk depth optical frame (matcher URDF og Gazebo gz_frame_id.
                "output_frame_id": "depth_camera_optical_frame",
            },
        ],
    )


def safety_node_rtabmap(
    use_sim_time: LaunchConfiguration,
    safety_stop: LaunchConfiguration,
    safety_angle: LaunchConfiguration,
    safety_to: LaunchConfiguration,
) -> Node:
    return Node(
        package="moonmapper_autonomy",
        executable="safety_obstacle_node",
        name="safety_obstacle_nav2_rtabmap",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {
                "input_cmd_topic": "/cmd_vel_raw",
                "output_cmd_topic": "/cmd_vel",
                "scan_topic": "/scan",
                "front_stop_distance": ParameterValue(safety_stop, value_type=float),
                "emergency_stop_distance_m": 0.12,
                "slowdown_distance_m": 0.38,
                "range_calibration_offset_m": 0.04,
                "min_trusted_front_range_m": 0.24,
                "enable_safety_gating": True,
                "slow_linear_speed": 0.22,
                "safe_turn_speed": 0.40,
                "min_turn_clearance_m": 0.26,
                "corridor_creep_enabled": True,
                "corridor_min_side_clearance_m": 0.26,
                "corridor_min_total_width_m": 0.48,
                "corridor_creep_speed_mps": 0.14,
                "corridor_creep_scale": 0.50,
                "allow_reverse_when_blocked": False,
                "safety_controls_backup": False,
                "front_angle_deg": ParameterValue(safety_angle, value_type=float),
                "side_angle_deg": 75.0,
                "scan_timeout_sec": ParameterValue(safety_to, value_type=float),
                "reverse_speed_when_blocked": 0.05,
                "publish_safety_debug": True,
                "debug_log_period_sec": 1.0,
            },
        ],
    )


def map_relay_node() -> Node:
    return Node(
        package="topic_tools",
        executable="relay",
        name="rtabmap_map_to_nav2_map",
        output="screen",
        arguments=["/rtabmap/map", "/map"],
        remappings=[],
    )
