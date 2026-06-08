"""Frontier explorer ROS-parametre og SafeGoalPickConfig fra node."""

from __future__ import annotations

from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node

from moonmapper_nav2.frontier_grid import SafeGoalPickConfig


def declare_parameter_if_not_declared(node: Node, name: str, default_value):
    """Deklarerer parameter bare hvis den ikke finnes (YAML/launch kan ha satt den)."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    try:
        node.declare_parameter(name, default_value)
    except ParameterAlreadyDeclaredException:
        pass
    return node.get_parameter(name).value


def declare_frontier_explorer_parameters(node: Node) -> None:
    """Deklarerer alle frontier_explorer-parametre med standardverdier."""
    d = declare_parameter_if_not_declared
    d(node, "use_sim_time", True)
    d(node, "map_topic", "/map")
    d(node, "map_frame", "map")
    d(node, "base_frame", "base_footprint")
    d(node, "navigate_action", "/navigate_to_pose")
    d(node, "cmd_vel_topic", "/frontier_explorer/explorer_cmd_vel")
    d(node, "explorer_stomp_cmd_vel_raw", False)
    d(node, "maneuver_cmd_vel_topic", "/cmd_vel")
    d(node, "exploration_rate_hz", 2.0)
    d(node, "min_frontier_cluster_size", 5)
    d(node, "min_goal_distance", 0.5)
    d(node, "min_goal_distance_m", 0.5)
    d(node, "max_goal_distance", 6.0)
    d(node, "max_goal_distance_m", 6.0)
    d(node, "max_goal_distance_stage_m", 6.0)
    d(node, "goal_timeout_sec", 120.0)
    d(node, "occupied_threshold", 65)
    d(node, "free_threshold", 0)
    d(node, "unknown_value", -1)
    d(node, "unknown_as_blocked_in_planner_grid", True)
    d(node, "goal_inflation_radius_m", 0.15)
    d(node, "bfs_goal_inflation_radius_m", 0.10)
    d(node, "approach_search_radius_min_m", 0.2)
    d(node, "approach_search_radius_max_m", 2.0)
    d(node, "min_obstacle_clearance_m", 0.45)
    d(node, "min_obstacle_clearance_floor_m", 0.32)
    d(node, "preferred_obstacle_clearance_m", 0.65)
    d(node, "min_unknown_clearance_m", 0.10)
    d(node, "preferred_unknown_clearance_m", 0.25)
    d(node, "frontier_goal_score_obstacle_weight", 3.0)
    d(node, "frontier_goal_score_distance_weight", 1.0)
    d(node, "frontier_goal_score_cluster_weight", 1.5)
    d(node, "frontier_goal_score_unknown_weight", 1.0)
    d(node, "min_free_space_around_goal_m", 0.45)
    d(node, "avoid_corner_goals", True)
    d(node, "allow_corner_fallback", True)
    d(node, "corner_penalty", 5.0)
    d(node, "corner_check_radius_m", 0.5)
    d(node, "publish_debug_markers", False)
    d(node, "enable_map_debug_log", False)
    d(node, "blacklist_failed_goal_radius_m", 0.75)
    d(node, "blacklist_failed_cluster", True)
    d(node, "min_clearance_after_failure_m", 0.55)
    d(node, "failure_retry_delay_sec", 1.0)
    d(node, "allow_frontier_adjacent_unknown", True)
    d(node, "require_strict_reachability_for_goal", False)
    d(node, "require_passable_for_approach", False)
    d(node, "frontier_neighbor_fallback_m", 1.0)
    d(node, "min_occupied_clearance_m", 0.15)
    d(node, "stuck_blocked_cancel_sec", 8.0)
    d(node, "retreat_from_frontier_steps", 3)
    d(node, "blacklist_radius", 0.55)
    d(node, "blacklist_timeout_sec", 120.0)
    d(node, "distance_weight", 1.0)
    d(node, "size_weight", 0.03)
    d(node, "publish_markers", True)
    d(node, "start_delay_sec", 5.0)
    d(node, "nav2_ready_timeout_sec", 120.0)
    d(node, "nav2_action_wait_sec", 2.0)
    d(node, "nav2_debug_log_interval_sec", 2.0)
    d(node, "wait_for_nav2", True)
    d(node, "require_nav_lifecycle_active", True)
    d(node, "require_cmd_vel_subscriber", True)
    d(node, "require_map_publisher_for_ready", False)
    d(node, "wait_log_interval_sec", 12.0)
    d(node, "integrated_initial_spin", False)
    d(node, "initial_spin_min_duration_sec", 1.5)
    d(node, "initial_spin_max_duration_sec", 40.0)
    d(node, "initial_spin_angular_z", 0.28)
    d(node, "initial_spin_target_rad", 6.28)
    d(node, "initial_spin_direct_cmd_vel", True)
    d(node, "initial_spin_cmd_vel_topic", "/cmd_vel")
    d(node, "map_settle_after_spin_sec", 0.5)
    d(node, "explore_immediately_after_spin", True)
    d(node, "bfs_bridge_known_free_radius_m", 2.5)
    d(node, "nearest_seed_search_radius_m", 0.6)
    d(node, "bfs_seed_search_radius_m", 3.0)
    d(node, "bfs_seed_search_step_m", 0.25)
    d(node, "allow_robot_seed_clearing", True)
    d(node, "robot_seed_clear_radius_m", 0.35)
    d(node, "min_reachable_cells_for_bfs", 120)
    d(node, "retry_when_no_frontier", True)
    d(node, "no_frontier_retry_delay_sec", 5.0)
    d(node, "max_no_frontier_retries", 0)
    d(node, "enable_staging_goal", False)
    d(node, "staging_min_distance_m", 0.2)
    d(node, "staging_max_distance_m", 0.8)
    d(node, "staging_clearance_m", 0.08)
    d(node, "staging_fan_angles_deg", [0.0])
    d(node, "staging_fan_distances_m", [0.3])
    d(node, "staging_require_free_value_zero", True)
    d(node, "consecutive_fail_limit", 3)
    d(node, "recovery_pause_sec", 2.0)
    d(node, "max_rescan_cycles", 8)
    d(node, "fallback_radius_min_m", 0.15)
    d(node, "fallback_radius_max_m", 0.8)
    d(node, "explored_mark_radius_m", 0.45)
    d(node, "min_passage_clearance_m", 0.35)
    d(node, "min_passage_clearance_floor_m", 0.28)
    d(node, "narrow_passage_penalty_weight", 4.0)
    d(node, "explored_revisit_penalty", 2.5)
    d(node, "enable_return_home_on_complete", True)
    d(node, "max_unknown_fraction_complete", 0.08)
    d(node, "min_frontier_clusters_to_continue", 1)
    d(node, "enable_active_backout_on_failure", False)
    d(node, "backout_duration_sec", 1.2)
    d(node, "backout_speed_mps", 0.10)
    d(node, "home_goal_tolerance_m", 0.35)
    d(node, "min_pause_after_success_sec", 2.5)
    d(node, "min_nav_commit_sec", 4.0)
    d(node, "min_time_between_goal_picks_sec", 3.0)
    d(node, "min_travel_after_goal_m", 0.4)
    d(node, "max_clusters_to_score", 25)
    d(node, "preferred_goal_distance_m", 1.5)
    d(node, "decision_timeout_sec", 3.0)
    d(node, "max_search_attempts_before_recovery", 3)
    d(node, "allow_relaxed_goal_after_attempts", 2)
    d(node, "goal_reselect_cooldown_sec", 1.0)
    d(node, "enable_recovery_maneuvers", True)
    d(node, "recovery_spin_angle_deg", 160.0)
    d(node, "recovery_forward_distance_m", 0.35)
    d(node, "recovery_backup_distance_m", 0.20)
    d(node, "recovery_turn_angle_deg", 45.0)
    d(node, "recovery_cmd_vel_linear", 0.05)
    d(node, "recovery_cmd_vel_angular", 0.25)
    d(node, "max_recovery_cycles", 5)
    d(node, "enable_nav_stuck_detection", False)
    d(node, "stuck_timeout_sec", 45.0)
    d(node, "min_progress_m", 0.03)
    d(node, "min_yaw_progress_rad", 0.08)
    d(node, "max_same_goal_failures", 2)
    d(node, "blacklist_radius_m", 0.6)
    d(node, "blacklist_duration_sec", 60.0)
    d(node, "completion_check_enabled", True)
    d(node, "completion_no_frontier_attempts", 5)
    d(node, "min_exploration_time_sec", 30.0)
    d(node, "reachable_unknown_threshold_ratio", 0.08)
    d(node, "return_home_enabled", True)
    d(node, "return_home_goal_tolerance_m", 0.35)
    d(node, "return_home_yaw_tolerance_rad", 0.6)
    d(node, "return_home_on_completion", True)
    d(node, "return_home_retry_tolerance_m", 0.5)
    d(node, "enable_too_close_recovery", True)
    d(node, "too_close_distance_m", 0.20)
    d(node, "too_close_sustain_sec", 5.0)
    d(node, "too_close_min_progress_m", 0.03)
    d(node, "recovery_backup_distance_m", 0.30)
    d(node, "backup_timeout_sec", 5.0)
    d(node, "no_turn_timeout_sec", 10.0)
    d(node, "enable_no_turn_recovery", False)
    d(node, "recovery_blacklist_radius_m", 0.7)
    d(node, "post_backup_turn_angle_deg", 30.0)


def safe_goal_config_from_node(node: Node) -> SafeGoalPickConfig:
    return SafeGoalPickConfig(
        min_goal_dist_m=float(node.get_parameter("min_goal_distance_m").value),
        max_goal_dist_m=min(
            float(node.get_parameter("max_goal_distance_m").value),
            float(node.get_parameter("max_goal_distance_stage_m").value),
        ),
        min_obstacle_clearance_m=float(node.get_parameter("min_obstacle_clearance_m").value),
        min_obstacle_clearance_floor_m=float(
            node.get_parameter("min_obstacle_clearance_floor_m").value
        ),
        preferred_obstacle_clearance_m=float(
            node.get_parameter("preferred_obstacle_clearance_m").value
        ),
        min_unknown_clearance_m=float(node.get_parameter("min_unknown_clearance_m").value),
        preferred_unknown_clearance_m=float(
            node.get_parameter("preferred_unknown_clearance_m").value
        ),
        approach_radius_min_m=float(node.get_parameter("approach_search_radius_min_m").value),
        approach_radius_max_m=float(node.get_parameter("approach_search_radius_max_m").value),
        score_distance_weight=float(
            node.get_parameter("frontier_goal_score_distance_weight").value
        ),
        score_obstacle_weight=float(
            node.get_parameter("frontier_goal_score_obstacle_weight").value
        ),
        score_cluster_weight=float(
            node.get_parameter("frontier_goal_score_cluster_weight").value
        ),
        score_unknown_weight=float(
            node.get_parameter("frontier_goal_score_unknown_weight").value
        ),
        min_free_space_around_goal_m=float(
            node.get_parameter("min_free_space_around_goal_m").value
        ),
        avoid_corner_goals=bool(node.get_parameter("avoid_corner_goals").value),
        allow_corner_fallback=bool(node.get_parameter("allow_corner_fallback").value),
        corner_penalty=float(node.get_parameter("corner_penalty").value),
        corner_check_radius_m=float(node.get_parameter("corner_check_radius_m").value),
        require_passable_for_approach=bool(
            node.get_parameter("require_passable_for_approach").value
        ),
        neighbor_fallback_m=float(node.get_parameter("frontier_neighbor_fallback_m").value),
        min_passage_clearance_m=float(node.get_parameter("min_passage_clearance_m").value),
        min_passage_clearance_floor_m=float(
            node.get_parameter("min_passage_clearance_floor_m").value
        ),
        narrow_passage_penalty_weight=float(
            node.get_parameter("narrow_passage_penalty_weight").value
        ),
        explored_revisit_penalty=float(node.get_parameter("explored_revisit_penalty").value),
    )
