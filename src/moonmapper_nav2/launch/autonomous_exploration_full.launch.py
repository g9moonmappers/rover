# Autonom utforskning (sim, RTAB-Map, Nav2 og frontier).

from __future__ import annotations

import os
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
import _nav2_rtabmap_common as _nav2_common 


def _truthy(s: str) -> bool:
    return s.lower() in ("1", "true", "yes", "on")


def _deep_merge(base: dict, overlay: dict) -> None:
    for key, val in overlay.items():
        if key in base and isinstance(base[key], dict) and isinstance(val, dict):
            _deep_merge(base[key], val)
        else:
            base[key] = val


def _merge_nav2_yaml(base_path: str, overlay_path: str) -> str:
    with open(base_path, encoding="utf-8") as f:
        merged = yaml.safe_load(f)
    with open(overlay_path, encoding="utf-8") as f:
        _deep_merge(merged, yaml.safe_load(f))
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, encoding="utf-8"
    )
    yaml.dump(merged, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


# build the launch description
def _build(context, *args, **kwargs):
    actions: list = []

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_st = _truthy(use_sim_time)
    start_sim = _truthy(LaunchConfiguration("start_sim").perform(context))
    start_slam = _truthy(LaunchConfiguration("start_slam").perform(context))
    start_nav2 = _truthy(LaunchConfiguration("start_nav2").perform(context))
    start_explorer = _truthy(LaunchConfiguration("start_explorer").perform(context))
    start_rviz = _truthy(LaunchConfiguration("start_rviz").perform(context))
    start_rtabmap_viz = _truthy(LaunchConfiguration("start_rtabmap_viz").perform(context))
    minimal_dbg = _truthy(LaunchConfiguration("minimal_nav2_debug").perform(context))
    initial_spin = _truthy(LaunchConfiguration("initial_spin").perform(context))
    delete_db = _truthy(LaunchConfiguration("delete_rtabmap_db").perform(context))
    use_sim_uwb = _truthy(LaunchConfiguration("use_sim_uwb").perform(context))

    try:
        nav_delay = float(LaunchConfiguration("navigation_stack_delay").perform(context))
    except ValueError:
        nav_delay = 12.0
    try:
        ex_extra = float(LaunchConfiguration("explorer_extra_delay_sec").perform(context))
    except ValueError:
        ex_extra = 5.0
    try:
        sim_stab_slam = float(
            LaunchConfiguration("sim_stabilization_before_slam_sec").perform(context)
        )
    except ValueError:
        sim_stab_slam = 6.0

    wpreset = LaunchConfiguration("world_preset").perform(context).strip()
    wp_lower = wpreset.lower()
    desc_share = get_package_share_directory("moonmapper_description")
    if wp_lower == "earth_explore":
        expected_sdf = os.path.join(desc_share, "worlds", "earth_arena_explore.sdf")
    elif wp_lower in ("expo_20x20", "expo"):
        expected_sdf = os.path.join(desc_share, "worlds", "earth_arena_expo_20x20.sdf")
    elif wp_lower == "earth":
        expected_sdf = os.path.join(desc_share, "worlds", "earth_arena.sdf")
    elif wp_lower == "moon":
        expected_sdf = os.path.join(desc_share, "worlds", "moon_arena.sdf")
    else:
        expected_sdf = "(resolver: gazebo_rover world_preset)"

    slam_start_after_launch = (
        max(0.0, sim_stab_slam) if (start_sim and start_slam) else 0.0
    )

    nav2_share = get_package_share_directory("moonmapper_nav2")
    params_cli = LaunchConfiguration("params_file").perform(context)
    if params_cli and os.path.isfile(os.path.expanduser(params_cli)):
        params_path = os.path.expanduser(params_cli)
    else:
        params_path = os.path.join(nav2_share, "config", "nav2_params_rtabmap_sim.yaml")

    if wp_lower in ("expo_20x20", "expo"):
        expo_overlay = os.path.join(nav2_share, "config", "nav2_params_expo_overlay.yaml")
        if os.path.isfile(expo_overlay):
            params_path = _merge_nav2_yaml(params_path, expo_overlay)

    db_raw = LaunchConfiguration("rtabmap_database_path").perform(context)
    db_path = os.path.expanduser(db_raw)

    map_topic = LaunchConfiguration("map_topic").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)

    if wp_lower in ("expo_20x20", "expo"):
        frontier_yaml = os.path.join(nav2_share, "config", "frontier_explorer_expo.yaml")
    else:
        frontier_yaml = os.path.join(nav2_share, "config", "frontier_explorer.yaml")

    rviz_nav = "true" if (start_rviz or minimal_dbg) else "false"

    actions.append(
        LogInfo(
            msg=(
                "[autonomous_exploration_full] bringup sequencing: "
                "sim på toppnivå (launch-arg scope), "
                f"RTAB-Map @{slam_start_after_launch}s, "
                f"Nav2 @{nav_delay}s, explorer @{nav_delay + ex_extra}s "
                "| earth til earth_arena.sdf, earth_explore til earth_arena_explore.sdf, "
                "expo_20x20 til earth_arena_expo_20x20.sdf"
            )
        )
    )
    actions.append(
        LogInfo(
            msg=(
                f"[autonomous_exploration_full] world_preset={wpreset} expected_world_sdf="
                f"{expected_sdf} use_sim_time={use_sim_time} params={params_path} "
                f"rtabmap_db={db_path} slam_start_after_launch_sec={slam_start_after_launch} "
                f"frontier_config={frontier_yaml} use_sim_uwb={use_sim_uwb}"
            )
        )
    )

    if delete_db and start_slam:
        exists = os.path.isfile(db_path)
        actions.append(
            LogInfo(
                msg=(
                    f"[autonomous_exploration_full] delete_rtabmap_db: removing {db_path} "
                    f"(exists={exists})"
                )
            )
        )
        actions.append(ExecuteProcess(cmd=["rm", "-f", db_path], output="log"))

    if use_sim_uwb:
        try:
            loc_share = get_package_share_directory("moonmapper_localization")
            loc_launch = os.path.join(loc_share, "launch", "sim_uwb_localization.launch.py")
            if os.path.isfile(loc_launch):
                loc_delay = max(0.0, sim_stab_slam * 0.5) if start_sim else 0.0
                actions.append(
                    TimerAction(
                        period=loc_delay,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(loc_launch),
                                launch_arguments={
                                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                                }.items(),
                            )
                        ],
                    )
                )
            else:
                actions.append(
                    LogInfo(
                        msg=(
                            "[autonomous_exploration_full] WARN: moonmapper_localization "
                            "ikke installert — use_sim_uwb ignorert"
                        )
                    )
                )
        except Exception as exc:  # noqa: BLE001
            actions.append(
                LogInfo(
                    msg=(f"[autonomous_exploration_full] WARN: localization pakke: {exc}")
                )
            )

    if start_slam:
        rtab_launch = os.path.join(_launch_dir, "rtabmap_sim.launch.py")
        rviz_rtab_arg = "true" if start_rtabmap_viz else "false"
        rtab_odom = "/odometry/filtered" if use_sim_uwb else "/diff_drive_controller/odom"
        slam_bundle = TimerAction(
            period=max(0.0, slam_start_after_launch),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtab_launch),
                    launch_arguments={
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "database_path": db_path,
                        "rtabmap_rviz": rviz_rtab_arg,
                        "odom_topic": rtab_odom,
                        "publish_tf_map": "true",
                    }.items(),
                ),
                TimerAction(period=1.0, actions=[_nav2_common.map_relay_node()]),
            ],
        )
        actions.append(slam_bundle)

    wait_map = start_slam or start_nav2
    if wait_map:
        actions.append(
            Node(
                package="moonmapper_nav2",
                executable="nav2_map_ready_wait_node",
                name="nav2_map_ready_wait",
                output="screen",
                parameters=[
                    {"use_sim_time": use_st},
                    {"map_topic": map_topic, "base_frame": base_frame, "check_period_sec": 2.5},
                ],
            )
        )

    if start_nav2:
        nav_launch = os.path.join(_launch_dir, "nav2_rtabmap_navigation.launch.py")
        actions.append(
            TimerAction(
                period=max(0.0, nav_delay),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "params_file": params_path,
                            "rviz": rviz_nav,
                            "world_preset": wpreset,
                            "depth_image_topic": "/depth_camera/depth_image",
                            "camera_info_topic": "/depth_camera/camera_info",
                        }.items(),
                    )
                ],
            )
        )

    if start_explorer:
        ex_launch = frontier_yaml if os.path.isfile(frontier_yaml) else ""
        ex_params = []
        if ex_launch:
            ex_params.append(ex_launch)
        ex_params.append(
            {
                "use_sim_time": use_st,
                "integrated_initial_spin": initial_spin,
                "map_topic": map_topic,
                "map_frame": LaunchConfiguration("map_frame").perform(context),
                "base_frame": base_frame,
                "cmd_vel_topic": cmd_vel_topic,
                "navigate_action": "/navigate_to_pose",
                "start_delay_sec": 0.0,
            }
        )
        actions.append(
            TimerAction(
                period=max(0.0, nav_delay + ex_extra),
                actions=[
                    Node(
                        package="moonmapper_nav2",
                        executable="frontier_explorer",
                        name="frontier_explorer",
                        output="screen",
                        parameters=ex_params,
                    )
                ],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    nav2_share = get_package_share_directory("moonmapper_nav2")
    default_params = os.path.join(nav2_share, "config", "nav2_params_rtabmap_sim.yaml")

    default_db = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), ".ros", "moonmapper_rtabmap.db"]
    )

    sim_include_actions: list = []
    try:
        _bringup = get_package_share_directory("moonmapper_bringup")
        _sim_lp = os.path.join(_bringup, "launch", "sim_rover_clean.launch.py")
        if os.path.isfile(_sim_lp):
            sim_include_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(_sim_lp),
                    launch_arguments={
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "world_preset": LaunchConfiguration("world_preset"),
                        "use_rviz": LaunchConfiguration("start_rviz"),
                        "force_set_pose_after_spawn": LaunchConfiguration(
                            "force_set_pose_after_spawn"
                        ),
                        "print_pose_after_spawn": LaunchConfiguration(
                            "print_pose_after_spawn"
                        ),
                    }.items(),
                    condition=IfCondition(LaunchConfiguration("start_sim")),
                )
            )
    except Exception:
        pass
    if not sim_include_actions:
        sim_include_actions.append(
            GroupAction(
                condition=IfCondition(LaunchConfiguration("start_sim")),
                actions=[
                    LogInfo(
                        msg=(
                            "[autonomous_exploration_full] WARN: moonmapper_bringup or "
                            "sim_rover_clean.launch.py not installed, start_sim har ingen effekt"
                        )
                    )
                ],
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world_preset",
                default_value="expo_20x20",
                description=(
                    "moon | earth | earth_explore | expo_20x20, videresendes til sim_rover_clean / gazebo_rover. "
                    "earth til earth_arena.sdf; earth_explore til earth_arena_explore.sdf; "
                    "expo_20x20 til earth_arena_expo_20x20.sdf (standard for autonomi)."
                ),
            ),
            DeclareLaunchArgument(
                "force_set_pose_after_spawn",
                default_value="false",
                description=(
                    "Tving set_pose etter spawn. Standard false for stabil autonom utforsking."
                ),
            ),
            DeclareLaunchArgument(
                "print_pose_after_spawn",
                default_value="false",
                description="Etter spawn: skriv Gazebo pose-topic én gang (sendes til gazebo_rover). Standard false.",
            ),
            DeclareLaunchArgument(
                "sim_stabilization_before_slam_sec",
                default_value="6.0",
                description=(
                    "Når start_sim og start_slam er true: sekunder etter launch før RTAB-Map startes "
                    "(sim starter med en gang)."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_uwb",
                default_value="false",
                description=(
                    "Når true: starter moonmapper_localization (fake UWB + EKF) for fused odom "
                    "på /odometry/filtered."
                ),
            ),
            DeclareLaunchArgument("start_sim", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_explorer", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("initial_spin", default_value="true"),
            DeclareLaunchArgument("delete_rtabmap_db", default_value="false"),
            DeclareLaunchArgument("navigation_stack_delay", default_value="12.0"),
            DeclareLaunchArgument("explorer_extra_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rtabmap_database_path", default_value=default_db),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_raw",
                description=(
                    "Initial spin publiserer hit. Bruk /cmd_vel_raw når safety_obstacle kobler "
                    "cmd_vel_raw til /cmd_vel (Nav2 publiserer til cmd_vel_raw)."
                ),
            ),
            DeclareLaunchArgument(
                "start_rtabmap_viz",
                default_value="false",
                description="Når true: starter RTAB-Map RViz (uavhengig av start_rviz for Nav2).",
            ),
            DeclareLaunchArgument(
                "minimal_nav2_debug",
                default_value="false",
                description="Når true: aktiverer Nav2 RViz i nav2_rtabmap_navigation.",
            ),
            *sim_include_actions,
            GroupAction(
                actions=[
                    LogInfo(
                        msg=(
                            "[autonomous_exploration_full] V1 stack: sim at top-level; delayed slam (Timer), "
                            "Nav2 and explorer timers; RTAB Viz only if start_rtabmap_viz; "
                            "Nav2 RViz only if start_rviz/minimal_nav2_debug."
                        )
                    ),
                    OpaqueFunction(function=_build),
                ]
            ),
        ]
    )
