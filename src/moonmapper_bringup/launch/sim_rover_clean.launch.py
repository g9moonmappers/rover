"""Gazebo-sim inngang til gazebo_rover.launch.py.

Starter robot_state_publisher, Gazebo, spawn, ros2_control og sensorbroer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    desc_pkg = FindPackageShare("moonmapper_description")
    default_world = PathJoinSubstitution([desc_pkg, "worlds", "moon_arena.sdf"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_preset",
            default_value="moon",
            description=(
                "moon til moon_arena. earth til earth_arena. earth_explore til earth_arena_explore. "
                "Se gazebo_rover.launch.py for logget gravitasjon/friksjon."
            ),
        ),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "force_set_pose_after_spawn",
            default_value="false",
            description="Videresendes til gazebo_rover: gz set_pose etter spawn. Standard false for stabil sim.",
        ),
        DeclareLaunchArgument(
            "print_pose_after_spawn",
            default_value="false",
            description="Videresendes til gazebo_rover: skriv ut pose-topic etter spawn. Standard false.",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.026",
            description="base_footprint spawn-z (overstyres av physics_profile spawn_z).",
        ),
        DeclareLaunchArgument("enable_diff_plugin", default_value="true"),
        DeclareLaunchArgument(
            "physics_profile",
            default_value="",
            description=(
                "Gazebo preset: safe_6wd | earth_stable_6wd | earth_6wd | realistic_6wd | debug_low_friction. "
                "Tom = world_preset velger (moon til safe_6wd, earth til earth_stable_6wd)."
            ),
        ),
        DeclareLaunchArgument(
            "simple_collision_debug",
            default_value="true",
            description=(
                "Brukes når physics_profile er tom. Med safe_6wd-profil settes denne til true."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([desc_pkg, "launch", "gazebo_rover.launch.py"])
            ),
            launch_arguments={
                "world_preset": LaunchConfiguration("world_preset"),
                "world": LaunchConfiguration("world"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "enable_diff_plugin": LaunchConfiguration("enable_diff_plugin"),
                "physics_profile": LaunchConfiguration("physics_profile"),
                "simple_collision_debug": LaunchConfiguration("simple_collision_debug"),
                "force_set_pose_after_spawn": LaunchConfiguration(
                    "force_set_pose_after_spawn"
                ),
                "print_pose_after_spawn": LaunchConfiguration(
                    "print_pose_after_spawn"
                ),
                "use_ekf": "false",
            }.items(),
        ),
    ])
