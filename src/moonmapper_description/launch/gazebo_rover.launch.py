"""Launch MoonMapper i Gazebo Sim 8 (Harmonic).
Start gz sim (server + GUI), robot_state_publisher, ros_gz_sim create, ros_gz_bridge parameter_bridge, spawner for joint_state_broadcaster og diff_drive_controller, cmd_vel_odom_relay, RViz.
"""

import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Helper functions for physics profiles, world presets, and logging.
def _load_physics_profiles() -> dict:
    desc_share = get_package_share_directory("moonmapper_description")
    path = os.path.join(desc_share, "config", "physics_profiles.yaml")
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)

# Parse world SDF file for gravity, friction, and collision properties.
def _parse_world_sdf(path: str) -> tuple[str, str, str]:
    """Return (gravity_str, ground_mu, ground_mu2) from an SDF world file."""
    gravity_s = "(unknown)"
    mu_s, mu2_s = "(unknown)", "(unknown)"
    try:
        tree = ET.parse(path)
        root = tree.getroot()
        grav = root.find(".//gravity")
        if grav is not None and grav.text:
            gravity_s = grav.text.strip()
        for friction in root.findall(".//model[@name='ground_plane']//friction"):
            ode = friction.find("ode")
            if ode is None:
                continue
            mu = ode.find("mu")
            mu2 = ode.find("mu2")
            if mu is not None and mu.text:
                mu_s = mu.text.strip()
            if mu2 is not None and mu2.text:
                mu2_s = mu2.text.strip()
            break
    except (ET.ParseError, OSError) as exc:
        gravity_s = f"(parse error: {exc})"
    return gravity_s, mu_s, mu2_s

# Extract gravity z-component in m/s² from SDF gravity string.
def _gravity_z_mps2(gravity_s: str) -> float | None:
    parts = gravity_s.split()
    if len(parts) >= 3:
        try:
            return float(parts[2])
        except ValueError:
            pass
    return None

# Resolve world SDF path, name, and default physics profile.
def _resolve_world_preset(context):
    """world_preset selects world SDF, world_name, default physics_profile."""
    preset = (context.launch_configurations.get("world_preset") or "moon").strip().lower()
    desc_share = get_package_share_directory("moonmapper_description")
    if preset == "earth":
        world_path = os.path.join(desc_share, "worlds", "earth_arena.sdf")
        world_name = "earth_arena"
        default_profile = "earth_stable_6wd"
    elif preset == "earth_explore":
        world_path = os.path.join(desc_share, "worlds", "earth_arena_explore.sdf")
        world_name = "earth_arena_explore"
        default_profile = "earth_stable_6wd"
    elif preset in ("expo_20x20", "expo"):
        world_path = os.path.join(desc_share, "worlds", "earth_arena_expo_20x20.sdf")
        world_name = "earth_arena_expo_20x20"
        default_profile = "earth_stable_6wd"
    elif preset == "moon":
        world_path = os.path.join(desc_share, "worlds", "moon_arena.sdf")
        world_name = "moon_arena"
        default_profile = "safe_6wd"
    else:
        return [
            LogInfo(
                msg=(
                    f"[gazebo_rover] Unknown world_preset='{preset}' "
                    "(use moon | earth | earth_explore | expo_20x20)"
                ),
            ),
        ]
    actions = [
        SetLaunchConfiguration("world", world_path),
        SetLaunchConfiguration("world_name", world_name),
    ]
    profile = (context.launch_configurations.get("physics_profile") or "").strip()
    if not profile:
        actions.append(SetLaunchConfiguration("physics_profile", default_profile))
    if preset in ("expo_20x20", "expo"):
        sx = (context.launch_configurations.get("spawn_x") or "").strip()
        sy = (context.launch_configurations.get("spawn_y") or "").strip()
        if sx in ("", "0.0", "0"):
            actions.append(SetLaunchConfiguration("spawn_x", "-7.5"))
        if sy in ("", "0.0", "0"):
            actions.append(SetLaunchConfiguration("spawn_y", "-7.5"))
    return actions

# Log world file, gravity, friction, and active physics/xacro keys.
def _log_world_physics_summary(context):
    """Log world file, gravity, friction, and active physics/xacro keys."""
    world_path = context.launch_configurations.get("world", "")
    profile = (context.launch_configurations.get("physics_profile") or "").strip()
    preset = (context.launch_configurations.get("world_preset") or "moon").strip()
    grav_s, g_mu, g_mu2 = _parse_world_sdf(world_path) if world_path else ("?", "?", "?")
    gz = _gravity_z_mps2(grav_s)
    lunar_warn = ""
    if gz is not None and -2.5 < gz < -0.5:
        lunar_warn = "  WARNING: lunar-like gravity detected, rotate/arc may slip."
    wheel_mu1 = context.launch_configurations.get("wheel_mu1", "?")
    wheel_mu2 = context.launch_configurations.get("wheel_mu2", "?")
    simple_col = context.launch_configurations.get("simple_collision_debug", "?")
    spawn_z = context.launch_configurations.get("spawn_z", "?")
    return [
        LogInfo(
            msg=(
                f"[gazebo_rover] world_preset={preset}  world={world_path}  "
                f"world_name={context.launch_configurations.get('world_name', '?')}"
            ),
        ),
        LogInfo(
            msg=(
                f"[gazebo_rover] gravity={grav_s}  ground_mu={g_mu}  ground_mu2={g_mu2}"
                f"{lunar_warn}"
            ),
        ),
        LogInfo(
            msg=(
                f"[gazebo_rover] physics_profile={profile or '(none)'}  "
                f"wheel_mu1={wheel_mu1}  wheel_mu2={wheel_mu2}  "
                f"simple_collision_debug={simple_col}  spawn_z={spawn_z}"
            ),
        ),
    ]

# Apply physics profile: set spawn_z, friction, collisions, and rocker_diff_coupling_mode.
def _apply_physics_profile(context):
    """When physics_profile is set, apply preset launch keys (spawn_z, friction, collisions)."""
    profile = (context.launch_configurations.get("physics_profile") or "").strip()
    if not profile:
        return []
    profiles = _load_physics_profiles()
    spec = profiles.get(profile)
    if spec is None:
        return [
            LogInfo(
                msg=(
                    f"[gazebo_rover] Unknown physics_profile='{profile}' "
                    f"(known: {', '.join(sorted(profiles))})"
                ),
            ),
        ]
    actions = [
        LogInfo(msg=f"[gazebo_rover] physics_profile={profile}"),
    ]
    for key, val in spec.items():
        if isinstance(val, bool):
            sval = "true" if val else "false"
        else:
            sval = str(val)
        actions.append(SetLaunchConfiguration(key, sval))
    return actions

# Physics-only debug mode: disable RViz, set rocker_diff_debug to false.
def _physics_only_debug(context):
    """physics_only_debug: uten RViz; tvinger ikke rocker_diff_debug (unngår gz-plugin-spam)."""
    raw = (context.launch_configurations.get("physics_only_debug") or "").strip().lower()
    if raw not in ("true", "1", "yes"):
        return []
    return [
        SetLaunchConfiguration("use_rviz", "false"),
        LogInfo(
            msg=(
                "[gazebo_rover] physics_only_debug: use_rviz=false (Gazebo+ros2_control isolert). "
                "rocker_diff_coupling_mode default=weak (anbefalt). "
                "rocker_diff_debug styres kun av launch-arg (default false), sett true for throttlet qL/qR-logg. "
                "coupling_mode:=off er diagnose/passiv URDF; ikke normal kjøring."
            ),
        ),
    ]

# Spawn diff_drive_controller, relay, and EKF after joint_state_broadcaster.
def _diff_relay_ekf_chain(context, *, load_jsb):
    """diff_drive-spawner (valgfri odom-TF-overlay), relay og valgfri EKF etter load_jsb."""
    desc_share = get_package_share_directory("moonmapper_description")
    use_ekf = (context.launch_configurations.get("use_ekf") or "false").strip().lower() in (
        "true",
        "1",
        "yes",
    )
    base_diff = os.path.join(desc_share, "config", "diff_drive_controller.yaml")
    overlay = os.path.join(desc_share, "config", "diff_drive_controller_disable_odom_tf.yaml")
    diff_args = [
        "diff_drive_controller",
        "--controller-manager",
        "/controller_manager",
        "-p",
        base_diff,
    ]
    if use_ekf:
        diff_args.extend(["-p", overlay])

    load_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=diff_args,
        output="screen",
    )

    # Configure cmd_vel_odom_relay: /cmd_vel til /diff_drive_controller/cmd_vel, /diff_drive_controller/odom til /odom.
    use_sim_time = LaunchConfiguration("use_sim_time")
    relay_frame = LaunchConfiguration("cmd_vel_relay_frame_id")
    relay_params = [
        {"use_sim_time": use_sim_time},
        {
            "frame_id": ParameterValue(relay_frame, value_type=str),
        },
        # /odom kommer fra topic_tools relay diff_drive til /odom, ikke denne noden.
        {"publish_odom_relay": False},
        {
            "use_smoothed_twist_stamped_input": ParameterValue(
                LaunchConfiguration("cmd_vel_relay_use_smoothed_twist_stamped"),
                value_type=bool,
            ),
        },
        {
            "twist_stamped_topic": LaunchConfiguration("cmd_vel_relay_twist_stamped_topic"),
        },
        {
            "publish_twist_cmd_vel_mirror": ParameterValue(
                LaunchConfiguration("cmd_vel_relay_publish_twist_cmd_vel_mirror"),
                value_type=bool,
            ),
        },
        {
            "invert_cmd_vel_twist": ParameterValue(
                LaunchConfiguration("invert_cmd_vel_twist"),
                value_type=bool,
            ),
        },
        {
            "cmd_linear_x_sign": ParameterValue(
                LaunchConfiguration("cmd_linear_x_sign"),
                value_type=float,
            ),
        },
        {
            "cmd_angular_z_sign": ParameterValue(
                LaunchConfiguration("cmd_angular_z_sign"),
                value_type=float,
            ),
        },
    ]

    cmd_vel_odom_relay = Node(
        package="moonmapper_bringup",
        executable="cmd_vel_odom_relay",
        name="cmd_vel_odom_relay",
        parameters=relay_params,
        output="screen",
    )

    ekf_params_path = os.path.join(desc_share, "config", "ekf_wheel_odom.yaml")
    ekf_pub_raw = (context.launch_configurations.get("ekf_publish_tf") or "true").strip().lower()
    ekf_publish_tf = ekf_pub_raw in ("true", "1", "yes")
    builtin_wheel_ekf = (context.launch_configurations.get("builtin_wheel_ekf") or "true").strip().lower() in (
        "true",
        "1",
        "yes",
    )

    on_exit_after_diff: list = [cmd_vel_odom_relay]
    if not use_ekf:
        odom_relay = Node(
            package="topic_tools",
            executable="relay",
            name="diff_drive_odom_relay",
            output="screen",
            arguments=["/diff_drive_controller/odom", "/odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
        on_exit_after_diff = [odom_relay, cmd_vel_odom_relay]

    if use_ekf and builtin_wheel_ekf:
        ekf_node = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params_path,
                {
                    "use_sim_time": use_sim_time,
                    "publish_tf": ekf_publish_tf,
                },
            ],
        )
        on_exit_after_diff.append(ekf_node)

    return [
        RegisterEventHandler(
            OnProcessExit(target_action=load_jsb, on_exit=[load_diff]),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=load_diff, on_exit=on_exit_after_diff),
        ),
    ]

# z_spawn:= is a common typo; copy to spawn_z if z_spawn is set.
def _spawn_z_alias(context):
    """z_spawn:= er vanlig skrivefeil; kopier til spawn_z hvis z_spawn er satt."""
    z_alias = (context.launch_configurations.get("z_spawn") or "").strip()
    if z_alias:
        return [
            LogInfo(
                msg=(
                    "[gazebo_rover] z_spawn er satt; bruk helst spawn_z:= for samme verdi. "
                    f"Bruker z={z_alias} m som spawn-høyde."
                ),
            ),
            SetLaunchConfiguration("spawn_z", z_alias),
        ]
    return []

# generate launch_description: main entry point for the launch file.
def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("moonmapper_description")
    default_xacro = PathJoinSubstitution(
        [pkg, "urdf", "moonmapper_rover_gazebo.urdf.xacro"],
    )
    default_world = PathJoinSubstitution([pkg, "worlds", "moon_arena.sdf"])
    default_rviz = PathJoinSubstitution([pkg, "rviz", "moonmapper.rviz"])
    default_bridge = PathJoinSubstitution([pkg, "config", "ros_gz_bridge.yaml"])
    default_controllers = PathJoinSubstitution(
        [pkg, "config", "wheel_controllers.yaml"],
    )
    default_joint_state_broadcaster = PathJoinSubstitution(
        [pkg, "config", "joint_state_broadcaster.yaml"],
    )

    args = [
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="ROS_DOMAIN_ID for alle prosesser i denne launch-filen.",
        ),
        DeclareLaunchArgument(
            "rmw_implementation",
            # FastDDS er standard på Ubuntu.
            default_value="rmw_fastrtps_cpp",
            description="RMW_IMPLEMENTATION for alle prosesser i denne launch-filen.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value=default_xacro,
            description="Path til Gazebo-xacro-wrapper.",
        ),
        DeclareLaunchArgument(
            "world_preset",
            default_value="moon",
            description=(
                "moon til moon_arena.sdf + safe_6wd. "
                "earth til earth_arena.sdf. "
                "earth_explore til earth_arena_explore.sdf. "
                "expo_20x20 til earth_arena_expo_20x20.sdf (spawn SW ~ -7.5,-7.5). "
                "Overstyrer world/world_name."
            ),
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Path til .sdf-verden (overstyres av world_preset).",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="moon_arena",
            description="Gazebo-verdensnavn (overstyres av world_preset).",
        ),
        DeclareLaunchArgument(
            "gz_sim_verbosity",
            default_value="2",
            description=(
                "gz sim -v nivaa (0-4). 3 = meget detaljert (tungt med GUI og mange sensorbroer); "
                "2 eller 1 gir mindre logg- og UI-belastning."
            ),
        ),
        DeclareLaunchArgument(
            "force_set_pose_after_spawn",
            default_value="false",
            description=(
                "Kall /world/<name>/set_pose etter spawn (sikrer at spawn_z brukes). "
                "Standard false for stabil autonom oppstart; slå på ved feil spawn-høyde."
            ),
        ),
        DeclareLaunchArgument(
            "print_pose_after_spawn",
            default_value="false",
            description="Skriv /world/<name>/pose/info én gang etter set_pose (viser faktisk x/y/z). Standard av.",
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz,
            description="RViz-konfig.",
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=default_bridge,
            description="YAML for ros_gz_bridge parameter_bridge.",
        ),
        DeclareLaunchArgument(
            "controllers",
            default_value=default_controllers,
            description="YAML for ros2_control-controllere.",
        ),
        DeclareLaunchArgument(
            "enable_triad_spectroscopy",
            default_value="false",
            description="Aktiver TriadSpectroscopy raycast-plugin (mye loggstøy).",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.006",
            description=(
                "Start-høyde (m) for ros_gz_sim create -z og set_pose (modellens rot-frame = base_footprint). "
                "URDF løfter base_link med base_link_z_above_footprint (~0,124 m); default ~0,006 m tilsvarer "
                "tidligere ~0,13 m for hele kroppen da footprint og base_link var sammenfallende. "
                "Juster til hjulene treffer regolith-plan (z=0). Ikke forveksle med z_spawn."
            ),
        ),
        DeclareLaunchArgument(
            "z_spawn",
            default_value="",
            description=(
                "Alias: hvis ikke tom overstyrer spawn_z (for vanlig skrivefeil). "
                "Foretrekk spawn_z:=0.006 på kommandolinja (eller juster etter arena)."
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="false",
            description=(
                "True: robot_localization EKF (hjul-odom), /odometry/filtered, "
                "TF odom til base_footprint; diff_drive enable_odom_tf av; ingen /odom-relay."
            ),
        ),
        DeclareLaunchArgument(
            "builtin_wheel_ekf",
            default_value="true",
            description=(
                "Nar use_ekf: true = start innebygd ekf_filter_node (ekf_wheel_odom.yaml). "
                "False = ekstern lokal/global EKF (f.eks. UWB digital tvilling); behold diff_drive TF-overlay."
            ),
        ),
        DeclareLaunchArgument(
            "ekf_publish_tf",
            default_value="true",
            description="Nar use_ekf: om EKF skal publisere TF (odom til base_footprint).",
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_frame_id",
            default_value="base_footprint",
            description="frame_id i TwistStamped fra cmd_vel_odom_relay til diff_drive.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_use_smoothed_twist_stamped",
            default_value="false",
            description=(
                "True: cmd_vel_odom_relay abonnerer paa TwistStamped-topic "
                "(se cmd_vel_relay_twist_stamped_topic) i stedet for /cmd_vel (Twist)."
            ),
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_twist_stamped_topic",
            default_value="/cmd_vel_smoothed",
            description="TwistStamped-inngang naar use_smoothed er true (default /cmd_vel_smoothed).",
        ),
        DeclareLaunchArgument(
            "cmd_vel_relay_publish_twist_cmd_vel_mirror",
            default_value="true",
            description=(
                "Nar use_smoothed: true = publiser ogsaa speil-Twist paa /cmd_vel. "
                "False naar velocity_smoother allerede publiserer til /cmd_vel."
            ),
        ),
        DeclareLaunchArgument(
            "invert_cmd_vel_twist",
            default_value="false",
            description=(
                "Utgått: True = legacy (neger både linear.x og angular.z inn mot diff_drive, "
                "overskriver cmd_*_sign). Bruk false og sett cmd_linear_x_sign / cmd_angular_z_sign."
            ),
        ),
        DeclareLaunchArgument(
            "cmd_linear_x_sign",
            default_value="1.0",
            description=(
                "Skalering inn til diff_drive: out.linear.x = sign * /cmd_vel.linear.x "
                "(standard 1.0, bruk -1.0 kun midlertidig dersom URDF/controller fortsatt er speilet)."
            ),
        ),
        DeclareLaunchArgument(
            "cmd_angular_z_sign",
            default_value="1.0",
            description=(
                "Skalering inn til diff_drive: out.angular.z = sign * /cmd_vel.angular.z "
                "(typisk 1.0: ikke snu yaw-rate bare fordi linear er invertert)."
            ),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Vis Gazebo GUI (false = uten vindu/headless).",
        ),
        DeclareLaunchArgument(
            "physics_only_debug",
            default_value="false",
            description=(
                "Når true: use_rviz=false (isoler Gazebo+diff_drive). "
                "Tvinger ikke rocker_diff_debug, default forblir false (ingen RockerBogieDiff-spam). "
                "rocker_diff_coupling_mode default forblir weak."
            ),
        ),
        DeclareLaunchArgument(
            "rocker_diff_coupling_mode",
            default_value="weak",
            description=(
                "RockerBogieDifferential: off | weak | full. "
                "weak=standard i sim (lav PD, walking-beam + diff-hint). "
                "off=diagnose/passiv URDF uten plugin-krefter (kan mette begge rockere ~0,61 rad; ikke normal modus). "
                "full=stivere PD."
            ),
        ),
        DeclareLaunchArgument(
            "rocker_diff_debug",
            default_value="false",
            description=(
                "Throttlet gzmsg (qL,qR,bogie,diff,tau) i RockerBogieDifferential hvert print_interval. "
                "Kun true når du eksplisitt vil ha plugin-debug."
            ),
        ),
        DeclareLaunchArgument(
            "rocker_diff_print_interval",
            default_value="0.5",
            description="Sekunder mellom debug-linjer når rocker_diff_debug:=true.",
        ),
        DeclareLaunchArgument(
            "simple_collision_debug",
            default_value="false",
            description=(
                "Når true: kun hjul har kollisjon (ingen base/rocker/bogie/diff-bar STL); "
                "anbefalt for 6WD-traksjon i Gazebo. Stereo uten mesh-collision."
            ),
        ),
        DeclareLaunchArgument(
            "physics_profile",
            default_value="",
            description=(
                "Preset: safe_6wd | earth_stable_6wd | earth_6wd | realistic_6wd | debug_low_friction. "
                "Tom = bruk individuelle args (wheel_mu*, simple_collision_debug, spawn_z, …). "
                "world_preset setter default: moon til safe_6wd, earth til earth_stable_6wd."
            ),
        ),
        DeclareLaunchArgument(
            "wheel_mu1",
            default_value="0.12",
            description="ODE mu1 (langs fdir1 når wheel_use_fdir1). Overstyres av physics_profile.",
        ),
        DeclareLaunchArgument(
            "wheel_mu2",
            default_value="2.35",
            description="ODE mu2 (tverr på hjul). Overstyres av physics_profile.",
        ),
        DeclareLaunchArgument(
            "wheel_kp",
            default_value="9000.0",
            description="Hjul kontakt kp. Overstyres av physics_profile.",
        ),
        DeclareLaunchArgument(
            "wheel_kd",
            default_value="55.0",
            description="Hjul kontakt kd. Overstyres av physics_profile.",
        ),
        DeclareLaunchArgument(
            "wheel_use_fdir1",
            default_value="true",
            description="Anisotrop friksjon: fdir1 langs hjulakse Y. Overstyres av physics_profile.",
        ),
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
                " use_gazebo:=true",
                " enable_urdf_mimic:=false",
                " enable_triad_spectroscopy:=",
                LaunchConfiguration("enable_triad_spectroscopy"),
                " rocker_diff_coupling_mode:=",
                LaunchConfiguration("rocker_diff_coupling_mode"),
                " rocker_diff_debug:=",
                LaunchConfiguration("rocker_diff_debug"),
                " rocker_diff_print_interval:=",
                LaunchConfiguration("rocker_diff_print_interval"),
                " simple_collision_debug:=",
                LaunchConfiguration("simple_collision_debug"),
                " wheel_mu1:=",
                LaunchConfiguration("wheel_mu1"),
                " wheel_mu2:=",
                LaunchConfiguration("wheel_mu2"),
                " wheel_kp:=",
                LaunchConfiguration("wheel_kp"),
                " wheel_kd:=",
                LaunchConfiguration("wheel_kd"),
                " wheel_use_fdir1:=",
                LaunchConfiguration("wheel_use_fdir1"),
            ],
        ),
        value_type=str,
    )

    # Gi gz sim tilgang til meshene via resource path.
    set_ros_domain = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=LaunchConfiguration("ros_domain_id"),
    )
    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value=LaunchConfiguration("rmw_implementation"),
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([pkg, ".."]),
            ":",
            PathJoinSubstitution([pkg, "worlds"]),
            ":",
            PathJoinSubstitution([pkg, "meshes"]),
        ],
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            PathJoinSubstitution(
                [FindPackageShare("moonmapper_description"), "..", "..", "lib"],
            ),
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"],
                ),
            ],
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                TextSubstitution(text=" -r -v "),
                LaunchConfiguration("gz_sim_verbosity"),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            LaunchConfiguration("world_name"),
            "-name",
            "moonmapper",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    # Setter modellens pose etter spawn.
    force_set_pose = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            ["/world/", LaunchConfiguration("world_name"), "/set_pose"],
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            [
                'name: "moonmapper" ',
                "position { x: ",
                LaunchConfiguration("spawn_x"),
                " y: ",
                LaunchConfiguration("spawn_y"),
                " z: ",
                LaunchConfiguration("spawn_z"),
                " } ",
                "orientation { w: 1 x: 0 y: 0 z: 0 }",
            ],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("force_set_pose_after_spawn")),
    )

    print_pose = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "gz topic -e -t /world/",
                LaunchConfiguration("world_name"),
                "/pose/info -n 1 | sed -n '/name: \"moonmapper\"/,/orientation/p'",
            ],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("print_pose_after_spawn")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("bridge_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    # Spawner for joint_state_broadcaster.
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "-p",
            default_joint_state_broadcaster,
        ],
        output="screen",
    )

    # Spawner for diff_drive_controller.
    diff_relay_ekf_opaque = OpaqueFunction(
        function=lambda context: _diff_relay_ekf_chain(context, load_jsb=load_jsb),
    )

    # Aktiver joint_state_broadcaster først; deretter diff_drive og relay og EKF via opaque.
    after_spawn_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_rover,
            on_exit=[force_set_pose, print_pose, load_jsb],
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        args
        + [
            OpaqueFunction(function=_resolve_world_preset),
            OpaqueFunction(function=_apply_physics_profile),
            OpaqueFunction(function=_log_world_physics_summary),
            OpaqueFunction(function=_physics_only_debug),
            OpaqueFunction(function=_spawn_z_alias),
            set_ros_domain,
            set_rmw,
            set_gz_resource_path,
            set_gz_plugin_path,
            gz_sim,
            robot_state_publisher,
            spawn_rover,
            bridge,
            after_spawn_jsb,
            diff_relay_ekf_opaque,
            rviz,
        ],
    )
