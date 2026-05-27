#!/usr/bin/env python3
"""Frontier utforsker: velger mål fra /map og sender NavigateToPose til Nav2."""

from __future__ import annotations

import math
import time
import traceback
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Float32, String
from visualization_msgs.msg import MarkerArray

from moonmapper_nav2.frontier_exploration_memory import (
    ExploredMemory,
    exploration_complete,
    map_unknown_fraction,
)
from moonmapper_nav2.frontier_explorer_recovery import (
    RecoveryPhase,
    RecoveryRunner,
)
from moonmapper_nav2.frontier_explorer_debug_bridge import maybe_log_map_robot_diag
from moonmapper_nav2.frontier_explorer_goal_pick import pick_frontier_goal
from moonmapper_nav2.frontier_explorer_params import declare_frontier_explorer_parameters
from moonmapper_nav2.frontier_grid import ValidatedGoal
from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown

MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
FRONTIER_TOPIC_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _St(Enum):
    START = auto()
    WAIT_MAP = auto()
    WAIT_TF = auto()
    SPIN = auto()
    WAIT_NAV2 = auto()
    SELECT = auto()
    SEND = auto()
    NAV = auto()
    PAUSE = auto()
    RECOVERY = auto()
    BACKOUT = auto()
    RETURN_HOME = auto()
    NAV_HOME = auto()
    DONE = auto()


def _yaw_to_q(yaw: float) -> Quaternion:
    h = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


def _norm(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self._cb = ReentrantCallbackGroup()
        self._tick_g = MutuallyExclusiveCallbackGroup()
        declare_frontier_explorer_parameters(self)

        self._logged_map_tf_diag = False
        self._no_frontier_soft_retries = 0
        self._pause_is_no_goal_retry = False
        self._st = _St.START
        self._map: Optional[OccupancyGrid] = None
        self._blacklist: List[Tuple[float, float, float]] = []
        self._blacklist_failed: List[Tuple[float, float, float]] = []
        self._blacklist_clusters: List[Tuple[float, float, float]] = []
        self._clearance_override_m: Optional[float] = None
        self._send_future: Optional[Future] = None
        self._goal_handle = None
        self._result_future: Optional[Future] = None
        self._last_goal: Optional[Tuple[float, float]] = None
        self._last_validated_goal: Optional[ValidatedGoal] = None
        self._fail_streak = 0
        self._rescan_count = 0
        self._pending_goal: Optional[Tuple[float, float]] = None
        self._did_initial_spin = False
        self._spin_accum = 0.0
        self._spin_t0 = 0.0
        self._last_odom_yaw: Optional[float] = None
        self._map_after_spin = 0.0
        self._goal_deadline = 0.0
        self._start_at = self.get_clock().now() + Duration(
            seconds=float(self.get_parameter("start_delay_sec").value)
        )
        self._nav_wait_t0 = 0.0
        self._nav_log_t0 = 0.0
        self._nav_action_wait_logged = False
        self._wait_log_t0 = 0.0
        self._lc_order: Tuple[str, ...] = ("controller_server", "planner_server", "bt_navigator")
        self._lc_clients: Dict[str, object] = {}
        self._lc_idx = 0
        self._lc_fut: Optional[Future] = None
        self._obstacle_blocked_since: Optional[float] = None
        self._explored_memory: Optional[ExploredMemory] = None
        self._home_pose: Optional[Tuple[float, float, float]] = None
        self._backout_end: float = 0.0
        self._last_n_clusters: int = 0
        self._last_unknown_fraction: float = 1.0
        self._nav_after_send: _St = _St.NAV
        self._next_goal_pick_at: float = 0.0
        self._nav_started_at: float = 0.0
        self._nav_start_xy: Optional[Tuple[float, float]] = None
        self._sent_cluster_ids: Dict[int, float] = {}
        self._search_fail_attempts = 0
        self._recovery_cycles = 0
        self._exploration_t0 = 0.0
        self._select_entered_at = 0.0
        # Coverage-/stabilitetslogikk er fjernet. Utforsking er kun frontier-basert.
        self._nav_progress_xy: Optional[Tuple[float, float]] = None
        self._nav_progress_yaw: Optional[float] = None
        self._nav_progress_t0 = 0.0
        self._same_goal_failures = 0
        self._last_pick_log_t = 0.0
        self._home_saved_published = False
        self._return_home_retry = False
        self._last_front_min: Optional[float] = None
        self._last_obstacle_state: str = ""
        self._too_close_since: Optional[float] = None
        self._no_turn_since: Optional[float] = None
        self._too_close_start_xy: Optional[Tuple[float, float]] = None
        self._last_cmd_vel_raw_x: float = 0.0

        self._tf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        tf2_ros.TransformListener(self._tf, self, spin_thread=False)

        nav_topic = str(self.get_parameter("navigate_action").value)
        self._nav = ActionClient(self, NavigateToPose, nav_topic, callback_group=self._cb)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd = self.create_publisher(Twist, cmd_topic, 10)
        self._cmd_spin: Optional[object] = None
        if bool(self.get_parameter("initial_spin_direct_cmd_vel").value):
            spin_topic = str(self.get_parameter("initial_spin_cmd_vel_topic").value)
            self._cmd_spin = self.create_publisher(Twist, spin_topic, 10)
            self.get_logger().info(
                f"startspin publiserer til {spin_topic} (utenom collision_monitor pa {cmd_topic})"
            )
        self._pub_stat = self.create_publisher(String, "/frontier_explorer/status", FRONTIER_TOPIC_QOS)
        self._pub_goal = self.create_publisher(PoseStamped, "/frontier_explorer/current_goal", FRONTIER_TOPIC_QOS)
        self._pub_start = self.create_publisher(
            PoseStamped, "/frontier_explorer/start_pose", FRONTIER_TOPIC_QOS
        )
        self._pub_return_goal = self.create_publisher(
            PoseStamped, "/frontier_explorer/return_home_goal", FRONTIER_TOPIC_QOS
        )
        self._pub_mk = self.create_publisher(MarkerArray, "/frontier_explorer/markers", 10)
        self._pub_debug_mk = None
        if bool(self.get_parameter("publish_debug_markers").value):
            self._pub_debug_mk = self.create_publisher(
                MarkerArray, "/frontier_explorer/debug_markers", 10
            )
        self._recovery = RecoveryRunner(
            self.get_parameter,
            lambda tw: self._cmd.publish(tw),
            self._odom_yaw,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            MAP_QOS,
            callback_group=self._cb,
        )
        self.create_subscription(
            String,
            "/obstacle/current_state",
            self._on_obstacle_state,
            10,
            callback_group=self._cb,
        )
        self.create_subscription(
            Float32,
            "/obstacle/front_min",
            self._on_front_min,
            10,
            callback_group=self._cb,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self._on_cmd_vel_raw,
            10,
            callback_group=self._cb,
        )
        for nm in self._lc_order:
            self._lc_clients[nm] = self.create_client(
                GetState, f"/{nm}/get_state", callback_group=self._cb
            )

        hz = max(0.2, float(self.get_parameter("exploration_rate_hz").value))
        self.create_timer(1.0 / hz, self._tick, callback_group=self._tick_g)
        self._status = ""

        mt = str(self.get_parameter("map_topic").value)
        cv = str(self.get_parameter("cmd_vel_topic").value)
        bf = str(self.get_parameter("base_frame").value)
        mf = str(self.get_parameter("map_frame").value)
        nav = str(self.get_parameter("navigate_action").value)
        lg = self.get_logger()
        lg.info("[frontier_explorer] STARTET (V1)")
        lg.info("[frontier_explorer] Venter pa kart / TF / Nav2 (se /frontier_explorer/status)")
        lg.info(
            "[frontier_explorer] config: "
            f"map_topic={mt} map_frame={mf} base_frame={bf} cmd_vel_topic={cv} navigate_action={nav}"
        )
        self._transition("STARTUP", "node_started")
        ps_ad = PoseStamped()
        ps_ad.header.frame_id = mf
        ps_ad.header.stamp = self.get_clock().now().to_msg()
        self._pub_goal.publish(ps_ad)
        self._log_frontier_config()

    def _transition(self, status: str, reason: str = "") -> None:
        prev = self._status
        if status == prev:
            return
        msg = f"STATE {prev} til {status}" if prev else f"STATE til {status}"
        if reason:
            msg += f" reason={reason}"
        self.get_logger().info(f"[frontier_explorer] {msg}")
        self._status = status
        self._pub_stat.publish(String(data=status))

    def _log_frontier_config(self) -> None:
        lg = self.get_logger()
        lg.info(
            "FRONTIER_CONFIG "
            f"initial_spin={bool(self.get_parameter('integrated_initial_spin').value)} "
            f"return_home={bool(self.get_parameter('return_home_enabled').value)} "
            f"decision_timeout_sec={float(self.get_parameter('decision_timeout_sec').value):.1f} "
            f"min_goal_distance_m={float(self.get_parameter('min_goal_distance_m').value):.2f} "
            f"max_goal_distance_m={float(self.get_parameter('max_goal_distance_m').value):.2f} "
            f"enable_recovery={bool(self.get_parameter('enable_recovery_maneuvers').value)}"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _stat(self, s: str) -> None:
        self._transition(s)

    def _pose_map(self) -> Optional[Tuple[float, float, float]]:
        mf = str(self.get_parameter("map_frame").value)
        bf = str(self.get_parameter("base_frame").value)
        try:
            t = self._tf.lookup_transform(mf, bf, Time(), timeout=Duration(seconds=0.25))
            tr = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return tr.x, tr.y, yaw
        except tf2_ros.TransformException:
            return None

    def _tf_ok(self, a: str, b: str) -> bool:
        try:
            self._tf.lookup_transform(a, b, Time(), timeout=Duration(seconds=0.2))
            return True
        except tf2_ros.TransformException:
            return False

    def _odom_yaw(self) -> Optional[float]:
        bf = str(self.get_parameter("base_frame").value)
        try:
            t = self._tf.lookup_transform("odom", bf, Time(), timeout=Duration(seconds=0.15))
            q = t.transform.rotation
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        except tf2_ros.TransformException:
            return None

    def _cmd_ok(self) -> bool:
        if not bool(self.get_parameter("require_cmd_vel_subscriber").value):
            return True
        if self._st == _St.SPIN and self._cmd_spin is not None:
            t = str(self.get_parameter("initial_spin_cmd_vel_topic").value)
        else:
            t = str(self.get_parameter("cmd_vel_topic").value)
        return self.count_subscribers(t) >= 1

    def _publish_cmd(self, tw: Twist, *, spin: bool = False) -> None:
        if spin and self._cmd_spin is not None:
            self._cmd_spin.publish(tw)
        else:
            self._cmd.publish(tw)

    def _on_obstacle_state(self, msg: String) -> None:
        self._last_obstacle_state = msg.data
        blocked = msg.data in (
            "blocked_front",
            "stop_turn",
            "stop",
            "backup_required",
            "emergency_stop",
        )
        if blocked:
            if self._obstacle_blocked_since is None:
                self._obstacle_blocked_since = time.monotonic()
        else:
            self._obstacle_blocked_since = None

    def _on_cmd_vel_raw(self, msg: Twist) -> None:
        self._last_cmd_vel_raw_x = float(msg.linear.x)

    def _on_front_min(self, msg: Float32) -> None:
        v = float(msg.data)
        if math.isnan(v) or math.isinf(v):
            self._last_front_min = None
        else:
            self._last_front_min = v

    def _start_too_close_recovery(self, reason: str) -> None:
        self.get_logger().warn(f"TOO_CLOSE_RECOVERY_TRIGGERED reason={reason}")
        if self._last_goal is not None:
            wx, wy = self._last_goal
            br = float(self.get_parameter("recovery_blacklist_radius_m").value)
            self._blacklist_failed.append((wx, wy, time.monotonic()))
            self.get_logger().info(
                f"BLACKLIST_GOAL world=({wx:.2f},{wy:.2f}) radius={br:.2f}"
            )
        self._too_close_since = None
        self._no_turn_since = None
        self._start_recovery(reason, RecoveryPhase.BACKUP)

    def _mark_explored_current(self) -> None:
        if self._map is None:
            return
        pose = self._pose_map()
        if pose is None:
            return
        rx, ry, _ = pose
        info = self._map.info
        w, h = int(info.width), int(info.height)
        if w <= 0 or h <= 0:
            return
        if self._explored_memory is None:
            self._explored_memory = ExploredMemory(w, h)
        self._explored_memory.mark_world(
            rx,
            ry,
            float(self.get_parameter("explored_mark_radius_m").value),
            float(info.origin.position.x),
            float(info.origin.position.y),
            float(info.resolution),
            w,
            h,
        )

    def _explored_cell_fn(self, w: int, h: int):
        mem = self._explored_memory

        def _fn(mx: int, my: int) -> bool:
            if mem is None:
                return False
            return mem.is_explored(mx, my, w, h)

        return _fn

    def _pose_to_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = str(self.get_parameter("map_frame").value)
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = _yaw_to_q(yaw)
        return ps

    def _save_home_pose_if_needed(self) -> None:
        if self._home_pose is not None:
            return
        pose = self._pose_map()
        if pose is None:
            return
        self._home_pose = (pose[0], pose[1], pose[2])
        self.get_logger().info(
            f"START_POSE_SAVED map=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f})"
        )
        self._pub_start.publish(self._pose_to_stamped(pose[0], pose[1], pose[2]))
        self._home_saved_published = True
        self._transition("SAVE_START_POSE", "nav2_ready")

    def _should_return_home(self, why: str) -> Tuple[bool, str]:
        ret_on = bool(self.get_parameter("return_home_enabled").value) or bool(
            self.get_parameter("enable_return_home_on_complete").value
        )
        if not ret_on or not bool(self.get_parameter("return_home_on_completion").value):
            return False, ""
        if not bool(self.get_parameter("completion_check_enabled").value):
            return False, ""
        if self._home_pose is None or not self._did_initial_spin:
            return False, ""
        if why not in ("no_clusters", "no_valid", "no_seed"):
            return False, ""
        nowm = time.monotonic()
        if self._exploration_t0 <= 0.0:
            return False, ""
        if nowm - self._exploration_t0 < float(self.get_parameter("min_exploration_time_sec").value):
            return False, ""
        min_nf = int(self.get_parameter("completion_no_frontier_attempts").value)
        if self._search_fail_attempts < min_nf:
            return False, ""
        max_rec = int(self.get_parameter("max_recovery_cycles").value)
        if self._recovery_cycles < min(max_rec, 1):
            return False, ""
        complete, reason = exploration_complete(
            self._last_n_clusters,
            self._last_unknown_fraction,
            float(self.get_parameter("reachable_unknown_threshold_ratio").value),
            int(self.get_parameter("min_frontier_clusters_to_continue").value),
        )
        if complete:
            return True, f"no_frontiers_and_map_stable:{reason}"
        if (
            self._search_fail_attempts >= min_nf
            and self._recovery_cycles >= int(self.get_parameter("max_recovery_cycles").value)
        ):
            return True, "no_frontiers_after_recovery"
        return False, ""

    def _start_recovery(self, reason: str, phase: RecoveryPhase = RecoveryPhase.SPIN) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._result_future = None
        self._send_future = None
        self._zero()
        self._recovery_cycles += 1
        self._recovery.start(phase)
        self._st = _St.RECOVERY
        self._transition("RECOVERY_MANEUVER", reason)
        self.get_logger().info(f"RECOVERY_MANEUVER phase={phase.name} cycle={self._recovery_cycles}")

    def _maybe_start_recovery_instead_of_pause(self, why: str) -> bool:
        if not bool(self.get_parameter("enable_recovery_maneuvers").value):
            return False
        max_rec = int(self.get_parameter("max_recovery_cycles").value)
        if self._recovery_cycles >= max_rec:
            return False
        need = int(self.get_parameter("max_search_attempts_before_recovery").value)
        if self._search_fail_attempts < need:
            return False
        if self._goal_handle is not None or self._st in (_St.SEND, _St.NAV, _St.NAV_HOME):
            return False
        phase = RecoveryPhase.SPIN
        if why == "no_seed" or self._obstacle_blocked_since is not None:
            phase = RecoveryPhase.BACKUP_TURN
        self._start_recovery(f"no_valid_frontier_timeout:{why}", phase)
        return True

    def _note_cluster_sent(self, cluster_id: int) -> None:
        self._sent_cluster_ids[cluster_id] = time.monotonic()

    def _cluster_recently_sent(self, cluster_id: int, cooldown_sec: float) -> bool:
        t0 = self._sent_cluster_ids.get(cluster_id)
        if t0 is None:
            return False
        return (time.monotonic() - t0) < cooldown_sec

    def _schedule_next_goal_pick(self, delay_sec: float) -> None:
        self._next_goal_pick_at = max(
            self._next_goal_pick_at, time.monotonic() + max(0.0, delay_sec)
        )

    def _start_backout(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._result_future = None
        self._send_future = None
        dur = float(self.get_parameter("backout_duration_sec").value)
        self._backout_end = time.monotonic() + max(0.3, dur)
        self._st = _St.BACKOUT
        self._stat("BACKOUT_MANEUVER")
        self.get_logger().info(
            f"BACKOUT_MANEUVER duration={dur:.1f}s speed="
            f"{float(self.get_parameter('backout_speed_mps').value):.2f}"
        )

    def _handle_goal_failed(self, reason: str) -> None:
        """Blacklist failed goal and schedule retry (recovery/backout or pause)."""
        self._blacklist_goal()
        self._obstacle_blocked_since = None
        obstacle_reasons = (
            "no_progress_during_navigation",
            "front blocked too long during navigation",
            "goal_timeout",
        )
        use_backout = bool(
            self.get_parameter("enable_active_backout_on_failure").value
        ) or (
            reason in obstacle_reasons
            or "blocked" in reason
            or "obstacle" in reason
        )
        if use_backout:
            self._start_backout()
            self.get_logger().warn(f"Goal failed ({reason}) — backout, then retry")
            return
        delay = float(self.get_parameter("failure_retry_delay_sec").value)
        self._schedule_next_goal_pick(delay)
        self._st = _St.PAUSE
        self._goal_deadline = time.monotonic() + delay
        self._transition("GOAL_FAILED", reason)
        self.get_logger().warn(
            f"Goal failed ({reason}) — blacklisted, retry in {delay:.1f}s"
        )

    def _send_nav_goal(self, wx: float, wy: float, yaw: float, after_send: _St = _St.NAV) -> None:
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = str(self.get_parameter("map_frame").value)
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        ps.pose.position.z = 0.0
        ps.pose.orientation = _yaw_to_q(yaw)
        goal.pose = ps
        self._pending_goal = (wx, wy)
        self._pub_goal.publish(ps)
        self._nav_after_send = after_send
        self._send_future = self._nav.send_goal_async(goal)
        self._st = _St.SEND

    # sjekk om roboten er for nær et mål og start recovery  
    def _check_too_close_recovery(self, nowm: float) -> bool:
        if not bool(self.get_parameter("enable_too_close_recovery").value):
            return False
        if self._st != _St.NAV:
            return False
        if nowm - self._nav_started_at < float(self.get_parameter("min_nav_commit_sec").value):
            return False
        if self._last_front_min is None:
            return False
        emergency = float(self.get_parameter("too_close_distance_m").value)
        fm = self._last_front_min
        if fm > emergency:
            self._too_close_since = None
            self._too_close_start_xy = None
            return False
        pose = self._pose_map()
        if self._too_close_since is None:
            self._too_close_since = nowm
            if pose is not None:
                self._too_close_start_xy = (pose[0], pose[1])
            return False
        sustain = float(self.get_parameter("too_close_sustain_sec").value)
        if nowm - self._too_close_since < sustain:
            return False
        moved = 0.0
        if pose is not None and self._too_close_start_xy is not None:
            moved = math.hypot(pose[0] - self._too_close_start_xy[0], pose[1] - self._too_close_start_xy[1])
        if moved >= float(self.get_parameter("too_close_min_progress_m").value):
            self._too_close_since = None
            self._too_close_start_xy = None
            return False
        if bool(self.get_parameter("enable_no_turn_recovery").value):
            if pose is not None and self._nav_progress_yaw is not None:
                yd = abs(_norm(pose[2] - self._nav_progress_yaw))
                if yd < float(self.get_parameter("min_yaw_progress_rad").value):
                    if self._no_turn_since is None:
                        self._no_turn_since = nowm
                    elif nowm - self._no_turn_since >= float(
                        self.get_parameter("no_turn_timeout_sec").value
                    ):
                        self.get_logger().info(
                            f"NO_TURN_DETECTED yaw_delta={yd:.3f} front_min={fm:.2f}"
                        )
                        if self._goal_handle is not None:
                            try:
                                self._goal_handle.cancel_goal_async()
                            except Exception:
                                pass
                        self._goal_handle = None
                        self._result_future = None
                        self._start_too_close_recovery("no_turn_space")
                        return True
                else:
                    self._no_turn_since = None
        self.get_logger().warn(
            f"TOO_CLOSE_RECOVERY_TRIGGERED front_min={fm:.2f}m moved={moved:.2f}m "
            f"sustain={sustain:.0f}s"
        )
        if self._goal_handle is not None:
            self.get_logger().info("CANCEL_NAV2_GOAL reason=too_close_emergency")
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._result_future = None
        self._too_close_since = None
        self._too_close_start_xy = None
        self._start_too_close_recovery("too_close_emergency")
        return True

    def _cancel_nav_stuck(self, reason: str) -> None:
        self.get_logger().warn(f"[frontier_explorer] {reason} - avbryter mal, blacklist, recovery")
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._result_future = None
        self._fail_streak += 1
        self._obstacle_blocked_since = None
        self._handle_goal_failed(reason)
        self._stat("STUCK_BLOCKED")

    def _prune_bl(self) -> None:
        now = time.monotonic()
        to = float(self.get_parameter("blacklist_duration_sec").value)
        if to <= 0.0:
            to = float(self.get_parameter("blacklist_timeout_sec").value)
        self._blacklist = [(x, y, tt) for x, y, tt in self._blacklist if now - tt < to]
        self._blacklist_failed = [(x, y, tt) for x, y, tt in self._blacklist_failed if now - tt < to]
        self._blacklist_clusters = [
            (x, y, tt) for x, y, tt in self._blacklist_clusters if now - tt < to
        ]

    def _lc_step(self) -> bool:
        if not bool(self.get_parameter("require_nav_lifecycle_active").value):
            return True
        if self._lc_idx >= len(self._lc_order):
            return True
        nm = self._lc_order[self._lc_idx]
        cl = self._lc_clients[nm]
        if not cl.service_is_ready():  # type: ignore[union-attr]
            return False
        if self._lc_fut is None:
            self._lc_fut = cl.call_async(GetState.Request())  # type: ignore[union-attr]
        if not self._lc_fut.done():
            return False
        res = self._lc_fut.result()
        self._lc_fut = None
        ok = res is not None and int(res.current_state.id) == State.PRIMARY_STATE_ACTIVE
        if not ok:
            self._lc_idx = 0
            return False
        self._lc_idx += 1
        return self._lc_idx >= len(self._lc_order)


    def _zero(self) -> None:
        try:
            z = Twist()
            self._cmd.publish(z)
            if self._cmd_spin is not None:
                self._cmd_spin.publish(z)
        except Exception:
            pass

    def _done(self, msg: str) -> None:
        self._st = _St.DONE
        self._recovery.stop()
        self._zero()
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._transition("DONE", msg)

    def _tick(self) -> None:
        try:
            if not rclpy.ok():
                return
            self._tick_impl()
        except Exception:
            if rclpy.ok():
                self.get_logger().error(
                    f"frontier_explorer tick-feil:\n{traceback.format_exc()}"
                )
            self._zero()
            if rclpy.ok():
                self._done("EXCEPTION")

    def _tick_impl(self) -> None:
        # Tilstandsmaskin: vent Nav2 til velg mal til NavigateToPose til recovery/home.
        if not rclpy.ok() or self._st == _St.DONE:
            return
        if self._st not in (_St.BACKOUT, _St.DONE, _St.RECOVERY):
            self._mark_explored_current()
        if self.get_clock().now() < self._start_at:
            self._stat("WAIT_START_DELAY")
            return

        if self._st == _St.START:
            mp = str(self.get_parameter("map_topic").value)
            if bool(self.get_parameter("require_map_publisher_for_ready").value):
                if self.count_publishers(mp) < 1:
                    self._throttle_log(f"wait publisher {mp}")
                    self._stat("WAIT_MAP_PUBLISHER")
                    return
            self._st = _St.WAIT_MAP
            return

        if self._st == _St.WAIT_MAP:
            if self._map_after_spin > 0.0:
                self._zero()
                if time.monotonic() < self._map_after_spin:
                    self._stat("WAIT_MAP_AFTER_SPIN")
                    return
                self._map_after_spin = 0.0
                fast = bool(self.get_parameter("explore_immediately_after_spin").value)
                if fast and (
                    not bool(self.get_parameter("wait_for_nav2").value)
                    or self._nav.server_is_ready()
                ):
                    self._lc_idx = 0
                    self._lc_fut = None
                    if not bool(self.get_parameter("require_nav_lifecycle_active").value):
                        self._st = _St.SELECT
                        self._stat("EXPLORE_AFTER_SPIN")
                        return
                self._begin_nav2()
                return
            if self._map is None or len(self._map.data) == 0:
                self._throttle_log("wait /map data")
                self._stat("WAITING_FOR_MAP")
                return
            w, h = int(self._map.info.width), int(self._map.info.height)
            if w <= 0 or h <= 0:
                self._stat("WAITING_FOR_MAP")
                return
            mf = str(self.get_parameter("map_frame").value)
            bf = str(self.get_parameter("base_frame").value)
            if not self._tf_ok(mf, "odom") or not self._tf_ok("odom", bf):
                self._throttle_log("wait TF map til odom til base")
                self._st = _St.WAIT_TF
                self._stat("WAITING_FOR_TF")
                return
            pm = self._pose_map()
            if pm is not None:
                maybe_log_map_robot_diag(self, pm[0], pm[1])
            if (
                bool(self.get_parameter("integrated_initial_spin").value)
                and not self._did_initial_spin
            ):
                self._st = _St.SPIN
                self._last_odom_yaw = None
                self._spin_accum = 0.0
                self._stat("INITIAL_SPIN")
                return
            self._begin_nav2()
            return

        if self._st == _St.WAIT_TF:
            mf = str(self.get_parameter("map_frame").value)
            bf = str(self.get_parameter("base_frame").value)
            if not self._tf_ok(mf, "odom") or not self._tf_ok("odom", bf):
                self._throttle_log("wait TF map til base_footprint")
                self._stat("WAITING_FOR_TF")
                return
            pm = self._pose_map()
            if pm is not None:
                maybe_log_map_robot_diag(self, pm[0], pm[1])
            if (
                bool(self.get_parameter("integrated_initial_spin").value)
                and not self._did_initial_spin
            ):
                self._st = _St.SPIN
                self._last_odom_yaw = None
                self._spin_accum = 0.0
                self._stat("INITIAL_SPIN")
                return
            self._begin_nav2()
            return

        if self._st == _St.SPIN:
            if not self._cmd_ok():
                t = str(self.get_parameter("cmd_vel_topic").value)
                self._throttle_log(f"waiting for subscriber on {t} before initial spin")
                self._stat("WAIT_CMD_VEL_SUB")
                return
            y = self._odom_yaw()
            nowt = time.monotonic()
            if y is None:
                return
            if self._last_odom_yaw is None:
                self._last_odom_yaw = y
                self._spin_accum = 0.0
                self._spin_t0 = nowt
            dy = _norm(y - self._last_odom_yaw)
            self._spin_accum += abs(dy)
            self._last_odom_yaw = y
            tgt = float(self.get_parameter("initial_spin_target_rad").value)
            mn = float(self.get_parameter("initial_spin_min_duration_sec").value)
            mx = float(self.get_parameter("initial_spin_max_duration_sec").value)
            az = float(self.get_parameter("initial_spin_angular_z").value)
            tw = Twist()
            tw.angular.z = az
            self._publish_cmd(tw, spin=True)
            if (self._spin_accum >= tgt and (nowt - self._spin_t0) >= mn) or (nowt - self._spin_t0) >= mx:
                self._zero()
                self._did_initial_spin = True
                self._map_after_spin = nowt + float(
                    self.get_parameter("map_settle_after_spin_sec").value
                )
                self._st = _St.WAIT_MAP
                return

        if self._st == _St.WAIT_NAV2:
            if not bool(self.get_parameter("wait_for_nav2").value):
                self._st = _St.SELECT
                return
            nowm = time.monotonic()
            if self._nav_wait_t0 <= 0.0:
                self._nav_wait_t0 = nowm
            tout = float(self.get_parameter("nav2_ready_timeout_sec").value)
            if nowm - self._nav_wait_t0 > tout:
                self.get_logger().error("Nav2 ble ikke klar innen tidsfrist")
                self._zero()
                self._done("NAV2_TIMEOUT")
                return
            iv = float(self.get_parameter("nav2_debug_log_interval_sec").value)
            if nowm - self._nav_log_t0 >= iv:
                self._nav_log_t0 = nowm
                self.get_logger().info(
                    f"venter Nav2 action={self._nav.server_is_ready()} lc_idx={self._lc_idx} "
                    f"(trenger /navigate_to_pose + lifecycle ACTIVE)"
                )
            action_wait = float(self.get_parameter("nav2_action_wait_sec").value)
            if not self._nav.server_is_ready():
                if not self._nav_action_wait_logged:
                    self.get_logger().info(
                        f"Venter opptil {action_wait:.1f}s pa NavigateToPose action-server"
                    )
                    self._nav_action_wait_logged = True
                if not self._nav.wait_for_server(timeout_sec=action_wait):
                    self._stat("WAITING_FOR_NAV2")
                    self._lc_idx = 0
                    self._lc_fut = None
                    return
            self._nav_action_wait_logged = False
            if bool(self.get_parameter("require_nav_lifecycle_active").value):
                if not self._lc_step():
                    nm = self._lc_order[min(self._lc_idx, len(self._lc_order) - 1)]
                    self._stat("WAITING_FOR_NAV2_LC")
                    self._throttle_log(f"venter Nav2 lifecycle ACTIVE: {nm}")
                    return
            self.get_logger().info("[frontier_explorer] Nav2 klar, lifecycle ACTIVE")
            self._save_home_pose_if_needed()
            self._lc_idx = 0
            self._lc_fut = None
            self._st = _St.SELECT
            self._stat("NAV2_ACTIVE")
            ps = PoseStamped()
            ps.header.frame_id = str(self.get_parameter("map_frame").value)
            ps.header.stamp = self.get_clock().now().to_msg()
            self._pub_goal.publish(ps)
            return

        if self._st == _St.RECOVERY:
            if self._goal_handle is not None:
                self._zero()
                return
            pose = self._pose_map()
            if self._recovery.tick(pose):
                self._search_fail_attempts = max(0, self._search_fail_attempts - 1)
                self._st = _St.SELECT
                self._select_entered_at = time.monotonic()
                self._transition("SEARCH_FRONTIER", "recovery_done")
            return

        if self._st == _St.SELECT:
            nowm = time.monotonic()
            if self._exploration_t0 <= 0.0 and self._did_initial_spin:
                self._exploration_t0 = nowm
            if self._select_entered_at <= 0.0:
                self._select_entered_at = nowm
            if nowm < self._next_goal_pick_at:
                return
            self._transition("SEARCH_FRONTIER")
            g, why = pick_frontier_goal(self)
            if g is None:
                soft_whys = {"no_clusters", "no_seed", "no_valid"}
                retry_nf = bool(self.get_parameter("retry_when_no_frontier").value)
                max_nf = int(self.get_parameter("max_no_frontier_retries").value)
                delay_nf = float(self.get_parameter("no_frontier_retry_delay_sec").value)

                use_soft = retry_nf and why in soft_whys and (
                    max_nf == 0 or self._no_frontier_soft_retries < max_nf
                )

                if use_soft:
                    self._no_frontier_soft_retries += 1
                    self._search_fail_attempts += 1
                    if self._maybe_start_recovery_instead_of_pause(why):
                        return
                    self.get_logger().warn(
                        "Ingen mal (%s), myk retry pause %.1fs (%s/%s)"
                        % (
                            why,
                            delay_nf,
                            self._no_frontier_soft_retries,
                            "∞" if max_nf == 0 else str(max_nf),
                        )
                    )
                    self._fail_streak += 1
                    self._st = _St.PAUSE
                    self._goal_deadline = time.monotonic() + delay_nf
                    self._pause_is_no_goal_retry = True
                    self._transition("SEARCH_FRONTIER", f"soft_retry:{why}")
                    return

                self._no_frontier_soft_retries = 0
                self._rescan_count += 1
                self._search_fail_attempts += 1
                if self._maybe_start_recovery_instead_of_pause(why):
                    return
                go_home, home_reason = self._should_return_home(why)
                if go_home:
                    self.get_logger().info(
                        f"EXPLORATION_COMPLETE reason={home_reason} - kjorer hjem"
                    )
                    self._st = _St.RETURN_HOME
                    self._transition("CHECK_COMPLETION", home_reason)
                    return
                if self._rescan_count > int(self.get_parameter("max_rescan_cycles").value):
                    go_home2, hr2 = self._should_return_home(why)
                    if go_home2:
                        self.get_logger().info(
                            f"EXPLORATION_COMPLETE reason={hr2} - kjorer hjem"
                        )
                        self._st = _St.RETURN_HOME
                        self._transition("CHECK_COMPLETION", hr2)
                        return
                    self._done("NO_VALID_FRONTIER")
                    return
                self.get_logger().warn(f"Ingen mal ({why}), kort retry")
                self._fail_streak += 1
                self._st = _St.PAUSE
                self._goal_deadline = time.monotonic() + min(
                    float(self.get_parameter("recovery_pause_sec").value),
                    float(self.get_parameter("decision_timeout_sec").value),
                )
                self._pause_is_no_goal_retry = False
                return
            self._rescan_count = 0
            self._no_frontier_soft_retries = 0
            self._search_fail_attempts = 0
            self._select_entered_at = 0.0
            self._last_validated_goal = g
            self._same_goal_failures = 0
            self._note_cluster_sent(g.cluster_id)
            pose = self._pose_map()
            if pose is not None:
                self._nav_start_xy = (pose[0], pose[1])
                self._nav_progress_xy = (pose[0], pose[1])
                self._nav_progress_yaw = pose[2]
            self._nav_started_at = time.monotonic()
            self._nav_progress_t0 = self._nav_started_at
            self.get_logger().info(
                f"GOAL_COMMITTED cluster_id={g.cluster_id} world=({g.wx:.2f},{g.wy:.2f}) "
                f"dist_robot={math.hypot(g.wx - pose[0], g.wy - pose[1]) if pose else 0:.2f}m"
            )
            self.get_logger().info(
                f"GOAL_SELECTED world=({g.wx:.2f},{g.wy:.2f}) approach={g.approach_ixy} "
                f"method={g.approach_method}"
            )
            self.get_logger().info("SENDING_NAV2_GOAL")
            self._send_nav_goal(g.wx, g.wy, g.yaw, _St.NAV)
            self._transition("SELECT_GOAL", "valid_frontier_found")
            self._transition("SEND_GOAL")
            return

        if self._st == _St.RETURN_HOME:
            self._transition("RETURN_HOME")
            if self._home_pose is None:
                self._done("DONE")
                return
            hx, hy, hyaw = self._home_pose
            ps = self._pose_to_stamped(hx, hy, hyaw)
            self._pub_return_goal.publish(ps)
            self.get_logger().info(
                f"RETURN_HOME_GOAL_SENT map=({hx:.2f},{hy:.2f},{hyaw:.2f})"
            )
            self._send_nav_goal(hx, hy, hyaw, _St.NAV_HOME)
            return

        if self._st == _St.BACKOUT:
            if time.monotonic() < self._backout_end:
                tw = Twist()
                tw.linear.x = -abs(float(self.get_parameter("backout_speed_mps").value))
                self._cmd.publish(tw)
                return
            self._zero()
            self._st = _St.SELECT
            self._stat("BACKOUT_DONE")
            return

        if self._st == _St.SEND:
            if self._send_future is None or not self._send_future.done():
                return
            gh = self._send_future.result()
            self._send_future = None
            if gh is None or not gh.accepted:
                self.get_logger().warn(
                    "[frontier_explorer] NavigateToPose rejected (Nav2 not executing goal)"
                )
                self._last_goal = self._pending_goal
                self._pending_goal = None
                self._fail_streak += 1
                self._handle_goal_failed("nav2_rejected")
                return
            self._goal_handle = gh
            self._result_future = gh.get_result_async()
            self._st = self._nav_after_send
            self._goal_deadline = time.monotonic() + float(
                self.get_parameter("goal_timeout_sec").value
            )
            self._last_goal = self._pending_goal
            self._pending_goal = None
            if self._st == _St.NAV:
                self._transition("NAVIGATING")
            elif self._st == _St.NAV_HOME:
                self._transition("RETURN_HOME")
            return

        if self._st == _St.NAV:
            nowm = time.monotonic()
            if self._check_too_close_recovery(nowm):
                return
            min_commit = float(self.get_parameter("min_nav_commit_sec").value)
            pose = self._pose_map()
            if (
                bool(self.get_parameter("enable_nav_stuck_detection").value)
                and pose is not None
                and self._nav_progress_xy is not None
            ):
                prog = math.hypot(pose[0] - self._nav_progress_xy[0], pose[1] - self._nav_progress_xy[1])
                yprog = 0.0
                if self._nav_progress_yaw is not None:
                    yprog = abs(_norm(pose[2] - self._nav_progress_yaw))
                if prog >= float(self.get_parameter("min_progress_m").value) or yprog >= float(
                    self.get_parameter("min_yaw_progress_rad").value
                ):
                    self._nav_progress_xy = (pose[0], pose[1])
                    self._nav_progress_yaw = pose[2]
                    self._nav_progress_t0 = nowm
                elif (
                    nowm - self._nav_progress_t0 >= float(
                        self.get_parameter("stuck_timeout_sec").value
                    )
                    and nowm - self._nav_started_at >= min_commit
                    and abs(self._last_cmd_vel_raw_x) > 0.02
                ):
                    lg = self.get_logger()
                    gxy = self._last_goal or (0.0, 0.0)
                    lg.warn(
                        f"STUCK_DETECTED goal=({gxy[0]:.2f},{gxy[1]:.2f}) progress={prog:.2f}m "
                        f"timeout={float(self.get_parameter('stuck_timeout_sec').value):.1f}s"
                    )
                    lg.info("CANCEL_NAV2_GOAL")
                    self._cancel_nav_stuck("no_progress_during_navigation")
                    return
            if pose is not None and self._nav_progress_xy is None:
                self._nav_progress_xy = (pose[0], pose[1])
                self._nav_progress_yaw = pose[2]
                self._nav_progress_t0 = nowm
            if nowm - self._nav_started_at < min_commit:
                pass
            else:
                stuck_sec = float(self.get_parameter("stuck_blocked_cancel_sec").value)
                if (
                    self._obstacle_blocked_since is not None
                    and self._last_obstacle_state
                    in ("stop", "backup_required", "emergency_stop", "blocked_front")
                    and nowm - self._obstacle_blocked_since >= stuck_sec
                ):
                    self._cancel_nav_stuck("front blocked too long during navigation")
                    return
            if self._result_future is not None and self._result_future.done():
                wrapped = self._result_future.result()
                self._goal_handle = None
                self._result_future = None
                st = int(wrapped.status) if wrapped is not None else -1
                if st == GoalStatus.STATUS_SUCCEEDED:
                    moved = 0.0
                    pose = self._pose_map()
                    if pose is not None and self._nav_start_xy is not None:
                        moved = math.hypot(
                            pose[0] - self._nav_start_xy[0], pose[1] - self._nav_start_xy[1]
                        )
                    min_travel = float(self.get_parameter("min_travel_after_goal_m").value)
                    if nowm - self._nav_started_at < min_commit or moved < min_travel:
                        pause = float(self.get_parameter("min_pause_after_success_sec").value)
                        pause += max(0.0, min_commit - (nowm - self._nav_started_at))
                        self.get_logger().info(
                            f"GOAL_SUCCESS_TOO_EARLY moved={moved:.2f}m "
                            f"nav_t={nowm - self._nav_started_at:.1f}s pause={pause:.1f}s"
                        )
                        self._schedule_next_goal_pick(pause)
                        self._st = _St.PAUSE
                        self._goal_deadline = time.monotonic() + pause
                        self._transition("GOAL_REACHED", "too_early")
                        return
                    self._fail_streak = 0
                    pause = float(self.get_parameter("min_pause_after_success_sec").value)
                    self._schedule_next_goal_pick(pause)
                    self._st = _St.PAUSE
                    self._goal_deadline = time.monotonic() + pause
                    self._transition("GOAL_REACHED")
                    self.get_logger().info(
                        f"GOAL_SUCCEEDED moved={moved:.2f}m pause={pause:.1f}s before next pick"
                    )
                    return
                self.get_logger().warn(
                    "[frontier_explorer] NavigateToPose ended without success "
                    f"(status={st}); blacklisting goal and retrying"
                )
                self._fail_streak += 1
                self._handle_goal_failed("nav2_abort")
                return
            if time.monotonic() > self._goal_deadline:
                if self._goal_handle is not None:
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._goal_handle = None
                self._result_future = None
                self._fail_streak += 1
                self._handle_goal_failed("goal_timeout")
            return

        if self._st == _St.NAV_HOME:
            if self._result_future is not None and self._result_future.done():
                wrapped = self._result_future.result()
                self._goal_handle = None
                self._result_future = None
                st = int(wrapped.status) if wrapped is not None else -1
                if st == GoalStatus.STATUS_SUCCEEDED:
                    hp = self._home_pose
                    self.get_logger().info("RETURN_HOME_SUCCESS")
                    self.get_logger().info(
                        "EXPLORATION_DONE "
                        f"home=({hp[0]:.2f},{hp[1]:.2f})" if hp is not None else ""
                    )
                    self._done("DONE")
                    return
                if not self._return_home_retry and self._home_pose is not None:
                    self._return_home_retry = True
                    hx, hy, hyaw = self._home_pose
                    tol = float(self.get_parameter("return_home_retry_tolerance_m").value)
                    self.get_logger().warn(
                        f"RETURN_HOME failed status={st} - retry with tolerance {tol:.2f}m"
                    )
                    self._send_nav_goal(hx, hy, hyaw, _St.NAV_HOME)
                    return
                self.get_logger().error("RETURN_HOME_FAILED")
                self._done("RETURN_HOME_FAILED")
                return
            if time.monotonic() > self._goal_deadline:
                if self._goal_handle is not None:
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._goal_handle = None
                self._result_future = None
                self._done("EXPLORATION_COMPLETE")
            return

        if self._st == _St.PAUSE:
            self._zero()
            if time.monotonic() < self._goal_deadline:
                return
            self._pause_is_no_goal_retry = False
            self._select_entered_at = 0.0
            self._st = _St.SELECT
            return

    def _blacklist_goal(self) -> None:
        if self._last_goal is None:
            return
        self._blacklist.append((self._last_goal[0], self._last_goal[1], time.monotonic()))

    def _maybe_recover(self) -> None:
        lim = int(self.get_parameter("consecutive_fail_limit").value)
        if self._fail_streak >= lim:
            self._goal_deadline = time.monotonic() + float(
                self.get_parameter("recovery_pause_sec").value
            )
            self._st = _St.PAUSE
            self._pause_is_no_goal_retry = False
            self._stat("RECOVERY_FAIL_STREAK")
            return
        self._st = _St.SELECT

    def _begin_nav2(self) -> None:
        # Vent til Nav2 lifecycle er ACTIVE for vi sender mal.
        self._nav_wait_t0 = 0.0
        self._nav_log_t0 = 0.0
        self._lc_idx = 0
        self._lc_fut = None
        self._st = _St.WAIT_NAV2

    def _throttle_log(self, msg: str) -> None:
        now = time.monotonic()
        iv = max(5.0, float(self.get_parameter("wait_log_interval_sec").value))
        if now - self._wait_log_t0 >= iv:
            self._wait_log_t0 = now
            self.get_logger().info(msg)

    def shutdown_stop(self) -> None:
        self._done("SHUTDOWN")


def main() -> int:
    rclpy.init()
    node = None
    ex = None
    try:
        node = FrontierExplorer()
        ex = MultiThreadedExecutor(num_threads=3)
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not is_shutdown_exception(exc):
            raise
    finally:
        if ex is not None:
            try:
                ex.shutdown(timeout_sec=0.5)
            except Exception:
                pass
        if node is not None:
            try:
                node.shutdown_stop()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        safe_shutdown()
    return 0


if __name__ == "__main__":
    main()
