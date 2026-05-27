"""Filtrerer cmd_vel ut fra laser: sakte ned, stopp eller backup ved hindring."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration

from moonmapper_autonomy.rclpy_shutdown import is_shutdown_exception, safe_shutdown
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class SafetyObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_obstacle_node")

        self.declare_parameter("front_stop_distance", 0.22)
        self.declare_parameter("emergency_stop_distance_m", 0.14)
        self.declare_parameter("slowdown_distance_m", 0.50)
        self.declare_parameter("range_calibration_offset_m", 0.12)
        self.declare_parameter("min_trusted_front_range_m", 0.16)
        self.declare_parameter("slow_linear_speed", 0.20)
        self.declare_parameter("safe_turn_speed", 0.25)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("side_angle_deg", 70.0)
        self.declare_parameter("min_turn_clearance_m", 0.45)
        self.declare_parameter("corridor_creep_enabled", True)
        self.declare_parameter("corridor_min_side_clearance_m", 0.26)
        self.declare_parameter("corridor_min_total_width_m", 0.48)
        self.declare_parameter("corridor_creep_speed_mps", 0.14)
        self.declare_parameter("corridor_creep_scale", 0.45)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("allow_reverse_when_blocked", False)
        self.declare_parameter("reverse_speed_when_blocked", 0.05)
        self.declare_parameter("safety_controls_backup", False)
        self.declare_parameter("enable_safety_gating", True)
        self.declare_parameter("publish_safety_debug", True)
        self.declare_parameter("debug_log_period_sec", 1.0)

        self._front_stop = float(self.get_parameter("front_stop_distance").value)
        self._emergency_m = float(self.get_parameter("emergency_stop_distance_m").value)
        self._slow_m = float(self.get_parameter("slowdown_distance_m").value)
        if self._slow_m <= self._front_stop:
            self._slow_m = self._front_stop + 0.25
        self._slow_lin_cap = float(self.get_parameter("slow_linear_speed").value)
        self._safe_turn = float(self.get_parameter("safe_turn_speed").value)
        self._front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self._side_angle_deg = float(self.get_parameter("side_angle_deg").value)
        self._min_turn_clr = float(self.get_parameter("min_turn_clearance_m").value)
        self._corridor_creep = bool(self.get_parameter("corridor_creep_enabled").value)
        self._corridor_side = float(self.get_parameter("corridor_min_side_clearance_m").value)
        self._corridor_width = float(self.get_parameter("corridor_min_total_width_m").value)
        self._corridor_speed = abs(
            float(self.get_parameter("corridor_creep_speed_mps").value)
        )
        self._corridor_scale = max(
            0.1, min(1.0, float(self.get_parameter("corridor_creep_scale").value))
        )
        self._input_topic = str(self.get_parameter("input_cmd_topic").value)
        self._output_topic = str(self.get_parameter("output_cmd_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self._allow_rev = bool(self.get_parameter("allow_reverse_when_blocked").value)
        self._safety_backup = bool(self.get_parameter("safety_controls_backup").value)
        self._range_offset = float(self.get_parameter("range_calibration_offset_m").value)
        self._min_trusted = float(self.get_parameter("min_trusted_front_range_m").value)
        self._gating = bool(self.get_parameter("enable_safety_gating").value)
        self._reverse_speed = abs(float(self.get_parameter("reverse_speed_when_blocked").value))
        self._pub_debug = bool(self.get_parameter("publish_safety_debug").value)
        self._debug_period = float(self.get_parameter("debug_log_period_sec").value)

        self._last_cmd: Optional[Twist] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_debug_t = 0.0

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._pub_blocked = self.create_publisher(Bool, "/safety/blocked_front", 10)
        self._pub_front_min = self.create_publisher(Float32, "/safety/front_min", 10)
        self._pub_obstacle_front = self.create_publisher(Float32, "/obstacle/front_min", 10)
        self._pub_obstacle_state = self.create_publisher(String, "/obstacle/current_state", 10)
        self.create_subscription(Twist, self._input_topic, self._on_cmd, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self.create_timer(0.05, self._publish_safe)

        self.get_logger().info(
            f"safety_obstacle_node: {self._input_topic} + {self._scan_topic} til "
            f"{self._output_topic} stop<{self._front_stop:.2f}m "
            f"slow<{self._slow_m:.2f}m emergency<{self._emergency_m:.2f}m"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._publish_safe()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._publish_safe()

    def _scan_fresh(self) -> bool:
        if self._last_scan is None:
            return False
        stamp = Time.from_msg(self._last_scan.header.stamp)
        age = self.get_clock().now() - stamp
        return age <= Duration(seconds=self._scan_timeout)

    @staticmethod
    def _beam_angle(scan: LaserScan, index: int) -> float:
        a = scan.angle_min + float(index) * scan.angle_increment
        return math.atan2(math.sin(a), math.cos(a))

    def _valid_range(self, scan: LaserScan, r: float) -> bool:
        return not (math.isnan(r) or math.isinf(r)) and scan.range_min < r < scan.range_max

    def _sector_min(
        self, scan: LaserScan, angle_min_rad: float, angle_max_rad: float
    ) -> Optional[float]:
        best: Optional[float] = None
        n = len(scan.ranges)
        for i in range(n):
            a = self._beam_angle(scan, i)
            if a < angle_min_rad or a > angle_max_rad:
                continue
            r = float(scan.ranges[i])
            if not self._valid_range(scan, r):
                continue
            best = r if best is None else min(best, r)
        return best

    def _cone_min(self, scan: LaserScan, half_angle_deg: float) -> Optional[float]:
        half = math.radians(half_angle_deg)
        return self._sector_min(scan, -half, half)

    def _scan_sectors(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        front = self._cone_min(scan, self._front_angle_deg)
        side = math.radians(self._side_angle_deg)
        left = self._sector_min(scan, 0.05, side)
        right = self._sector_min(scan, -side, -0.05)
        return front, left, right

    def _copy_raw_twist(self, raw: Twist) -> Twist:
        out = Twist()
        out.linear.x = float(raw.linear.x)
        out.linear.y = float(raw.linear.y)
        out.linear.z = float(raw.linear.z)
        out.angular.x = float(raw.angular.x)
        out.angular.y = float(raw.angular.y)
        out.angular.z = float(raw.angular.z)
        return out

    def _forward_scale(self, min_front: float) -> float:
        if min_front >= self._slow_m:
            return 1.0
        if min_front <= self._front_stop:
            return 0.0
        denom = self._slow_m - self._front_stop
        if denom < 1e-6:
            return 0.0
        return max(0.0, min(1.0, (min_front - self._front_stop) / denom))

    def _turn_allowed(self, left: Optional[float], right: Optional[float]) -> bool:
        if left is None or right is None:
            return True
        return left >= self._min_turn_clr and right >= self._min_turn_clr

    def _corridor_opening(
        self, left: Optional[float], right: Optional[float]
    ) -> Tuple[bool, float]:
        """True when side sectors show a gap wide enough for the rover (~0.40 m)."""
        if left is None or right is None:
            return False, 0.0
        width = float(left) + float(right)
        ok = (
            left >= self._corridor_side
            and right >= self._corridor_side
            and width >= self._corridor_width
        )
        return ok, width

    def _maybe_log_debug(
        self,
        raw: Twist,
        safe: Twist,
        front: Optional[float],
        left: Optional[float],
        right: Optional[float],
        state: str,
        reason: str,
    ) -> None:
        if self._debug_period <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_debug_t < self._debug_period:
            return
        self._last_debug_t = now
        fid = self._last_scan.header.frame_id if self._last_scan else "?"
        ff = float("nan") if front is None else front
        self.get_logger().info(
            "DISTANCE_DEBUG "
            f"scan_frame={fid} front_min={ff:.3f} front_left_min={left} front_right_min={right} "
            f"used_stop_distance={self._front_stop:.2f} slowdown_distance={self._slow_m:.2f} "
            f"nav_cmd_linear={raw.linear.x:.3f} safe_cmd_linear={safe.linear.x:.3f} "
            f"reason={reason}"
        )
        self.get_logger().info(
            "SAFETY_DECISION "
            f"state={state} front_min={ff:.3f} left_min={left} right_min={right} "
            f"cmd_in=({raw.linear.x:.3f},{raw.angular.z:.3f}) "
            f"cmd_out=({safe.linear.x:.3f},{safe.angular.z:.3f})"
        )

    def _publish_safe(self) -> None:
        if self._last_cmd is None:
            return

        def pub_topics(blocked: bool, front: Optional[float], state: str) -> None:
            fm = Float32()
            if front is None or math.isnan(front) or math.isinf(front):
                fm.data = float("nan")
            else:
                fm.data = float(front)
            ost = String()
            ost.data = state if self._scan_fresh() else "no_scan"
            self._pub_obstacle_front.publish(fm)
            self._pub_obstacle_state.publish(ost)
            if self._pub_debug:
                b = Bool()
                b.data = blocked
                self._pub_blocked.publish(b)
                self._pub_front_min.publish(fm)

        raw = self._last_cmd
        if not self._gating:
            passthrough = self._copy_raw_twist(raw)
            pub_topics(False, None, "gating_disabled")
            self._pub.publish(passthrough)
            return

        if self._last_scan is None or not self._scan_fresh():
            passthrough = self._copy_raw_twist(raw)
            pub_topics(False, None, "no_scan")
            self._pub.publish(passthrough)
            return

        front, left, right = self._scan_sectors(self._last_scan)
        if front is not None:
            front = min(
                float(self._last_scan.range_max),
                front + self._range_offset,
            )
            if front < self._min_trusted:
                front = None
        safe = self._copy_raw_twist(raw)
        state = "CLEAR"
        reason = "pass_through"
        lx = float(raw.linear.x)
        wz = float(raw.angular.z)

        if front is None:
            pub_topics(False, None, "no_returns")
            self._pub.publish(safe)
            return

        scale = self._forward_scale(front)
        turn_ok = self._turn_allowed(left, right)
        corridor_ok, passage_w = self._corridor_opening(left, right)

        # Nav2 publiserer til cmd_vel_raw; vi sender filtrert hastighet videre.
        if front > self._slow_m:
            state = "CLEAR"
            reason = "beyond_slowdown"
        elif front > self._front_stop:
            state = "SLOWDOWN"
            reason = "gradual_scale"
            if lx > 0.0:
                safe.linear.x = lx * scale
        elif front > self._emergency_m:
            if (
                self._corridor_creep
                and corridor_ok
                and lx > 0.0
            ):
                state = "CORRIDOR_CREEP"
                reason = f"gap_width={passage_w:.2f}m"
                creep = min(
                    self._corridor_speed,
                    abs(lx) * self._corridor_scale * scale,
                )
                safe.linear.x = creep
                if abs(wz) > 0.02:
                    safe.angular.z = math.copysign(
                        min(abs(wz), self._safe_turn), wz
                    )
            else:
                state = "STOP"
                reason = "stop_forward"
                if lx > 0.0:
                    safe.linear.x = 0.0
                if abs(wz) > 0.02 and turn_ok:
                    state = "ALLOW_TURN"
                    reason = "stop_allow_turn"
                    safe.angular.z = math.copysign(min(abs(wz), self._safe_turn), wz)
        else:
            state = "EMERGENCY_STOP"
            reason = "emergency_stop_only"
            if lx > 0.0:
                safe.linear.x = 0.0
            if self._safety_backup and self._allow_rev and lx > 0.0:
                state = "BACKUP_REQUIRED"
                reason = "emergency_backup"
                safe.linear.x = -self._reverse_speed
            elif abs(wz) > 0.02 and turn_ok:
                safe.angular.z = math.copysign(min(abs(wz), self._safe_turn), wz)

        blocked = state in ("STOP", "EMERGENCY_STOP", "BACKUP_REQUIRED")
        pub_topics(blocked, front, state.lower())
        self._pub.publish(safe)
        self._maybe_log_debug(raw, safe, front, left, right, state, reason)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafetyObstacleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not is_shutdown_exception(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        safe_shutdown()


if __name__ == "__main__":
    main()
