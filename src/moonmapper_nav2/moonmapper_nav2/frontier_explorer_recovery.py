"""Recovery (spinn/frem) og dekningssjekk for frontier_explorer."""

from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Callable, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist


class RecoveryPhase(Enum):
    SPIN = auto()
    FORWARD = auto()
    BACKUP = auto()
    BACKUP_TURN = auto()


def map_known_cell_count(
    data: Sequence[int],
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> int:
    known = 0
    for v in data:
        iv = int(v)
        if iv == unknown_value:
            continue
        if iv >= occupied_threshold or iv >= free_threshold:
            known += 1
    return known


def coverage_stable(
    known_cells: int,
    last_known: int,
    stable_since: float,
    now: float,
    delta_threshold: int,
    stable_time_sec: float,
) -> Tuple[bool, float]:
    """Returnerer (er_stabil, oppdatert_stabil_siden)."""
    if last_known < 0:
        return False, 0.0
    delta = abs(known_cells - last_known)
    if delta <= delta_threshold:
        if stable_since <= 0.0:
            return False, now
        if now - stable_since >= stable_time_sec:
            return True, stable_since
        return False, stable_since
    return False, 0.0


class RecoveryRunner:
    """Lav hastighet cmd_vel-recovery når Nav2 er idle (ingen aktivt mål)."""

    def __init__(
        self,
        get_param: Callable[[str], object],
        publish_cmd: Callable[[Twist], None],
        odom_yaw: Callable[[], Optional[float]],
    ) -> None:
        self._get = get_param
        self._pub = publish_cmd
        self._odom_yaw = odom_yaw
        self._phase = RecoveryPhase.SPIN
        self._t0 = 0.0
        self._start_yaw: Optional[float] = None
        self._accum_angle = 0.0
        self._last_yaw: Optional[float] = None
        self._start_xy: Optional[Tuple[float, float]] = None
        self._active = False

    @property
    def active(self) -> bool:
        return self._active

    def start(self, phase: RecoveryPhase = RecoveryPhase.SPIN) -> None:
        self._phase = phase
        self._t0 = time.monotonic()
        self._start_yaw = self._odom_yaw()
        self._last_yaw = self._start_yaw
        self._accum_angle = 0.0
        self._start_xy = None
        self._active = True

    def stop(self) -> None:
        self._active = False
        self._pub(Twist())

    def tick(self, pose_map: Optional[Tuple[float, float, float]]) -> bool:
        """Returnerer True når recovery-fasen er ferdig."""
        if not self._active:
            return True
        now = time.monotonic()
        lin = float(self._get("recovery_cmd_vel_linear").value)
        ang = float(self._get("recovery_cmd_vel_angular").value)

        if self._phase == RecoveryPhase.SPIN:
            y = self._odom_yaw()
            if y is None:
                return False
            if self._last_yaw is not None:
                dy = y - self._last_yaw
                while dy > math.pi:
                    dy -= 2.0 * math.pi
                while dy < -math.pi:
                    dy += 2.0 * math.pi
                self._accum_angle += abs(dy)
            self._last_yaw = y
            tw = Twist()
            tw.angular.z = ang if float(self._get("recovery_spin_angle_deg").value) >= 0 else -ang
            self._pub(tw)
            tgt = math.radians(abs(float(self._get("recovery_spin_angle_deg").value)))
            if self._accum_angle >= tgt or now - self._t0 > 12.0:
                self._phase = RecoveryPhase.FORWARD
                self._t0 = now
                if pose_map is not None:
                    self._start_xy = (pose_map[0], pose_map[1])
            return False

        if self._phase == RecoveryPhase.BACKUP:
            if pose_map is not None and self._start_xy is None:
                self._start_xy = (pose_map[0], pose_map[1])
            tw = Twist()
            tw.linear.x = -abs(lin)
            self._pub(tw)
            dist_tgt = float(self._get("recovery_backup_distance_m").value)
            moved = 0.0
            if pose_map is not None and self._start_xy is not None:
                moved = math.hypot(pose_map[0] - self._start_xy[0], pose_map[1] - self._start_xy[1])
            if moved >= dist_tgt or now - self._t0 > float(self._get("backup_timeout_sec").value):
                self.stop()
                return True
            return False

        if self._phase == RecoveryPhase.FORWARD:
            tw = Twist()
            tw.linear.x = abs(lin)
            self._pub(tw)
            dist_tgt = float(self._get("recovery_forward_distance_m").value)
            moved = 0.0
            if pose_map is not None and self._start_xy is not None:
                moved = math.hypot(pose_map[0] - self._start_xy[0], pose_map[1] - self._start_xy[1])
            if moved >= dist_tgt or now - self._t0 > 10.0:
                self.stop()
                return True
            return False

        if self._phase == RecoveryPhase.BACKUP_TURN:
            elapsed = now - self._t0
            if elapsed < 2.5:
                tw = Twist()
                tw.linear.x = -abs(lin)
                self._pub(tw)
                return False
            y = self._odom_yaw()
            if self._last_yaw is None:
                self._last_yaw = y
            if y is not None and self._last_yaw is not None:
                dy = y - self._last_yaw
                while dy > math.pi:
                    dy -= 2.0 * math.pi
                while dy < -math.pi:
                    dy += 2.0 * math.pi
                self._accum_angle += abs(dy)
                self._last_yaw = y
            tw = Twist()
            tw.angular.z = ang
            self._pub(tw)
            tgt = math.radians(float(self._get("recovery_turn_angle_deg").value))
            if self._accum_angle >= tgt or elapsed > 8.0:
                self.stop()
                return True
            return False

        self.stop()
        return True
