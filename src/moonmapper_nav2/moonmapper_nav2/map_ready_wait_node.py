#!/usr/bin/env python3
"""Venter pa gyldig /map og sjekker TF for RTAB-Map + Nav2."""

from __future__ import annotations

import sys

import rclpy
import tf2_ros
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown


def _declare_parameter_if_not_declared(node: Node, name: str, default_value):
    if node.has_parameter(name):
        return node.get_parameter(name).value
    try:
        node.declare_parameter(name, default_value)
    except ParameterAlreadyDeclaredException:
        pass
    return node.get_parameter(name).value


class MapReadyWaitNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_map_ready_wait")
        _d = _declare_parameter_if_not_declared
        _d(self, "map_topic", "/map")
        _d(self, "base_frame", "base_footprint")
        _d(self, "check_period_sec", 2.0)
        _d(self, "publish_ready", True)
        _d(self, "navigate_action", "/navigate_to_pose")
        _d(self, "check_nav2_action", True)

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._nav_action = str(self.get_parameter("navigate_action").value)
        self._check_nav2_action = bool(self.get_parameter("check_nav2_action").value)
        self._base = str(self.get_parameter("base_frame").value)
        period = max(1.0, float(self.get_parameter("check_period_sec").value))
        self._pub_ready = bool(self.get_parameter("publish_ready").value)

        self._ready = False
        self._last_map: OccupancyGrid | None = None
        self._buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buf, self, spin_thread=False)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, qos)
        self._nav: ActionClient | None = None
        self._nav_ready = False
        if self._check_nav2_action:
            self._nav = ActionClient(self, NavigateToPose, self._nav_action)
        if self._pub_ready:
            self._ready_pub = self.create_publisher(Bool, "/nav2/map_ready", 10)
        self._timer = self.create_timer(period, self._check)
        self.get_logger().info(f"map_ready_wait: lytter pa {self._map_topic}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map = msg
        w, h = int(msg.info.width), int(msg.info.height)
        if w > 0 and h > 0 and not self._ready:
            self._ready = True
            out = Bool()
            out.data = True
            if self._pub_ready:
                self._ready_pub.publish(out)
            self.get_logger().info(
                f"/map KLAR opplosning={msg.info.resolution:.3f} storrelse={w}x{h}"
            )

    def _lookup(self, parent: str, child: str) -> str:
        try:
            t = self._buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
            tr = t.transform.translation
            return f"OK xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})"
        except tf2_ros.TransformException as ex:
            return f"MISSING ({ex})"

    def _check(self) -> None:
        if not rclpy.ok():
            return
        map_s = "ingen data"
        if self._last_map is not None:
            w, h = int(self._last_map.info.width), int(self._last_map.info.height)
            map_s = f"{w}x{h} klar={self._ready}"
        nav_s = "n/a"
        if self._check_nav2_action and self._nav is not None:
            if not self._nav_ready:
                self._nav_ready = self._nav.server_is_ready() or self._nav.wait_for_server(
                    timeout_sec=0.5
                )
            nav_s = "klar" if self._nav_ready else "venter"
        self.get_logger().info(
            f"map={map_s} | odom til {self._base}: {self._lookup('odom', self._base)} | "
            f"map til odom: {self._lookup('map', 'odom')} | "
            f"map til {self._base}: {self._lookup('map', self._base)} | "
            f"nav2_action({self._nav_action})={nav_s}"
        )


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = MapReadyWaitNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception as exc:
        if not is_shutdown_exception(exc):
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        safe_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
