"""Operative RViz-markører for frontier explorer (/frontier_explorer/markers)."""

from __future__ import annotations

import math
from typing import List, Optional

from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from moonmapper_nav2.frontier_grid import ValidatedGoal


def _yaw_to_q(yaw: float) -> Quaternion:
    h = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


def publish_frontier_markers(
    node: Node,
    pub_mk,
    clusters,
    best: Optional[ValidatedGoal],
    map_msg: OccupancyGrid,
) -> None:
    if not bool(node.get_parameter("publish_markers").value):
        return
    info = map_msg.info
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    res = float(info.resolution)
    frame = str(node.get_parameter("map_frame").value)
    arr = MarkerArray()
    now = node.get_clock().now().to_msg()
    c = Marker()
    c.header.frame_id = frame
    c.header.stamp = now
    c.ns = "clear"
    c.action = Marker.DELETEALL
    arr.markers.append(c)
    pts = Marker()
    pts.header.frame_id = frame
    pts.header.stamp = now
    pts.ns = "frontier"
    pts.id = 1
    pts.type = Marker.POINTS
    pts.action = Marker.ADD
    pts.scale.x = res * 0.4
    pts.scale.y = res * 0.4
    pts.color = ColorRGBA(r=0.7, g=0.7, b=0.2, a=0.85)
    for cl in clusters:
        for cx, cy in cl.cells:
            p = Point()
            p.x = ox + (cx + 0.5) * res
            p.y = oy + (cy + 0.5) * res
            p.z = 0.04
            pts.points.append(p)
    arr.markers.append(pts)
    if best is not None:
        a = Marker()
        a.header.frame_id = frame
        a.header.stamp = now
        a.ns = "goal"
        a.id = 2
        a.type = Marker.ARROW
        a.action = Marker.ADD
        a.scale.x = 0.35
        a.scale.y = 0.06
        a.scale.z = 0.06
        a.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9)
        a.pose.position.x = best.wx
        a.pose.position.y = best.wy
        a.pose.position.z = 0.05
        a.pose.orientation = _yaw_to_q(best.yaw)
        arr.markers.append(a)
    pub_mk.publish(arr)
