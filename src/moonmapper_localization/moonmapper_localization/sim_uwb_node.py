#!/usr/bin/env python3
"""Simuler BU04 UWB i Gazebo basert på roverens sim-pose/odom.

Publiserer:
- `/uwb/pose`: base-korrigert pose i odom (egnet for EKF input)
- `/uwb/tag_pose`: debug-pose for selve taggen på roboten
- `/uwb/ranges`: `[dist0, dist1, dist2, dist3]` (til ankere)
- `/uwb/status`: `UWB_OK_GROUND_TRUTH`, `UWB_OK_ODOM_FALLBACK`, ...
"""

from __future__ import annotations

import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformListener


class SimUwbNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_uwb_node")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("ground_truth_topic", "/gz/moonmapper_gz/ground_truth")
        self.declare_parameter("ground_truth_child_frame_id", "moonmapper_gz")
        self.declare_parameter("source_odom_topic", "/diff_drive_controller/odom")
        self.declare_parameter("publish_rate_hz", 10.0)

        # Gazebo world pose at spawn → odom origin (expo default spawn SW).
        self.declare_parameter("world_origin_x", -7.5)
        self.declare_parameter("world_origin_y", -7.5)

        self.declare_parameter("tag_offset_x", -0.25)
        self.declare_parameter("tag_offset_y", 0.0)
        self.declare_parameter("tag_offset_z", 0.20)

        self.declare_parameter("noise_std_x", 0.02)
        self.declare_parameter("noise_std_y", 0.02)
        self.declare_parameter("range_noise_std", 0.04)
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("max_jump_m", 2.5)
        self.declare_parameter("position_covariance", 0.02)

        self.declare_parameter("anchors.BS0", [-9.5, -9.5])
        self.declare_parameter("anchors.BS1", [-9.5, 9.5])
        self.declare_parameter("anchors.BS2", [9.5, -9.5])
        self.declare_parameter("anchors.BS3", [9.5, 9.5])

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_odom: Optional[Odometry] = None
        self._last_base_xy: Optional[tuple[float, float]] = None
        self._last_truth_odom_xy_yaw: Optional[tuple[float, float, float]] = None
        self._warned_no_gt = False

        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/uwb/pose", 10)
        self._tag_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/uwb/tag_pose", 10)
        self._ranges_pub = self.create_publisher(Float32MultiArray, "/uwb/ranges", 10)
        self._status_pub = self.create_publisher(String, "/uwb/status", 10)

        odom_topic = str(self.get_parameter("source_odom_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        gt_topic = str(self.get_parameter("ground_truth_topic").value)
        self.create_subscription(PoseArray, gt_topic, self._on_ground_truth_pose_array, 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self.get_logger().info(
            f"sim_uwb_node: GT fra {gt_topic} (world→odom offset "
            f"{self.get_parameter('world_origin_x').value}, "
            f"{self.get_parameter('world_origin_y').value})"
        )

    def _world_to_odom_xy_yaw(self, wx: float, wy: float, yaw: float) -> tuple[float, float, float]:
        ox0 = float(self.get_parameter("world_origin_x").value)
        oy0 = float(self.get_parameter("world_origin_y").value)
        return (wx - ox0, wy - oy0, yaw)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
        return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    def _on_ground_truth_pose_array(self, msg: PoseArray) -> None:
        child_id = str(self.get_parameter("ground_truth_child_frame_id").value)
        idx = -1
        try:
            idx = list(msg.header.frame_id.split(",")).index(child_id)
        except Exception:
            pass
        if idx < 0:
            # Fallback: many setups publish PoseArray with one pose
            idx = 0
        if not msg.poses or idx >= len(msg.poses):
            return
        p = msg.poses[idx]
        yaw = self._yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        ox, oy, oyaw = self._world_to_odom_xy_yaw(p.position.x, p.position.y, yaw)
        self._last_truth_odom_xy_yaw = (ox, oy, oyaw)

    def _safe_lookup_base_xy(self) -> Optional[tuple[float, float]]:
        base = str(self.get_parameter("base_frame_id").value)
        frame = str(self.get_parameter("frame_id").value)
        try:
            tf = self._tf_buffer.lookup_transform(frame, base, rclpy.time.Time())
            t = tf.transform.translation
            return (float(t.x), float(t.y))
        except Exception:
            return None

    def _tick(self) -> None:
        dropout = float(self.get_parameter("dropout_probability").value)
        if dropout > 0.0 and random.random() < dropout:
            return

        truth = self._last_truth_odom_xy_yaw
        status = "UWB_OK_GROUND_TRUTH"

        if truth is None:
            if self._last_odom is None:
                if not self._warned_no_gt:
                    self.get_logger().warn("sim_uwb_node: mangler ground_truth og odom; venter…")
                    self._warned_no_gt = True
                return
            # Fallback: use odom pose
            o = self._last_odom.pose.pose
            yaw = self._yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
            truth = (float(o.position.x), float(o.position.y), float(yaw))
            status = "UWB_OK_ODOM_FALLBACK"

        bx_by = self._safe_lookup_base_xy()
        if bx_by is not None:
            self._last_base_xy = bx_by

        x, y, yaw = truth

        noise_x = float(self.get_parameter("noise_std_x").value)
        noise_y = float(self.get_parameter("noise_std_y").value)
        x += random.gauss(0.0, noise_x)
        y += random.gauss(0.0, noise_y)

        # Clamp large jumps to avoid EKF blow-ups when GT glitches.
        max_jump = float(self.get_parameter("max_jump_m").value)
        if self._last_base_xy is not None and max_jump > 0.0:
            lx, ly = self._last_base_xy
            dx, dy = x - lx, y - ly
            d = math.hypot(dx, dy)
            if d > max_jump:
                scale = max_jump / max(d, 1e-6)
                x = lx + dx * scale
                y = ly + dy * scale

        qx, qy, qz, qw = self._quat_from_yaw(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        cov = float(self.get_parameter("position_covariance").value)
        msg.pose.covariance[0] = cov
        msg.pose.covariance[7] = cov
        msg.pose.covariance[35] = cov
        self._pose_pub.publish(msg)

        # Debug tag pose (apply tag offset in base frame if TF is available).
        tag = PoseWithCovarianceStamped()
        tag.header = msg.header
        tag.pose = msg.pose
        self._tag_pose_pub.publish(tag)

        # Ranges (simple euclidean to anchors).
        ranges = Float32MultiArray()
        anchors = []
        for k in ("BS0", "BS1", "BS2", "BS3"):
            anchors.append(self.get_parameter(f"anchors.{k}").value)
        rs = []
        rn = float(self.get_parameter("range_noise_std").value)
        for ax, ay in anchors:
            rs.append(float(math.hypot(x - float(ax), y - float(ay)) + random.gauss(0.0, rn)))
        ranges.data = rs
        self._ranges_pub.publish(ranges)

        s = String()
        s.data = status
        self._status_pub.publish(s)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimUwbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

