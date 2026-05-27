"""
Relay for cmd_vel + odom (rover-konvensjon i sim).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


class CmdVelOdomRelay(Node):
    """Liten Twist/Odometry-relay som binder rover-topics til Gazebo-controller."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_odom_relay")

        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("use_smoothed_twist_stamped_input", False)
        self.declare_parameter("twist_stamped_topic", "/cmd_vel_smoothed")
        self.declare_parameter("publish_twist_cmd_vel_mirror", True)
        self.declare_parameter("publish_odom_relay", True)
        self.declare_parameter("cmd_linear_x_sign", 1.0)
        self.declare_parameter("cmd_angular_z_sign", 1.0)
        self.declare_parameter("invert_cmd_vel_twist", False)

        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self._use_smoothed = (
            self.get_parameter("use_smoothed_twist_stamped_input")
            .get_parameter_value()
            .bool_value
        )
        self._stamped_topic = (
            self.get_parameter("twist_stamped_topic").get_parameter_value().string_value
        )
        self._mirror_twist = (
            self.get_parameter("publish_twist_cmd_vel_mirror")
            .get_parameter_value()
            .bool_value
        )
        self._publish_odom_relay = (
            self.get_parameter("publish_odom_relay")
            .get_parameter_value()
            .bool_value
        )
        self._invert_legacy = (
            self.get_parameter("invert_cmd_vel_twist")
            .get_parameter_value()
            .bool_value
        )
        self._lin_sign = float(self.get_parameter("cmd_linear_x_sign").value)
        self._ang_sign = float(self.get_parameter("cmd_angular_z_sign").value)
        if self._invert_legacy:
            self.get_logger().warn(
                "invert_cmd_vel_twist=true er utdatert: bruk invert_cmd_vel_twist:=false og "
                "cmd_linear_x_sign / cmd_angular_z_sign. Legacy-modus tilsvarer "
                "cmd_linear_x_sign=-1.0, cmd_angular_z_sign=-1.0 (overskriver sign-parametre)."
            )
            self._lin_sign = -1.0
            self._ang_sign = -1.0

        self._pub_cmd = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10,
        )
        self._pub_cmd_vel_mirror = None
        if self._use_smoothed and self._mirror_twist:
            self._pub_cmd_vel_mirror = self.create_publisher(Twist, "/cmd_vel", 10)

        if self._use_smoothed:
            self.create_subscription(
                TwistStamped,
                self._stamped_topic,
                self._on_cmd_vel_smoothed,
                10,
            )
            self.get_logger().info(
                "cmd_vel_odom_relay klar: %s (TwistStamped) til "
                "/diff_drive_controller/cmd_vel (TwistStamped, frame_id=%s)%s%s."
                % (
                    self._stamped_topic,
                    self._frame_id,
                    "; speiler Twist til /cmd_vel" if self._pub_cmd_vel_mirror else "",
                    f" cmd_linear_x_sign={self._lin_sign} cmd_angular_z_sign={self._ang_sign}",
                ),
            )
        else:
            self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
            self.get_logger().info(
                f"cmd_vel_odom_relay klar: /cmd_vel (Twist) til "
                f"/diff_drive_controller/cmd_vel (TwistStamped, frame_id={self._frame_id}). "
                f"cmd_linear_x_sign={self._lin_sign} cmd_angular_z_sign={self._ang_sign}"
            )

        if self._publish_odom_relay:
            self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.create_subscription(
                Odometry, "/diff_drive_controller/odom", self._on_odom, 10,
            )
        else:
            self._pub_odom = None
            self.get_logger().info(
                "cmd_vel_odom_relay: publish_odom_relay=false, ingen /odom-speil "
                "(bruk f.eks. /odometry/filtered fra EKF).",
            )

    def _scale_twist_to_controller(self, twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = self._lin_sign * float(twist.linear.x)
        out.linear.y = float(twist.linear.y)
        out.linear.z = float(twist.linear.z)
        out.angular.x = float(twist.angular.x)
        out.angular.y = float(twist.angular.y)
        out.angular.z = self._ang_sign * float(twist.angular.z)
        return out

    def _on_cmd_vel(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = self._scale_twist_to_controller(msg)
        self._pub_cmd.publish(stamped)

    def _on_cmd_vel_smoothed(self, msg: TwistStamped) -> None:
        out = TwistStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame_id
        t = self._scale_twist_to_controller(msg.twist)
        out.twist = t
        self._pub_cmd.publish(out)
        if self._pub_cmd_vel_mirror is not None:
            self._pub_cmd_vel_mirror.publish(t)

    def _on_odom(self, msg: Odometry) -> None:
        if self._pub_odom is not None:
            self._pub_odom.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
