#!/usr/bin/env python3
"""Logger TF/odom/IMU/UWB-tilstand ved oppstart (preflight for localization stack)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class LocalizationPreflightNode(Node):
  def __init__(self) -> None:
    super().__init__('localization_preflight')
    self.declare_parameter('check_period_sec', 5.0)
    self.declare_parameter('odom_topics', [
      '/odom',
      '/diff_drive_controller/odom',
      '/odometry/filtered',
    ])
    self.declare_parameter('imu_topic', '/imu/data')
    self.declare_parameter('uwb_pose_topic', '/uwb/pose')
    # rclpy Parameter API håndterer ikke "list of lists" som default-verdi på alle ROS 2 distroer.
    # Bruk en flat liste "parent,child,parent,child,...", som vi parser selv.
    self.declare_parameter('tf_pairs_flat', [
      'map', 'odom',
      'odom', 'base_footprint',
    ])

    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self)
    period = float(self.get_parameter('check_period_sec').value)
    self.create_timer(period, self._report)

    self.get_logger().info(
      'Localization preflight (sim): sjekker topics og TF. Forvent /uwb/* + '
      '/odometry/filtered og TF odom->base_footprint nar localization_fusion er aktiv.'
    )

  def _report(self) -> None:
    lines: list[str] = ['--- localization preflight ---']

    for topic in self.get_parameter('odom_topics').value:
      n = self.count_publishers(str(topic))
      lines.append(f'  publishers on {topic}: {n}')

    imu = str(self.get_parameter('imu_topic').value)
    lines.append(f'  publishers on {imu}: {self.count_publishers(imu)}')

    uwb = str(self.get_parameter('uwb_pose_topic').value)
    lines.append(f'  publishers on {uwb}: {self.count_publishers(uwb)}')

    tf_flat = [str(x) for x in self.get_parameter('tf_pairs_flat').value]
    pairs: list[tuple[str, str]] = []
    if len(tf_flat) % 2 != 0:
      self.get_logger().warn(
        f"tf_pairs_flat har oddetall elementer (len={len(tf_flat)}). "
        "Forventet parent,child,parent,child,..."
      )
    for i in range(0, len(tf_flat) - 1, 2):
      pairs.append((tf_flat[i], tf_flat[i + 1]))

    for parent, child in pairs:
      try:
        tf = self._tf_buffer.lookup_transform(
          str(parent), str(child), rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
        )
        t = tf.transform.translation
        lines.append(
          f'  TF {parent}->{child}: OK (t=({t.x:.3f},{t.y:.3f},{t.z:.3f}))'
        )
      except Exception as exc:
        lines.append(f'  TF {parent}->{child}: MISSING ({exc})')

    msg = '\n'.join(lines)
    self.get_logger().info(msg)

    map_odom_pub = self.count_publishers('/tf') + self.count_publishers('/tf_static')
    if map_odom_pub > 0:
      try:
        self._tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
      except Exception:
        pass


def main(args: list[str] | None = None) -> None:
  rclpy.init(args=args)
  node = LocalizationPreflightNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
