"""Depth-bilde til LaserScan for Nav2/local costmap."""

from __future__ import annotations

import math
import struct
import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from moonmapper_autonomy.rclpy_shutdown import is_shutdown_exception, safe_shutdown
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class DepthToScanNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_scan_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("depth_image_topic", "/depth_camera/depth_image")
        self.declare_parameter("camera_info_topic", "/depth_camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_height", 10)
        self.declare_parameter("scan_height_mode", "roi_percentile")
        self.declare_parameter("roi_top_ratio", 0.35)
        self.declare_parameter("roi_bottom_ratio", 0.65)
        self.declare_parameter("center_crop_ratio", 0.90)
        self.declare_parameter("min_valid_points_per_column", 2)
        self.declare_parameter("ground_filter_enabled", True)
        self.declare_parameter("ground_filter_bottom_roi_ratio", 0.12)
        self.declare_parameter("roi_percentile", 0.10)
        self.declare_parameter("front_percentile", 0.10)
        self.declare_parameter("depth_min_valid_m", 0.20)
        self.declare_parameter("depth_max_valid_m", 4.0)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 3.0)
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("output_frame_id", "depth_camera_optical_frame")
        self.declare_parameter("debug_log_period_sec", 1.0)

        self._depth_topic = str(self.get_parameter("depth_image_topic").value)
        self._info_topic = str(self.get_parameter("camera_info_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._stripe = int(self.get_parameter("scan_height").value)
        if self._stripe < 1:
            self._stripe = 1
        self._mode = str(self.get_parameter("scan_height_mode").value).strip().lower()
        self._roi_top = float(self.get_parameter("roi_top_ratio").value)
        self._roi_bot = float(self.get_parameter("roi_bottom_ratio").value)
        self._crop = max(0.1, min(1.0, float(self.get_parameter("center_crop_ratio").value)))
        self._min_pts = max(1, int(self.get_parameter("min_valid_points_per_column").value))
        self._ground_en = bool(self.get_parameter("ground_filter_enabled").value)
        self._ground_skip = max(0.0, min(0.5, float(self.get_parameter("ground_filter_bottom_roi_ratio").value)))
        self._pct = max(0.0, min(1.0, float(self.get_parameter("roi_percentile").value)))
        fp = float(self.get_parameter("front_percentile").value)
        self._front_pct = max(0.0, min(1.0, fp if fp > 0.0 else self._pct))
        self._z_min = float(self.get_parameter("depth_min_valid_m").value)
        self._z_max = float(self.get_parameter("depth_max_valid_m").value)
        self._rmin = float(self.get_parameter("range_min").value)
        self._rmax = float(self.get_parameter("range_max").value)
        self._scan_time = float(self.get_parameter("scan_time").value)
        self._frame_override = str(self.get_parameter("output_frame_id").value).strip()
        self._debug_period = float(self.get_parameter("debug_log_period_sec").value)
        self._last_debug_t = 0.0

        self._ci: Optional[CameraInfo] = None
        self._bad_enc_logged = False
        self._logged_ci_frame = False
        self._pub = self.create_publisher(LaserScan, self._scan_topic, 10)
        self.create_subscription(Image, self._depth_topic, self._on_depth, 10)
        self.create_subscription(CameraInfo, self._info_topic, self._on_info, 10)

        self.get_logger().info(
            f"depth_to_scan_node: mode={self._mode} roi_v={self._roi_top:.2f}-{self._roi_bot:.2f} "
            f"crop={self._crop:.2f} z_valid=[{self._z_min:.2f},{self._z_max:.2f}] "
            f"pct={self._front_pct:.2f}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self._ci = msg
        if not self._logged_ci_frame:
            self._logged_ci_frame = True
            self.get_logger().info(
                f"depth_to_scan_node: camera_info frame_id={msg.header.frame_id}"
            )

    def _read_z(self, msg: Image, u: int, v: int) -> Optional[float]:
        step = msg.step
        enc = msg.encoding
        row = v * step
        if enc in ("32FC1", "PASS_THROUGH", "TYPE_32FC1"):
            off = row + u * 4
            if off + 4 > len(msg.data):
                return None
            z = struct.unpack_from("<f", msg.data, off)[0]
            return float(z)
        if enc == "16UC1":
            off = row + u * 2
            if off + 2 > len(msg.data):
                return None
            raw = struct.unpack_from("<H", msg.data, off)[0]
            return raw * 0.001
        if not self._bad_enc_logged:
            self.get_logger().warning(
                f"depth_to_scan_node: ukjent depth-encoding '{enc}', forventer 32FC1 eller 16UC1."
            )
            self._bad_enc_logged = True
        return None

    def _roi_rows(self, height: int) -> tuple[int, int]:
        # Vertikal ROI: unngår himmel og mye bakke i nederste stripe.
        if self._mode == "center":
            vc = height // 2
            half = self._stripe // 2
            v0 = max(0, vc - half)
            v1 = min(height, v0 + self._stripe)
            return v0, v1
        top = max(0.0, min(1.0, self._roi_top))
        bot = max(0.0, min(1.0, self._roi_bot))
        if bot <= top:
            bot = min(1.0, top + 0.05)
        v0 = int(height * top)
        v1 = int(math.ceil(height * bot))
        v1 = min(height, max(v0 + 1, v1))
        if self._ground_en and v1 > v0 + 2:
            skip = int((v1 - v0) * self._ground_skip)
            v1 = max(v0 + 1, v1 - skip)
        return v0, v1

    def _column_range(self, zs: List[float]) -> Optional[float]:
        # Percentile/min per kolonne gir stabil front-avstand uten enkeltpunkts-stoy.
        if len(zs) < self._min_pts:
            return None
        zs = [z for z in zs if self._z_min <= z <= self._z_max]
        if len(zs) < self._min_pts:
            return None
        zs.sort()
        use_pct = self._mode in ("roi_percentile", "percentile", "roi_min")
        if use_pct:
            idx = int(round(self._front_pct * float(len(zs) - 1)))
            idx = max(0, min(len(zs) - 1, idx))
            return zs[idx]
        return zs[0]

    def _on_depth(self, msg: Image) -> None:
        if self._ci is None:
            return
        if self._ci.width != msg.width or self._ci.height != msg.height:
            return
        if len(self._ci.k) < 9:
            return

        fx = float(self._ci.k[0])
        cx = float(self._ci.k[2])
        if fx <= 1e-6:
            return

        width = int(msg.width)
        height = int(msg.height)
        v0, v1 = self._roi_rows(height)

        margin = (1.0 - self._crop) / 2.0
        u_lo = int(width * margin)
        u_hi = int(math.ceil(width * (1.0 - margin)))
        u_hi = max(u_lo + 1, min(width, u_hi))

        cols = list(range(u_lo, u_hi))
        ncols = len(cols)
        if ncols < 1:
            return

        angle_min = math.atan2((float(cols[0]) - cx) / fx, 1.0)
        angle_max = math.atan2((float(cols[-1]) - cx) / fx, 1.0)
        inc = (angle_max - angle_min) / float(ncols - 1) if ncols > 1 else 0.0

        ranges_out: List[float] = []
        all_raw: List[float] = []
        center_zs: List[float] = []
        cu = (u_lo + u_hi) // 2

        for u in cols:
            zs: List[float] = []
            for v in range(v0, v1):
                z = self._read_z(msg, u, v)
                if z is None or math.isnan(z) or math.isinf(z):
                    continue
                if z <= 0.0:
                    continue
                zs.append(z)
                if u == cu:
                    all_raw.append(z)

            z_col = self._column_range(zs)
            if u == cu:
                center_zs = list(zs)
            if z_col is None:
                ranges_out.append(float("inf"))
            else:
                rng = z_col * math.sqrt(1.0 + ((float(u) - cx) / fx) ** 2)
                if rng < self._rmin or rng > self._rmax:
                    ranges_out.append(float("inf"))
                else:
                    ranges_out.append(float(rng))

        out = LaserScan()
        out.header = msg.header
        fid = self._frame_override or self._ci.header.frame_id or msg.header.frame_id
        out.header.frame_id = fid
        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = inc
        out.time_increment = 0.0
        out.scan_time = self._scan_time
        out.range_min = self._rmin
        out.range_max = self._rmax
        out.ranges = ranges_out
        self._pub.publish(out)

        now = time.monotonic()
        if self._debug_period > 0.0 and now - self._last_debug_t >= self._debug_period:
            self._last_debug_t = now
            finite = [r for r in ranges_out if not math.isnan(r) and not math.isinf(r)]
            front = min(finite) if finite else float("nan")
            raw_min = min(all_raw) if all_raw else float("nan")
            p10 = float("nan")
            med = float("nan")
            if center_zs:
                s = sorted(center_zs)
                p10 = s[max(0, int(round(0.10 * (len(s) - 1))))]
                med = s[len(s) // 2]
            self.get_logger().info(
                "DEPTH_TO_SCAN_DEBUG "
                f"valid_points={len(center_zs)} raw_min={raw_min:.3f} p10={p10:.3f} "
                f"median={med:.3f} published_front_range={front:.3f} frame={fid}"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DepthToScanNode()
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
