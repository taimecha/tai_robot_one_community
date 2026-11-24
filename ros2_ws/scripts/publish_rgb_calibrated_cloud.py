#!/usr/bin/env python3
"""Project registered Astra depth with rays from calibrated raw RGB pixels.

This assumes depth_registration maps depth pixels to the raw RGB image. It does
not correct an absolute Depth range bias or a remaining RGB/Depth pixel offset.
The floor-reference mode shares pallet recognition's per-capture correction;
the configurable static slope remains available for legacy preview tools.
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pallet_plane_heading import floor_depth_skew
from pallet_target_geometry import corrected_depth_m


def project_corrected_depth(depth_mm, rays, u_coords, slope, reference_u):
    """Use exactly the optical-Z correction used for pallet-leg geometry."""
    z = corrected_depth_m(depth_mm, u_coords, slope, reference_u).astype(np.float32)
    xyz = np.empty(z.shape + (3,), dtype=np.float32)
    xyz[:, :, 0] = rays[:, :, 0] * z
    xyz[:, :, 1] = rays[:, :, 1] * z
    xyz[:, :, 2] = z
    raw = np.asarray(depth_mm)
    xyz[~np.isfinite(raw) | (raw <= 0) | (raw > 10000)
        | ~np.isfinite(z) | (z <= 0) | (z > 10)] = np.nan
    return xyz


def validate_capture_stamp(stamp_s, now_s, max_age_s):
    if not np.isfinite([stamp_s, now_s]).all() or not -.05 <= now_s - stamp_s <= max_age_s:
        raise ValueError("registered depth capture is missing/stale or future-dated")


class CalibratedCloud(Node):
    def __init__(self):
        super().__init__("rgb_calibrated_cloud")
        self.declare_parameter("pixel_stride", 2)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("inverse_depth_x_slope", 0.0)
        self.declare_parameter("reference_u", 370.0)
        self.declare_parameter("output_topic", "/camera/depth/points_rgb_calibrated")
        self.declare_parameter("floor_reference", False)
        self.declare_parameter("fallback_to_static_slope", False)
        self.declare_parameter("max_input_age_s", .5)
        self.declare_parameter("correction_status_topic", "/camera/depth/correction_status")
        self.stride = int(self.get_parameter("pixel_stride").value)
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self.inverse_depth_x_slope = float(self.get_parameter("inverse_depth_x_slope").value)
        self.reference_u = float(self.get_parameter("reference_u").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.floor_reference = bool(self.get_parameter("floor_reference").value)
        self.fallback_to_static_slope = bool(self.get_parameter("fallback_to_static_slope").value)
        self.last_valid_slope = self.inverse_depth_x_slope
        self.max_input_age_s = float(self.get_parameter("max_input_age_s").value)
        if self.stride < 1 or self.max_rate_hz <= 0 or not np.isfinite(self.max_input_age_s) or self.max_input_age_s <= 0:
            raise ValueError("pixel_stride and max_rate_hz must be positive")
        if not np.isfinite(self.inverse_depth_x_slope) or not np.isfinite(self.reference_u):
            raise ValueError("inverse_depth_x_slope and reference_u must be finite")

        self.rays = None
        self.u_coords = None
        self.info_key = None
        self.frame_id = None
        self.last_publish = 0.0
        self.next_publish = 0.0
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String,
            str(self.get_parameter("correction_status_topic").value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.intrinsics = None
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data
        )

    def on_info(self, msg):
        key = (msg.width, msg.height, tuple(msg.k), tuple(msg.d), msg.header.frame_id)
        if key == self.info_key:
            return
        if msg.width == 0 or msg.height == 0 or msg.k[0] <= 0 or msg.k[4] <= 0:
            self.get_logger().warn("Waiting for calibrated RGB CameraInfo")
            return

        k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(msg.d, dtype=np.float64)
        u, v = np.meshgrid(
            np.arange(0, msg.width, self.stride),
            np.arange(0, msg.height, self.stride),
        )
        pixels = np.stack((u, v), axis=-1).astype(np.float32).reshape(-1, 1, 2)
        self.rays = cv2.undistortPoints(pixels, k, d).reshape(u.shape + (2,)).astype(np.float32)
        self.intrinsics = (k, d)
        self.u_coords = u.astype(np.float32)
        self.info_key = key
        self.frame_id = msg.header.frame_id
        self.get_logger().info(
            f"RGB rays ready: {msg.width}x{msg.height}, fx={msg.k[0]:.2f}, "
            f"fy={msg.k[4]:.2f}, stride={self.stride}"
        )

    def on_depth(self, msg):
        if self.rays is None:
            return
        if (msg.width, msg.height) != self.info_key[:2] or msg.header.frame_id != self.frame_id:
            self.get_logger().warn("Depth size/frame differs from RGB calibration", throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if now < self.next_publish:
            return
        # Throttle rejected frames too; an invalid floor must not saturate the
        # Pi while the display is deliberately withholding uncorrected data.
        self.next_publish = now + 1.0 / self.max_rate_hz

        if msg.encoding == "16UC1":
            dtype, scale = np.dtype("<u2"), 0.001
        elif msg.encoding == "32FC1":
            dtype, scale = np.dtype("<f4"), 1.0
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}", throttle_duration_sec=5.0)
            return
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        status = dict(ready=False, depth_stamp_s=stamp_s,
            source="pallet_floor_depth_skew" if self.floor_reference else "configured_static_preview",
            reference_u=self.reference_u, frame_id=msg.header.frame_id,
            shared_pallet_depth_geometry=self.floor_reference,
            camera_roll_assumed_zero=self.floor_reference,
            empirical_pallet_target_bias_applied=False)
        try:
            if self.floor_reference:
                validate_capture_stamp(stamp_s, self.get_clock().now().nanoseconds * 1e-9,
                                       self.max_input_age_s)
            row_width = msg.step // dtype.itemsize
            depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)[:, : msg.width]
            depth_mm = depth if scale == .001 else depth.astype(np.float64) * 1000.
            slope = self.inverse_depth_x_slope
            reason = "current capture corrected with shared pallet geometry"
            if self.floor_reference:
                try:
                    slope, diagnostics = floor_depth_skew(depth_mm, self.intrinsics)
                    self.last_valid_slope = slope
                    status.update(diagnostics)
                except ValueError as floor_exc:
                    if not self.fallback_to_static_slope:
                        raise
                    slope = self.last_valid_slope
                    status["source"] = "floor_skew_fallback_last_valid"
                    status["floor_fit_issue"] = str(floor_exc)
                    reason = "floor obscured; continuing RGB-D obstacle cloud with last valid slope"
            xyz = project_corrected_depth(depth_mm[::self.stride, ::self.stride],
                self.rays, self.u_coords, slope, self.reference_u)
            status.update(ready=True, inverse_depth_x_slope=slope, reason=reason)
        except (ValueError, TypeError) as exc:
            status['reason'] = str(exc)
            self.publish_status(status)
            self.get_logger().warn("Withholding camera obstacle cloud: " + str(exc),
                                   throttle_duration_sec=5.)
            return

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = xyz.shape[:2]
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.is_dense = False
        cloud.data = xyz.tobytes()
        self.publisher.publish(cloud)
        self.publish_status(status)
        self.last_publish = now

    def publish_status(self, status):
        status['published_ros_s'] = self.get_clock().now().nanoseconds * 1e-9
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(message)


def main():
    rclpy.init()
    node = CalibratedCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
