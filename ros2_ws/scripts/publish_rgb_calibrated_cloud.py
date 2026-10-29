#!/usr/bin/env python3
"""Preview registered Astra depth with rays from calibrated raw RGB pixels.

This assumes depth_registration maps depth pixels to the raw RGB image. It does
not correct an absolute Depth range bias or a remaining RGB/Depth pixel offset.
An optional inverse-depth x slope can preview a spatial sensor-skew correction.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class CalibratedCloud(Node):
    def __init__(self):
        super().__init__("rgb_calibrated_cloud")
        self.declare_parameter("pixel_stride", 2)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("inverse_depth_x_slope", 0.0)
        self.declare_parameter("reference_u", 370.0)
        self.declare_parameter("output_topic", "/camera/depth/points_rgb_calibrated")
        self.stride = int(self.get_parameter("pixel_stride").value)
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self.inverse_depth_x_slope = float(self.get_parameter("inverse_depth_x_slope").value)
        self.reference_u = float(self.get_parameter("reference_u").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        if self.stride < 1 or self.max_rate_hz <= 0:
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

        if msg.encoding == "16UC1":
            dtype, scale = np.dtype("<u2"), 0.001
        elif msg.encoding == "32FC1":
            dtype, scale = np.dtype("<f4"), 1.0
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}", throttle_duration_sec=5.0)
            return
        row_width = msg.step // dtype.itemsize
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)[:, : msg.width]
        z = depth[:: self.stride, :: self.stride].astype(np.float32) * scale
        if self.inverse_depth_x_slope != 0.0:
            valid = np.isfinite(z) & (z > 0) & (z <= 10)
            with np.errstate(divide="ignore", invalid="ignore"):
                inverse_z = np.where(valid, 1.0 / z, np.nan)
                inverse_z += self.inverse_depth_x_slope * (self.reference_u - self.u_coords)
                z = np.where(inverse_z > 0, 1.0 / inverse_z, np.nan).astype(np.float32)
        xyz = np.empty(z.shape + (3,), dtype=np.float32)
        xyz[:, :, 0] = self.rays[:, :, 0] * z
        xyz[:, :, 1] = self.rays[:, :, 1] * z
        xyz[:, :, 2] = z
        xyz[~np.isfinite(z) | (z <= 0) | (z > 10)] = np.nan

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = z.shape
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
        self.last_publish = now
        interval = 1.0 / self.max_rate_hz
        if self.next_publish == 0.0:
            self.next_publish = now + interval
        else:
            self.next_publish += interval
            if self.next_publish <= now:
                self.next_publish = now + interval


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
