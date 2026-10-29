#!/usr/bin/env python3
"""Publish experimental pallet opening geometry from YOLO RGB and Astra depth.

This node publishes measurements only. It never commands the robot. A valid
message means that the current image passed geometric checks, not that the
camera calibration or robot insertion path has been verified for motion.
"""

import argparse
from collections import deque
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
import torch
from ultralytics import YOLO
import yaml

from detected_pallets_interfaces.msg import PalletTarget

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from yolo_depth import (CameraReceiver, OPTICAL_FRAME, SHAPE,
                        decode_image, stamp_ns, validate_intrinsics)
from pallet_target_geometry import measure_pallet_target


class TargetNode(CameraReceiver):
    def __init__(self, args):
        super().__init__(args, node_name='pallet_target')
        self.last_source_log = float('inf')
        self.pending_images = {'rgb': deque(maxlen=12), 'depth': deque(maxlen=16)}
        self.pending_color_info = deque(maxlen=16)
        self.publisher = self.create_publisher(
            PalletTarget, '/pallet/target',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE))

    def receive(self, stream, msg):
        with self.lock:
            self.pending_images[stream].append((msg, time.monotonic()))

    def receive_info(self, stream, msg):
        with self.lock:
            self.infos[stream] = msg
            if stream == 'rgb':
                self.pending_color_info.append((msg, time.monotonic()))

    def take_pair(self):
        """Match the freshest RGB/Depth pair within 50 ms, keeping short queues."""
        with self.lock:
            now = time.monotonic()
            max_age = self.args.max_age_ms * 0.001
            for stream in ('rgb', 'depth'):
                queue = self.pending_images[stream]
                while queue and now - queue[0][1] > max_age:
                    queue.popleft()
            while self.pending_color_info and now - self.pending_color_info[0][1] > 1.0:
                self.pending_color_info.popleft()
            rgb_queue = self.pending_images['rgb']
            depth_queue = self.pending_images['depth']
            candidates = []
            for ri, (rgb, rt) in enumerate(rgb_queue):
                if not stamp_ns(rgb):
                    continue
                for di, (depth, dt) in enumerate(depth_queue):
                    delta_ms = (stamp_ns(depth) - stamp_ns(rgb)) / 1e6
                    if stamp_ns(depth) and abs(delta_ms) <= self.args.max_delta_ms:
                        candidates.append((max(rt, dt), -abs(delta_ms), ri, di, delta_ms))
            if not candidates:
                self.status = 'Waiting for synchronized RGB / Depth'
                return None
            _, _, ri, di, delta_ms = max(candidates)
            rgb, _ = rgb_queue[ri]
            depth, _ = depth_queue[di]
            for _ in range(ri + 1):
                rgb_queue.popleft()
            for _ in range(di + 1):
                depth_queue.popleft()
            info = min(self.pending_color_info,
                       key=lambda entry: abs(stamp_ns(entry[0]) - stamp_ns(rgb)))[0] \
                if self.pending_color_info else self.infos.get('rgb')
            registration = dict(self.registration)
            registration['source_diagnostics'] = dict(self.source_diagnostics)
            registration['pair_publisher_gids'] = dict(self.last_source_gid)
            registration['rgb_camera_info_delta_ms'] = (
                (stamp_ns(info) - stamp_ns(rgb)) / 1e6 if info is not None else None)
            registration['check_age_ms'] = (now - self.registration_checked) * 1000
            if registration['check_age_ms'] > 3000:
                registration.update(verified=False, reason='registration confirmation expired')
            self.status = f'Accepted dt={delta_ms:+.2f} ms'
            return rgb, depth, delta_ms, info, self.infos.get('depth'), registration


def empty_message(stamp, status, delta_ms=math.nan):
    msg = PalletTarget()
    msg.header.stamp = stamp
    msg.header.frame_id = OPTICAL_FRAME
    msg.valid = False
    msg.status = status
    for name in ("camera_forward_m", "chassis_front_forward_m", "opening_lateral_m", "face_yaw_rad",
                 "entry_heading_rad", "opening_width_m", "opening_height_estimate_m", "leg_center_separation_m",
                 "left_forward_m", "right_forward_m", "yolo_confidence"):
        setattr(msg, name, math.nan)
    msg.rgb_depth_delta_ms = float(delta_ms)
    return msg


def confirmed(current, previous):
    return (abs(current['camera_forward_m'] - previous['camera_forward_m']) < 0.06
            and abs(current['opening_lateral_m'] - previous['opening_lateral_m']) < 0.035
            and abs(current['face_yaw_rad'] - previous['face_yaw_rad']) < math.radians(5))


def process_pair(pair, model, args, node, history):
    rgb_msg, depth_msg, delta_ms, color_info, _, registration = pair
    stamp = rgb_msg.header.stamp
    if (rgb_msg.height, rgb_msg.width) != SHAPE or (depth_msg.height, depth_msg.width) != SHAPE:
        raise ValueError('camera resolution is not 640x480')
    age_s = (node.get_clock().now().nanoseconds - stamp_ns(rgb_msg)) * 1e-9
    if not -0.1 <= age_s <= args.max_result_age_ms * 0.001:
        raise ValueError(f'RGB image age {age_s:.3f} s outside limit')
    intrinsics, status = validate_intrinsics(rgb_msg, depth_msg, color_info, registration)
    if intrinsics is None:
        raise ValueError(status)
    image = decode_image(rgb_msg)
    depth = decode_image(depth_msg, depth=True)
    results = model.predict(image, conf=args.conf, imgsz=640,
                            retina_masks=True, device=args.device, verbose=False)[0]
    if results.masks is None or len(results.boxes) != 1:
        raise ValueError(f'expected one pallet, found {len(results.boxes)}')
    mask = results.masks.data[0].cpu().numpy().astype(bool)
    bbox = results.boxes.xyxy[0].cpu().numpy()
    measurement = measure_pallet_target(
        mask, bbox, depth, intrinsics, pitch_deg=args.pitch_deg,
        inverse_depth_x_slope=args.inverse_depth_x_slope,
        reference_u=args.reference_u,
        camera_ahead_of_chassis_m=args.camera_ahead_of_chassis_m,
        expected_opening_width_m=args.expected_opening_width_m,
        opening_width_tolerance_m=args.opening_width_tolerance_m)
    # A slow inference result is never published as a fresh target.
    age_s = (node.get_clock().now().nanoseconds - stamp_ns(rgb_msg)) * 1e-9
    if not -0.1 <= age_s <= args.max_result_age_ms * 0.001:
        raise ValueError(f'result age {age_s:.3f} s outside limit')
    if history and not confirmed(measurement, history[-1]):
        history.clear()
    history.append(measurement)
    if len(history) < args.confirm_frames:
        raise ValueError(f'confirming front legs {len(history)}/{args.confirm_frames}')
    msg = empty_message(stamp, 'MEASURED: verify physical calibration before robot motion', delta_ms)
    msg.valid = True
    for name in ('camera_forward_m', 'chassis_front_forward_m', 'opening_lateral_m', 'face_yaw_rad',
                 'entry_heading_rad', 'opening_width_m', 'opening_height_estimate_m',
                 'leg_center_separation_m',
                 'left_forward_m', 'right_forward_m', 'left_depth_samples',
                 'right_depth_samples'):
        setattr(msg, name, measurement[name])
    msg.yolo_confidence = float(results.boxes.conf[0].item())
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=PROJECT / 'models/best_v2.pt')
    parser.add_argument('--correction-yaml', type=Path,
                        default=PROJECT / 'config/calibration/astra_pro_depth_x_skew_preview.yaml')
    parser.add_argument('--device', default='0' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--conf', type=float, default=0.60)
    parser.add_argument('--pitch-deg', type=float, default=20.0)
    parser.add_argument('--camera-ahead-of-chassis-m', type=float, default=0.0665,
                        help='Measured front of lens ahead of chassis front, metres')
    parser.add_argument('--expected-opening-width-m', type=float, default=0.252)
    parser.add_argument('--opening-width-tolerance-m', type=float, default=0.08)
    parser.add_argument('--confirm-frames', type=int, default=3)
    parser.add_argument('--max-delta-ms', type=float, default=50.0)
    parser.add_argument('--max-age-ms', type=float, default=250.0)
    parser.add_argument('--max-result-age-ms', type=float, default=500.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not args.model.is_file() or not args.correction_yaml.is_file():
        parser.error('Model or depth correction YAML missing')
    if not 0 <= args.conf <= 1 or args.confirm_frames < 1:
        parser.error('Invalid confidence or confirmation frame count')
    if not 0 < args.max_delta_ms <= 50 or args.max_age_ms <= 0 or args.max_result_age_ms <= 0:
        parser.error('Invalid image timing limit')
    if not all(math.isfinite(v) for v in (args.pitch_deg, args.expected_opening_width_m,
                                          args.camera_ahead_of_chassis_m)):
        parser.error('Invalid geometry parameter')
    parameters = yaml.safe_load(args.correction_yaml.read_text())['/spatial_depth_cloud']['ros__parameters']
    args.inverse_depth_x_slope = float(parameters['inverse_depth_x_slope'])
    args.reference_u = float(parameters['reference_u'])
    print(f'Pallet measurement: model={args.model.name}, device={args.device}, '
          f'pitch={args.pitch_deg:g} deg, correction={args.inverse_depth_x_slope:g} 1/m/pixel',
          flush=True)
    model = YOLO(str(args.model))
    if model.task != 'segment':
        parser.error('Expected a YOLO segmentation model')
    model.predict(np.zeros((*SHAPE, 3), np.uint8), conf=args.conf, imgsz=640,
                  retina_masks=True, device=args.device, verbose=False)

    rclpy.init(args=sys.argv)
    node = TargetNode(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    receiver = threading.Thread(target=executor.spin, daemon=True)
    receiver.start()
    history = deque(maxlen=args.confirm_frames)
    last_pair = time.monotonic()
    last_invalid = 0.0
    last_log = 0.0
    try:
        while rclpy.ok():
            pair = node.take_pair()
            now = time.monotonic()
            if pair is None:
                if now - last_pair > args.max_age_ms * 0.001 and now - last_invalid > 0.2:
                    history.clear()
                    msg = empty_message(node.get_clock().now().to_msg(), node.status)
                    node.publisher.publish(msg)
                    last_invalid = now
                time.sleep(0.01)
                continue
            last_pair = now
            try:
                msg = process_pair(pair, model, args, node, history)
            except (ValueError, RuntimeError) as exc:
                status = str(exc)
                if not status.startswith('confirming front legs'):
                    history.clear()
                msg = empty_message(pair[0].header.stamp, status, pair[2])
            node.publisher.publish(msg)
            if not msg.valid:
                last_invalid = time.monotonic()
            if time.monotonic() - last_log >= 2:
                if msg.valid:
                    node.get_logger().info(
                        f'forward={msg.camera_forward_m:.3f} m, left={msg.opening_lateral_m:.3f} m, '
                        f'face_yaw={math.degrees(msg.face_yaw_rad):+.2f} deg, '
                        f'width={msg.opening_width_m:.3f} m, height~={msg.opening_height_estimate_m:.3f} m, '
                        f'RGB/Depth dt={msg.rgb_depth_delta_ms:+.1f} ms')
                else:
                    node.get_logger().warn(msg.status)
                last_log = time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        receiver.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
