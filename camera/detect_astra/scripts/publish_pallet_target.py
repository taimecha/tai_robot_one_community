#!/usr/bin/env python3
"""Measure pallet opening from live registered RGB-D; no YOLO/model imports.

YOLO supplies an initial mask via /pallet/detection. This process then tracks
the two front legs in each new RGB image and measures CURRENT registered Depth.
It publishes measurements and previews only, never robot commands.
"""
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import DurabilityPolicy
from detected_pallets_interfaces.msg import PalletDetection, PalletTarget

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from astra_rgbd_stream import (CameraReceiver, OPTICAL_FRAME, SHAPE, decode_image,
                               stamp_ns, validate_intrinsics)
from pallet_perception_config import parse_args
from pallet_rgbd_estimator import RGBDTargetEstimator
from pallet_messages import empty_message
from pallet_measurement_evidence import MeasurementEvidenceRecorder
from pallet_detection_selection import MIN_PALLET_CONFIDENCE


class TargetNode(CameraReceiver):
    def __init__(self, args):
        super().__init__(args, node_name='pallet_target')
        self.last_source_log = float('inf')
        self.pending_images = {'rgb': deque(maxlen=12), 'depth': deque(maxlen=16)}
        self.pending_color_info = deque(maxlen=16)
        self.pending_seed = None
        self.input_diagnostics = {}
        self.processing_diagnostics = {}
        self.last_seed_stamp_ns = 0
        self.recognition_start_ns = 0
        self.recognition_reset_pending = False
        self.info_wait_started_at = None
        evidence_dir = getattr(args, 'measurement_evidence_dir', None)
        self.measurement_evidence = MeasurementEvidenceRecorder(evidence_dir) if evidence_dir else None
        self.publisher = self.create_publisher(PalletTarget, '/pallet/target', 1)
        self.tracking_publisher = self.create_publisher(String, '/pallet/tracking/status', 1)
        self.preview_publisher = self.create_publisher(
            CompressedImage, '/pallet/preview/compressed',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.seed_sub = self.create_subscription(PalletDetection, '/pallet/detection', self.on_seed, 1)
        self.session_sub = self.create_subscription(String, '/pallet/recognition/status', self.on_session,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.r3_tracking_paused = bool(getattr(args, 'start_paused', False))
        self.create_service(SetBool, '/pallet/r3/pause_target_tracking', self.pause_r3_tracking)

    def pause_r3_tracking(self, request, response):
        with self.lock:
            self.r3_tracking_paused = bool(request.data)
            for queue in self.pending_images.values():
                queue.clear()
        response.success = True
        response.message = ('Pallet tracking paused; locked target remains unchanged'
                            if request.data else 'Pallet tracking resumed; new YOLO seed required')
        if request.data and self.measurement_evidence is not None:
            try:
                index = self.measurement_evidence.flush()
                delta = index['closest_capture_delta_s']
                relation = f'{delta:+.3f} s from locked capture' if delta is not None else 'no locked snapshot'
                response.message += (f'; RGB-D evidence saved: {self.measurement_evidence.directory}'
                                     f' ({relation})')
            except Exception as exc:
                # Recording failures must be visible without crashing the
                # measurement node in the middle of its service response.
                warning = f'RGB-D evidence save failed: {exc}'
                self.get_logger().error(warning)
                response.message += '; WARNING: ' + warning
        return response

    def on_session(self, msg):
        try:
            state = json.loads(msg.data)
            start = state.get('started_ros_s')
            if not isinstance(start, (float, int)) or not math.isfinite(start) or start <= 0:
                return
            start_ns = int(start*1e9)
        except (ValueError, AttributeError):
            return
        with self.lock:
            if start_ns > self.recognition_start_ns:
                self.recognition_start_ns = start_ns
                self.recognition_reset_pending = True
                self.pending_seed = None
                if self.measurement_evidence is not None:
                    self.measurement_evidence.clear()

    def take_session_reset(self):
        with self.lock:
            reset = self.recognition_reset_pending
            self.recognition_reset_pending = False
            return reset

    def publish_tracking(self, estimator, issue=''):
        """Acknowledge a current visual track, never a cached depth pose."""
        now = self.get_clock().now().nanoseconds/1e9
        with self.lock:
            inputs = {name: dict(value) for name, value in self.input_diagnostics.items()}
        state = dict(published_ros_s=now,
            started_ros_s=self.recognition_start_ns/1e9,
            seed_capture_ros_s=estimator.seed_stamp_ns/1e9,
            rgb_capture_ros_s=estimator.tracker.stamp_ns/1e9,
            active=bool(estimator.tracker.active and not self.r3_tracking_paused),
            confirming_samples=len(estimator.history), issue=issue,
            inputs=inputs, processing=dict(self.processing_diagnostics))
        self.tracking_publisher.publish(String(data=json.dumps(state, allow_nan=False)))

    def check_registration(self):
        started = time.monotonic()
        super().check_registration()
        elapsed = (time.monotonic()-started)*1000
        self.processing_diagnostics['registration_ms'] = elapsed
        if elapsed > 200:
            self.get_logger().warn(f'Camera source check took {elapsed:.0f} ms on its separate callback group')

    def on_seed(self, msg):
        if msg.valid and stamp_ns(msg) > max(self.last_seed_stamp_ns, self.recognition_start_ns):
            with self.lock:
                self.pending_seed = msg
                self.last_seed_stamp_ns = stamp_ns(msg)

    def take_seed(self, rgb_stamp_ns):
        with self.lock:
            msg = self.pending_seed
            if msg is None or stamp_ns(msg) > rgb_stamp_ns:
                return None
            self.pending_seed = None
        age = (self.get_clock().now().nanoseconds - stamp_ns(msg)) / 1e9
        if not -.1 <= age <= getattr(self.args, 'max_seed_age_ms', 1000.) / 1000:
            return None
        if (msg.header.frame_id != OPTICAL_FRAME or stamp_ns(msg.rgb) != stamp_ns(msg)
                or stamp_ns(msg.mask) != stamp_ns(msg)
                or msg.rgb.header.frame_id != OPTICAL_FRAME
                or msg.mask.header.frame_id != OPTICAL_FRAME):
            return None
        mask = cv2.imdecode(np.frombuffer(msg.mask.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if (mask is None or mask.shape != SHAPE or msg.mask.format != 'png'
                or msg.class_name.strip().lower() not in ('pallets', 'emty pallet')
                or not np.isfinite(msg.bbox_xyxy).all()
                or not MIN_PALLET_CONFIDENCE <= msg.confidence <= 1):
            return None
        image = decode_image(msg.rgb)
        if image.shape[:2] != SHAPE:
            return None
        return dict(rgb=image, mask=mask > 0, bbox=np.array(msg.bbox_xyxy),
                    stamp_ns=stamp_ns(msg), class_name=msg.class_name, confidence=msg.confidence)

    def receive(self, stream, msg):
        wall = time.monotonic()
        with self.lock:
            previous = self.input_diagnostics.get(stream, {})
            self.input_diagnostics[stream] = dict(received_wall_s=wall,
                callback_gap_ms=(wall-previous.get('received_wall_s', wall))*1000,
                capture_age_ms=(self.get_clock().now().nanoseconds-stamp_ns(msg))/1e6)
            self.pending_images[stream].append((msg, wall))

    def receive_info(self, stream, msg):
        with self.lock:
            self.infos[stream] = msg
            if stream == 'rgb':
                self.pending_color_info.append((msg, time.monotonic()))

    def take_pair(self):
        """Prefer a complete RGB/Depth/CameraInfo set, allowing callback skew."""
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
            requires_info_sync = len(self.source_diagnostics.get('endpoints', {}).get('rgb', [])) == 2
            candidates = []
            complete = []
            for ri, (rgb, rt) in enumerate(rgb_queue):
                if not stamp_ns(rgb):
                    continue
                for di, (depth, dt) in enumerate(depth_queue):
                    delta_ms = (stamp_ns(depth) - stamp_ns(rgb)) / 1e6
                    if stamp_ns(depth) and abs(delta_ms) <= self.args.max_delta_ms:
                        info = min(self.pending_color_info,
                                   key=lambda entry: abs(stamp_ns(entry[0])-stamp_ns(rgb)))[0] \
                            if self.pending_color_info else self.infos.get('rgb')
                        candidate = (max(rt, dt), -abs(delta_ms), ri, di, delta_ms, info)
                        candidates.append(candidate)
                        if (info is not None and (not requires_info_sync or
                                (stamp_ns(info) and abs(stamp_ns(info)-stamp_ns(rgb)) <= 50_000_000))):
                            complete.append(candidate)
            if not candidates:
                self.status = 'Waiting for synchronized RGB / Depth'
                self.info_wait_started_at = None
                return None
            if not complete:
                self.status = 'SYNC_WAIT: waiting for matching RGB CameraInfo (50 ms)'
                if self.info_wait_started_at is None:
                    self.info_wait_started_at = now
                if now-self.info_wait_started_at < .05:
                    return None
                # No matching info arrived. Return a fresh RGB-D frame so the
                # visual tracker advances, while validate_intrinsics rejects
                # its geometry. Never relax the CameraInfo timing gate.
            self.info_wait_started_at = None
            _, _, ri, di, delta_ms, info = max(complete or candidates, key=lambda c: c[:5])
            rgb, _ = rgb_queue[ri]
            depth, _ = depth_queue[di]
            for _ in range(ri + 1):
                rgb_queue.popleft()
            for _ in range(di + 1):
                depth_queue.popleft()
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

    def preview(self, header, image, estimator, msg):
        if not self.preview_publisher.get_subscription_count():
            return
        view = image.copy()
        if estimator.tracker.active:
            contours, _ = cv2.findContours(estimator.tracker.mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(view, contours, -1, (0, 220, 80), 1)
            x1, y1, x2, y2 = np.rint(estimator.tracker.bbox).astype(int)
            cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 80), 1)
        # Measurements and status live in the web cards, not over camera pixels.
        ok, encoded = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            self.preview_publisher.publish(CompressedImage(header=header, format='jpeg', data=encoded.tobytes()))


def process_pair(pair, args, node, estimator):
    rgb_msg, depth_msg, delta_ms, color_info, _, registration = pair
    stamp = rgb_msg.header.stamp
    if stamp_ns(rgb_msg) < getattr(node, 'recognition_start_ns', 0):
        estimator.history.clear()
        raise ValueError('Waiting for RGB captured after recognition started')
    age = (node.get_clock().now().nanoseconds - stamp_ns(rgb_msg)) / 1e9
    if not -.1 <= age <= args.max_result_age_ms / 1000:
        estimator.history.clear()
        raise ValueError('RGB-D pair outside result age limit')
    if (rgb_msg.header.frame_id != OPTICAL_FRAME or (rgb_msg.height, rgb_msg.width) != SHAPE):
        estimator.reset('RGB source frame or resolution changed')
        raise ValueError('RGB source frame or resolution changed')
    image = decode_image(rgb_msg)
    intrinsics, status = validate_intrinsics(rgb_msg, depth_msg, color_info, registration)
    if intrinsics is None:
        # A late CameraInfo packet must not erase the visual target. No pose is
        # emitted until current RGB-D metadata and all depth gates pass again.
        estimator.suspend_measurement(image, stamp_ns(rgb_msg))
        raise ValueError('SYNC_WAIT: ' + status)
    depth = decode_image(depth_msg, depth=True)
    seed = node.take_seed(stamp_ns(rgb_msg))
    measurement = estimator.process(image, depth, intrinsics, stamp_ns(rgb_msg), seed)
    age = (node.get_clock().now().nanoseconds - stamp_ns(rgb_msg)) / 1e9
    if not -.1 <= age <= args.max_result_age_ms / 1000:
        estimator.history.clear()
        raise ValueError('RGB-D processing result outside age limit')
    status = (f'TRACKED_RGBD; class={estimator.seed_class}; '
              f'seed_age_s={(stamp_ns(rgb_msg)-estimator.seed_stamp_ns)/1e9:.1f}')
    if args.measurement_bias is not None:
        status += '; target_bias_applied'
        raw_lateral = measurement.get('opening_lateral_camera_raw_m')
        if raw_lateral is not None:
            status += (f'; lateral_camera_raw_m={raw_lateral:+.4f}'
                       f'; lateral_total_bias_m={args.measurement_bias["lateral_bias_m"]:+.4f}'
                       f'; lateral_fork_m={measurement["opening_lateral_m"]:+.4f}')
        heading_bias = measurement.get('applied_heading_bias_deg',args.measurement_bias['face_yaw_bias_deg'])
        status += (f'; heading_bias_deg={heading_bias:.6f}'
                   f'; pre_bias_entry_deg={math.degrees(measurement["entry_heading_rad"])+heading_bias:.6f}')
    if measurement.get('heading_source'):
        status += (f'; heading_source={measurement["heading_source"]}'
                   f'; heading_depth_x_slope={measurement["heading_depth_x_slope"]:.9f}')
    if 'position_depth_x_slope' in measurement:
        status += (f'; position_depth_x_slope={measurement["position_depth_x_slope"]:.9f}'
                   f'; depth_correction_shared={str(measurement.get("depth_correction_shared", False)).lower()}'
                   f'; heading_requires_zero_camera_roll={str(measurement.get("heading_requires_zero_camera_roll", False)).lower()}')
    forward_bias = args.measurement_bias.get('forward_bias_m', 0.) if args.measurement_bias else 0.
    status += (f'; camera_pitch_deg={args.pitch_deg:.6f}'
               f'; depth_reference_u={args.reference_u:.6f}'
               f'; forward_bias_m={forward_bias:.6f}'
               '; camera_mount_yaw_deg=0.000000; camera_mount_yaw_verified=false')
    msg = empty_message(stamp, status, delta_ms)
    msg.valid = True
    for name in ('camera_forward_m', 'chassis_front_forward_m', 'opening_lateral_m',
                 'face_yaw_rad', 'entry_heading_rad', 'opening_width_m',
                 'opening_height_estimate_m', 'leg_center_separation_m',
                 'left_forward_m', 'right_forward_m', 'left_depth_samples', 'right_depth_samples'):
        setattr(msg, name, measurement[name])
    msg.yolo_confidence = math.nan
    recorder = getattr(node, 'measurement_evidence', None)
    if recorder is not None:
        try:
            recorder.record(stamp_ns(rgb_msg), image, depth, intrinsics,
                            estimator.tracker.mask, estimator.tracker.bbox, measurement,
                            dict(config=vars(args),
                                 rgb_stamp_ns=stamp_ns(rgb_msg), depth_stamp_ns=stamp_ns(depth_msg),
                                 rgb_frame_id=rgb_msg.header.frame_id,
                                 depth_frame_id=depth_msg.header.frame_id,
                                 camera_info_stamp_ns=stamp_ns(color_info),
                                 distortion_model=color_info.distortion_model,
                                 rgb_depth_delta_ms=delta_ms, registration=registration,
                                 seed_stamp_ns=estimator.seed_stamp_ns,
                                 seed_class=estimator.seed_class,
                                 seed_confidence=estimator.seed_confidence,
                                 heading_filter_history=list(estimator.history),
                                 tracker_points=estimator.tracker.points,
                                 tracker_point_leg_labels=estimator.tracker.labels))
        except Exception as exc:
            node.get_logger().error(f'RGB-D evidence buffer failed: {exc}')
    return msg


def main():
    args = parse_args('tracker')
    cv2.setNumThreads(1)
    rclpy.init()
    node = TargetNode(args)
    estimator = RGBDTargetEstimator(args)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_stop = threading.Event()
    def spin():
        try:
            while not spin_stop.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=.1)
        except ExternalShutdownException:
            pass
        except RuntimeError:
            if rclpy.ok():
                raise  # Preserve real executor failures; ignore only shutdown races.
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    last_pair, last_invalid, last_log = time.monotonic(), 0.0, 0.0
    next_measurement, next_preview = 0.0, 0.0
    try:
        while rclpy.ok():
            if node.r3_tracking_paused:
                time.sleep(.05)
                continue
            if node.take_session_reset():
                estimator.reset('new recognition session; waiting for a fresh YOLO seed')
                node.publish_tracking(estimator, estimator.loss_reason)
            now = time.monotonic()
            if now < next_measurement:
                time.sleep(min(.005, next_measurement-now))
                continue
            pair = node.take_pair()
            if pair is None:
                if now-last_pair > args.max_age_ms / 1000 and now-last_invalid > .2:
                    # Numbers become invalid now. Preserve visual state only
                    # within its separate 500-ms optical-flow gap limit.
                    if now-last_pair > .5:
                        estimator.reset('camera RGB-D input gap exceeded 500 ms')
                    node.publisher.publish(empty_message(node.get_clock().now().to_msg(), node.status))
                    node.publish_tracking(estimator, node.status)
                    last_invalid = now
                time.sleep(.002)
                continue
            last_pair = now
            next_measurement = now+1/args.track_max_fps if args.track_max_fps else 0
            processing_started = time.monotonic()
            try:
                msg = process_pair(pair, args, node, estimator)
            except (ValueError, RuntimeError, cv2.error) as exc:
                msg = empty_message(pair[0].header.stamp, str(exc), pair[2])
            if stamp_ns(msg) < node.recognition_start_ns:
                msg = empty_message(msg.header.stamp, 'Waiting for new recognition session')
            node.processing_diagnostics['rgbd_ms'] = (time.monotonic()-processing_started)*1000
            publishing_started = time.monotonic()
            node.publish_tracking(estimator, '' if msg.valid else msg.status)
            node.publisher.publish(msg)
            node.processing_diagnostics['publish_ms'] = (time.monotonic()-publishing_started)*1000
            if node.processing_diagnostics['publish_ms'] > 200:
                node.get_logger().warn('RGB-D publish elapsed '
                    f'{node.processing_diagnostics["publish_ms"]:.0f} ms')
            if now >= next_preview:
                node.preview(pair[0].header, decode_image(pair[0]), estimator, msg)
                next_preview = now+1/args.preview_fps if args.preview_fps else 0
            if now-last_log >= 2:
                if msg.valid:
                    if args.measurement_bias is not None and 'lateral_camera_raw_m=' in msg.status:
                        report = msg.status.split('; lateral_camera_raw_m=', 1)[1].split('; heading_bias_deg=', 1)[0]
                        node.get_logger().info('VỊ TRÍ PALLET TRƯỚC KHI CHẠY: camera_raw_m=' +
                                               report.replace('lateral_total_bias_m', 'bu_ngang_tong_m')
                                               .replace('lateral_fork_m', 'lech_theo_tam_cang_m') +
                                               f'; khoang_cach_goc_cang_m={msg.chassis_front_forward_m-.065:.3f}'
                                               f'; goc_huong_deg={math.degrees(msg.entry_heading_rad):+.2f}')
                    node.get_logger().info(f'RGB-D forward={msg.camera_forward_m:.3f} m, '
                                          f'left={msg.opening_lateral_m:+.3f} m, '
                                          f'entry={math.degrees(msg.entry_heading_rad):+.2f} deg; {msg.status}')
                else:
                    node.get_logger().warn(msg.status)
                last_log = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        spin_stop.set()
        executor.wake()
        thread.join(timeout=2)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
