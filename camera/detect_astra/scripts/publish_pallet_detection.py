#!/usr/bin/env python3
"""YOLO semantic acquisition only. Pausing this node does not stop RGB-D tracking."""
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from action_msgs.msg import GoalStatusArray
from detected_pallets_interfaces.msg import PalletDetection, PalletTarget

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from astra_rgbd_stream import SHAPE, OPTICAL_FRAME, decode_image, stamp_ns
from pallet_perception_config import parse_args
from pallet_preview import draw_detections
from pallet_acquisition import AcquisitionSession, valid_target
from pallet_goal_gate import RecognitionGoalGate
from pallet_rgbd_health import RGBDHealth
from pallet_detection_selection import (unique_pallet_indices, PALLET_CLASSES,
                                        MIN_PALLET_CONFIDENCE)


class DetectionNode(Node):
    VERIFY_RGBD_GRACE_S = RGBDHealth.WINDOW_S
    VERIFY_RGBD_TARGET_TIMEOUT_S = 1.0

    def __init__(self, args):
        super().__init__('pallet_detection')
        self.args = args
        self.lock = threading.Lock()
        self.pending_images = {'rgb': deque(maxlen=1)}
        self.inference_enabled = not args.start_paused
        self.inference_generation = 0
        self.inference_busy = False
        self.inference_started = None
        self.inference_attempts = 0
        self.inference_ms = None
        self.input_age_ms = None
        self.result_age_ms = None
        self.model_ms = None
        self.seed_pack_ms = None
        self.seed_publish_ms = None
        self.published_seeds = 0
        self.last_issue = ''
        self.last_issue_log_wall = 0.
        self.pallet_candidates = []
        self.rgb_received = 0
        self.rgb_received_wall = None
        self.rgbd_health = RGBDHealth()
        self.measure_after_ros = None
        self.verify_started_wall = None
        self.verify_last_valid_wall = None
        self.tracking_status = None
        self.goal_gate = (RecognitionGoalGate(self.get_clock().now().nanoseconds/1e9)
                          if getattr(args, 'recognize_after_goal', False) else None)
        self.acquisition = AcquisitionSession(getattr(args, 'max_seed_age_ms', 1000.)/1000)
        self.alignment_active = False
        self.alignment_reserved_until = 0.
        self.prepare_waiters = []
        if self.inference_enabled:
            self.acquisition.start(time.monotonic(), self.get_clock().now().nanoseconds/1e9)
        self.publisher = self.create_publisher(PalletDetection, '/pallet/detection', 1)
        self.preview_publisher = self.create_publisher(
            CompressedImage, '/pallet/detection/preview/compressed',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', lambda msg: self.receive('rgb', msg),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.enabled_publisher = self.create_publisher(
            Bool, '/pallet/enabled', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                             durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.enabled_service = self.create_service(SetBool, '/pallet/set_enabled', self.set_enabled)
        self.create_service(Trigger, '/pallet/r3/start_recognition', self.start_recognition)
        self.phase_publisher = self.create_publisher(
            String, '/pallet/recognition/status', QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PalletTarget, '/pallet/target', self.on_target, 1)
        self.create_subscription(String, '/pallet/tracking/status', self.on_tracking, 1)
        self.create_subscription(String, '/pallet/alignment/status', self.on_alignment, 1)
        self.prepare_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '/pallet/prepare_alignment', self.prepare_alignment,
                            callback_group=self.prepare_group)
        if self.goal_gate is not None:
            self.acquisition.reason = 'Chờ goal nhận diện mới; YOLO đang tắt'
            self.create_subscription(GoalStatusArray, args.nav_status_topic, self.on_nav_status,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_service(Trigger, '/pallet/arm_recognition_goal', self.arm_goal)
        self.enabled_timer = self.create_timer(.2, self.publish_enabled)
        self.control_timer = self.create_timer(.05, self.control_tick)
        self.publish_enabled()

    def publish_enabled(self):
        with self.lock:
            wall, ros = time.monotonic(), self.get_clock().now().nanoseconds/1e9
            self.enabled_publisher.publish(Bool(data=self.inference_enabled))
            state = self.acquisition.snapshot(wall, ros)
            state.update(enabled=self.inference_enabled, busy=self.inference_busy)
            state.update(published_ros_s=ros, rgb_received=self.rgb_received,
                         rgb_receive_age_s=(wall-self.rgb_received_wall
                                            if self.rgb_received_wall is not None else None),
                         inference_busy_s=(wall-self.inference_started
                            if self.inference_busy and self.inference_started is not None else 0.))
            state.update(inference_attempts=self.inference_attempts, inference_ms=self.inference_ms,
                         input_age_ms=self.input_age_ms, result_age_ms=self.result_age_ms,
                         model_ms=self.model_ms, seed_pack_ms=self.seed_pack_ms,
                         seed_publish_ms=self.seed_publish_ms, published_seeds=self.published_seeds,
                         last_issue=self.last_issue,
                         min_pallet_confidence=MIN_PALLET_CONFIDENCE,
                         pallet_candidates=list(self.pallet_candidates))
            state['rgb_tracker_acknowledged'] = self.tracker_ready(ros)
            state['rgb_tracker'] = self.tracking_status
            state['rgbd_health'] = self.rgbd_health.snapshot(
                time.monotonic(), self.get_clock().now().nanoseconds/1e9)
            state['rgbd_health']['stable'] &= (self.acquisition.phase == 'READY'
                and not self.inference_enabled and not self.inference_busy)
            state['goal_gate'] = self.goal_gate.snapshot() if self.goal_gate is not None else None
            state['yolo_stopped_ros_s'] = self.measure_after_ros
            state['rgbd_verify_elapsed_s'] = (wall-self.verify_started_wall
                if self.verify_started_wall is not None else None)
            state['rgbd_last_valid_age_s'] = (wall-self.verify_last_valid_wall
                if self.verify_last_valid_wall is not None else None)
            if self.acquisition.phase == 'VERIFYING_RGBD':
                state['reason'] = 'YOLO đã tắt; đang chờ RGB-D mới và ổn định'
            self.phase_publisher.publish(String(data=json.dumps(state, allow_nan=False)))

    def on_tracking(self, msg):
        try:
            state = json.loads(msg.data)
            fields = ('published_ros_s', 'started_ros_s', 'seed_capture_ros_s', 'rgb_capture_ros_s')
            if (not isinstance(state, dict) or any(isinstance(state.get(k), bool)
                    or not isinstance(state.get(k), (float, int)) or not math.isfinite(state[k])
                    for k in fields) or not isinstance(state.get('active'), bool)):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            if (self.tracking_status is None
                    or state['published_ros_s'] > self.tracking_status['published_ros_s']):
                self.tracking_status = state

    def tracker_ready(self, now):
        """Do not switch off the seed producer before its consumer has a track.

        The ACK is tied to this recognition's start and genuine recent RGB.
        It grants no metric-pose validity; the post-off RGB-D gate is separate.
        """
        s = self.tracking_status or {}
        start = self.acquisition.started_ros
        return (start is not None and s.get('active') is True
            and abs(s['started_ros_s']-start) <= 1e-6
            and s['seed_capture_ros_s'] >= start-1e-6
            and s['rgb_capture_ros_s'] >= s['seed_capture_ros_s']
            and -.05 <= now-s['published_ros_s'] <= .25
            and -.05 <= now-s['rgb_capture_ros_s'] <= .25)

    def pause_locked(self):
        if self.inference_enabled:
            self.inference_enabled = False
            self.inference_generation += 1
            self.pending_images['rgb'].clear()

    def on_target(self, msg):
        now, wall = self.get_clock().now().nanoseconds/1e9, time.monotonic()
        with self.lock:
            valid = valid_target(msg, now)
            capture_ros = stamp_ns(msg)/1e9
            post_yolo_sample = (self.measure_after_ros is not None
                and capture_ros > self.measure_after_ros
                and not self.inference_enabled and not self.inference_busy)
            previous_count = len(self.acquisition.targets)
            self.acquisition.target(capture_ros, wall, now, valid,
                msg.camera_forward_m, msg.opening_lateral_m, msg.entry_heading_rad,
                lost=msg.status.startswith('LOST:'))
            geometry_reset = previous_count >= 3 and len(self.acquisition.targets) <= 1
            if self.acquisition.phase == 'VERIFYING_RGBD' and post_yolo_sample:
                if valid:
                    self.verify_last_valid_wall = wall
                elif (msg.status.startswith('LOST:') and -.05 <= now-capture_ros <= .25
                      and self.verify_started_wall is not None
                      and wall-self.verify_started_wall >= self.VERIFY_RGBD_GRACE_S):
                    self.verification_failed_locked()
                    return
                # Confirming/metadata gaps may clear the geometry window. Give
                # the independent tracker time to rebuild it with YOLO off.
                # Initial LOST also means "no RGB-D track yet", so it receives
                # the same grace instead of bypassing that collection window.
                # Invalid samples remain invalid in RGBDHealth below; they do
                # not extend the last-valid watchdog or qualify old geometry.
            elif self.acquisition.phase == 'READY' and (not valid or geometry_reset):
                if self.alignment_active:
                    self.invalidate_locked('Phép đo pallet mất ổn định trong khi xe đang căn')
                elif not self.acquisition.accuracy_validated:
                    self.acquisition.phase = 'CALIBRATION_REQUIRED'
                    self.acquisition.locked = False
                    self.acquisition.reason = 'YOLO đã tắt; RGB-D mất dấu, cần nhận diện lại sau hiệu chuẩn'
                else:
                    self.start_locked()
                return
            if (post_yolo_sample
                    and self.acquisition.phase in ('VERIFYING_RGBD', 'READY', 'CALIBRATION_REQUIRED')):
                self.rgbd_health.sample(capture_ros, wall, now, valid,
                    msg.camera_forward_m, msg.opening_lateral_m, msg.entry_heading_rad)

    def verification_failed_locked(self):
        if self.acquisition.accuracy_validated:
            self.start_locked()
        else:
            self.acquisition.phase = 'CALIBRATION_REQUIRED'
            self.acquisition.locked = False
            self.acquisition.reason = 'YOLO đã tắt; RGB-D mất dấu, cần nhận diện lại sau hiệu chuẩn'

    def invalidate_locked(self, reason):
        self.pause_locked()
        self.acquisition.pause()
        self.acquisition.locked = False
        self.acquisition.phase = 'PAUSED'
        self.acquisition.reason = reason
        self.rgbd_health.reset()
        self.measure_after_ros = None
        self.verify_started_wall = self.verify_last_valid_wall = None

    def start_locked(self):
        self.inference_enabled = True
        self.inference_generation += 1
        self.pending_images['rgb'].clear()
        self.inference_attempts = 0
        self.last_issue = ''
        self.pallet_candidates = []
        self.published_seeds = 0
        self.seed_publish_ms = None
        self.rgbd_health.reset()
        self.measure_after_ros = None
        self.verify_started_wall = self.verify_last_valid_wall = None
        self.tracking_status = None
        self.acquisition.start(time.monotonic(), self.get_clock().now().nanoseconds/1e9)

    def arm_goal(self, request, response):
        with self.lock:
            if self.alignment_active or time.monotonic() < self.alignment_reserved_until:
                response.success, response.message = False, 'Dừng căn xe trước khi chờ goal mới'
                return response
            self.goal_gate.arm(self.get_clock().now().nanoseconds/1e9)
            self.invalidate_locked('Chờ goal nhận diện mới; YOLO đang tắt')
        self.publish_enabled()
        response.success, response.message = True, 'Đã chờ goal mới; chỉ nhận diện khi goal thành công'
        return response

    def on_nav_status(self, msg):
        entries = [(bytes(s.goal_info.goal_id.uuid).hex(),
                    s.goal_info.stamp.sec+s.goal_info.stamp.nanosec/1e9, s.status)
                   for s in msg.status_list]
        with self.lock:
            acquire = self.goal_gate.update(entries)
            if self.goal_gate.navigation_active:
                self.invalidate_locked('Xe đang tới goal; YOLO đang tắt')
            elif acquire and not self.alignment_active and time.monotonic() >= self.alignment_reserved_until:
                self.start_locked()
            elif self.goal_gate.phase == 'GOAL_FAILED':
                self.invalidate_locked('Goal bị hủy/thất bại; chưa bật YOLO. Chờ chọn lại goal nhận diện')
        self.publish_enabled()

    def on_alignment(self, msg):
        try:
            state = json.loads(msg.data)
            active = state.get('active') is True or state.get('preparing') is True
        except (ValueError, AttributeError):
            return
        with self.lock:
            self.alignment_active = active
            if active:
                self.pause_locked()

    def control_tick(self):
        wall, now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
        with self.lock:
            if self.tracker_ready(now) and self.acquisition.tick(wall, now):
                self.pause_locked()
            if (not self.inference_enabled and not self.inference_busy
                    and self.acquisition.phase == 'STOPPING_YOLO'):
                self.rgbd_health.reset()
                self.measure_after_ros = now
                self.verify_started_wall = wall
                self.verify_last_valid_wall = None
                self.acquisition.yolo_stopped()
            if self.acquisition.phase == 'VERIFYING_RGBD':
                if self.rgbd_health.snapshot(wall, now)['stable']:
                    self.acquisition.verified()
                elif (self.verify_started_wall is not None
                      and wall-self.verify_started_wall >= self.VERIFY_RGBD_GRACE_S
                      and wall-(self.verify_last_valid_wall if self.verify_last_valid_wall is not None
                                else self.verify_started_wall) >= self.VERIFY_RGBD_TARGET_TIMEOUT_S):
                    # Do not re-enable YOLO indefinitely when accuracy is not
                    # validated; invalid callbacks cannot keep this alive.
                    self.verification_failed_locked()
            remaining = []
            for future, deadline in self.prepare_waiters:
                if future.done():
                    continue
                if not self.inference_busy or wall >= deadline:
                    ready = (not self.inference_enabled and not self.inference_busy
                             and self.acquisition.phase == 'READY'
                             and self.rgbd_health.snapshot(wall, now)['stable'])
                    future.set_result((ready, 'YOLO đã tắt; phép đo sẵn sàng' if ready else
                        'Chưa xác nhận pallet/phép đo mới hoặc YOLO chưa dừng xong'))
                else:
                    remaining.append((future, deadline))
            self.prepare_waiters = remaining

    async def prepare_alignment(self, request, response):
        with self.lock:
            self.pause_locked()
            self.acquisition.pause()
            self.alignment_reserved_until = time.monotonic()+3.
            future = Future(executor=self.executor)
            self.prepare_waiters.append((future, time.monotonic()+1.0))
        self.publish_enabled()
        response.success, response.message = await future
        return response

    def enable_issue_locked(self):
        if self.goal_gate is not None and (self.goal_gate.armed or self.goal_gate.navigation_active
                                           or self.goal_gate.phase != 'GOAL_SUCCEEDED'):
            return 'Chờ goal nhận diện thành công trước khi bật YOLO'
        if self.alignment_active or time.monotonic() < self.alignment_reserved_until:
            return 'Dừng căn xe trước khi nhận diện lại'
        return None

    def start_recognition(self, request, response):
        """One request creates one fresh session, without an off/on race."""
        with self.lock:
            issue = self.enable_issue_locked()
            if issue:
                response.success, response.message = False, issue
                return response
            self.start_locked()
            identity = dict(session=self.acquisition.session,
                            started_ros_s=self.acquisition.started_ros)
        self.publish_enabled()
        response.success, response.message = True, json.dumps(identity)
        return response

    def set_enabled(self, request, response):
        with self.lock:
            issue = self.enable_issue_locked() if request.data else None
            if issue:
                response.success, response.message = False, issue
                return response
            if self.inference_enabled != request.data:
                if request.data:
                    self.start_locked()
                else:
                    self.pause_locked()
                    self.acquisition.pause()
        self.publish_enabled()
        response.success = True
        response.message = ('Nhận diện tới khi đủ dữ liệu mới và ổn định; không giới hạn tổng thời gian'
                            if request.data else 'YOLO paused; RGB-D tracker continues independently')
        return response

    def inference_state(self):
        with self.lock:
            return self.inference_enabled, self.inference_generation

    def receive(self, stream, msg):
        with self.lock:
            self.rgb_received += 1
            self.rgb_received_wall = time.monotonic()
            if self.inference_enabled:
                self.pending_images[stream].append((msg, time.monotonic()))

    def take_image(self):
        with self.lock:
            if not self.inference_enabled or not self.pending_images['rgb']:
                return None
            msg, received = self.pending_images['rgb'].pop()
            self.pending_images['rgb'].clear()
            capture_age = (self.get_clock().now().nanoseconds-stamp_ns(msg))/1e9
            if (time.monotonic()-received > self.args.max_age_ms/1000
                    or not -.1 <= capture_age <= self.args.max_age_ms/1000):
                self.last_issue = f'Waiting for fresh RGB; previous capture age {capture_age*1000:.0f} ms'
                return None
            return msg

    def publish_result(self, msg, generation):
        with self.lock:
            wall, now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
            if self.acquisition.tick(wall, now):
                self.pause_locked()
            if not self.inference_enabled or generation != self.inference_generation:
                return False
            self.acquisition.detection(stamp_ns(msg)/1e9, wall, now,
                msg.valid and msg.header.frame_id == OPTICAL_FRAME, msg.confidence,
                msg.bbox_xyxy, msg.class_name.strip().lower())
            started = time.monotonic()
            self.publisher.publish(msg)
            self.seed_publish_ms = (time.monotonic()-started)*1000
            if msg.valid:
                self.published_seeds += 1
            return True

    def begin_inference(self, generation):
        with self.lock:
            if self.acquisition.tick(time.monotonic(), self.get_clock().now().nanoseconds/1e9):
                self.pause_locked()
            if not self.inference_enabled or generation != self.inference_generation:
                return False
            self.inference_busy = True
            self.inference_started = time.monotonic()
            self.inference_attempts += 1
            return True

    def end_inference(self):
        with self.lock:
            self.inference_busy = False
            if self.inference_started is not None:
                self.inference_ms = (time.monotonic()-self.inference_started)*1000

    def inference_issue(self, reason):
        with self.lock:
            self.last_issue = reason
        self.get_logger().warn(reason)

    def predict(self, rgb_msg, model, generation):
        age = (self.get_clock().now().nanoseconds - stamp_ns(rgb_msg)) / 1e9
        with self.lock:
            self.input_age_ms = age*1000
        if (rgb_msg.height, rgb_msg.width) != SHAPE or rgb_msg.header.frame_id != OPTICAL_FRAME:
            raise ValueError('RGB resolution or frame does not match the calibrated camera')
        if not -.1 <= age <= self.args.max_age_ms / 1000:
            self.inference_issue(f'Ảnh đầu vào YOLO quá cũ: {age*1000:.0f} ms')
            return False
        image = decode_image(rgb_msg)
        model_started = time.monotonic()
        result = model.predict(image, conf=self.args.conf, iou=self.args.nms_iou, imgsz=SHAPE,
                               retina_masks=True, device=self.args.device, verbose=False)[0]
        model_finished = time.monotonic()
        detection = PalletDetection()
        detection.header = rgb_msg.header
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        confidence = result.boxes.conf.cpu().numpy()
        indices = unique_pallet_indices(boxes, classes, confidence, result.names)
        with self.lock:
            self.pallet_candidates = [dict(class_name=result.names[int(cls)],
                confidence=float(confidence[i]), bbox_xyxy=boxes[i].astype(float).tolist(),
                eligible=i in indices) for i, cls in enumerate(classes)
                if result.names[int(cls)].strip().lower() in PALLET_CLASSES
                and np.isfinite(confidence[i]) and np.isfinite(boxes[i]).all()]
        detection.status = (f'expected one pallet with confidence >= {MIN_PALLET_CONFIDENCE:.2f}, '
                            f'found {len(indices)}')
        if len(indices) == 1 and result.masks is not None:
            index = indices[0]
            mask = result.masks.data[index].cpu().numpy().astype(np.uint8)
            if mask.shape != SHAPE:
                raise ValueError('YOLO mask is not full-resolution RGB coordinates')
            ok, png = cv2.imencode('.png', mask * 255)
            if not ok:
                raise ValueError('Could not encode seed mask')
            detection.valid = True
            detection.status = 'YOLO_SEED'
            detection.class_name = result.names[int(result.boxes.cls[index].item())]
            detection.confidence = float(result.boxes.conf[index].item())
            detection.bbox_xyxy = result.boxes.xyxy[index].cpu().numpy().astype(float).tolist()
            detection.rgb = rgb_msg
            detection.mask.header = rgb_msg.header
            detection.mask.format = 'png'
            detection.mask.data = png.tobytes()
        age = (self.get_clock().now().nanoseconds - stamp_ns(rgb_msg)) / 1e9
        with self.lock:
            self.result_age_ms = age*1000
            self.model_ms = (model_finished-model_started)*1000
            self.seed_pack_ms = (time.monotonic()-model_finished)*1000
        seed_age_ms = getattr(self.args, 'max_seed_age_ms', 1000.)
        if not -.1 <= age <= seed_age_ms / 1000:
            self.inference_issue(f'Mẫu nhận diện YOLO quá cũ: {age*1000:.0f} ms, giới hạn {seed_age_ms:.0f} ms')
            return True
        if not self.publish_result(detection, generation):
            return True
        with self.lock:
            self.last_issue = '' if detection.valid else detection.status
        # Record why repeated inference produces no seed. Otherwise the target
        # log only repeats its old loss reason, hiding the actual blocker.
        if not detection.valid and time.monotonic()-self.last_issue_log_wall >= 2.:
            self.get_logger().warn('YOLO chưa có seed pallet hợp lệ: ' + detection.status)
            self.last_issue_log_wall = time.monotonic()
        if self.preview_publisher.get_subscription_count():
            visible = [i for i, cls in enumerate(classes)
                       if result.names[int(cls)].strip().lower() not in PALLET_CLASSES or i in indices]
            annotated = draw_detections(image, boxes[visible], classes[visible],
                                       confidence[visible], result.names)
            ok, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                preview = CompressedImage(header=rgb_msg.header, format='jpeg', data=jpeg.tobytes())
                with self.lock:
                    if self.inference_enabled and generation == self.inference_generation:
                        self.preview_publisher.publish(preview)
        return True


def load_model(args):
    # Only this process imports or loads a neural network.
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(args.cpu_threads)
    model = YOLO(str(args.model), task='segment')
    if model.task != 'segment':
        raise ValueError('Expected a YOLO segmentation model')
    model.predict(np.zeros((*SHAPE, 3), np.uint8), imgsz=SHAPE, retina_masks=True,
                  device=args.device, verbose=False)
    param = next(args.model.glob('*.ncnn.param'), None) if args.model.is_dir() else None
    if param is not None:
        net = model.predictor.model.net
        net.clear()
        net.opt.num_threads = args.cpu_threads
        net.opt.openmp_blocktime = 0
        net.opt.use_fp16_packed = False
        net.opt.use_fp16_storage = False
        net.opt.use_fp16_arithmetic = False
        if net.load_param(str(param)) != 0 or net.load_model(str(param.with_suffix('.bin'))) != 0:
            raise ValueError('Could not load FP32 NCNN model')
        # The initial Ultralytics warmup used its default NCNN network, which
        # was just replaced above. Warm the configured FP32 network before
        # reporting ready, while the real workflow is still paused at A/Nav2.
        model.predict(np.zeros((*SHAPE, 3), np.uint8), imgsz=SHAPE, retina_masks=True,
                      device=args.device, verbose=False)
    return model


def configure_detector_transport():
    """Do not hold camera callbacks on synchronous network writes to readers.

    This process publishes semantic seeds/status only. The receiver still
    validates the original image capture stamp; queued seeds never gain age.
    """
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') == 'rmw_fastrtps_cpp':
        os.environ['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'


def main():
    args = parse_args('detector')
    # Prefer current camera/measurement callbacks when inference saturates CPU.
    # This affects only the detector process, not the RGB-D tracker.
    os.nice(5)
    cv2.setNumThreads(1)
    model = load_model(args)
    print('YOLO model ready; waiting for a fresh recognition request.' if args.start_paused
          else 'YOLO model ready; acquiring pallet.', flush=True)
    configure_detector_transport()
    rclpy.init()
    node = DetectionNode(args)
    executor = SingleThreadedExecutor()
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
                raise
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    next_at = 0.0
    try:
        while rclpy.ok():
            enabled, generation = node.inference_state()
            if not enabled or time.monotonic() < next_at:
                time.sleep(.01)
                continue
            msg = node.take_image()
            if msg is None:
                time.sleep(.005)
                continue
            if not node.begin_inference(generation):
                continue
            started_at = time.monotonic()
            try:
                used_inference = node.predict(msg, model, generation)
                # A discarded stale frame must not consume the next whole
                # YOLO period. Wait for the next fresh camera capture instead.
                next_at = (started_at + 1/args.max_fps
                           if used_inference is not False and args.max_fps else 0.)
            except (ValueError, RuntimeError, cv2.error) as exc:
                next_at = started_at+1/args.max_fps if args.max_fps else 0.
                node.inference_issue(str(exc))
            finally:
                node.end_inference()
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
