#!/usr/bin/env python3
"""EKF odom pose r3 follower behind the collision monitor.

The locked RGB-D target is used once to build a frozen plan. During motion,
Wheel/IMU EKF odom supplies XY and yaw; raw IMU is diagnostic only. The
filtered LiDAR scan may stop, never steer, the robot.
The default workflow completes the APPROACH at C. Opt-in pickup traverses
C continuously, stops at B, then raises the homed lift from real feedback.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String, Float64MultiArray
from detected_pallets_interfaces.msg import PalletTarget

from r3_core import FrozenPlan, errors, fork_for_base, wrap, compose
from r3_real_obstacle_filter import read_real_target
from r3_tracking import FixedPlanFollower, write_csv
from r3_localization_contract import localization_spec, checked_status
from r3_lift_control import HomeBaseGuard, LiftControl
from r3_insertion_clearance import insertion_clearance

LIDAR_TIMEOUT_S = 1.0
LIDAR_START_FRESH_S = .30
LIDAR_START_STABLE_S = .50
ODOM_TIMEOUT_S = .20
YOLO_HEARTBEAT_TIMEOUT_S = .70
SENSOR_HOLD_LATCH_S = 1.0
C_RGBD_WAIT_LIMIT_S = 6.0
CONTROL_PERIOD_S = .05
INPUT_DRAIN_BUDGET_S = .015
LIFT_WHEEL_ODOM_MAX_AGE_S = .5
LIFT_GRAPH_CHECK_INTERVAL_S = .25


class LocalizationPending(ValueError):
    """Temporary delivery/freshness loss, never a reference-integrity failure."""


def tracking_localization_status(data, now_ros, expected):
    """Check identity before readiness so a restart cannot hide in a hold."""
    if data is None:
        raise LocalizationPending('waiting for localization control status')
    state = json.loads(data) if isinstance(data, str) else data
    if not isinstance(state, dict):
        raise ValueError('invalid localization control status')
    if state.get('integrity_fault'):
        raise ValueError(str(state['integrity_fault']))
    for key in ('run_id', 'map_file'):
        if state.get(key) is not None and state[key] != expected[key]:
            raise ValueError('AMCL restarted or its map/reference changed; recognize again')
    if state.get('frame_id') not in (None, 'r3_trial'):
        raise ValueError('invalid localization control frame')
    for key in ('correction_xyyaw', 'baseline_map_odom'):
        if key in state:
            value = state[key]
            if (not isinstance(value, list) or len(value) != 3
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                               and math.isfinite(v) for v in value)):
                raise ValueError('invalid localization control transform: '+key)
            if key == 'baseline_map_odom' and any(
                    abs(a-b) > 1e-8 for a,b in zip(value, expected[key])):
                raise ValueError('AMCL restarted or its map/reference changed; recognize again')
    if state.get('ready') is not True:
        raise LocalizationPending('localization pending: '+str(state.get('reason', 'not ready')))
    if (state.get('frame_id') != 'r3_trial' or not state.get('run_id')
            or not state.get('map_file')):
        raise ValueError('invalid localization control identity')
    published = state.get('published_ros_s')
    if (not isinstance(published, (int, float)) or isinstance(published, bool)
            or not math.isfinite(published) or now_ros-published < -.05):
        raise ValueError('invalid localization control timestamp')
    if now_ros-published > .5:
        raise LocalizationPending('localization control status is stale')
    return checked_status(state, now_ros, expected)


def configure_command_transport():
    """Keep Fast DDS network writes off the control thread, before ROS init."""
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') == 'rmw_fastrtps_cpp':
        os.environ['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'


def sensor_age(receive_age, capture_stamp, now_ros):
    """Include transport age, without extending freshness on message receipt."""
    if capture_stamp is None:  # No accepted message yet; the pose guard also applies.
        return receive_age
    capture_age = now_ros-capture_stamp
    return max(receive_age, capture_age) if capture_age >= -.05 else float('inf')


def drain_control_inputs(node, executor, max_callbacks=64):
    """Take pending source callbacks before deciding whether a source is stale.

    Sensor subscriptions retain one sample. A delayed control cycle must read
    that latest sample before testing its age, rather than run an overdue ROS
    timer ahead of queued input. Bound the drain so busy sources cannot prevent
    command generation. Source header age checks remain in each callback.
    """
    started = time.monotonic()
    initial_revision = node.input_revision
    handled = 0
    empty_waits = 0
    while (not node.finished and handled < max_callbacks
           and time.monotonic()-started < INPUT_DRAIN_BUDGET_S):
        try:
            handler, _, _ = executor.wait_for_ready_callbacks(timeout_sec=0.)
        except TimeoutException:
            # An iterator left by the waiting spin may describe an earlier
            # readiness snapshot. Its first timeout exhausts that snapshot;
            # refresh the wait set once before declaring the queue empty.
            empty_waits += 1
            if empty_waits >= 2:
                break
            continue
        # Same task/error handling as SingleThreadedExecutor.spin_once. A
        # discovery/QoS event is work too; it does not mean the inputs are empty.
        handler()
        if handler.exception() is not None:
            raise handler.exception()
        handler.result()
        handled += 1
        empty_waits = 0
    return node.input_revision-initial_revision, time.monotonic()-started


def run_control_loop(node, executor):
    """Service inputs continuously and compute commands at 20 Hz, without catch-up ticks."""
    next_tick = time.monotonic()
    while rclpy.ok() and not node.finished:
        now = time.monotonic()
        if now < next_tick:
            executor.spin_once(timeout_sec=min(.01, next_tick-now))
            continue
        count, duration = drain_control_inputs(node, executor)
        node.input_drain_callbacks = count
        node.input_drain_duration_s = duration
        if node.finished:
            break
        tick_started = time.monotonic()
        node.tick()
        # Never replay expired timer events after a slow callback. Drain fresh
        # input again before the next scheduled control decision.
        next_tick = max(tick_started+CONTROL_PERIOD_S, time.monotonic())


def diagnostic_numbers(values):
    """Preserve invalid raw measurements as JSON null without affecting control."""
    return [float(v) if math.isfinite(v) else None for v in values]


class OdomImuDiagnostics:
    """Bounded 20 Hz snapshots of independent source messages and issued commands.

    Source header stamps and local receive times are retained separately: a
    repeated or stale raw sample remains visible in each control snapshot.
    sensor_msgs/Imu does not contain the BNO055's calibration status.
    """
    def __init__(self, max_samples=6000, interval_s=.05):
        self.latest = {'wheel_odom': None, 'fused_odom': None, 'imu': None,
                       'localized_odom': None, 'wheel_pid': None}
        self.rows = deque(maxlen=max_samples)
        self.interval_s = interval_s
        self.last_sample = None
        self.dropped_samples = 0

    @staticmethod
    def header(msg, received):
        return dict(stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    frame_id=msg.header.frame_id, received_monotonic_s=received)

    def on_odom(self, source, msg, received):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.latest[source] = dict(self.header(msg, received),
            child_frame_id=msg.child_frame_id,
            position_m=diagnostic_numbers((p.x, p.y, p.z)),
            quaternion_xyzw=diagnostic_numbers((q.x, q.y, q.z, q.w)),
            linear_velocity_mps=diagnostic_numbers((v.x, v.y, v.z)),
            angular_velocity_rps=diagnostic_numbers((w.x, w.y, w.z)),
            pose_covariance=diagnostic_numbers(msg.pose.covariance),
            twist_covariance=diagnostic_numbers(msg.twist.covariance))

    def on_imu(self, msg, received):
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        self.latest['imu'] = dict(self.header(msg, received),
            quaternion_xyzw=diagnostic_numbers((q.x, q.y, q.z, q.w)),
            angular_velocity_rps=diagnostic_numbers((w.x, w.y, w.z)),
            linear_acceleration_mps2=diagnostic_numbers((a.x, a.y, a.z)),
            orientation_covariance=diagnostic_numbers(msg.orientation_covariance),
            angular_velocity_covariance=diagnostic_numbers(msg.angular_velocity_covariance),
            linear_acceleration_covariance=diagnostic_numbers(msg.linear_acceleration_covariance),
            calibration_status_available=False)

    def on_wheel_pid(self, msg, received):
        if len(msg.data) != 18:
            return
        values = diagnostic_numbers(msg.data)
        self.latest['wheel_pid'] = dict(received_monotonic_s=received,
            sequence=values[0], uptime_ms=values[1],
            wheels={name: dict(target_rad_s=values[2+4*i], measured_rad_s=values[3+4*i],
                output_pwm=values[4+4*i], position_rad=values[5+4*i])
                for i, name in enumerate(('FL', 'FR', 'RL', 'RR'))})

    def sample(self, monotonic_s, wall_s, state, pose, command, heading_source='odom_ekf'):
        if self.last_sample is not None and monotonic_s-self.last_sample < self.interval_s-1e-6:
            return False
        self.last_sample = monotonic_s
        if len(self.rows) == self.rows.maxlen:
            self.dropped_samples += 1
        self.rows.append(dict(monotonic_s=monotonic_s, wall_s=wall_s, state=state,
                              control_heading_source=heading_source,
                              accepted_control_pose_xyyaw=(diagnostic_numbers(pose)
                                                           if pose is not None else None),
                              issued_cmd_v_mps=command[0], issued_cmd_w_rps=command[1],
                              **self.latest))
        return True

    def write(self, output):
        output.mkdir(parents=True, exist_ok=True)
        path = output/'odom_imu_diagnostics.jsonl'
        with path.open('w') as stream:
            for row in self.rows:
                stream.write(json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n')
        return dict(path=str(path), samples=len(self.rows),
                    dropped_oldest_samples=self.dropped_samples,
                    max_samples=self.rows.maxlen, sampling_interval_s=self.interval_s,
                    physical_alignment_independently_measured=False,
                    imu_calibration_status_available=False)


def check_c_rgbd(samples, odom_gap_m, odom_yaw_error_rad, now):
    """Report fresh stationary RGB-D geometry and its disagreement with odom."""
    recent = [s for s in samples if now-s['received_wall'] <= 1.0]
    if (len(recent) < 3 or recent[-1]['received_wall'] < now-.35
            or recent[-1]['stamp_s']-recent[0]['stamp_s'] < .35):
        return None, 'need three fresh RGB-D pallet measurements at C'
    heading = statistics.median(s['heading_rad'] for s in recent)
    lateral = statistics.median(s['lateral_m'] for s in recent)
    gap = statistics.median(s['gap_m'] for s in recent)
    if (max(s['heading_rad'] for s in recent)-min(s['heading_rad'] for s in recent)
            > math.radians(1.)
            or max(s['lateral_m'] for s in recent)-min(s['lateral_m'] for s in recent) > .01):
        return None, 'RGB-D pallet angle or lateral position is unstable at C'
    report = dict(camera_heading_deg=math.degrees(heading),
                  odom_yaw_error_deg=math.degrees(odom_yaw_error_rad),
                  camera_odom_yaw_disagreement_deg=math.degrees(
                      wrap(heading+odom_yaw_error_rad)),
                  lateral_cm=lateral*100,
                  fork_tip_clearance_cm=(gap-.385)*100,
                  camera_odom_gap_difference_cm=(gap-odom_gap_m)*100)
    if abs(math.degrees(heading)) > .5 or abs(lateral) > .009:
        return report, (f'RGB-D at C: heading={math.degrees(heading):+.2f} deg, '
                        f'lateral={lateral*100:+.1f} cm; insertion limits exceeded')
    if abs(report['camera_odom_yaw_disagreement_deg']) > .5:
        return report, (f'RGB-D/odom yaw disagreement '
                        f'{report["camera_odom_yaw_disagreement_deg"]:+.2f} deg at C')
    if gap-.385 < .10 or abs(gap-odom_gap_m) > .05:
        return report, (f'RGB-D at C: fork-tip clearance={(gap-.385)*100:.1f} cm, '
                        f'camera/odom gap difference={(gap-odom_gap_m)*100:+.1f} cm')
    return report, ''


class RealTrial(Node):
    def __init__(self, plan, output, stop_at_c=False, control_timer=True, arrive_at_c=False,
                 pickup=False):
        super().__init__('r3_real_fixed_plan_trial')
        self.plan = plan
        self.output = output
        self.stop_at_c = stop_at_c
        self.arrive_at_c = arrive_at_c
        self.pickup = pickup
        self.lift = LiftControl(self) if pickup else None
        self.lift_history = []
        self.lift_base_pose = None
        self.lift_wheels = HomeBaseGuard() if pickup else None
        self.lift_next_graph_check = 0.
        self.lift_lidar_source_warnings = 0
        self.insertion_report = None
        self.insertion_logged = False
        self.insertion_frame_lock = None
        self.insertion_lock_requested_at = None
        self.insertion_lock_last_sent = None
        self.insertion_lock_wait_since = None
        self.geometry = json.loads(plan.geometry_json)
        self.target = json.loads(plan.target_json)['pose']
        self.localization = localization_spec(json.loads(plan.target_json))
        self.localized_pose = None
        self.localized_wall = 0.
        self.localized_capture_stamp = None
        self.localization_status = None
        self.localization_fault = None
        self.control_stream_id = None
        self.control_odom_status = None
        self.last_control_mode=None
        self.last_control_pose = None
        self.heading_source = 'amcl_odom_ekf' if self.localization else 'odom_ekf'
        self.phases = json.loads(plan.phases_json)
        self.follow = FixedPlanFollower(self.phases, self.target, self.geometry,
                                        approach_only=arrive_at_c,
                                        continuous_insertion=pickup)
        self.pose = None
        self.twist = None
        self.odom_wall = 0.
        self.odom_capture_stamp = None
        self.scan_wall = {'lidar': 0.}
        self.lidar_stable_since = 0.
        self.monitor_stop = False
        self.yolo_enabled = None
        self.yolo_wall = 0.
        self.started = False
        self.armed = False
        self.armed_at = None
        self.finished = False
        self.state = 'WAIT_SENSORS'
        self.reason = ''
        self.rows = []
        self.diagnostics = OdomImuDiagnostics()
        self.last_command = (0., 0.)
        self.last_tick = time.monotonic()
        self.prev_pose = None
        self.last_progress_pose = None
        self.last_progress_wall = time.monotonic()
        self.braking_since = None
        self.sensor_hold_since = None
        self.sensor_hold_reason = ''
        self.sensor_holds = deque(maxlen=64)
        self.c_rgbd_samples = deque(maxlen=20)
        self.c_wait_started = None
        self.c_verified = None
        self.c_rgbd_report = None
        self.c_arrival_report = None
        self.c_last_issue = 'no fresh pallet RGB-D at C'
        self.c_braking_since = None
        self.input_revision = 0
        self.input_drain_callbacks = 0
        self.input_drain_duration_s = 0.
        self.last_publish_duration_s = 0.
        self.last_tick_started = None
        self.publisher = self.create_publisher(Twist, '/pallet/r3/cmd_vel',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       lifespan=Duration(seconds=ODOM_TIMEOUT_S)))
        self.insertion_request = (self.create_publisher(String,
            '/pallet/r3/control_odom/insertion_request',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
            if pickup and self.localization else None)
        latest_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscribe_input(Odometry, '/odom', self.on_odom, latest_sensor)
        if self.localization:
            self.subscribe_input(Odometry, '/pallet/r3/localized_odom',
                                 self.on_localized_odom, latest_sensor)
            status_topic=self.localization.get('control_status_topic','/pallet/r3/localization/status')
            self.subscribe_input(String, status_topic, self.on_localization,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            if status_topic!='/pallet/r3/control_odom/status':
                self.subscribe_input(String, '/pallet/r3/control_odom/status',self.on_control_odom_status,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscribe_input(Odometry, '/base_controller/odom', self.on_wheel_odom, latest_sensor)
        self.subscribe_input(Float64MultiArray, '/wheel_pid_telemetry',
            lambda msg: self.diagnostics.on_wheel_pid(msg, time.monotonic()), latest_sensor)
        self.create_subscription(Imu, '/imu',
            lambda msg: self.diagnostics.on_imu(msg, time.monotonic()), latest_sensor)
        self.subscribe_input(LaserScan, '/pallet/r3/filtered_lidar_scan',
                             lambda _: self.on_scan('lidar'), latest_sensor)
        self.subscribe_input(CollisionMonitorState, '/pallet/r3/collision_state',
                             self.on_collision_state, 1)
        self.subscribe_input(Bool, '/pallet/enabled', self.on_yolo_enabled, 1)
        self.subscribe_input(PalletTarget, '/pallet/target', self.on_target, 1)
        if control_timer:
            self.create_timer(CONTROL_PERIOD_S, self.tick)

    def subscribe_input(self, message_type, topic, callback, qos):
        def receive(message):
            self.input_revision += 1
            callback(message)
        return self.create_subscription(message_type, topic, receive, qos)

    def on_localization(self, msg):
        try:
            value = json.loads(msg.data)
            self.observe_control_stream(value)
            self.localization_status = value
            tracking_localization_status(value, self.get_clock().now().nanoseconds*1e-9,
                                         self.localization)
            if self.localization_status.get('control_status_topic')=='/pallet/r3/control_odom/status':
                self.control_odom_status=self.localization_status
                mode=self.localization_status.get('control_mode')
                if mode!=self.last_control_mode:
                    self.last_control_mode=mode
                    if mode=='ekf_tracking':
                        self.get_logger().info('AMCL đang cập nhật chậm/chưa hợp lệ; tiếp tục bám đường bằng EKF trong mốc đã khóa.')
                    elif mode=='amcl_corrected_ekf':
                        self.get_logger().info('Đã nhận hiệu chỉnh AMCL hợp lệ; tiếp tục kết hợp với EKF.')
                    elif mode=='insertion_locked_ekf':
                        self.get_logger().info('Đã giữ mốc chèn C→B; odom giữ hướng thẳng, AMCL tiếp tục hiển thị trên RViz.')
        except LocalizationPending:
            # Preserve the real status and its original timestamp. The next
            # tick holds zero until both status and odometry are fresh again.
            return
        except (ValueError, TypeError, KeyError) as exc:
            self.localization_fault = str(exc)
            self.localization_status = None
            if self.started:
                self.fail(str(exc))

    def on_localized_odom(self, msg):
        self.diagnostics.on_odom('localized_odom',msg,time.monotonic())
        p,q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        if (msg.header.frame_id != 'r3_trial' or msg.child_frame_id != 'base_footprint'
                or not -.05 <= now-stamp <= ODOM_TIMEOUT_S
                or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .02):
            return
        self.localized_pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.localized_wall = time.monotonic()
        self.localized_capture_stamp = stamp

    def on_control_odom_status(self,msg):
        try:
            value=json.loads(msg.data)
            self.observe_control_stream(value)
            self.control_odom_status=value
            tracking_localization_status(value,self.get_clock().now().nanoseconds*1e-9,
                                         self.localization)
        except LocalizationPending:
            return
        except (ValueError,TypeError,KeyError) as exc:
            self.localization_fault=str(exc)
            self.control_odom_status=None
            if self.started:
                self.fail(str(exc))

    def observe_control_stream(self, value):
        if not isinstance(value, dict):
            raise ValueError('invalid localization control status')
        stream = value.get('control_stream_id')
        if self.control_stream_id is not None and stream != self.control_stream_id:
            raise ValueError('control odometry process restarted; recognize again')
        if stream is not None:
            if not isinstance(stream, str) or not stream:
                raise ValueError('invalid control odometry process identity')
            self.control_stream_id = stream

    def ensure_insertion_frame(self, pose, wall, now_ros):
        """Confirm a shared held frame before either tine reaches C."""
        if not self.pickup or not self.localization:
            return True
        c = self.follow.c_join_pose
        to_c = (c[0]-pose[0])*math.cos(c[2])+(c[1]-pose[1])*math.sin(c[2])
        if to_c > .40 and self.insertion_lock_requested_at is None:
            return True
        state = tracking_localization_status(self.control_odom_status, now_ros, self.localization)
        lock = state.get('insertion_lock') or {}
        matching = (lock.get('ready') is True and lock.get('plan_id') == self.plan.plan_id
                    and lock.get('run_id') == self.localization['run_id']
                    and lock.get('map_file') == self.localization['map_file']
                    and state.get('control_mode') == 'insertion_locked_ekf')
        if matching:
            correction = lock.get('correction_xyyaw')
            if (not isinstance(correction, list) or len(correction) != 3
                    or not all(math.isfinite(v) for v in correction)
                    or correction != state['correction_xyyaw']):
                raise ValueError('insertion frame acknowledgement has an inconsistent correction')
            capture = self.localization['correction_at_capture_xyyaw']
            if (math.dist(correction[:2], capture[:2]) > .0005
                    or abs(wrap(correction[2]-capture[2])) > math.radians(.05)):
                raise ValueError('insertion frame did not settle to the pallet capture reference')
            if self.insertion_frame_lock is not None and lock != self.insertion_frame_lock:
                raise ValueError('insertion frame changed after confirmation')
            self.insertion_frame_lock = dict(lock)
            self.heading_source = 'insertion_locked_ekf'
            if self.insertion_lock_wait_since is not None:
                self.last_tick = self.last_progress_wall = wall
                self.insertion_lock_wait_since = None
            return True
        if self.insertion_frame_lock is not None:
            raise ValueError('confirmed insertion frame was lost; restart and recognize again')
        if self.insertion_lock_requested_at is None:
            self.insertion_lock_requested_at = wall
        if self.insertion_lock_last_sent is None or wall-self.insertion_lock_last_sent >= .2:
            request = dict(action='lock_for_insertion', plan_id=self.plan.plan_id,
                published_ros_s=now_ros,
                capture_correction_xyyaw=list(self.localization['correction_at_capture_xyyaw']),
                **{key: self.localization[key]
                    for key in ('run_id', 'map_file', 'baseline_map_odom')})
            self.insertion_request.publish(String(data=json.dumps(request)))
            self.insertion_lock_last_sent = wall
        # Leave enough distance for the drive's command delay and braking
        # if the shared-frame acknowledgement never arrives.
        if to_c > .12:
            return True
        self.send()
        self.follow.reset_motion()
        self.last_tick = self.last_progress_wall = wall
        self.state = 'WAIT_INSERTION_FRAME'
        if self.insertion_lock_wait_since is None:
            self.insertion_lock_wait_since = wall
        if wall-self.insertion_lock_wait_since >= 2.:
            raise ValueError('insertion frame acknowledgement timed out: '
                             + str(state.get('insertion_lock_issue') or 'no matching acknowledgement'))
        return False


    def on_odom(self, msg):
        self.diagnostics.on_odom('fused_odom', msg, time.monotonic())
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (msg.header.frame_id != 'odom'
                or msg.child_frame_id not in ('base_footprint', 'base_link')
                or not all(math.isfinite(v) for v in (p.x, p.y, norm, *twist))
                or abs(norm - 1.) > .02):
            self.fail('invalid /odom frame, pose or velocity')
            return
        # A delayed sample cannot steer the robot. Leave the last valid pose
        # untouched so tick() commands zero until fresh odom arrives.
        if not -.05 <= now - stamp <= ODOM_TIMEOUT_S:
            return
        if self.odom_capture_stamp is not None and stamp <= self.odom_capture_stamp:
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        pose = (p.x, p.y, yaw)
        if self.prev_pose and (math.dist(pose[:2], self.prev_pose[:2]) > .10
                               or abs(wrap(pose[2] - self.prev_pose[2])) > .15):
            self.fail('odom jumped or reset')
            return
        self.prev_pose, self.pose, self.odom_wall = pose, pose, time.monotonic()
        self.odom_capture_stamp = stamp
        self.twist = twist

    def on_wheel_odom(self, msg):
        # Encoder odom verifies that the base stays parked while lifting.
        # The drive follower still uses the EKF/localized pose.
        received = time.monotonic()
        self.diagnostics.on_odom('wheel_odom', msg, received)
        if self.lift_wheels is not None:
            self.lift_wheels.update(msg, received, self.get_clock().now().nanoseconds*1e-9)

    def on_scan(self, name):
        now = time.monotonic()
        if name == 'lidar' and (self.scan_wall[name] == 0.
                                or now - self.scan_wall[name] > LIDAR_START_FRESH_S):
            self.lidar_stable_since = now
        self.scan_wall[name] = now

    def on_collision_state(self, msg):
        if self.finished:
            return
        if msg.action_type == CollisionMonitorState.DO_NOTHING:
            if not self.started or self.state == 'LIFTING':
                self.monitor_stop = False
            return
        self.monitor_stop = True
        if msg.polygon_name == 'invalid source' and self.state == 'LIFTING':
            # The drive remains stopped. A scan timestamp/TF diagnostic can
            # inhibit the wheels without interrupting encoder-guarded lift motion.
            self.lift_lidar_source_warnings += 1
            if self.lift_lidar_source_warnings == 1:
                self.get_logger().warn('LiDAR/TF tạm thiếu dữ liệu khi nâng tại B; giữ bánh đứng yên.')
            return
        # A real obstacle remains latched. The drive phase also latches loss
        # of a collision source once it has moved from the start pose.
        if msg.polygon_name != 'invalid source' or self.started:
            self.fail('collision monitor stopped: ' + msg.polygon_name)

    def on_yolo_enabled(self, msg):
        self.yolo_enabled = bool(msg.data)
        self.yolo_wall = time.monotonic()
        if self.yolo_enabled and self.started:
            self.fail('YOLO was re-enabled during odom-controlled motion')

    def on_target(self, msg):
        if self.follow.state != 'WAIT_C_RGBD':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        now_ros = self.get_clock().now().nanoseconds*1e-9
        if (not msg.valid or msg.header.frame_id != 'camera_color_optical_frame'
                or not -.05 <= now_ros-stamp <= .30
                or not math.isfinite(msg.rgb_depth_delta_ms)
                or abs(msg.rgb_depth_delta_ms) > 50.
                or min(msg.left_depth_samples, msg.right_depth_samples) < 60
                or 'heading_source=floor_referenced_plane' not in msg.status
                or not all(math.isfinite(v) for v in
                           (msg.entry_heading_rad, msg.opening_lateral_m,
                            msg.chassis_front_forward_m))):
            return
        if self.c_rgbd_samples and stamp <= self.c_rgbd_samples[-1]['stamp_s']:
            return
        self.c_rgbd_samples.append(dict(stamp_s=stamp, received_wall=time.monotonic(),
            heading_rad=msg.entry_heading_rad, lateral_m=msg.opening_lateral_m,
            gap_m=msg.chassis_front_forward_m-.065))

    def send(self, v=0., w=0.):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        started = time.monotonic()
        self.publisher.publish(cmd)
        self.last_publish_duration_s = time.monotonic()-started
        self.last_command = (v, w)

    def record_pose(self, control_pose, v=0., w=0.):
        e = errors(control_pose, self.target, self.geometry)
        fork = fork_for_base(control_pose, self.geometry)
        self.rows.append(dict(wall_s=time.time(), state=self.state,
            x_m=control_pose[0], y_m=control_pose[1], yaw_rad=control_pose[2],
            raw_odom_x_m=self.pose[0], raw_odom_y_m=self.pose[1], raw_odom_yaw_rad=self.pose[2],
            control_yaw_rad=control_pose[2], heading_source=self.heading_source,
            insertion_heading_target_rad=self.follow.insertion_heading,
            insertion_heading_error_deg=(self.follow.insertion_heading_pid.report or {}).get('yaw_error_deg'),
            insertion_yaw_rate_rps=(self.follow.insertion_heading_pid.report or {}).get('yaw_rate_rps'),
            insertion_requested_omega_rps=(self.follow.insertion_heading_pid.report or {}).get('requested_omega_rps'),
            insertion_rate_integral_rps=self.follow.insertion_heading_pid.rate_integral,
            fork_x_m=fork[0], fork_y_m=fork[1], fork_yaw_rad=fork[2],
            cmd_v_mps=v, cmd_w_rps=w, **e))

    def fail(self, reason):
        if not self.finished:
            self.state, self.reason, self.finished = 'STOP_LATCHED', reason, True
        self.send()
        if self.lift is not None:
            self.lift.stop()

    def hold_for_sensors(self, missing, wall, lidar_age):
        self.send()
        self.sensor_hold_reason = ', '.join(missing)
        if self.started:
            if self.sensor_hold_since is None:
                self.sensor_hold_since = wall
                self.sensor_holds.append(dict(started_monotonic_s=wall,
                    reason=self.sensor_hold_reason, resumed=False))
                self.get_logger().warn('Tạm giữ vận tốc 0 để chờ dữ liệu mới: '
                                       + self.sensor_hold_reason)
            if lidar_age > LIDAR_TIMEOUT_S or wall-self.sensor_hold_since >= SENSOR_HOLD_LATCH_S:
                self.fail('sensor unavailable during motion: '+self.sensor_hold_reason)
        elif self.armed_at is not None and wall-self.armed_at > 10.:
            self.fail('sensor unavailable at start: '+self.sensor_hold_reason)

    def tick(self):
        started = time.monotonic()
        interval = started-self.last_tick_started if self.last_tick_started is not None else None
        self.last_tick_started = started
        try:
            self.control_tick()
        finally:
            sampled = self.diagnostics.sample(time.monotonic(), time.time(), self.state,
                                              self.last_control_pose or self.pose, self.last_command,
                                              self.heading_source)
            if sampled:
                self.diagnostics.rows[-1]['control_loop'] = dict(
                    tick_interval_s=interval,
                    tick_duration_s=time.monotonic()-started,
                    publish_duration_s=self.last_publish_duration_s,
                    input_drain_callbacks=self.input_drain_callbacks,
                    input_drain_duration_s=self.input_drain_duration_s)
                self.diagnostics.rows[-1]['sensor_hold'] = dict(
                    active=self.sensor_hold_since is not None, reason=self.sensor_hold_reason,
                    duration_s=(time.monotonic()-self.sensor_hold_since
                                if self.sensor_hold_since is not None else 0.))
                self.diagnostics.rows[-1]['longitudinal_control'] = dict(
                    self.follow.speed_control.report or {},
                    braking_overspeed_reserve_m=self.follow.braking_overspeed_reserve_m)
            if self.localization and sampled:
                self.diagnostics.rows[-1]['localization'] = self.localization_status
                self.diagnostics.rows[-1]['control_odom_status'] = self.control_odom_status
                self.diagnostics.rows[-1]['localized_odom_receive_age_s']=(
                    time.monotonic()-self.localized_wall if self.localized_wall else None)

    def tick_parked_lift(self, wall, now_ros):
        self.send()
        wheels = self.lift_wheels
        try:
            if wheels.anchor is None or wheels.age(wall, now_ros) > LIFT_WHEEL_ODOM_MAX_AGE_S:
                age = now_ros-wheels.stamp if wheels.stamp is not None else float('inf')
                raise ValueError('fresh wheel encoder odom unavailable while lifting at B '
                                 f'(source age {age:.3f}s, receive age {wall-wheels.received:.3f}s)')
            displacement, turn = wheels.difference(wheels.pose, wheels.anchor)
            if (displacement > .005 or turn > math.radians(.5)
                    or abs(wheels.twist[0]) >= .005 or abs(wheels.twist[1]) >= .02):
                raise ValueError('base moved while lifting at B')
            if wall >= self.lift_next_graph_check:
                if any(names != [owner] for _, _, owner, names in command_graph_status(self)):
                    raise ValueError('wheel command ownership changed while lifting at B')
                self.lift.check_owners()
                self.lift_next_graph_check = wall+LIFT_GRAPH_CHECK_INTERVAL_S
            done = self.lift.raise_tick()
            self.lift_history.append(dict(wall_s=time.time(), **self.lift.report()))
        except ValueError as exc:
            self.fail(str(exc))
            return
        if done:
            self.state, self.finished = 'PICKUP_COMPLETE', True
            self.lift.stop()

    def control_tick(self):
        wall = time.monotonic()
        if not self.armed:
            self.send()
            return
        if self.finished:
            self.send()
            return
        now_ros = self.get_clock().now().nanoseconds*1e-9
        if self.state == 'LIFTING':
            self.tick_parked_lift(wall, now_ros)
            return
        if self.monitor_stop and self.started:
            self.fail('obstacle stop')
            return
        missing = []
        if self.localization:
            if self.localization_fault is not None:
                self.fail(self.localization_fault)
                return
            try:
                tracking_localization_status(self.localization_status, now_ros, self.localization)
            except LocalizationPending as exc:
                missing.append(str(exc))
            except (ValueError, TypeError, KeyError) as exc:
                self.fail(str(exc))
                return
            localized_age = sensor_age(wall-self.localized_wall,
                                       self.localized_capture_stamp, now_ros)
            if self.localized_pose is None or localized_age > ODOM_TIMEOUT_S:
                missing.append('localized odom unavailable '
                    f'(last valid receive age {wall-self.localized_wall:.3f}s; '
                    f'source age {localized_age:.3f}s; '
                    f'EKF receive age {wall-self.odom_wall:.3f}s; limit {ODOM_TIMEOUT_S:.2f}s)')
        control_pose = self.localized_pose if self.localization else self.pose
        self.last_control_pose = control_pose
        odom_age = sensor_age(wall-self.odom_wall, self.odom_capture_stamp, now_ros)
        yolo_age = wall - self.yolo_wall
        lidar_age = wall - self.scan_wall['lidar']
        if self.pose is None or odom_age > ODOM_TIMEOUT_S:
            missing.append(f'odom={odom_age:.2f}s')
        if self.yolo_enabled is not False or yolo_age > YOLO_HEARTBEAT_TIMEOUT_S:
            missing.append(f'YOLO-off={yolo_age:.2f}s, enabled={self.yolo_enabled}')
        if lidar_age > LIDAR_TIMEOUT_S:
            missing.append(f'LiDAR={lidar_age:.2f}s')
        if missing:
            self.hold_for_sensors(missing, wall, lidar_age)
            return
        if self.sensor_hold_since is not None:
            if wall-self.sensor_hold_since >= SENSOR_HOLD_LATCH_S:
                self.fail('sensor recovery exceeded 1 second: '+self.sensor_hold_reason)
                return
            # The last physical command during a sensor gap was zero. Resume
            # the profile from zero after all sources become fresh again.
            self.follow.reset_motion()
            self.last_tick = self.last_progress_wall = wall
            if self.sensor_holds:
                self.sensor_holds[-1].update(resumed=True,
                    duration_s=wall-self.sensor_hold_since)
            self.get_logger().info('Dữ liệu mới đã phục hồi; tiếp tục từ vận tốc 0.')
            self.sensor_hold_since = None
            self.sensor_hold_reason = ''
        if self.pickup:
            try:
                self.lift.check_owners()
                self.lift.at_home()
            except ValueError as exc:
                self.send()
                if self.started or (self.armed_at is not None and wall-self.armed_at > 10.):
                    self.fail(str(exc))
                return
        if not self.started:
            if self.localization and self.localization_status.get('correction_settled') is not True:
                self.send()
                if self.armed_at is not None and wall-self.armed_at > 10.:
                    self.fail('AMCL correction did not settle at start; recognize again')
                return
            if (self.monitor_stop or wall - self.scan_wall['lidar'] > LIDAR_START_FRESH_S
                    or wall - self.lidar_stable_since < LIDAR_START_STABLE_S):
                self.send()
                return
            start = self.phases[0]['poses'][0]
            if (math.dist(control_pose[:2], start[:2]) > .02
                    or abs(wrap(control_pose[2] - start[2])) > math.radians(.5)):
                self.fail('current odom does not match frozen start pose')
                return
            self.started = True
            self.follow.start(True)
            self.state = self.follow.state
            self.last_progress_pose, self.last_progress_wall = self.pose, wall
        capture_correction = self.localization['correction_at_capture_xyyaw'] if self.localization else (0.,0.,0.)
        raw_reference_pose = compose(capture_correction, self.pose)
        try:
            if not self.ensure_insertion_frame(control_pose, wall, now_ros):
                self.record_pose(control_pose)
                return
        except LocalizationPending as exc:
            self.hold_for_sensors([str(exc)], time.monotonic(), lidar_age)
            return
        except (ValueError, TypeError, KeyError) as exc:
            self.fail(str(exc))
            return
        if self.braking_since is not None:
            self.send()
            wheel_parked = (not self.pickup or (self.lift_wheels.ready(wall, now_ros)
                and abs(self.lift_wheels.twist[0]) < .005 and abs(self.lift_wheels.twist[1]) < .02))
            if wall - self.braking_since >= .5:
                e = errors(control_pose, self.target, self.geometry)
                still = (self.twist and abs(self.twist[0]) < .005 and abs(self.twist[1]) < .005
                         and wheel_parked
                         and (not self.localization or self.localization_status.get('correction_settled') is True))
                if self.pickup and not still and wall-self.braking_since < 2.:
                    return
                aligned = (abs(e['ey_m']) <= self.geometry['max_lateral']
                           and abs(math.degrees(e['yaw_error_rad'])) <= self.geometry['max_yaw_deg'])
                if self.pickup:
                    self.insertion_report = insertion_clearance(
                        control_pose, self.target, self.geometry, project_to_b=False)
                    aligned = self.insertion_report['ready']
                if (not still or abs(e['gap_m'] - self.geometry['stop_gap']) > .005 or not aligned):
                    detail = ''
                    if self.pickup:
                        detail = (f': B gap={e["gap_m"]*100:.2f} cm, '
                                  f'fork lateral={e["ey_m"]*100:+.2f} cm, '
                                  f'yaw={math.degrees(e["yaw_error_rad"]):+.2f} deg; '
                                  + self.insertion_report['reason'])
                    self.fail('after-braking odom goal criteria failed'+detail)
                    if self.pickup:
                        self.record_pose(control_pose)
                else:
                    if self.pickup:
                        try:
                            self.lift.begin_raise(self.geometry['lift_target'])
                        except ValueError as exc:
                            self.fail(str(exc))
                            return
                        self.state = 'LIFTING'
                        self.lift_base_pose = tuple(self.pose)
                        self.lift_wheels.arm()
                        # Include the actual settled B pose in the trajectory,
                        # rather than ending its CSV at the first brake command.
                        self.record_pose(control_pose)
                        self.get_logger().info('Đã dừng tại B; tự nâng càng lên 8 cm.')
                    else:
                        self.state, self.finished = 'DONE_TRACKING', True
            return
        dt = max(.001, min(.10, wall - self.last_tick))
        self.last_tick = wall
        camera_verified = self.c_verified is not None
        if (self.stop_at_c or self.arrive_at_c) and self.c_braking_since is not None:
            self.send()
            e = errors(control_pose, self.target, self.geometry)
            still = (self.twist and abs(self.twist[0]) < .005 and abs(self.twist[1]) < .005
                     and (self.arrive_at_c or not self.localization
                          or self.localization_status.get('correction_settled') is True))
            clearance = e['gap_m']-self.geometry['fork_length']
            settling_time = wall-self.c_braking_since
            report_needed = (clearance < self.follow.min_straight_entry_clearance_m
                             or (not still and settling_time >= 2.0)
                             or (still and settling_time >= .5))
            if report_needed:
                print(('KẾT QUẢ TẠI C: ' if self.arrive_at_c else 'KẾT QUẢ DỪNG TRƯỚC C: ') + json.dumps(dict(
                    control_yaw_error_deg=round(math.degrees(e['yaw_error_rad']), 2),
                    odom_yaw_error_deg=round(math.degrees(
                        errors(raw_reference_pose, self.target, self.geometry)['yaw_error_rad']), 2),
                    estimated_lateral_cm=round(e['ey_m']*100, 2),
                    fork_tip_clearance_cm=round(clearance*100, 1),
                    stationary=bool(still)), ensure_ascii=False), flush=True)
            if clearance < self.follow.min_straight_entry_clearance_m:
                self.fail('fork-tip clearance lost while braking before C')
            elif not still and settling_time >= 2.0:
                self.fail('robot did not settle within 2 s after braking before C')
            elif still and settling_time >= .5:
                if self.arrive_at_c:
                    end = self.phases[0]['poses'][-1]
                    xy_error = math.dist(control_pose[:2], end[:2])
                    yaw_error = abs(math.degrees(wrap(control_pose[2]-end[2])))
                    self.c_arrival_report = dict(position_error_m=xy_error,
                        heading_error_deg=yaw_error, fork_tip_clearance_m=clearance,
                        position_tolerance_m=self.follow.c_position_tolerance_m,
                        heading_tolerance_deg=self.follow.c_heading_tolerance_deg,
                        amcl_correction_settled=(self.localization_status or {}).get('correction_settled'),
                        insertion_alignment_satisfied=(abs(e['ey_m']) <= self.geometry['max_lateral']
                            and abs(math.degrees(e['yaw_error_rad'])) <= self.geometry['max_yaw_deg']),
                        insertion_authorized=False, physical_alignment_verified=False)
                    if (xy_error > self.follow.c_position_tolerance_m
                            or yaw_error > self.follow.c_heading_tolerance_deg):
                        # Finish the approach without another correction loop
                        # or cancellation. Keep the remaining error visible;
                        # this status is not a claim of qualified C alignment.
                        self.state, self.finished = 'C_ALIGNMENT_WARNING', True
                    else:
                        self.state, self.finished = 'C_ARRIVED', True
                elif (abs(e['ey_m']) > self.geometry['max_lateral']
                        or abs(math.degrees(e['yaw_error_rad'])) > self.geometry['max_yaw_deg']):
                    self.fail('odom alignment outside C diagnostic limits')
                else:
                    self.state, self.finished = 'C_DIAGNOSTIC_DONE', True
            self.record_pose(control_pose)
            return
        if self.follow.state == 'WAIT_C_RGBD' and not camera_verified:
            if self.c_wait_started is None:
                self.c_wait_started = wall
            if self.twist and (abs(self.twist[0]) > .005 or abs(self.twist[1]) > .02):
                self.c_rgbd_samples.clear()
                self.c_last_issue = 'robot still moving at C'
            else:
                odom_error = errors(control_pose, self.target, self.geometry)
                camera_result, self.c_last_issue = check_c_rgbd(
                    self.c_rgbd_samples, odom_error['gap_m'],
                    odom_error['yaw_error_rad'], wall)
                if camera_result is not None:
                    self.c_rgbd_report = camera_result
                    print('RGB-D ĐO LẠI TẠI C: ' + json.dumps(camera_result,
                          ensure_ascii=False), flush=True)
                    if self.c_last_issue:
                        self.fail('C RGB-D verification failed: ' + self.c_last_issue)
                        return
                    self.c_verified = camera_result
                    camera_verified = True
                    if self.stop_at_c:
                        self.state, self.finished = 'C_DIAGNOSTIC_DONE', True
                        self.send()
                        return
            if not camera_verified and wall-self.c_wait_started > C_RGBD_WAIT_LIMIT_S:
                self.fail('C RGB-D verification failed: ' + self.c_last_issue)
                return
        clearance_hold = self.follow.state == 'CLEARANCE_HOLD'
        wheels_stopped = (clearance_hold and self.lift_wheels.ready(wall, now_ros)
                          and abs(self.lift_wheels.twist[0]) < .005 and abs(self.lift_wheels.twist[1]) < .01)
        v, w = self.follow.step(control_pose, dt, obstacle=self.monitor_stop,
                                c_verified=camera_verified,
                                insertion_yaw=raw_reference_pose[2],
                                insertion_yaw_rate=self.twist[1],
                                insertion_pose=raw_reference_pose,
                                measured_speed=self.twist[0], wheels_stopped=wheels_stopped)
        if self.follow.state == 'STOP_LATCHED':
            if self.pickup:
                self.insertion_report = self.follow.insertion_report
            self.fail(self.follow.reason)
            if self.pickup:
                self.record_pose(control_pose)
            return
        if self.follow.state == 'CLEARANCE_HOLD' and not clearance_hold:
            self.get_logger().warn('Tạm phanh ở C→B để kiểm tra lại khe hở càng sau khi bánh dừng.')
        elif clearance_hold and self.follow.state == 'STRAIGHT':
            self.get_logger().info('Đã xác nhận bánh đứng yên và khe hở đạt. Tiếp tục đến B từ vận tốc 0.')
            self.last_progress_pose, self.last_progress_wall = control_pose, wall
        if self.pickup and self.follow.insertion_started and not self.insertion_logged:
            self.insertion_logged = True
            self.get_logger().info('Tiếp tục C→B, bám góc 0° theo trục pallet bằng odom gắn mốc A; '
                'khe hở ngang ước lượng tại C '
                f'{self.follow.insertion_report["minimum_clearance_m"]*1000:.1f} mm '
                '(đã kiểm tra cả hai càng trong quãng lệnh và phanh). '
                f'Góc odom tại C: {math.degrees(wrap(raw_reference_pose[2]-self.target[2])):+.2f}°.')
        if ((self.stop_at_c and self.follow.state == 'C_SETTLE')
                or (self.arrive_at_c and self.follow.state == 'C_REACHED')):
            self.c_braking_since = wall
            self.state = 'C_BRAKING'
            self.send()
            v = w = 0.
        elif self.follow.state == 'STOPPED':
            self.state, self.braking_since = 'BRAKING', wall
            self.send()
        else:
            if self.last_progress_pose and (
                    math.dist(control_pose[:2], self.last_progress_pose[:2]) > .005
                    or abs(wrap(control_pose[2] - self.last_progress_pose[2])) > math.radians(.3)):
                self.last_progress_pose, self.last_progress_wall = control_pose, wall
            if (v > .01 or abs(w) > .03 or self.follow.state == 'ALIGN_YAW') \
                    and wall - self.last_progress_wall > 3.:
                self.fail('no odom progress while commanded')
                return
            self.state = self.follow.state
            self.send(v, w)
        self.record_pose(control_pose, v, w)


def command_graph_status(probe):
    expected = (
        ('publisher', '/cmd_vel_safe', probe.get_publishers_info_by_topic,
         '/r3_collision_monitor'),
        ('consumer', '/cmd_vel_safe', probe.get_subscriptions_info_by_topic,
         '/cmd_vel_stamper'),
        ('publisher', '/base_controller/cmd_vel', probe.get_publishers_info_by_topic,
         '/cmd_vel_stamper'),
    )
    observed = []
    for role, topic, lookup, owner in expected:
        names = [f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                 for info in lookup(topic)]
        observed.append((role, topic, owner, names))
    return observed


def trial_command_consumers(node):
    return [(f'{info.node_namespace.rstrip("/")}/{info.node_name}', info.topic_type)
            for info in node.get_subscriptions_info_by_topic('/pallet/r3/cmd_vel')]


def monitor_ready():
    probe = Node('r3_check_collision_monitor')
    try:
        client = probe.create_client(GetState, '/r3_collision_monitor/get_state')
        if not client.wait_for_service(timeout_sec=20.):
            services = [name for name, _ in probe.get_service_names_and_types()
                        if 'collision_monitor' in name]
            nodes = [f'{namespace.rstrip("/")}/{name}'
                     for name, namespace in probe.get_node_names_and_namespaces()
                     if 'collision_monitor' in name]
            raise RuntimeError('r3 collision monitor lifecycle service missing after 20 s; '
                               f'discovered nodes={nodes}; services={services}')
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(probe, future, timeout_sec=3.)
        if future.result() is None or future.result().current_state.label != 'active':
            raise RuntimeError('r3 collision monitor is not active')
        deadline = time.monotonic() + 10.
        while True:
            observed = command_graph_status(probe)
            if all(names == [owner] for _, _, owner, names in observed):
                break
            if time.monotonic() >= deadline:
                detail = '; '.join(f'{role} {topic}: expected {owner}, found {names}'
                                   for role, topic, owner, names in observed)
                raise RuntimeError('safe command graph not ready: ' + detail)
            rclpy.spin_once(probe, timeout_sec=.1)
    finally:
        probe.destroy_node()


def main():
    configure_command_transport()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute', action='store_true', help='Explicitly start real motion after guards pass')
    parser.add_argument('--allow-provisional', action='store_true',
                        help='Use a plan up to 0.10 m/s with explicit URDF/user geometry assumptions')
    parser.add_argument('--stop-at-c', action='store_true',
                        help='Brake before C without a second camera measurement or straight insertion')
    parser.add_argument('--arrive-at-c', action='store_true',
                        help='Complete APPROACH to C and stop there; never enter C-to-B')
    parser.add_argument('--pickup', action='store_true',
                        help='Track continuously through C to B, stop wheels, then lift 0.08 m')
    parser.add_argument('--deliver-after-pickup', action='store_true',
                        help='Include the automatic delivery leg in the pickup confirmation')
    args = parser.parse_args()
    if sum((args.stop_at_c, args.arrive_at_c, args.pickup)) > 1:
        parser.error('choose only one of --stop-at-c, --arrive-at-c and --pickup')
    if args.deliver_after_pickup and not args.pickup:
        parser.error('--deliver-after-pickup requires --pickup')
    target, geometry = read_real_target(args.plan, args.allow_provisional)
    data = json.loads(args.plan.read_text())
    plan = FrozenPlan(**{key: data[key] for key in
                         ('target_json', 'geometry_json', 'phases_json', 'config_json')})
    report = target['measurement_report']
    phases = json.loads(plan.phases_json)
    start = phases[0]['poses'][0]
    initial = errors(start, target['pose'], geometry)
    c_clearance = errors(phases[1]['poses'][0], target['pose'], geometry)['gap_m'] - geometry['fork_length']
    print('\nVỊ TRÍ PALLET TRƯỚC KHI CHẠY', flush=True)
    print(f"  Camera đo ngang: {report['camera_lateral_raw_m'] * 100:+.1f} cm", flush=True)
    print(f"  Bù camera sang trái: {report['camera_mount_offset_m'] * 100:+.1f} cm", flush=True)
    print(f"  Pallet ngang sau bù vị trí lắp camera: {report['fork_lateral_corrected_m'] * 100:+.1f} cm", flush=True)
    residual = report.get('pallet_axis_lateral_residual_m', 0.)
    print(f"  Bù sai số ngang thực nghiệm theo trục pallet: {residual*100:+.1f} cm "
          '(dương về phía trái hướng vào pallet)', flush=True)
    final_lateral = report.get('target_lateral_after_axis_residual_m', report['fork_lateral_corrected_m'])
    print(f"  Tâm pallet so với tâm gốc càng sau toàn bộ bù: {final_lateral*100:+.1f} cm", flush=True)
    print(f"  Khoảng cách gốc càng tới mặt pallet: {initial['gap_m']:.3f} m", flush=True)
    print(f"  Tại C, đầu càng còn cách mặt pallet: {c_clearance * 100:.1f} cm", flush=True)
    print(f"  Góc hướng xe so với hướng vào pallet: {math.degrees(initial['yaw_error_rad']):+.2f}°", flush=True)
    print(f"  Nguồn góc: {report['heading_source']} "
          f"(độ dốc depth X={report['heading_depth_x_slope']:.8f})", flush=True)
    assumptions = report.get('calibration_assumptions') or {}
    if assumptions:
        print('  Hiệu chỉnh depth vị trí/góc: ' +
              ('dùng chung' if assumptions.get('depth_correction_shared') else 'khác nhau (bản cũ)'),
              flush=True)
        print('  Góc lắp camera so với càng chưa hiệu chuẩn; góc pallet vẫn là ước lượng.', flush=True)
    print(f'  Plan ID: {plan.plan_id}', flush=True)
    if target.get('localization'):
        print('  Định vị: AMCL LiDAR + bản đồ. Bộ bám lấy hướng và tốc độ quay từ /odom (EKF).',
              flush=True)
        print('  Bản đồ: ' + target['localization']['map_file'], flush=True)
    else:
        print('  Điều khiển X/Y, góc và tốc độ quay từ /odom (EKF).', flush=True)
    print(f"  Tốc độ tối đa: {geometry['max_speed']:.2f} m/s; giảm tốc theo độ cong và khoảng phanh.",
          flush=True)
    if phases[0].get('tracking_controller') == 'pose_feedback':
        print('  Bám A→C: bù độ cong + phản hồi góc odom và sai lệch ngang theo đường.', flush=True)
    print(f"  Khoảng lệch vuông góc từ tâm gốc càng tới trục vào pallet tại A: "
          f"{initial['ey_m']*100:+.2f} cm.", flush=True)
    print('  Giữ nguyên xe và pallet sau khi chốt kế hoạch; nếu đã kéo hoặc đẩy, hủy để đo lại.',
          flush=True)
    if args.pickup:
        print('LẤY HÀNG: A→C→B liên tục; C không dừng/xoay; tại B dừng bánh rồi nâng 8 cm.', flush=True)
    elif args.arrive_at_c:
        print('ĐÍCH C: chạy hết A→C rồi dừng tại C; không đưa càng vào hoặc nâng pallet.', flush=True)
    elif args.stop_at_c:
        print('CHẨN ĐOÁN: phanh trước C khoảng 5 cm, ghi góc odom cuối; '
              'không chạy C→B, không nâng pallet.', flush=True)
    if not args.execute:
        print(json.dumps({'status': 'READY_TO_CHECK_LIVE_SENSORS', 'plan_id': plan.plan_id,
                          'robot_commands': 0, 'execute_required': True}))
        return
    if not sys.stdin.isatty():
        parser.error('real execution requires an interactive terminal to review the printed measurement')
    try:
        prompt = ('Enter để lấy pallet, tự giao tới D và hạ càng, Ctrl+C để hủy: '
                  if args.deliver_after_pickup else
                  'Kế hoạch đã khóa; Enter để chạy A→C→B và tự nâng 8 cm, Ctrl+C để hủy: '
                  if args.pickup else
                  'Kế hoạch đã khóa; Enter để chạy A→C, Ctrl+C để hủy: '
                  if args.arrive_at_c else
                  'Kế hoạch đã khóa; Enter để chạy chẩn đoán tới gần C, Ctrl+C để hủy: '
                  if args.stop_at_c else
                  'Kế hoạch đã khóa; Enter để kiểm tra cảm biến và chạy, Ctrl+C để hủy: ')
        input(prompt)
    except (EOFError, KeyboardInterrupt):
        print('\nĐã hủy; không phát lệnh xe.')
        return
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('choose a fresh output directory')
    # The operator may review the locked plan for an unlimited time. The
    # unchanged plan hash and the current start odom/safety scans still gate motion.
    read_real_target(args.plan, args.allow_provisional, max_age_s=None)
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = None
    executor = None
    try:
        monitor_ready()
        read_real_target(args.plan, args.allow_provisional, max_age_s=None)
        node = RealTrial(plan, args.output, stop_at_c=args.stop_at_c, control_timer=False,
                         arrive_at_c=args.arrive_at_c, pickup=args.pickup)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        discovery_deadline = time.monotonic() + 20.
        expected_consumer = [('/r3_collision_monitor', 'geometry_msgs/msg/Twist')]
        while True:
            consumers = trial_command_consumers(node)
            if consumers == expected_consumer:
                break
            if time.monotonic() >= discovery_deadline:
                raise RuntimeError('safe collision monitor is not the sole command consumer '
                                   f'after 20 s: expected {expected_consumer}, found {consumers}')
            node.send()
            executor.spin_once(timeout_sec=.1)
        node.armed = True
        node.armed_at = time.monotonic()
        run_control_loop(node, executor)
    except KeyboardInterrupt:
        if node:
            node.fail('operator stopped real trial')
    finally:
        if node:
            for _ in range(10):
                node.send()
                if node.lift is not None:
                    node.lift.stop()
                executor.spin_once(timeout_sec=.05)
            if node.rows:
                write_csv(args.output/'executed_odom_base.csv', node.rows)
            diagnostics_report = node.diagnostics.write(args.output)
            if node.lift_history:
                (args.output/'lift_feedback.jsonl').write_text(''.join(
                    json.dumps(row)+'\n' for row in node.lift_history))
            (args.output/'mission_result.json').write_text(json.dumps({
                'status': node.state, 'reason': node.reason,
                'execution_mode': ('pickup' if node.pickup else
                                   'arrive_at_c' if node.arrive_at_c else
                                   'stop_before_c' if node.stop_at_c else 'full_plan'),
                'plan_id': plan.plan_id, 'odom_samples': len(node.rows),
                'physical_alignment_verified': False,
                'heading_source': node.heading_source,
                'localization': node.localization_status,
                'odom_imu_diagnostics': diagnostics_report,
                'final_odom_yaw_error_deg': (
                    math.degrees(wrap(node.rows[-1]['raw_odom_yaw_rad'] +
                        (node.localization['correction_at_capture_xyyaw'][2] if node.localization else 0.)
                        - target['pose'][2]))
                    if node.rows else None),
                'final_control_yaw_error_deg': (
                    math.degrees(wrap(node.rows[-1]['control_yaw_rad']-target['pose'][2]))
                    if node.rows else None),
                'pallet_remeasured_at_stop': node.c_verified is not None,
                'c_rgbd_verification': node.c_verified,
                'c_rgbd_measurement': node.c_rgbd_report,
                'c_arrival': node.c_arrival_report,
                'lift_commands': node.lift.set_commands if node.lift else 0,
                'lift': node.lift.report() if node.lift else None,
                'base_odom_at_raise_start': node.lift_base_pose,
                'wheel_odom_at_raise_start': node.lift_wheels.anchor if node.lift_wheels else None,
                'lift_lidar_source_warnings': node.lift_lidar_source_warnings,
                'insertion_clearance': node.insertion_report or node.follow.insertion_report,
                'insertion_frame_lock': node.insertion_frame_lock,
                'insertion_camera_frame_clearance': node.follow.insertion_camera_frame_report,
                'insertion_clearance_holds': node.follow.clearance_holds,
                'insertion_heading_rad': node.follow.insertion_heading,
                'longitudinal_control': dict(node.follow.speed_control.report or {},
                    braking_overspeed_reserve_m=node.follow.braking_overspeed_reserve_m),
                'terminal_goal_check': node.follow.terminal_goal_report,
                'insertion_heading_control': dict(
                    feedback_source='camera_at_A_with_fused_odom_yaw_change',
                    rate_feedback_source='fused_odom',
                    **(node.follow.insertion_heading_pid.report or {})) if node.pickup else None,
                'sensor_holds': list(node.sensor_holds),
                'control_stream_id': node.control_stream_id,
                'command_publication_mode': os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE'),
                'command_lifespan_s': ODOM_TIMEOUT_S,
            }, indent=2))
            print(json.dumps(dict(status=node.state, reason=node.reason,
                                  c_arrival=node.c_arrival_report),
                             ensure_ascii=False, indent=2), flush=True)
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
