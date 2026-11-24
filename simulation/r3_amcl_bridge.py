#!/usr/bin/env python3
"""Bounded AMCL correction of continuous EKF odom in one frozen trial frame.

Publishes an Odometry message, never TF or velocity commands. The existing EKF
owns odom->base_footprint and AMCL owns map->odom. The trial frame is map viewed
through the inverse of the map->odom transform frozen before target capture.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from r3_core import compose, inverse, wrap
from r3_scan_map_check import (ScanMapCheck, LIVE_SCAN_MAX_SAMPLES, validate_navigation_scan_match,
                               validate_recognition_scan_match)
from r3_stationary_localization import StationaryEvidence
from r3_amcl_observation_pair import validate_observation_pair
from r3_amcl_quality import concentrated_particle_support,validate_runtime_particles
from r3_wait_localization_active import LifecyclePoll, readiness as lifecycle_readiness


FRAME = 'r3_trial'
AMCL_MAX_AGE_S = 2.0
SCAN_MAX_AGE_S = .5
ODOM_MAX_AGE_S = .2
MOTION_PREDICTION_MAX_S=2.
MOTION_PREDICTION_MAX_M=.08
MOTION_PREDICTION_MAX_YAW_RAD=.08


def planar_pose(position, quaternion):
    values = (position.x, position.y, quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = sum(v*v for v in values[2:])
    if not all(math.isfinite(v) for v in values) or abs(norm-1.) > .02:
        raise ValueError('nonfinite position or invalid orientation quaternion')
    yaw = math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
                     1-2*(quaternion.y*quaternion.y+quaternion.z*quaternion.z))
    return position.x, position.y, yaw


def bounded_correction(current, target, dt, translation_rate=.03, yaw_rate=math.radians(.5)):
    """Limit correction velocity without modifying wheel/IMU odometry."""
    dx, dy = target[0]-current[0], target[1]-current[1]
    distance = math.hypot(dx, dy)
    scale = min(1., translation_rate*max(0., dt)/max(distance, 1e-12))
    dyaw = wrap(target[2]-current[2])
    dyaw = max(-yaw_rate*max(0., dt), min(yaw_rate*max(0., dt), dyaw))
    return current[0]+scale*dx, current[1]+scale*dy, wrap(current[2]+dyaw)


def map_odom_from_capture(map_base, odom_base):
    """Derive a map correction from two base poses at one sensor timestamp.

    AMCL future-dates its broadcast map->odom TF. Using that transform at the
    scan stamp can interpolate an older correction; derive the current one
    from AMCL's map pose and the historical EKF odom TF instead.
    """
    return compose(map_base, inverse(odom_base))


def rotate_covariance(covariance, yaw):
    """Rotate the 6D pose covariance into the corrected world axes."""
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = [[c,-s,0.],[s,c,0.],[0.,0.,1.]]
    jacobian = [[rotation[i % 3][j % 3] if i//3 == j//3 else 0.
                 for j in range(6)] for i in range(6)]
    return [sum(jacobian[i][k]*covariance[k*6+l]*jacobian[j][l]
                for k in range(6) for l in range(6))
            for i in range(6) for j in range(6)]


def particle_cloud_metrics(particles, reference):
    """Check the full cloud, not just AMCL's strongest reported hypothesis.

    Each input is (x, y, yaw, weight). A narrow strongest-mode covariance is
    insufficient when another mode has significant probability elsewhere.
    """
    if not particles or not all(len(p)==4 and all(math.isfinite(v) for v in p)
                                and p[3]>=0 for p in particles):
        raise ValueError('invalid or empty AMCL particle cloud')
    total=sum(p[3] for p in particles)
    if total<=0:
        raise ValueError('AMCL particle weights have no positive mass')
    weights=[p[3]/total for p in particles]
    support={}
    for particle,weight in zip(particles,weights):
        if weight<=0:
            continue
        key=(round(particle[0]/.001),round(particle[1]/.001),
             round(wrap(particle[2])/math.radians(.1)))
        support[key]=support.get(key,0.)+weight
    mean_x=sum(w*p[0] for w,p in zip(weights,particles))
    mean_y=sum(w*p[1] for w,p in zip(weights,particles))
    sine=sum(w*math.sin(p[2]) for w,p in zip(weights,particles))
    cosine=sum(w*math.cos(p[2]) for w,p in zip(weights,particles))
    resultant=math.hypot(sine,cosine)
    mean_yaw=math.atan2(sine,cosine)
    var_x=sum(w*(p[0]-mean_x)**2 for w,p in zip(weights,particles))
    var_y=sum(w*(p[1]-mean_y)**2 for w,p in zip(weights,particles))
    xy_std=math.sqrt(max(var_x,var_y))
    xy_rms=math.sqrt(var_x+var_y)
    # Circular standard deviation handles particles around +pi/-pi, and a
    # broad/uniform yaw distribution remains broad rather than averaging zero.
    yaw_std=math.sqrt(-2*math.log(min(1.,max(resultant,1e-15))))
    mass=sum(w for w,p in zip(weights,particles)
             if math.dist(p[:2],reference[:2])<=.30
             and abs(wrap(p[2]-reference[2]))<=math.radians(15.))
    return dict(particle_count=len(particles),xy_std_m=xy_std,xy_rms_m=xy_rms,
                unique_pose_bins=len(support),largest_pose_bin_mass=max(support.values()),
                pose_bin_resolution=dict(xy_m=.001,yaw_deg=.1),
                yaw_std_deg=math.degrees(yaw_std),dominant_mass=mass,
                weighted_mean_xyyaw=[mean_x,mean_y,mean_yaw],
                converged=xy_std<=.15 and math.degrees(yaw_std)<=8. and mass>.90)


class GlobalParticleGate:
    """Require fresh, unimodal cloud evidence before freezing a global trial."""
    def __init__(self):
        self.window=deque(maxlen=32)
        self.last_stamp=None
        self.metrics=None
        self.reason='waiting for global AMCL particle cloud'
        self.healthy=False

    def accept(self, particles, reference, stamp, now,*,metrics=None):
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return self.healthy
        self.last_stamp=stamp
        self.healthy=False
        if not -.05<=now-stamp<=AMCL_MAX_AGE_S:
            self.window.clear()
            self.reason='global AMCL particle cloud is stale'
            return False
        try:
            self.metrics=particle_cloud_metrics(particles,reference) if metrics is None else metrics
        except ValueError as exc:
            self.window.clear()
            self.reason=str(exc)
            return False
        if not self.metrics['converged']:
            self.window.clear()
            self.reason='global AMCL is ambiguous; waiting for a single trusted location; no automatic motion'
            return False
        self.window.append(stamp)
        while len(self.window)>3 and stamp-self.window[0]>1.5:
            self.window.popleft()
        if len(self.window)<3 or stamp-self.window[0]<1.:
            self.reason='waiting for >=3 unambiguous particle clouds spanning >=1 second'
            return False
        self.healthy=True
        self.reason='global AMCL full particle cloud is concentrated around the reported pose'
        return True

    def ready(self, now):
        if not self.healthy or self.last_stamp is None:
            return False,self.reason
        if not -.05<=now-self.last_stamp<=AMCL_MAX_AGE_S:
            return False,'global AMCL particle cloud is stale; no odom fallback'
        return True,self.reason


class TrialCorrection:
    """Pure state machine: one baseline, bounded correction, no stale fallback."""
    def __init__(self, *, bound_total=True, recover_jumps=False):
        self.baseline = None
        self.bound_total = bound_total
        self.recover_jumps = recover_jumps
        self.navigation_recovery = dict(hold_commands=False)
        self.recovery_scans = deque(maxlen=32)
        self.last_jump = None
        self.target = self.applied = (0.,0.,0.)
        self.last_transform = None
        self.amcl_stamp = None
        self.xy_std = self.yaw_std_deg = None
        self.window = deque(maxlen=32)
        self.fault = None
        self.measurement_ok = False
        self.reason = 'waiting for fresh AMCL observations'

    def recover_navigation_jump(self, transform, stamp, now, scan):
        """Validate a candidate with distinct live scans, retaining its AMCL stamp.

        A stopped AMCL need not publish another posterior. Repeated callbacks
        for one paired posterior may therefore supply new scan evidence, but
        repeated scan stamps cannot count as independent checks.
        """
        state = self.navigation_recovery
        if not state['hold_commands']:
            state.update(hold_commands=True, started_ros_s=now,
                         previous_map_odom_xyyaw=list(self.last_transform),
                         candidate_map_odom_xyyaw=list(transform),
                         previous_amcl_stamp_s=self.amcl_stamp,
                         candidate_amcl_stamp_s=stamp)
            self.recovery_scans.clear()
        candidate = state['candidate_map_odom_xyyaw']
        if (math.dist(candidate[:2], transform[:2]) > .03
                or abs(wrap(candidate[2]-transform[2])) > math.radians(1.)):
            self.recovery_scans.clear()
            state['candidate_map_odom_xyyaw'] = list(transform)
        state['candidate_amcl_stamp_s'] = stamp
        try:
            if not isinstance(scan, dict):
                raise ValueError('waiting for candidate LiDAR/map agreement')
            if scan.get('ready') is not True:
                raise ValueError(scan.get('reason', 'candidate LiDAR/map fit is not ready'))
            validate_recognition_scan_match(scan)
            scan_stamp = scan.get('scan_stamp_s')
            if (not isinstance(scan_stamp, (int, float)) or not math.isfinite(scan_stamp)
                    or not -.05 <= now-scan_stamp <= SCAN_MAX_AGE_S):
                raise ValueError('candidate LiDAR scan is stale')
            if (not self.recovery_scans or scan_stamp > self.recovery_scans[-1]):
                if self.recovery_scans and scan_stamp-self.recovery_scans[-1] > SCAN_MAX_AGE_S:
                    self.recovery_scans.clear()
                self.recovery_scans.append(scan_stamp)
        except ValueError as exc:
            self.recovery_scans.clear()
            state['reason'] = str(exc)
            self.reason = 'navigation correction held: '+str(exc)
            state.update(distinct_scan_count=0, scan_span_s=0.)
            return False
        span = self.recovery_scans[-1]-self.recovery_scans[0]
        state.update(distinct_scan_count=len(self.recovery_scans), scan_span_s=span,
                     scan_map_fit=dict(scan), original_amcl_stamp_preserved=True)
        if len(self.recovery_scans) < 3 or span < .5-1e-9:
            self.reason = 'navigation correction held: waiting for three matching scans spanning 0.5 seconds'
            state['reason'] = self.reason
            return False
        state.update(hold_commands=False, recovered_ros_s=now,
                     reason='candidate correction validated by fresh distinct LiDAR scans')
        return True

    def accept(self, transform, covariance, stamp, now, *, navigation_scan=None):
        if self.fault:
            return False
        if self.amcl_stamp is not None and stamp <= self.amcl_stamp:
            return False
        self.measurement_ok = False
        if (not all(math.isfinite(v) for v in (*transform, stamp, now))
                or not -.05 <= now-stamp <= AMCL_MAX_AGE_S):
            self.reason = 'AMCL pose timestamp or transform is invalid/stale'
            return False
        variances = [covariance[i] for i in (0,7,35)]
        if any(not math.isfinite(v) or v < 0 for v in variances):
            self.reason = 'invalid AMCL covariance'
            return False
        self.xy_std = math.sqrt(max(variances[:2]))
        self.yaw_std_deg = math.degrees(math.sqrt(variances[2]))
        if self.xy_std > .10 or self.yaw_std_deg > 5.:
            self.reason = 'AMCL covariance has not converged (limits: 0.10 m / 5 deg std)'
            self.window.clear()
            return False
        if self.last_transform is not None:
            jump = compose(inverse(self.last_transform), transform)
            jumped = math.hypot(*jump[:2]) > .15 or abs(jump[2]) > math.radians(5.)
            if jumped and (self.last_jump is None
                    or self.last_jump['previous_amcl_stamp_s'] != self.amcl_stamp
                    or self.last_jump['candidate_amcl_stamp_s'] != stamp
                    or self.last_jump['candidate_map_odom_xyyaw'] != list(transform)):
                self.last_jump = dict(previous_map_odom_xyyaw=list(self.last_transform),
                    candidate_map_odom_xyyaw=list(transform),
                    previous_amcl_stamp_s=self.amcl_stamp, candidate_amcl_stamp_s=stamp,
                    translation_m=math.hypot(*jump[:2]), yaw_deg=math.degrees(jump[2]),
                    observed_ros_s=now)
            if (self.recover_jumps and self.baseline is not None
                    and (jumped or self.navigation_recovery['hold_commands'])):
                if not self.recover_navigation_jump(transform, stamp, now, navigation_scan):
                    return False
            elif jumped:
                if self.baseline is None:
                    # Global search can move between hypotheses before it
                    # converges. Restart the stable window; do not latch a
                    # motion fault before any trial frame has been accepted.
                    self.window.clear()
                else:
                    self.fault = 'AMCL map->odom jumped; restart localization and recognize again'
                    self.reason = self.fault
                    return False
        self.amcl_stamp, self.last_transform = stamp, tuple(transform)
        self.measurement_ok = True
        if self.baseline is None:
            self.window.append((stamp, tuple(transform)))
            while len(self.window) > 3 and stamp-self.window[0][0] > 1.5:
                self.window.popleft()
            if len(self.window) < 3 or stamp-self.window[0][0] < 1.:
                self.reason = 'waiting for >=3 AMCL observations spanning >=1 second'
                return True
            origin = self.window[0][1]
            if any(math.dist(p[:2], origin[:2]) > .05
                   or abs(wrap(p[2]-origin[2])) > math.radians(2.) for _,p in self.window):
                self.reason = 'AMCL baseline is still changing'
                return True
            self.baseline = tuple(transform)
        candidate = compose(inverse(self.baseline), transform)
        if self.bound_total and (math.hypot(*candidate[:2]) > .20 or abs(candidate[2]) > math.radians(5.)):
            self.fault = 'AMCL correction exceeds 0.20 m / 5 deg trial limits'
            self.reason = self.fault
            return False
        self.target = candidate
        self.reason = 'AMCL baseline locked; confidence is not independent physical accuracy proof'
        return True

    def ready(self, now, scan_stamp, odom_stamp, *, stationary_cached=False,motion_predicted=False,
              external_odom=False):
        if self.fault or not self.measurement_ok or self.baseline is None:
            return False, self.fault or self.reason
        for source, stamp, limit in (('AMCL', self.amcl_stamp, AMCL_MAX_AGE_S),
                                     ('LiDAR scan', scan_stamp, SCAN_MAX_AGE_S),
                                     ('EKF odom', odom_stamp, ODOM_MAX_AGE_S)):
            if source=='AMCL' and (stationary_cached or motion_predicted):
                continue
            if source=='EKF odom' and external_odom:
                # The separate control process checks every EKF capture
                # against the same 0.2 s limit, independently of this queue.
                continue
            if stamp is None or not -.05 <= now-stamp <= limit:
                return False, f'{source} is missing/stale; no raw odom fallback'
        return True, self.reason

    def step(self, dt):
        self.applied = bounded_correction(self.applied, self.target, dt)
        return self.applied


class AmclTrialBridge(Node):
    def __init__(self, args, *, context=None):
        super().__init__('r3_amcl_trial_bridge', context=context)
        self.args = args
        self.stationary_inspection=bool(getattr(args,'stationary_inspection',False))
        self.navigation_monitor=bool(getattr(args,'navigation_monitor',False))
        self.external_control_odom=bool(getattr(args,'external_control_odom',False))
        self.continuous_localization=bool(getattr(args,'continuous_localization',False) or self.navigation_monitor)
        self.run_id = str(uuid.uuid4())
        self.map_file = str(args.map.resolve())
        self.correction = TrialCorrection(bound_total=not self.navigation_monitor,
                                          recover_jumps=self.navigation_monitor)
        self.raw_odom = None
        self.odom_stamp = self.scan_stamp = None
        self.map_received = False
        self.map_signature = None
        self.scan_checker = None
        self.latest_scan = None
        self.scan_match = dict(ready=False,reason='waiting for LiDAR/map fit')
        self.last_fit_key = None
        self.last_scored_scan = None
        self.last_scored_match = None
        self.last_scored_observation_stamp = None
        self.active = False
        self.state_future = self.nomotion_future = None
        self.next_state = self.next_nomotion = 0.
        self.initial_pose_sent = False
        seed_ready=bool(getattr(args,'global_seed_ready',False))
        self.stationary_cache_enabled=(self.stationary_inspection and seed_ready) or self.continuous_localization
        self.stationary_evidence=StationaryEvidence()
        self.stationary_snapshot=None
        self.seed_refresh_requested=False
        self.nomotion_request_count=0
        self.nomotion_interfaces_since=None
        self.refresh_conditions={}
        self.parked_since_wall=None
        self.parking_refresh_armed=True
        self.refresh_odom_anchor=None
        self.motion_witness_stamp=None
        self.motion_witness_pose=None
        self.motion_witness_last_stamp=None
        self.motion_witness_count=0
        self.last_parked_validation=None
        self.previous_robot_stationary=None
        self.motion_prediction=None
        self.continuous_invalid_after_stamp=None
        self.stream_recovery_witness=None
        self.stream_recovery_requested_key=None
        self.stream_clock_reversed=False
        self.last_stationary_fault=None
        explicit_global=bool(getattr(args,'global_localization',False))
        current_seed=seed_ready or (self.continuous_localization and not explicit_global)
        self.global_localization = bool(explicit_global or seed_ready or self.continuous_localization)
        self.global_requested = self.global_initialized = current_seed
        self.global_requested_ros_s = self.get_clock().now().nanoseconds*1e-9 if current_seed else None
        self.global_future = None
        self.global_gate = GlobalParticleGate()
        self.latest_cloud = None
        self.particle_cloud_history=deque(maxlen=12)
        self.global_fault = None
        self.ready_file_written = False
        self.trial_handoff = None
        self.delivery_handoff = None
        self.delivery_refresh_after = None
        self.pending = deque(maxlen=20)
        self.pending_issue = ''
        self.pair_validation_cache={}
        self.paired_metrics=None
        self.processing_times={}
        self.last_received_amcl_stamp = None
        self.last_received_amcl_at = None
        self.latest_observation = None
        self.last_tick = time.monotonic()
        self.last_progress_log = 0.
        self.last_marker_wall = 0.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_client = self.create_client(GetState, '/amcl/get_state')
        self.state_poll = LifecyclePoll({'amcl':self.state_client},poll_period_s=.5,
                                        recreate_client=self.recreate_state_client)
        self.last_state_error = None
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.global_client = self.create_client(Empty, '/reinitialize_global_localization')
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/pallet/r3/localization/status', reliable)
        self.marker_pub = self.create_publisher(MarkerArray, '/pallet/r3/localization/markers', reliable)
        self.odom_pub = (None if self.external_control_odom else
                         self.create_publisher(Odometry, '/pallet/r3/localized_odom', 10))
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.amcl_subscription=self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, reliable)
        self.cloud_subscription=self.create_subscription(ParticleCloud, '/particle_cloud', self.on_particles, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, reliable)
        self.create_service(Trigger, '/pallet/r3/localization/enable_trial', self.enable_trial)
        self.create_service(Trigger, '/pallet/r3/localization/enable_delivery_navigation',
                            self.enable_delivery_navigation)
        self.create_service(Trigger, '/pallet/r3/localization/reset_cycle',
                            self.reset_cycle)
        self.create_timer(.05, self.tick)

    def _check_handoff_context(self, now, *, from_navigation):
        if (not self.continuous_localization or self.navigation_monitor != from_navigation
                or self.stationary_inspection):
            raise ValueError('localization handoff requires the previous continuous role')
        if not self.active or not self.map_received or self.scan_checker is None:
            raise ValueError('active AMCL and the selected map are required for trial handoff')
        if self.correction.fault:
            raise ValueError(self.correction.fault)
        if not self.global_initialized or self.global_fault:
            raise ValueError(self.global_fault or 'global localization has not been validated')
        if not self.robot_is_stationary(now):
            raise ValueError('robot must be stationary with fresh odometry before trial handoff')
        for topic in ('/cmd_vel_nav','/cmd_vel_nav_checked','/cmd_vel_localized','/cmd_vel_smoothed','/cmd_vel_safe'):
            publishers = self.get_publishers_info_by_topic(topic)
            if publishers:
                owners = ', '.join(getattr(endpoint,'node_name','unknown') for endpoint in publishers)
                raise ValueError(f'command publishers must stop before trial handoff: {topic}: {owners}')

    def _parked_handoff_observation(self, now, *, newer_than=None,
                                   validate_scan=validate_recognition_scan_match):
        if not self.correction.measurement_ok:
            raise ValueError('AMCL observation is not valid for handoff')
        cached, reason = self.stationary_evidence.ready(now)
        if not cached:
            raise ValueError('stationary localization is not validated: '+reason)
        snapshot = self.stationary_snapshot
        if not isinstance(snapshot, dict) or self.latest_observation is None:
            raise ValueError('paired AMCL snapshot is unavailable for trial handoff')
        stamp = snapshot['amcl_stamp_s']
        if newer_than is not None and stamp <= newer_than:
            raise ValueError('waiting for the parked AMCL observation requested after pickup')
        if (self.continuous_invalid_after_stamp is not None
                and stamp <= self.continuous_invalid_after_stamp):
            raise ValueError('new AMCL observation required after localization interruption')
        if (snapshot.get('run_id') != self.run_id or snapshot.get('map_file') != self.map_file
                or snapshot.get('occupancy_map_signature') != self.map_signature):
            raise ValueError('trial handoff snapshot belongs to another map/session')
        observation_stamp, transform = self.latest_observation
        if (observation_stamp != stamp or self.correction.amcl_stamp != stamp
                or len(transform) != 3 or not all(math.isfinite(v) for v in transform)
                or math.dist(transform[:2], snapshot['map_odom_xyyaw'][:2]) > 1e-8
                or abs(wrap(transform[2]-snapshot['map_odom_xyyaw'][2])) > 1e-8):
            raise ValueError('trial handoff map correction does not match the original observation')
        if any(self.stamp(message) > stamp for message in self.pending):
            raise ValueError('waiting for a newer pending AMCL observation before trial handoff')
        if self.nomotion_future is not None and not self.nomotion_future.done():
            raise ValueError('waiting for the pending AMCL refresh before trial handoff')
        fit = self.scan_match
        scored_stamp = fit.get('scan_stamp_s')
        if (self.scan_stamp is None or not -.05 <= now-self.scan_stamp <= SCAN_MAX_AGE_S
                or scored_stamp is None or not -.05 <= now-scored_stamp <= SCAN_MAX_AGE_S
                or fit.get('ready') is not True):
            raise ValueError('trial handoff requires fresh scan/map agreement')
        validate_scan(fit)
        covariance = snapshot['covariance_6x6']
        if (len(covariance) != 36 or not all(math.isfinite(v) for v in covariance)
                or any(covariance[i] < 0 for i in (0,7,35))
                or math.sqrt(covariance[0]+covariance[7]) >= .05-1e-12
                or math.degrees(math.sqrt(covariance[35])) > 2.+1e-9):
            raise ValueError('trial handoff AMCL uncertainty requires <5 cm / 2 deg')
        metrics = snapshot['particle_metrics']
        validate_runtime_particles(metrics)
        return tuple(transform), covariance, stamp

    def _check_trial_handoff(self, now):
        """Validate the existing parked posterior without updating AMCL."""
        self._check_handoff_context(now, from_navigation=True)
        return self._parked_handoff_observation(now)

    def enable_trial(self, request, response):
        """Reanchor the trial frame using the already validated posterior.

        No AMCL service, new pose, TF or motor command is emitted here. Nav2
        must have released all command topics before the first role change.
        Repeated requests cannot reanchor a target after capture has started.
        """
        if self.delivery_handoff is not None:
            response.success = False
            response.message = 'delivery navigation already owns this localization session'
            return response
        if self.trial_handoff is not None and not self.navigation_monitor:
            response.success = True
            response.message = json.dumps(dict(status='TRIAL_LOCALIZATION_ENABLED',
                run_id=self.run_id, trial_handoff=self.trial_handoff), allow_nan=False)
            return response
        now = self.get_clock().now().nanoseconds*1e-9
        try:
            transform, covariance, stamp = self._check_trial_handoff(now)
            correction = TrialCorrection(bound_total=True)
            correction.baseline = correction.last_transform = transform
            correction.amcl_stamp = stamp
            correction.xy_std = math.sqrt(max(covariance[0],covariance[7]))
            correction.yaw_std_deg = math.degrees(math.sqrt(covariance[35]))
            correction.measurement_ok = True
            correction.reason = 'trial frame reanchored to the live-scan-validated navigation posterior; no AMCL refresh'
            handoff = dict(enabled=True, at_ros_s=now, enabled_ros_s=now,
                source_amcl_stamp_s=stamp, source_cloud_stamp_s=self.stationary_snapshot['cloud_stamp_s'],
                baseline_map_odom=list(transform), nomotion_request_count=self.nomotion_request_count,
                occupancy_map_signature=self.map_signature,
                previous_role='navigation_monitor',
                source='validated_navigation_snapshot_without_amcl_refresh')
            self.correction = correction
            self.navigation_monitor = False
            self.trial_handoff = handoff
            self.motion_prediction = None
            self.last_parked_validation = None
            self.refresh_odom_anchor = planar_pose(self.raw_odom.pose.pose.position,self.raw_odom.pose.pose.orientation)
            self.parking_refresh_armed = False
            self.motion_witness_stamp = self.motion_witness_pose = self.motion_witness_last_stamp = None
            self.motion_witness_count = 0
            # Keep the original first-ready file as startup evidence. The
            # latched status and the caller's handoff report expose this frame.
            self.ready_file_written = True
            response.success = True
            response.message = json.dumps(dict(status='TRIAL_LOCALIZATION_ENABLED',
                run_id=self.run_id, trial_handoff=handoff), allow_nan=False)
        except (ValueError,TypeError,KeyError,RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def enable_delivery_navigation(self, request, response):
        """Release the bounded docking frame only from a validated parked pose."""
        if self.delivery_handoff is not None:
            response.success = True
            response.message = json.dumps(self.delivery_handoff)
            return response
        now = self.get_clock().now().nanoseconds*1e-9
        try:
            resumed = getattr(self.args, 'resume_delivery', False)
            if not resumed and self.trial_handoff is None:
                raise ValueError('delivery handoff requires an existing docking session')
            if not resumed and self.trial_handoff.get('occupancy_map_signature') != self.map_signature:
                raise ValueError('map changed since the docking handoff')
            self._check_handoff_context(now, from_navigation=resumed)
            cached, _ = self.stationary_evidence.ready(now)
            previous_stamp = (self.stationary_snapshot or {}).get('amcl_stamp_s')
            old_snapshot = previous_stamp is not None and now-previous_stamp > AMCL_MAX_AGE_S
            if (not resumed and self.delivery_refresh_after is None and not cached
                    and (self.stationary_evidence.fault or old_snapshot)):
                # After pickup, allow one new observation if the previous
                # parked evidence was invalidated or hasn't validated again.
                self.delivery_refresh_after = now
                self.parking_refresh_armed = True
                self.get_logger().info('Validating AMCL once after the completed pickup; retaining the same session')
            transform, covariance, stamp = self._parked_handoff_observation(
                now, newer_than=self.delivery_refresh_after, validate_scan=validate_navigation_scan_match)
            correction = TrialCorrection(bound_total=False, recover_jumps=True)
            correction.baseline = correction.last_transform = transform
            correction.amcl_stamp = stamp
            correction.xy_std = math.sqrt(max(covariance[0], covariance[7]))
            correction.yaw_std_deg = math.degrees(math.sqrt(covariance[35]))
            correction.measurement_ok = True
            correction.reason = 'delivery navigation retains the validated parked AMCL observation'
        except (ValueError, TypeError, KeyError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        self.correction = correction
        self.navigation_monitor = True
        self.motion_prediction = None
        self.last_parked_validation = None
        self.delivery_handoff = dict(run_id=self.run_id, map_file=self.map_file,
                                     at_ros_s=now, source_amcl_stamp_s=stamp,
                                     post_pickup_refresh_requested_ros_s=self.delivery_refresh_after)
        response.success = True
        response.message = json.dumps(self.delivery_handoff)
        return response

    def reset_cycle(self, request, response):
        """Reset per-cycle handoff flags for a repeat A->B->D cycle without re-running global localization."""
        now = self.get_clock().now().nanoseconds*1e-9
        try:
            if not self.continuous_localization:
                raise ValueError('reset_cycle requires continuous localization mode')
            for topic in ('/cmd_vel_nav', '/cmd_vel_nav_checked', '/cmd_vel_localized', '/cmd_vel_smoothed', '/cmd_vel_safe'):
                publishers = self.get_publishers_info_by_topic(topic)
                if publishers:
                    owners = ', '.join(getattr(endpoint, 'node_name', 'unknown') for endpoint in publishers)
                    raise ValueError(f'command publishers must stop before cycle reset: {topic}: {owners}')
            self.trial_handoff = None
            self.delivery_handoff = None
            self.delivery_refresh_after = None
            self.navigation_monitor = True
            self.motion_prediction = None
            self.last_parked_validation = None
            prev = self.correction
            correction = TrialCorrection(bound_total=False, recover_jumps=True)
            if prev is not None and prev.measurement_ok and prev.baseline is not None:
                correction.baseline = prev.baseline
                correction.last_transform = prev.last_transform
                correction.amcl_stamp = prev.amcl_stamp
                correction.xy_std = prev.xy_std
                correction.yaw_std_deg = prev.yaw_std_deg
                correction.measurement_ok = True
                correction.reason = 'retained continuous AMCL session for next cycle'
            self.correction = correction
            cached, _ = self.stationary_evidence.ready(now)
            if not cached:
                self.parking_refresh_armed = True
            self.get_logger().info('Reset cycle handoff state; keeping existing AMCL localization session')
            response.success = True
            response.message = json.dumps(dict(
                status='CYCLE_RESET', run_id=self.run_id, map_file=self.map_file,
                at_ros_s=now, stationary_ready=bool(cached),
            ), allow_nan=False)
        except (ValueError, TypeError, KeyError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


    def on_scan(self, msg):
        stamp=self.stamp(msg) if msg.header.frame_id == 'lidar_link' else None
        if self.continuous_localization and self.scan_stamp is not None and stamp is not None:
            if stamp<self.scan_stamp:
                self.stream_clock_reversed=True
            if stamp<self.scan_stamp or stamp-self.scan_stamp>.5:
                self.stationary_evidence.invalidate('LiDAR capture gap/clock change requires a new AMCL observation')
                self.continuous_invalid_after_stamp=self.get_clock().now().nanoseconds*1e-9
        self.scan_stamp = stamp
        self.latest_scan = msg if self.scan_stamp is not None else None

    def on_map(self,msg):
        # Identical transient-local map deliveries are harmless. An actual
        # geometry/content change must invalidate the original observation.
        geometry=(msg.header.frame_id,msg.info.width,msg.info.height,msg.info.resolution,
                  msg.info.origin.position.x,msg.info.origin.position.y,
                  msg.info.origin.orientation.x,msg.info.origin.orientation.y,
                  msg.info.origin.orientation.z,msg.info.origin.orientation.w)
        signature=hashlib.sha256(repr(geometry).encode()+bytes(v & 255 for v in msg.data)).hexdigest()
        if self.map_signature is not None and signature!=self.map_signature:
            self.stationary_evidence.invalidate('map content changed; acquire a new AMCL snapshot')
            if self.continuous_localization:
                self.continuous_invalid_after_stamp=self.get_clock().now().nanoseconds*1e-9
        self.map_signature=signature
        self.map_received=False
        try:
            self.scan_checker=ScanMapCheck(msg)
        except (ValueError,TypeError) as exc:
            self.scan_checker=None
            self.scan_match=dict(ready=False,reason='invalid localization map: '+str(exc))
            return
        self.map_received=True
        self.last_fit_key=None
        self.last_scored_scan=self.last_scored_match=self.last_scored_observation_stamp=None

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_footprint','base_link'):
            self.raw_odom = None
            self.stationary_evidence.invalidate('invalid odometry frame')
            return
        try:
            odom_pose=planar_pose(msg.pose.pose.position, msg.pose.pose.orientation)
            twist=msg.twist.twist
            if not all(math.isfinite(v) for v in (twist.linear.x,twist.linear.y,twist.angular.z)):
                raise ValueError('nonfinite odometry twist')
        except ValueError:
            self.raw_odom = None
            self.stationary_evidence.invalidate('invalid odometry pose')
            return
        previous_stamp=self.odom_stamp
        self.raw_odom, self.odom_stamp = msg, self.stamp(msg)
        if previous_stamp is not None and self.odom_stamp<previous_stamp:
            self.stream_clock_reversed=True
        if self.continuous_localization:
            now=self.get_clock().now().nanoseconds*1e-9
            fresh=-.05<=now-self.odom_stamp<=ODOM_MAX_AGE_S
            continuous=(previous_stamp is None or 0.<self.odom_stamp-previous_stamp<=.25)
            if not fresh or not continuous:
                self.motion_witness_stamp=self.motion_witness_pose=self.motion_witness_last_stamp=None
                self.motion_witness_count=0
            stopped=math.hypot(twist.linear.x,twist.linear.y)<.005 and abs(twist.angular.z)<.01
            if fresh and not stopped and self.previous_robot_stationary is True:
                proof=self.last_parked_validation
                snapshot=self.stationary_snapshot
                if (proof is not None and snapshot is not None
                        and -.05<=now-proof['validated_ros_s']<=.25
                        and proof['source_amcl_stamp_s']==snapshot['amcl_stamp_s']
                        and proof['map_signature']==self.map_signature):
                    self.motion_prediction=dict(started_ros_s=self.odom_stamp,
                        anchor_odom_xyyaw=list(proof['odom_xyyaw']),
                        source_amcl_stamp_s=snapshot['amcl_stamp_s'],
                        source_parked_validation_ros_s=proof['validated_ros_s'],
                        map_signature=self.map_signature,ended=False,
                        max_duration_s=MOTION_PREDICTION_MAX_S,
                        max_distance_m=MOTION_PREDICTION_MAX_M,
                        max_yaw_rad=MOTION_PREDICTION_MAX_YAW_RAD)
                # The original parked proof can arm only one onset window,
                # even if velocity briefly returns to zero while maneuvering.
                self.last_parked_validation=None
            self.previous_robot_stationary=stopped if fresh else None
            if fresh and stopped:
                if self.parked_since_wall is None:
                    self.parked_since_wall=time.monotonic()
                self.motion_witness_stamp=self.motion_witness_pose=self.motion_witness_last_stamp=None
                self.motion_witness_count=0
            else:
                self.parked_since_wall=None
                if fresh:
                    if self.motion_witness_stamp is None:
                        self.motion_witness_stamp=self.odom_stamp
                        self.motion_witness_pose=odom_pose
                    if self.motion_witness_last_stamp is None or self.odom_stamp>self.motion_witness_last_stamp:
                        self.motion_witness_count+=1
                        self.motion_witness_last_stamp=self.odom_stamp
                    witnessed_travel=(math.dist(odom_pose[:2],self.motion_witness_pose[:2])>=.0005
                        or abs(wrap(odom_pose[2]-self.motion_witness_pose[2]))>=.001)
                    if (self.motion_witness_count>=3
                            and self.odom_stamp-self.motion_witness_stamp>=.1-1e-9
                            and witnessed_travel):
                        self.parking_refresh_armed=True
            anchor=self.refresh_odom_anchor
            if fresh and anchor is not None and (math.dist(odom_pose[:2],anchor[:2])>=.005
                    or abs(wrap(odom_pose[2]-anchor[2]))>=math.radians(.5)):
                # Genuine travel creates a new scene. Refresh once after the
                # next stop, including a short final Nav2 adjustment smaller
                # than AMCL's normal motion-update threshold. One noisy twist
                # sample or nonzero twist with a fixed pose cannot rearm it.
                self.parking_refresh_armed=True
        if self.stationary_cache_enabled:
            twist=msg.twist.twist
            self.stationary_evidence.update_odom(
                planar_pose(msg.pose.pose.position,msg.pose.pose.orientation),
                (twist.linear.x,twist.linear.y,twist.angular.z),self.odom_stamp,
                self.get_clock().now().nanoseconds*1e-9)

    def robot_is_stationary(self,now):
        if self.raw_odom is None or self.odom_stamp is None or not -.05<=now-self.odom_stamp<=ODOM_MAX_AGE_S:
            return False
        twist=self.raw_odom.twist.twist
        return math.hypot(twist.linear.x,twist.linear.y)<.005 and abs(twist.angular.z)<.01

    def stream_recovery_ready(self,now,wall,sensors_ready):
        """One genuine new AMCL update after a stationary capture gap recovers.

        Old evidence stays invalid until that new pose/cloud pair is received.
        No recovery refresh is permitted after a target's trial frame is frozen.
        """
        fault=self.stationary_evidence.fault or ''
        snapshot=self.stationary_snapshot
        eligible=(self.continuous_localization and self.navigation_monitor
            and self.seed_refresh_requested and not self.stream_clock_reversed
            and snapshot is not None and self.continuous_invalid_after_stamp is not None
            and snapshot.get('occupancy_map_signature')==self.map_signature
            and (fault.startswith('odom gap/') or fault.startswith('LiDAR capture gap/'))
            and not self.correction.fault and not self.global_fault
            and sensors_ready and self.robot_is_stationary(now))
        if not eligible:
            self.stream_recovery_witness=None
            return False
        key=(self.continuous_invalid_after_stamp,snapshot['amcl_stamp_s'],self.map_signature)
        if key==self.stream_recovery_requested_key:
            return False
        raw=self.raw_odom.pose.pose
        pose=planar_pose(raw.position,raw.orientation)
        witness=self.stream_recovery_witness
        if (witness is None or witness['key']!=key
                or math.dist(pose[:2],witness['pose'][:2])>.005
                or abs(wrap(pose[2]-witness['pose'][2]))>math.radians(.5)):
            self.stream_recovery_witness=dict(key=key,pose=pose,started=wall)
            return False
        return wall-witness['started']>=1.-1e-9

    def on_amcl(self, msg):
        if msg.header.frame_id != 'map':
            self.pending_issue = 'AMCL pose has unexpected frame'
            self.correction.measurement_ok = False
            self.pending.clear()
            self.stationary_evidence.invalidate('invalid AMCL frame')
            return
        try:
            planar_pose(msg.pose.pose.position, msg.pose.pose.orientation)
            if len(msg.pose.covariance)!=36 or not all(math.isfinite(v) for v in msg.pose.covariance):
                raise ValueError('AMCL covariance contains invalid values')
        except ValueError as exc:
            self.pending_issue = str(exc)
            self.correction.measurement_ok = False
            self.pending.clear()
            self.stationary_evidence.invalidate('invalid AMCL observation')
            return
        self.last_received_amcl_stamp = self.stamp(msg)
        self.last_received_amcl_at = self.get_clock().now().nanoseconds*1e-9
        self.pending.append(msg)

    def on_particles(self,msg):
        if msg.header.frame_id!='map':
            self.latest_cloud=None
            self.particle_cloud_history.clear()
            self.global_gate.healthy=False
            self.global_gate.window.clear()
            self.global_gate.reason='AMCL particle cloud has unexpected frame'
            return
        try:
            particles=[(*planar_pose(p.pose.position,p.pose.orientation),p.weight) for p in msg.particles]
        except ValueError as exc:
            self.latest_cloud=None
            self.particle_cloud_history.clear()
            self.global_gate.healthy=False
            self.global_gate.window.clear()
            self.global_gate.reason=str(exc)
            return
        stamp=self.stamp(msg)
        if self.global_requested_ros_s is not None and stamp<self.global_requested_ros_s:
            return
        self.latest_cloud=(stamp,particles)
        self.particle_cloud_history.append(self.latest_cloud)

    def pair_observation_cloud(self,map_base,covariance,stamp,now):
        """Keep a newer publication from discarding a pose's matching cloud.

        ROS delivers AMCL pose and cloud on separate subscriptions. A pending
        pose can outlive the next cloud publication while waiting for TF. Any
        fallback to retained data must numerically reproduce that exact pose
        and covariance; timestamp proximity alone cannot identify an old cloud.
        All candidates keep the existing two-second freshness bounds.
        """
        if self.latest_cloud is None:
            raise ValueError('waiting for global AMCL particle cloud')
        cache=getattr(self,'pair_validation_cache',None)
        if cache is None:
            cache={}
            self.pair_validation_cache=cache

        def validate(cloud_stamp,particles,*,retained=False):
            # Cached proofs never bypass the normal capture-age limits.
            if not all(-.05<=now-s<=AMCL_MAX_AGE_S for s in (stamp,cloud_stamp)):
                raise ValueError('AMCL/cloud pair was not fresh at validation')
            key=(id(particles),cloud_stamp,stamp,tuple(map_base),tuple(covariance),retained)
            if key not in cache:
                try:
                    metrics=particle_cloud_metrics(particles,map_base)
                    concentrated=concentrated_particle_support(metrics)
                    proof=validate_observation_pair(particles,map_base,covariance,stamp,cloud_stamp,now,
                        require_posterior_identity=retained or concentrated)
                    if concentrated:
                        proof['concentrated_cloud_posterior_verified']=True
                    cache[key]=(proof,metrics,None)
                except ValueError as exc:
                    cache[key]=(None,None,str(exc))
                if len(cache)>64:del cache[next(iter(cache))]
            proof,metrics,error=cache[key]
            if error is not None:raise ValueError(error)
            self.paired_metrics=(stamp,cloud_stamp,tuple(map_base),metrics)
            return copy.deepcopy(proof)

        cloud_stamp,particles=self.latest_cloud
        try:
            proof=validate(cloud_stamp,particles)
            return cloud_stamp,particles,proof
        except ValueError as latest_issue:
            history=getattr(self,'particle_cloud_history',())
            for candidate_stamp,candidate_particles in reversed(history):
                if candidate_stamp==cloud_stamp:
                    continue
                try:
                    proof=validate(candidate_stamp,candidate_particles,retained=True)
                except ValueError:
                    continue
                proof['retained_cloud_posterior_verified']=True
                return candidate_stamp,candidate_particles,proof
            raise latest_issue

    def recreate_state_client(self,name,client):
        self.destroy_client(client)
        self.state_client=self.create_client(GetState,'/amcl/get_state')
        return self.state_client

    def poll_services(self, wall):
        # A lifecycle read is idempotent. Drop a lost response after two
        # seconds and retry rather than leaving startup behind one forever-
        # pending future. Recreate the client after repeated transport failures.
        self.state_poll.clients['amcl']=self.state_client
        self.state_poll.next_poll['amcl']=self.next_state
        self.state_poll.tick(wall)
        self.next_state=self.state_poll.next_poll['amcl']
        pending=self.state_poll.pending.get('amcl')
        self.state_future=pending[0] if pending is not None else None
        if self.state_poll.states:
            self.active,state_reason=lifecycle_readiness(self.state_poll.states,wall,names=('amcl',))
            state_error=None if self.active else state_reason
            if state_error is not None and state_error!=self.last_state_error:
                self.get_logger().warn('AMCL lifecycle check: '+state_error)
            self.last_state_error=state_error
        if (self.global_localization and self.active and self.map_received
                and not self.global_requested and self.global_client.service_is_ready()):
            self.global_future=self.global_client.call_async(Empty.Request())
            self.global_requested=True
            self.global_requested_ros_s=self.get_clock().now().nanoseconds*1e-9
            self.pending.clear()
            self.latest_cloud=None
            self.particle_cloud_history.clear()
            self.get_logger().info('Requested global AMCL localization once; waiting for unambiguous LiDAR/map match')
        if self.global_future is not None and self.global_future.done():
            try:
                if self.global_future.result() is None:
                    raise RuntimeError('empty global localization service response')
                self.global_initialized=True
            except Exception as exc:
                self.global_fault='global localization service failed: '+str(exc)
            self.global_future=None
        if self.active and self.map_received and self.args.initial_pose is not None and not self.initial_pose_sent:
            x,y,yaw_deg = self.args.initial_pose
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.pose.position.x, message.pose.pose.position.y = x,y
            yaw = math.radians(yaw_deg)
            message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
            message.pose.covariance[0] = message.pose.covariance[7] = .25**2
            message.pose.covariance[35] = math.radians(10.)**2
            self.initial_pub.publish(message)
            self.initial_pose_sent = True
            self.get_logger().info(f'Published explicit initial pose: x={x}, y={y}, yaw={yaw_deg} deg')
        if self.nomotion_future is not None and self.nomotion_future.done():
            self.nomotion_future = None
        stationary = (self.raw_odom is not None
                      and abs(self.raw_odom.twist.twist.linear.x) < .005
                      and abs(self.raw_odom.twist.twist.linear.y) < .005
                      and abs(self.raw_odom.twist.twist.angular.z) < .01)
        # One initial refresh catches the seed published before this bridge
        # subscribed. Repeated stationary likelihood/resampling updates would
        # deplete the cloud and move a parked robot's map estimate.
        request_allowed=(not self.stationary_cache_enabled or not self.seed_refresh_requested)
        if self.continuous_localization and self.stationary_cache_enabled:
            request_allowed=(self.parking_refresh_armed and self.parked_since_wall is not None
                             and wall-self.parked_since_wall>=.5)
        recovering_stream=False
        if self.stationary_cache_enabled:
            # A ready service alone does not establish that AMCL has matched
            # our best-effort cloud reader. Wait for both data interfaces to
            # settle before the sole refresh, otherwise that cloud can vanish
            # before discovery completes and will never be re-published.
            interfaces=(self.amcl_subscription.get_publisher_count()>0
                        and self.cloud_subscription.get_publisher_count()>0)
            if not interfaces:
                self.nomotion_interfaces_since=None
            elif self.nomotion_interfaces_since is None:
                self.nomotion_interfaces_since=wall
            request_allowed=(request_allowed and self.nomotion_interfaces_since is not None
                             and wall-self.nomotion_interfaces_since>=1.)
            now=self.get_clock().now().nanoseconds*1e-9
            # A newly matched TF reader initially has no history. AMCL can
            # process a queued scan a little older than our first TF sample.
            # Cover the whole allowed observation age before consuming the
            # sole refresh, and verify the latest scan has an exact TF too.
            sensors_ready=(self.latest_scan is not None and self.scan_stamp is not None
                and -.05<=now-self.scan_stamp<=SCAN_MAX_AGE_S
                and self.odom_stamp is not None and -.05<=now-self.odom_stamp<=ODOM_MAX_AGE_S)
            history_tf_ready=sensors_ready and self.tf_buffer.can_transform('odom','base_footprint',
                    Time(nanoseconds=round((now-AMCL_MAX_AGE_S)*1e9)))
            scan_tf_ready=sensors_ready and self.tf_buffer.can_transform('odom','base_footprint',
                    Time.from_msg(self.latest_scan.header.stamp))
            capture_tf_ready=sensors_ready and history_tf_ready and scan_tf_ready
            self.refresh_conditions=dict(sensors_fresh=bool(sensors_ready),
                history_tf_ready=bool(history_tf_ready),scan_tf_ready=bool(scan_tf_ready),
                parked_duration_s=(wall-self.parked_since_wall if self.parked_since_wall is not None else None),
                interfaces_ready=interfaces,service_ready=self.nomotion_client.service_is_ready(),
                stationary=stationary,refresh_armed=self.parking_refresh_armed)
            recovering_stream=self.stream_recovery_ready(now,wall,
                bool(capture_tf_ready and interfaces and self.nomotion_client.service_is_ready()))
            request_allowed=(request_allowed or recovering_stream) and capture_tf_ready
            if request_allowed and not self.navigation_monitor and self.delivery_refresh_after is not None:
                request_allowed = self.scan_stamp > self.delivery_refresh_after
        if (request_allowed and self.active and stationary and wall >= self.next_nomotion
                and self.nomotion_future is None and self.nomotion_client.service_is_ready()):
            self.nomotion_future = self.nomotion_client.call_async(Empty.Request())
            if recovering_stream:
                self.stream_recovery_requested_key=self.stream_recovery_witness['key']
                self.get_logger().info('Recovered fresh stationary sensor streams; requesting one new AMCL observation after the capture gap')
            self.seed_refresh_requested=True
            if self.continuous_localization:
                self.parking_refresh_armed=False
                self.refresh_odom_anchor=planar_pose(self.raw_odom.pose.pose.position,
                                                    self.raw_odom.pose.pose.orientation)
            self.nomotion_request_count+=1
            if self.stationary_cache_enabled:
                self.get_logger().info('Requested one AMCL refresh for the new parked episode; further parked checks use live LiDAR/odom without resampling')
            self.next_nomotion = wall+.5

    def process_pending(self, now):
        navigation_monitor = getattr(self, 'navigation_monitor', False)
        for _ in range(len(self.pending)):
            msg = self.pending.popleft()
            stamp = self.stamp(msg)
            if (self.global_localization and self.global_requested_ros_s is not None
                    and stamp<self.global_requested_ros_s):
                continue
            if self.correction.amcl_stamp is not None and stamp <= self.correction.amcl_stamp:
                continue
            recovery = self.correction.navigation_recovery
            if (navigation_monitor and recovery['hold_commands']
                    and stamp < recovery['candidate_amcl_stamp_s']):
                # Once a newer paired posterior is being checked, retrying an
                # older queued candidate must not reset the consistency window.
                continue
            if now-stamp > AMCL_MAX_AGE_S:
                if getattr(self,'continuous_localization',False) and not self.pending:
                    self.pending_issue=''
                continue
            try:
                transform = self.tf_buffer.lookup_transform('odom','base_footprint',Time.from_msg(msg.header.stamp))
                odom_base = planar_pose(transform.transform.translation, transform.transform.rotation)
                map_base = planar_pose(msg.pose.pose.position, msg.pose.pose.orientation)
                pose = map_odom_from_capture(map_base, odom_base)
            except (TransformException, ValueError) as exc:
                self.pending.append(msg)
                self.pending_issue = 'waiting for EKF odom->base at AMCL capture stamp: '+str(exc)
                continue
            self.pending_issue = ''
            if not getattr(self,'continuous_localization',False):
                self.latest_observation = (stamp, pose)
            particle_pairing=None
            if self.global_localization:
                if not self.global_initialized or self.global_fault:
                    self.correction.measurement_ok=False
                    self.pending_issue=self.global_fault or 'waiting for global AMCL initialization service'
                    continue
                if self.latest_cloud is None:
                    self.correction.measurement_ok=False
                    self.pending_issue='waiting for global AMCL particle cloud'
                    if self.stationary_cache_enabled:
                        self.pending.append(msg)
                    continue
                cloud_stamp,particles=self.latest_cloud
                try:
                    if self.stationary_cache_enabled:
                        cloud_stamp,particles,particle_pairing=self.pair_observation_cloud(
                            map_base,list(msg.pose.covariance),stamp,now)
                    elif abs(cloud_stamp-stamp)>.25:
                        raise ValueError('waiting for particle cloud matching current AMCL pose')
                except ValueError as exc:
                    self.correction.measurement_ok=False
                    self.pending_issue=str(exc)
                    if self.stationary_cache_enabled:
                        self.pending.append(msg)
                    continue
                paired=getattr(self,'paired_metrics',None)
                metrics=(paired[3] if paired is not None and paired[:3]==(stamp,cloud_stamp,tuple(map_base)) else None)
                accepted=self.global_gate.accept(particles,map_base,cloud_stamp,now,metrics=metrics)
                if self.stationary_cache_enabled:
                    # Two global scan searches already established this seed.
                    # Require a unimodal fresh cloud, then independent fresh
                    # scans over time instead of manufacturing AMCL updates.
                    accepted=bool(self.global_gate.metrics and self.global_gate.metrics['converged'])
                if not accepted:
                    self.correction.measurement_ok=False
                    if self.correction.baseline is None:
                        self.correction.window.clear()
                    self.pending_issue=self.global_gate.reason
                    continue
            if self.correction.baseline is None and not self.stationary_cache_enabled:
                self.scan_match=self.score_scan_fit(now,pose)
                if not self.scan_match.get('ready',False):
                    self.correction.measurement_ok=False
                    self.correction.window.clear()
                    self.pending_issue=self.scan_match.get('reason','waiting for initial scan-map fit')
                    continue
            fit = None
            if (navigation_monitor and self.correction.last_transform is not None
                    and (self.correction.navigation_recovery['hold_commands']
                         or math.dist(pose[:2], self.correction.last_transform[:2]) > .15
                         or abs(wrap(pose[2]-self.correction.last_transform[2])) > math.radians(5.))):
                fit = self.score_scan_fit(now, pose)
            previous_jump = self.correction.last_jump
            self.correction.accept(pose, msg.pose.covariance, stamp, now, navigation_scan=fit)
            event = self.correction.last_jump
            def event_key(value):
                return ((value['previous_amcl_stamp_s'], value['candidate_amcl_stamp_s'],
                         tuple(value['candidate_map_odom_xyyaw'])) if value is not None else None)
            if event is not None and event_key(event) != event_key(previous_jump):
                self.correction.last_jump.update(odom_at_amcl_capture_xyyaw=list(odom_base),
                    tf_query_stamp_s=stamp,
                    tf_returned_stamp_s=(self.stamp(transform) if
                        getattr(getattr(transform, 'header', None), 'stamp', None) is not None else None),
                    run_id=self.run_id, map_file=self.map_file)
                self.get_logger().warn('AMCL map update event: '+json.dumps(self.correction.last_jump))
            if navigation_monitor and self.correction.navigation_recovery['hold_commands']:
                self.pending.append(msg)
                self.pending_issue = self.correction.reason
            if (getattr(self,'continuous_localization',False)
                    and self.correction.measurement_ok and not self.correction.fault):
                self.latest_observation=(stamp,pose)
                if self.motion_prediction is not None and stamp>self.motion_prediction['source_amcl_stamp_s']:
                    self.motion_prediction.update(ended=True,
                        end_reason='new live AMCL observation ended the onset prediction')
            if (self.stationary_cache_enabled and self.correction.measurement_ok
                    and not self.correction.fault):
                self.stationary_evidence.capture(stamp,odom_base,self.global_gate.metrics)
                self.stationary_snapshot=dict(run_id=self.run_id,map_file=self.map_file,
                    occupancy_map_signature=self.map_signature,
                    amcl_stamp_s=stamp,map_pose_xyyaw=list(map_base),
                    covariance_6x6=list(msg.pose.covariance),odom_anchor_xyyaw=list(odom_base),
                    map_odom_xyyaw=list(pose),cloud_stamp_s=cloud_stamp,
                    particle_pairing=particle_pairing,
                    particle_metrics=copy.deepcopy(self.global_gate.metrics))

    def _remember_continuous_fault(self,now):
        fault=self.stationary_evidence.fault
        if fault!=self.last_stationary_fault:
            self.last_stationary_fault=fault
            if fault and not (fault.startswith('robot motion ') or fault.startswith('robot displacement ')):
                # A fresh AMCL observation is needed after actual sensor gaps,
                # map changes or invalid data. Expected motion only disables
                # parked reuse; live AMCL continues to track that motion.
                self.continuous_invalid_after_stamp=now

    def continuous_live_ready(self,now):
        snapshot=self.stationary_snapshot
        if not self.continuous_localization or snapshot is None:
            return False,'waiting for a paired live AMCL observation'
        stamp=snapshot['amcl_stamp_s']
        if (self.continuous_invalid_after_stamp is not None
                and stamp<=self.continuous_invalid_after_stamp):
            return False,'new AMCL observation required after localization stream interruption'
        for source,value,limit in (('AMCL',stamp,AMCL_MAX_AGE_S),
                ('particle cloud',snapshot['cloud_stamp_s'],AMCL_MAX_AGE_S),
                ('LiDAR',self.scan_stamp,SCAN_MAX_AGE_S),('odom',self.odom_stamp,ODOM_MAX_AGE_S)):
            if source=='odom' and getattr(self,'external_control_odom',False) and not self.navigation_monitor:
                continue
            if value is None or not -.05<=now-value<=limit:
                return False,f'continuous {source} is missing/stale'
        metrics=snapshot['particle_metrics']
        try:
            validate_runtime_particles(metrics)
        except ValueError as exc:
            return False,str(exc)
        if not self.scan_match.get('ready',False):
            return False,self.scan_match.get('reason','waiting for current scan/map agreement')
        return True,'fresh paired AMCL posterior checked by live LiDAR and EKF odom'

    def motion_prediction_ready(self,now):
        prediction=self.motion_prediction
        if not self.continuous_localization or prediction is None:
            return False,'no validated parked-to-moving prediction'
        if prediction['ended']:
            return False,prediction.get('end_reason','motion onset prediction ended')
        source=prediction['source_amcl_stamp_s']
        snapshot=self.stationary_snapshot
        if (snapshot is None or snapshot['amcl_stamp_s']!=source
                or prediction['map_signature']!=self.map_signature
                or self.continuous_invalid_after_stamp is not None
                and source<=self.continuous_invalid_after_stamp):
            prediction.update(ended=True,end_reason='localization/map change invalidated onset prediction')
            return False,prediction['end_reason']
        external_odom=getattr(self,'external_control_odom',False) and not self.navigation_monitor
        if self.raw_odom is None or (not external_odom and
                (self.odom_stamp is None or not -.05<=now-self.odom_stamp<=ODOM_MAX_AGE_S)):
            return False,'motion onset prediction needs fresh odometry'
        raw=planar_pose(self.raw_odom.pose.pose.position,self.raw_odom.pose.pose.orientation)
        anchor=prediction['anchor_odom_xyyaw']
        elapsed=now-prediction['started_ros_s']
        distance=math.dist(raw[:2],anchor[:2])
        yaw=abs(wrap(raw[2]-anchor[2]))
        prediction.update(elapsed_s=elapsed,displacement_m=distance,yaw_change_rad=yaw,
                          source_amcl_age_s=now-source)
        if (not -.05<=elapsed<=MOTION_PREDICTION_MAX_S
                or distance>MOTION_PREDICTION_MAX_M or yaw>MOTION_PREDICTION_MAX_YAW_RAD):
            prediction.update(ended=True,end_reason='bounded motion onset prediction expired')
            return False,prediction['end_reason']
        scored_stamp=self.scan_match.get('scan_stamp_s')
        if (scored_stamp is None or not -.05<=now-scored_stamp<=SCAN_MAX_AGE_S
                or not self.scan_match.get('ready',False)):
            return False,'motion onset prediction needs fresh matching LiDAR'
        metrics=snapshot['particle_metrics']
        try:
            validate_runtime_particles(metrics)
        except ValueError as exc:
            prediction.update(ended=True,end_reason=str(exc))
            return False,prediction['end_reason']
        return True,'bounded odometry prediction from a live-scan-validated parked AMCL posterior'

    def score_scan_fit(self, now, current_map_odom, *, scan=None):
        scan=self.latest_scan if scan is None else scan
        if (self.scan_checker is None or scan is None or current_map_odom is None
                or not -.05<=now-self.stamp(scan)<=SCAN_MAX_AGE_S):
            return dict(ready=False,reason='waiting for fresh LiDAR and selected map for scan-map fit')
        try:
            transform=self.tf_buffer.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp))
            odom_sensor=planar_pose(transform.transform.translation,transform.transform.rotation)
            map_sensor=compose(current_map_odom,odom_sensor)
            if self.continuous_localization:
                result=self.scan_checker.score(scan,map_sensor,max_samples=LIVE_SCAN_MAX_SAMPLES)
            else:
                result=self.scan_checker.score(scan,map_sensor)
            result['scan_stamp_s']=self.stamp(scan)
            if self.stationary_inspection:
                try:
                    mount=self.tf_buffer.lookup_transform('base_footprint',scan.header.frame_id,
                                                          Time.from_msg(scan.header.stamp))
                    base_sensor=planar_pose(mount.transform.translation,mount.transform.rotation)
                    result['trusted_sectors']=self.scan_checker.score_trusted_sectors(scan,map_sensor,base_sensor)
                except (TransformException,ValueError) as exc:
                    result['trusted_sectors']=dict(ready=False,reason=str(exc))
            return result
        except TransformException as exc:
            return dict(ready=False,pending_capture_tf=True,
                        reason='scan-map fit transform unavailable: '+str(exc))
        except ValueError as exc:
            return dict(ready=False,scan_stamp_s=self.stamp(scan),
                        reason='invalid scan/map geometry or transform: '+str(exc))

    def check_scan_fit(self, now):
        observation = self.latest_observation
        if observation is None or (not self.stationary_cache_enabled
                                  and not -.05<=now-observation[0]<=AMCL_MAX_AGE_S):
            self.scan_match=dict(ready=False,reason='waiting for fresh AMCL for scan-map fit')
            return
        observation_stamp,current_map_odom = observation
        key=(self.scan_stamp,observation_stamp)
        if key==self.last_fit_key:
            if self.scan_stamp is None or not -.05<=now-self.scan_stamp<=SCAN_MAX_AGE_S:
                self.scan_match=dict(ready=False,reason='LiDAR scan is stale for scan-map fit')
            return
        result=self.score_scan_fit(now,current_map_odom)
        # A capture-time TF may arrive just after this scan. Retry the same
        # scan until it can be scored, while retaining the normal age limit.
        if result.get('scan_stamp_s')==self.scan_stamp:
            self.last_fit_key=key
            self.scan_match=result
            self.last_scored_scan=self.latest_scan
            self.last_scored_match=copy.deepcopy(result)
            self.last_scored_observation_stamp=observation_stamp
        elif (self.continuous_localization and result.get('pending_capture_tf') is True
                and self.last_scored_scan is not None):
            previous_stamp=self.stamp(self.last_scored_scan)
            if -.05<=now-previous_stamp<=SCAN_MAX_AGE_S:
                previous=self.last_scored_match
                if observation_stamp!=self.last_scored_observation_stamp:
                    # A changed map correction must be checked at the old
                    # scan's exact capture TF, never reuse a score for another
                    # pose. A genuinely bad completed score remains bad.
                    previous=self.score_scan_fit(now,current_map_odom,scan=self.last_scored_scan)
                    self.last_scored_match=copy.deepcopy(previous)
                    self.last_scored_observation_stamp=observation_stamp
                self.scan_match=copy.deepcopy(previous)
                self.scan_match.update(latest_scan_pending_tf=True,
                    pending_scan_stamp_s=self.scan_stamp)
            else:
                self.scan_match=result
        else:
            self.scan_match=result

    def tick(self):
        if not self.context.ok():
            return
        now = self.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        dt = max(0., min(.1, wall-self.last_tick))
        self.last_tick = wall
        self.poll_services(wall)
        if (self.stationary_inspection and self.correction.fault and self.raw_odom is not None
                and self.odom_stamp is not None and -.05<=now-self.odom_stamp<=ODOM_MAX_AGE_S
                and abs(self.raw_odom.twist.twist.linear.x)<.005
                and abs(self.raw_odom.twist.twist.angular.z)<.01):
            # Inspection has no frozen target or controller. A new hypothesis
            # may reacquire here; a moving trial must keep its latched fault.
            self.correction=TrialCorrection()
            self.pending_issue=''
        work_started=time.monotonic()
        self.process_pending(now)
        pending_done=time.monotonic()
        self.check_scan_fit(now)
        self.processing_times=dict(particle_observation_s=pending_done-work_started,
                                   scan_fit_s=time.monotonic()-pending_done)
        stationary=self.robot_is_stationary(now)
        cached=False
        hold_reason='stationary cache disabled'
        if self.stationary_cache_enabled and self.scan_stamp is not None:
            scored_stamp=self.scan_match.get('scan_stamp_s')
            if not self.continuous_localization and scored_stamp is None:
                scored_stamp=self.scan_stamp
            if scored_stamp is not None:
                if self.continuous_localization:
                    # Raw capture continuity is checked in on_scan. Delayed
                    # capture-time TF can pause scoring without losing scans;
                    # that clears confidence but must not masquerade as a
                    # physical sensor gap requiring another AMCL update.
                    self.stationary_evidence.observe_scan(scored_stamp,self.scan_match,now,
                                                         check_capture_continuity=False)
                else:
                    self.stationary_evidence.observe_scan(scored_stamp,self.scan_match,now)
            cached,hold_reason=self.stationary_evidence.ready(now)
            if self.continuous_localization:
                self._remember_continuous_fault(now)
                cached=cached and stationary
            if cached and self.correction.baseline is None and self.correction.last_transform is not None:
                self.correction.baseline=self.correction.last_transform
                self.correction.target=self.correction.applied=(0.,0.,0.)
                self.correction.reason='stationary AMCL snapshot validated by fresh LiDAR/odom; covariance preserved'
        live,live_reason=self.continuous_live_ready(now)
        predicted,prediction_reason=self.motion_prediction_ready(now)
        if self.continuous_localization and live and self.correction.baseline is None:
            # Navigation may begin with a live observation before a stationary
            # window. The snapshot and current scan provide the measurement;
            # no synthetic additional AMCL observations are generated.
            self.correction.baseline=self.correction.last_transform
            self.correction.target=self.correction.applied=(0.,0.,0.)
        ready,reason = self.correction.ready(now, self.scan_stamp, self.odom_stamp,
            stationary_cached=cached,motion_predicted=predicted,
            external_odom=self.external_control_odom and not self.navigation_monitor)
        if not self.active or not self.map_received:
            ready,reason = False,(self.last_state_error or
                'waiting for active AMCL and map; set initial pose in RViz or CLI')
        if self.pending_issue and not (self.continuous_localization and (cached or live or predicted)):
            ready,reason = False,self.pending_issue
        if self.global_localization:
            trusted,global_reason=self.global_gate.ready(now)
            if self.stationary_cache_enabled:
                trusted,global_reason=cached,hold_reason
            if self.continuous_localization:
                trusted=cached or live or predicted
                global_reason=hold_reason if cached else live_reason if live else prediction_reason
            if not self.global_initialized or not trusted or self.global_fault:
                ready,reason=False,self.global_fault or global_reason
        if self.raw_odom is None:
            ready,reason = False,'waiting for valid EKF odom'
        if not self.scan_match.get('ready',False):
            ready,reason=False,self.scan_match.get('reason','waiting for LiDAR/map fit')
        if self.stationary_inspection:
            metrics=self.global_gate.metrics
            if metrics:
                try:
                    validate_runtime_particles(metrics)
                except ValueError as exc:
                    ready,reason=False,str(exc)
            fit=self.scan_match
            if (fit.get('inlier_ratio',0.)<.80 or fit.get('median_distance_m') is None
                    or fit.get('median_distance_m',math.inf)>.05+1e-9):
                ready,reason=False,'inspection scan/map fit is below 80% / 5 cm limits'
            if self.stationary_cache_enabled and not cached:
                ready,reason=False,self.pending_issue or hold_reason
        if self.continuous_localization:
            metrics=(self.stationary_snapshot or {}).get('particle_metrics') or {}
            try:
                validate_runtime_particles(metrics)
            except ValueError as exc:
                ready,reason=False,str(exc)
            if not cached and not live and not predicted:
                ready,reason=False,live_reason if not stationary else hold_reason
            elif ready:
                reason=hold_reason if cached else live_reason if live else prediction_reason
            if ready and cached:
                self.last_parked_validation=dict(validated_ros_s=now,
                    odom_xyyaw=planar_pose(self.raw_odom.pose.pose.position,self.raw_odom.pose.pose.orientation),
                    source_amcl_stamp_s=self.stationary_snapshot['amcl_stamp_s'],
                    map_signature=self.map_signature)
        if not self.active and self.last_state_error is not None:
            # Do not misreport a transport failure as a map/particle failure
            # after the independent geometric gates above have run.
            ready,reason=False,self.last_state_error
        if ready:
            applied = self.correction.step(dt)
            if not self.stationary_inspection and not self.navigation_monitor and not self.external_control_odom:
                raw = self.raw_odom
                pose = compose(applied, planar_pose(raw.pose.pose.position,raw.pose.pose.orientation))
                message = copy.deepcopy(raw)
                message.header.frame_id = FRAME
                message.child_frame_id = 'base_footprint'
                message.pose.pose.position.x, message.pose.pose.position.y = pose[:2]
                message.pose.pose.orientation.x = message.pose.pose.orientation.y = 0.
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(pose[2]/2),math.cos(pose[2]/2)
                message.pose.covariance = rotate_covariance(raw.pose.covariance, applied[2])
                # Preserve the conservative EKF + AMCL displayed bound.
                message.pose.covariance[0] += self.correction.xy_std**2
                message.pose.covariance[7] += self.correction.xy_std**2
                message.pose.covariance[35] += math.radians(self.correction.yaw_std_deg)**2
                self.odom_pub.publish(message)
        pending_translation = math.dist(self.correction.applied[:2],self.correction.target[:2])
        pending_yaw_deg = abs(math.degrees(wrap(self.correction.target[2]-self.correction.applied[2])))
        status = dict(run_id=self.run_id, ready=ready, reason=reason, frame_id=FRAME,
                      published_ros_s=now, map_file=self.map_file,
                      correction_xyyaw=list(self.correction.applied),
                      raw_correction_xyyaw=list(self.correction.target),
                      baseline_map_odom=(list(self.correction.baseline) if self.correction.baseline is not None else None),
                      amcl_stamp_s=self.correction.amcl_stamp,
                      xy_std_m=self.correction.xy_std, yaw_std_deg=self.correction.yaw_std_deg,
                      covariance_limits=dict(xy_std_m=.10,yaw_std_deg=5.),
                      covariance_model='EKF rotated plus AMCL diagonal bound',
                      correction_pending_translation_m=pending_translation,
                      correction_pending_yaw_deg=pending_yaw_deg,
                      correction_settled=pending_translation <= .005 and pending_yaw_deg <= .2,
                      correction_limits=dict(translation_m=.20,yaw_deg=5.,translation_rate_mps=.03,yaw_rate_degps=.5),
                      physical_alignment_verified=False,
                      covariance_is_accuracy_proof=False)
        status['scan_map_fit']=self.scan_match
        status['stationary_cache']=dict(enabled=self.stationary_cache_enabled,validated=cached,
            reason=hold_reason,original_amcl_stamp_s=self.correction.amcl_stamp,
            amcl_observation_age_s=(now-self.correction.amcl_stamp if self.correction.amcl_stamp is not None else None),
            original_covariance_preserved=True,repeated_nomotion_updates=False if self.stationary_cache_enabled else True,
            nomotion_request_count=self.nomotion_request_count,
            interfaces_matched_since_wall_s=self.nomotion_interfaces_since)
        status['stationary_snapshot']=self.stationary_snapshot
        status['pending_observation_issue']=self.pending_issue
        status['localization_fault']=self.correction.fault or self.global_fault
        status['map_update_event']=self.correction.last_jump
        status['navigation_recovery']=dict(self.correction.navigation_recovery)
        status['refresh_conditions']=self.refresh_conditions
        status['lifecycle_services']=self.state_poll.diagnostics(wall)
        # Distinguish delayed delivery from a posterior waiting for capture-
        # time TF/particle pairing. These timestamps are diagnostics only;
        # they never promote an unvalidated observation to a usable pose.
        status['amcl_pipeline']=dict(
            latest_received_pose_stamp_s=self.last_received_amcl_stamp,
            latest_pose_received_at_ros_s=self.last_received_amcl_at,
            latest_accepted_pose_stamp_s=self.correction.amcl_stamp,
            latest_cloud_stamp_s=(self.latest_cloud[0] if self.latest_cloud is not None else None),
            pending_pose_count=len(self.pending),pending_reason=self.pending_issue)
        status['stationary_inspection']=self.stationary_inspection
        status['continuous_localization']=self.continuous_localization
        status['navigation_monitor']=self.navigation_monitor
        status['external_control_odom']=self.external_control_odom
        if self.external_control_odom:
            status['control_status_topic']='/pallet/r3/control_odom/status'
        status['ekf_freshness_owner']=('r3_control_odom' if self.external_control_odom and not self.navigation_monitor
                                       else 'r3_amcl_trial_bridge')
        status['supervisor_ekf_capture_age_s']=(now-self.odom_stamp if self.odom_stamp is not None else None)
        status['processing_times_s']=self.processing_times
        status['role']=('navigation_monitor' if self.navigation_monitor else
                        'stationary_inspection' if self.stationary_inspection else 'trial')
        status['trial_handoff']=self.trial_handoff
        status['delivery_handoff']=self.delivery_handoff
        status['trial_handoff_at_ros_s']=(self.trial_handoff['at_ros_s'] if self.trial_handoff is not None else None)
        status['robot_stationary']=stationary
        status['live_observation_validated']=bool(live and ready)
        status['moving_live_validated']=bool(live and ready and not stationary)
        status['motion_prediction_validated']=bool(predicted and ready)
        status['motion_prediction']=(dict(self.motion_prediction,validated=bool(predicted and ready),
            reason=prediction_reason) if self.motion_prediction is not None else None)
        status['current_map_odom_xyyaw']=(list(self.latest_observation[1])
            if self.latest_observation is not None else None)
        status['current_map_pose_xyyaw']=(list(compose(self.latest_observation[1],
            planar_pose(self.raw_odom.pose.pose.position,self.raw_odom.pose.pose.orientation)))
            if self.latest_observation is not None and self.raw_odom is not None else None)
        status['stationary_refresh_armed']=self.parking_refresh_armed
        status['global_localization']=dict(enabled=self.global_localization,
            requested=self.global_requested,service_completed=self.global_initialized,
            requested_ros_s=self.global_requested_ros_s,
            particle_gate_ready=self.global_gate.ready(now)[0],
            particle_gate_reason=self.global_gate.reason,
            particle_metrics=self.global_gate.metrics,
            latest_cloud_stamp_s=(self.latest_cloud[0] if self.latest_cloud is not None else None),
            autonomous_motion_for_localization=False)
        self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
        if wall-self.last_marker_wall >= .5:
            label=Marker()
            label.header.frame_id='map'
            label.header.stamp=self.get_clock().now().to_msg()
            label.ns='localization_health';label.id=0
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
            label.pose.orientation.w=1.
            if self.latest_observation is not None and self.raw_odom is not None:
                map_pose=compose(self.latest_observation[1],planar_pose(
                    self.raw_odom.pose.pose.position,self.raw_odom.pose.pose.orientation))
                label.pose.position.x,label.pose.position.y=map_pose[0]+1.,map_pose[1]+.55
            label.pose.position.z=1.1
            label.scale.z=.07
            label.color.a=1.;label.color.r=.1 if ready else 1.
            label.color.g=.8 if ready else .1;label.color.b=.1
            match=self.scan_match.get('inlier_ratio')
            fit=f' | scan-map {match*100:.0f}%' if isinstance(match,(float,int)) else ''
            label.text=('AMCL HOLD' if ready and cached else
                        'AMCL READY' if ready else 'AMCL NOT READY')+fit
            self.marker_pub.publish(MarkerArray(markers=[label]))
            self.last_marker_wall=wall
        if not ready and wall-self.last_progress_log>=2.:
            progress=reason
            if self.global_localization and self.global_gate.metrics is not None:
                metrics=self.global_gate.metrics
                progress+=(f"; particles stdXY={metrics['xy_std_m']:.3f} m, "
                           f"stdYaw={metrics['yaw_std_deg']:.1f} deg, "
                           f"massNearPose={metrics['dominant_mass']:.1%}")
            if self.stationary_cache_enabled and self.nomotion_request_count==0:
                progress+='; startup refresh '+json.dumps(self.refresh_conditions,sort_keys=True)
            self.get_logger().info('Waiting for localization: '+progress)
            self.last_progress_log=wall
        if ready and self.args.wait_ready is not None and not self.ready_file_written:
            path = self.args.wait_ready
            path.parent.mkdir(parents=True,exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.tmp')
            temporary.write_text(json.dumps(status,indent=2,allow_nan=False)+'\n')
            temporary.replace(path)
            self.ready_file_written = True
            self.get_logger().info(f'AMCL trial baseline ready; saved {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--initial-pose',type=float,nargs=3,metavar=('X','Y','YAW_DEG'))
    parser.add_argument('--global-localization',action='store_true',
                        help='Search the full selected map once; require unambiguous full particle cloud before ready')
    parser.add_argument('--global-seed-ready',action='store_true',
                        help='A fresh stationary global scan search initialized AMCL; validate clouds without resetting its pose')
    parser.add_argument('--stationary-inspection',action='store_true',
                        help='Allow stationary reacquisition and withhold trial control odometry')
    parser.add_argument('--continuous-localization',action='store_true',
                        help='Validate parked snapshots and live moving AMCL; refresh once per genuine parked episode')
    parser.add_argument('--navigation-monitor',action='store_true',
                        help='Monitor continuous localization for Nav2 without publishing short-trial control odometry')
    parser.add_argument('--resume-delivery',action='store_true',
                        help='Allow loaded navigation handoff after validating a new parked session')
    parser.add_argument('--external-control-odom',action='store_true',
                        help='A separate r3_control_odom process owns fresh EKF-to-trial odometry after handoff')
    parser.add_argument('--wait-ready',type=Path,
                        help='Write first ready baseline JSON atomically, then keep publishing')
    args = parser.parse_args()
    if args.external_control_odom and not (args.continuous_localization or args.navigation_monitor):
        parser.error('--external-control-odom requires continuous localization')
    if args.resume_delivery and not args.navigation_monitor:
        parser.error('--resume-delivery requires --navigation-monitor')
    if not args.map.is_absolute() or not args.map.is_file():
        parser.error('--map must be the absolute path of the selected map YAML')
    if args.initial_pose is not None and not all(math.isfinite(v) for v in args.initial_pose):
        parser.error('initial pose must be finite')
    if args.global_localization and args.initial_pose is not None:
        parser.error('--global-localization and --initial-pose are mutually exclusive')
    if args.global_seed_ready and (args.global_localization or args.initial_pose is not None):
        parser.error('--global-seed-ready cannot reset or override an initial pose')
    if args.wait_ready is not None and args.wait_ready.exists():
        parser.error('--wait-ready path already exists; choose a new run directory')
    rclpy.init()
    node = AmclTrialBridge(args)
    node.get_logger().info('Localization only: no trajectory or motor commands; '
                          +('validating fresh global scan seed' if args.global_seed_ready else
                            'global map search selected' if args.global_localization else 'explicit map initial pose required'))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (_rclpy.RCLError,RuntimeError):
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
