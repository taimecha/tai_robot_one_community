#!/usr/bin/env python3
"""Capture a fresh stationary RGB-D target and build one provisional real trial plan.

This process has no command publisher. It records the measured camera result,
the odom pose at capture, and the assumptions used for the first supervised run.
"""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import re
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from detected_pallets_interfaces.msg import PalletTarget

from r3_core import (R3_PLANNED_C_CLEARANCE_M, FrozenPlan, PlanReferenceError,
                     compose, dense, digest, errors, fork_for_base, goal_base,
                     validate_phases, validate_plan_reference, wrap)
from r3_approach_planner import minimum_jerk_approach
from r3_tracking import predict_tracking, write_csv
from r3_localization_contract import checked_status


ROOT = Path(__file__).resolve().parents[1]
RAW_PATTERN = re.compile(r'(?:^|; )lateral_camera_raw_m=([+-]?[0-9.]+); '
                         r'lateral_total_bias_m=([+-]?[0-9.]+); '
                         r'lateral_fork_m=([+-]?[0-9.]+)')
HEADING_PATTERN = re.compile(r'(?:^|; )heading_source=floor_referenced_plane; '
                             r'heading_depth_x_slope=([+-]?[0-9.]+)')


def provisional_geometry():
    """Use documented robot dimensions without falsely calling them surveyed."""
    geometry = json.loads((ROOT / 'simulation/r3/robot_geometry.json').read_text())
    geometry.update(provenance='USER_ACCEPTED_APPROXIMATE_REAL_TRIAL',
                    confirmed_real_geometry=False,
                    approximate_real_trial=True,
                    max_speed=.10, max_lateral=.009,
                    base_fork=[.3655, 0., 0.], housing_y=.02)
    geometry['source_notes']['trial'] = ('Fork centered per user and URDF; camera eye +25 mm per user. '
        'Tine centers, pillar centers and absolute camera yaw remain approximate.')
    return geometry


def load_axis_lateral_calibration():
    path = ROOT/'simulation/r3/real_trial_lateral_calibration.json'
    profile = json.loads(path.read_text())
    residual = profile.get('pallet_axis_lateral_residual_m')
    if (profile.get('frame') != 'pallet_entry_axis_positive_left'
            or isinstance(residual, bool) or not isinstance(residual, (int, float))
            or not math.isfinite(residual) or abs(residual) > .05):
        raise ValueError('invalid empirical pallet-axis lateral calibration')
    return profile


def snapshot_from_samples(samples, session, *, lateral_residual_m=0., lateral_calibration=None):
    if not math.isfinite(lateral_residual_m) or abs(lateral_residual_m) > .05:
        raise ValueError('invalid empirical pallet-axis lateral residual')
    if len(samples) < 5 or samples[-1]['stamp'] - samples[0]['stamp'] < .2:
        raise ValueError('need five distinct RGB-D captures over at least 0.2 s')
    if any(b['stamp'] <= a['stamp'] for a, b in zip(samples, samples[1:])):
        raise ValueError('nonmonotonic RGB-D captures')
    stopped = session.get('yolo_stopped_ros_s')
    if stopped is not None and samples[-1]['stamp'] <= stopped:
        raise ValueError('need a fresh RGB-D capture after YOLO stopped')
    if samples[-1]['stamp']-samples[0]['stamp'] > 4.0:
        raise ValueError('RGB-D capture window is too long')
    if any(abs(s['raw']+.025-s['lateral']) > .00015 for s in samples):
        raise ValueError('camera lateral correction is not exactly one +2.5 cm offset')
    if any(s.get('heading_source') != 'floor_referenced_plane'
           or not math.isfinite(s.get('heading_depth_x_slope', float('nan')))
           for s in samples):
        raise ValueError('fresh floor-referenced pallet heading is required')
    if any(max(s[k] for s in samples)-min(s[k] for s in samples) > limit
           for k, limit in (('forward', .01), ('lateral', .01), ('heading', math.radians(.5)))):
        raise ValueError('RGB-D result is still moving/noisy')
    start = samples[-1]['odom']
    if (max(math.dist(s['odom'][:2], start[:2]) for s in samples) > .005
            or max(abs(wrap(s['odom'][2]-start[2])) for s in samples) > math.radians(.3)):
        raise ValueError('robot was moving during capture')
    forward = statistics.median(s['forward'] for s in samples)
    lateral = statistics.median(s['lateral'] for s in samples)
    heading = statistics.median(s['heading'] for s in samples)
    raw = statistics.median(s['raw'] for s in samples)
    camera_mount_target = compose(start, (forward, lateral, heading))
    # Correct the entry axis once along its own left normal. This changes
    # signed perpendicular error by exactly -residual at any robot/pallet yaw;
    # it is separate from the physical +25 mm camera mount offset above.
    target = compose(camera_mount_target, (0., lateral_residual_m, 0.))
    target_forward = forward-lateral_residual_m*math.sin(heading)
    target_lateral = lateral+lateral_residual_m*math.cos(heading)
    shared_depth = all(s.get('depth_correction_shared', False) for s in samples)
    projection = dict(
        depth_correction_shared=shared_depth,
        camera_pitch_deg=samples[-1].get('camera_pitch_deg', 20.),
        depth_reference_u=samples[-1].get('depth_reference_u', 370.),
        camera_mount_lateral_m=.025,
        camera_mount_yaw_deg=0., camera_mount_yaw_verified=False,
        camera_roll_assumed_zero=True, camera_roll_verified=False,
        forward_bias_m=samples[-1].get('forward_bias_m', .038429884910583446),
        forward_bias_verified_with_current_depth_model=False)
    projection['axis_lateral_calibration'] = lateral_calibration or dict(
        pallet_axis_lateral_residual_m=lateral_residual_m,
        frame='pallet_entry_axis_positive_left')
    report = dict(camera_lateral_raw_m=raw, camera_mount_offset_m=.025,
                  fork_lateral_corrected_m=lateral, forward_base_m=forward,
                  pallet_axis_lateral_residual_m=lateral_residual_m,
                  target_forward_after_axis_residual_m=target_forward,
                  target_lateral_after_axis_residual_m=target_lateral,
                  entry_heading_deg=math.degrees(heading),
                  heading_source='floor_referenced_plane',
                  heading_depth_x_slope=statistics.median(
                      s['heading_depth_x_slope'] for s in samples),
                  position_depth_x_slope=statistics.median(
                      s.get('position_depth_x_slope', .00015807) for s in samples),
                  calibration_assumptions=projection)
    return dict(frame_id='odom', feature='front_entry_corridor_center',
                pose=list(target), source='stationary_real_RGBD',
                capture_stamp_s=samples[-1]['stamp'],
                calibration_hash=digest(dict(projection=projection,
                                            heading_source='floor_referenced_plane')),
                raw_session_hash=digest(samples), yolo_off_ack=True,
                applied_bias={'lateral_bias_m': .025,
                              'pallet_axis_lateral_residual_m': lateral_residual_m},
                measurement_report=report,
                accuracy_claim='provisional_camera_result; no independent absolute yaw proof'), start


def rgbd_qualified(status, samples, now_ros):
    """A YOLO seed plus a fresh post-off sample, with recent stationary geometry.

    The detector's five-second health ratio includes intermittent invalid frames
    and can never recover on this Pi even when the measured pallet is stable.
    Validate the actual captures used by the plan instead.
    """
    stopped = status.get('yolo_stopped_ros_s')
    return (status.get('enabled') is False and status.get('busy') is False
            and status.get('detections', 0) >= 3 and stopped is not None
            and len(samples) >= 5 and samples[-1]['stamp'] > stopped
            and -.05 <= now_ros-samples[-1]['stamp'] <= .35
            and samples[-1]['stamp']-samples[0]['stamp'] <= 4.0)


def reacquire_needed(status):
    """A lost pre-plan RGB-D track needs a new YOLO seed, never a cached pose."""
    return (status.get('phase') == 'CALIBRATION_REQUIRED'
            and status.get('enabled') is False
            and status.get('busy') is False
            and status.get('yolo_stopped_ros_s') is not None
            and 'RGB-D mất dấu' in status.get('reason', ''))


def perception_stall(status, now_ros, receive_age, elapsed):
    """Recover a stuck pre-motion detector; never extend image/pose freshness."""
    if elapsed < 5.:
        return None
    if not status or receive_age > 3.:
        return 'YOLO recognition heartbeat unavailable for more than 3 seconds'
    published = status.get('published_ros_s')
    if published is not None and not -.05 <= now_ros-published <= 3.:
        return 'YOLO recognition heartbeat source is stale'
    if status.get('busy') is True and status.get('inference_busy_s', 0.) > 5.:
        return 'YOLO inference has not finished for more than 5 seconds'
    if status.get('enabled') is True and 'rgb_receive_age_s' in status:
        age = status['rgb_receive_age_s']
        if age is None or age > 3.:
            return 'YOLO is enabled but receives no fresh camera RGB stream'
        if status.get('inference_attempts') == 0 and status.get('elapsed_s', 0.) > 8.:
            return 'YOLO receives RGB but has not started inference'
    return None


def build_plan(snapshot, start, geometry, output):
    # An invalid coordinate contract is independent of curve shape or camera
    # noise. Reject it before the expensive four-candidate refinement search.
    validate_plan_reference(snapshot, dict(localization=snapshot.get('localization')))
    target = snapshot['pose']
    start_tip_clearance = errors(start, target, geometry)['gap_m'] - geometry['fork_length']
    if start_tip_clearance <= R3_PLANNED_C_CLEARANCE_M + 1e-6:
        raise ValueError('START_TOO_CLOSE_FOR_C: đầu càng đang quá gần pallet; '
                         'đưa xe lùi về A để còn khoảng A→C trước khi thử lại')
    attempts = []
    # Prefer 2.5 seconds of straight heading feedback before C. At an A pose
    # accepted within 10 cm, a fixed 25 cm lead can leave too little curved
    # approach to remove the lateral error within the turning-radius limit.
    # Keep the longest feasible lead, with the previously supported 12 cm as
    # the lower bound. Every candidate still passes the complete fork sweep
    # and predicted-follower checks; C->B stays the same zero-heading line.
    lead_candidates = (.25, .20, .15, .12)
    for straight_lead_in_m, extra in ((lead, extra) for lead in lead_candidates
            for extra in (R3_PLANNED_C_CLEARANCE_M, .20, .25, .30)):
        gap = geometry['fork_length'] + extra
        c = goal_base(target, geometry, gap)
        curve_end = goal_base(target, geometry, gap+straight_lead_in_m)
        b = goal_base(target, geometry)
        try:
            curve, curvature, overshoot, shape = minimum_jerk_approach(
                start, curve_end, target, geometry, radius_bound=.6,
                curvature_cap=1/.6)
            approach = [*curve, *list(dense([curve_end, c], step=.01))[1:]]
            phases = [dict(type='APPROACH', poses=approach,
                           straight_lead_in_m=straight_lead_in_m,
                           tracking_controller='pose_feedback'),
                      dict(type='STRAIGHT', stop_at_join=True,
                           poses=list(dense([c, b], step=.01)))]
            validate_phases(phases, target, geometry)
            rows, follower = predict_tracking(phases, target, geometry, start)
            if follower.state != 'STOPPED' or abs(rows[-1]['ey_m']) > geometry['max_lateral']:
                raise ValueError('predicted odom follower did not reach the lateral goal')
            config = dict(model='provisional_real_odom_trial',
                          approach_refinement=dict(
                              method='bounded fork-root shape search on clamped quintic y(x)',
                              selected_C_unchanged=True, terminal_line_unchanged=True,
                              straight_lead_in_m=straight_lead_in_m,
                              preferred_straight_lead_in_m=lead_candidates[0],
                              minimum_straight_lead_in_m=lead_candidates[-1],
                              max_curvature_1pm=curvature, fork_overshoot_m=overshoot,
                              shape_coefficients=shape),
                          tracking_controller='curvature feedforward + odom heading and lateral feedback',
                          heading_source='odom_ekf',
                          geometry_hash=digest(geometry))
            if snapshot.get('localization'):
                config['localization'] = snapshot['localization']
            plan = FrozenPlan.create(snapshot, geometry, phases, config)
            output.mkdir(parents=True, exist_ok=False)
            (output/'frozen_plan.json').write_text(json.dumps(dict(vars(plan), plan_id=plan.plan_id), indent=2))
            (output/'target_snapshot.json').write_text(json.dumps(snapshot, indent=2))
            write_csv(output/'path_planned_base.csv', [
                dict(phase=p['type'], x_m=v[0], y_m=v[1], yaw_rad=v[2])
                for p in phases for v in p['poses']])
            write_csv(output/'path_planned_fork_root.csv', [
                dict(phase=p['type'], x_m=fork_for_base(v, geometry)[0],
                     y_m=fork_for_base(v, geometry)[1], yaw_rad=fork_for_base(v, geometry)[2])
                for p in phases for v in p['poses']])
            write_preview_svg(output/'trajectory.svg', phases, target, geometry)
            return plan, phases, rows
        except PlanReferenceError:
            raise
        except ValueError as exc:
            attempts.append(f'lead {straight_lead_in_m:.2f} m, C gap {gap:.3f} m: {exc}')
    raise ValueError('no feasible A→C→B path: ' + '; '.join(attempts))


def write_preview_svg(path, phases, target, geometry):
    base = [v for phase in phases for v in phase['poses']]
    fork = [fork_for_base(v, geometry) for v in base]
    half = geometry['pallet_width']/2
    pallet = [compose(target, (x, y, 0.)) for x, y in
              ((0., -half), (geometry['pallet_length'], -half),
               (geometry['pallet_length'], half), (0., half))]
    all_points = [*base, *fork, *pallet]
    x0, x1 = min(v[0] for v in all_points)-.15, max(v[0] for v in all_points)+.15
    y0, y1 = min(v[1] for v in all_points)-.20, max(v[1] for v in all_points)+.20
    scale = min(920/(x1-x0), 540/(y1-y0))
    def point(p):
        return f'{60+(p[0]-x0)*scale:.1f},{600-(p[1]-y0)*scale:.1f}'
    def poly(points):
        return ' '.join(point(p) for p in points)
    a, c, b = base[0], phases[1]['poses'][0], base[-1]
    labels = ''.join(f'<circle cx="{point(p).split(",")[0]}" cy="{point(p).split(",")[1]}" r="5" fill="#172554"/>'
                     f'<text x="{float(point(p).split(",")[0])+8:.1f}" y="{float(point(p).split(",")[1])-8:.1f}" '
                     f'font-size="20" fill="#172554">{name}</text>'
                     for name, p in (('A', a), ('C', c), ('B', b)))
    path.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="1040" height="680" viewBox="0 0 1040 680">
<rect width="1040" height="680" fill="#f8fafc"/>
<text x="40" y="35" font-size="22" font-family="sans-serif" fill="#0f172a">RGB-D live A → C → B · odom planned</text>
<polygon points="{poly(pallet)}" fill="#25a18e" fill-opacity=".45" stroke="#0f766e" stroke-width="2"/>
<polyline points="{poly(base)}" fill="none" stroke="#2563eb" stroke-width="3"/>
<polyline points="{poly(fork)}" fill="none" stroke="#e11d48" stroke-width="3"/>
{labels}
<text x="40" y="645" font-size="17" font-family="sans-serif" fill="#2563eb">Blue: base_footprint</text>
<text x="300" y="645" font-size="17" font-family="sans-serif" fill="#e11d48">Red: fork root</text>
<text x="570" y="645" font-size="17" font-family="sans-serif" fill="#475569">Provisional geometry · no motion measured</text>
</svg>''')


class Capture(Node):
    def __init__(self, localization='odom'):
        super().__init__('r3_live_stationary_capture')
        self.status = None
        self.status_wall = 0.
        self.odom = None
        self.odom_stamp = 0.
        self.odom_history = deque(maxlen=64)
        self.pending_targets = deque(maxlen=4)
        self.capture_rejections = Counter()
        self.capture_last_issue = ''
        self.capture_accepted_samples = 0
        self.localization_identity = None
        self.control_localization_identity = None
        self.localization_mode = localization
        self.localization_status = None
        self.control_localization_status = None
        self.samples = deque(maxlen=32)
        self.last_target_wall = 0.
        self.yolo_off_wall = 0.
        self.create_subscription(String, '/pallet/recognition/status', self.on_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PalletTarget, '/pallet/target', self.on_target, 10)
        pose_topic = '/pallet/r3/localized_odom' if localization == 'amcl' else '/odom'
        self.create_subscription(Odometry, pose_topic, self.on_odom, qos_profile_sensor_data)
        if localization == 'amcl':
            self.create_subscription(String, '/pallet/r3/localization/status',
                self.on_localization, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String,'/pallet/r3/control_odom/status',self.on_control_localization,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_control_localization(self,msg):
        try:
            self.control_localization_status=checked_status(msg.data,self.get_clock().now().nanoseconds*1e-9)
            self.observe_reference(self.control_localization_status, control=True)
        except (ValueError,TypeError,KeyError):
            self.control_localization_status=None

    def on_localization(self, msg):
        try:
            now = self.get_clock().now().nanoseconds*1e-9
            status = checked_status(msg.data, now)
            self.observe_reference(status)
            self.localization_status = status
        except (ValueError, TypeError, KeyError):
            self.localization_status = None
            self.samples.clear()

    def observe_reference(self, status, control=False):
        """A map/run change invalidates capture-time pose witnesses too."""
        identity = (status['run_id'], status['map_file'], tuple(status['baseline_map_odom']))
        key = 'control_localization_identity' if control else 'localization_identity'
        previous = getattr(self, key)
        if previous is not None and previous != identity:
            self.samples.clear()
            self.pending_targets.clear()
            self.odom_history.clear()
            self.odom = None
            self.odom_stamp = 0.
            self.reject_capture('localization reference changed; awaiting new odom')
        setattr(self, key, identity)

    def reject_capture(self, reason):
        self.capture_rejections[reason] += 1
        self.capture_last_issue = reason

    def capture_diagnostics(self):
        return dict(accepted_samples=self.capture_accepted_samples,
                    pending_targets=len(self.pending_targets),
                    last_issue=self.capture_last_issue,
                    rejections=dict(self.capture_rejections))

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            if status.get('session') != (self.status or {}).get('session'):
                self.samples.clear()
                self.pending_targets.clear()
                self.last_target_wall = 0.
                self.yolo_off_wall = 0.
            if (status.get('yolo_stopped_ros_s') is not None
                    and status.get('yolo_stopped_ros_s') != (self.status or {}).get('yolo_stopped_ros_s')):
                self.yolo_off_wall = time.monotonic()
            self.status = status
            self.status_wall = time.monotonic()
        except (ValueError, TypeError):
            self.status = None

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        expected_frame = 'r3_trial' if self.localization_mode == 'amcl' else 'odom'
        if (msg.header.frame_id != expected_frame or msg.child_frame_id not in ('base_link', 'base_footprint')
                or not -.05 <= now-stamp <= .2 or abs(norm-1) > .02
                or not all(math.isfinite(v) for v in (p.x, p.y, norm,
                    msg.twist.twist.linear.x, msg.twist.twist.angular.z))):
            self.odom = None
            self.reject_capture('invalid or stale odom input')
            return
        if self.odom_history and stamp <= self.odom_history[-1][0]:
            self.reject_capture('duplicate or out-of-order odom capture')
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.odom = (p.x, p.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.odom_stamp = stamp
        self.odom_history.append((stamp, self.odom))
        while self.odom_history and stamp-self.odom_history[0][0] > 1.:
            self.odom_history.popleft()
        pending = sorted(self.pending_targets,
            key=lambda target: target.header.stamp.sec+target.header.stamp.nanosec*1e-9)
        self.pending_targets.clear()
        for target in pending:
            self.on_target(target)

    def odom_at_capture(self, stamp):
        """Pair a target to genuine pose captures within the existing 70 ms gate.

        Camera processing can finish after newer odom has already arrived.
        Interpolate only between close bracketing captures; otherwise retain
        the nearest genuine sample and report its actual time offset.
        """
        history = list(self.odom_history)
        if not history:
            return None
        before = next((item for item in reversed(history) if item[0] <= stamp), None)
        after = next((item for item in history if item[0] >= stamp), None)
        if before and after and stamp-before[0] <= .07+1e-9 and after[0]-stamp <= .07+1e-9:
            span = after[0]-before[0]
            fraction = (stamp-before[0])/span if span else 0.
            a,b = before[1],after[1]
            pose = tuple(a[i]+fraction*(b[i]-a[i]) if i != 2 else
                         wrap(a[2]+fraction*wrap(b[2]-a[2])) for i in range(5))
            return pose, dict(method='interpolated' if span else 'exact',
                odom_capture_stamps_s=[before[0],after[0]],
                maximum_capture_offset_s=max(stamp-before[0],after[0]-stamp)), (a,b)
        nearest = min(history, key=lambda item: abs(item[0]-stamp))
        if abs(nearest[0]-stamp) <= .07+1e-9:
            return nearest[1], dict(method='nearest_capture',
                odom_capture_stamps_s=[nearest[0]],
                maximum_capture_offset_s=abs(nearest[0]-stamp)), (nearest[1],)
        return None

    def queue_target(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if all(stamp != old.header.stamp.sec+old.header.stamp.nanosec*1e-9
               for old in self.pending_targets):
            self.pending_targets.append(msg)
        self.reject_capture('waiting for odom captured within 70 ms of target')

    def on_target(self, msg):
        s = self.status or {}
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if s.get('enabled') is False and not msg.valid:
            self.samples.clear()
            self.pending_targets.clear()
            self.reject_capture('RGB-D became invalid after YOLO paused')
            return
        if time.monotonic()-self.status_wall > .5:
            self.reject_capture('recognition status older than 500 ms')
            return
        if (s.get('detections', 0) < 3 or s.get('enabled') not in (True, False)
                or (s.get('enabled') is False and s.get('busy') is True)):
            self.reject_capture('waiting for semantic evidence or YOLO pause completion')
            return
        if not msg.valid or msg.header.frame_id != 'camera_color_optical_frame':
            self.reject_capture('invalid RGB-D target or frame')
            return
        if not -.05 <= now-stamp <= .25:
            self.reject_capture('RGB-D target capture older than 250 ms or future dated')
            return
        if isinstance(s.get('started_ros_s'), (int,float)) and stamp < s['started_ros_s']:
            self.reject_capture('target belongs to an earlier recognition session')
            return
        if (not math.isfinite(msg.rgb_depth_delta_ms) or abs(msg.rgb_depth_delta_ms) > 50
                or min(msg.left_depth_samples, msg.right_depth_samples) < 60
                or not .172 <= msg.opening_width_m <= .332
                or not .15 <= msg.leg_center_separation_m <= .60):
            self.reject_capture('RGB-D geometry, depth support or synchronization rejected')
            return
        if self.odom is None or not -.05 <= now-self.odom_stamp <= .2:
            self.queue_target(msg)
            return
        paired = self.odom_at_capture(stamp)
        if paired is None:
            self.queue_target(msg)
            return
        capture_odom, pairing, witnesses = paired
        if any(abs(odom[3]) > .005 or abs(odom[4]) > .01
               for odom in (*witnesses, self.odom)):
            self.reject_capture('robot moving at capture or at latest odom')
            return
        match = RAW_PATTERN.search(msg.status)
        heading_match = HEADING_PATTERN.search(msg.status)
        if not match or not heading_match:
            self.reject_capture('missing shared calibration or floor-heading evidence')
            return
        raw, offset, corrected = map(float, match.groups())
        slope = float(heading_match.group(1))
        if (abs(offset-.025) > 1e-9 or abs(raw+offset-corrected) > .00015
                or abs(corrected-msg.opening_lateral_m) > .00015
                or not math.isfinite(slope)
                or not all(math.isfinite(v) for v in (msg.chassis_front_forward_m,
                    msg.entry_heading_rad, raw, corrected))):
            self.reject_capture('invalid shared depth or camera calibration evidence')
            return
        if self.samples and stamp <= self.samples[-1]['stamp']:
            self.reject_capture('duplicate or out-of-order RGB-D capture')
            return
        if self.samples and stamp-self.samples[-1]['stamp'] > 1.0:
            self.samples.clear()
        diagnostics = {}
        for name in ('position_depth_x_slope', 'camera_pitch_deg', 'depth_reference_u',
                     'forward_bias_m'):
            value = re.search(r'(?:^|; )'+name+r'=([+-]?[0-9.]+)', msg.status)
            if value and math.isfinite(float(value.group(1))):
                diagnostics[name] = float(value.group(1))
        diagnostics['depth_correction_shared'] = '; depth_correction_shared=true' in msg.status
        self.samples.append(dict(stamp=stamp, odom=list(capture_odom[:3]),
            odom_pairing=pairing,
            forward=msg.chassis_front_forward_m+.3005,
            lateral=msg.opening_lateral_m, heading=msg.entry_heading_rad,
            raw=raw, heading_source='floor_referenced_plane',
            heading_depth_x_slope=slope, **diagnostics))
        self.last_target_wall = time.monotonic()
        self.capture_accepted_samples += 1
        self.capture_last_issue = ''
        self.pending_targets = deque((target for target in self.pending_targets
            if target.header.stamp.sec+target.header.stamp.nanosec*1e-9 > stamp), maxlen=4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--localization', choices=('odom', 'amcl'), default='odom')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout < 0:
        parser.error('timeout must be finite and nonnegative (0 means unlimited)')
    if args.output.exists():
        parser.error('choose a new output directory')
    rclpy.init()
    node = Capture(args.localization)
    deadline = time.monotonic()+args.timeout if args.timeout else float('inf')
    started_wall = time.monotonic()
    last_progress_log = started_wall
    last_attempt = 0.
    last_error = ''
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            wall = time.monotonic()
            now_ros = node.get_clock().now().nanoseconds*1e-9
            issue = perception_stall(node.status, now_ros, wall-node.status_wall, wall-started_wall)
            if issue:
                print(json.dumps(dict(status='RESTART_PERCEPTION', reason=issue,
                                      last_status=node.status, robot_commands=0),
                                 ensure_ascii=False), flush=True)
                return 4
            if wall-last_progress_log >= 5.:
                state = node.status or {}
                print(json.dumps(dict(status='RECOGNIZING', phase=state.get('phase'),
                    detections=state.get('detections', 0), rgbd_samples=len(node.samples),
                    rgb_tracker_acknowledged=state.get('rgb_tracker_acknowledged'),
                    rgb_tracker_issue=(state.get('rgb_tracker') or {}).get('issue'),
                    inference_ms=state.get('inference_ms'), last_issue=state.get('last_issue', ''),
                    min_pallet_confidence=state.get('min_pallet_confidence'),
                    pallet_candidates=state.get('pallet_candidates', []),
                    capture=node.capture_diagnostics(),
                    robot_commands=0), ensure_ascii=False), flush=True)
                last_progress_log = wall
            # Allow brief tracking interruptions to recover. A lost seed with
            # no new valid measurement for two seconds starts a new session.
            if (time.monotonic()-node.status_wall <= .5
                    and reacquire_needed(node.status or {})
                    and time.monotonic()-max(node.last_target_wall, node.yolo_off_wall) > 2.0):
                print(json.dumps(dict(status='REACQUIRE',
                    reason='RGB-D lost the pallet after YOLO stopped; start a fresh recognition session',
                    robot_commands=0), ensure_ascii=False), flush=True)
                return 3
            samples = list(node.samples)[-5:]
            now_ros = node.get_clock().now().nanoseconds * 1e-9
            if (time.monotonic()-node.status_wall > .5
                    or not rgbd_qualified(node.status or {}, samples, now_ros)
                    or time.monotonic()-last_attempt < 1.):
                continue
            last_attempt = time.monotonic()
            try:
                localization = None
                if args.localization == 'amcl':
                    state = checked_status(node.localization_status or {}, now_ros)
                    if state.get('correction_settled') is not True:
                        raise ValueError('waiting for AMCL correction to settle before locking pallet')
                    external=state.get('external_control_odom') is True
                    if external:
                        # Capture the same applied frame used by the actual
                        # odometry publisher, including a held correction.
                        state=checked_status(node.control_localization_status or {},now_ros,state)
                        if state.get('correction_settled') is not True:
                            raise ValueError('waiting for the applied control correction to settle before locking pallet')
                    localization = dict(mode='amcl', run_id=state['run_id'], map_file=state['map_file'],
                        baseline_map_odom=state['baseline_map_odom'],
                        correction_at_capture_xyyaw=state['correction_xyyaw'])
                    if external:
                        localization['control_status_topic']='/pallet/r3/control_odom/status'
                lateral_calibration = load_axis_lateral_calibration()
                snapshot, start = snapshot_from_samples(
                    samples, node.status,
                    lateral_residual_m=lateral_calibration['pallet_axis_lateral_residual_m'],
                    lateral_calibration=lateral_calibration)
                if localization:
                    snapshot.update(frame_id='r3_trial', localization=localization)
                print(json.dumps(dict(status='RGBD_READY_PLANNING',
                    recognition_session=node.status.get('session'),
                    recognition_elapsed_s=node.status.get('elapsed_s'),
                    samples=len(samples),
                    capture_age_ms=round((now_ros-samples[-1]['stamp'])*1000, 1),
                    heading_deg=round(snapshot['measurement_report']['entry_heading_deg'], 2),
                    robot_commands=0), ensure_ascii=False), flush=True)
                planning_started_wall = time.monotonic()
                plan, phases, rows = build_plan(snapshot, start, provisional_geometry(), args.output)
                # A hash alone cannot reconstruct a wrong locked target.
                # Preserve the actual accepted scalar captures and poses too.
                (args.output/'capture_samples.json').write_text(json.dumps(
                    dict(samples=samples, recognition_session=node.status), indent=2))
            except PlanReferenceError as exc:
                print(json.dumps(dict(status='PLAN_BLOCKED',
                    reason='Invalid plan frame/reference: '+str(exc),
                    robot_commands=0), ensure_ascii=False), flush=True)
                return 5
            except ValueError as exc:
                if str(exc).startswith(('START_TOO_CLOSE_FOR_C:', 'no feasible A→C→B path:')):
                    # Planning failure is not a missing camera heartbeat.
                    # Never loop back to its watchdog after a long search and
                    # restart a detector which already provided valid data.
                    print(json.dumps(dict(status='PLAN_BLOCKED', reason=str(exc),
                                          robot_commands=0), ensure_ascii=False), flush=True)
                    return 5
                if str(exc) != last_error:
                    last_error = str(exc)
                    print('Chưa lập được quỹ đạo: ' + last_error, flush=True)
                continue
            report = snapshot['measurement_report']
            print(json.dumps(dict(status='PLAN_READY', plan=str(args.output/'frozen_plan.json'),
                planning_elapsed_s=round(time.monotonic()-planning_started_wall, 3),
                preview=str(args.output/'trajectory.svg'),
                plan_id=plan.plan_id, camera_raw_cm=round(report['camera_lateral_raw_m']*100, 2),
                camera_correction_cm=2.5,
                camera_mount_corrected_cm=round(report['fork_lateral_corrected_m']*100, 2),
                axis_lateral_residual_cm=round(report['pallet_axis_lateral_residual_m']*100, 2),
                fork_lateral_cm=round(report['target_lateral_after_axis_residual_m']*100, 2),
                fork_gap_m=round(errors(start, snapshot['pose'], provisional_geometry())['gap_m'], 3),
                c_fork_tip_clearance_cm=round(
                    (errors(phases[1]['poses'][0], snapshot['pose'], provisional_geometry())['gap_m']
                     - provisional_geometry()['fork_length'])*100, 1),
                straight_lead_in_cm=round(phases[0]['straight_lead_in_m']*100, 1),
                heading_deg=round(report['entry_heading_deg'], 2),
                heading_source=report['heading_source'],
                heading_depth_x_slope=report['heading_depth_x_slope'],
                robot_commands=0), indent=2, ensure_ascii=False))
            return 0
        print(json.dumps(dict(status='BLOCKED', reason='No fresh, stable post-YOLO RGB-D + stationary odom plan',
                              samples=len(node.samples), last_status=node.status,
                              capture=node.capture_diagnostics(),
                              last_plan_error=last_error,
                              robot_commands=0), ensure_ascii=False))
        return 2
    except (KeyboardInterrupt,ExternalShutdownException):
        print(json.dumps(dict(status='CANCELLED',reason='Recognition stopped by operator',
                              robot_commands=0),ensure_ascii=False),flush=True)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
