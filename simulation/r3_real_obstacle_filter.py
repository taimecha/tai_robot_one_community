#!/usr/bin/env python3
"""Filter the one locked pallet from the LiDAR scan for r3 safety monitor.

No wheel/lift publisher. A missing transform or stale scan yields no output,
so the downstream collision monitor must time out and stop the vehicle.
"""

import argparse
from collections import deque
import copy
import json
import math
from pathlib import Path
import time
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import CollisionMonitorState
from tf2_ros import Buffer, TransformException, TransformListener

from r3_core import (R3_PLANNED_C_CLEARANCE_M, FrozenPlan, errors,
                     real_geometry_mode, validate_phases, validate_axis_lateral_residual, compose, inverse)
from r3_localization_contract import localization_spec, checked_status, CorrectionHistory
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from r3_pallet_obstacles import classify_ranges


def inside_polygon(point, polygon):
    """Diagnostic point membership only; never used to filter/stop a scan."""
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        cross = (x-a[0])*(b[1]-a[1])-(y-a[1])*(b[0]-a[0])
        if (abs(cross) < 1e-10 and min(a[0], b[0])-1e-10 <= x <= max(a[0], b[0])+1e-10
                and min(a[1], b[1])-1e-10 <= y <= max(a[1], b[1])+1e-10):
            return True
        if (a[1] > y) != (b[1] > y):
            crossing = a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1])
            if x < crossing:
                inside = not inside
    return inside


def json_finite(value):
    """Encode no-return scan values as null, keeping the beam index intact."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_finite(item) for item in value]
    return value


class ScanStopEvidence:
    """Bounded sensor snapshots; write once only after an actual StopZone event."""
    def __init__(self, directory, config):
        self.directory = Path(directory)
        self.config = str(Path(config))
        params = yaml.safe_load(Path(config).read_text())['r3_collision_monitor']['ros__parameters']
        self.polygon = yaml.safe_load(params['StopZone']['points'])
        self.base_shift_correction = params.get('base_shift_correction', False)
        self.frames = deque(maxlen=4)
        self.written = False

    def add(self, message, ranges, received_ros_s, raw_sensor_pose, sensor_pose,
            correction, base_pose, base_stamp, counts):
        self.frames.append(dict(
            source_stamp_ros_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            source_stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
            received_ros_s=received_ros_s, frame_id=message.header.frame_id,
            angle_min=message.angle_min, angle_increment=message.angle_increment,
            range_min=message.range_min, range_max=message.range_max,
            raw_ranges_m=tuple(message.ranges), filtered_ranges_m=tuple(ranges),
            raw_sensor_pose_odom_xyyaw=list(raw_sensor_pose),
            sensor_pose_trial_xyyaw=list(sensor_pose), applied_correction_xyyaw=list(correction),
            base_pose_at_capture_odom_xyyaw=base_pose, base_pose_stamp_ros_s=base_stamp,
            masked_pallet_points=counts[0], retained_nonpallet_points=counts[1]))

    def write(self, event_ros_s, base_now, base_now_stamp, target, localization, geometry=None):
        if self.written:
            return None
        # The trial latches this stop. Do not repeatedly retry disk I/O on
        # subsequent state publications, including when storage is full.
        self.written = True
        frames = []
        for snapshot in self.frames:
            frame = dict(snapshot)
            retained = []
            # The monitor can compensate base motion between scan capture
            # and its decision. Preserve both reconstructions and their
            # genuine TF stamps; neither is claimed as the exact trigger.
            capture_base = frame['base_pose_at_capture_odom_xyyaw']
            for i, distance in enumerate(frame['filtered_ranges_m']):
                if (not math.isfinite(distance) or distance <= 0
                        or distance < frame['range_min'] or distance > frame['range_max']):
                    continue
                angle = frame['angle_min']+i*frame['angle_increment']
                sensor_point = (distance*math.cos(angle), distance*math.sin(angle), 0.)
                odom_point = compose(frame['raw_sensor_pose_odom_xyyaw'], sensor_point)
                trial_point = compose(frame['sensor_pose_trial_xyyaw'], sensor_point)
                capture_point = compose(inverse(capture_base), odom_point) if capture_base else None
                event_point = compose(inverse(base_now), odom_point) if base_now else None
                at_capture = bool(capture_point and inside_polygon(capture_point[:2], self.polygon))
                at_event = bool(event_point and inside_polygon(event_point[:2], self.polygon))
                if at_capture or at_event:
                    retained.append(dict(beam_index=i, range_m=distance,
                        sensor_xy=list(sensor_point[:2]), odom_xy=list(odom_point[:2]),
                        trial_xy=list(trial_point[:2]),
                        base_capture_xy=list(capture_point[:2]) if capture_point else None,
                        base_event_xy=list(event_point[:2]) if event_point else None,
                        inside_at_capture=at_capture, inside_at_event=at_event))
            frame['retained_points_in_StopZone'] = retained
            frame['source_age_at_stop_s'] = event_ros_s-frame['source_stamp_ros_s']
            frames.append(frame)
        report = dict(schema='r3_stop_zone_evidence_v1', observer_only=True,
            robot_commands_from_this_diagnostic=0, event_ros_s=event_ros_s,
            event_received_ros_s=event_ros_s, collision_config=self.config,
            event=dict(action='STOP', polygon_name='StopZone'),
            stop_zone_polygon_base_xy=self.polygon, base_shift_correction=self.base_shift_correction,
            base_pose_at_event_odom_xyyaw=base_now, base_pose_at_event_stamp_ros_s=base_now_stamp,
            target=target, pallet_mask_geometry=geometry,
            localization_status=localization, recent_filtered_scans=frames,
            exact_triggering_scan_verified=False,
            note='Bounded recent scan evidence; StopZone state does not identify its exact triggering beam. '
                 'Capture/event reconstructions preserve original timestamps. Nonfinite ranges are null.')
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory/'stop_zone_evidence.json'
        with path.open('x') as stream:
            json.dump(json_finite(report), stream, indent=2, allow_nan=False)
        return path


def read_real_target(path, allow_provisional=False, max_age_s=30.):
    data = json.loads(path.read_text())
    plan = FrozenPlan(**{key: data[key] for key in
                         ('target_json', 'geometry_json', 'phases_json', 'config_json')})
    if plan.plan_id != data.get('plan_id'):
        raise ValueError('frozen plan hash mismatch')
    target = json.loads(plan.target_json)
    localization_spec(target)
    geometry = json.loads(plan.geometry_json)
    phases = json.loads(plan.phases_json)
    config = json.loads(plan.config_json)
    validate_phases(phases, target['pose'], geometry)
    clearance = (errors(phases[1]['poses'][0], target['pose'], geometry)['gap_m']
                 - geometry['fork_length'])
    if clearance < R3_PLANNED_C_CLEARANCE_M - 1e-6:
        raise ValueError('C leaves less than 15 cm between fork tips and pallet face')
    if geometry['max_lateral'] > .010:
        raise ValueError('real terminal lateral error must stay under 1 cm')
    refinement = config.get('approach_refinement', {})
    if ([phase.get('type') for phase in phases] != ['APPROACH', 'STRAIGHT']
            or refinement.get('method') != 'bounded fork-root shape search on clamped quintic y(x)'
            or refinement.get('selected_C_unchanged') is not True
            or refinement.get('terminal_line_unchanged') is not True):
        raise ValueError('the reviewed refined A→C and straight C→B plan is required')
    real_geometry_mode(geometry, allow_provisional)
    if (target.get('source') != 'stationary_real_RGBD'
            or not target.get('raw_session_hash')
            or not target.get('yolo_off_ack')
            or geometry.get('provenance') == 'SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION'):
        raise ValueError('fresh locked real pallet required')
    report = target.get('measurement_report') or {}
    validate_axis_lateral_residual(target, phases[0]['poses'][0])
    raw, offset, corrected = (report.get('camera_lateral_raw_m'),
                              report.get('camera_mount_offset_m'),
                              report.get('fork_lateral_corrected_m'))
    if (not all(isinstance(v, (int, float)) and math.isfinite(v)
                for v in (raw, offset, corrected))
            or not math.isclose(offset, .025, abs_tol=1e-9)
            or not math.isclose(raw + offset, corrected, abs_tol=1e-4)):
        raise ValueError('camera raw + one +2.5 cm correction report required')
    if (report.get('heading_source') != 'floor_referenced_plane'
            or not isinstance(report.get('heading_depth_x_slope'), (int, float))
            or not math.isfinite(report['heading_depth_x_slope'])):
        raise ValueError('fresh floor-referenced pallet heading required')
    if max_age_s is not None and not 0 <= time.time() - target['capture_stamp_s'] <= max_age_s:
        raise ValueError('locked pallet capture is stale; recognize again')
    return target, geometry


class PalletObstacleFilter(Node):
    def __init__(self, target, geometry, diagnostics_dir=None, collision_config=None):
        super().__init__('r3_pallet_obstacle_filter')
        self.target, self.geometry = target['pose'], geometry
        self.target_evidence = copy.deepcopy(target)
        self.evidence = None
        if diagnostics_dir:
            config = collision_config or (Path(__file__).resolve().parents[1]/
                'camera/detect_astra/config/r3_collision_monitor.yaml')
            try:
                self.evidence = ScanStopEvidence(diagnostics_dir, config)
            except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
                self.get_logger().warn('StopZone evidence disabled: '+str(exc))
        self.localization = localization_spec(target)
        self.localization_status = None
        self.correction_history = CorrectionHistory()
        if self.localization:
            self.create_subscription(String, self.localization.get('control_status_topic','/pallet/r3/localization/status'), self.on_localization,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.received = {'lidar': 0.}
        self.published = {'lidar': 0.}
        self.last_drop = {'lidar': 'no input'}
        self.outputs = {
            'lidar': self.create_publisher(LaserScan, '/pallet/r3/filtered_lidar_scan', qos_profile_sensor_data),
        }
        self.create_subscription(LaserScan, '/scan_filtered',
                                 lambda msg: self.on_scan('lidar', msg), qos_profile_sensor_data)
        if self.evidence is not None:
            self.create_subscription(CollisionMonitorState, '/pallet/r3/collision_state',
                                     self.on_collision_state, 1)
        self.create_timer(.5, self.report)

    def on_localization(self, msg):
        try:
            self.localization_status = checked_status(
                msg.data, self.get_clock().now().nanoseconds*1e-9, self.localization)
            self.correction_history.add(self.localization_status)
        except (ValueError, TypeError, KeyError):
            self.localization_status = None
            self.correction_history.clear()

    def on_scan(self, name, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.received[name] = time.monotonic()
        if not msg.header.frame_id or not -.05 <= now - stamp <= .3:
            self.last_drop[name] = f'bad frame or stamp (age {now-stamp:.3f} s)'
            return
        try:
            transform = self.tf.lookup_transform('odom', msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=.03))
        except TransformException as exc:
            self.last_drop[name] = f'odom transform unavailable at scan stamp: {exc}'
            return
        t, q = transform.transform.translation, transform.transform.rotation
        if not all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w)):
            self.last_drop[name] = 'nonfinite odom transform'
            return
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        scan_pose = (t.x,t.y,yaw)
        raw_scan_pose = scan_pose
        correction = (0., 0., 0.)
        if self.localization:
            try:
                state = checked_status(self.localization_status or {}, now, self.localization)
                # Transform the raw scan with the applied correction at its
                # capture stamp, not the newest correction at callback time.
                correction = self.correction_history.at(stamp)
                scan_pose = compose(correction, scan_pose)
            except (ValueError, TypeError, KeyError) as exc:
                self.last_drop[name] = str(exc)
                return
        try:
            ranges, pallet, other = classify_ranges(
                msg.ranges, msg.angle_min, msg.angle_increment,
                scan_pose, self.target, self.geometry)
        except ValueError:
            self.last_drop[name] = 'invalid scan geometry'
            return
        filtered = copy.copy(msg)
        filtered.ranges = ranges
        self.outputs[name].publish(filtered)
        self.published[name] = time.monotonic()
        self.last_drop[name] = 'none'
        if self.evidence is not None and not self.evidence.written:
            # Copy at most four successful scans in memory. No diagnostic
            # file is created or written on the scan/command hot path.
            base_pose, base_stamp = self.diagnostic_base_pose(Time.from_msg(msg.header.stamp))
            self.evidence.add(msg, ranges, now, raw_scan_pose, scan_pose,
                              correction, base_pose, base_stamp, (pallet, other))
        if pallet:
            self.get_logger().debug(f'{name}: ignored {pallet} pallet returns, kept {other} other returns')

    def diagnostic_base_pose(self, stamp):
        try:
            transform = self.tf.lookup_transform('odom', 'base_footprint', stamp)
            t, q = transform.transform.translation, transform.transform.rotation
            if not all(math.isfinite(v) for v in (t.x,t.y,q.x,q.y,q.z,q.w)):
                return None, None
            return ([t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
                    transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9)
        except TransformException:
            return None, None

    def on_collision_state(self, message):
        if (self.evidence is None or self.evidence.written
                or message.action_type != CollisionMonitorState.STOP
                or message.polygon_name != 'StopZone'):
            return
        now = self.get_clock().now().nanoseconds*1e-9
        base_pose, base_stamp = self.diagnostic_base_pose(Time())
        try:
            path = self.evidence.write(now, base_pose, base_stamp,
                                       self.target_evidence, self.localization_status, self.geometry)
            if path is not None:
                self.get_logger().info('StopZone scan evidence: '+str(path))
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().warn('Could not write StopZone scan evidence: '+str(exc))

    def report(self):
        now = time.monotonic()
        stale = [name for name, stamp in self.published.items() if now - stamp > 1.0]
        if stale:
            detail = ', '.join(f'{name} input_age={now-self.received[name]:.2f}s '
                               f'output_age={now-self.published[name]:.2f}s '
                               f'last_drop={self.last_drop[name]}' for name in stale)
            self.get_logger().warn('No fresh filtered scan: ' + detail, throttle_duration_sec=2.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--allow-provisional', action='store_true')
    parser.add_argument('--diagnostics-dir', default='',
                        help='Optional per-run StopZone evidence directory; no scan-loop disk writes')
    parser.add_argument('--collision-config', type=Path)
    args = parser.parse_args()
    target, geometry = read_real_target(args.plan, args.allow_provisional)
    rclpy.init()
    node = PalletObstacleFilter(target, geometry, args.diagnostics_dir.strip() or None,
                               args.collision_config)
    # Scan callbacks wait briefly for the transform at the scan timestamp.
    # A second executor thread lets the TF listener receive it during that wait.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
