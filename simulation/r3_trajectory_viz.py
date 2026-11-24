#!/usr/bin/env python3
"""Read-only RViz trajectory publisher and CSV recorder for one R3 cycle.

Publishes two latched nav_msgs/Path topics in the map frame:
  - /pallet/r3/rviz/actual_trajectory: robot map->base_footprint trail from
    cycle start through A, pickup (A->C->B), and delivery (B->D).
  - /pallet/r3/rviz/planned_trajectory: concatenated planned path across
    Leg 1 (Nav2 -> A), Leg 2 (A->C->B from plan/path_planned_base.csv mapped
    with baseline_map_odom), and Leg 3 (Nav2 B -> D).

Also writes full_trajectory_actual.csv and full_trajectory_planned.csv into
cycle_dir so full-cycle plots can be rendered onto the occupancy map.
Publishes zero velocity, localization, or actuator commands.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


ACTUAL_TOPIC = '/pallet/r3/rviz/actual_trajectory'
PLANNED_TOPIC = '/pallet/r3/rviz/planned_trajectory'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca*b[0] - sa*b[1],
            a[1] + sa*b[0] + ca*b[1],
            wrap(a[2] + b[2]))


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.*(w*z + x*y), 1. - 2.*(y*y + z*z))


def pose_xyyaw(pose_stamped):
    p = pose_stamped.pose.position
    q = pose_stamped.pose.orientation
    return (float(p.x), float(p.y), quat_to_yaw(q.x, q.y, q.z, q.w))


def xy_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_xyyaw_from_msg(msg):
    if msg.header.frame_id and msg.header.frame_id != 'map':
        return []
    out = []
    for ps in msg.poses:
        pt = pose_xyyaw(ps)
        if all(math.isfinite(v) for v in pt):
            out.append(pt)
    return out


def load_baseline_map_odom(cycle_dir):
    cycle_dir = Path(cycle_dir)
    for name in ('localization_at_A.json', 'localization_at_recognition.json'):
        path = cycle_dir / name
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        handoff = data.get('trial_handoff') if isinstance(data, dict) else None
        baseline = (handoff.get('baseline_map_odom') if isinstance(handoff, dict) else None)
        if not baseline and isinstance(data, dict):
            baseline = data.get('baseline_map_odom')
        if (isinstance(baseline, (list, tuple)) and len(baseline) == 3
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in baseline)):
            return (float(baseline[0]), float(baseline[1]), float(baseline[2]))
    return None


def load_pickup_planned_map_path(cycle_dir):
    """Return (plan_id, [(x, y, yaw), ...]) in map frame when plan & baseline exist."""
    cycle_dir = Path(cycle_dir)
    plan_json = cycle_dir / 'plan' / 'frozen_plan.json'
    csv_path = cycle_dir / 'plan' / 'path_planned_base.csv'
    if not plan_json.is_file() or not csv_path.is_file():
        return None, []
    baseline = load_baseline_map_odom(cycle_dir)
    if baseline is None:
        return None, []
    try:
        plan_id = str(json.loads(plan_json.read_text()).get('plan_id', ''))
        points = []
        with csv_path.open(newline='') as handle:
            for row in csv.DictReader(handle):
                x = float(row['x_m'])
                y = float(row['y_m'])
                yaw = float(row['yaw_rad'])
                if all(math.isfinite(v) for v in (x, y, yaw)):
                    points.append(compose(baseline, (x, y, yaw)))
        if not points:
            return None, []
        return plan_id, points
    except (OSError, ValueError, KeyError):
        return None, []


class TrajectoryAccumulator:
    """Pure state machine for actual & 3-leg planned paths in map frame."""

    def __init__(self, *, min_step_m=0.02, min_yaw_rad=0.05, leg_lock_move_m=0.15):
        self.min_step_m = min_step_m
        self.min_yaw_rad = min_yaw_rad
        self.leg_lock_move_m = leg_lock_move_m
        self.actual = []
        self.actual_meta = []  # (wall_s, phase)
        self.leg_to_a = []
        self.leg_to_a_origin = None
        self.leg_pickup = []
        self.pickup_plan_id = None
        self.leg_to_d = []
        self.leg_to_d_origin = None

    def add_actual(self, pose, *, wall_s=None, phase='to_A'):
        if not all(math.isfinite(v) for v in pose):
            return False
        t = float(time.time() if wall_s is None else wall_s)
        if not self.actual:
            self.actual.append(pose)
            self.actual_meta.append((t, str(phase)))
            return True
        prev = self.actual[-1]
        if (xy_distance(prev, pose) >= self.min_step_m
                or abs(wrap(pose[2] - prev[2])) >= self.min_yaw_rad):
            self.actual.append(pose)
            self.actual_meta.append((t, str(phase)))
            return True
        return False

    def update_nav_plan(self, points, *, delivery_phase):
        if not points:
            return False
        if not delivery_phase:
            if not self.leg_to_a:
                self.leg_to_a = list(points)
                self.leg_to_a_origin = points[0]
                return True
            if (self.leg_to_a_origin is not None
                    and xy_distance(self.leg_to_a_origin, points[0]) <= self.leg_lock_move_m):
                self.leg_to_a = list(points)
                return True
            return False
        if not self.leg_to_d:
            self.leg_to_d = list(points)
            self.leg_to_d_origin = points[0]
            return True
        if (self.leg_to_d_origin is not None
                and xy_distance(self.leg_to_d_origin, points[0]) <= self.leg_lock_move_m):
            self.leg_to_d = list(points)
            return True
        return False

    def set_pickup_plan(self, plan_id, points):
        if not points:
            return False
        if self.pickup_plan_id == plan_id and self.leg_pickup:
            return False
        self.pickup_plan_id = plan_id
        self.leg_pickup = list(points)
        return True

    def combined_planned(self):
        combined = []
        for leg in (self.leg_to_a, self.leg_pickup, self.leg_to_d):
            for pt in leg:
                if combined and xy_distance(combined[-1], pt) < 0.005:
                    continue
                combined.append(pt)
        return combined

    def write_csvs(self, cycle_dir):
        cycle_dir = Path(cycle_dir)
        cycle_dir.mkdir(parents=True, exist_ok=True)
        if self.actual:
            tmp = cycle_dir / 'full_trajectory_actual.csv.tmp'
            with tmp.open('w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['wall_s', 'x_m', 'y_m', 'yaw_rad', 'phase'])
                for (x, y, yaw), (wall_s, phase) in zip(self.actual, self.actual_meta):
                    w.writerow([f'{wall_s:.6f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}', phase])
            tmp.replace(cycle_dir / 'full_trajectory_actual.csv')
        legs = [('to_A', self.leg_to_a), ('pickup_A_to_B', self.leg_pickup), ('to_D', self.leg_to_d)]
        if any(pts for _, pts in legs):
            tmp = cycle_dir / 'full_trajectory_planned.csv.tmp'
            with tmp.open('w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['leg', 'x_m', 'y_m', 'yaw_rad'])
                for leg_name, pts in legs:
                    for x, y, yaw in pts:
                        w.writerow([leg_name, f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
            tmp.replace(cycle_dir / 'full_trajectory_planned.csv')


def build_path_msg(points, stamp):
    msg = NavPath()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    poses = []
    for x, y, yaw in points:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = stamp
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.02
        ps.pose.orientation.z = math.sin(yaw / 2.)
        ps.pose.orientation.w = math.cos(yaw / 2.)
        poses.append(ps)
    msg.poses = poses
    return msg


class TrajectoryVizNode(Node):
    def __init__(self, run_dir, cycle_dir=None):
        super().__init__('r3_trajectory_viz')
        self.run_dir = Path(run_dir)
        self.cycle_dir = Path(cycle_dir) if cycle_dir is not None else self.run_dir
        self.acc = TrajectoryAccumulator()
        self.localization_ready = False
        self.in_delivery_phase = False
        self.in_trial_phase = False
        self.dirty = False
        self.last_flush = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.actual_pub = self.create_publisher(NavPath, ACTUAL_TOPIC, latched)
        self.planned_pub = self.create_publisher(NavPath, PLANNED_TOPIC, latched)

        # Clear any previous cycle paths on RViz immediately.
        stamp = self.get_clock().now().to_msg()
        self.actual_pub.publish(build_path_msg([], stamp))
        self.planned_pub.publish(build_path_msg([], stamp))

        self.create_subscription(String, '/pallet/r3/localization/status',
                                 self.on_localization_status, latched)
        self.create_subscription(NavPath, '/plan', self.on_nav_plan, 5)
        self.create_timer(0.2, self.tick)

    def current_phase(self):
        if self.in_delivery_phase:
            return 'to_D'
        if self.in_trial_phase or bool(self.acc.leg_pickup):
            return 'pickup'
        return 'to_A'

    def on_localization_status(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict):
            return
        if data.get('ready') is True:
            self.localization_ready = True
        self.in_trial_phase = data.get('trial_handoff') is not None
        self.in_delivery_phase = data.get('delivery_handoff') is not None

    def on_nav_plan(self, msg):
        points = path_xyyaw_from_msg(msg)
        if not points:
            return
        delivery = self.in_delivery_phase or bool(self.acc.leg_pickup)
        if self.acc.update_nav_plan(points, delivery_phase=delivery):
            self.dirty = True
            self.publish_planned()

    def publish_actual(self):
        stamp = self.get_clock().now().to_msg()
        self.actual_pub.publish(build_path_msg(self.acc.actual, stamp))

    def publish_planned(self):
        pts = self.acc.combined_planned()
        if not pts:
            return
        stamp = self.get_clock().now().to_msg()
        self.planned_pub.publish(build_path_msg(pts, stamp))

    def flush(self):
        if self.dirty:
            self.acc.write_csvs(self.cycle_dir)
            self.dirty = False
            self.last_flush = time.monotonic()

    def tick(self):
        if not self.localization_ready:
            if (self.run_dir / 'localization_ready.json').is_file():
                self.localization_ready = True
        if self.localization_ready:
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                pose = (float(t.x), float(t.y), quat_to_yaw(q.x, q.y, q.z, q.w))
                if self.acc.add_actual(pose, wall_s=time.time(), phase=self.current_phase()):
                    self.dirty = True
                    self.publish_actual()
            except TransformException:
                pass

        plan_id, pickup_pts = load_pickup_planned_map_path(self.cycle_dir)
        if pickup_pts and self.acc.set_pickup_plan(plan_id, pickup_pts):
            self.dirty = True
            self.publish_planned()

        if self.dirty and time.monotonic() - self.last_flush >= 1.0:
            self.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--cycle-dir', type=Path, default=None)
    args = parser.parse_args()

    stop_flag = [False]
    def on_signal(*_):
        stop_flag[0] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    rclpy.init()
    node = TrajectoryVizNode(args.run_dir, args.cycle_dir)
    try:
        while rclpy.ok() and not stop_flag[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.flush()
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
