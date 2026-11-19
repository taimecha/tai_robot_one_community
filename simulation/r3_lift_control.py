#!/usr/bin/env python3
"""HOME and raise the real lift through its existing serial bridge."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Float64MultiArray, String

FIRMWARE_PROFILE = 'HOME_SKIP_ACTIVE_1MM_NEW_CONTACT_V4'
SUPPORTED_FIRMWARE_PROFILES = {FIRMWARE_PROFILE, 'HOME_SKIP_ACTIVE_1MM_NEW_CONTACT_V3'}
HOME_RETRY_FAULT = 'HOME_SETTLE_LOWER_UNSTABLE'
STATUS_TIMEOUT_S = .5
# HOME sits 1 mm beyond the lower-switch trigger. Stop above that switch in
# normal position mode. The approved fork top then sits 24 mm below the deck.
DELIVERY_HEIGHT_M = .002


class HomeBaseGuard:
    """Observe encoder odom during HOME; body/IMU vibration is not wheel roll."""
    def __init__(self):
        self.pose = None
        self.twist = None
        self.stamp = None
        self.received = 0.
        self.parked_pose = None
        self.parked_since = None
        self.parked_stamp = None
        self.anchor = None
        self.moving_since = None
        self.max_gap_s = 0.

    def update(self, msg, now, now_ros):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_link', 'base_footprint')
                or not all(math.isfinite(a) for a in (p.x, p.y, q.x, q.y, q.z, q.w, v, w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .02
                or not -.05 <= now_ros-stamp <= 1.
                or (self.stamp is not None and stamp <= self.stamp)):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if self.received:
            self.max_gap_s = max(self.max_gap_s, now-self.received)
        self.pose, self.twist, self.stamp, self.received = (p.x, p.y, yaw), (v, w), stamp, now

    @staticmethod
    def difference(a, b):
        return math.dist(a[:2], b[:2]), abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2])))

    def age(self, now, now_ros):
        return max(now-self.received, now_ros-self.stamp) if self.stamp is not None else float('inf')

    def ready(self, now, now_ros):
        if self.pose is None or self.age(now, now_ros) > .5:
            self.parked_since = None
            return False
        xy, yaw = self.difference(self.pose, self.parked_pose or self.pose)
        if self.parked_since is None or xy > .005 or yaw > math.radians(.5):
            self.parked_pose, self.parked_since = self.pose, now
            self.parked_stamp = self.stamp
        return now-self.parked_since >= .5 and self.stamp-self.parked_stamp >= .4

    def arm(self):
        self.anchor = self.pose

    def check(self, now, now_ros):
        age = self.age(now, now_ros)
        if age > 1.:
            source_age = now_ros-self.stamp if self.stamp is not None else float('inf')
            raise ValueError(f'wheel encoder odom unavailable during HOME: receive age {now-self.received:.3f}s, '
                             f'source age {source_age:.3f}s')
        if age > .5:
            return False  # Wait for a fresh confirmation, bounded by the lease above.
        xy, yaw = self.difference(self.pose, self.anchor)
        if xy > .03 or yaw > math.radians(3.):
            raise ValueError(f'wheel encoder displacement during HOME: {xy*100:.2f} cm / {math.degrees(yaw):.2f} deg')
        if xy > .01 or yaw > math.radians(1.):
            if self.moving_since is None:
                self.moving_since = now
            if now-self.moving_since >= .2:
                raise ValueError(f'sustained wheel displacement during HOME: {xy*100:.2f} cm / {math.degrees(yaw):.2f} deg')
            return False
        self.moving_since = None
        return True


class LiftControl:
    def __init__(self, node):
        self.node = node
        self.value = None
        self.received = 0.
        self.sample_ms = None
        self.error = ''
        self.home_seen = False
        self.retrying_home = False
        self.home_generation_before = 0
        self.requested_at = None
        self.raise_started = None
        self.lower_started = None
        self.height = None
        self.set_commands = 0
        self.home_commands = 0
        self.stop_commands = 0
        self.stable_since = None
        self.home_pub = node.create_publisher(Empty, '/lift/home', 1)
        self.stop_pub = node.create_publisher(Empty, '/lift/stop', 1)
        self.target_pub = node.create_publisher(Float64MultiArray, '/lift_controller/commands', 1)
        self.subscription = node.create_subscription(String, '/lift/status', self.on_status, 1)

    def on_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('invalid lift status')
            if value.get('serial_connected') is not True:
                raise ValueError('lift serial disconnected')
            sample = value.get('sample_ms')
            if isinstance(sample, bool) or not isinstance(sample, int) or not 0 <= sample <= 0xFFFFFFFF:
                raise ValueError('lift telemetry has no source sample')
            if self.sample_ms is not None:
                delta = (sample-self.sample_ms) & 0xFFFFFFFF
                if delta == 0:
                    return  # Replays cannot refresh the telemetry lease.
                if delta >= 0x80000000:
                    self.error = 'ESP32 lift restarted or telemetry moved backwards'
                    return
            for key in ('position', 'target', 'velocity', 'published_ros_s'):
                v = value.get(key)
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                    raise ValueError('nonfinite/missing lift '+key)
            target_min = 0. if value.get('homed') else -.60
            if not (-.60 <= value['position'] <= .53 and target_min <= value['target'] <= .52
                    and abs(value['velocity']) <= .02):
                raise ValueError('lift feedback outside physical limits')
            for key in ('homed', 'lower_active', 'upper_active'):
                if not isinstance(value.get(key), bool):
                    raise ValueError('invalid lift '+key)
            generation = value.get('home_completion_count')
            if value.get('firmware_profile') in SUPPORTED_FIRMWARE_PROFILES and (
                    isinstance(generation, bool) or not isinstance(generation, int)
                    or not 0 <= generation <= 0xFFFFFFFF):
                raise ValueError('missing/invalid lift HOME acknowledgement')
            self.sample_ms = sample
            self.value, self.received = value, time.monotonic()
            if self.requested_at is not None and value.get('state') == 'HOMING':
                self.home_seen = True
        except (ValueError, TypeError) as exc:
            self.value = None
            self.received = 0.

    def fresh_feedback(self):
        if self.error:
            raise ValueError(self.error)
        value = self.value
        if value is None or time.monotonic()-self.received > STATUS_TIMEOUT_S:
            raise ValueError('lift telemetry missing/stale')
        age = self.node.get_clock().now().nanoseconds*1e-9-value['published_ros_s']
        if not -.05 <= age <= STATUS_TIMEOUT_S:
            raise ValueError('lift status transport is stale')
        profile = value.get('firmware_profile')
        if profile in (None, ''):
            raise ValueError('lift firmware identity not received; waiting for LIFT,INFO response')
        if profile not in SUPPORTED_FIRMWARE_PROFILES:
            raise ValueError('incompatible lift firmware '+repr(profile)+'; expected V3 or V4 HOME profile')
        return value

    @staticmethod
    def check_status(value):
        if value.get('state') not in ('UNHOMED', 'IDLE', 'HOMING', 'MOVING') or value.get('fault') != 'NONE':
            raise ValueError('lift fault: '+str(value.get('fault', value.get('state'))))

    def status(self):
        value = self.fresh_feedback()
        self.check_status(value)
        return value

    def home_preflight(self, *, retry_home=False):
        value = self.fresh_feedback()
        if (value.get('firmware_profile') == FIRMWARE_PROFILE
                and value.get('state') == 'UNHOMED' and value.get('fault') == HOME_RETRY_FAULT):
            if not retry_home:
                raise ValueError(f'lift fault: {HOME_RETRY_FAULT}; sau khi nâng càng rời công tắc đáy, '
                                 'dùng --retry-home để thử lại một lần')
            if value['homed'] or value['lower_active'] or value['upper_active'] or abs(value['velocity']) > .0005:
                raise ValueError('HOME retry requires an unhomed, stationary lift with both switches released')
        else:
            self.check_status(value)
        return value

    def check_owners(self):
        owner = self.node.get_fully_qualified_name()
        for topic in ('/lift/home', '/lift_controller/commands'):
            publishers = [f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                          for p in self.node.get_publishers_info_by_topic(topic)]
            consumers = [f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                         for p in self.node.get_subscriptions_info_by_topic(topic)]
            if publishers != [owner] or consumers != ['/esp32_imu_bridge']:
                raise ValueError(f'lift command ownership conflict/unavailable on {topic}: '
                                 f'publishers={publishers}, consumers={consumers}')
        stops = self.node.get_subscriptions_info_by_topic('/lift/stop')
        if len(stops) != 1 or stops[0].node_name != 'esp32_imu_bridge':
            raise ValueError('lift STOP receiver unavailable')

    def at_home(self, require_lower=False, require_ready=True):
        value = self.status()
        if (value['homed'] is not True or value['state'] != 'IDLE'
                or abs(value['position']) > .0015 or abs(value['target']) > .0015
                or abs(value['velocity']) > .0005 or value['upper_active']
                or (require_ready and value.get('position_commands_ready') is not True)
                or (require_lower and not value['lower_active'])):
            raise ValueError('lift must be homed and stationary at zero before insertion')
        return value

    def home(self, *, retry_home=False):
        value = self.home_preflight(retry_home=retry_home)
        self.check_owners()
        self.requested_at = time.monotonic()
        self.home_seen = False
        self.retrying_home = value.get('fault') == HOME_RETRY_FAULT
        self.home_generation_before = value['home_completion_count']
        self.home_pub.publish(Empty())
        self.home_commands += 1

    def home_complete(self):
        value = self.fresh_feedback()
        # V4 can retain the old fault string through HOMING after a STOP.
        # A new fault switches to FAULT and must immediately abort the retry.
        waiting_for_start = not self.home_seen and value.get('state') == 'UNHOMED'
        retained_fault = (self.retrying_home and value.get('firmware_profile') == FIRMWARE_PROFILE
                          and value.get('fault') == HOME_RETRY_FAULT
                          and value['home_completion_count'] == self.home_generation_before
                          and not value['homed'] and not value['upper_active']
                          and (value.get('state') == 'HOMING' or (waiting_for_start
                               and not value['lower_active'] and abs(value['velocity']) <= .0005)))
        if not retained_fault:
            self.check_status(value)
        now = time.monotonic()
        completed = 0 < ((value['home_completion_count']-self.home_generation_before) & 0xFFFFFFFF) < 0x80000000
        if now-self.requested_at > 70. or (not self.home_seen and not completed and now-self.requested_at > 3.):
            raise ValueError('lift HOME did not complete/acknowledge in time')
        if not completed or value['state'] == 'HOMING':
            return False
        self.at_home(require_lower=True, require_ready=False)
        if self.retrying_home and value.get('home_skipped_active') is not False:
            raise ValueError('HOME retry requires a new lower-switch contact')
        return True

    def begin_raise(self, height):
        if not math.isfinite(height) or not 0 < height <= .08:
            raise ValueError('pickup lift height must be in (0, 0.08] m')
        self.at_home(require_lower=True)
        self.check_owners()
        self.height = height
        self.raise_started = time.monotonic()
        self.stable_since = None

    def raise_tick(self):
        value = self.status()
        now = time.monotonic()
        if value['homed'] is not True or value['state'] not in ('IDLE', 'MOVING'):
            raise ValueError('lift lost HOME during raise')
        if value['upper_active'] or value['position'] > self.height+.002:
            raise ValueError('lift upper limit or height overshoot')
        if now-self.raise_started > 25.:
            raise ValueError('lift did not reach requested height within 25 s')
        if (abs(value['target']-self.height) > .0001
                and now-self.raise_started > 2.):
            raise ValueError('lift target was not accepted or another controller changed it')
        if (value['state'] == 'IDLE' and abs(value['target']-self.height) <= .0001
                and abs(value['position']-self.height) <= .001
                and abs(value['velocity']) < .0005):
            if self.stable_since is None:
                self.stable_since = now
            if now-self.stable_since >= .5:
                return True
        else:
            self.stable_since = None
        self.target_pub.publish(Float64MultiArray(data=[self.height]))
        self.set_commands += 1
        return False

    def at_travel_height(self, height, home_generation):
        value = self.status()
        if (value['homed'] is not True or value['state'] != 'IDLE'
                or value['home_completion_count'] != home_generation
                or value['upper_active'] or value['lower_active']
                or abs(value['position']-height) > .002
                or abs(value['target']-height) > .002 or abs(value['velocity']) > .0005
                or value.get('position_commands_ready') is not True):
            raise ValueError('fresh lift feedback does not show the same raised, stationary forks')
        return value

    def begin_lower(self, height, home_generation):
        self.at_travel_height(height, home_generation)
        self.check_owners()
        self.height = DELIVERY_HEIGHT_M
        self.lower_started = time.monotonic()
        self.stable_since = None

    def lower_tick(self, original_height, home_generation):
        value = self.status()
        now = time.monotonic()
        at_bottom = abs(value['position']-DELIVERY_HEIGHT_M) <= .001
        if (not value['homed'] or value['home_completion_count'] != home_generation
                or value['state'] not in ('IDLE', 'MOVING') or value['upper_active']
                or value['position'] < DELIVERY_HEIGHT_M-.001
                or value['position'] > original_height+.002 or value['velocity'] > .0005
                or (value['lower_active'] and not at_bottom)):
            raise ValueError('lift position, motion or limits became invalid during delivery')
        accepted = abs(value['target']-DELIVERY_HEIGHT_M) <= .0001
        if now-self.lower_started > 25. or (not accepted and now-self.lower_started > 2.):
            raise ValueError('lift did not accept or reach the delivery height')
        if accepted and at_bottom and value['state'] == 'IDLE' and abs(value['velocity']) <= .0005:
            if self.stable_since is None:
                self.stable_since = now
            if now-self.stable_since >= .5:
                return True
        else:
            self.stable_since = None
        self.target_pub.publish(Float64MultiArray(data=[DELIVERY_HEIGHT_M]))
        self.set_commands += 1
        return False

    def stop(self):
        self.stop_pub.publish(Empty())
        self.stop_commands += 1

    def report(self):
        return dict(feedback=self.value, height_requested_m=self.height,
                    home_commands=self.home_commands, position_commands=self.set_commands,
                    stop_commands=self.stop_commands, physical_load_lift_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('home', 'stop'))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--telemetry', type=Path, help='Write HOME feedback to a JSON Lines file')
    parser.add_argument('--retry-home', action='store_true', help='Retry V4 HOME once after raising clear of the bottom switch')
    args = parser.parse_args()
    if not args.execute:
        parser.error('--execute is required for a lift command')
    if args.telemetry is not None and args.action != 'home':
        parser.error('--telemetry is only supported with home')
    if args.retry_home and args.action != 'home':
        parser.error('--retry-home is only supported with home')
    rclpy.init()
    node = Node('r3_lift_home' if args.action == 'home' else 'r3_lift_stop')
    lift = LiftControl(node)
    base = HomeBaseGuard()

    def on_odom(msg):
        base.update(msg, time.monotonic(), node.get_clock().now().nanoseconds*1e-9)
    sub = node.create_subscription(Odometry, '/base_controller/odom', on_odom, qos_profile_sensor_data)
    started = time.monotonic()
    next_wait_report = started + 2.
    complete_since = None
    trace = None
    last_trace_sample = None
    try:
        if args.telemetry is not None:
            trace = args.telemetry.open('x', buffering=1, encoding='utf-8')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if trace is not None and lift.value is not None and lift.sample_ms != last_trace_sample:
                trace.write(json.dumps(lift.value, allow_nan=False)+'\n')
                last_trace_sample = lift.sample_ms
            if args.action == 'stop':
                if node.count_subscribers('/lift/stop'):
                    for _ in range(3):
                        lift.stop()
                        rclpy.spin_once(node, timeout_sec=.05)
                    return
                if now-started > 2.:
                    return
                continue
            now_ros = node.get_clock().now().nanoseconds*1e-9
            if lift.requested_at is None:
                try:
                    lift.home_preflight(retry_home=args.retry_home)
                    lift.check_owners()
                except ValueError as exc:
                    if now-started > 10.:
                        raise
                    if now >= next_wait_report:
                        print(json.dumps(dict(status='WAITING_FOR_LIFT', reason=str(exc),
                            firmware_profile=(lift.value or {}).get('firmware_profile'),
                            firmware_info=(lift.value or {}).get('firmware_info')),
                            ensure_ascii=False), flush=True)
                        next_wait_report = now + 2.
                    continue
                if not base.ready(now, now_ros):
                    if now-started > 10.:
                        raise ValueError('fresh stationary wheel encoder odom unavailable before HOME')
                    continue
                base.arm()
                lift.home(retry_home=args.retry_home)
                if lift.retrying_home:
                    print('Đã gửi một lệnh thử lại HOME để tìm tiếp điểm đáy mới.', flush=True)
                else:
                    print('Kiểm tra công tắc đáy: đã chạm thì HOME tại chỗ; chưa chạm mới tìm và hạ thêm 1 mm.', flush=True)
            elif base.check(now, now_ros) and lift.home_complete():
                if complete_since is None:
                    complete_since = now
                if now-complete_since >= .5:
                    print(json.dumps(dict(status='LIFT_HOME_READY', wheel_odom_max_gap_s=base.max_gap_s,
                                          **lift.report())), flush=True)
                    return
    except (ValueError, OSError) as exc:
        print(json.dumps(dict(status='LIFT_HOME_BLOCKED', reason=str(exc), telemetry=str(args.telemetry)
                              if args.telemetry is not None else None, **lift.report()), ensure_ascii=False),
              flush=True)
        return 2
    finally:
        if trace is not None:
            trace.close()
        if args.action == 'home' and lift.requested_at is not None:
            for _ in range(3):
                lift.stop()
                rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
