#!/usr/bin/env python3
"""Drive the picked-up pallet to saved D, then lower with the base parked."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from r3_core import digest
from r3_delivery_goal import DEFAULT_GOAL, PURPOSE, SCHEMA
from r3_delivery_topics import PRIVATE_ACTION, check_navigation_command_owners
from r3_enable_trial import fresh, validate_source
from r3_lift_control import HomeBaseGuard, LiftControl
from r3_recognition_goal import (RecognitionGoal, atomic_json, map_identity, pose_error,
                                 validate_a_localization, validate_arrival)
from r3_scan_map_check import validate_navigation_scan_match


def read_goal(path, selected_map):
    value = json.loads(path.read_text())
    if (not isinstance(value, dict) or value.get('schema') != SCHEMA
            or value.get('purpose') != PURPOSE or value.get('frame_id') != 'map'
            or not value.get('goal_id') or not value.get('saved_at_utc')):
        raise ValueError('Cần điểm giao hàng D đã lưu trong hệ tọa độ map.')
    pose = value.get('pose_xyyaw')
    if (not isinstance(pose, list) or len(pose) != 3
            or any(type(item) not in (int, float) or not math.isfinite(item) for item in pose)
            or abs(pose[2]) > math.pi):
        raise ValueError('Tọa độ hoặc hướng của D không hợp lệ.')
    if (value.get('map') or {}).get('fingerprint_sha256') != selected_map['fingerprint_sha256']:
        raise ValueError('Điểm D không thuộc bản đồ đang dùng.')
    return value


def pickup_reference(path, *, resume_map=None):
    result = json.loads(path.read_text())
    lift = result.get('lift') or {}
    feedback = lift.get('feedback') or {}
    height = lift.get('height_requested_m')
    generation = feedback.get('home_completion_count')
    run_id = (result.get('localization') or {}).get('run_id')
    if (result.get('status') != 'PICKUP_COMPLETE' or not run_id
            or type(height) not in (int, float) or not math.isclose(height, .08, abs_tol=1e-8)
            or type(generation) is not int or not 0 <= generation <= 0xffffffff
            or feedback.get('homed') is not True):
        raise ValueError('Chưa có kết quả lấy hàng và nâng càng hợp lệ trong lượt này.')
    if resume_map is not None:
        previous = json.loads((path.parent.parent/'delivery_arrival.json').read_text())
        if (previous.get('status') != 'DELIVERY_BLOCKED' or previous.get('phase') != 'navigate'
                or 'goal_sent_ros_s' in (previous.get('delivery_timing') or {})
                or (result.get('localization') or {}).get('map_file') != str(resume_map.resolve())):
            raise ValueError('Chỉ tiếp tục lượt đã nâng pallet và dừng trước khi gửi goal D trên bản đồ này.')
        sample = feedback.get('sample_ms')
        stamp = feedback.get('published_ros_s')
        if (type(sample) is not int or not 0 <= sample <= 0xffffffff
                or type(stamp) not in (int, float) or not math.isfinite(stamp)):
            raise ValueError('Thiếu mốc phản hồi ESP32 từ lúc nâng pallet.')
    return run_id, height, generation, feedback


def validate_lift_continuity(before, current):
    elapsed = current['published_ros_s']-before['published_ros_s']
    ticks = (current['sample_ms']-before['sample_ms']) & 0xffffffff
    if not 0 < ticks < 0x80000000 or not math.isclose(ticks/1000., elapsed, rel_tol=.005, abs_tol=2.):
        raise ValueError('Phản hồi ESP32 không nối tiếp lượt vừa nâng pallet.')


class DeliveryMission(RecognitionGoal):
    def __init__(self, action, map_file, goal, pickup, *, resume_delivery=False):
        super().__init__(node_name='r3_delivery_'+action, selected_map_file=map_file,
                         continuous_localization=True, navigation_action=PRIVATE_ACTION)
        self.goal = goal
        self.target = goal['pose_xyyaw']
        self.pickup_run_id, self.height, self.home_generation, self.pickup_feedback = pickup
        self.resume_delivery = resume_delivery
        self.delivery_run_id = None if resume_delivery else self.pickup_run_id
        self.lift = LiftControl(self)
        self.wheels = HomeBaseGuard()
        self.next_navigation_graph_check = 0.
        self.delivery_timing = {'raised_feedback_ros_s': self.pickup_feedback.get('published_ros_s')}
        self.create_subscription(Odometry, '/base_controller/odom', self.on_wheels,
                                 qos_profile_sensor_data)

    def on_wheels(self, msg):
        self.wheels.update(msg, time.monotonic(), self.now())

    def lifted(self):
        value = self.lift.at_travel_height(self.height, self.home_generation)
        if self.resume_delivery:
            validate_lift_continuity(self.pickup_feedback, value)
        return value

    def command_owners(self, *, navigation):
        relay = self.get_fully_qualified_name() if self.base_output is not None else None
        check_navigation_command_owners(self, relay=relay, navigation=navigation)

    def check_session(self, *, navigation):
        data = self.stationary_status
        if not isinstance(data, dict) or not fresh(data.get('published_ros_s'), self.now(), .75):
            raise ValueError('Thiếu trạng thái định vị mới.')
        run_id = data.get('run_id')
        if (not run_id or self.delivery_run_id not in (None, run_id)
                or data.get('map_file') != self.selected_map_file
                or data.get('navigation_monitor') is not navigation or data.get('localization_fault')):
            raise RuntimeError('Phiên định vị giao hàng hoặc bản đồ đã thay đổi.')
        self.delivery_run_id = run_id
        return data

    def switch_to_navigation(self):
        switch = self.create_client(Trigger, '/pallet/r3/localization/enable_delivery_navigation')
        next_attempt = 0.
        reason = ''

        def request_handoff():
            nonlocal next_attempt, reason
            if time.monotonic() < next_attempt:
                raise ValueError(reason)
            if not switch.service_is_ready():
                raise ValueError('Chưa tìm thấy dịch vụ bàn giao định vị sang Nav2.')
            self.lifted()
            self.command_owners(navigation=False)
            existing = (self.stationary_status or {}).get('delivery_handoff')
            data = self.check_session(navigation=self.resume_delivery or existing is not None)
            if (data.get('robot_stationary') is not True or not self.wheels.ready(time.monotonic(), self.now())
                    or abs(self.wheels.twist[0]) >= .005 or abs(self.wheels.twist[1]) >= .01):
                raise ValueError('Chờ odom xác nhận bánh xe đã dừng để bàn giao sang Nav2.')
            if existing is not None:
                return existing
            reply = self.await_future(switch.call_async(Trigger.Request()), time.monotonic()+5.)
            if not reply.success:
                reason = 'Chưa bàn giao định vị sang Nav2: '+reply.message
                next_attempt = time.monotonic()+.5
                raise ValueError(reason)
            return json.loads(reply.message)

        handoff = self.wait_check(request_handoff, 45.)

        def switched():
            current = (self.stationary_status or {}).get('delivery_handoff')
            if (current != handoff or not isinstance(current, dict)
                    or current.get('run_id') != self.delivery_run_id
                    or current.get('map_file') != self.selected_map_file):
                raise ValueError('Chờ trạng thái xác nhận bàn giao định vị cho chuyến giao hàng.')
            data = self.check_session(navigation=True)
            return validate_source(data, self.selected_map_file, self.now(),
                                   validate_scan=validate_navigation_scan_match)

        self.wait_check(switched, 5.)
        return dict(status='DELIVERY_NAVIGATION_READY', handoff=handoff,
                    goal_id=self.goal['goal_id'])

    def on_safe_velocity(self, message):
        if not self.motion_enabled:
            return
        try:
            self.check_session(navigation=True)
            self.lifted()
        except (ValueError, RuntimeError):
            self.base_output.publish(Twist())
            return  # The action monitor cancels and reports the fault.
        components = (message.linear.x, message.angular.z)
        if not all(math.isfinite(value) for value in components):
            self.base_output.publish(Twist())
            return
        safe = Twist()
        safe.linear.x, safe.angular.z = components
        self.base_output.publish(safe)
        if any(abs(value) > 1e-6 for value in components):
            self.delivery_timing.setdefault('first_wheel_command_ros_s', self.now())

    def stationary_pose(self):
        self.check_session(navigation=True)
        pose = self.current_pose(require_still=True)
        validate_navigation_scan_match(self.map_match)
        self.a_quality = validate_a_localization(self.amcl.pose.covariance,
            self.particle_gate.metrics, self.recent_a_poses(), pose, teaching=False)
        return pose

    def monitor_navigation_session(self):
        super().monitor_navigation_session()
        self.check_session(navigation=True)
        self.lifted()
        now = time.monotonic()
        if self.navigation_action_active and now >= self.next_navigation_graph_check:
            self.command_owners(navigation=True)
            self.next_navigation_graph_check = now+.5

    def drive(self):
        def ready_to_drive():
            if not self.client.server_is_ready():
                raise ValueError('Nav2 chưa nhận goal cho D.')
            self.check_navigation_relay()
            self.lifted()
            return self.stationary_pose()

        # The loaded collision monitor is isolated. This relay stays closed
        # until Nav2 accepts the saved D goal.
        self.open_navigation_relay(self.on_safe_velocity)
        start = self.wait_check(ready_to_drive, 35.)
        self.delivery_timing['navigation_ready_ros_s'] = self.now()
        self.navigation_run_id = self.delivery_run_id
        self.navigation_map_signature = self.map_signature
        self.navigation_observation_stamp = self.stationary_snapshot['amcl_stamp_s']
        command = NavigateToPose.Goal()
        command.pose.header.frame_id = 'map'
        command.pose.header.stamp = self.get_clock().now().to_msg()
        command.pose.pose.position.x, command.pose.pose.position.y = self.target[:2]
        command.pose.pose.orientation.z = math.sin(self.target[2]/2.)
        command.pose.pose.orientation.w = math.cos(self.target[2]/2.)
        print(f'Nav2 đang chở pallet tới D tại X={self.target[0]:.3f} m, '
              f'Y={self.target[1]:.3f} m, hướng {math.degrees(self.target[2]):.2f}°.', flush=True)
        self.navigation_action_active = True
        try:
            self.delivery_timing['goal_sent_ros_s'] = self.now()
            self.goal_handle = self.await_future(self.client.send_goal_async(command), time.monotonic()+8.)
            if not self.goal_handle.accepted:
                raise RuntimeError('Nav2 từ chối goal D.')
            self.result_future = self.goal_handle.get_result_async()
            self.motion_enabled = True
            result = self.await_future(self.result_future, time.monotonic()+900., check_localization=True)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'Nav2 chưa tới D (status {result.status}): {result.result.error_msg}')
        finally:
            self.motion_enabled = False
            self.base_output.publish(Twist())
            self.navigation_action_active = False
            self.cancel()

        def arrived():
            self.monitor_navigation_session()
            pose = self.stationary_pose()
            try:
                xy, yaw = validate_arrival(pose, self.target, label='D')
            except RuntimeError as exc:
                raise ValueError(str(exc)) from exc
            return pose, xy, yaw

        pose, xy, yaw = self.wait_check(arrived, 30.)
        return dict(status='ARRIVED_AT_D', goal_id=self.goal['goal_id'], run_id=self.delivery_run_id,
            pickup_run_id=self.pickup_run_id,
            map_file=self.selected_map_file, map_signature=digest(self.map_signature),
            action_status=result.status, start_pose_xyyaw=list(start), pose_xyyaw=list(pose),
            goal_pose_xyyaw=self.target, position_error_m=xy, heading_error_deg=math.degrees(yaw),
            localization_evidence=self.localization_evidence(), delivery_timing=self.delivery_timing)

    def lower(self, arrival):
        run_id = arrival.get('run_id')
        if self.resume_delivery:
            if not run_id or arrival.get('pickup_run_id') != self.pickup_run_id:
                raise RuntimeError('Xác nhận đến D không thuộc pallet vừa nâng.')
            self.delivery_run_id = run_id
        if (arrival.get('status') != 'ARRIVED_AT_D' or arrival.get('goal_id') != self.goal['goal_id']
                or run_id != self.delivery_run_id
                or arrival.get('map_file') != self.selected_map_file
                or arrival.get('goal_pose_xyyaw') != self.target
                or arrival.get('action_status') != GoalStatus.STATUS_SUCCEEDED):
            raise RuntimeError('Thiếu xác nhận Nav2 đã tới đúng điểm giao D trong lượt này.')

        def parked_at_d():
            pose = self.stationary_pose()
            if arrival['map_signature'] != digest(self.map_signature):
                raise RuntimeError('Bản đồ ROS đã thay đổi sau khi tới D.')
            try:
                validate_arrival(pose, self.target, label='D')
            except RuntimeError as exc:
                raise ValueError(str(exc)) from exc
            xy, yaw = pose_error(pose, arrival['pose_xyyaw'])
            if xy > .04 or yaw > math.radians(2.):
                raise ValueError('Tư thế xe đã thay đổi sau khi tới D.')
            self.command_owners(navigation=False)
            self.lift.check_owners()
            self.lifted()
            if (not self.wheels.ready(time.monotonic(), self.now())
                    or abs(self.wheels.twist[0]) >= .005 or abs(self.wheels.twist[1]) >= .01):
                raise ValueError('Chờ encoder xác nhận bánh xe đứng yên tại D.')
            return pose

        pose = self.wait_check(parked_at_d, 35.)
        self.wheels.arm()
        self.lift.begin_lower(self.height, self.home_generation)
        print('Đã dừng tại D. Đang hạ càng xuống vị trí giao.', flush=True)
        next_graph_check = 0.
        next_lift_tick = 0.
        try:
            while rclpy.ok():
                self.spin_once()
                now = time.monotonic()
                if now >= next_graph_check:
                    self.command_owners(navigation=False)
                    self.lift.check_owners()
                    next_graph_check = now+.5
                if (not self.wheels.check(now, self.now())
                        or abs(self.wheels.twist[0]) >= .005 or abs(self.wheels.twist[1]) >= .01):
                    raise RuntimeError('Encoder chưa xác nhận xe đứng yên trong khi hạ càng.')
                if now >= next_lift_tick:
                    next_lift_tick = now+.05
                    if self.lift.lower_tick(self.height, self.home_generation):
                        return dict(status='DELIVERY_COMPLETE', goal_id=self.goal['goal_id'],
                                    run_id=self.delivery_run_id, pickup_run_id=self.pickup_run_id,
                                    pose_xyyaw=list(pose),
                                    navigation_arrival=arrival, lift=self.lift.report(),
                                    pallet_contact_sensor=False, forks_withdrawn=False)
            raise RuntimeError('ROS dừng trong khi hạ càng.')
        finally:
            if rclpy.ok():
                for _ in range(3):
                    self.lift.stop()
                    self.spin_once()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('inspect', 'handoff', 'navigate', 'lower'))
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--goal', default=DEFAULT_GOAL, type=Path)
    parser.add_argument('--pickup-result', type=Path)
    parser.add_argument('--resume-delivery', action='store_true')
    parser.add_argument('--arrival', type=Path)
    parser.add_argument('--handoff-output', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.action != 'inspect' and (not args.pickup_result or not args.output):
        parser.error('Cần --pickup-result và --output cho thao tác giao hàng.')
    if args.resume_delivery and args.pickup_result is None:
        parser.error('--resume-delivery cần --pickup-result.')
    if args.action == 'lower' and not args.arrival:
        parser.error('Cần --arrival để xác nhận đã tới D trước khi hạ càng.')
    node = None
    code = 0
    try:
        selected_map = map_identity(args.map)
        goal = read_goal(args.goal, selected_map)
        pickup = (pickup_reference(args.pickup_result, resume_map=args.map if args.resume_delivery else None)
                  if args.pickup_result is not None else None)
        if args.resume_delivery:
            original_goal = args.pickup_result.parent.parent/'delivery_goal.json'
            if goal != read_goal(original_goal, selected_map):
                raise ValueError('Điểm D khác với điểm đã chọn ở lượt trước.')
        if args.action == 'inspect':
            result = dict(status='DELIVERY_GOAL_VALID', goal_id=goal['goal_id'],
                          pose_xyyaw=goal['pose_xyyaw'])
            if args.output is not None:
                atomic_json(args.output, goal)
        else:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = DeliveryMission(args.action, args.map, goal, pickup, resume_delivery=args.resume_delivery)
            if args.action == 'handoff':
                result = node.switch_to_navigation()
            elif args.action == 'navigate':
                handoff = node.switch_to_navigation()
                if args.handoff_output is not None:
                    atomic_json(args.handoff_output, handoff)
                result = node.drive()
            else:
                result = node.lower(json.loads(args.arrival.read_text()))
    except (OSError, TypeError, KeyError, ValueError, RuntimeError, TransformException,
            KeyboardInterrupt, ExternalShutdownException) as exc:
        result = dict(status='DELIVERY_BLOCKED', phase=args.action, reason=str(exc) or 'Đã hủy.')
        if node is not None and args.action == 'navigate':
            result['delivery_timing'] = node.delivery_timing
        code = 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if args.output is not None and args.action != 'inspect':
        atomic_json(args.output, result)
    print(json.dumps({k: result[k] for k in ('status', 'phase', 'reason', 'goal_id', 'pose_xyyaw')
                      if k in result}, ensure_ascii=False), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
