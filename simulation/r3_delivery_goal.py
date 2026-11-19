#!/usr/bin/env python3
"""Teach a delivery pose with the existing RViz Nav2 Goal; save only explicitly.

The public NavigateToPose action forwards to a private Nav2 action after the
existing localization gates pass. This node never commands wheels or the lift.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from r3_recognition_goal import (
    ARRIVAL_XY_TOLERANCE_M, ARRIVAL_YAW_TOLERANCE_RAD, RecognitionGoal,
    atomic_json, create_goal_json, map_identity, validate_a_localization,
    validate_a_scan_match, validate_arrival, yaw_of,
)
from r3_stop_localization import holds_session_lock, process_identity
from r3_delivery_topics import PRIVATE_ACTION


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GOAL = ROOT/'simulation/r3/delivery_goal.json'
SAVE_SERVICE = '/pallet/r3/delivery/save'
SCHEMA = 'r3_delivery_goal_v1'
PURPOSE = 'pallet_delivery'
SAVE_HOLD_S = 2.
NAV_HOLD_S = .5


def request_pose(request, now):
    stamped = request.pose
    if stamped.header.frame_id != 'map':
        raise ValueError('Chọn goal trong hệ tọa độ map.')
    p, q = stamped.pose.position, stamped.pose.orientation
    if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or abs(p.z) > .05 or abs(q.x) > .01 or abs(q.y) > .01):
        raise ValueError('Goal phải là tư thế phẳng hợp lệ.')
    pose = (p.x, p.y, yaw_of(q))
    stamp = stamped.header.stamp.sec+stamped.header.stamp.nanosec*1e-9
    if stamp and not -.5 <= now-stamp <= 5.:
        raise ValueError('Goal đã cũ hoặc đồng hồ laptop/Pi chưa đồng bộ; chọn lại goal.')
    if request.behavior_tree:
        raise ValueError('Dùng cây điều hướng mặc định khi dạy điểm giao hàng.')
    return pose


def delivery_value(pose, selected_map, requested_pose, evidence, arrival):
    return dict(schema=SCHEMA, purpose=PURPOSE, frame_id='map',
        goal_id=str(uuid.uuid4()),
        saved_at_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        pose_xyyaw=list(pose), map=selected_map,
        source='operator_confirmed_stationary_delivery_pose',
        requested_goal_pose_xyyaw=list(requested_pose),
        pose=dict(position=dict(x=pose[0], y=pose[1], z=0.),
            orientation=dict(x=0., y=0., z=math.sin(pose[2]/2), w=math.cos(pose[2]/2))),
        user_confirmation=dict(method='explicit_delivery_save_service'),
        localization_evidence=evidence, navigation_arrival=arrival,
        arrival_limits=dict(position_m=ARRIVAL_XY_TOLERANCE_M,
                            heading_deg=math.degrees(ARRIVAL_YAW_TOLERANCE_RAD)),
        physical_alignment_verified=False, pallet_placement_verified=False)


def delivery_session(root=ROOT):
    folder = root/'simulation/output/r3_live'
    state = json.loads((folder/'localization_session.json').read_text())
    pid = state.get('pid')
    if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 1:
        raise ValueError('Danh tính phiên giao hàng không hợp lệ.')
    identity = process_identity(pid)
    expected = str(root/'simulation/run_r3_live_trial.sh').encode()
    if (state.get('mode') != 'delivery_teaching' or identity is None
            or expected not in identity[0] or b'--teach-delivery' not in identity[0]
            or state.get('process_start_ticks') != identity[1]
            or not holds_session_lock(pid, folder/'.trial.lock')):
        raise ValueError('Chưa có phiên dạy điểm giao hàng đang chạy.')
    return state


class DeliveryCancelled(RuntimeError):
    pass


class DeliveryGoal(RecognitionGoal):
    def __init__(self, map_file, output, goal_file=DEFAULT_GOAL, *, context=None):
        super().__init__(node_name='r3_delivery_goal', selected_map_file=map_file,
            continuous_localization=True, navigation_action=PRIVATE_ACTION, context=context)
        self.selected_map = map_identity(map_file)
        self.output = Path(output)
        self.goal_file = Path(goal_file)
        # Never allow the delivery save command to replace the taught A file.
        if self.goal_file.resolve() == (ROOT/'simulation/r3/recognition_A.json').resolve():
            raise ValueError('Điểm giao hàng phải dùng tệp riêng với A.')
        self.phase = 'WAITING_FOR_LOCALIZATION'
        self.reason = 'Đang chờ định vị; chưa nhận goal.'
        self.busy = False
        self.quality_since = None
        self.qualified_pose = None
        self.requested_pose = None
        self.arrival = None
        self.saved = None
        self.outer_goal = None
        self.pending_send = None
        self.waiters = []
        self.fatal = None
        self.last_report = None
        self.next_marker = 0.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/pallet/r3/delivery/status', qos)
        # Existing laptop RViz already displays this marker topic.
        self.markers_pub = self.create_publisher(MarkerArray,
            '/pallet/r3/localization/a_markers', qos)
        self.save_service = self.create_service(Trigger, SAVE_SERVICE, self.on_save)
        self.server = ActionServer(self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(.1, self.tick)

    def quality_pose(self):
        pose = self.current_pose(require_still=True)
        validate_a_scan_match(self.map_match, teaching=False)
        self.a_quality = validate_a_localization(self.amcl.pose.covariance,
            self.particle_gate.metrics, self.recent_a_poses(), pose, teaching=False)
        if not self.a_scan_quality_ready():
            raise ValueError('Chờ scan LiDAR/bản đồ và tư thế ổn định.')
        status = self.stationary_status
        if status.get('navigation_monitor') is not True:
            raise ValueError('Phiên định vị chưa ở chế độ điều hướng.')
        if status.get('localization_fault'):
            raise ValueError(str(status['localization_fault']))
        if self.navigation_run_id is not None:
            self.monitor_navigation_session()
        return pose

    def quality_duration(self):
        return 0. if self.quality_since is None else time.monotonic()-self.quality_since

    def can_save(self):
        return (not self.busy and self.phase == 'ARRIVED' and self.arrival is not None
                and self.qualified_pose is not None and self.quality_duration() >= SAVE_HOLD_S
                and self.fatal is None)

    def tick(self):
        try:
            if self.navigation_action_active:
                self.monitor_navigation_session()
            if not self.busy or self.phase == 'VERIFYING_ARRIVAL':
                try:
                    self.qualified_pose = self.quality_pose()
                    if self.arrival is not None:
                        validate_arrival(self.qualified_pose, self.requested_pose)
                    if self.quality_since is None:
                        self.quality_since = time.monotonic()
                    if self.phase in ('WAITING_FOR_LOCALIZATION', 'READY_FOR_GOAL'):
                        self.phase = 'READY_FOR_GOAL'
                        self.reason = 'Đã định vị. Dùng Nav2 Goal chọn điểm và kéo hướng giao hàng.'
                    elif self.phase == 'ARRIVED':
                        self.reason = 'Xe đã tới và dừng. Chờ bạn kiểm tra rồi xác nhận “lưu”.'
                except (ValueError, RuntimeError, TransformException) as exc:
                    self.quality_since = None
                    self.qualified_pose = None
                    self.reason = str(exc)
                    if self.phase == 'READY_FOR_GOAL':
                        self.phase = 'WAITING_FOR_LOCALIZATION'
            self.publish_status()
        except (ValueError, RuntimeError, TransformException) as exc:
            # Identity/integrity faults during an action cause cancellation.
            self.fatal = str(exc)
            self.reason = str(exc)
        finally:
            waiters, self.waiters = self.waiters, []
            for future in waiters:
                if not future.done():
                    future.set_result(None)

    def publish_status(self):
        report = dict(status=self.phase, reason=self.reason, published_ros_s=self.now(),
            map=self.selected_map, save_ready=self.can_save(), busy=self.busy,
            pose_xyyaw=list(self.qualified_pose) if self.qualified_pose is not None else None,
            requested_goal_pose_xyyaw=self.requested_pose, arrival=self.arrival,
            saved_goal=self.saved, delivery_goal_file=str(self.goal_file),
            delivery_goal_exists=self.goal_file.exists(),
            direct_velocity_commands_from_this_process=0, lift_commands_from_this_process=0)
        atomic_json(self.output, report)
        self.status_pub.publish(String(data=json.dumps(report, ensure_ascii=False, allow_nan=False)))
        key = (self.phase, self.can_save())
        if key != self.last_report:
            self.get_logger().info(json.dumps(report, ensure_ascii=False, allow_nan=False))
            self.last_report = key
        if time.monotonic() < self.next_marker:
            return
        self.next_marker = time.monotonic()+.5
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'delivery_teaching'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.
        pose = self.qualified_pose or self.requested_pose
        if pose is not None:
            marker.pose.position.x, marker.pose.position.y = pose[:2]
        marker.pose.position.z = 1.4
        marker.scale.z = .08
        marker.color.a = 1.
        ready = self.can_save() or self.phase in ('READY_FOR_GOAL', 'SAVED')
        marker.color.r, marker.color.g = (.1, .9) if ready else (1., .2)
        marker.text = 'DELIVERY: '+('READY - SAY LUU TO SAVE' if self.can_save() else self.phase)
        self.markers_pub.publish(MarkerArray(markers=[marker]))

    def on_goal(self, request):
        try:
            if self.busy or self.fatal:
                raise ValueError('Đang xử lý goal hoặc có lỗi; chờ xe dừng trước khi chọn goal mới.')
            pose = request_pose(request, self.now())
            self.quality_pose()  # Recheck live data at the actual user request.
            if self.quality_duration() < NAV_HOLD_S or not self.client.server_is_ready():
                raise ValueError('Định vị/Nav2 chưa sẵn sàng; chờ thông báo READY_FOR_GOAL.')
        except (ValueError, RuntimeError, TransformException) as exc:
            self.get_logger().warn('Từ chối goal giao hàng: '+str(exc))
            return GoalResponse.REJECT
        self.requested_pose = list(pose)
        self.arrival = None
        self.qualified_pose = None
        self.quality_since = None
        self.busy = True
        self.phase = 'NAVIGATING'
        self.reason = 'Đang đi tới goal giao hàng do bạn chọn trên RViz.'
        self.publish_status()
        return GoalResponse.ACCEPT

    async def pause(self):
        future = Future(executor=self.executor)
        self.waiters.append(future)
        await future

    async def await_action(self, future, deadline, outer, *, allow_cancel=True):
        while not future.done():
            if allow_cancel and outer.is_cancel_requested:
                raise DeliveryCancelled('Bạn đã hủy goal giao hàng.')
            if self.fatal:
                raise RuntimeError(self.fatal)
            self.monitor_navigation_session()
            if time.monotonic() >= deadline:
                raise RuntimeError('Hết thời gian chờ Nav2 tới điểm giao hàng.')
            await self.pause()
        return future.result()

    def on_feedback(self, feedback):
        if self.outer_goal is not None and self.outer_goal.is_active:
            self.outer_goal.publish_feedback(feedback.feedback)

    async def cancel_inner(self):
        # A goal response may be in flight when a localization fault occurs.
        deadline = time.monotonic()+5.
        if self.pending_send is not None and self.goal_handle is None:
            while not self.pending_send.done() and time.monotonic() < deadline:
                await self.pause()
            if not self.pending_send.done():
                self.fatal = 'Không xác nhận được goal Nav2 đang gửi; đóng phiên để dừng Nav2.'
                return
            self.goal_handle = self.pending_send.result()
        handle = self.goal_handle
        if handle is None or not handle.accepted:
            return
        if self.result_future is None:
            self.result_future = handle.get_result_async()
        if self.result_future.done():
            return
        cancel = handle.cancel_goal_async()
        while (not cancel.done() or not self.result_future.done()) and time.monotonic() < deadline:
            await self.pause()
        if not self.result_future.done():
            self.fatal = 'Nav2 chưa xác nhận hủy goal; đóng phiên để dừng các node điều hướng.'

    async def execute(self, outer):
        self.outer_goal = outer
        self.goal_handle = self.result_future = self.pending_send = None
        result_message = NavigateToPose.Result()
        succeeded = False
        cancelled = False
        try:
            self.navigation_run_id = self.stationary_status['run_id']
            self.navigation_map_signature = self.map_signature
            self.navigation_observation_stamp = self.stationary_snapshot['amcl_stamp_s']
            self.navigation_action_active = True
            request = copy.deepcopy(outer.request)
            request.pose.header.stamp = self.get_clock().now().to_msg()
            self.pending_send = self.client.send_goal_async(request, feedback_callback=self.on_feedback)
            self.goal_handle = await self.await_action(self.pending_send, time.monotonic()+10., outer,
                                                       allow_cancel=False)
            if not self.goal_handle.accepted:
                raise RuntimeError('Nav2 từ chối goal giao hàng.')
            self.result_future = self.goal_handle.get_result_async()
            response = await self.await_action(self.result_future, time.monotonic()+900., outer)
            result_message = response.result
            if outer.is_cancel_requested:
                raise DeliveryCancelled('Bạn đã hủy goal giao hàng.')
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Nav2 chưa tới điểm giao hàng (action status {response.status}).')
            self.phase = 'VERIFYING_ARRIVAL'
            self.quality_since = None
            self.navigation_action_active = False
            deadline = time.monotonic()+30.
            while True:
                if outer.is_cancel_requested:
                    raise DeliveryCancelled('Bạn đã hủy goal giao hàng.')
                self.monitor_navigation_session()
                if self.qualified_pose is not None and self.quality_duration() >= NAV_HOLD_S:
                    xy, yaw = validate_arrival(self.quality_pose(), self.requested_pose)
                    self.arrival = dict(action_status=response.status,
                        pose_xyyaw=list(self.qualified_pose), position_error_m=xy,
                        heading_error_deg=math.degrees(yaw), run_id=self.navigation_run_id,
                        occupancy_map_signature=self.map_signature,
                        verified_ros_s=self.now())
                    succeeded = True
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('Chưa xác nhận được xe dừng và định vị hợp lệ tại goal: '+self.reason)
                await self.pause()
        except DeliveryCancelled as exc:
            cancelled = True
            self.reason = str(exc)
        except (ValueError, RuntimeError, TransformException) as exc:
            self.reason = str(exc)
            # Jazzy's result has a uint16 error_code but no UNKNOWN constant.
            result_message.error_code = 1
            result_message.error_msg = self.reason
        finally:
            self.navigation_action_active = False
            if not succeeded:
                await self.cancel_inner()
            self.outer_goal = None
            self.pending_send = None
            self.goal_handle = self.result_future = None
            self.busy = False
            self.quality_since = None
            self.qualified_pose = None
            self.phase = 'ARRIVED' if succeeded else 'BLOCKED'
            if succeeded:
                self.reason = 'Xe đã tới goal. Đang kiểm tra dừng ổn định trước khi cho lưu.'
                outer.succeed()
            elif cancelled:
                outer.canceled()
            else:
                outer.abort()
            self.publish_status()
        return result_message

    def on_save(self, request, response):
        del request
        try:
            if not self.can_save():
                raise ValueError('Chỉ lưu sau khi goal đã tới và xe dừng ổn định 2 giây. '+self.reason)
            pose = self.quality_pose()
            validate_arrival(pose, self.requested_pose)
            if (self.arrival['run_id'] != self.stationary_status['run_id']
                    or self.arrival['occupancy_map_signature'] != self.map_signature):
                raise ValueError('Phiên định vị/bản đồ đã thay đổi sau khi tới goal.')
            selected_map = map_identity(self.selected_map_file)
            if selected_map != self.selected_map:
                raise ValueError('Tệp bản đồ đã thay đổi; chưa lưu điểm giao hàng.')
            value = delivery_value(pose, selected_map, self.requested_pose,
                                   self.localization_evidence(), copy.deepcopy(self.arrival))
            create_goal_json(self.goal_file, value)
            self.saved = dict(path=str(self.goal_file), goal_id=value['goal_id'],
                              pose_xyyaw=list(pose), heading_deg=math.degrees(pose[2]))
            self.phase = 'SAVED'
            self.reason = 'Đã lưu điểm giao hàng theo xác nhận của bạn.'
            response.success = True
            response.message = json.dumps(self.saved, ensure_ascii=False)
            self.publish_status()
        except FileExistsError:
            response.success = False
            response.message = 'Đã có điểm giao hàng; giữ nguyên tệp hiện có, chưa ghi đè.'
        except (OSError, ValueError, RuntimeError, TransformException) as exc:
            response.success = False
            response.message = str(exc)
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    serve = sub.add_parser('serve')
    serve.add_argument('--map', type=Path, required=True)
    serve.add_argument('--output', type=Path, required=True)
    serve.add_argument('--goal-file', type=Path, default=DEFAULT_GOAL)
    save = sub.add_parser('save')
    save.add_argument('--confirm', required=True, choices=('lưu',),
                      help='Chỉ dùng sau khi người vận hành xác nhận lưu.')
    sub.add_parser('status')
    args = parser.parse_args()
    if args.command != 'serve':
        try:
            state = delivery_session()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            parser.error(str(exc))
        if args.command == 'status':
            report = json.loads((Path(state['run_dir'])/'delivery_status.json').read_text())
            if not -.05 <= time.time()-report['published_ros_s'] <= 2.:
                report.update(save_ready=False, status='STALE', reason='Báo cáo đã cũ.')
            print(json.dumps(report, indent=2, ensure_ascii=False))
            return 0
    rclpy.init()
    if args.command == 'save':
        node = rclpy.create_node('r3_delivery_save_request')
        try:
            client = node.create_client(Trigger, SAVE_SERVICE)
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Chưa có dịch vụ lưu điểm giao hàng.')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            if not future.done():
                raise RuntimeError('Chưa nhận được phản hồi lưu; kiểm tra status trước khi thử lại.')
            response = future.result()
            print(response.message)
            return 0 if response.success else 2
        finally:
            node.destroy_node()
            rclpy.shutdown()
    node = DeliveryGoal(args.map, args.output, args.goal_file)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and (node.fatal is None or node.busy):
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        # The owning launcher stops Nav2 and all owned hardware processes.
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 2 if node.fatal else 0


if __name__ == '__main__':
    raise SystemExit(main())
