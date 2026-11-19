"""Delivery teaching with a fake Nav2 action, in a separate DDS domain.

No wheel/lift publisher, serial port or production ROS domain is used.
"""
import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.task import Future
    from std_srvs.srv import Trigger
    from r3_delivery_goal import (DeliveryGoal, PRIVATE_ACTION, SAVE_SERVICE,
                                  delivery_session, request_pose)
except ImportError as exc:
    raise unittest.SkipTest(str(exc))


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT/'ros2_ws/maps/tai_map_real_edited.yaml'


def goal_at(x=1., y=2., yaw=.3):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw/2)
    goal.pose.pose.orientation.w = math.cos(yaw/2)
    return goal


class GoalInputTests(unittest.TestCase):
    def test_planar_map_goal_and_orientation_are_retained(self):
        self.assertEqual(request_pose(goal_at(), 100.), (1., 2., .3))

    def test_wrong_frame_nan_quaternion_stale_and_custom_bt_are_rejected(self):
        invalid = []
        goal = goal_at(); goal.pose.header.frame_id = 'odom'; invalid.append(goal)
        goal = goal_at(); goal.pose.pose.position.x = math.nan; invalid.append(goal)
        goal = goal_at(); goal.pose.pose.orientation.w = 4.; invalid.append(goal)
        goal = goal_at(); goal.pose.header.stamp.sec = 1; invalid.append(goal)
        goal = goal_at(); goal.behavior_tree = '/tmp/override.xml'; invalid.append(goal)
        for goal in invalid:
            with self.subTest(goal=goal), self.assertRaises(ValueError):
                request_pose(goal, 100.)


class DeliveryActionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='r3-delivery-test-')
        self.folder = Path(self.temp.name)
        self.context = Context()
        self.context.init(domain_id=197)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.supervisor = DeliveryGoal(MAP, self.folder/'status.json',
            self.folder/'delivery.json', context=self.context)
        self.peer = Node('delivery_test_peer', context=self.context)
        self.nodes = [self.supervisor, self.peer]
        for node in self.nodes:
            self.executor.add_node(node)
        self.pose = (1., 2., .3)
        self.available = True
        self.finished = False
        self.inner_fail = False
        self.inner_reject = False
        self.inner_cancelled = False
        self.inner_count = 0
        self.inner_goal = None
        self.waiters = []
        self.server = ActionServer(self.peer, NavigateToPose, PRIVATE_ACTION,
            execute_callback=self.inner_execute,
            goal_callback=lambda _: GoalResponse.REJECT if self.inner_reject else GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.peer.create_timer(.02, self.wake)
        self.client = ActionClient(self.peer, NavigateToPose, '/navigate_to_pose')
        self.saver = self.peer.create_client(Trigger, SAVE_SERVICE)
        self.supervisor.quality_pose = self.quality
        self.supervisor.monitor_navigation_session = lambda: None
        self.supervisor.localization_evidence = lambda: {'test': 'isolated synthetic localization'}
        self.supervisor.stationary_status = {'run_id': 'test-run', 'navigation_monitor': True}
        self.supervisor.stationary_snapshot = {'amcl_stamp_s': 100.}
        self.supervisor.map_signature = 'test-map'
        self.pump_until(lambda: self.client.server_is_ready()
                        and self.supervisor.client.server_is_ready() and self.saver.service_is_ready())
        self.pump_until(lambda: self.supervisor.quality_duration() >= .55)

    def tearDown(self):
        # Finish any fake goal before destroying the server.
        self.finished = True
        end = time.monotonic()+.3
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
        self.server.destroy()
        self.supervisor.server.destroy()
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        self.context.shutdown()
        self.temp.cleanup()

    def quality(self):
        if not self.available:
            raise ValueError('test localization missing/stale')
        return self.pose

    def wake(self):
        pending, self.waiters = self.waiters, []
        for future in pending:
            if not future.done():
                future.set_result(None)

    async def inner_execute(self, goal):
        self.inner_count += 1
        self.inner_goal = copy.deepcopy(goal.request)
        while not self.finished and not goal.is_cancel_requested:
            future = Future(executor=self.executor)
            self.waiters.append(future)
            await future
        result = NavigateToPose.Result()
        if goal.is_cancel_requested:
            self.inner_cancelled = True
            goal.canceled()
        elif self.inner_fail:
            result.error_code = 17
            result.error_msg = 'synthetic Nav2 failure'
            goal.abort()
        else:
            goal.succeed()
        return result

    def pump_until(self, predicate, timeout=5.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.02)
        self.assertTrue(predicate(), self.supervisor.output.read_text()
                        if self.supervisor.output.exists() else 'no status')

    def send(self):
        response = self.client.send_goal_async(goal_at())
        self.pump_until(response.done)
        return response.result()

    def save(self):
        future = self.saver.call_async(Trigger.Request())
        self.pump_until(future.done)
        return future.result()

    def arrive(self):
        self.finished = True
        goal = self.send()
        self.assertTrue(goal.accepted)
        done = goal.get_result_async()
        self.pump_until(done.done)
        self.assertEqual(done.result().status, 4)  # STATUS_SUCCEEDED
        self.pump_until(self.supervisor.can_save)
        return goal

    def test_no_navigation_or_goal_file_before_operator_goal(self):
        self.assertEqual(self.inner_count, 0)
        self.assertFalse(self.supervisor.goal_file.exists())
        self.assertFalse(self.save().success)

    def test_unlocalized_goal_is_rejected_and_never_forwarded(self):
        self.available = False
        self.assertFalse(self.send().accepted)
        self.assertEqual(self.inner_count, 0)
        self.assertFalse(self.supervisor.goal_file.exists())

    def test_running_goal_refuses_save_and_another_goal(self):
        goal = self.send()
        self.assertTrue(goal.accepted)
        self.pump_until(lambda: self.inner_count == 1)
        self.assertFalse(self.save().success)
        self.assertFalse(self.send().accepted)
        self.assertFalse(self.supervisor.goal_file.exists())
        cancel = goal.cancel_goal_async()
        self.pump_until(cancel.done)
        result = goal.get_result_async()
        self.pump_until(result.done)
        self.assertTrue(self.inner_cancelled)
        self.assertEqual(result.result().status, 5)  # STATUS_CANCELED
        self.assertFalse(self.supervisor.can_save())

    def test_arrival_does_not_save_until_explicit_request_and_saves_measured_pose(self):
        self.pose = (1.02, 2.01, .31)
        self.arrive()
        self.assertFalse(self.supervisor.goal_file.exists())
        self.assertEqual(request_pose(self.inner_goal, time.time()), (1., 2., .3))
        response = self.save()
        self.assertTrue(response.success, response.message)
        saved = json.loads(self.supervisor.goal_file.read_text())
        self.assertEqual(saved['purpose'], 'pallet_delivery')
        self.assertEqual(saved['pose_xyyaw'], list(self.pose))
        self.assertEqual(saved['requested_goal_pose_xyyaw'], [1., 2., .3])
        self.assertIn('fingerprint_sha256', saved['map'])
        self.assertFalse(saved['pallet_placement_verified'])

    def test_existing_destination_and_A_are_not_overwritten(self):
        existing = b'{"existing_destination": true}\n'
        self.supervisor.goal_file.write_bytes(existing)
        a_file = ROOT/'simulation/r3/recognition_A.json'
        a_before = a_file.read_bytes() if a_file.exists() else None
        self.arrive()
        self.assertFalse(self.save().success)
        self.assertEqual(self.supervisor.goal_file.read_bytes(), existing)
        self.assertEqual(a_file.read_bytes() if a_file.exists() else None, a_before)

    def test_localization_loss_or_pose_outside_arrival_refuses_save(self):
        self.arrive()
        self.available = False
        self.assertFalse(self.save().success)
        self.available = True
        self.pose = (3., 2., .3)
        self.assertFalse(self.save().success)
        self.assertFalse(self.supervisor.goal_file.exists())

    def test_nav2_failure_cannot_be_saved(self):
        self.inner_fail = self.finished = True
        goal = self.send()
        result = goal.get_result_async()
        self.pump_until(result.done)
        self.assertEqual(result.result().status, 6)  # STATUS_ABORTED
        self.assertFalse(self.save().success)
        self.assertFalse(self.supervisor.goal_file.exists())

    def test_nav2_rejection_returns_aborted_instead_of_hanging(self):
        self.inner_reject = True
        goal = self.send()
        result = goal.get_result_async()
        self.pump_until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertFalse(self.save().success)

    def test_session_integrity_fault_cancels_active_nav2(self):
        goal = self.send()
        self.pump_until(lambda: self.inner_count == 1)
        self.supervisor.fatal = 'test map/session changed'
        result = goal.get_result_async()
        self.pump_until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertTrue(self.inner_cancelled)
        self.assertFalse(self.supervisor.can_save())

    def test_next_goal_invalidates_prior_arrival_before_save(self):
        self.arrive()
        self.finished = False
        goal = self.send()
        self.assertTrue(goal.accepted)
        self.assertIsNone(self.supervisor.arrival)
        self.assertFalse(self.save().success)
        cancel = goal.cancel_goal_async()
        self.pump_until(cancel.done)
        result = goal.get_result_async()
        self.pump_until(result.done)


class DeliverySessionTests(unittest.TestCase):
    def test_exited_owner_cannot_authorize_save(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            state = root/'simulation/output/r3_live/localization_session.json'
            state.parent.mkdir(parents=True)
            state.write_text(json.dumps(dict(pid=os.getpid(), mode='delivery_teaching',
                                             process_start_ticks=0)))
            with patch('r3_delivery_goal.process_identity', return_value=None):
                with self.assertRaises(ValueError):
                    delivery_session(root)


if __name__ == '__main__':
    unittest.main()
