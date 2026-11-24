"""Exercise odometry and lift feedback in the pickup sequence."""

import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import CollisionMonitorState
    from std_msgs.msg import String
    from r3_lift_control import FIRMWARE_PROFILE, HOME_RETRY_FAULT, HomeBaseGuard, LiftControl
    from collections import deque
    from r3_real_trial import (
        OBSTACLE_CLEAR_SETTLE_S, OBSTACLE_HOLD_TIMEOUT_S, RealTrial)
except ImportError as exc:
    raise unittest.SkipTest(str(exc))


def armed_pickup():
    wall = time.monotonic()
    pose = (1., 2., .4)
    return SimpleNamespace(
        armed=True, finished=False, started=True, pickup=True, state='APPROACH',
        localization=None, monitor_stop=False, pose=pose, twist=(.06, -.13),
        odom_wall=wall, odom_capture_stamp=99.98,
        yolo_enabled=False, yolo_wall=wall, scan_wall={'lidar': wall},
        lidar_stable_since=wall-1., last_control_pose=pose,
        sensor_hold_since=None, lift=Mock(), braking_since=None, c_braking_since=None,
        insertion_lock_wait_since=None,
        obstacle_hold_since=None, obstacle_clear_since=None, obstacle_hold_polygon='',
        obstacle_holds=deque(maxlen=64),
        stop_at_c=False, arrive_at_c=False, c_wait_started=None, c_verified=None,
        last_tick=wall-.05, last_progress_pose=pose, last_progress_wall=wall,
        follow=SimpleNamespace(step=Mock(return_value=(.06, -.04)), reset_motion=Mock(),
                               state='APPROACH', insertion_started=False),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        get_logger=Mock(),
        ensure_insertion_frame=Mock(return_value=True), record_pose=Mock(),
        send=Mock(), fail=Mock(), hold_for_sensors=Mock(), diagnostics=Mock(),
    )


class OdometryControlTests(unittest.TestCase):
    def test_pickup_uses_yaw_and_turn_rate_from_odom_without_imu(self):
        node = armed_pickup()
        RealTrial.control_tick(node)
        node.send.assert_called_once_with(.06, -.04)
        feedback = node.follow.step.call_args.kwargs
        self.assertAlmostEqual(feedback['insertion_yaw'], node.pose[2])
        self.assertAlmostEqual(feedback['insertion_yaw_rate'], node.twist[1])
        node.hold_for_sensors.assert_not_called()
        node.fail.assert_not_called()

    def test_stale_odom_still_holds_the_robot(self):
        node = armed_pickup()
        node.odom_capture_stamp = 99.7
        RealTrial.control_tick(node)
        node.follow.step.assert_not_called()
        node.hold_for_sensors.assert_called_once()
        self.assertEqual(node.hold_for_sensors.call_args.args[0], ['odom=0.30s'])

    def test_invalid_odom_turn_rate_triggers_stop(self):
        node = armed_pickup()
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.header.stamp.sec = 100
        msg.pose.pose.orientation.w = 1.
        msg.twist.twist.angular.z = float('nan')
        RealTrial.on_odom(node, msg)
        node.fail.assert_called_once_with('invalid /odom frame, pose or velocity')
        self.assertEqual(node.twist, (.06, -.13))


class ParkedLiftTests(unittest.TestCase):
    def setUp(self):
        self.node = armed_pickup()
        self.node.state = 'LIFTING'
        self.node.get_logger = Mock()
        self.node.lift_lidar_source_warnings = 0
        self.node.lift_next_graph_check = 0.
        self.node.lift_history = []
        self.node.lift.report.return_value = {}
        self.node.lift.raise_tick.return_value = True
        wheels = self.node.lift_wheels = HomeBaseGuard()
        wheels.pose = (1., 2., .4)
        wheels.twist = (0., 0.)
        wheels.stamp = 99.8
        wheels.received = time.monotonic()
        wheels.arm()
        self.node.tick_parked_lift = lambda wall, now_ros: RealTrial.tick_parked_lift(
            self.node, wall, now_ros)

    def monitor_stop(self, name):
        msg = CollisionMonitorState(action_type=CollisionMonitorState.STOP, polygon_name=name)
        RealTrial.on_collision_state(self.node, msg)

    @patch('r3_real_trial.command_graph_status', return_value=[
        ('publisher', '/cmd_vel_safe', '/r3_collision_monitor', ['/r3_collision_monitor'])])
    def test_lift_keeps_wheels_stopped_and_can_complete_with_stale_navigation_inputs(self, _):
        self.monitor_stop('invalid source')
        self.assertTrue(self.node.monitor_stop)
        self.node.odom_capture_stamp = 98.
        self.node.scan_wall['lidar'] -= 2.
        self.node.localization = {'map_file': 'test-map'}
        RealTrial.control_tick(self.node)
        self.node.send.assert_called_once_with()
        self.node.fail.assert_not_called()
        self.node.lift.raise_tick.assert_called_once()
        self.assertTrue(self.node.finished)
        self.assertEqual(self.node.state, 'PICKUP_COMPLETE')

    def test_obstacle_and_invalid_source_during_drive_still_latch(self):
        self.monitor_stop('StopZone')
        self.node.fail.assert_called_once_with('collision monitor stopped: StopZone')
        self.node.fail.reset_mock()
        self.node.state = 'APPROACH'
        self.monitor_stop('invalid source')
        self.node.fail.assert_called_once_with('collision monitor stopped: invalid source')

    def test_stale_encoder_and_wheel_movement_stop_lift_before_next_raise_command(self):
        self.node.lift_wheels.stamp = 99.4
        RealTrial.control_tick(self.node)
        self.node.fail.assert_called_once()
        self.assertIn('source age 0.600s', self.node.fail.call_args.args[0])
        self.node.lift_wheels.stamp = 99.9
        self.node.lift_wheels.pose = (1.006, 2., .4)
        self.node.fail.reset_mock()
        RealTrial.control_tick(self.node)
        self.node.fail.assert_called_once_with('base moved while lifting at B')
        self.node.lift.raise_tick.assert_not_called()

    @patch('r3_real_trial.command_graph_status', return_value=[
        ('publisher', '/cmd_vel_safe', '/r3_collision_monitor', ['/another_controller'])])
    def test_extra_wheel_command_owner_prevents_lifting(self, _):
        RealTrial.control_tick(self.node)
        self.node.fail.assert_called_once_with('wheel command ownership changed while lifting at B')
        self.node.lift.raise_tick.assert_not_called()


class HomeRetryTests(unittest.TestCase):
    def setUp(self):
        node = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000
        node.create_publisher.side_effect = [Mock(), Mock(), Mock()]
        self.lift = LiftControl(node)
        self.lift.check_owners = Mock()
        self.sample = 1900
        self.feedback()

    def feedback(self, **changes):
        self.sample += 100
        value = dict(serial_connected=True, sample_ms=self.sample, published_ros_s=100.,
                     firmware_profile=FIRMWARE_PROFILE, state='UNHOMED', homed=False,
                     position=0., target=0., velocity=0., lower_active=False, upper_active=False,
                     fault=HOME_RETRY_FAULT, home_completion_count=2, home_skipped_active=False)
        self.lift.on_status(String(data=json.dumps({**value, **changes})))

    def test_retry_requires_explicit_request_and_switches_released(self):
        with self.assertRaisesRegex(ValueError, '--retry-home'):
            self.lift.home()
        for changes in (dict(lower_active=True), dict(upper_active=True), dict(velocity=-.01),
                        dict(fault='BOTH_LIMITS'), dict(state='FAULT'),
                        dict(firmware_profile='HOME_SKIP_ACTIVE_1MM_NEW_CONTACT_V3')):
            with self.subTest(changes=changes):
                self.feedback(**changes)
                with self.assertRaises(ValueError):
                    self.lift.home(retry_home=True)
        self.feedback(published_ros_s=98.)
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.lift.home(retry_home=True)
        self.lift.home_pub.publish.assert_not_called()

    def test_retry_accepts_only_new_home_after_in_flight_retained_fault(self):
        self.lift.home(retry_home=True)
        self.lift.home_pub.publish.assert_called_once()
        self.assertFalse(self.lift.home_complete())
        self.feedback(state='HOMING', velocity=-.01)
        self.assertFalse(self.lift.home_complete())
        with self.assertRaisesRegex(ValueError, HOME_RETRY_FAULT):
            self.lift.status()
        self.feedback(state='HOMING', lower_active=True)
        self.assertFalse(self.lift.home_complete())
        self.feedback(state='IDLE', homed=True, lower_active=True, fault='NONE', home_completion_count=3)
        self.assertTrue(self.lift.home_complete())
        self.lift.target_pub.publish.assert_not_called()

    def test_retry_fails_on_new_fault_even_if_code_matches(self):
        self.lift.home(retry_home=True)
        self.feedback(state='HOMING', velocity=-.01)
        self.assertFalse(self.lift.home_complete())
        self.feedback(state='FAULT')
        with self.assertRaisesRegex(ValueError, HOME_RETRY_FAULT):
            self.lift.home_complete()
        self.lift.home_pub.publish.assert_called_once()

    def test_retry_fails_if_home_bypasses_new_contact(self):
        self.lift.home(retry_home=True)
        self.feedback(state='IDLE', homed=True, lower_active=True, fault='NONE', home_completion_count=3,
                      home_skipped_active=True)
        with self.assertRaisesRegex(ValueError, 'new lower-switch contact'):
            self.lift.home_complete()

    def test_retry_has_acknowledgement_deadline(self):
        self.lift.home(retry_home=True)
        self.lift.requested_at -= 4.
        with self.assertRaisesRegex(ValueError, 'acknowledge in time'):
            self.lift.home_complete()
        self.lift.home_pub.publish.assert_called_once()

    def test_normal_home_can_accept_already_pressed_switch(self):
        self.feedback(fault='NONE', lower_active=True)
        self.lift.home()
        self.feedback(state='IDLE', homed=True, lower_active=True, fault='NONE', home_completion_count=3,
                      home_skipped_active=True)
        self.assertTrue(self.lift.home_complete())


class ObstacleHoldResumeTests(unittest.TestCase):
    def setUp(self):
        self.node = armed_pickup()

    def send_state(self, action_type, polygon_name=''):
        msg = CollisionMonitorState(action_type=action_type, polygon_name=polygon_name)
        RealTrial.on_collision_state(self.node, msg)

    def test_stopzone_pauses_then_resumes_after_sustained_clear(self):
        self.send_state(CollisionMonitorState.STOP, 'StopZone')
        self.node.fail.assert_not_called()
        self.node.send.assert_called_once_with()
        self.assertIsNotNone(self.node.obstacle_hold_since)
        self.assertEqual(len(self.node.obstacle_holds), 1)
        self.assertFalse(self.node.obstacle_holds[0]['resumed'])

        # While blocked, control_tick commands zero and does not call follower.step
        self.node.send.reset_mock()
        RealTrial.control_tick(self.node)
        self.node.send.assert_called_once_with()
        self.node.follow.step.assert_not_called()
        self.assertEqual(self.node.state, 'OBSTACLE_HOLD')

        # Brief DO_NOTHING flicker followed by STOP resets clear timer
        self.send_state(CollisionMonitorState.DO_NOTHING)
        self.assertIsNotNone(self.node.obstacle_clear_since)
        self.send_state(CollisionMonitorState.STOP, 'StopZone')
        self.assertIsNone(self.node.obstacle_clear_since)

        # Sustained DO_NOTHING for < settle time still holds zero
        self.send_state(CollisionMonitorState.DO_NOTHING)
        self.node.obstacle_clear_since = time.monotonic() - (OBSTACLE_CLEAR_SETTLE_S * 0.5)
        self.node.send.reset_mock()
        RealTrial.control_tick(self.node)
        self.node.follow.step.assert_not_called()
        self.assertIsNotNone(self.node.obstacle_hold_since)

        # Sustained DO_NOTHING >= settle time clears hold and resets follower motion
        self.node.obstacle_hold_since = time.monotonic() - 5.0
        self.node.obstacle_clear_since = time.monotonic() - (OBSTACLE_CLEAR_SETTLE_S + 0.05)
        self.node.last_progress_wall = time.monotonic() - 5.0
        RealTrial.control_tick(self.node)
        self.assertFalse(self.node.monitor_stop)
        self.assertIsNone(self.node.obstacle_hold_since)
        self.assertEqual(self.node.state, 'APPROACH')
        self.node.follow.reset_motion.assert_called_once()
        self.assertTrue(self.node.obstacle_holds[0]['resumed'])
        self.assertEqual(self.node.obstacle_holds[0]['outcome'], 'resumed')

        # Next tick resumes normal motion cleanly without tripping progress watchdog
        self.node.send.reset_mock()
        RealTrial.control_tick(self.node)
        self.node.fail.assert_not_called()
        self.node.follow.step.assert_called_once()
        self.node.send.assert_called_once_with(.06, -.04)

    def test_stopzone_timeout_latches_stop(self):
        self.send_state(CollisionMonitorState.STOP, 'StopZone')
        self.node.obstacle_hold_since = time.monotonic() - (OBSTACLE_HOLD_TIMEOUT_S + 0.1)
        RealTrial.control_tick(self.node)
        self.node.fail.assert_called_once()
        self.assertIn('StopZone', self.node.fail.call_args.args[0])
        self.assertEqual(self.node.obstacle_holds[0]['outcome'], 'timeout')

    def test_stopzone_during_braking_delays_lift_and_resets_braking_settle(self):
        self.node.state = 'BRAKING'
        self.node.braking_since = time.monotonic() - 0.6
        self.send_state(CollisionMonitorState.STOP, 'StopZone')
        RealTrial.control_tick(self.node)
        self.node.lift.begin_raise.assert_not_called()
        self.node.fail.assert_not_called()
        self.send_state(CollisionMonitorState.DO_NOTHING)
        self.node.obstacle_clear_since = time.monotonic() - (OBSTACLE_CLEAR_SETTLE_S + 0.05)
        before = time.monotonic()
        RealTrial.control_tick(self.node)
        self.assertGreaterEqual(self.node.braking_since, before - 0.05)
        self.node.lift.begin_raise.assert_not_called()

    def test_sensor_loss_during_obstacle_hold_is_still_caught(self):
        self.send_state(CollisionMonitorState.STOP, 'StopZone')
        self.node.odom_capture_stamp = 99.5
        RealTrial.control_tick(self.node)
        self.node.hold_for_sensors.assert_called_once()
