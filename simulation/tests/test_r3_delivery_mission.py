"""Exercise the loaded Nav2 settings and lowering guards without hardware."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from r3_amcl_bridge import AmclTrialBridge, TrialCorrection
    from r3_core import digest
    from r3_delivery_mission import DeliveryMission, pickup_reference, validate_lift_continuity
    from r3_delivery_topics import check_navigation_command_owners, delivery_velocity
    from r3_lift_control import DELIVERY_HEIGHT_M, LiftControl
    from r3_navigation_recovery_gate import RecoveryInterlock
    from r3_recognition_goal import RecognitionGoal
    from r3_scan_map_check import validate_navigation_scan_match
    from r3_stationary_localization import StationaryEvidence
except ImportError as exc:
    raise unittest.SkipTest(str(exc))


class LoadedNavigationTests(unittest.TestCase):
    def test_warming_ignores_trial_status_and_cannot_release_before_delivery_handoff(self):
        gate = RecoveryInterlock(wait_for_delivery_handoff=True)
        status = dict(run_id='run', map_file='map', navigation_monitor=False,
                      published_ros_s=10., navigation_recovery={'hold_commands': False})
        command = (.12, 0., .2)
        self.assertFalse(gate.update(status, 10.))
        self.assertEqual(gate.filter(command, 10., 10.)[0], (0., 0., 0.))
        self.assertIsNone(gate.fault)
        self.assertFalse(gate.update({**status, 'navigation_monitor': True}, 10.))
        status.update(navigation_monitor=True, published_ros_s=11.,
                      delivery_handoff={'run_id': 'run', 'map_file': 'map'})
        self.assertTrue(gate.update(status, 11.))
        self.assertEqual(gate.filter(command, 11., 11.)[0], command)

    def test_shared_nav2_holds_between_a_and_certified_d(self):
        gate = RecoveryInterlock(shared_navigation=True)
        command = (.12, 0., .2)
        status = dict(run_id='run', map_file='map', navigation_monitor=True,
                      published_ros_s=10., navigation_recovery={'hold_commands': False})
        self.assertEqual(gate.filter(command, 10., 10.)[0], (0., 0., 0.))
        gate.update(status, 10.)
        self.assertEqual(gate.filter(command, 10., 10.)[0], command)
        status.update(navigation_monitor=False, trial_handoff={'enabled': True}, published_ros_s=11.)
        gate.update(status, 11.)
        self.assertEqual(gate.filter(command, 11., 11.)[0], (0., 0., 0.))
        self.assertIsNone(gate.fault)
        status.update(navigation_monitor=True, published_ros_s=12.)
        gate.update(status, 12.)
        self.assertEqual(gate.filter(command, 12., 12.)[0], (0., 0., 0.))
        status.update(delivery_handoff={'run_id': 'run', 'map_file': 'map'}, published_ros_s=12.1)
        gate.update(status, 12.1)
        self.assertEqual(gate.filter(command, 12.1, 12.1)[0], command)
        status.update(navigation_monitor=False, published_ros_s=12.2)
        gate.update(status, 12.2)
        self.assertEqual(gate.filter(command, 12.2, 12.2)[0], (0., 0., 0.))
        self.assertIsNotNone(gate.fault)

    def test_a_relay_only_forwards_during_its_action_in_same_role(self):
        relay = SimpleNamespace(motion_enabled=False, base_output=Mock(), navigation_run_id='run',
            selected_map_file='map', map_signature='cells', navigation_map_signature='cells',
            now=lambda: 10., forwarded_velocity_commands=0,
            stationary_status=dict(run_id='run', map_file='map', navigation_monitor=True, published_ros_s=10.))
        msg = Twist()
        msg.linear.x = .12
        RecognitionGoal.on_a_safe_velocity(relay, msg)
        relay.base_output.publish.assert_not_called()
        relay.motion_enabled = True
        RecognitionGoal.on_a_safe_velocity(relay, msg)
        self.assertEqual(relay.base_output.publish.call_args.args[0].linear.x, .12)
        relay.stationary_status['trial_handoff'] = {'enabled': True}
        RecognitionGoal.on_a_safe_velocity(relay, msg)
        self.assertEqual(relay.base_output.publish.call_args.args[0].linear.x, 0.)

    def test_private_wheels_allow_multiple_native_behavior_publishers(self):
        nodes = {'/base_controller/cmd_vel': ['cmd_vel_stamper'],
                 delivery_velocity('/cmd_vel_nav'): ['controller_server', 'behavior_server', 'behavior_server'],
                 delivery_velocity('/cmd_vel_nav_checked'): ['r3_navigation_recovery_gate'],
                 delivery_velocity('/cmd_vel_smoothed'): ['velocity_smoother'],
                 delivery_velocity('/cmd_vel_safe'): ['collision_monitor']}
        owner = SimpleNamespace(get_publishers_info_by_topic=lambda topic: [
            SimpleNamespace(node_namespace='/', node_name=name) for name in nodes.get(topic, [])])
        check_navigation_command_owners(owner, navigation=True)
        nodes['/cmd_vel_safe'] = ['unrelated_publisher']
        with self.assertRaisesRegex(ValueError, '/cmd_vel_safe'):
            check_navigation_command_owners(owner, navigation=True)

    def test_delivery_relay_needs_authorized_action_and_current_lift_and_localization(self):
        relay = SimpleNamespace(motion_enabled=False, base_output=Mock(),
                               check_session=Mock(), lifted=Mock(),
                               delivery_timing={}, now=lambda: 10.)
        msg = Twist()
        msg.linear.x = .12
        DeliveryMission.on_safe_velocity(relay, msg)
        relay.base_output.publish.assert_not_called()
        relay.motion_enabled = True
        relay.lifted.side_effect = ValueError('stale lift')
        DeliveryMission.on_safe_velocity(relay, msg)
        self.assertEqual(relay.base_output.publish.call_args.args[0].linear.x, 0.)
        relay.lifted.side_effect = None
        DeliveryMission.on_safe_velocity(relay, msg)
        self.assertEqual(relay.base_output.publish.call_args.args[0].linear.x, .12)
        self.assertEqual(relay.delivery_timing['first_wheel_command_ros_s'], 10.)

    def test_same_nav2_planner_controller_and_tree_with_wider_pallet_envelope(self):
        root = Path(__file__).resolve().parents[2]
        path = root/'camera/detect_astra/launch/r3_navigation.launch.py'
        spec = importlib.util.spec_from_file_location('r3_nav_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        unloaded = module.navigation_parameters()
        loaded = module.navigation_parameters(loaded_delivery=True)
        for key in ('planner_server', 'controller_server', 'bt_navigator', 'velocity_smoother'):
            self.assertEqual(unloaded[key], loaded[key])
        for costmap in ('local_costmap', 'global_costmap'):
            params = loaded[costmap][costmap]['ros__parameters']
            points = json.loads(params['footprint'])
            nose = max(point[0] for point in points)
            y = [point[1] for point in points if point[0] == nose]
            self.assertGreaterEqual(max(y)-min(y), .355)

    def test_drive_sends_saved_d_after_readiness_and_requires_no_second_enter(self):
        pose = (.22, -.32, .3)
        goal_handle = Mock(accepted=True)
        result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0))
        mission = SimpleNamespace(base_output=Mock(), motion_enabled=False, on_safe_velocity=Mock(),
            open_navigation_relay=Mock(), check_navigation_relay=Mock(), lifted=Mock(), client=Mock(),
            delivery_run_id='run', pickup_run_id='run', selected_map_file='map', map_signature='ROS-map',
            stationary_snapshot={'amcl_stamp_s': 73.}, goal={'goal_id': 'D'}, target=list(pose),
            stationary_pose=Mock(return_value=pose), wait_check=lambda check, timeout: check(),
            monitor_navigation_session=Mock(), localization_evidence=Mock(), cancel=Mock(),
            now=lambda: 75., delivery_timing={}, await_future=Mock(side_effect=[goal_handle, result]),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=75))))
        arrival = DeliveryMission.drive(mission)
        mission.client.send_goal_async.assert_called_once()
        command = mission.client.send_goal_async.call_args.args[0]
        self.assertEqual(command.pose.header.frame_id, 'map')
        self.assertEqual(command.pose.pose.position.x, pose[0])
        self.assertEqual(arrival['status'], 'ARRIVED_AT_D')
        self.assertFalse(mission.motion_enabled)
        self.assertEqual(mission.delivery_timing['goal_sent_ros_s'], 75.)
        mission.check_navigation_relay.assert_called_once()

class DeliveryHandoffTests(unittest.TestCase):
    def setUp(self):
        fixture = Path(__file__).parent/'fixtures/handoff_20261117_184021.json'
        source = json.loads(fixture.read_text())['source_localization_ready']
        self.now = 75.
        self.pose = (1., 2., .3)
        self.bridge = SimpleNamespace(args=SimpleNamespace(resume_delivery=False),
            trial_handoff={'enabled': True, 'occupancy_map_signature': 'ROS-map'}, delivery_handoff=None,
            delivery_refresh_after=None, run_id='run', map_file='map', map_signature='ROS-map',
            continuous_localization=True, navigation_monitor=False, stationary_inspection=False,
            active=True, map_received=True, scan_checker=object(), correction=TrialCorrection(),
            global_initialized=True, global_fault=None, stream_clock_reversed=False,
            robot_is_stationary=lambda now: True, get_publishers_info_by_topic=Mock(return_value=[]),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.now*1e9))),
            get_logger=Mock(), stationary_evidence=StationaryEvidence(), parking_refresh_armed=False,
            pending=(), nomotion_future=None, continuous_invalid_after_stamp=None,
            stationary_snapshot={**source['stationary_snapshot'], 'run_id': 'run', 'map_file': 'map',
                                 'occupancy_map_signature': 'ROS-map', 'map_odom_xyyaw': self.pose},
            scan_match=source['scan_map_fit'])
        for method in (AmclTrialBridge._check_handoff_context, AmclTrialBridge._parked_handoff_observation):
            setattr(self.bridge, method.__name__, MethodType(method, self.bridge))
        self.observe_parked(73.5)

    def observe_parked(self, stamp):
        bridge = self.bridge
        bridge.stationary_snapshot['amcl_stamp_s'] = stamp
        bridge.correction.amcl_stamp = stamp
        bridge.correction.measurement_ok = True
        bridge.latest_observation = (stamp, self.pose)
        bridge.stationary_evidence.capture(stamp, self.pose)
        for age in (1.1, .7, .35, .03):
            scan_stamp = self.now-age
            bridge.scan_match['scan_stamp_s'] = scan_stamp
            bridge.stationary_evidence.update_odom(self.pose, (0., 0., 0.), scan_stamp, scan_stamp)
            bridge.stationary_evidence.observe_scan(scan_stamp, bridge.scan_match, scan_stamp)
        bridge.scan_stamp = self.now-.03

    def handoff(self):
        response = SimpleNamespace()
        return AmclTrialBridge.enable_delivery_navigation(self.bridge, None, response)

    def test_handoff_keeps_observation_and_releases_trial_bounds(self):
        response = self.handoff()
        bridge = self.bridge
        self.assertTrue(response.success, response.message)
        self.assertEqual(bridge.correction.amcl_stamp, 73.5)
        self.assertEqual(bridge.correction.last_transform, self.pose)
        self.assertTrue(bridge.navigation_monitor)
        self.assertFalse(bridge.correction.bound_total)
        self.assertTrue(bridge.correction.recover_jumps)
        self.assertIsNone(bridge.delivery_refresh_after)

    def test_delivery_accepts_recorded_5_72_cm_wall_fit_without_relaxing_recognition(self):
        self.bridge.scan_match.update(inlier_ratio=.76, median_distance_m=.0572)
        with self.assertRaisesRegex(ValueError, 'recognition scan/map'):
            self.bridge._parked_handoff_observation(self.now)
        response = self.handoff()
        self.assertTrue(response.success, response.message)

    def test_delivery_still_rejects_fit_outside_navigation_limits(self):
        for fit in (dict(inlier_ratio=.649, median_distance_m=.0572),
                    dict(inlier_ratio=.76, median_distance_m=.121)):
            with self.subTest(fit=fit):
                self.bridge.scan_match.update(fit)
                response = self.handoff()
                self.assertFalse(response.success)
                self.assertIn('navigation scan/map', response.message)

    def test_lift_noise_requires_one_new_parked_observation(self):
        bridge = self.bridge
        bridge.stationary_evidence.update_odom(self.pose, (0., 0., -.01148), self.now, self.now)
        bridge.stationary_evidence.update_odom(self.pose, (0., 0., 0.), self.now+.01, self.now+.01)
        self.now += .02
        response = self.handoff()
        self.assertFalse(response.success)
        self.assertIn('robot motion invalidated', response.message)
        self.assertTrue(bridge.parking_refresh_armed)
        self.assertFalse(bridge.navigation_monitor)
        requested_at = bridge.delivery_refresh_after
        self.assertAlmostEqual(requested_at, self.now)
        self.request_parked_update()
        self.assertFalse(self.handoff().success)
        self.assertFalse(bridge.parking_refresh_armed)
        self.assertEqual(bridge.delivery_refresh_after, requested_at)
        self.now += 1.5
        self.observe_parked(requested_at-.01)
        self.assertFalse(self.handoff().success)
        self.now += 1.5
        self.observe_parked(self.now-1.3)
        response = self.handoff()
        self.assertTrue(response.success, response.message)
        handoff = json.loads(response.message)
        self.assertEqual(handoff['run_id'], 'run')
        self.assertGreater(handoff['source_amcl_stamp_s'], requested_at)

    def test_old_snapshot_without_a_fault_rearms_once_if_matching_window_never_forms(self):
        bridge = self.bridge
        bridge.stationary_evidence.restart_scan_window('waiting for new scans')
        self.now += 1.
        self.assertIsNone(bridge.stationary_evidence.fault)
        self.assertFalse(self.handoff().success)
        self.assertTrue(bridge.parking_refresh_armed)
        bridge.parking_refresh_armed = False
        self.assertFalse(self.handoff().success)
        self.assertFalse(bridge.parking_refresh_armed)

    def request_parked_update(self):
        bridge = self.bridge
        bridge.args.initial_pose = None
        bridge.state_poll = SimpleNamespace(clients={}, next_poll={}, pending={}, states={}, tick=Mock())
        bridge.state_client = object()
        bridge.next_state = bridge.next_nomotion = 0.
        bridge.global_localization = True
        bridge.global_requested = True
        bridge.global_future = None
        bridge.raw_odom = Odometry()
        bridge.raw_odom.pose.pose.orientation.w = 1.
        bridge.latest_scan = LaserScan()
        bridge.stationary_cache_enabled = bridge.seed_refresh_requested = True
        bridge.parked_since_wall = bridge.nomotion_interfaces_since = 8.
        bridge.nomotion_client = Mock()
        bridge.nomotion_client.call_async.return_value.done.return_value = True
        bridge.amcl_subscription = bridge.cloud_subscription = Mock()
        bridge.amcl_subscription.get_publisher_count.return_value = 1
        bridge.tf_buffer = Mock()
        bridge.stream_recovery_ready = Mock(return_value=False)
        bridge.nomotion_request_count = 2
        bridge.odom_stamp = self.now
        AmclTrialBridge.poll_services(bridge, 10.)
        bridge.nomotion_client.call_async.assert_not_called()
        self.now += .1
        bridge.scan_stamp = bridge.odom_stamp = self.now
        AmclTrialBridge.poll_services(bridge, 10.1)
        AmclTrialBridge.poll_services(bridge, 10.7)
        bridge.nomotion_client.call_async.assert_called_once()
        self.assertEqual(bridge.nomotion_request_count, 3)

    def test_changed_map_cannot_rearm_validation(self):
        self.bridge.map_signature = 'another map'
        response = self.handoff()
        self.assertFalse(response.success)
        self.assertFalse(self.bridge.parking_refresh_armed)

    def test_resume_can_handoff_from_its_own_validated_session(self):
        self.bridge.args.resume_delivery = True
        self.bridge.navigation_monitor = True
        self.bridge.trial_handoff = None
        response = self.handoff()
        self.assertTrue(response.success, response.message)
        self.assertEqual(json.loads(response.message)['run_id'], 'run')

    @patch('r3_delivery_mission.validate_source')
    @patch('r3_delivery_mission.time.monotonic', return_value=10.)
    def test_client_requests_handoff_even_if_old_parked_snapshot_is_invalid(self, clock, validate):
        service = Mock()
        handoff = {'run_id': 'run', 'map_file': 'map'}
        mission = SimpleNamespace(resume_delivery=False, selected_map_file='map', delivery_run_id='run',
            goal={'goal_id': 'D1'}, stationary_status={}, create_client=Mock(return_value=service),
            lifted=Mock(), command_owners=Mock(), now=lambda: 75.,
            check_session=Mock(return_value={'ready': False, 'robot_stationary': True}),
            wheels=SimpleNamespace(ready=Mock(return_value=True), twist=(0., 0.)))

        def response(*args):
            if service.call_async.call_count == 1:
                return SimpleNamespace(success=False, message='parked snapshot invalid')
            mission.stationary_status['delivery_handoff'] = handoff
            return SimpleNamespace(success=True, message=json.dumps(handoff))

        def wait(check, timeout):
            for _ in range(3):
                try:
                    return check()
                except ValueError:
                    clock.return_value += .6
            self.fail('handoff validation did not finish')

        mission.wait_check = wait
        mission.await_future = response
        self.assertEqual(DeliveryMission.switch_to_navigation(mission)['handoff'], handoff)
        self.assertEqual(service.call_async.call_count, 2)
        validate.assert_called_once_with(mission.check_session.return_value, 'map', 75.,
                                         validate_scan=validate_navigation_scan_match)


class ResumeDeliveryTests(unittest.TestCase):
    def test_resume_accepts_only_recorded_pickup_stopped_before_sending_d(self):
        with TemporaryDirectory() as folder:
            source = Path(folder)
            selected_map = source/'map.yaml'
            trial = source/'trial'
            trial.mkdir()
            pickup = trial/'mission_result.json'
            pickup.write_text(json.dumps(dict(status='PICKUP_COMPLETE',
                localization=dict(run_id='pickup', map_file=str(selected_map)),
                lift=dict(height_requested_m=.08, feedback=dict(homed=True, home_completion_count=4,
                                                               sample_ms=2000, published_ros_s=100.)))))
            arrival = source/'delivery_arrival.json'
            block = dict(status='DELIVERY_BLOCKED', phase='navigate', delivery_timing={})
            arrival.write_text(json.dumps(block))
            self.assertEqual(pickup_reference(pickup, resume_map=selected_map)[:3], ('pickup', .08, 4))
            block['delivery_timing']['goal_sent_ros_s'] = 120.
            arrival.write_text(json.dumps(block))
            with self.assertRaisesRegex(ValueError, 'trước khi gửi goal D'):
                pickup_reference(pickup, resume_map=selected_map)

    def test_resume_can_detect_a_restarted_esp32(self):
        prior = dict(sample_ms=5000, published_ros_s=100.)
        validate_lift_continuity(prior, dict(sample_ms=65000, published_ros_s=160.))
        with self.assertRaisesRegex(ValueError, 'ESP32'):
            validate_lift_continuity(prior, dict(sample_ms=8000, published_ros_s=160.))


class LoweringTests(unittest.TestCase):
    def setUp(self):
        self.pose = [0.22, -.32, math.pi]
        lift = Mock()
        wheels = Mock(twist=(0., 0.), ready=Mock(return_value=True), check=Mock(return_value=True))
        self.mission = SimpleNamespace(goal={'goal_id': 'D1'}, pickup_run_id='run', delivery_run_id='run',
            resume_delivery=False,
            selected_map_file='map', target=self.pose, map_signature='ROS-map',
            height=.08, home_generation=7, lift=lift, wheels=wheels,
            stationary_pose=lambda: self.pose, command_owners=Mock(), lifted=Mock(),
            wait_check=lambda check, timeout: check(), spin_once=Mock(), now=lambda: 100.)
        self.arrival = dict(status='ARRIVED_AT_D', goal_id='D1', run_id='run', map_file='map',
            map_signature=digest('ROS-map'), goal_pose_xyyaw=self.pose,
            pose_xyyaw=self.pose, action_status=4)

    def test_lower_requires_successful_arrival_and_parked_base(self):
        with self.assertRaises(RuntimeError):
            DeliveryMission.lower(self.mission, {**self.arrival, 'action_status': 6})
        self.mission.wheels.ready.return_value = False
        with self.assertRaises(ValueError):
            DeliveryMission.lower(self.mission, self.arrival)
        self.mission.lift.begin_lower.assert_not_called()
        self.mission.lift.lower_tick.assert_not_called()

    @patch('r3_delivery_mission.rclpy.ok', return_value=True)
    def test_wheel_roll_stops_lift_without_another_height_command(self, _):
        self.mission.wheels.check.return_value = False
        with self.assertRaises(RuntimeError):
            DeliveryMission.lower(self.mission, self.arrival)
        self.mission.lift.lower_tick.assert_not_called()
        self.assertGreaterEqual(self.mission.lift.stop.call_count, 1)

    @patch('r3_delivery_mission.rclpy.ok', return_value=True)
    def test_completion_requires_feedback_then_stops(self, _):
        self.mission.lift.lower_tick.return_value = True
        result = DeliveryMission.lower(self.mission, self.arrival)
        self.mission.lift.lower_tick.assert_called_once_with(.08, 7)
        self.assertEqual(result['status'], 'DELIVERY_COMPLETE')
        self.assertFalse(result['forks_withdrawn'])
        self.mission.lift.stop.assert_called()

    def test_lower_limit_above_target_is_a_fault_and_cannot_send_another_setpoint(self):
        lift = SimpleNamespace(status=lambda: dict(homed=True, home_completion_count=7,
            state='IDLE', upper_active=False, lower_active=True, position=.03,
            target=DELIVERY_HEIGHT_M, velocity=0.), target_pub=Mock())
        with self.assertRaises(ValueError):
            LiftControl.lower_tick(lift, .08, 7)
        lift.target_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
