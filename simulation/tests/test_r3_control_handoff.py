"""Control ownership epochs and an isolated DDS handoff; no robot interfaces."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from r3_control_odom import ControlOdomGuard, ControlOdomNode
    from r3_enable_trial import TrialEnableClient, enable_trial
    from r3_recognition_goal import RecognitionGoal
    from r3_stationary_localization import StationaryEvidence
except ImportError as exc:
    raise unittest.SkipTest('ROS message/runtime dependencies unavailable: '+str(exc))


FIXTURE = json.loads((Path(__file__).parent/'fixtures/handoff_20261117_184021.json').read_text())
SOURCE = FIXTURE['source_localization_ready']
EPOCH = SOURCE['published_ros_s']
MAP = SOURCE['map_file']


def state_at(now, *, retime=False):
    """A synthetic valid trial certificate using the recorded posterior.

    Only test fixtures are retimed for an isolated live DDS test. Production
    capture stamps, posterior pairing and deadlines are never altered.
    """
    state = copy.deepcopy(SOURCE)
    if retime:
        delta = now-EPOCH
        snapshot = state['stationary_snapshot']
        for key in ('amcl_stamp_s', 'cloud_stamp_s'):
            snapshot[key] += delta
        for key in ('pose_stamp_s', 'cloud_stamp_s', 'validated_ros_s'):
            snapshot['particle_pairing'][key] += delta
        state['amcl_stamp_s'] = snapshot['amcl_stamp_s']
    state.update(published_ros_s=now, ready=True, role='trial', navigation_monitor=False,
                 live_observation_validated=False, robot_stationary=True)
    state['stationary_cache']['validated'] = True
    state['scan_map_fit']['scan_stamp_s'] = now-.02
    state['current_map_odom_xyyaw'] = list(state['baseline_map_odom'])
    state['trial_handoff'] = dict(enabled=True, at_ros_s=now-.01,
        source_amcl_stamp_s=state['stationary_snapshot']['amcl_stamp_s'],
        baseline_map_odom=list(state['baseline_map_odom']),
        nomotion_request_count=state['stationary_cache']['nomotion_request_count'])
    return state


def odom_at(stamp, *, x_offset=0.):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    x,y,yaw = SOURCE['stationary_snapshot']['odom_anchor_xyyaw']
    message.pose.pose.position.x = x+x_offset
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw/2)
    message.pose.pose.orientation.w = math.cos(yaw/2)
    return message


class StationaryReaderGapTests(unittest.TestCase):
    def setUp(self):
        self.pose = SOURCE['stationary_snapshot']['odom_anchor_xyyaw']
        self.fit = SOURCE['scan_map_fit']
        self.now = 100.
        self.reader = SimpleNamespace(
            continuous_localization=True, stationary_startup_pending=False,
            stationary_mode=True, stationary_snapshot=SOURCE['stationary_snapshot'],
            stationary_status=dict(run_id='run', ready=True, stationary_cache={'validated': True}),
            stationary_evidence=StationaryEvidence(), stationary_validated_poses=[],
            last_validated_scan=object(), stationary_fit_key=object(),
            scan_stamp=None, stamp=RecognitionGoal.stamp, now=lambda: self.now,
            stationary_since=time.monotonic()-2., particle_gate=SimpleNamespace(),
            stationary_amcl=object(), _score_stationary_scan=Mock(return_value=self.pose))
        for method in (RecognitionGoal._reset_a_scan_quality, RecognitionGoal._restart_local_scan_window):
            setattr(self.reader, method.__name__, MethodType(method, self.reader))
        self.reader.stationary_evidence.capture(97., self.pose)
        for stamp in (98.8, 99.2, 99.6, 100.):
            self.feed(stamp)

    def feed(self, stamp, *, odom=True, scan=True, offset=0.):
        self.now = stamp
        if odom:
            RecognitionGoal.on_odom(self.reader, odom_at(stamp, x_offset=offset))
        if scan:
            message = LaserScan()
            message.header.frame_id = 'lidar_link'
            message.header.stamp = odom_at(stamp).header.stamp
            RecognitionGoal.on_scan(self.reader, message)
            self.reader.stationary_evidence.observe_scan(stamp, self.fit, stamp,
                                                        check_capture_continuity=False)

    def current_pose(self):
        return RecognitionGoal._current_stationary_pose(self.reader, self.now, True)

    def test_forward_reader_gaps_require_a_new_window_on_same_observation(self):
        self.assertEqual(self.current_pose(), self.pose)
        self.feed(100.7)
        self.assertIsNone(self.reader.stationary_evidence.fault)
        self.assertIsNone(self.reader.last_validated_scan)
        with self.assertRaisesRegex(ValueError, 'distinct matching scans'):
            self.current_pose()
        for stamp in (101., 101.3, 101.6, 101.8):
            self.feed(stamp)
        self.reader.stationary_status['stationary_cache']['validated'] = False
        with self.assertRaisesRegex(ValueError, 'bridge is not validating'):
            self.current_pose()
        self.reader.stationary_status['stationary_cache']['validated'] = True
        self.assertEqual(self.current_pose(), self.pose)
        self.assertEqual(self.reader.stationary_evidence.observation_stamp, 97.)

    def test_reader_gap_cannot_clear_displacement_or_reversed_lidar_clock(self):
        self.feed(100.7, offset=.01)
        self.assertIn('robot displacement', self.reader.stationary_evidence.fault)
        self.feed(100.6, odom=False)
        for stamp in (101., 101.3, 101.6, 101.9):
            self.feed(stamp)
        with self.assertRaisesRegex(ValueError, 'LiDAR timestamp moved backwards'):
            self.current_pose()

    def test_supervisor_still_invalidates_source_odom_gaps(self):
        source = StationaryEvidence()
        source.capture(97., self.pose)
        source.update_odom(self.pose, (0., 0., 0.), 100., 100.)
        source.update_odom(self.pose, (0., 0., 0.), 100.7, 100.7)
        self.assertIn('odom gap', source.fault)


class ControlEpochTests(unittest.TestCase):
    def test_navigation_gap_cannot_latch_an_unowned_control_frame(self):
        guard = ControlOdomGuard(MAP)
        for now in (EPOCH-2., EPOCH-1.3, EPOCH-.1):
            state = state_at(now)
            state.update(role='navigation_monitor', navigation_monitor=True)
            with self.assertRaises(ValueError):
                guard.transform(state, odom_at(now), now)
            self.assertIsNone(guard.fault)
            self.assertIsNone(guard.reference_state)
        output = guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH)
        self.assertEqual(output.header.frame_id, 'r3_trial')
        self.assertEqual(output.header.stamp, odom_at(EPOCH).header.stamp)

    def test_initial_trial_validation_wait_does_not_start_continuity(self):
        guard = ControlOdomGuard(MAP)
        for now in (EPOCH-1., EPOCH-.3):
            state = state_at(now)
            state['ready'] = False
            with self.assertRaises(ValueError):
                guard.transform(state, odom_at(now), now)
            self.assertIsNone(guard.fault)
        self.assertEqual(guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH).header.frame_id, 'r3_trial')

    def test_capture_gap_after_first_control_output_remains_latched(self):
        guard = ControlOdomGuard(MAP)
        guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH)
        with self.assertRaisesRegex(ValueError, 'EKF capture gap'):
            guard.transform(state_at(EPOCH+.6), odom_at(EPOCH+.6), EPOCH+.6)
        with self.assertRaisesRegex(ValueError, 'EKF capture gap'):
            guard.transform(state_at(EPOCH+.62), odom_at(EPOCH+.62), EPOCH+.62)

    def test_capture_reversal_after_activation_remains_latched(self):
        guard = ControlOdomGuard(MAP)
        guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH)
        with self.assertRaisesRegex(ValueError, 'capture clock reversed'):
            guard.transform(state_at(EPOCH+.02), odom_at(EPOCH-.02), EPOCH+.02)

    def test_pose_jump_after_activation_remains_latched(self):
        guard = ControlOdomGuard(MAP)
        guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH)
        with self.assertRaisesRegex(ValueError, 'pose jumped'):
            guard.transform(state_at(EPOCH+.02), odom_at(EPOCH+.02, x_offset=.11), EPOCH+.02)

    def test_reference_change_after_activation_remains_latched(self):
        guard = ControlOdomGuard(MAP)
        guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH)
        state = state_at(EPOCH+.02)
        state['baseline_map_odom'][0] += .01
        with self.assertRaisesRegex(ValueError, 'map/reference changed'):
            guard.transform(state, odom_at(EPOCH+.02), EPOCH+.02)

    def test_role_reversion_after_activation_remains_latched(self):
        guard = ControlOdomGuard(MAP)
        guard.transform(state_at(EPOCH), odom_at(EPOCH), EPOCH)
        state = state_at(EPOCH+.02)
        state.update(role='navigation_monitor', navigation_monitor=True)
        with self.assertRaisesRegex(ValueError, 'publisher ownership changed'):
            guard.transform(state, odom_at(EPOCH+.02), EPOCH+.02)

    def test_stale_odom_or_supervisor_cannot_create_initial_frame(self):
        for stale_source in ('odom', 'supervisor'):
            with self.subTest(stale_source=stale_source):
                guard = ControlOdomGuard(MAP)
                state = state_at(EPOCH)
                message = odom_at(EPOCH-.21 if stale_source=='odom' else EPOCH)
                if stale_source=='supervisor':state['published_ros_s'] = EPOCH-.51
                with self.assertRaises(ValueError):
                    guard.transform(state, message, EPOCH)
                self.assertIsNone(guard.reference_state)

    def test_delayed_supervisor_keeps_established_frame_on_fresh_ekf(self):
        guard = ControlOdomGuard(MAP)
        state = state_at(EPOCH)
        first = guard.transform(state, odom_at(EPOCH), EPOCH)
        for step in range(1, 31):
            now = EPOCH+step*.02
            output = guard.transform(state, odom_at(now), now)
            self.assertIsNone(guard.fault)
            self.assertEqual(output.header.frame_id, first.header.frame_id)
        self.assertEqual(guard.mode, 'ekf_tracking')


class ControlHandoffRosTests(unittest.TestCase):
    def test_live_dds_handoff_after_pre_trial_gap(self):
        self.run_live_handoff(start_late=False)

    def test_late_control_node_receives_existing_trial_certificate(self):
        self.run_live_handoff(start_late=True)

    def run_live_handoff(self, *, start_late):
        # Domain 176 contains only synthetic odometry, a fake localization
        # supervisor, and the actual control node/client. No motor publishers.
        rclpy.init(domain_id=176)
        executor = MultiThreadedExecutor(num_threads=2)
        supervisor = Node('isolated_handoff_supervisor')
        control = None
        client = None
        spin = None
        with tempfile.TemporaryDirectory(prefix='r3-handoff-test-') as folder:
            try:
                class ObservedControlNode(ControlOdomNode):
                    def __init__(self):
                        self.observed_input_stamps=[]
                        super().__init__(MAP, diagnostics_path=Path(folder)/'health.jsonl')
                    def on_odom(self,message):
                        self.observed_input_stamps.append(message.header.stamp.sec
                            +message.header.stamp.nanosec*1e-9)
                        super().on_odom(message)
                if not start_late:control = ObservedControlNode()
                start = supervisor.get_clock().now().nanoseconds/1e9
                state = state_at(start, retime=True)
                state.update(role='navigation_monitor', navigation_monitor=True, trial_handoff=None)
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                status_pub = supervisor.create_publisher(String, '/pallet/r3/localization/status', qos)
                odom_pub = supervisor.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
                def now():return supervisor.get_clock().now().nanoseconds/1e9
                def publish_status():
                    current = now()
                    state['published_ros_s'] = current
                    state['scan_map_fit']['scan_stamp_s'] = current-.02
                    status_pub.publish(String(data=json.dumps(state)))
                def publish_odom():
                    current = now()
                    if not .4 <= current-start <= 1.1:
                        odom_pub.publish(odom_at(current))
                def change_role(request,response):
                    current = now()
                    if state['trial_handoff'] is None:
                        state.update(role='trial', navigation_monitor=False,
                            trial_handoff=dict(enabled=True,at_ros_s=current,
                            source_amcl_stamp_s=state['stationary_snapshot']['amcl_stamp_s'],
                            baseline_map_odom=list(state['baseline_map_odom']),
                            nomotion_request_count=state['stationary_cache']['nomotion_request_count']))
                    response.success = True
                    response.message = 'synthetic trial handoff accepted'
                    publish_status()
                    return response
                supervisor.create_timer(.02, publish_odom)
                supervisor.create_timer(.05, publish_status)
                supervisor.create_service(Trigger, '/pallet/r3/localization/enable_trial', change_role)
                executor.add_node(supervisor)
                if control is not None:executor.add_node(control)
                spin = threading.Thread(target=executor.spin)
                spin.start()
                time.sleep(1.5)
                if start_late:
                    # A real Python worker can finish initializing after the
                    # coordinator's service ACK. Join an already enabled role.
                    change_role(None, Trigger.Response())
                    time.sleep(.1)
                    control=ObservedControlNode()
                    executor.add_node(control)
                    scenario='late control node receives the existing trial certificate'
                else:
                    self.assertIsNone(control.guard.fault)
                    self.assertEqual(control.count, 0)
                    gaps=[b-a for a,b in zip(control.observed_input_stamps,control.observed_input_stamps[1:])]
                    self.assertTrue(gaps)
                    self.assertGreater(max(gaps), .5)
                    scenario=f'measured pre-trial gap {max(gaps):.3f}s'
                client = TrialEnableClient()
                result = enable_trial(client, MAP, 6.)
                self.assertEqual(result['status'], 'TRIAL_LOCALIZATION_READY')
                self.assertGreaterEqual(result['confirmed_odom_samples'], 3)
                self.assertGreaterEqual(result['confirmed_odom_span_s'], .08)
                self.assertEqual(result['role_change_service_calls'], 1)
                self.assertEqual(result['successful_role_changes'], 0 if start_late else 1)
                self.assertEqual(result['nomotion_request_count_before'], result['nomotion_request_count_after'])
                self.assertTrue(result['original_covariance_preserved'])
                self.assertIsNone(control.guard.fault)
                self.assertEqual(result['actuator_commands'], 0)
                print(f'Isolated DDS: {scenario}; one handoff request; '
                      f'{result["confirmed_odom_samples"]} fresh control captures over '
                      f'{result["confirmed_odom_span_s"]:.3f}s; original capture stamps retained.')
            finally:
                executor.shutdown(timeout_sec=3.)
                if spin is not None:spin.join(timeout=2.)
                if client is not None:client.destroy_node()
                if control is not None:control.destroy_node()
                supervisor.destroy_node()
                if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
