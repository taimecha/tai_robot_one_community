"""Recorded terminal faults and closed-loop response checks; no ROS/hardware."""
from collections import deque
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from r3_core import collision, compose, errors, goal_base
from r3_insertion_clearance import insertion_motion_clearance
from r3_longitudinal_control import MeasuredSpeedControl, profiled_speed
from r3_tracking import FixedPlanFollower


FIXTURE = Path(__file__).parent/'fixtures/pickup_terminal_stops_20261117.json'


class LongitudinalControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recording = json.loads(FIXTURE.read_text())
        cls.g = cls.recording['geometry']

    def straight_follower(self):
        target = (0., 0., 0.)
        start = goal_base(target, self.g, 1.3)
        c = goal_base(target, self.g, self.g['fork_length']+.15)
        b = goal_base(target, self.g)
        phases = [dict(type='APPROACH', poses=[start, c], straight_lead_in_m=.25),
                  dict(type='STRAIGHT', poses=[c, b])]
        follower = FixedPlanFollower(phases, target, self.g, continuous_insertion=True)
        follower.start(True)
        return follower, target, start

    def test_recorded_stall_is_outside_valid_B_with_stationary_wheels(self):
        case = self.recording['cases'][0]
        row = case['observations'][-1]
        self.assertEqual(case['original_reason'], 'no odom progress while commanded')
        self.assertAlmostEqual(row['gap_m']-.02, .005330929864609185)
        self.assertLess(abs(row['wheel_speed_mps']), 1e-6)
        self.assertGreater(row['command_mps'], .01)

    def test_feedback_recovers_effort_at_recorded_stall_without_false_arrival(self):
        case = self.recording['cases'][0]
        row = case['observations'][-1]
        follower = FixedPlanFollower(case['phases'], case['target'], self.g, continuous_insertion=True)
        follower.start(True)
        follower.v = follower.reference_v = row['command_mps']
        original = previous = follower.v
        for _ in range(20):
            command, _ = follower.step(row['pose'], .05, insertion_pose=row['pose'],
                insertion_yaw=row['pose'][2], insertion_yaw_rate=row['yaw_rate_rps'], measured_speed=0.)
            self.assertNotIn(follower.state, ('STOP_LATCHED', 'STOPPED'), follower.reason)
            self.assertLessEqual(command, self.g['max_speed'])
            self.assertLessEqual(command-previous, self.g['acceleration']*.05+1e-12)
            previous = command
        self.assertGreater(command, original+.005)

    def test_recorded_unsafe_second_state_still_requires_stop(self):
        case = self.recording['cases'][1]
        row = case['observations'][-1]
        speed = case['original_clearance']['command_speed_mps']
        clearance = insertion_motion_clearance(row['pose'], case['target'], self.g,
                                              speed, row['yaw_rate_rps'], latency_s=.4)
        self.assertFalse(clearance['ready'])
        self.assertIn('body/pillar', clearance['reason'])
        follower = FixedPlanFollower(case['phases'], case['target'], self.g, continuous_insertion=True)
        follower.start(True)
        follower.v = follower.reference_v = case['observations'][-2]['command_mps']
        self.assertEqual(follower.step(row['pose'], .05, measured_speed=speed,
            insertion_pose=row['pose'], insertion_yaw=row['pose'][2],
            insertion_yaw_rate=row['yaw_rate_rps']), (0., 0.))
        self.assertEqual(follower.state, 'STOP_LATCHED')

    def test_observed_overspeed_requests_braking_before_the_recorded_fault(self):
        case = self.recording['cases'][1]
        row = min((r for r in case['observations'] if .125 < r['gap_m'] < .14
                   and r['speed_mps'] > .102), key=lambda r:r['gap_m'])
        remaining = row['gap_m']-self.g['stop_gap']
        a, delay = self.g['acceleration'], .45
        old = min(.1, math.sqrt((a*.4)**2+2*a*remaining)-a*.4)
        corrected, reserve = profiled_speed(remaining, a, delay, row['speed_mps'], .1)
        self.assertEqual(old, .1)
        self.assertLess(corrected, old)
        self.assertGreater(reserve, 0.)
        self.assertTrue(insertion_motion_clearance(row['pose'], case['target'], self.g,
            row['speed_mps'], row['yaw_rate_rps'], latency_s=.4)['ready'])

    def test_stationary_B_uses_existing_five_mm_tolerance(self):
        for offset in (-.004, .004):
            with self.subTest(offset=offset):
                follower, target, _ = self.straight_follower()
                pose = goal_base(target, self.g, self.g['stop_gap']+offset)
                self.assertEqual(follower.step(pose, measured_speed=0.), (0., 0.))
                self.assertEqual(follower.state, 'STOPPED')
                self.assertTrue(follower.terminal_goal_report['stationary_tolerance_used'])
                self.assertEqual(follower.terminal_goal_report['gap_tolerance_m'], .005)

    def test_stationary_rule_cannot_accept_outside_B_or_while_moving(self):
        for offset, speed, omega in ((.0054, 0., 0.), (.004, .006, 0.), (.004, 0., .006)):
            with self.subTest(offset=offset, speed=speed, omega=omega):
                follower, target, _ = self.straight_follower()
                pose = goal_base(target, self.g, self.g['stop_gap']+offset)
                follower.step(pose, measured_speed=speed, insertion_yaw_rate=omega)
                self.assertNotEqual(follower.state, 'STOPPED')

    def test_zero_reference_clears_effort_and_produces_exact_zero(self):
        control = MeasuredSpeedControl()
        control.integral = .03
        self.assertEqual(control.step(0., 0., .02, .05, .08, .1), 0.)
        self.assertEqual(control.integral, 0.)

    def test_speed_feedback_stays_bounded_and_rejects_invalid_input(self):
        control = MeasuredSpeedControl()
        command = 0.
        for _ in range(1000):
            previous = command
            command = control.step(.01, 0., command, .05, .08, .1)
            self.assertGreaterEqual(command, 0.)
            self.assertLessEqual(command, .1)
            self.assertLessEqual(abs(command-previous), .004+1e-12)
            self.assertLessEqual(abs(control.integral), control.integral_limit_mps)
        for invalid in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                control.step(.01, invalid, .02, .05, .08, .1)
            with self.assertRaises(ValueError):
                profiled_speed(.1, .08, .4, invalid, .1)

    def test_real_zero_hold_clears_reference_and_integrator(self):
        follower, _, _ = self.straight_follower()
        follower.v = .06
        follower.reference_v = .04
        follower.speed_control.integral = .03
        follower.reset_motion()
        self.assertEqual((follower.v, follower.w, follower.reference_v), (0., 0., 0.))
        self.assertEqual(follower.speed_control.integral, 0.)

    def test_no_feedback_prediction_keeps_the_original_profile(self):
        expected = math.sqrt((.08*.4)**2+2*.08*.1)-.08*.4
        limit, reserve = profiled_speed(.1, .08, .4, None, .1)
        self.assertAlmostEqual(limit, expected)
        self.assertEqual(reserve, 0.)

    def test_feedback_with_delayed_or_low_speed_drive_reaches_valid_B(self):
        # Response models exercise a failure mode, not physical accuracy or a
        # claim that these unmeasured plant parameters calibrate the robot.
        cases = [(1., 0., .2), (1.1, 0., .4), (1.4, .025, .2), (1.5, .03, .4)]
        for gain, deadband, delay in cases:
            with self.subTest(gain=gain, deadband=deadband, delay=delay):
                follower, target, pose = self.straight_follower()
                speed = delayed = 0.
                commands = deque()
                dt = .05
                progress_pose, progress_at = pose, 0.
                for i in range(700):
                    t = i*dt
                    command, _ = follower.step(pose, dt, measured_speed=speed,
                        insertion_pose=pose, insertion_yaw=pose[2])
                    self.assertNotEqual(follower.state, 'STOP_LATCHED', follower.reason)
                    if math.dist(pose[:2], progress_pose[:2]) > .005:
                        progress_pose, progress_at = pose, t
                    self.assertFalse(command > .01 and t-progress_at > 3., 'drive made no progress')
                    commands.append((t+delay, command))
                    while commands and commands[0][0] <= t+1e-9:
                        _, delayed = commands.popleft()
                    requested = gain*max(0., delayed-deadband)
                    actual = max(speed-.08*dt, min(speed+.08*dt, requested))
                    pose = compose(pose, ((speed+actual)*dt/2, 0., 0.))
                    speed = actual
                    self.assertIsNone(collision(pose, target, self.g))
                    if follower.state == 'STOPPED' and speed < 1e-6 and not any(u > 0. for _,u in commands):
                        break
                self.assertEqual(follower.state, 'STOPPED')
                self.assertLessEqual(abs(errors(pose, target, self.g)['gap_m']-.02), .005)
                self.assertTrue(insertion_motion_clearance(pose, target, self.g, 0., 0., latency_s=.4)['ready'])


if __name__ == '__main__':
    unittest.main()
