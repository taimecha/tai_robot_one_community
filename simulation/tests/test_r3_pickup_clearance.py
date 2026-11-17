"""Recorded pickup regression and obstacle checks; no ROS or actuator access."""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from r3_core import base_for_fork, collision, compose, errors, overlap, rectangle
from r3_insertion_clearance import (braking_trajectory, insertion_clearance,
                                    insertion_motion_clearance)
from r3_tracking import FixedPlanFollower


FIXTURE = Path(__file__).parent / 'fixtures' / 'pickup_20261117_stop.json'


def advance(pose, speed, omega, dt):
    angle = omega*dt
    travel = ((speed*dt, 0., angle) if abs(omega) < 1e-9 else
              (speed/omega*math.sin(angle), speed/omega*(1-math.cos(angle)), angle))
    return compose(pose, travel)


class PickupClearanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recording = json.loads(FIXTURE.read_text())
        cls.g = cls.recording['geometry']
        cls.target = cls.recording['target']

    def follower(self, **kwargs):
        follower = FixedPlanFollower(self.recording['phases'], self.target,
                                     self.g, **kwargs)
        follower.start(True)
        return follower

    def replay(self):
        follower = self.follower(continuous_insertion=True)
        for i, row in enumerate(self.recording['observations']):
            raw = row['camera_frame_pose']
            command = follower.step(row['control_pose'], row['dt_s'],
                insertion_pose=raw, insertion_yaw=raw[2],
                insertion_yaw_rate=row['yaw_rate_rps'],
                measured_speed=row['measured_speed_mps'])
            self.assertNotEqual(follower.state, 'STOP_LATCHED',
                f'observation {i}: {follower.reason}')
        return follower, command

    def test_recording_exposes_constant_heading_false_stop(self):
        raw = self.recording['observations'][-1]['camera_frame_pose']
        projection = insertion_clearance(raw, self.target, self.g)
        current = insertion_clearance(raw, self.target, self.g, project_to_b=False)
        self.assertFalse(projection['ready'])
        self.assertAlmostEqual(projection['minimum_clearance_m'], .003683156137687646)
        self.assertTrue(current['ready'])
        self.assertAlmostEqual(current['minimum_clearance_m'], .025284857149443923)
        self.assertIsNone(collision(raw, self.target, self.g))
        self.assertGreater(current['gap_m']-self.g['fork_length'], .39)

    def test_all_122_observations_allow_first_heading_correction(self):
        follower, (speed, omega) = self.replay()
        self.assertEqual(len(self.recording['observations']), 122)
        self.assertTrue(follower.insertion_heading_active)
        self.assertFalse(follower.insertion_started)
        self.assertIsNotNone(follower.insertion_heading_pid.report)
        self.assertGreater(speed, 0.)
        self.assertLess(omega, 0.)
        self.assertTrue(follower.insertion_camera_frame_report['ready'])

    def test_synthetic_continuation_can_finish_with_heading_feedback(self):
        # This checks controller feasibility, not real slip or pickup success.
        follower, (_, omega) = self.replay()
        pose = self.recording['observations'][-1]['camera_frame_pose']
        for _ in range(1000):
            speed, omega = follower.step(pose, .05, insertion_pose=pose,
                insertion_yaw=pose[2], insertion_yaw_rate=omega)
            self.assertNotEqual(follower.state, 'STOP_LATCHED', follower.reason)
            self.assertIsNone(collision(pose, self.target, self.g))
            if follower.state == 'STOPPED':
                break
            pose = advance(pose, speed, omega, .05)
        self.assertEqual(follower.state, 'STOPPED')
        self.assertTrue(follower.insertion_started)
        e = errors(pose, self.target, self.g)
        self.assertLess(abs(e['gap_m']-self.g['stop_gap']), .005)
        self.assertLess(abs(math.degrees(e['yaw_error_rad'])), .2)
        self.assertTrue(insertion_clearance(pose, self.target, self.g,
                                           project_to_b=False)['ready'])

    def test_outside_pallet_can_correct_without_infinite_corridor_gate(self):
        # The lateral corridor is not an obstacle 40 cm before the front legs.
        base = base_for_fork((-.8, .08, math.radians(2)), self.g)
        self.assertFalse(insertion_clearance(base, (0., 0., 0.), self.g,
                                            project_to_b=False)['ready'])
        report = insertion_motion_clearance(base, (0., 0., 0.), self.g,
                                             .1, -.04, latency_s=.2)
        self.assertTrue(report['ready'], report['reason'])

    def test_clear_now_but_obstacle_in_braking_sweep_stops(self):
        base = base_for_fork((-.45, .03, math.radians(2)), self.g)
        target = (0., 0., 0.)
        self.assertIsNone(collision(base, target, self.g))
        self.assertIsNone(collision(advance(base, .1, 0., .2), target, self.g))
        stopped = insertion_motion_clearance(base, target, self.g, 0., 0., latency_s=.2)
        moving = insertion_motion_clearance(base, target, self.g, .1, 0., latency_s=.2)
        self.assertTrue(stopped['ready'])
        self.assertFalse(moving['ready'])
        self.assertIn('tine/pillar', moving['reason'])
        self.assertAlmostEqual(moving['stopping_distance_m'], .0825)

    def test_ten_mm_leg_margin_is_retained_on_both_sides(self):
        for side in (-1., 1.):
            with self.subTest(side=side):
                # Actual clearance is 5 mm: no contact, but insufficient margin.
                base = base_for_fork((-.2, side*.045, 0.), self.g)
                self.assertIsNone(collision(base, (0., 0., 0.), self.g))
                report = insertion_motion_clearance(base, (0., 0., 0.),
                    self.g, 0., 0., latency_s=.4)
                self.assertFalse(report['ready'])
                self.assertEqual(report['required_clearance_m'], .01)

    def test_full_tines_reject_collision_even_when_root_is_centered(self):
        base = base_for_fork((-.05, 0., math.radians(10)), self.g)
        self.assertEqual(collision(base, (0., 0., 0.), self.g), 'tine/pillar')
        report = insertion_motion_clearance(base, (0., 0., 0.), self.g,
                                             0., 0., latency_s=.4)
        self.assertFalse(report['ready'])

    def test_braking_sweep_keeps_body_deck_and_height_checks(self):
        target = (0., 0., 0.)
        for gap, updates, expected in (
                (-.05, {}, 'body/pillar'),
                (-.05, {'body_bounds': [-.303, .3305, -.1, .1]}, 'body/deck'),
                (.2, {'fork_bottom_z': 0.}, 'tine/floor'),
                (.2, {'fork_bottom_z': .10}, 'tine/deck')):
            with self.subTest(expected=expected):
                geometry = dict(self.g, **updates)
                base = base_for_fork((-gap, 0., 0.), geometry)
                report = insertion_motion_clearance(base, target, geometry,
                                                     0., 0., latency_s=.4)
                self.assertFalse(report['ready'])
                self.assertIn(expected, report['reason'])

    def test_obstacle_stop_remains_latched(self):
        follower = self.follower(continuous_insertion=True)
        start = self.recording['phases'][0]['poses'][0]
        self.assertEqual(follower.step(start, obstacle=True), (0., 0.))
        self.assertEqual(follower.reason, 'obstacle stop')
        self.assertEqual(follower.step(start, obstacle=False), (0., 0.))
        self.assertEqual(follower.state, 'STOP_LATCHED')

    def test_invalid_odometry_still_prevents_motion(self):
        follower = self.follower(continuous_insertion=True)
        start = self.recording['phases'][0]['poses'][0]
        self.assertEqual(follower.step(start, odom_valid=False), (0., 0.))
        self.assertEqual(follower.reason, 'invalid odom')

    def test_measured_speed_is_included_in_braking_budget(self):
        follower = self.follower(continuous_insertion=True)
        start = self.recording['phases'][0]['poses'][0]
        speed, _ = follower.step(start, measured_speed=.14)
        self.assertNotEqual(follower.state, 'STOP_LATCHED', follower.reason)
        self.assertLessEqual(speed, self.g['max_speed'])
        report = follower.insertion_report
        self.assertEqual(report['command_speed_mps'], .14)
        self.assertAlmostEqual(report['stopping_distance_m'], .14*.2+.14**2/(2*.08))

    def test_approach_only_still_arrives_outside_pallet(self):
        follower = self.follower(approach_only=True)
        pose = self.recording['phases'][0]['poses'][0]
        for _ in range(1000):
            speed, omega = follower.step(pose)
            self.assertNotEqual(follower.state, 'STOP_LATCHED', follower.reason)
            if follower.state == 'C_REACHED':
                break
            pose = advance(pose, speed, omega, .05)
        self.assertEqual(follower.state, 'C_REACHED')
        self.assertGreater(errors(pose, self.target, self.g)['gap_m']
                           -self.g['fork_length'], .10)

    def test_nonfinite_motion_is_rejected(self):
        for speed, omega in ((float('nan'), 0.), (.1, float('inf'))):
            with self.subTest(speed=speed, omega=omega):
                with self.assertRaises(ValueError):
                    insertion_motion_clearance((0., 0., 0.), self.target,
                        self.g, speed, omega, latency_s=.2)


class IndependentBrakingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = FIXTURE.parent / 'pickup_20261117_123354_stop.json'
        cls.recording = json.loads(path.read_text())
        cls.g = cls.recording['geometry']
        cls.target = cls.recording['target']

    def test_recorded_gyro_uses_its_own_braking_time(self):
        row = self.recording['observations'][-1]
        original = self.recording['original_stop_clearance']
        report = insertion_motion_clearance(row['control_pose'], self.target,
            self.g, row['measured_speed_mps'], row['yaw_rate_rps'], latency_s=.4)
        self.assertTrue(report['ready'], report['reason'])
        self.assertAlmostEqual(row['yaw_rate_rps'], .049087)
        self.assertAlmostEqual(report['linear_braking_time_s'], 1.3360782444720765)
        self.assertAlmostEqual(report['angular_braking_time_s'], .1227175)
        self.assertAlmostEqual(math.degrees(original['stopping_yaw_rad']),
                               3.0038364586673136)
        self.assertAlmostEqual(math.degrees(report['stopping_yaw_rad']),
                               1.2975613017069618)
        # The linear stopping reserve and 10 mm leg margin are unchanged.
        self.assertAlmostEqual(report['stopping_distance_m'], original['stopping_distance_m'])
        self.assertEqual(report['required_clearance_m'], .01)

    def test_all_167_observations_continue_past_second_false_stop(self):
        recording = self.recording
        follower = FixedPlanFollower(recording['phases'], self.target, self.g,
                                      continuous_insertion=True)
        follower.start(True)
        for i, row in enumerate(recording['observations']):
            raw = row['camera_frame_pose']
            speed, omega = follower.step(row['control_pose'], row['dt_s'],
                insertion_pose=raw, insertion_yaw=raw[2],
                insertion_yaw_rate=row['yaw_rate_rps'],
                measured_speed=row['measured_speed_mps'])
            self.assertNotEqual(follower.state, 'STOP_LATCHED',
                                f'observation {i}: {follower.reason}')
        self.assertEqual(len(recording['observations']), 167)
        self.assertEqual(follower.state, 'STRAIGHT')
        self.assertGreater(speed, 0.)
        self.assertTrue(follower.insertion_report['ready'])
        self.assertTrue(follower.insertion_camera_frame_report['ready'])
        self.assertEqual(follower.insertion_report['velocity_source'], 'measured_motion')

    def test_strong_measured_turn_still_stops_before_pillar(self):
        row = self.recording['observations'][-1]
        report = insertion_motion_clearance(row['control_pose'], self.target,
            self.g, row['measured_speed_mps'], .2, latency_s=.4)
        self.assertFalse(report['ready'])
        self.assertIn('tine/pillar', report['reason'])
        self.assertIsNotNone(report['first_intersection'])

    def test_turn_stops_while_linear_braking_continues(self):
        samples = list(braking_trajectory((0., 0., 0.), .10688625955776612,
                                          .049087, .08, .4, .4))
        angular_stop_time = .4+.049087/.4
        expected_yaw = .049087*.4+.049087**2/(2*.4)
        after_turn = [(pose, t) for pose, t in samples if t >= angular_stop_time-1e-10]
        self.assertGreater(len(after_turn), 5)
        for pose, _ in after_turn:
            self.assertAlmostEqual(pose[2], expected_yaw)
        self.assertGreater(after_turn[-1][0][0], after_turn[0][0][0]+.04)

    def test_linear_stop_does_not_shorten_remaining_rotation(self):
        samples = list(braking_trajectory((0., 0., 0.), .01, .2, .08, .4, .4))
        after_translation = [pose for pose, t in samples if t >= .4+.01/.08-1e-10]
        self.assertGreater(len(after_translation), 5)
        for pose in after_translation:
            self.assertAlmostEqual(pose[0], after_translation[0][0])
            self.assertAlmostEqual(pose[1], after_translation[0][1])
        self.assertGreater(after_translation[-1][2], after_translation[0][2])
        self.assertAlmostEqual(after_translation[-1][2], .2*.4+.2**2/(2*.4))

    def test_straight_stop_and_opposite_turns_have_expected_endpoints(self):
        straight = list(braking_trajectory((0., 0., 0.), .1, 0., .08, .4, .2))[-1]
        self.assertAlmostEqual(straight[0][0], .0825)
        self.assertAlmostEqual(straight[0][1], 0.)
        self.assertAlmostEqual(straight[0][2], 0.)
        self.assertAlmostEqual(straight[1], 1.45)
        positive = list(braking_trajectory((0., 0., 0.), .1, .06, .08, .4, .4))[-1][0]
        negative = list(braking_trajectory((0., 0., 0.), .1, -.06, .08, .4, .4))[-1][0]
        self.assertAlmostEqual(positive[0], negative[0])
        self.assertAlmostEqual(positive[1], -negative[1])
        self.assertAlmostEqual(positive[2], -negative[2])

    def test_rectangle_bounds_keep_contact_and_rotated_sat(self):
        square = rectangle((0., 0., 0.), (-1., 1., -1., 1.))
        touching = rectangle((2., 0., 0.), (-1., 1., -1., 1.))
        self.assertTrue(overlap(square, touching))
        angle = math.pi/4
        a = rectangle((0., 0., angle), (-1., 1., -.05, .05))
        b = rectangle((-.2*math.sin(angle), .2*math.cos(angle), angle),
                      (-1., 1., -.05, .05))
        # Their axis bounds overlap, so only the oriented test can reject them.
        for axis in (0, 1):
            self.assertLess(min(p[axis] for p in a), max(p[axis] for p in b))
            self.assertLess(min(p[axis] for p in b), max(p[axis] for p in a))
        self.assertFalse(overlap(a, b))


if __name__ == '__main__':
    unittest.main()
