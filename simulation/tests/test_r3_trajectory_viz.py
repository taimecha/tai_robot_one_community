"""Unit tests for the read-only full-mission RViz trajectory accumulator and cycle reset."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from r3_trajectory_viz import (TrajectoryAccumulator, compose,
                               load_pickup_planned_map_path)


class TrajectoryVizTests(unittest.TestCase):
    def test_actual_trajectory_filters_tiny_jitter_and_records_motion(self):
        acc = TrajectoryAccumulator(min_step_m=0.02, min_yaw_rad=0.05)
        self.assertTrue(acc.add_actual((1.0, 2.0, 0.0)))
        self.assertFalse(acc.add_actual((1.005, 2.005, 0.01)))
        self.assertTrue(acc.add_actual((1.05, 2.0, 0.0)))
        self.assertTrue(acc.add_actual((1.05, 2.0, 0.1)))
        self.assertEqual(len(acc.actual), 3)

    def test_three_leg_planned_concatenation_preserves_full_route_across_replans(self):
        acc = TrajectoryAccumulator(leg_lock_move_m=0.15)
        self.assertTrue(acc.update_nav_plan([(3.2, 1.7, -1.57), (3.05, 1.46, -1.57)], delivery_phase=False))
        self.assertFalse(acc.update_nav_plan([(3.1, 1.50, -1.57), (3.05, 1.46, -1.57)], delivery_phase=False))
        self.assertTrue(acc.set_pickup_plan('plan-1', [(3.05, 1.46, -1.57), (3.02, 0.06, -1.53)]))
        self.assertFalse(acc.set_pickup_plan('plan-1', [(3.05, 1.46, -1.57), (3.02, 0.06, -1.53)]))
        self.assertTrue(acc.update_nav_plan([(3.02, 0.06, -1.53), (0.22, -0.33, 3.14)], delivery_phase=True))
        combined = acc.combined_planned()
        self.assertEqual(len(combined), 4)
        self.assertEqual(combined[0][:2], (3.2, 1.7))
        self.assertEqual(combined[-1][:2], (0.22, -0.33))

    def test_load_pickup_planned_map_path_transforms_csv_using_baseline(self):
        with tempfile.TemporaryDirectory(prefix='r3-viz-test-') as tmp:
            run_dir = Path(tmp)
            (run_dir / 'plan').mkdir()
            baseline = [3.2, 1.6, -math.pi / 2]
            (run_dir / 'localization_at_A.json').write_text(json.dumps({
                'trial_handoff': {'baseline_map_odom': baseline}
            }))
            (run_dir / 'plan' / 'frozen_plan.json').write_text(json.dumps({'plan_id': 'abc'}))
            (run_dir / 'plan' / 'path_planned_base.csv').write_text(
                'x_m,y_m,yaw_rad\n0.0,0.0,0.0\n1.0,0.0,0.0\n'
            )
            plan_id, pts = load_pickup_planned_map_path(run_dir)
            self.assertEqual(plan_id, 'abc')
            self.assertEqual(len(pts), 2)
            self.assertAlmostEqual(pts[0][0], 3.2, places=6)
            self.assertAlmostEqual(pts[0][1], 1.6, places=6)
            self.assertAlmostEqual(pts[1][0], 3.2, places=6)
            self.assertAlmostEqual(pts[1][1], 0.6, places=6)
            expected = compose(baseline, (1.0, 0.0, 0.0))
            self.assertAlmostEqual(pts[1][2], expected[2], places=6)

    def test_write_csvs_creates_actual_and_planned_files(self):
        with tempfile.TemporaryDirectory(prefix='r3-viz-csv-') as tmp:
            cycle_dir = Path(tmp)
            acc = TrajectoryAccumulator()
            acc.add_actual((0.2, -0.3, 3.14), wall_s=100.0, phase='to_A')
            acc.add_actual((3.0, 1.4, -1.57), wall_s=120.0, phase='pickup')
            acc.update_nav_plan([(0.2, -0.3, 3.14), (3.0, 1.4, -1.57)], delivery_phase=False)
            acc.set_pickup_plan('p1', [(3.0, 1.4, -1.57), (3.0, 0.1, -1.57)])
            acc.update_nav_plan([(3.0, 0.1, -1.57), (0.2, -0.3, 3.14)], delivery_phase=True)
            acc.write_csvs(cycle_dir)
            actual_lines = (cycle_dir / 'full_trajectory_actual.csv').read_text().splitlines()
            planned_lines = (cycle_dir / 'full_trajectory_planned.csv').read_text().splitlines()
            self.assertEqual(actual_lines[0], 'wall_s,x_m,y_m,yaw_rad,phase')
            self.assertEqual(len(actual_lines), 3)
            self.assertEqual(planned_lines[0], 'leg,x_m,y_m,yaw_rad')
            self.assertEqual(len(planned_lines), 7)



    def test_amcl_bridge_reset_cycle_clears_handoffs_and_checks_publishers(self):
        from types import SimpleNamespace
        from r3_amcl_bridge import AmclTrialBridge, TrialCorrection
        from r3_stationary_localization import StationaryEvidence

        class FakeClock:
            def now(self):
                return SimpleNamespace(nanoseconds=100_000_000_000)

        class FakeLogger:
            def info(self, msg): pass

        bridge = AmclTrialBridge.__new__(AmclTrialBridge)
        bridge.continuous_localization = True
        bridge.run_id = 'run-1'
        bridge.map_file = '/tmp/map.yaml'
        bridge.trial_handoff = {'enabled': True}
        bridge.delivery_handoff = {'run_id': 'run-1'}
        bridge.delivery_refresh_after = 99.0
        bridge.navigation_monitor = False
        bridge.motion_prediction = {'x': 1}
        bridge.last_parked_validation = {'ok': True}
        bridge.parking_refresh_armed = False
        bridge.stationary_evidence = StationaryEvidence()
        c = TrialCorrection(bound_total=True)
        c.baseline = c.last_transform = (1.0, 2.0, 0.0)
        c.amcl_stamp = 99.5
        c.xy_std = 0.01
        c.yaw_std_deg = 0.2
        c.measurement_ok = True
        bridge.correction = c
        bridge.get_clock = lambda: FakeClock()
        bridge.get_logger = lambda: FakeLogger()

        # First: reject when velocity publisher still exists
        bridge.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(node_name='nav2')] if topic == '/cmd_vel_nav' else []
        resp = SimpleNamespace(success=False, message='')
        AmclTrialBridge.reset_cycle(bridge, None, resp)
        self.assertFalse(resp.success)
        self.assertIn('command publishers must stop', resp.message)

        # Second: succeed when velocity topics are clear
        bridge.get_publishers_info_by_topic = lambda topic: []
        resp2 = SimpleNamespace(success=False, message='')
        AmclTrialBridge.reset_cycle(bridge, None, resp2)
        self.assertTrue(resp2.success, resp2.message)
        self.assertIsNone(bridge.trial_handoff)
        self.assertIsNone(bridge.delivery_handoff)
        self.assertTrue(bridge.navigation_monitor)
        self.assertTrue(bridge.parking_refresh_armed)
        self.assertTrue(bridge.correction.measurement_ok)


if __name__ == '__main__':
    unittest.main()
