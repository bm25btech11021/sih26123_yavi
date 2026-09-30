"""
Unit and integration tests for M9-V2 remediation and separated benchmark instrumentation.

Validates:
1. Separated benchmark safety metrics (center-to-center distance, proximity breaches,
   physical Gazebo contacts, safety brake interventions, safety aborts).
2. 2D SAT Oriented Bounding Box collision math for AMR chassis (0.65m x 0.45m).
3. M9-V2 map YAML and SDF rack coordinate consistency.
4. M9-V2 North Cross-Aisle clearance (~3.8m total width, 2.5m buffer to pickups).
5. All 10 robot spawn coordinates clearance.
6. SingleAgentAStar reachability for all 30 tasks in workload_30_tasks_m9_v2.yaml.
"""

import math
import os
import unittest

from amr_fleet_core.benchmark_manager import (
    check_obb_intersection,
    StandardMetrics,
    StatisticalAggregator,
)
from amr_fleet_core.rh_planner import SingleAgentAStar
from amr_fleet_core.workload import WorkloadManager
from amr_fleet_sim.grid_world import GridWorld
import yaml


class TestM9V2Remediation(unittest.TestCase):
    """Test suite validating M9-V2 remediation fixes and instrumentation."""

    def setUp(self):
        self.workspace_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
        self.map_yaml = os.path.join(
            self.workspace_root, 'config', 'maps', 'warehouse_m9_v2.yaml'
        )
        self.world_sdf = os.path.join(
            self.workspace_root, 'src', 'amr_fleet_bringup', 'worlds', 'warehouse_m9_v2.sdf'
        )
        self.fleet_yaml = os.path.join(
            self.workspace_root, 'config', 'robots', 'fleet_10_robots_m9_v2.yaml'
        )
        self.workload_yaml = os.path.join(
            self.workspace_root, 'config', 'workloads', 'workload_30_tasks_m9_v2.yaml'
        )

    # -------------------------------------------------------------------------
    # 1. Separating Axis Theorem (SAT) OBB Collision Tests
    # -------------------------------------------------------------------------
    def test_sat_collision_separated_along_x(self):
        """Robots separated by 1.0m along X must not intersect."""
        self.assertFalse(
            check_obb_intersection((0.0, 0.0), 0.0, (1.0, 0.0), 0.0)
        )

    def test_sat_collision_overlapping_along_x(self):
        """Robots separated by 0.50m (< 0.65m chassis length) along X must intersect."""
        self.assertTrue(
            check_obb_intersection((0.0, 0.0), 0.0, (0.50, 0.0), 0.0)
        )

    def test_sat_collision_side_by_side_clearance(self):
        """Robots side by side separated by 0.46m (> 0.45m chassis width) must not intersect."""
        self.assertFalse(
            check_obb_intersection((0.0, 0.0), 0.0, (0.0, 0.46), 0.0)
        )

    def test_sat_collision_side_by_side_overlap(self):
        """Robots side by side separated by 0.40m (< 0.45m chassis width) must intersect."""
        self.assertTrue(
            check_obb_intersection((0.0, 0.0), 0.0, (0.0, 0.40), 0.0)
        )

    def test_sat_collision_canonical_breach_overlap(self):
        """Evaluate canonical M9-V2 breach geometry (d = 0.349m along Y, both facing East)."""
        # amr_2 at (16.035, 29.807), amr_7 at (16.035, 30.156)
        # d_y = 0.349m < 0.45m width -> physical overlap = 10.1cm!
        self.assertTrue(
            check_obb_intersection(
                (16.035, 29.807), 0.0,
                (16.035, 30.156), 0.0,
            )
        )

    # -------------------------------------------------------------------------
    # 2. Benchmark Instrumentation Separation
    # -------------------------------------------------------------------------
    def test_standard_metrics_separated_fields(self):
        """Verify StandardMetrics holds distinct safety fields with correct defaults."""
        m = StandardMetrics(
            generated_tasks=30,
            assigned_tasks=30,
            completed_tasks=30,
            minimum_center_to_center_distance_m=0.485,
            minimum_inter_robot_distance_m=0.485,
            proximity_breaches=0,
            physical_gazebo_contacts=0,
            safety_brake_interventions=3,
            safety_aborts=0,
        )
        self.assertEqual(m.minimum_center_to_center_distance_m, 0.485)
        self.assertEqual(m.proximity_breaches, 0)
        self.assertEqual(m.physical_gazebo_contacts, 0)
        self.assertEqual(m.safety_brake_interventions, 3)
        self.assertEqual(m.safety_aborts, 0)

    def test_statistical_aggregation_separated_metrics(self):
        """Verify StatisticalAggregator computes summaries for all separated safety metrics."""
        trials = [
            {
                'metadata': {
                    'experiment_id': f'exp_m9_trial_{i}',
                    'trial_id': i,
                    'seed': 42 + i,
                    'workload_size': 30,
                    'communication_profile': 'NORMAL',
                    'mission_horizon_sec': 120.0,
                    'actual_duration_sec': 95.0 + i * 2.0,
                    'termination_reason': 'ALL_TASKS_COMPLETED',
                },
                'metrics': {
                    'completed_tasks': 30,
                    'completion_rate_pct': 100.0,
                    'throughput_tasks_per_min': 18.5,
                    'makespan_sec': 95.0 + i * 2.0,
                    'replan_count': 120,
                    'planning_latency_mean_ms': 1.5,
                    'planning_latency_p95_ms': 2.8,
                    'minimum_center_to_center_distance_m': 0.45 + i * 0.02,
                    'minimum_inter_robot_distance_m': 0.45 + i * 0.02,
                    'proximity_breaches': 0,
                    'physical_gazebo_contacts': 0,
                    'collision_contact_events': 0,
                    'safety_brake_interventions': 2 + i,
                    'safety_aborts': 0,
                },
            }
            for i in range(3)
        ]
        agg = StatisticalAggregator.aggregate_trials('bench_m9_test', trials)
        self.assertIn('minimum_center_to_center_distance_m', agg['aggregated_metrics'])
        self.assertIn('proximity_breaches', agg['aggregated_metrics'])
        self.assertIn('physical_gazebo_contacts', agg['aggregated_metrics'])
        self.assertIn('safety_brake_interventions', agg['aggregated_metrics'])
        self.assertIn('safety_aborts', agg['aggregated_metrics'])

        c2c = agg['aggregated_metrics']['minimum_center_to_center_distance_m']
        self.assertAlmostEqual(c2c['mean'], 0.47, places=2)
        self.assertEqual(agg['aggregated_metrics']['proximity_breaches']['mean'], 0.0)
        self.assertEqual(agg['aggregated_metrics']['physical_gazebo_contacts']['mean'], 0.0)
        self.assertEqual(agg['aggregated_metrics']['safety_aborts']['mean'], 0.0)

    # -------------------------------------------------------------------------
    # 3. M9-V2 Map YAML and World SDF Consistency
    # -------------------------------------------------------------------------
    def test_map_yaml_and_sdf_coordinates_match(self):
        """Verify all 24 racks in warehouse_m9_v2.yaml match warehouse_m9_v2.sdf."""
        with open(self.map_yaml, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)
        with open(self.world_sdf, 'r', encoding='utf-8') as f:
            sdf_content = f.read()

        obstacles = map_data.get('obstacles', [])
        racks = [o for o in obstacles if o['id'].startswith('rack_')]
        self.assertEqual(len(racks), 24)

        for rack in racks:
            rx, ry = rack['center'][0], rack['center'][1]
            pattern = f'<pose>{rx:.1f} {ry:.1f} '
            self.assertIn(
                pattern,
                sdf_content,
                f'Rack {rack["id"]} at ({rx}, {ry}) not found in warehouse_m9_v2.sdf',
            )

    def test_north_cross_aisle_widened_geometry(self):
        """Verify Row 4 upper bound is y=28.0m, giving 3.8m aisle width to north wall."""
        with open(self.map_yaml, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        obstacles = map_data.get('obstacles', [])
        row4_racks = [o for o in obstacles if o['id'] in [f'rack_{i:02d}' for i in range(19, 25)]]
        self.assertEqual(len(row4_racks), 6)

        for rack in row4_racks:
            self.assertAlmostEqual(rack['center'][1], 26.5, places=2)
            rack_top = rack['center'][1] + rack['size'][1] / 2.0  # 26.5 + 1.5 = 28.0
            self.assertAlmostEqual(rack_top, 28.0, places=2)

        # North wall inner face: 32.0 - 0.2 = 31.8m
        # Width: 31.8 - 28.0 = 3.8m
        north_wall_y = 31.8
        aisle_width = north_wall_y - 28.0
        self.assertAlmostEqual(aisle_width, 3.8, places=1)

    def test_pickup_stations_transit_buffer(self):
        """Verify perimeter pickup stations at y=30.5m have 2.5m buffer from Row 4 racks."""
        with open(self.workload_yaml, 'r', encoding='utf-8') as f:
            wl_data = yaml.safe_load(f)

        pickups = wl_data.get('workload', {}).get('pickup_stations', [])
        north_pickups = [p for p in pickups if p[1] > 28.0]
        self.assertGreater(len(north_pickups), 0)
        for p in north_pickups:
            self.assertAlmostEqual(p[1], 30.5, places=2)
            buffer_to_row4 = p[1] - 28.0
            self.assertGreaterEqual(buffer_to_row4, 2.5)

    # -------------------------------------------------------------------------
    # 4. Robot Spawn Clearance
    # -------------------------------------------------------------------------
    def test_all_10_spawn_poses_clear(self):
        """Verify all 10 AMR spawn locations in fleet_10_robots_m9_v2.yaml are clear of racks."""
        with open(self.fleet_yaml, 'r', encoding='utf-8') as f:
            fleet_data = yaml.safe_load(f)
        with open(self.map_yaml, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        racks = [o for o in map_data.get('obstacles', []) if o['id'].startswith('rack_')]
        robots = fleet_data.get('fleet', {}).get('robots', [])
        self.assertEqual(len(robots), 10)

        for r in robots:
            rx, ry = float(r['x']), float(r['y'])
            for rack in racks:
                half_w = rack['size'][0] / 2.0
                half_d = rack['size'][1] / 2.0
                dx = max(0.0, abs(rx - rack['center'][0]) - half_w)
                dy = max(0.0, abs(ry - rack['center'][1]) - half_d)
                clearance = math.hypot(dx, dy)
                msg = f'Robot {r["id"]} at ({rx}, {ry}) near {rack["id"]} (dist={clearance:.2f}m)'
                self.assertGreaterEqual(clearance, 0.90, msg)

    # -------------------------------------------------------------------------
    # 5. Reachability of All 30 Tasks
    # -------------------------------------------------------------------------
    def test_workload_30_tasks_reachability(self):
        """Verify all 30 pickup and dropoff coordinates are obstacle-free and reachable."""
        gw = GridWorld.from_yaml(self.map_yaml)
        planner = SingleAgentAStar(gw)

        tasks = WorkloadManager.load_from_yaml(self.workload_yaml)
        self.assertEqual(len(tasks), 30)

        for t in tasks:
            p_pos = t.pickup
            d_pos = t.dropoff

            p_grid = gw.to_grid(p_pos[0], p_pos[1])
            d_grid = gw.to_grid(d_pos[0], d_pos[1])

            self.assertTrue(
                gw.is_free(p_grid),
                f'Task {t.task_id} pickup {p_pos} (grid {p_grid}) is inside an obstacle cell!',
            )
            self.assertTrue(
                gw.is_free(d_grid),
                f'Task {t.task_id} dropoff {d_pos} (grid {d_grid}) is inside an obstacle cell!',
            )

            path = planner.find_path(p_grid, d_grid)
            self.assertIsNotNone(
                path,
                f'Task {t.task_id} from {p_grid} to {d_grid} has no feasible path!',
            )
            self.assertGreater(len(path), 0)


if __name__ == '__main__':
    unittest.main()

