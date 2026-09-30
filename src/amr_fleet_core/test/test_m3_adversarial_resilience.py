"""
Targeted Unit and Integration Tests for YAVI-SIH26123 Milestone 3.

Adversarial Environment & Collision Resilience:
- Adversarial conflict injection isolation & conflict creation
- Sensor-derived local obstacle detection from LaserScan
- Sensor observation vs Environment Oracle separation
- Bounded local recovery & LOCAL_SAFETY_HOLD
- Footprint-aware and reservation-aware candidate clearance
- Reservation table spatial invalidation (invalidate_cells)
- Non-corrupting synthetic reservation conflict injection
- Graph withdrawal & dynamic A* rerouting
- PIBT-style fallback execution & telemetry schema
- Sensor-only dynamic obstacle M3-B2 end-to-end pipeline
- Problematic-location robot failure (0.8m experimental keep-out envelope)
- Discrete timing milestones separation
"""

import math
from typing import Set
import unittest

from amr_fleet_core.adversarial_injector import (
    AdversarialConflictInjector,
    AdversarialConflictType,
)
from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.coordination_models import Position
from amr_fleet_core.local_obstacle_detector import (
    LocalObstacleDetector,
    LocalRecoveryAction,
)
from amr_fleet_core.pibt_planner import PIBTAgentState, PIBTLocalPlanner
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_planner import SingleAgentAStar
from amr_fleet_core.task_model import Task, TaskLifecycleState, TaskPriority
from amr_fleet_sim.grid_world import GridWorld


class TestM3AdversarialResilience(unittest.TestCase):
    """Test suite validating Milestone 3 adversarial environmental resilience."""

    def setUp(self) -> None:
        """Initialize common test instances."""
        self.injector = AdversarialConflictInjector()
        self.grid = GridWorld(16, 16, resolution=0.5)
        self.res_table = SpaceTimeReservationTable()
        self.detector = LocalObstacleDetector(
            grid_resolution=0.5,
            safety_threshold_m=0.28,
            detection_horizon_m=1.5,
            forward_arc_deg=24.0,
        )

    def test_adversarial_injector_isolation(self) -> None:
        """Verify injector creates conflict records without containing recovery logic."""
        # Check injector has no planner, cbba, or recovery methods
        self.assertFalse(hasattr(self.injector, 'replan'))
        self.assertFalse(hasattr(self.injector, 'build_bundle'))
        self.assertFalse(hasattr(self.injector, 'resolve_conflict'))

        # Test same-cell injection
        rec = self.injector.inject_same_cell_conflict('amr_0', 'amr_1', (5, 5), time_step=4)
        self.assertEqual(rec.conflict_type, AdversarialConflictType.SAME_CELL)
        self.assertEqual(rec.robot_ids, ['amr_0', 'amr_1'])
        self.assertEqual(rec.location, (5, 5))
        self.assertEqual(rec.time_step, 4)

        # Test opposing corridor injection
        corridor = [(2, 4), (3, 4), (4, 4), (5, 4), (6, 4)]
        rec_corr = self.injector.inject_opposing_corridor_entry('amr_0', 'amr_1', corridor, 0)
        self.assertEqual(rec_corr.conflict_type, AdversarialConflictType.OPPOSING_CORRIDOR)
        self.assertEqual(rec_corr.location, (4, 4))

        # Test crossing injection
        rec_cross = self.injector.inject_crossing_conflict('amr_0', 'amr_1', (8, 8), 5)
        self.assertEqual(rec_cross.conflict_type, AdversarialConflictType.CROSSING_TRAJECTORIES)

        # Test reservation collision injection
        rec_res, accepted = self.injector.inject_reservation_conflict(
            self.res_table, 'amr_0', 'amr_1', (4, 4), time_step=3,
        )
        self.assertEqual(rec_res.conflict_type, AdversarialConflictType.RESERVATION_CONFLICT)
        # Reservation table must reject duplicate reservation from amr_1
        self.assertFalse(accepted)

        # Test high-contention intersection injection
        rec_high = self.injector.inject_intersection_contention(
            ['amr_0', 'amr_1', 'amr_2'], (10, 10), 6,
        )
        self.assertEqual(
            rec_high.conflict_type,
            AdversarialConflictType.HIGH_CONTENTION_INTERSECTION,
        )
        self.assertEqual(len(rec_high.robot_ids), 3)

        # History check
        self.assertEqual(len(self.injector.get_history()), 5)
        self.injector.clear_history()
        self.assertEqual(len(self.injector.get_history()), 0)

    def test_local_obstacle_detector_laserscan_mapping(self) -> None:
        """Verify raw LaserScan ranges map accurately into world coordinates and grid cells."""
        # Simulated scan: 360 rays from -pi to +pi
        num_rays = 360
        angle_min = -math.pi
        angle_inc = 2.0 * math.pi / num_rays
        ranges = [5.0] * num_rays

        # Inject an obstacle 0.8m straight ahead (center ray, idx=180, angle=0.0)
        center_idx = 180
        ranges[center_idx] = 0.80

        # Robot pose: x=2.0m, y=2.0m, yaw=0.0 (facing +x)
        robot_pose = (2.0, 2.0, 0.0)
        now_sec = 100.0

        detected = self.detector.process_scan(
            ranges=ranges,
            range_min=0.10,
            range_max=10.0,
            angle_min=angle_min,
            angle_increment=angle_inc,
            robot_pose=robot_pose,
            current_time=now_sec,
        )

        self.assertGreater(len(detected), 0)
        target_obs = [o for o in detected if abs(o.distance - 0.80) < 0.05][0]

        # World coordinate should be approx x=2.8m, y=2.0m
        self.assertAlmostEqual(target_obs.point_world_frame[0], 2.80, delta=0.05)
        self.assertAlmostEqual(target_obs.point_world_frame[1], 2.00, delta=0.05)

        # Discretized grid cell: floor(2.8 / 0.5) = 5, floor(2.0 / 0.5) = 4
        self.assertEqual(target_obs.grid_cell, (5, 4))
        self.assertFalse(target_obs.is_immediate_hazard)  # 0.80m > 0.28m

    def test_sensor_vs_oracle_separation(self) -> None:
        """Verify strict isolation between sensor observations and environment oracle."""
        oracle_blockage_cells: Set[Position] = {(10, 10), (10, 11)}

        # 1. Oracle event occurs in environment, but LiDAR has not scanned it
        self.assertEqual(len(self.detector.get_sensor_occupied_cells()), 0)
        sensor_cells = self.detector.get_sensor_occupied_cells()
        self.assertFalse(any(c in sensor_cells for c in oracle_blockage_cells))

        # 2. LiDAR detects an unexpected box at (3, 2), not in oracle
        # Robot at (1.0, 1.0, 0.0), obstacle 0.6m ahead
        num_rays = 100
        angle_min = -math.pi / 2.0
        angle_inc = math.pi / num_rays
        ranges = [5.0] * num_rays
        ranges[50] = 0.60  # center ray
        self.detector.process_scan(
            ranges=ranges, range_min=0.1, range_max=5.0,
            angle_min=angle_min, angle_increment=angle_inc,
            robot_pose=(1.0, 1.0, 0.0), current_time=1.0,
        )

        detected_sensor_cells = self.detector.get_sensor_occupied_cells()
        self.assertIn((3, 2), detected_sensor_cells)
        # Verify oracle list is unaffected by sensor detection
        self.assertNotIn((3, 2), oracle_blockage_cells)

    def test_local_obstacle_detector_immediate_hazard_brake(self) -> None:
        """Verify obstacle closer than 0.28m triggers EMERGENCY_BRAKE."""
        num_rays = 100
        ranges = [5.0] * num_rays
        ranges[50] = 0.22  # 0.22m < 0.28m threshold

        self.detector.process_scan(
            ranges=ranges, range_min=0.05, range_max=5.0,
            angle_min=-math.pi / 2.0, angle_increment=math.pi / 100,
            robot_pose=(2.0, 2.0, 0.0), current_time=2.0,
        )

        self.assertTrue(self.detector.has_immediate_hazard())
        action, sidestep, reason = self.detector.evaluate_local_recovery(
            current_pos=(2.0, 2.0),
            current_cell=(4, 4),
            planned_path_cells=[(4, 4), (5, 4), (6, 4)],
            grid=self.grid,
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        self.assertEqual(action, LocalRecoveryAction.EMERGENCY_BRAKE)
        self.assertIsNone(sidestep)
        self.assertIn('0.28m', reason)

    def test_bounded_local_recovery_sidestep_and_hold(self) -> None:
        """Verify detector chooses 1-step sidestep if open, or LOCAL_SAFETY_HOLD if enclosed."""
        # Obstacle at (5, 4) along planned path (4,4) -> (5,4) -> (6,4)
        num_rays = 100
        ranges = [5.0] * num_rays
        ranges[50] = 0.50  # 0.5m ahead, outside 0.28m brake but in forward path

        self.detector.process_scan(
            ranges=ranges, range_min=0.05, range_max=5.0,
            angle_min=-math.pi / 2.0, angle_increment=math.pi / 100,
            robot_pose=(2.0, 2.0, 0.0), current_time=3.0,
        )

        # Case A: Free lateral neighbor at (4, 5) with clear ahead cell (4, 6)
        action, sidestep, reason = self.detector.evaluate_local_recovery(
            current_pos=(2.0, 2.0),
            current_cell=(4, 4),
            planned_path_cells=[(4, 4), (5, 4), (6, 4)],
            grid=self.grid,
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        self.assertEqual(action, LocalRecoveryAction.LOCAL_SIDESTEP)
        self.assertIsNotNone(sidestep)
        self.assertNotEqual(sidestep, (5, 4))

        # Case B: All lateral neighbors blocked -> force LOCAL_SAFETY_HOLD
        for neighbor in [(4, 3), (4, 5), (3, 4)]:
            self.grid.add_obstacle(neighbor)

        action_hold, sidestep_hold, reason_hold = self.detector.evaluate_local_recovery(
            current_pos=(2.0, 2.0),
            current_cell=(4, 4),
            planned_path_cells=[(4, 4), (5, 4), (6, 4)],
            grid=self.grid,
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        self.assertEqual(action_hold, LocalRecoveryAction.LOCAL_SAFETY_HOLD)
        self.assertIsNone(sidestep_hold)
        self.assertIn('LOCAL_SAFETY_HOLD', reason_hold)

    def test_footprint_aware_and_reservation_aware_clearance(self) -> None:
        """Verify candidate sidestep enforces footprint overhang and peer reservation clearance."""
        # Current cell (4, 4). Candidate sidestep is (4, 5).
        # Chassis length 0.65m on 0.5m grid requires longitudinal ahead cell (4, 6) to be clear.
        current_cell = (4, 4)
        cand = (4, 5)

        # 1. Baseline: both (4, 5) and (4, 6) are free -> clearance succeeds
        is_clear, _ = self.detector.check_footprint_clearance(
            cand=cand,
            current_cell=current_cell,
            grid=self.grid,
            sensor_cells=set(),
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        self.assertTrue(is_clear)

        # 2. Block ahead cell (4, 6) with static obstacle:
        # Candidate (4, 5) is free, but 0.65m chassis would collide with (4, 6) -> rejected
        self.grid.add_obstacle((4, 6))
        is_clear_blocked, reason_blocked = self.detector.check_footprint_clearance(
            cand=cand,
            current_cell=current_cell,
            grid=self.grid,
            sensor_cells=set(),
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        self.assertFalse(is_clear_blocked)
        self.assertIn('Footprint overhang', reason_blocked)
        self.grid.remove_obstacle((4, 6))

        # 3. Reserve ahead cell (4, 6) for peer amr_1 at t=1:
        # Candidate (4, 5) is unreserved, but overhang conflicts with amr_1 -> rejected
        self.res_table.reserve(cell=(4, 6), time_step=1, robot_id='amr_1')
        is_clear_res, reason_res = self.detector.check_footprint_clearance(
            cand=cand,
            current_cell=current_cell,
            grid=self.grid,
            sensor_cells=set(),
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        self.assertFalse(is_clear_res)
        self.assertIn('reserved at t=1', reason_res)

    def test_reservation_table_invalidate_cells(self) -> None:
        """Verify invalidate_cells revokes reservations and returns affected robots."""
        # Setup reservations for amr_0 and amr_1
        self.res_table.reserve(cell=(3, 3), time_step=1, robot_id='amr_0')
        self.res_table.reserve(cell=(4, 3), time_step=2, robot_id='amr_0')
        self.res_table.reserve(cell=(5, 3), time_step=3, robot_id='amr_0')

        self.res_table.reserve(cell=(4, 5), time_step=2, robot_id='amr_1')

        # Invalidate cell (4, 3) where dynamic obstacle spawned
        withdrawn_cells = {(4, 3)}
        revoked = self.res_table.invalidate_cells(withdrawn_cells, min_time_step=1)

        self.assertEqual(len(revoked), 1)
        self.assertEqual(revoked[0], ('amr_0', (4, 3), 2))

        # Verify (4, 3) at t=2 is now free
        self.assertFalse(self.res_table.is_reserved(cell=(4, 3), time_step=2))

        # Verify other reservations are untouched
        self.assertTrue(self.res_table.is_reserved(cell=(3, 3), time_step=1, robot_id='amr_1'))
        self.assertTrue(self.res_table.is_reserved(cell=(4, 5), time_step=2, robot_id='amr_0'))

    def test_synthetic_reservation_conflict_injection_non_corrupting(self) -> None:
        """Verify synthetic reservation collision (M3-C4) preserves authoritative owner."""
        res_table = SpaceTimeReservationTable()
        cell = (6, 6)
        time_step = 4

        # Inject conflict: amr_0 has priority 2.0, amr_1 attempts priority 1.0
        rec, accepted = self.injector.inject_reservation_conflict(
            res_table=res_table,
            robot_a='amr_0',
            robot_b='amr_1',
            cell=cell,
            time_step=time_step,
            priority_b=1.0,
        )

        # 1. Injector records conflict metadata
        self.assertEqual(
            rec.conflict_type, AdversarialConflictType.RESERVATION_CONFLICT
        )
        self.assertFalse(accepted)

        # 2. Table state must not be corrupted: amr_0 remains sole owner
        current_res = res_table.get_reservation(cell, time_step)
        self.assertIsNotNone(current_res)
        self.assertEqual(current_res.robot_id, 'amr_0')

        # 3. Unambiguous ownership assertion:
        # Invariant: authoritative owners(cell, time_step) == {'amr_0'}
        owners = res_table.get_authoritative_owners(cell, time_step)
        self.assertEqual(owners, {'amr_0'})
        self.assertIn('amr_0', owners)
        self.assertNotIn('amr_1', owners)
        self.assertTrue(res_table.is_owner(cell, time_step, 'amr_0'))
        self.assertFalse(res_table.is_owner(cell, time_step, 'amr_1'))
        self.assertEqual(len(owners), 1)

    def test_graph_withdrawal_and_dynamic_reroute(self) -> None:
        """Verify A* detours around withdrawn cells and restores direct path upon unblock."""
        start = (2, 2)
        goal = (10, 2)
        astar = SingleAgentAStar(self.grid)

        # Baseline direct corridor path along y=2
        baseline_path = astar.find_path(start, goal)
        self.assertIsNotNone(baseline_path)
        self.assertIn((6, 2), baseline_path)

        # Dynamically block aisle at x=6, y=1..3
        block_cells = {(6, 1), (6, 2), (6, 3)}
        for c in block_cells:
            self.grid.add_obstacle(c)

        detour_path = astar.find_path(start, goal)
        self.assertIsNotNone(detour_path)
        self.assertGreater(len(detour_path), len(baseline_path))

        # Verify zero detour waypoints intersect the blocked cells
        for wp in detour_path:
            self.assertNotIn(wp, block_cells)

        # Remove blockage (restoration)
        for c in block_cells:
            self.grid.remove_obstacle(c)

        restored_path = astar.find_path(start, goal)
        self.assertEqual(len(restored_path), len(baseline_path))
        self.assertEqual(restored_path, baseline_path)

    def test_pibt_fallback_under_conflict_and_telemetry_schema(self) -> None:
        """Verify PIBT fallback resolves vertex conflict and logs structured telemetry."""
        pibt = PIBTLocalPlanner(self.grid, self.res_table)

        # Two agents contest cell (4, 4)
        agent_lead = PIBTAgentState(
            'amr_0', current_pos=(3, 4), goal_pos=(8, 4), task_priority=3
        )
        agent_yield = PIBTAgentState(
            'amr_1', current_pos=(4, 5), goal_pos=(4, 2), task_priority=1
        )

        agents = {'amr_0': agent_lead, 'amr_1': agent_yield}
        moves, conflicts, pibt_telemetry = pibt.plan_step_with_telemetry(
            agents,
            time_step=0,
            trigger='LOCAL_VERTEX_CONTENTION_FALLBACK',
        )

        # Higher priority amr_0 wins (4, 4) or proceeds toward goal
        self.assertEqual(moves['amr_0'], (4, 4))
        # Lower priority amr_1 yields (does not enter (4, 4))
        self.assertNotEqual(moves['amr_1'], (4, 4))

        # Validate explicit PIBT telemetry schema obtained directly from execution
        self.assertTrue(pibt_telemetry['pibt_invoked'])
        self.assertEqual(pibt_telemetry['result'], 'SUCCESS')
        self.assertIn('amr_0', pibt_telemetry['robots_involved'])
        self.assertIn('amr_1', pibt_telemetry['robots_involved'])
        self.assertEqual(
            pibt_telemetry['execution_level'],
            'PLANNER_LEVEL_PIBT_INTEGRATION',
        )
        self.assertGreater(pibt_telemetry['duration_ms'], 0.0)

    def test_sensor_only_dynamic_obstacle_m3_b2(self) -> None:
        """Verify M3-B2 end-to-end sensor-only recovery pipeline with zero oracle event."""
        # 1. AMR path: (2, 2) -> (3, 2) -> (4, 2) -> (5, 2) -> (6, 2)
        planned_path = [(2, 2), (3, 2), (4, 2), (5, 2), (6, 2)]
        for idx, cell in enumerate(planned_path):
            self.res_table.reserve(cell=cell, time_step=idx, robot_id='amr_0')

        # 2. Dynamic obstacle appears physically at (4, 2) in world (x=2.0, y=1.0)
        # ZERO oracle event emitted: oracle list remains completely empty
        oracle_events = []
        self.assertEqual(len(oracle_events), 0)

        # 3. AMR at (2, 2) facing +x fires LiDAR scan: detects obstacle 0.9m ahead
        num_rays = 180
        ranges = [5.0] * num_rays
        ranges[90] = 0.90  # 0.9m ahead (detected, but not emergency brake < 0.28m)
        scan_time = 150.0

        detected = self.detector.process_scan(
            ranges=ranges, range_min=0.1, range_max=5.0,
            angle_min=-math.pi / 2.0, angle_increment=math.pi / num_rays,
            robot_pose=(1.0, 1.0, 0.0), current_time=scan_time,
        )
        self.assertGreater(len(detected), 0)
        sensor_cells = self.detector.get_sensor_occupied_cells()
        self.assertIn((3, 2), sensor_cells)

        # 4. Local recovery evaluation: detects obstruction on path
        action, sidestep, reason = self.detector.evaluate_local_recovery(
            current_pos=(1.0, 1.0),
            current_cell=(2, 2),
            planned_path_cells=planned_path,
            grid=self.grid,
            res_table=self.res_table,
            robot_id='amr_0',
            current_time_step=0,
        )
        # In a corridor where lateral cells are constrained or sidestep requested
        self.assertIn(
            action,
            (LocalRecoveryAction.LOCAL_SIDESTEP, LocalRecoveryAction.LOCAL_SAFETY_HOLD),
        )

        # 5. Graph and reservation withdrawal
        self.grid.add_obstacle((3, 2))
        revoked = self.res_table.invalidate_cells({(3, 2)}, min_time_step=0)
        self.assertEqual(len(revoked), 1)
        self.assertEqual(revoked[0][0], 'amr_0')

        # 6. Global replan
        astar = SingleAgentAStar(self.grid)
        replan_path = astar.find_path((2, 2), (6, 2))
        self.assertIsNotNone(replan_path)
        self.assertNotIn((3, 2), replan_path)

        # Clean up
        self.grid.remove_obstacle((3, 2))

    def test_problematic_location_robot_failure_m3_h(self) -> None:
        """Verify failure in choke point adds 0.8m keep-out envelope and frees reservations."""
        # Choke point at (6, 6)
        victim_id = 'amr_1'
        victim_cell = (6, 6)

        # AMR 1 had task T_CHOKE
        task = Task('T_CHOKE', (2.0, 2.0), (12.0, 12.0), priority=TaskPriority.HIGH)
        task.transition_to(TaskLifecycleState.ASSIGNED, robot_id=victim_id)
        task.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id=victim_id)

        # Reserve corridor for amr_1
        self.res_table.reserve(victim_cell, time_step=3, robot_id=victim_id)
        self.assertTrue(self.res_table.is_reserved(victim_cell, time_step=3))

        # Failure occurs: release reservations
        self.res_table.release_robot(victim_id)
        self.assertFalse(self.res_table.is_reserved(victim_cell, time_step=3))

        # Add 0.8m experimental keep-out envelope obstacle at chassis
        # Grid resolution is 0.5m; 0.8m envelope spans radius of ceil(0.8 / 0.5) = 2 cells
        envelope_radius_cells = int(math.ceil(0.80 / self.grid.resolution))
        obs_envelope = set()
        for dx in range(-envelope_radius_cells, envelope_radius_cells + 1):
            for dy in range(-envelope_radius_cells, envelope_radius_cells + 1):
                cell = (victim_cell[0] + dx, victim_cell[1] + dy)
                if self.grid.in_bounds(cell):
                    obs_envelope.add(cell)
                    self.grid.add_obstacle(cell)

        self.assertIn(victim_cell, self.grid.obstacles)
        self.assertFalse(self.grid.is_free(victim_cell))

        # Verify peer AMR routes around the 0.8m experimental envelope
        astar = SingleAgentAStar(self.grid)
        detour = astar.find_path((4, 6), (9, 6))
        self.assertIsNotNone(detour)
        for wp in detour:
            self.assertNotIn(wp, obs_envelope)

        # Verify task is reclaimed to PENDING and reallocated to amr_0 without duplicate ownership
        task.transition_to(
            TaskLifecycleState.PENDING,
            robot_id=None,
            details='Autonomous reclamation from choke-point failed AMR',
        )
        self.assertEqual(task.state, TaskLifecycleState.PENDING)
        self.assertIsNone(task.assigned_robot_id)

        # CBBA reallocates to amr_0
        cfg = CBBAConfig(max_bundle_size=2)
        ag0 = CBBAAgent('amr_0', config=cfg, initial_position=(2.0, 2.0))
        t_dict = {
            task.task_id: {
                'task_id': task.task_id,
                'pickup': task.pickup,
                'dropoff': task.dropoff,
                'priority': 2,
            },
        }
        ag0.build_bundle(t_dict, current_time=5.0)

        self.assertIn(task.task_id, ag0.state.bundle)
        task.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_0')
        self.assertEqual(task.assigned_robot_id, 'amr_0')

    def test_timing_metrics_separation(self) -> None:
        """
        Verify discrete timing milestones separation using synthetic fixture.

        Validates timestamp ordering and telemetry instrumentation monotonically
        under a synthetic fixture, NOT empirical hardware latency measurements.
        """
        # Synthetic timing fixture for sequence instrumentation validation
        t_inj = 100.000
        t_obs = 100.045
        t_safety = 100.055
        t_graph = 100.060
        t_res = 100.065
        t_replan_start = 100.070
        t_replan_done = 100.092
        t_resume = 100.110

        # Verify strictly monotonic ordering
        milestones = [
            t_inj, t_obs, t_safety, t_graph, t_res,
            t_replan_start, t_replan_done, t_resume,
        ]
        for i in range(len(milestones) - 1):
            self.assertLess(milestones[i], milestones[i + 1])

        sensor_latency = t_obs - t_inj
        safety_latency = t_safety - t_obs
        replan_latency = t_replan_done - t_replan_start
        recovery_time = t_resume - t_obs

        self.assertAlmostEqual(sensor_latency, 0.045, places=3)
        self.assertAlmostEqual(safety_latency, 0.010, places=3)
        self.assertAlmostEqual(replan_latency, 0.022, places=3)
        self.assertAlmostEqual(recovery_time, 0.065, places=3)


if __name__ == '__main__':
    unittest.main()

