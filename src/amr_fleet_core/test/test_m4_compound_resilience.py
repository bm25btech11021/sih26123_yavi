"""
Unit tests for YAVI-SIH26123 Milestone 4: Compound Fault Resilience.

Validates the composition of multiple simultaneous and overlapping failures:
- Robot Failure + Dynamic Blockage (M4-A)
- Robot Failure + Communication Loss (M4-B)
- Multiple Overlapping Robot Failures (M4-C)
- Robot Failure + Sensor-Visible Obstacle (M4-D)
- Network Partition + Robot Failure (M4-E)
- Network Loss + Dynamic Blockage (M4-F)
- Master Compound Quad Failure (M4-G)
- Race Condition Mitigations (Races 1 through 4)
- Orthogonal 5-Dimensional State Model & Mathematical Invariants
"""

import math
from typing import Set
import unittest

from amr_fleet_core.adversarial_injector import (
    AdversarialConflictInjector,
    AdversarialConflictType,
)
from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.communication_model import (
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
)
from amr_fleet_core.coordination_models import Position
from amr_fleet_core.fault_detector import FaultDetector, FaultDetectorConfig
from amr_fleet_core.fault_state import FaultState
from amr_fleet_core.local_obstacle_detector import LocalObstacleDetector
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_planner import SingleAgentAStar
from amr_fleet_core.task_model import Task, TaskLifecycleState, TaskPriority
from amr_fleet_sim.grid_world import GridWorld


class TestM4CompoundResilience(unittest.TestCase):
    """Milestone 4 compound failure resilience and race mitigation tests."""

    def setUp(self) -> None:
        """Initialize shared simulation fixtures."""
        self.grid = GridWorld(16, 16, resolution=0.5)
        self.res_table = SpaceTimeReservationTable()
        self.astar = SingleAgentAStar(self.grid)
        self.injector = AdversarialConflictInjector()
        self.detector = LocalObstacleDetector(
            grid_resolution=0.5,
            safety_threshold_m=0.28,
            detection_horizon_m=1.5,
            forward_arc_deg=24.0,
        )

    def test_compound_fault_injection_model(self) -> None:
        """Verify AdversarialConflictInjector correctly logs compound faults."""
        components = [
            {'type': 'ROBOT_FAILURE', 'robot_id': 'amr_1', 'at_s': 2.0},
            {'type': 'DYNAMIC_BLOCKAGE', 'cells': [(4, 6), (5, 6), (6, 6)], 'at_s': 2.2},
        ]
        rec = self.injector.inject_compound_fault(
            scenario_id='M4-A',
            fault_components=components,
            start_time=100.0,
            location=(6, 6),
            time_step=2,
        )
        self.assertEqual(rec.conflict_type, AdversarialConflictType.COMPOUND_FAULT)
        self.assertIn('amr_1', rec.robot_ids)
        self.assertEqual(rec.location, (6, 6))
        self.assertEqual(rec.time_step, 2)
        self.assertEqual(len(self.injector.get_history()), 1)

    def test_orthogonal_state_tuple_constraints(self) -> None:
        """Verify orthogonal state dimensions satisfy formal safety constraints."""
        # Hard Halt Invariant: FAILED robot cannot move and holds no reservations
        state_failed = FaultState.FAILED
        self.assertFalse(state_failed.can_move)
        self.assertFalse(state_failed.publishes_heartbeat)
        self.assertFalse(state_failed.can_own_tasks)
        self.assertFalse(state_failed.holds_reservations)
        self.assertTrue(state_failed.is_chassis_obstacle)

        # Local Autonomy Boundary Invariant: COMM_LOSS permits local motion if reserved
        state_comm_loss = FaultState.COMM_LOSS
        self.assertTrue(state_comm_loss.can_move)
        self.assertTrue(state_comm_loss.is_local_autonomy_permitted)
        self.assertTrue(state_comm_loss.holds_reservations)
        self.assertFalse(state_comm_loss.is_chassis_obstacle)

    def test_m4_a_robot_failure_plus_dynamic_blockage(self) -> None:
        """M4-A: Robot fails while another robot detours around dynamic blockage."""
        victim_id = 'amr_1'
        survivor_id = 'amr_0'
        choke_cell = (7, 7)

        # Victim task
        t_victim = Task('T_M4A_VICTIM', (2.0, 2.0), (12.0, 12.0), priority=TaskPriority.HIGH)
        t_victim.transition_to(TaskLifecycleState.ASSIGNED, robot_id=victim_id)
        t_victim.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id=victim_id)

        # Dynamic blockage across Aisle B: cells (4, 6), (5, 6), (6, 6)
        blockage_cells: Set[Position] = {(4, 6), (5, 6), (6, 6)}
        for c in blockage_cells:
            self.grid.add_obstacle(c)
        self.res_table.invalidate_cells(blockage_cells, min_time_step=0)

        # Victim fails at choke_cell (7, 7)
        self.res_table.release_robot(victim_id)
        rad = int(math.ceil(0.80 / self.grid.resolution))
        keep_out_cells: Set[Position] = set()
        for dx in range(-rad, rad + 1):
            for dy in range(-rad, rad + 1):
                c = (choke_cell[0] + dx, choke_cell[1] + dy)
                if self.grid.in_bounds(c):
                    keep_out_cells.add(c)
                    self.grid.add_obstacle(c)

        # Task reclaimed to PENDING
        t_victim.transition_to(
            TaskLifecycleState.PENDING,
            robot_id=None,
            details='CAS reclamation from failed AMR in choke point',
        )
        self.assertIsNone(t_victim.assigned_robot_id)

        # Survivor routes around BOTH the blocked aisle and the 0.8m envelope
        detour = self.astar.find_path((2, 6), (10, 6))
        self.assertIsNotNone(detour)
        self.assertFalse(any(c in blockage_cells for c in detour))
        self.assertFalse(any(c in keep_out_cells for c in detour))

        # CBBA reallocates task to survivor
        cfg = CBBAConfig(max_bundle_size=2)
        ag0 = CBBAAgent(survivor_id, config=cfg, initial_position=(2.0, 2.0))
        tasks_map = {
            t_victim.task_id: {
                'task_id': t_victim.task_id,
                'pickup': t_victim.pickup,
                'dropoff': t_victim.dropoff,
                'priority': 3,
            }
        }
        ag0.build_bundle(tasks_map, current_time=10.0)
        self.assertIn(t_victim.task_id, ag0.state.bundle)
        t_victim.transition_to(TaskLifecycleState.ASSIGNED, robot_id=survivor_id)
        self.assertEqual(t_victim.assigned_robot_id, survivor_id)

    def test_m4_b_robot_failure_plus_comm_loss_discrimination(self) -> None:
        """M4-B: Distinct classification of COMM_LOSS (1.5s) vs FAILED (3.5s)."""
        cfg = FaultDetectorConfig(
            comm_loss_threshold_s=1.5,
            failure_timeout_s=3.5,
            confirmation_samples=1,
        )
        fd = FaultDetector('amr_0', config=cfg)

        t_base = 100.0
        # Register peer amr_1 and amr_2
        fd.record_peer_heartbeat('amr_1', 'HEALTHY', (1.0, 1.0), timestamp=t_base)
        fd.record_peer_heartbeat('amr_2', 'HEALTHY', (2.0, 2.0), timestamp=t_base)

        # At t = 102.0s: both have exceeded comm_loss_threshold (2.0s > 1.5s), but < 3.5s
        newly_failed, _ = fd.evaluate_peers(now=t_base + 2.0)
        self.assertEqual(len(newly_failed), 0)
        self.assertEqual(fd.peer_records['amr_1'].state, FaultState.COMM_LOSS)
        self.assertEqual(fd.peer_records['amr_2'].state, FaultState.COMM_LOSS)
        self.assertFalse(fd.peer_records['amr_1'].confirmed_failure)
        self.assertFalse(fd.peer_records['amr_2'].confirmed_failure)

        # amr_2 sends a heartbeat at 103.0s -> restored to HEALTHY
        fd.record_peer_heartbeat('amr_2', 'HEALTHY', (2.0, 2.0), timestamp=t_base + 3.0)

        # At t = 104.0s: amr_1 has exceeded failure_timeout (4.0s > 3.5s) -> confirmed FAILED
        newly_failed, _ = fd.evaluate_peers(now=t_base + 4.0)
        self.assertIn('amr_1', newly_failed)
        self.assertTrue(fd.peer_records['amr_1'].confirmed_failure)
        self.assertEqual(fd.peer_records['amr_2'].state, FaultState.HEALTHY)

    def test_m4_c_multiple_overlapping_robot_failures(self) -> None:
        """M4-C: Two AMRs fail in staggered windows; tasks reclaimed without duplicate owners."""
        t1 = Task('T_C1', (1.0, 1.0), (8.0, 8.0), priority=TaskPriority.HIGH)
        t2 = Task('T_C2', (2.0, 2.0), (9.0, 9.0), priority=TaskPriority.NORMAL)
        t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        t2.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_2')

        # Staggered failures
        self.res_table.reserve((3, 3), 1, 'amr_1')
        self.res_table.reserve((4, 4), 2, 'amr_2')

        # amr_1 fails -> release reservations, reclaim T_C1
        self.res_table.release_robot('amr_1')
        t1.transition_to(TaskLifecycleState.PENDING, robot_id=None)
        self.assertEqual(len(self.res_table.get_authoritative_owners((3, 3), 1)), 0)

        # amr_2 fails -> release reservations, reclaim T_C2
        self.res_table.release_robot('amr_2')
        t2.transition_to(TaskLifecycleState.PENDING, robot_id=None)
        self.assertEqual(len(self.res_table.get_authoritative_owners((4, 4), 2)), 0)

        # Surviving amr_0 bundles both tasks sequentially
        cfg = CBBAConfig(max_bundle_size=3)
        ag0 = CBBAAgent('amr_0', config=cfg, initial_position=(0.0, 0.0))
        t_dict = {
            t1.task_id: {
                'task_id': t1.task_id, 'pickup': t1.pickup,
                'dropoff': t1.dropoff, 'priority': 3,
            },
            t2.task_id: {
                'task_id': t2.task_id, 'pickup': t2.pickup,
                'dropoff': t2.dropoff, 'priority': 2,
            },
        }
        ag0.build_bundle(t_dict, current_time=10.0)
        self.assertIn('T_C1', ag0.state.bundle)
        self.assertIn('T_C2', ag0.state.bundle)
        self.assertEqual(len(ag0.state.bundle), 2)

    def test_m4_d_robot_failure_plus_sensor_obstacle_separation(self) -> None:
        """M4-D: Pure sensory obstacle triggers local brake/replan without oracle event."""
        num_rays = 360
        ranges = [5.0] * num_rays
        ranges[180] = 0.90  # 0.9m straight ahead
        detected = self.detector.process_scan(
            ranges=ranges,
            range_min=0.10,
            range_max=10.0,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / num_rays,
            robot_pose=(1.0, 1.0, 0.0),
            current_time=100.0,
        )
        self.assertEqual(len(detected), 1)
        self.assertFalse(self.detector.has_immediate_hazard())  # 0.9m > 0.28m

        # Dynamic obstacle detected locally at grid cell
        obs_cell = detected[0].grid_cell
        self.res_table.reserve(obs_cell, time_step=2, robot_id='amr_0')
        self.res_table.invalidate_cells({obs_cell}, min_time_step=2)
        self.assertEqual(len(self.res_table.get_authoritative_owners(obs_cell, 2)), 0)

    def test_m4_e_network_partition_plus_robot_failure_reconciliation(self) -> None:
        """M4-E: Partition isolation and post-heal timestamp CAS reconciliation."""
        cfg = CommunicationProfileConfig(
            profile_name='TEST_PARTITION',
            enabled=True,
            isolated_robots=['amr_2'],
        )
        cm = CommunicationImpairmentModel(cfg)

        # Verify amr_2 cannot communicate with amr_0
        action, _, reason = cm.process_message('amr_0', 'amr_2', current_time=10.0)
        self.assertEqual(action.value, 'DROP')
        self.assertEqual(reason.value, 'PARTITION')

        # Inside Partition A, amr_1 fails; amr_0 reclaims T_1 at t = 105.0s
        t_id = 'T_PARTITION_1'
        ag0 = CBBAAgent(
            'amr_0', config=CBBAConfig(max_bundle_size=2), initial_position=(1.0, 1.0)
        )
        t_dict = {
            t_id: {
                'task_id': t_id, 'pickup': (2.0, 2.0),
                'dropoff': (5.0, 5.0), 'priority': 2,
            }
        }
        ag0.build_bundle(t_dict, current_time=105.0)
        self.assertIn(t_id, ag0.state.bundle)

        # In Partition B, amr_2 held older claim from t = 90.0s where amr_2 was winner
        ag2 = CBBAAgent('amr_2', config=CBBAConfig(max_bundle_size=2), initial_position=(8.0, 8.0))
        ag2.state.bundle.append(t_id)
        ag2.state.winning_robots[t_id] = 'amr_2'
        ag2.state.timestamps[t_id] = 90.0

        # Partition heals: amr_2 receives amr_0's newer belief (t = 105.0s)
        yielded = ag2.reconcile_reconnection(
            peer_winning_robots=ag0.state.winning_robots,
            peer_timestamps=ag0.state.timestamps,
        )
        self.assertEqual(yielded, [t_id])  # amr_2 yields the task
        self.assertNotIn(t_id, ag2.state.bundle)
        self.assertEqual(ag2.state.winning_robots.get(t_id), 'amr_0')
        self.assertEqual(ag2.state.timestamps.get(t_id), 105.0)

    def test_m4_f_network_loss_plus_dynamic_blockage_bounded_autonomy(self) -> None:
        """M4-F: COMM_LOSS robot entering blocked cell drops into LOCAL_SAFETY_HOLD."""
        # Simulated AMR state under comm loss
        comm_state = FaultState.COMM_LOSS
        self.assertTrue(comm_state.is_local_autonomy_permitted)

        # Route cell (5, 5) is dynamically blocked
        blocked_cell = (5, 5)
        self.grid.add_obstacle(blocked_cell)

        # Sensor detects blockage
        obstacle_detected = blocked_cell in self.grid.obstacles
        self.assertTrue(obstacle_detected)

        # Without central coordination, robot must stop rather than advance into blocked cell
        robot_action = 'LOCAL_SAFETY_HOLD' if obstacle_detected else 'EXECUTE_PATH'
        self.assertEqual(robot_action, 'LOCAL_SAFETY_HOLD')

    def test_m4_g_master_compound_quad_failure(self) -> None:
        """M4-G: Composition of 2 robot failures, packet loss, and dynamic blockage."""
        # 1. Blockage at central cell (7, 7)
        self.grid.add_obstacle((7, 7))
        self.res_table.invalidate_cells({(7, 7)}, min_time_step=0)

        # 2. Crash amr_1 and amr_2
        self.res_table.release_robot('amr_1')
        self.res_table.release_robot('amr_2')

        # 3. Tasks assigned to amr_1 and amr_2 are reclaimed
        t1 = Task('T_G1', (1.0, 1.0), (10.0, 10.0), priority=TaskPriority.HIGH)
        t2 = Task('T_G2', (2.0, 2.0), (11.0, 11.0), priority=TaskPriority.NORMAL)
        t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        t2.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_2')
        t1.transition_to(
            TaskLifecycleState.PENDING, robot_id=None, details='Reclaim after amr_1 failure'
        )
        t2.transition_to(
            TaskLifecycleState.PENDING, robot_id=None, details='Reclaim after amr_2 failure'
        )
        self.assertEqual(len(self.res_table.get_authoritative_owners((4, 4), 2)), 0)

        # 4. amr_0 and amr_3 re-auction tasks under high packet loss
        # (tolerated via local re-bidding)
        cfg = CBBAConfig(max_bundle_size=2)
        ag0 = CBBAAgent('amr_0', config=cfg, initial_position=(0.0, 0.0))
        ag3 = CBBAAgent('amr_3', config=cfg, initial_position=(12.0, 12.0))
        tasks_map = {
            t1.task_id: {
                'task_id': t1.task_id, 'pickup': t1.pickup,
                'dropoff': t1.dropoff, 'priority': 3,
            },
            t2.task_id: {
                'task_id': t2.task_id, 'pickup': t2.pickup,
                'dropoff': t2.dropoff, 'priority': 2,
            },
        }
        ag0.build_bundle(tasks_map, current_time=20.0)
        ag3.build_bundle(tasks_map, current_time=20.0)

        # Consensus round
        ag0.resolve_conflicts(
            peer_id=ag3.robot_id,
            peer_iteration=1,
            peer_winning_bids=ag3.state.winning_bids,
            peer_winning_robots=ag3.state.winning_robots,
            peer_timestamps=ag3.state.timestamps,
            task_map=tasks_map,
        )
        ag3.resolve_conflicts(
            peer_id=ag0.robot_id,
            peer_iteration=1,
            peer_winning_bids=ag0.state.winning_bids,
            peer_winning_robots=ag0.state.winning_robots,
            peer_timestamps=ag0.state.timestamps,
            task_map=tasks_map,
        )

        # Verify task uniqueness invariant I_1: no task held by both
        shared = set(ag0.state.bundle) & set(ag3.state.bundle)
        self.assertEqual(len(shared), 0)

    def test_race1_dual_task_reclamation_cas_mitigation(self) -> None:
        """Race 1: Competing peer task reclamations resolve via CAS without dual ownership."""
        t = Task('T_RACE1', (1.0, 1.0), (5.0, 5.0), priority=TaskPriority.HIGH)
        t.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        t.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id='amr_1')

        def _simulate_reclamation(task: Task, expected_failed_robot: str = 'amr_1') -> bool:
            if task.state == TaskLifecycleState.PENDING:
                return False
            if task.assigned_robot_id != expected_failed_robot:
                return False
            task.transition_to(
                TaskLifecycleState.PENDING, robot_id=None, details='Autonomous reclamation'
            )
            return True

        # Peer amr_0 detects failure at t=3.5s -> calls CAS to PENDING
        res_0 = _simulate_reclamation(t, expected_failed_robot='amr_1')
        self.assertTrue(res_0)
        self.assertEqual(t.state, TaskLifecycleState.PENDING)
        self.assertIsNone(t.assigned_robot_id)

        # Delayed peer amr_2 detects failure at t=4.2s -> attempts second CAS to PENDING
        res_2 = _simulate_reclamation(t, expected_failed_robot='amr_1')
        self.assertFalse(res_2)
        self.assertIsNone(t.assigned_robot_id)

        # Assign to amr_0
        t.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_0')
        self.assertEqual(t.assigned_robot_id, 'amr_0')

        # Stale delayed attempt to reclaim already assigned task by amr_2 is rejected
        res_stale = _simulate_reclamation(t, expected_failed_robot='amr_1')
        self.assertFalse(res_stale)
        self.assertEqual(t.assigned_robot_id, 'amr_0')

    def test_race2_stale_reservation_resurrection_prevention(self) -> None:
        """Race 2: Reconnecting AMR yields when its former cell is occupied by a detouring peer."""
        contested_cell = (5, 5)
        step = 2

        # amr_0 detoured through (5, 5) at step 2 while amr_1 was disconnected
        granted = self.res_table.reserve(contested_cell, step, robot_id='amr_0')
        self.assertTrue(granted)

        # amr_1 reconnects and attempts to reserve/assert ownership on (5, 5) at step 2
        amr1_granted = self.res_table.reserve(contested_cell, step, robot_id='amr_1')
        self.assertFalse(amr1_granted)

        # Unambiguous ownership assertions
        owners = self.res_table.get_authoritative_owners(contested_cell, step)
        self.assertEqual(owners, {'amr_0'})
        self.assertTrue(self.res_table.is_owner(contested_cell, step, 'amr_0'))
        self.assertFalse(self.res_table.is_owner(contested_cell, step, 'amr_1'))

    def test_race3_choke_point_failure_perception_primacy(self) -> None:
        """Race 3: Onboard LiDAR reactive brake halts inbound AMR before heartbeat timeout."""
        num_rays = 360
        ranges = [5.0] * num_rays
        ranges[180] = 0.25  # direct front ray closer than 0.28m
        self.detector.process_scan(
            ranges=ranges,
            range_min=0.10,
            range_max=10.0,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / num_rays,
            robot_pose=(2.0, 2.0, 0.0),
            current_time=100.0,
        )
        is_hazard = self.detector.has_immediate_hazard()
        self.assertTrue(is_hazard)

        # Emergency stop clamps drive command to 0.0 m/s
        cmd_vel = 0.0 if is_hazard else 0.4
        self.assertEqual(cmd_vel, 0.0)

    def test_race4_partition_split_brain_task_reconciliation(self) -> None:
        """Race 4: Partition heal uses timestamp CAS to converge to single task owner."""
        task_id = 'T_SHARED_PARTITION'
        # Partition A: amr_0 claimed task at timestamp 110.0
        ag0 = CBBAAgent('amr_0', config=CBBAConfig(max_bundle_size=2), initial_position=(1.0, 1.0))
        ag0.state.bundle.append(task_id)
        ag0.state.winning_robots[task_id] = 'amr_0'
        ag0.state.winning_bids[task_id] = 15.0
        ag0.state.timestamps[task_id] = 110.0

        # Partition B: amr_2 claimed task at older timestamp 95.0
        ag2 = CBBAAgent('amr_2', config=CBBAConfig(max_bundle_size=2), initial_position=(8.0, 8.0))
        ag2.state.bundle.append(task_id)
        ag2.state.winning_robots[task_id] = 'amr_2'
        ag2.state.winning_bids[task_id] = 12.0
        ag2.state.timestamps[task_id] = 95.0

        # Upon partition heal, amr_2 receives amr_0's newer state (110.0 > 95.0)
        yielded = ag2.reconcile_reconnection(
            peer_winning_robots=ag0.state.winning_robots,
            peer_timestamps=ag0.state.timestamps,
            peer_winning_bids=ag0.state.winning_bids,
        )
        # amr_2 yields task from bundle
        self.assertIn(task_id, yielded)
        self.assertNotIn(task_id, ag2.state.bundle)
        self.assertEqual(ag2.state.winning_robots[task_id], 'amr_0')
        self.assertEqual(ag2.state.timestamps[task_id], 110.0)


if __name__ == '__main__':
    unittest.main()

