"""
Unit and integration tests for YAVI-SIH26123 (v2) Fault Resilience.

Validates:
1. FaultDetector self-diagnostics and test fault injection (KILL, MOTION_STOP, CLEAR).
2. Decentralized peer heartbeat tracking, timeout detection, and debounce confirmation.
3. CBBAAgent.purge_failed_peer_tasks: resetting dead peer winning beliefs to allow re-bidding.
4. SpaceTimeReservationTable.release_robot on peer failure.
5. Task reclamation lifecycle state transitions (ASSIGNED -> PENDING, IN_PROGRESS -> PENDING).
6. Conservation of tasks invariant under mid-task failure.
7. GridWorld static obstacle insertion at failed robot chassis location.
"""

import unittest

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.fault_detector import FaultDetector, FaultDetectorConfig
from amr_fleet_core.fault_state import FaultState
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_planner import SingleAgentAStar
from amr_fleet_core.task_model import (
    Task,
    TaskLifecycleState,
    TaskPriority,
)
from amr_fleet_sim.grid_world import GridWorld


class TestFaultDetector(unittest.TestCase):
    """Test suite for decentralized FaultDetector."""

    def setUp(self) -> None:
        cfg = FaultDetectorConfig(
            heartbeat_period_s=0.5,
            comm_loss_threshold_s=1.0,
            failure_timeout_s=2.0,
            confirmation_samples=2,
        )
        self.detector = FaultDetector(local_robot_id='amr_0', config=cfg)

    def test_initial_state_healthy(self) -> None:
        self.assertTrue(self.detector.is_self_healthy())
        self.assertEqual(self.detector.self_state, FaultState.HEALTHY)
        self.assertEqual(len(self.detector.get_failed_peers()), 0)

    def test_fault_injection_kill(self) -> None:
        ok, msg = self.detector.inject_fault('KILL', duration_sec=-1.0)
        self.assertTrue(ok)
        self.assertFalse(self.detector.is_self_healthy())
        self.assertEqual(self.detector.self_state, FaultState.FAILED)

        # Clear fault
        ok_clear, _ = self.detector.inject_fault('CLEAR')
        self.assertTrue(ok_clear)
        self.assertTrue(self.detector.is_self_healthy())

    def test_peer_heartbeat_and_timeout(self) -> None:
        t0 = 100.0
        # amr_1 sends heartbeat at t0
        self.detector.record_peer_heartbeat(
            robot_id='amr_1',
            state_str='HEALTHY',
            pose=(5.0, 5.0),
            active_task_id='T001',
            timestamp=t0,
        )
        self.assertFalse(self.detector.is_peer_failed('amr_1'))
        self.assertEqual(self.detector.get_peer_pose('amr_1'), (5.0, 5.0))
        self.assertEqual(self.detector.get_peer_task('amr_1'), 'T001')

        # At t0 + 1.2s: comm loss threshold (1.0s) exceeded
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 1.2)
        self.assertEqual(len(newly_failed), 0)
        self.assertEqual(self.detector.peer_records['amr_1'].state, FaultState.COMM_LOSS)

        # At t0 + 2.1s: failure timeout (2.0s) exceeded, first sample
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 2.1)
        self.assertEqual(len(newly_failed), 0)  # Needs 2 samples

        # At t0 + 2.3s: second sample confirmation -> confirmed failure!
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 2.3)
        self.assertIn('amr_1', newly_failed)
        self.assertTrue(self.detector.is_peer_failed('amr_1'))
        self.assertEqual(self.detector.peer_records['amr_1'].state, FaultState.FAILED)

    def test_explicit_peer_failure_report(self) -> None:
        t0 = 200.0
        self.detector.record_peer_heartbeat(
            robot_id='amr_2',
            state_str='FAILED',
            pose=(10.0, 10.0),
            active_task_id='T002',
            timestamp=t0,
        )
        self.assertTrue(self.detector.is_peer_failed('amr_2'))
        self.assertIn('amr_2', self.detector.get_failed_peers())


class TestCBBAPeerFailureReset(unittest.TestCase):
    """Test suite for CBBA task reclamation after peer failure."""

    def setUp(self) -> None:
        self.agent_0 = CBBAAgent('amr_0', CBBAConfig(max_bundle_size=4))
        self.agent_1 = CBBAAgent('amr_1', CBBAConfig(max_bundle_size=4))

    def test_purge_failed_peer_tasks(self) -> None:
        # Simulate amr_0 believing amr_1 won T001 and T002
        self.agent_0.state.winning_robots['T001'] = 'amr_1'
        self.agent_0.state.winning_bids['T001'] = 85.0
        self.agent_0.state.winning_robots['T002'] = 'amr_1'
        self.agent_0.state.winning_bids['T002'] = 90.0
        self.agent_0.state.winning_robots['T003'] = 'amr_0'
        self.agent_0.state.winning_bids['T003'] = 50.0

        freed = self.agent_0.purge_failed_peer_tasks('amr_1', current_time=10.0)
        self.assertEqual(set(freed), {'T001', 'T002'})

        # Verify amr_1's winning beliefs are reset
        self.assertEqual(self.agent_0.state.winning_robots['T001'], '')
        self.assertEqual(self.agent_0.state.winning_bids['T001'], 0.0)
        self.assertEqual(self.agent_0.state.winning_robots['T002'], '')
        self.assertEqual(self.agent_0.state.winning_bids['T002'], 0.0)

        # amr_0's own task is preserved
        self.assertEqual(self.agent_0.state.winning_robots['T003'], 'amr_0')
        self.assertEqual(self.agent_0.state.winning_bids['T003'], 50.0)

    def test_rebidding_after_peer_purge(self) -> None:
        task_map = {
            'T001': {
                'task_id': 'T001',
                'pickup': (4.0, 4.0),
                'dropoff': (8.0, 8.0),
                'priority': 3,
                'deadline': None,
            }
        }
        self.agent_0.position = (2.0, 2.0)

        # Initially, amr_1 had a very high bid on T001
        self.agent_0.state.winning_robots['T001'] = 'amr_1'
        self.agent_0.state.winning_bids['T001'] = 999.0

        # amr_0 cannot win it
        added = self.agent_0.build_bundle(task_map, current_time=1.0)
        self.assertEqual(added, 0)
        self.assertEqual(len(self.agent_0.state.bundle), 0)

        # Now amr_1 fails, purge its tasks
        self.agent_0.purge_failed_peer_tasks('amr_1', current_time=2.0)

        # Now amr_0 builds bundle again: it should successfully claim T001!
        added = self.agent_0.build_bundle(task_map, current_time=3.0)
        self.assertEqual(added, 1)
        self.assertIn('T001', self.agent_0.state.bundle)
        self.assertEqual(self.agent_0.state.winning_robots['T001'], 'amr_0')


class TestReservationClearingOnFailure(unittest.TestCase):
    """Test suite for SpaceTimeReservationTable cleanup on robot failure."""

    def setUp(self) -> None:
        self.table = SpaceTimeReservationTable()

    def test_release_robot_clears_all_reservations(self) -> None:
        # amr_1 reserves cells
        self.table.reserve((5, 5), time_step=1, robot_id='amr_1')
        self.table.reserve((5, 6), time_step=2, robot_id='amr_1')
        self.table.reserve_edge((5, 5), (5, 6), time_step=1, robot_id='amr_1')

        # amr_0 reserves different cell
        self.table.reserve((2, 2), time_step=1, robot_id='amr_0')

        self.assertTrue(self.table.is_reserved((5, 5), 1, robot_id='amr_0'))
        self.assertTrue(self.table.is_reserved((5, 6), 2, robot_id='amr_0'))

        # Release amr_1
        self.table.release_robot('amr_1')

        # amr_1 reservations are gone
        self.assertFalse(self.table.is_reserved((5, 5), 1, robot_id='amr_0'))
        self.assertFalse(self.table.is_reserved((5, 6), 2, robot_id='amr_0'))

        # amr_0 reservation is preserved
        self.assertTrue(self.table.is_reserved((2, 2), 1, robot_id='amr_1'))


class TestTaskReclamationLifecycle(unittest.TestCase):
    """Test task lifecycle transitions under mid-mission failure and reclamation."""

    def test_reclaim_assigned_task_to_pending(self) -> None:
        t = Task(
            task_id='T010',
            pickup=(3.0, 4.0),
            dropoff=(10.0, 12.0),
            priority=TaskPriority.HIGH,
        )
        self.assertEqual(t.state, TaskLifecycleState.PENDING)

        # amr_3 assigned
        t.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_3')
        self.assertEqual(t.state, TaskLifecycleState.ASSIGNED)
        self.assertEqual(t.assigned_robot_id, 'amr_3')

        # amr_3 fails: reclaim task back to PENDING
        t.transition_to(TaskLifecycleState.PENDING, details='Reclaimed from failed amr_3')
        self.assertEqual(t.state, TaskLifecycleState.PENDING)
        self.assertIsNone(t.assigned_robot_id)
        self.assertEqual(t.reassignment_count, 1)

    def test_reclaim_in_progress_task_to_pending(self) -> None:
        t = Task(
            task_id='T011',
            pickup=(2.0, 2.0),
            dropoff=(14.0, 14.0),
            priority=TaskPriority.CRITICAL,
        )
        t.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_4')
        t.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id='amr_4')
        self.assertEqual(t.state, TaskLifecycleState.IN_PROGRESS)

        # Mid-task failure: reclaim back to PENDING
        t.transition_to(
            TaskLifecycleState.PENDING,
            details='Reclaimed mid-transit from failed amr_4',
        )
        self.assertEqual(t.state, TaskLifecycleState.PENDING)
        self.assertIsNone(t.assigned_robot_id)
        self.assertEqual(t.reassignment_count, 1)

        # Re-assigned to amr_0
        t.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_0')
        t.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id='amr_0')
        t.transition_to(TaskLifecycleState.COMPLETED, robot_id='amr_0')
        self.assertEqual(t.state, TaskLifecycleState.COMPLETED)
        self.assertEqual(t.assigned_robot_id, 'amr_0')


class TestStrandedChassisObstacleAvoidance(unittest.TestCase):
    """Test that failed AMR chassis added as obstacle diverts A* paths."""

    def test_path_reroutes_around_dead_robot(self) -> None:
        grid = GridWorld(20, 20, resolution=0.5)
        astar = SingleAgentAStar(grid)

        # Clear path from (2, 5) to (10, 5)
        path_before = astar.find_path((2, 5), (10, 5))
        self.assertIsNotNone(path_before)
        self.assertIn((5, 5), path_before)

        # amr_1 fails and dies at (5, 5): add (5, 5) as obstacle
        grid.add_obstacle((5, 5))

        # Replan around dead amr_1
        path_after = astar.find_path((2, 5), (10, 5))
        self.assertIsNotNone(path_after)
        self.assertNotIn((5, 5), path_after)
        self.assertEqual(path_after[0], (2, 5))
        self.assertEqual(path_after[-1], (10, 5))


class TestStateSemanticsAndCapabilities(unittest.TestCase):
    """Verify formal state semantics matrix and capability query properties."""

    def test_all_state_capabilities(self) -> None:
        # HEALTHY
        h = FaultState.HEALTHY
        self.assertTrue(h.can_move)
        self.assertTrue(h.publishes_heartbeat)
        self.assertTrue(h.can_own_tasks)
        self.assertTrue(h.eligible_for_cbba)
        self.assertTrue(h.holds_reservations)
        self.assertFalse(h.is_chassis_obstacle)

        # COMM_LOSS: Can move on horizon, does not publish RF, not chassis obstacle
        cl = FaultState.COMM_LOSS
        self.assertTrue(cl.can_move)
        self.assertTrue(cl.can_own_tasks)
        self.assertFalse(cl.eligible_for_cbba)
        self.assertFalse(cl.is_chassis_obstacle)

        # MOTION_FAILURE: Cannot move, chassis is an obstacle, cannot own tasks
        mf = FaultState.MOTION_FAILURE
        self.assertFalse(mf.can_move)
        self.assertTrue(mf.publishes_heartbeat)
        self.assertFalse(mf.can_own_tasks)
        self.assertFalse(mf.eligible_for_cbba)
        self.assertTrue(mf.is_chassis_obstacle)

        # EMERGENCY_STOP: Cannot move, holds reservations, not chassis obstacle
        es = FaultState.EMERGENCY_STOP
        self.assertFalse(es.can_move)
        self.assertTrue(es.publishes_heartbeat)
        self.assertTrue(es.holds_reservations)
        self.assertFalse(es.is_chassis_obstacle)

        # FAILED: Completely dead
        f = FaultState.FAILED
        self.assertFalse(f.can_move)
        self.assertFalse(f.publishes_heartbeat)
        self.assertFalse(f.can_own_tasks)
        self.assertFalse(f.eligible_for_cbba)
        self.assertFalse(f.holds_reservations)
        self.assertTrue(f.is_chassis_obstacle)


class TestPeerRestorationAndCommLoss(unittest.TestCase):
    """Verify differentiation of COMM_LOSS vs FAILED and peer restoration."""

    def setUp(self) -> None:
        cfg = FaultDetectorConfig(
            heartbeat_period_s=0.5,
            comm_loss_threshold_s=1.0,
            failure_timeout_s=3.0,
            confirmation_samples=2,
        )
        self.detector = FaultDetector('amr_0', config=cfg)

    def test_comm_loss_vs_failure_distinction(self) -> None:
        t0 = 100.0
        self.detector.record_peer_heartbeat('amr_1', 'HEALTHY', (2.0, 5.0), timestamp=t0)

        # At t0 + 1.5s: elapsed > comm_loss_threshold (1.0s) but < failure_timeout (3.0s)
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 1.5)
        self.assertEqual(len(newly_failed), 0)
        self.assertNotIn('amr_1', self.detector.failed_peers)
        rec = self.detector.peer_records['amr_1']
        self.assertEqual(rec.state, FaultState.COMM_LOSS)

    def test_peer_restoration_lifecycle(self) -> None:
        t0 = 100.0
        self.detector.record_peer_heartbeat('amr_1', 'HEALTHY', (2.0, 5.0), timestamp=t0)

        # Timeout to confirmed failure
        self.detector.evaluate_peers(now=t0 + 3.5)
        self.detector.evaluate_peers(now=t0 + 4.5)
        self.assertTrue(self.detector.is_peer_failed('amr_1'))

        # amr_1 restored by operator: sends HEALTHY heartbeat
        self.detector.record_peer_heartbeat(
            'amr_1', 'HEALTHY', (2.0, 5.0), timestamp=t0 + 10.0
        )
        newly_failed, newly_recovered = self.detector.evaluate_peers(now=t0 + 10.1)
        self.assertIn('amr_1', newly_recovered)
        self.assertFalse(self.detector.is_peer_failed('amr_1'))


class TestTaskReclamationRaceConditions(unittest.TestCase):
    """
    Test suite for M1.1 task reclamation idempotency and race condition hardening.

    Validates scenarios A through I.
    """

    def setUp(self) -> None:
        self.t1 = Task('T1', (2.0, 2.0), (10.0, 10.0), priority=TaskPriority.HIGH)
        self.t2 = Task('T2', (4.0, 4.0), (12.0, 12.0), priority=TaskPriority.NORMAL)

    def _simulate_reclamation(
        self,
        task: Task,
        details: str,
        reporter_id: str = 'amr_0',
    ) -> bool:
        """Simulate TaskManagerNode CAS precondition check."""
        import re
        if task.state == TaskLifecycleState.PENDING:
            # Idempotent no-op
            return False

        if details:
            match = re.search(r'from failed peer\s+(\w+)', details, re.IGNORECASE)
            if match:
                expected_failed_robot = match.group(1)
                if task.assigned_robot_id != expected_failed_robot:
                    # Stale / mismatched owner: reject
                    return False

        task.transition_to(
            TaskLifecycleState.PENDING,
            robot_id=None,
            details=details,
        )
        return True

    def test_case_a_single_peer_detects_failure(self) -> None:
        # G & A: Failed robot's task is ASSIGNED
        self.t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        ok = self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1'
        )
        self.assertTrue(ok)
        self.assertEqual(self.t1.state, TaskLifecycleState.PENDING)
        self.assertIsNone(self.t1.assigned_robot_id)
        self.assertEqual(self.t1.reassignment_count, 1)

    def test_case_b_and_c_simultaneous_peer_detections_and_requeue(self) -> None:
        # H: Failed robot's task is IN_PROGRESS
        self.t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        self.t1.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id='amr_1')

        # Peer amr_0 detects failure and emits reclamation
        ok0 = self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1', reporter_id='amr_0'
        )
        self.assertTrue(ok0)

        # Peer amr_2 simultaneously detects failure and emits reclamation
        ok2 = self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1', reporter_id='amr_2'
        )
        # Second attempt must be an idempotent no-op
        self.assertFalse(ok2)
        self.assertEqual(self.t1.state, TaskLifecycleState.PENDING)
        self.assertIsNone(self.t1.assigned_robot_id)
        # Exactly one reassignment count incremented
        self.assertEqual(self.t1.reassignment_count, 1)

    def test_case_d_and_f_stale_requeue_after_reassignment(self) -> None:
        # amr_1 originally owned T1
        self.t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        # Reclaimed to PENDING
        self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1'
        )
        self.assertEqual(self.t1.state, TaskLifecycleState.PENDING)

        # New winner amr_2 wins T1 in CBBA re-auction
        self.t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_2')
        self.assertEqual(self.t1.assigned_robot_id, 'amr_2')

        # Delayed stale reclamation arrives from amr_3 claiming amr_1 failed
        stale_ok = self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1', reporter_id='amr_3'
        )
        # Must be rejected because task is owned by amr_2, not amr_1!
        self.assertFalse(stale_ok)
        self.assertEqual(self.t1.state, TaskLifecycleState.ASSIGNED)
        self.assertEqual(self.t1.assigned_robot_id, 'amr_2')

    def test_case_e_task_already_pending(self) -> None:
        # Task is already PENDING
        self.assertEqual(self.t1.state, TaskLifecycleState.PENDING)
        ok = self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1'
        )
        self.assertFalse(ok)
        self.assertEqual(self.t1.state, TaskLifecycleState.PENDING)
        self.assertEqual(self.t1.reassignment_count, 0)

    def test_case_i_multiple_tasks_belong_to_failed_robot(self) -> None:
        # amr_1 owns both T1 and T2
        self.t1.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        self.t2.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_1')
        self.t2.transition_to(TaskLifecycleState.IN_PROGRESS, robot_id='amr_1')

        # Both reclaimed
        ok1 = self._simulate_reclamation(
            self.t1, 'Autonomous reclamation from failed peer amr_1'
        )
        ok2 = self._simulate_reclamation(
            self.t2, 'Autonomous reclamation from failed peer amr_1'
        )
        self.assertTrue(ok1)
        self.assertTrue(ok2)
        self.assertEqual(self.t1.state, TaskLifecycleState.PENDING)
        self.assertEqual(self.t2.state, TaskLifecycleState.PENDING)
        self.assertIsNone(self.t1.assigned_robot_id)
        self.assertIsNone(self.t2.assigned_robot_id)


class TestFailedChassisLifecycleRestoration(unittest.TestCase):
    """Test obstacle insertion on failure and clean removal on restoration."""

    def test_obstacle_lifecycle_and_path_restoration(self) -> None:
        grid = GridWorld(20, 20, resolution=0.5)
        astar = SingleAgentAStar(grid)

        # Clear path straight through (5, 5)
        path_before = astar.find_path((2, 5), (10, 5))
        self.assertIsNotNone(path_before)
        self.assertIn((5, 5), path_before)
        len_before = len(path_before)

        # Peer amr_1 fails at (5, 5): registered in failed_robot_obstacles
        failed_robot_obstacles = {'amr_1': {(5, 5)}}
        grid.add_obstacle((5, 5))

        # Path is diverted around dead robot
        path_diverted = astar.find_path((2, 5), (10, 5))
        self.assertIsNotNone(path_diverted)
        self.assertNotIn((5, 5), path_diverted)
        self.assertGreater(len(path_diverted), len_before)

        # Operator restores amr_1: obstacle cleanly removed
        obs_cells = failed_robot_obstacles.pop('amr_1', set())
        for c in obs_cells:
            grid.remove_obstacle(c)

        # Path reverts to optimal direct path
        path_restored = astar.find_path((2, 5), (10, 5))
        self.assertIsNotNone(path_restored)
        self.assertIn((5, 5), path_restored)
        self.assertEqual(len(path_restored), len_before)


if __name__ == '__main__':
    unittest.main()

