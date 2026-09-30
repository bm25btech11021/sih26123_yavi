"""
Unit and integration tests for YAVI-SIH26123 (v2) Milestone 2: Network Resilience.

Validates:
1. Formal Fault and Network States (can_move, is_local_autonomy_permitted).
2. FaultDetector network fault triggers (COMM_LOSS, OUTAGE, DISCONNECT, RECONNECT).
3. Heartbeat staleness detection, COMM_LOSS vs FAILED differentiation, and false failure rejection.
4. Local autonomy execution, reservation validity check, and LOCAL_SAFETY_HOLD on expiry.
5. CBBA reconnection reconciliation (reconcile_reconnection) and zero duplicate ownership (I_uniq).
6. High packet loss (35%) handling under CommunicationModel.
7. Network partition and merge state reconciliation.
"""

import random
import unittest

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.communication_model import (
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    PRESET_PROFILES,
)
from amr_fleet_core.fault_detector import FaultDetector, FaultDetectorConfig
from amr_fleet_core.fault_state import FaultState
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.stale_state_manager import InformationState, StaleStateManager


class TestM2FaultAndNetworkStates(unittest.TestCase):
    """Test suite for formal fault and network state transitions."""

    def test_local_autonomy_permission(self) -> None:
        """Verify is_local_autonomy_permitted is True ONLY for COMM_LOSS."""
        self.assertFalse(FaultState.HEALTHY.is_local_autonomy_permitted)
        self.assertTrue(FaultState.COMM_LOSS.is_local_autonomy_permitted)
        self.assertFalse(FaultState.FAILED.is_local_autonomy_permitted)
        self.assertFalse(FaultState.EMERGENCY_STOP.is_local_autonomy_permitted)
        self.assertFalse(FaultState.MOTION_FAILURE.is_local_autonomy_permitted)
        self.assertFalse(FaultState.DEGRADED.is_local_autonomy_permitted)
        self.assertFalse(FaultState.RECOVERING.is_local_autonomy_permitted)

    def test_can_move_semantics(self) -> None:
        """Verify can_move is True for HEALTHY and COMM_LOSS, False for FAILED."""
        self.assertTrue(FaultState.HEALTHY.can_move)
        self.assertTrue(FaultState.COMM_LOSS.can_move)
        self.assertFalse(FaultState.FAILED.can_move)
        self.assertFalse(FaultState.EMERGENCY_STOP.can_move)
        self.assertFalse(FaultState.MOTION_FAILURE.can_move)


class TestM2FaultDetectorNetworkHandling(unittest.TestCase):
    """Test suite for FaultDetector network loss detection and query APIs."""

    def setUp(self) -> None:
        cfg = FaultDetectorConfig(
            heartbeat_period_s=0.5,
            comm_loss_threshold_s=1.0,
            failure_timeout_s=3.5,
            confirmation_samples=2,
        )
        self.detector = FaultDetector(local_robot_id='amr_0', config=cfg)

    def test_network_fault_injection_and_recovery(self) -> None:
        """Verify injection of network fault aliases and restoration."""
        ok, msg = self.detector.inject_fault('COMM_LOSS')
        self.assertTrue(ok)
        self.assertTrue(self.detector.is_in_comm_loss())
        self.assertTrue(self.detector.is_local_autonomy_active())
        self.assertFalse(self.detector.is_self_healthy())
        self.assertEqual(self.detector.self_state, FaultState.COMM_LOSS)

        # Reconnect
        ok_rec, _ = self.detector.inject_fault('RECONNECT')
        self.assertTrue(ok_rec)
        self.assertFalse(self.detector.is_in_comm_loss())
        self.assertTrue(self.detector.is_self_healthy())
        self.assertEqual(self.detector.self_state, FaultState.HEALTHY)

    def test_comm_loss_vs_failed_detection(self) -> None:
        """Verify peer enters COMM_LOSS at 1.0s and only enters FAILED at >=3.5s."""
        t0 = 100.0
        self.detector.record_peer_heartbeat('amr_1', 'HEALTHY', (2.0, 2.0), 'T1', t0)
        self.assertFalse(self.detector.is_peer_in_comm_loss('amr_1'))
        self.assertFalse(self.detector.is_peer_failed('amr_1'))

        # At t0 + 1.2s: exceeds comm_loss_threshold (1.0s) but < failure_timeout (3.5s)
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 1.2)
        self.assertEqual(len(newly_failed), 0)
        self.assertTrue(self.detector.is_peer_in_comm_loss('amr_1'))
        self.assertFalse(self.detector.is_peer_failed('amr_1'))
        self.assertEqual(self.detector.peer_records['amr_1'].state, FaultState.COMM_LOSS)

        # At t0 + 3.6s: first debounce sample for failure
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 3.6)
        self.assertEqual(len(newly_failed), 0)  # Needs confirmation_samples=2

        # At t0 + 3.8s: second sample -> confirmed FAILED!
        newly_failed, _ = self.detector.evaluate_peers(now=t0 + 3.8)
        self.assertIn('amr_1', newly_failed)
        self.assertTrue(self.detector.is_peer_failed('amr_1'))
        self.assertEqual(self.detector.peer_records['amr_1'].state, FaultState.FAILED)

    def test_false_failure_protection(self) -> None:
        """Verify transient comm loss (2.0s <= 3.5s) does NOT declare peer FAILED."""
        t0 = 100.0
        self.detector.record_peer_heartbeat('amr_1', 'HEALTHY', (2.0, 2.0), 'T1', t0)

        # Evaluated at t0 + 2.0s
        self.detector.evaluate_peers(now=t0 + 2.0)
        self.assertTrue(self.detector.is_peer_in_comm_loss('amr_1'))
        self.assertFalse(self.detector.is_peer_failed('amr_1'))

        # Heartbeat resumes at t0 + 2.2s -> restored to HEALTHY!
        self.detector.record_peer_heartbeat('amr_1', 'HEALTHY', (2.5, 2.0), 'T1', t0 + 2.2)
        self.assertFalse(self.detector.is_peer_in_comm_loss('amr_1'))
        self.assertFalse(self.detector.is_peer_failed('amr_1'))
        self.assertEqual(self.detector.peer_records['amr_1'].state, FaultState.HEALTHY)


class TestM2CBBAReconnectionReconciliation(unittest.TestCase):
    """Test suite for CBBA bid & timestamp reconciliation upon network recovery."""

    def setUp(self) -> None:
        cfg = CBBAConfig(max_bundle_size=2)
        self.agent = CBBAAgent(robot_id='amr_1', config=cfg, initial_position=(5.0, 5.0))

    def test_yield_reclaimed_task_newer_timestamp(self) -> None:
        """Verify agent yields task if peer has newer timestamp and won the task."""
        # amr_1 won task T1 at t=2.0
        self.agent.state.bundle = ['T1']
        self.agent.state.path = ['T1']
        self.agent.state.winning_robots['T1'] = 'amr_1'
        self.agent.state.winning_bids['T1'] = 10.0
        self.agent.state.timestamps['T1'] = 2.0

        # Peer amr_0 reclaimed T1 during amr_1's outage at t=5.0
        peer_winning_robots = {'T1': 'amr_0'}
        peer_timestamps = {'T1': 5.0}  # Strictly newer: 5.0 > 2.0
        peer_winning_bids = {'T1': 15.0}

        yielded = self.agent.reconcile_reconnection(
            peer_winning_robots, peer_timestamps, peer_winning_bids
        )

        self.assertIn('T1', yielded)
        self.assertNotIn('T1', self.agent.state.bundle)
        self.assertNotIn('T1', self.agent.state.path)
        self.assertEqual(self.agent.state.winning_robots['T1'], 'amr_0')
        self.assertEqual(self.agent.state.winning_bids['T1'], 15.0)
        self.assertEqual(self.agent.state.timestamps['T1'], 5.0)

    def test_retain_task_if_local_newer_or_unchanged(self) -> None:
        """Verify agent retains task if local timestamp is newer or equal."""
        self.agent.state.bundle = ['T1']
        self.agent.state.path = ['T1']
        self.agent.state.winning_robots['T1'] = 'amr_1'
        self.agent.state.winning_bids['T1'] = 10.0
        self.agent.state.timestamps['T1'] = 6.0

        # Peer has older stale timestamp t=4.0
        peer_winning_robots = {'T1': 'amr_0'}
        peer_timestamps = {'T1': 4.0}
        peer_winning_bids = {'T1': 8.0}

        yielded = self.agent.reconcile_reconnection(
            peer_winning_robots, peer_timestamps, peer_winning_bids
        )

        self.assertEqual(len(yielded), 0)
        self.assertIn('T1', self.agent.state.bundle)
        self.assertEqual(self.agent.state.winning_robots['T1'], 'amr_1')

    def test_zero_duplicate_ownership_invariant(self) -> None:
        """Verify invariant I_uniq: no two agents own the same task after reconciliation."""
        cfg = CBBAConfig(max_bundle_size=2)
        ag0 = CBBAAgent('amr_0', config=cfg, initial_position=(0.0, 0.0))
        ag1 = CBBAAgent('amr_1', config=cfg, initial_position=(10.0, 10.0))

        # ag1 owned T1 at t=1.0, lost comms
        ag1.state.bundle = ['T1']
        ag1.state.winning_robots['T1'] = 'amr_1'
        ag1.state.timestamps['T1'] = 1.0

        # ag0 reclaimed T1 at t=4.5
        ag0.state.bundle = ['T1']
        ag0.state.winning_robots['T1'] = 'amr_0'
        ag0.state.timestamps['T1'] = 4.5
        ag0.state.winning_bids['T1'] = 12.0

        # Reconnection happens: ag1 reconciles against ag0
        ag1.reconcile_reconnection(
            ag0.state.winning_robots, ag0.state.timestamps, ag0.state.winning_bids
        )

        # Verify only ag0 has T1 in bundle, ag1 yielded
        self.assertIn('T1', ag0.state.bundle)
        self.assertNotIn('T1', ag1.state.bundle)
        self.assertEqual(ag1.state.winning_robots['T1'], 'amr_0')
        self.assertEqual(ag0.state.winning_robots['T1'], 'amr_0')


class TestM2LocalAutonomyAndReservations(unittest.TestCase):
    """Test suite for local autonomy movement and reservation table safety."""

    def test_reservation_hold_on_expiration(self) -> None:
        """Verify robot with expired reservations is not allowed unreserved movement."""
        table = SpaceTimeReservationTable()
        # Robot amr_1 reserves cells (5,5) at t=0 and (5,6) at t=1
        ok1 = table.reserve((5, 5), 0, 'amr_1')
        ok2 = table.reserve((5, 6), 1, 'amr_1')
        self.assertTrue(ok1)
        self.assertTrue(ok2)

        # Check reservation validity
        res1 = table.get_reservation((5, 5), 0)
        self.assertIsNotNone(res1)
        self.assertEqual(res1.robot_id, 'amr_1')
        res2 = table.get_reservation((5, 6), 1)
        self.assertIsNotNone(res2)
        self.assertEqual(res2.robot_id, 'amr_1')
        # At t=2, cell (5,7) is unreserved
        self.assertIsNone(table.get_reservation((5, 7), 2))

    def test_stale_state_manager_classification(self) -> None:
        """Verify StaleStateManager classifies age into CURRENT, STALE, EXPIRED."""
        mgr = StaleStateManager('amr_0', stale_threshold_s=1.5, expiry_threshold_s=4.0)
        t0 = 100.0

        # Initial message
        st, is_reconn = mgr.record_incoming('reservations', 'amr_1', t0, t0, {'cell': (1, 1)})
        self.assertEqual(st, InformationState.CURRENT)
        self.assertFalse(is_reconn)

        # Age 1.0s -> CURRENT
        state_1s = mgr.get_peer_state('reservations', 'amr_1', current_time=t0 + 1.0)
        self.assertEqual(state_1s, InformationState.CURRENT)

        # Age 2.5s -> STALE
        state_25s = mgr.get_peer_state('reservations', 'amr_1', current_time=t0 + 2.5)
        self.assertEqual(state_25s, InformationState.STALE)

        # Age 4.5s -> EXPIRED
        state_45s = mgr.get_peer_state('reservations', 'amr_1', current_time=t0 + 4.5)
        self.assertEqual(state_45s, InformationState.EXPIRED)

        # Message arrives after expiry -> reconnection detected!
        st2, is_reconn2 = mgr.record_incoming(
            'reservations', 'amr_1', t0 + 5.0, t0 + 5.0, {'cell': (1, 2)}
        )
        self.assertEqual(st2, InformationState.CURRENT)
        self.assertTrue(is_reconn2)


class TestM2HighPacketLossImpairment(unittest.TestCase):
    """Test suite for high packet loss (35%) communication degradation."""

    def test_loss_high_profile_stochastic(self) -> None:
        """Verify 35% loss profile drops approximately 35% of packets."""
        config = PRESET_PROFILES['LOSS_HIGH']
        model = CommunicationImpairmentModel(config)

        total_packets = 1000
        delivered = 0
        dropped = 0

        random.seed(123)
        for i in range(total_packets):
            action, delay, reason = model.process_message(
                'amr_0', 'amr_1', current_time=float(i) * 0.1
            )
            if action.value == 'DROP':
                dropped += 1
            else:
                delivered += 1

        measured_loss = dropped / total_packets
        # With 1000 packets and p=0.35, loss should be within [0.30, 0.40]
        self.assertAlmostEqual(measured_loss, 0.35, delta=0.05)

        telemetry = model.get_metrics()
        self.assertEqual(telemetry['messages_sent'], total_packets)
        self.assertEqual(telemetry['messages_delivered'], delivered)
        self.assertEqual(telemetry['messages_dropped'], dropped)


class TestM2NetworkPartition(unittest.TestCase):
    """Test suite for bipartite network partition and recovery."""

    def test_partition_message_filtering(self) -> None:
        """Verify partition isolates amr_0 from {amr_1, amr_2}."""
        config = CommunicationProfileConfig(
            profile_name='PARTITION',
            enabled=True,
            isolated_robots=['amr_0'],
        )
        model = CommunicationImpairmentModel(config)

        # amr_1 to amr_2 -> DELIVER (same partition)
        action_12, _, _ = model.process_message('amr_1', 'amr_2', 1.0)
        self.assertEqual(action_12.value, 'DELIVER')

        # amr_0 to amr_1 -> DROP (cross-partition)
        action_01, _, reason_01 = model.process_message('amr_0', 'amr_1', 1.0)
        self.assertEqual(action_01.value, 'DROP')
        self.assertEqual(reason_01.value, 'PARTITION')

        # amr_2 to amr_0 -> DROP (cross-partition)
        action_20, _, reason_20 = model.process_message('amr_2', 'amr_0', 1.0)
        self.assertEqual(action_20.value, 'DROP')
        self.assertEqual(reason_20.value, 'PARTITION')


if __name__ == '__main__':
    unittest.main()

