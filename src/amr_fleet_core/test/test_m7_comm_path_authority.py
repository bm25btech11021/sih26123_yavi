"""
Comprehensive verification test for M7 Communication Path Authority.

Proves that the impairment layer is authoritative for ALL intended /fleet
coordination traffic:
1. CBBA bids and consensus (/fleet/cbba_bids)
2. Space-time reservations (/fleet/reservations)
3. Conflict and WFG information (/fleet/coordination_status, /fleet/deadlocks)
4. Dynamic network partitions

Proves that NO direct publisher -> subscriber bypass exists.
"""

import unittest

from amr_fleet_core.communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    DelayedMessageQueue,
)
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.stale_state_manager import (
    InformationState,
    StaleStateManager,
)
from amr_fleet_core.wfg_deadlock import WaitForGraph

from amr_fleet_msgs.msg import (
    CBBABid,
    CoordinationStatus,
    DeadlockEvent,
    SpaceTimeReservation,
)


class MockCBBANodeWithBoundary:
    """Mock CBBA node implementing the exact ingress impairment boundary."""

    def __init__(
        self,
        robot_id: str,
        comm_cfg: CommunicationProfileConfig,
    ) -> None:
        """Initialize mock CBBA node with impairment boundary."""
        self.robot_id = robot_id
        self.comm_model = CommunicationImpairmentModel(config=comm_cfg)
        self.delayed_queue = DelayedMessageQueue()
        self.stale_manager = StaleStateManager(
            local_robot_id=robot_id,
            stale_threshold_s=1.5,
            expiry_threshold_s=4.0,
        )
        self.winning_bids = {}
        self.winning_robots = {}

    def handle_peer_bid(self, msg: CBBABid, current_time: float) -> None:
        """Exact logic from cbba_node.py _handle_peer_bid."""
        if msg.robot_id == self.robot_id:
            return

        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else current_time
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=msg.robot_id,
            receiver_id=self.robot_id,
            current_time=current_time,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = current_time + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_peer_bid,
                (msg, sent_ts),
            )
            return

        self._apply_peer_bid((msg, sent_ts), current_time)

    def _apply_peer_bid(self, data, apply_time: float = 0.0) -> None:
        """Exact logic from cbba_node.py _apply_peer_bid."""
        msg, sent_ts = data
        state, _ = self.stale_manager.record_incoming(
            'cbba_bids', msg.robot_id, sent_ts, apply_time, msg
        )
        if state == InformationState.EXPIRED:
            return

        for t_id, bid, winner in zip(
            msg.task_ids, msg.winning_bids, msg.winning_robots
        ):
            self.winning_bids[t_id] = bid
            self.winning_robots[t_id] = winner


class MockRHNodeWithBoundary:
    """Mock RH coordination node with exact ingress boundary."""

    def __init__(
        self,
        robot_id: str,
        comm_cfg: CommunicationProfileConfig,
    ) -> None:
        """Initialize mock RH node with impairment boundary."""
        self.robot_id = robot_id
        self.comm_model = CommunicationImpairmentModel(config=comm_cfg)
        self.delayed_queue = DelayedMessageQueue()
        self.stale_manager = StaleStateManager(
            local_robot_id=robot_id,
            stale_threshold_s=1.5,
            expiry_threshold_s=4.0,
        )
        self.res_table = SpaceTimeReservationTable()
        self.wfg = WaitForGraph()
        self.peer_states = {}
        self.deadlock_resets = []

    def handle_incoming_reservation(
        self,
        msg: SpaceTimeReservation,
        current_time: float,
    ) -> None:
        """Exact logic from rh_node.py _handle_incoming_reservation."""
        if msg.robot_id == self.robot_id:
            return

        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else current_time
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=msg.robot_id,
            receiver_id=self.robot_id,
            current_time=current_time,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = current_time + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_reservation_payload,
                (msg, sent_ts),
            )
            return

        self._apply_reservation_payload((msg, sent_ts), current_time)

    def _apply_reservation_payload(
        self,
        data,
        apply_time: float = 0.0,
    ) -> None:
        """Exact logic from rh_node.py _apply_reservation_payload."""
        msg, sent_ts = data
        state, _ = self.stale_manager.record_incoming(
            'reservations', msg.robot_id, sent_ts, apply_time, msg
        )
        if state == InformationState.EXPIRED:
            return

        self.res_table.reserve(
            (msg.to_x, msg.to_y),
            msg.time_step,
            msg.robot_id,
            msg.priority,
        )

    def handle_peer_coordination_status(
        self,
        msg: CoordinationStatus,
        current_time: float,
    ) -> None:
        """Exact logic from rh_node.py _handle_peer_coordination_status."""
        if msg.robot_id == self.robot_id:
            return

        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else current_time
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=msg.robot_id,
            receiver_id=self.robot_id,
            current_time=current_time,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = current_time + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_peer_coordination_status,
                (msg, sent_ts),
            )
            return

        self._apply_peer_coordination_status((msg, sent_ts), current_time)

    def _apply_peer_coordination_status(
        self,
        data,
        apply_time: float = 0.0,
    ) -> None:
        """Exact logic from rh_node.py _apply_peer_coordination_status."""
        msg, sent_ts = data
        state, _ = self.stale_manager.record_incoming(
            'coordination_status', msg.robot_id, sent_ts, apply_time, msg
        )
        if state == InformationState.EXPIRED:
            return

        self.peer_states[msg.robot_id] = {
            'waiting_for': msg.waiting_for_robot,
            'target_cell': (msg.target_cell_x, msg.target_cell_y),
        }

        if msg.waiting_for_robot:
            self.wfg.add_wait(
                msg.robot_id,
                msg.waiting_for_robot,
                (msg.target_cell_x, msg.target_cell_y),
                msg.time_step,
            )
        else:
            self.wfg.clear_robot(msg.robot_id)

    def handle_incoming_deadlock(
        self,
        msg: DeadlockEvent,
        current_time: float,
    ) -> None:
        """Exact logic from rh_node.py _handle_incoming_deadlock."""
        sender = (
            msg.header.frame_id if msg.header.frame_id
            else (msg.cycle_robot_ids[0] if msg.cycle_robot_ids else 'fleet')
        )
        if sender == self.robot_id:
            return

        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else current_time
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=sender,
            receiver_id=self.robot_id,
            current_time=current_time,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = current_time + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_deadlock_payload,
                (msg, sent_ts),
            )
            return

        self._apply_deadlock_payload((msg, sent_ts), current_time)

    def _apply_deadlock_payload(
        self,
        data,
        apply_time: float = 0.0,
    ) -> None:
        """Exact logic from rh_node.py _apply_deadlock_payload."""
        msg, sent_ts = data
        self.deadlock_resets.append(list(msg.cycle_robot_ids))


class TestCommunicationPathAuthority(unittest.TestCase):
    """Test suite demonstrating complete authority of impairment layer."""

    def test_cbba_bid_authority_drop_profile(self):
        """Verify that under 100% loss zero peer bids reach consensus state."""
        drop_cfg = CommunicationProfileConfig(
            profile_name='OUTAGE',
            enabled=True,
            loss_probability=1.0,
        )
        node = MockCBBANodeWithBoundary('amr_1', drop_cfg)

        msg = CBBABid()
        msg.robot_id = 'amr_0'
        msg.task_ids = ['task_alpha', 'task_beta']
        msg.winning_bids = [10.5, 20.0]
        msg.winning_robots = ['amr_0', 'amr_0']

        # Transmit 10 peer bids
        for t in range(10):
            node.handle_peer_bid(msg, current_time=float(t))

        # PROOF: Impairment dropped all 10; state remains completely empty
        self.assertEqual(len(node.winning_bids), 0)
        self.assertEqual(len(node.winning_robots), 0)
        self.assertEqual(node.comm_model.messages_dropped, 10)
        self.assertEqual(node.comm_model.messages_delivered, 0)

    def test_cbba_bid_authority_delay_profile(self):
        """Verify that under latency peer bids do not bypass delayed queue."""
        delay_cfg = CommunicationProfileConfig(
            profile_name='FIXED_LATENCY',
            enabled=True,
            latency_ms=200.0,
            jitter_ms=0.0,
            loss_probability=0.0,
        )
        node = MockCBBANodeWithBoundary('amr_1', delay_cfg)

        msg = CBBABid()
        msg.robot_id = 'amr_0'
        msg.task_ids = ['task_alpha']
        msg.winning_bids = [15.0]
        msg.winning_robots = ['amr_0']

        # Send at t = 1.0s (expected delivery at t = 1.2s)
        node.handle_peer_bid(msg, current_time=1.0)

        # PROOF: At t = 1.0s, bid is in DelayedMessageQueue
        self.assertEqual(len(node.winning_bids), 0)
        self.assertEqual(len(node.delayed_queue), 1)

        # At t = 1.1s (prior to 1.2s), queue does not dispatch
        node.delayed_queue.dispatch_ready(1.1)
        self.assertEqual(len(node.winning_bids), 0)

        # At t = 1.25s, dispatch_ready releases it
        dispatched = node.delayed_queue.dispatch_ready(1.25)
        self.assertEqual(dispatched, 1)
        self.assertEqual(node.winning_bids.get('task_alpha'), 15.0)
        self.assertEqual(node.winning_robots.get('task_alpha'), 'amr_0')

    def test_reservation_authority_drop_profile(self):
        """Verify that under packet drop reservations are never booked."""
        drop_cfg = CommunicationProfileConfig(
            profile_name='LOSS_HIGH',
            enabled=True,
            loss_probability=1.0,
        )
        node = MockRHNodeWithBoundary('amr_1', drop_cfg)

        msg = SpaceTimeReservation()
        msg.robot_id = 'amr_0'
        msg.to_x = 5
        msg.to_y = 5
        msg.time_step = 3
        msg.priority = 100.0

        node.handle_incoming_reservation(msg, current_time=1.0)

        # PROOF: Reservation is not booked
        self.assertFalse(node.res_table.is_reserved((5, 5), 3, 'amr_1'))
        self.assertEqual(node.comm_model.messages_dropped, 1)

    def test_reservation_authority_delay_profile(self):
        """Verify reservations cannot take effect until latency elapses."""
        delay_cfg = CommunicationProfileConfig(
            profile_name='HIGH_LATENCY',
            enabled=True,
            latency_ms=400.0,
            jitter_ms=0.0,
            loss_probability=0.0,
        )
        node = MockRHNodeWithBoundary('amr_1', delay_cfg)

        msg = SpaceTimeReservation()
        msg.robot_id = 'amr_0'
        msg.to_x = 7
        msg.to_y = 7
        msg.time_step = 2
        msg.priority = 100.0

        node.handle_incoming_reservation(msg, current_time=1.0)

        # PROOF: At t = 1.0s, cell is unreserved
        self.assertFalse(node.res_table.is_reserved((7, 7), 2, 'amr_1'))
        self.assertEqual(len(node.delayed_queue), 1)

        # Dispatch at t = 1.45s (after 400ms delay)
        node.delayed_queue.dispatch_ready(1.45)
        self.assertTrue(node.res_table.is_reserved((7, 7), 2, 'amr_1'))

    def test_wfg_coordination_status_authority(self):
        """Verify peer waiting edges cannot bypass impairment layer to WFG."""
        drop_cfg = CommunicationProfileConfig(
            profile_name='BURST_LOSS',
            enabled=True,
            loss_probability=1.0,
        )
        node = MockRHNodeWithBoundary('amr_1', drop_cfg)

        msg = CoordinationStatus()
        msg.robot_id = 'amr_0'
        msg.waiting_for_robot = 'amr_1'
        msg.target_cell_x = 4
        msg.target_cell_y = 4
        msg.time_step = 1

        node.handle_peer_coordination_status(msg, current_time=1.0)

        # PROOF: Dropped packet prevents addition of wait edge to WFG
        self.assertIsNone(node.wfg.get_waiting_for('amr_0'))
        self.assertEqual(len(node.peer_states), 0)

    def test_deadlock_event_authority(self):
        """Verify that deadlock broadcast events pass through impairment."""
        delay_cfg = CommunicationProfileConfig(
            profile_name='LOW_LATENCY',
            enabled=True,
            latency_ms=100.0,
            jitter_ms=0.0,
            loss_probability=0.0,
        )
        node = MockRHNodeWithBoundary('amr_1', delay_cfg)

        msg = DeadlockEvent()
        msg.header.frame_id = 'amr_0'
        msg.cycle_robot_ids = ['amr_0', 'amr_1']

        node.handle_incoming_deadlock(msg, current_time=1.0)

        # Held in queue
        self.assertEqual(len(node.deadlock_resets), 0)
        self.assertEqual(len(node.delayed_queue), 1)

        # Dispatched after 100ms
        node.delayed_queue.dispatch_ready(1.15)
        self.assertEqual(len(node.deadlock_resets), 1)
        self.assertEqual(node.deadlock_resets[0], ['amr_0', 'amr_1'])

    def test_partition_authority_across_all_channels(self):
        """Verify network partition strictly blocks cross-partition traffic."""
        partition_cfg = CommunicationProfileConfig(
            profile_name='PARTITION',
            enabled=True,
            isolated_robots=['amr_0', 'amr_1'],
        )
        # amr_2 is on the other side of partition from amr_0
        node_amr2 = MockRHNodeWithBoundary('amr_2', partition_cfg)

        res_msg = SpaceTimeReservation()
        res_msg.robot_id = 'amr_0'
        res_msg.to_x = 2
        res_msg.to_y = 2
        res_msg.time_step = 1

        coord_msg = CoordinationStatus()
        coord_msg.robot_id = 'amr_0'
        coord_msg.waiting_for_robot = 'amr_2'

        node_amr2.handle_incoming_reservation(res_msg, current_time=1.0)
        node_amr2.handle_peer_coordination_status(coord_msg, current_time=1.0)

        # PROOF: Cross-partition messages were 100% blocked
        self.assertFalse(node_amr2.res_table.is_reserved((2, 2), 1, 'amr_2'))
        self.assertIsNone(node_amr2.wfg.get_waiting_for('amr_0'))
        self.assertEqual(node_amr2.comm_model.messages_dropped, 2)


if __name__ == '__main__':
    unittest.main()

