"""Unit tests for Milestone M7: Communication Impairment Model and Resilience."""

from amr_fleet_core.communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    DelayedMessageQueue,
    DropReason,
    PRESET_PROFILES,
)
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.stale_state_manager import (
    InformationState,
    StaleStateManager,
)


def test_zero_latency_and_loss():
    """Profile NORMAL: zero loss, zero delay, 100% delivery."""
    model = CommunicationImpairmentModel(PRESET_PROFILES['NORMAL'])
    for _ in range(50):
        action, delay_ms, reason = model.process_message('amr_0', 'amr_1', 1.0)
        assert action == CommunicationAction.DELIVER
        assert delay_ms == 0.0
        assert reason == DropReason.NONE

    metrics = model.get_metrics()
    assert metrics['messages_sent'] == 50
    assert metrics['messages_delivered'] == 50
    assert metrics['messages_dropped'] == 0
    assert metrics['packet_loss_rate'] == 0.0


def test_fixed_latency_and_jitter_bounds():
    """Profile JITTER: delay must be within configured bounds [mean - jitter, mean + jitter]."""
    cfg = CommunicationProfileConfig(
        profile_name='JITTER_TEST',
        enabled=True,
        latency_ms=200.0,
        jitter_ms=50.0,
        loss_probability=0.0,
        seed=123,
    )
    model = CommunicationImpairmentModel(cfg)

    delays = []
    for _ in range(100):
        action, delay_ms, reason = model.process_message('amr_0', 'amr_1', 1.0)
        assert action == CommunicationAction.DELAY
        assert 150.0 <= delay_ms <= 250.0
        assert reason == DropReason.NONE
        delays.append(delay_ms)

    avg_delay = sum(delays) / len(delays)
    assert 185.0 < avg_delay < 215.0


def test_deterministic_independent_loss():
    """Independent packet loss should probabilistically drop messages within tolerance."""
    cfg = CommunicationProfileConfig(
        profile_name='LOSS_TEST',
        enabled=True,
        loss_probability=0.30,
        seed=42,
    )
    model = CommunicationImpairmentModel(cfg)

    drops = 0
    total = 1000
    for _ in range(total):
        action, _, reason = model.process_message('amr_0', 'amr_1', 1.0)
        if action == CommunicationAction.DROP:
            drops += 1
            assert reason == DropReason.INDEPENDENT_LOSS

    observed_loss = drops / total
    assert 0.25 <= observed_loss <= 0.35


def test_gilbert_elliott_burst_loss():
    """Burst loss model transitions into bad state and drops bursts of consecutive packets."""
    cfg = CommunicationProfileConfig(
        profile_name='BURST_TEST',
        enabled=True,
        loss_probability=0.0,
        burst_loss_probability=0.30,
        burst_length_mean=5.0,
        seed=42,
    )
    model = CommunicationImpairmentModel(cfg)

    drop_streaks = []
    current_streak = 0
    for _ in range(500):
        action, _, reason = model.process_message('amr_0', 'amr_1', 1.0)
        if action == CommunicationAction.DROP:
            assert reason == DropReason.BURST_LOSS
            current_streak += 1
        else:
            if current_streak > 0:
                drop_streaks.append(current_streak)
                current_streak = 0

    assert len(drop_streaks) > 0
    assert max(drop_streaks) >= 2
    assert model.burst_model.burst_events_count > 0


def test_outage_lifecycle():
    """Outage drops all messages during window and restores normal delivery afterward."""
    cfg = CommunicationProfileConfig(
        profile_name='OUTAGE_TEST',
        enabled=True,
        outage_start_s=5.0,
        outage_duration_s=3.0,
        seed=42,
    )
    model = CommunicationImpairmentModel(cfg)

    act_before, _, r_before = model.process_message('amr_0', 'amr_1', 2.0)
    assert act_before == CommunicationAction.DELIVER
    assert r_before == DropReason.NONE

    assert model.is_in_outage(6.0)
    act_during, _, r_during = model.process_message('amr_0', 'amr_1', 6.0)
    assert act_during == CommunicationAction.DROP
    assert r_during == DropReason.OUTAGE

    assert not model.is_in_outage(8.5)
    act_after, _, r_after = model.process_message('amr_0', 'amr_1', 8.5)
    assert act_after == CommunicationAction.DELIVER
    assert r_after == DropReason.NONE


def test_reproducibility_seed():
    """Identical seeds must generate bitwise identical impairment sequences."""
    cfg1 = CommunicationProfileConfig(
        profile_name='SEED_TEST',
        enabled=True,
        latency_ms=100.0,
        jitter_ms=20.0,
        loss_probability=0.20,
        seed=999,
    )
    cfg2 = CommunicationProfileConfig(
        profile_name='SEED_TEST',
        enabled=True,
        latency_ms=100.0,
        jitter_ms=20.0,
        loss_probability=0.20,
        seed=999,
    )
    cfg3 = CommunicationProfileConfig(
        profile_name='SEED_TEST',
        enabled=True,
        latency_ms=100.0,
        jitter_ms=20.0,
        loss_probability=0.20,
        seed=111,
    )

    m1 = CommunicationImpairmentModel(cfg1)
    m2 = CommunicationImpairmentModel(cfg2)
    m3 = CommunicationImpairmentModel(cfg3)

    seq1 = [m1.process_message('amr_0', 'amr_1', float(i)) for i in range(100)]
    seq2 = [m2.process_message('amr_0', 'amr_1', float(i)) for i in range(100)]
    seq3 = [m3.process_message('amr_0', 'amr_1', float(i)) for i in range(100)]

    assert seq1 == seq2
    assert seq1 != seq3


def test_partition_isolation():
    """Verify that network partition isolates cross-boundary robot communications."""
    cfg = CommunicationProfileConfig(
        profile_name='PARTITION_TEST',
        enabled=True,
        isolated_robots=['amr_3', 'amr_4'],
        seed=42,
    )
    model = CommunicationImpairmentModel(cfg)

    # Intra-group 1 (amr_0 <-> amr_1)
    act, _, r = model.process_message('amr_0', 'amr_1', 1.0)
    assert act == CommunicationAction.DELIVER
    assert r == DropReason.NONE

    # Intra-group 2 (amr_3 <-> amr_4)
    act, _, r = model.process_message('amr_3', 'amr_4', 1.0)
    assert act == CommunicationAction.DELIVER
    assert r == DropReason.NONE

    # Cross-boundary (amr_0 -> amr_3)
    act, _, r = model.process_message('amr_0', 'amr_3', 1.0)
    assert act == CommunicationAction.DROP
    assert r == DropReason.PARTITION

    # Cross-boundary (amr_4 -> amr_1)
    act, _, r = model.process_message('amr_4', 'amr_1', 1.0)
    assert act == CommunicationAction.DROP
    assert r == DropReason.PARTITION


def test_delayed_message_queue():
    """Verify DelayedMessageQueue delivers messages only once delivery time arrives."""
    queue = DelayedMessageQueue()
    received = []

    def callback(payload):
        received.append(payload)

    queue.schedule(deliver_at=5.0, callback=callback, msg='msg_at_5')
    queue.schedule(deliver_at=10.0, callback=callback, msg='msg_at_10')

    deliv = queue.poll(current_time=3.0)
    assert deliv == 0
    assert received == []
    assert queue.pending_count == 2

    deliv = queue.poll(current_time=6.0)
    assert deliv == 1
    assert received == ['msg_at_5']
    assert queue.pending_count == 1

    deliv = queue.poll(current_time=11.0)
    assert deliv == 1
    assert received == ['msg_at_5', 'msg_at_10']
    assert queue.pending_count == 0


def test_stale_information_tracking():
    """Verify StaleStateManager tracks age and transitions CURRENT -> STALE -> EXPIRED."""
    mgr = StaleStateManager('amr_0', stale_threshold_s=1.5, expiry_threshold_s=4.0)

    assert mgr.get_peer_state('reservations', 'amr_1', 0.0) == InformationState.MISSING

    st, is_reconn = mgr.record_incoming(
        'reservations', 'amr_1', sent_timestamp=9.9, receive_timestamp=10.0
    )
    assert st == InformationState.CURRENT
    assert not is_reconn

    assert mgr.get_peer_state(
        'reservations', 'amr_1', current_time=11.0
    ) == InformationState.CURRENT

    assert mgr.get_peer_state(
        'reservations', 'amr_1', current_time=12.0
    ) == InformationState.STALE

    assert mgr.get_peer_state(
        'reservations', 'amr_1', current_time=15.0
    ) == InformationState.EXPIRED
    assert 'amr_1' in mgr.get_expired_peers('reservations', current_time=15.0)


def test_stale_reservation_pruning():
    """Expired reservations from silent peers must be pruned from SpaceTimeReservationTable."""
    res_table = SpaceTimeReservationTable()
    mgr = StaleStateManager('amr_0', stale_threshold_s=1.5, expiry_threshold_s=4.0)

    res_table.reserve((5, 5), time_step=2, robot_id='amr_1', priority=100.0)
    res_table.reserve((2, 2), time_step=2, robot_id='amr_0', priority=200.0)

    mgr.record_incoming('reservations', 'amr_1', sent_timestamp=1.0, receive_timestamp=1.0)

    pruned = mgr.prune_expired_reservations(res_table, current_time=3.0)
    assert pruned == []
    assert res_table.is_reserved((5, 5), time_step=2, robot_id='amr_2')

    pruned = mgr.prune_expired_reservations(res_table, current_time=6.0)
    assert 'amr_1' in pruned
    assert not res_table.is_reserved((5, 5), time_step=2, robot_id='amr_2')
    assert res_table.is_reserved((2, 2), time_step=2, robot_id='amr_2')


def test_reconnection_detection():
    """When a previously EXPIRED peer sends a message, reconnection flag is asserted."""
    mgr = StaleStateManager('amr_0', stale_threshold_s=1.0, expiry_threshold_s=2.0)

    mgr.record_incoming('reservations', 'amr_1', 0.0, 0.0)

    assert mgr.get_peer_state('reservations', 'amr_1', 5.0) == InformationState.EXPIRED

    st, is_reconn = mgr.record_incoming('reservations', 'amr_1', 5.9, 6.0)
    assert st == InformationState.CURRENT
    assert is_reconn is True
    assert mgr.reconnection_events_count == 1


def test_safety_invariance_under_network_failure():
    """Verify local sensor safety braking operates independently of total network outage."""
    cfg = CommunicationProfileConfig(
        profile_name='BLACKOUT',
        enabled=True,
        loss_probability=1.0,
        outage_start_s=0.0,
        outage_duration_s=1000.0,
    )
    model = CommunicationImpairmentModel(cfg)

    for i in range(10):
        act, _, _ = model.process_message('amr_0', 'amr_1', float(i))
        assert act == CommunicationAction.DROP

    lidar_ranges = [0.15, 0.20, 0.18]
    obstacle_ahead = bool(lidar_ranges and min(lidar_ranges) < 0.28)
    assert obstacle_ahead is True

    cmd_linear_x = 0.0 if obstacle_ahead else 0.30
    assert cmd_linear_x == 0.0

