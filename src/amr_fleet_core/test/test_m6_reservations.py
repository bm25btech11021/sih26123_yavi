"""Unit tests for discrete space-time reservations (M6)."""

from amr_fleet_core.coordination_models import ConflictType
from amr_fleet_core.reservation_table import SpaceTimeReservationTable


def test_vertex_reservation_basic() -> None:
    """Test successful vertex reservation and ownership."""
    table = SpaceTimeReservationTable()
    # amr_0 reserves (2, 3) at t=5
    assert table.reserve((2, 3), time_step=5, robot_id='amr_0', priority=10.0) is True
    # Idempotent for same owner
    assert table.reserve((2, 3), time_step=5, robot_id='amr_0', priority=10.0) is True
    # Denied for peer robot
    assert table.reserve((2, 3), time_step=5, robot_id='amr_1', priority=5.0) is False

    # Check query
    assert table.is_reserved((2, 3), time_step=5, robot_id='amr_1') is True
    assert table.is_reserved((2, 3), time_step=5, robot_id='amr_0') is False
    assert table.is_reserved((2, 3), time_step=6, robot_id='amr_1') is False


def test_edge_reservation_and_swap_rejection() -> None:
    """Test edge reservation from u to v and automatic reverse edge-swap rejection."""
    table = SpaceTimeReservationTable()
    # amr_0 reserves traversal (2, 2) -> (2, 3) across [t=1, t=2]
    assert table.reserve_edge((2, 2), (2, 3), time_step=1, robot_id='amr_0') is True

    # Check target vertex reservation at arrival t=2
    assert table.is_reserved((2, 3), time_step=2, robot_id='amr_1') is True

    # amr_1 attempts opposite edge (2, 3) -> (2, 2) at t=1 (Edge-swap conflict)
    assert table.is_edge_conflict((2, 3), (2, 2), time_step=1, robot_id='amr_1') is True
    assert table.reserve_edge((2, 3), (2, 2), time_step=1, robot_id='amr_1') is False

    # Check conflict query
    conf = table.get_conflict((2, 3), (2, 2), time_step=1, robot_id='amr_1')
    assert conf is not None
    assert conf.conflict_type == ConflictType.EDGE_SWAP.value
    assert conf.robot_a == 'amr_1'
    assert conf.robot_b == 'amr_0'


def test_reservation_release() -> None:
    """Test releasing all reservations owned by an individual AMR."""
    table = SpaceTimeReservationTable()
    table.reserve((5, 5), time_step=3, robot_id='amr_2')
    table.reserve_edge((5, 5), (5, 6), time_step=4, robot_id='amr_2')

    assert table.is_reserved((5, 5), time_step=3, robot_id='amr_0') is True
    assert len(table.get_reservations_for_robot('amr_2')) >= 2

    table.release('amr_2')
    assert table.is_reserved((5, 5), time_step=3, robot_id='amr_0') is False
    assert len(table.get_reservations_for_robot('amr_2')) == 0


def test_rolling_horizon_expiration() -> None:
    """Test pruning of expired reservations as horizon window advances."""
    table = SpaceTimeReservationTable()
    table.reserve((1, 1), time_step=1, robot_id='amr_0')
    table.reserve((1, 2), time_step=2, robot_id='amr_0')
    table.reserve((1, 3), time_step=5, robot_id='amr_0')

    # Advance horizon: expire all steps strictly before t=3
    table.release_time_before(min_time_step=3)

    assert table.is_reserved((1, 1), time_step=1, robot_id='amr_1') is False
    assert table.is_reserved((1, 2), time_step=2, robot_id='amr_1') is False
    assert table.is_reserved((1, 3), time_step=5, robot_id='amr_1') is True

