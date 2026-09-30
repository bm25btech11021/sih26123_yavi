"""Unit tests for M9-V2 Pillar B: Headway-Aware Reservation."""

from amr_fleet_core.coordination_models import ConflictType
from amr_fleet_core.reservation_table import SpaceTimeReservationTable


def test_headway_conflict_single_lane_convoy():
    """Verify that follower cannot reserve adjacent cell behind lead AMR in same lane."""
    table = SpaceTimeReservationTable()

    # Lead robot amr_3 reserves bay cell (29, 29) at t=10
    assert table.reserve((29, 29), time_step=10, robot_id='amr_3', priority=500.0) is True

    # amr_4 at (29, 31) attempts to step to (29, 30) along south vector (0, -1)
    has_conflict = table.is_headway_conflict(
        from_pos=(29, 31),
        to_pos=(29, 30),
        time_step=10,
        robot_id='amr_4',
        min_headway_cells=2,
    )
    assert has_conflict is True, 'Follower must detect headway conflict behind lead robot'

    conf = table.get_headway_conflict(
        from_pos=(29, 31),
        to_pos=(29, 30),
        time_step=10,
        robot_id='amr_4',
        min_headway_cells=2,
    )
    assert conf is not None
    assert conf.conflict_type == ConflictType.VERTEX.value
    assert conf.robot_a == 'amr_4'
    assert conf.robot_b == 'amr_3'
    assert conf.cell == (29, 30)


def test_headway_allowed_at_safe_distance():
    """Moving into a cell 2 or more steps behind the lead AMR is allowed."""
    table = SpaceTimeReservationTable()

    table.reserve((29, 29), time_step=10, robot_id='amr_3', priority=500.0)

    has_conflict = table.is_headway_conflict(
        from_pos=(29, 32),
        to_pos=(29, 31),
        time_step=10,
        robot_id='amr_4',
        min_headway_cells=2,
    )
    assert has_conflict is False, 'Stepping to (29, 31) maintains k >= 2 headway'


def test_headway_self_idempotence():
    """A robot's own reservations ahead of itself do not trigger a headway conflict."""
    table = SpaceTimeReservationTable()

    table.reserve((29, 29), time_step=10, robot_id='amr_0', priority=100.0)
    has_conflict = table.is_headway_conflict(
        from_pos=(29, 31),
        to_pos=(29, 30),
        time_step=10,
        robot_id='amr_0',
        min_headway_cells=2,
    )
    assert has_conflict is False, 'Self-reservations must be ignored'


def test_headway_horizontal_aisle():
    """Verify headway detection along horizontal (East-West) corridors."""
    table = SpaceTimeReservationTable()

    table.reserve((15, 10), time_step=5, robot_id='amr_1', priority=200.0)

    has_conflict = table.is_headway_conflict(
        from_pos=(13, 10),
        to_pos=(14, 10),
        time_step=5,
        robot_id='amr_2',
        min_headway_cells=2,
    )
    assert has_conflict is True

