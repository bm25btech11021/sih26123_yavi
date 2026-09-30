"""Unit tests for deterministic conflict detection (M6)."""

from amr_fleet_core.conflict_detector import ConflictDetector
from amr_fleet_core.coordination_models import ConflictType


def test_vertex_conflict_detection() -> None:
    """Test detection of vertex co-occupation at identical timestep."""
    # Robot A and Robot B arrive at (5, 5) at t=2
    traj_a = [(5, 3), (5, 4), (5, 5)]
    traj_b = [(3, 5), (4, 5), (5, 5)]

    conflicts = ConflictDetector.check_trajectories(
        traj_a, traj_b, robot_a='amr_0', robot_b='amr_1',
    )
    assert len(conflicts) == 1
    c = conflicts[0]
    assert c.conflict_type == ConflictType.VERTEX.value
    assert c.cell == (5, 5)
    assert c.time_step == 2
    assert c.robot_a == 'amr_0'
    assert c.robot_b == 'amr_1'


def test_edge_swap_conflict_detection() -> None:
    """Test mandatory edge-swap example from project brief where A and B swap cells."""
    traj_a = [(2, 2), (2, 3)]
    traj_b = [(2, 3), (2, 2)]

    conflicts = ConflictDetector.check_trajectories(
        traj_a, traj_b, start_time_a=0, start_time_b=0, robot_a='amr_0', robot_b='amr_1',
    )
    assert len(conflicts) == 1
    c = conflicts[0]
    assert c.conflict_type == ConflictType.EDGE_SWAP.value
    assert c.time_step == 0
    assert c.cell == (2, 3) or c.cell == (2, 2)
    assert 'Edge swap' in c.details or 'swaps' in c.details.lower()


def test_same_cell_waiting_conflict() -> None:
    """Test conflict when one agent is stationary and another attempts to move into it."""
    # amr_0 stays at (4, 4) at t=0 and t=1
    pos_a_curr = (4, 4)
    pos_a_next = (4, 4)
    # amr_1 moves from (4, 3) into (4, 4) at t=0 -> t=1
    pos_b_curr = (4, 3)
    pos_b_next = (4, 4)

    conf = ConflictDetector.check_step_conflict(
        pos_a_curr, pos_a_next, pos_b_curr, pos_b_next, time_step=0,
        robot_a='amr_0', robot_b='amr_1',
    )
    assert conf is not None
    assert conf.conflict_type == ConflictType.WAITING.value
    assert conf.cell == (4, 4)
    assert conf.time_step == 1


def test_no_conflict_parallel_and_time_separated() -> None:
    """Test trajectories with no conflicts (parallel motion or time separation)."""
    # Parallel non-overlapping lanes
    lane_1 = [(2, 2), (2, 3), (2, 4), (2, 5)]
    lane_2 = [(3, 2), (3, 3), (3, 4), (3, 5)]
    assert len(ConflictDetector.check_trajectories(lane_1, lane_2)) == 0

    # Crossing intersection, but at different timesteps
    # amr_0 passes (4, 4) at t=1: (4, 3) -> (4, 4) -> (4, 5)
    traj_0 = [(4, 3), (4, 4), (4, 5), (4, 6)]
    # amr_1 passes (4, 4) at t=3: (2, 4) -> (3, 4) -> (4, 4) -> (5, 4)
    traj_1 = [(1, 4), (2, 4), (3, 4), (4, 4)]
    assert len(ConflictDetector.check_trajectories(traj_0, traj_1)) == 0


def test_pairwise_fleet_conflicts() -> None:
    """Test checking pairwise conflicts across a 3-robot fleet dictionary."""
    fleet = {
        'amr_0': [(1, 1), (1, 2)],
        'amr_1': [(1, 2), (1, 1)],  # edge swap with amr_0
        'amr_2': [(8, 8), (8, 9)],  # clear
    }
    all_confs = ConflictDetector.check_fleet_trajectories(fleet)
    assert len(all_confs) == 1
    assert all_confs[0].conflict_type == ConflictType.EDGE_SWAP.value
    assert {'amr_0', 'amr_1'} == {all_confs[0].robot_a, all_confs[0].robot_b}

