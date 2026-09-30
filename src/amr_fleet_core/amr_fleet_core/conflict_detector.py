"""Deterministic Conflict Detection Engine for Multi-Agent Space-Time Paths (M6)."""

from typing import Dict, List, Optional, Sequence

from amr_fleet_core.coordination_models import Conflict, ConflictType, Position
from amr_fleet_core.reservation_table import SpaceTimeReservationTable


class ConflictDetector:
    """
    Deterministic Conflict Detection Engine.

    Tests multi-agent space-time trajectories independently of planning logic.
    Detects:
    1. Vertex Conflict: Two robots occupying cell c at time t.
    2. Edge-Swap Conflict: Robot A moves u -> v while Robot B moves v -> u at time t.
    3. Same-Cell Waiting Conflict: Robot A stays at c while Robot B moves into c at t+1.
    4. Reservation Conflict: Trajectory violates an active space-time reservation.
    """

    @staticmethod
    def check_step_conflict(
        pos_a_curr: Position,
        pos_a_next: Position,
        pos_b_curr: Position,
        pos_b_next: Position,
        time_step: int,
        robot_a: str = 'robot_a',
        robot_b: str = 'robot_b',
    ) -> Optional[Conflict]:
        """
        Check conflicts across a single discrete step transition [t, t+1].

        Returns Conflict if vertex or edge-swap conflict occurs, else None.
        """
        # 1. Edge-swap conflict
        if pos_a_curr == pos_b_next and pos_a_next == pos_b_curr and pos_a_curr != pos_a_next:
            return Conflict(
                conflict_type=ConflictType.EDGE_SWAP.value,
                robot_a=robot_a,
                robot_b=robot_b,
                cell=pos_a_next,
                time_step=time_step,
                details=(
                    f'Edge-swap conflict: {robot_a} ({pos_a_curr}->{pos_a_next}) and '
                    f'{robot_b} ({pos_b_curr}->{pos_b_next}) at t={time_step}'
                ),
            )

        # 2. Vertex conflict at arrival t+1
        if pos_a_next == pos_b_next:
            conf_type = (
                ConflictType.WAITING.value
                if (pos_a_curr == pos_a_next or pos_b_curr == pos_b_next)
                else ConflictType.VERTEX.value
            )
            return Conflict(
                conflict_type=conf_type,
                robot_a=robot_a,
                robot_b=robot_b,
                cell=pos_a_next,
                time_step=time_step + 1,
                details=(
                    f'{conf_type} conflict: {robot_a} and {robot_b} contest cell '
                    f'{pos_a_next} at t={time_step+1}'
                ),
            )

        return None

    @classmethod
    def check_trajectories(
        cls,
        traj_a: Sequence[Position],
        traj_b: Sequence[Position],
        start_time_a: int = 0,
        start_time_b: int = 0,
        robot_a: str = 'amr_a',
        robot_b: str = 'amr_b',
    ) -> List[Conflict]:
        """
        Evaluate complete trajectories between two agents across space-time.

        Each trajectory is a list of Positions indexed by relative time step.
        """
        conflicts: List[Conflict] = []
        if not traj_a or not traj_b:
            return conflicts

        max_len = max(len(traj_a) + start_time_a, len(traj_b) + start_time_b)

        # Helper to get position of robot at discrete global time step t
        def get_pos(traj: Sequence[Position], start_t: int, t: int) -> Optional[Position]:
            idx = t - start_t
            if idx < 0:
                return None
            if idx >= len(traj):
                # Robot remains at terminal waypoint
                return traj[-1]
            return traj[idx]

        for t in range(max_len):
            p_a_curr = get_pos(traj_a, start_time_a, t)
            p_b_curr = get_pos(traj_b, start_time_b, t)

            if p_a_curr is None or p_b_curr is None:
                continue

            # Vertex conflict at current step
            if p_a_curr == p_b_curr:
                conflicts.append(
                    Conflict(
                        conflict_type=ConflictType.VERTEX.value,
                        robot_a=robot_a,
                        robot_b=robot_b,
                        cell=p_a_curr,
                        time_step=t,
                        details=f'Co-occupation of cell {p_a_curr} at t={t}',
                    )
                )

            # Step transition check [t, t+1]
            p_a_next = get_pos(traj_a, start_time_a, t + 1)
            p_b_next = get_pos(traj_b, start_time_b, t + 1)

            if p_a_next is not None and p_b_next is not None:
                # Check edge swap
                if p_a_curr == p_b_next and p_a_next == p_b_curr and p_a_curr != p_a_next:
                    conflicts.append(
                        Conflict(
                            conflict_type=ConflictType.EDGE_SWAP.value,
                            robot_a=robot_a,
                            robot_b=robot_b,
                            cell=p_a_next,
                            time_step=t,
                            details=(
                                f'Edge swap: {robot_a} ({p_a_curr}->{p_a_next}) vs '
                                f'{robot_b} ({p_b_curr}->{p_b_next}) at t={t}'
                            ),
                        )
                    )

        return conflicts

    @classmethod
    def check_fleet_trajectories(
        cls,
        fleet_trajectories: Dict[str, Sequence[Position]],
        start_time_step: int = 0,
    ) -> List[Conflict]:
        """Check pairwise conflicts across all robots in fleet."""
        all_conflicts: List[Conflict] = []
        robot_ids = sorted(fleet_trajectories.keys())
        n = len(robot_ids)

        for i in range(n):
            for j in range(i + 1, n):
                r_a = robot_ids[i]
                r_b = robot_ids[j]
                c_list = cls.check_trajectories(
                    fleet_trajectories[r_a],
                    fleet_trajectories[r_b],
                    start_time_a=start_time_step,
                    start_time_b=start_time_step,
                    robot_a=r_a,
                    robot_b=r_b,
                )
                all_conflicts.extend(c_list)

        return all_conflicts

    @staticmethod
    def check_reservation_conflicts(
        table: SpaceTimeReservationTable,
        robot_id: str,
        path: Sequence[Position],
        start_time_step: int = 0,
    ) -> List[Conflict]:
        """Check if proposed path conflicts with any active reservation in table."""
        conflicts: List[Conflict] = []
        for step_idx in range(len(path) - 1):
            t = start_time_step + step_idx
            curr_pos = path[step_idx]
            next_pos = path[step_idx + 1]

            conf = table.get_conflict(curr_pos, next_pos, t, robot_id)
            if conf:
                conflicts.append(conf)

        return conflicts

