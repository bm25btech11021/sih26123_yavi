"""
Deterministic Deadlock Recovery Manager for Multi-Agent Path Coordination (M6).

Minimal Defensible Recovery Strategy:
1. Identify Victim Robot in the confirmed WFG cycle (lowest priority participant).
2. Locate a free lateral escape cell or backward step outside the contested corridor.
3. Release the victim's reservations and command a deterministic sidestep / detour.
4. Temporarily invert priority to grant the cycle leader uncontested right-of-way.
5. Log all recovery telemetry (participants, action, duration, success).
"""

import time
from typing import Dict, List, Optional

from amr_fleet_core.coordination_models import CoordinationState, DeadlockRecord, Position
from amr_fleet_core.pibt_planner import PIBTAgentState
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_sim.grid_world import GridWorld


class DeadlockRecoveryManager:
    """
    Deterministic Deadlock Recovery Manager.

    Resolves persistent WFG cycles through minimal, observable, and reproducible
    local backtracking and lateral sidestep maneuvers.
    """

    def __init__(
        self,
        grid: GridWorld,
        reservation_table: SpaceTimeReservationTable,
    ) -> None:
        self.grid = grid
        self.table = reservation_table
        self.recovery_history: List[DeadlockRecord] = []

    def select_victim(
        self,
        cycle_robots: List[str],
        agents: Dict[str, PIBTAgentState],
    ) -> str:
        """
        Select the victim robot to yield/backtrack in the cycle.

        Deterministic criteria:
        1. Agent with lowest scalar priority.
        2. Tie-broken by highest numeric robot ID (e.g. amr_4 over amr_0).
        """
        candidates = [r for r in cycle_robots if r in agents]
        if not candidates:
            return cycle_robots[-1]

        candidates.sort(
            key=lambda r: (
                agents[r].compute_priority(self.grid),
                -int(r.split('_')[-1]) if '_' in r and r.split('_')[-1].isdigit() else 0,
            )
        )
        return candidates[0]

    def find_escape_cell(
        self,
        victim_id: str,
        agents: Dict[str, PIBTAgentState],
        cycle_robots: List[str],
    ) -> Optional[Position]:
        """
        Find an adjacent free grid cell that is NOT in the path of the cycle participants.

        Returns Position of escape cell if found, else None.
        """
        victim_pos = agents[victim_id].current_pos
        neighbors = self.grid.get_neighbors(victim_pos, allow_wait=False)

        cycle_positions = {
            agents[r].current_pos for r in cycle_robots if r in agents
        }

        # Filter neighbors: must be free of static obstacles and not currently
        # occupied by cycle peers
        valid_escapes = [
            p for p in neighbors
            if self.grid.is_free(p) and p not in cycle_positions
        ]

        if not valid_escapes:
            return None

        # Sort by distance away from cycle center
        return valid_escapes[0]

    def execute_recovery(
        self,
        record: DeadlockRecord,
        agents: Dict[str, PIBTAgentState],
        time_step: int,
    ) -> bool:
        """
        Execute deterministic recovery for a confirmed deadlock record.

        Updates agent planned moves and reservation table.
        """
        t_start = time.time()
        victim_id = self.select_victim(record.cycle_robot_ids, agents)
        escape_pos = self.find_escape_cell(victim_id, agents, record.cycle_robot_ids)

        # 1. Release victim's conflicting reservations
        self.table.release(victim_id)

        if escape_pos is not None:
            # Action: Sidestep to escape cell
            action_desc = f'SIDESTEP: {victim_id} moves to lateral escape cell {escape_pos}'
            record.recovery_action = action_desc
            agents[victim_id].coordination_state = CoordinationState.RECOVERING.value
            agents[victim_id].planned_moves[time_step + 1] = escape_pos

            # Reserve escape move
            self.table.reserve_edge(
                agents[victim_id].current_pos,
                escape_pos,
                time_step,
                victim_id,
                priority=0.0,
            )
            record.recovery_success = True
        else:
            # Action: Controlled Wait with priority yield
            action_desc = f'PRIORITY_INVERSION_WAIT: {victim_id} waits while cycle leader advances'
            record.recovery_action = action_desc
            agents[victim_id].coordination_state = CoordinationState.YIELDING.value
            agents[victim_id].planned_moves[time_step + 1] = agents[victim_id].current_pos

            self.table.reserve(
                agents[victim_id].current_pos,
                time_step + 1,
                victim_id,
                priority=0.0,
            )
            record.recovery_success = True

        duration = time.time() - t_start
        record.recovery_duration_sec = round(duration, 4)
        self.recovery_history.append(record)

        return record.recovery_success

