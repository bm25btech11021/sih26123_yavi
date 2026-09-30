"""
Prioritized Local Planning using PIBT Principles for Multi-Agent Fleets (M6).

Variant Description:
- Implements Windowed Discrete Priority Inheritance with Backtracking (PIBT).
- Uses M5 SingleAgentAStar path as directional guidance heuristic.
- Evaluates candidate neighboring cells ordered by A* preference and distance to goal.
- Supports recursive priority inheritance: a higher-priority robot pushes a lower-priority
  occupant to a feasible alternative neighbor.
- If the pushed robot cannot find a valid cell, backtracking occurs and the pusher
  evaluates its next best candidate move.
- Commits reservations directly into SpaceTimeReservationTable.
"""

import time
from typing import Any, Dict, List, Optional, Set, Tuple

from amr_fleet_core.coordination_models import Conflict, CoordinationState, Position
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_sim.grid_world import GridWorld


class PIBTAgentState:
    """State representation for an individual AMR participating in PIBT coordination."""

    def __init__(
        self,
        robot_id: str,
        current_pos: Position,
        goal_pos: Position,
        task_priority: int = 1,
        subgoal_phase: str = 'TRANSIT_TO_PICKUP',
        preferred_path: Optional[List[Position]] = None,
    ) -> None:
        self.robot_id = robot_id
        self.current_pos = current_pos
        self.goal_pos = goal_pos
        self.task_priority = task_priority
        self.subgoal_phase = subgoal_phase
        self.preferred_path = list(preferred_path) if preferred_path else []
        self.planned_moves: Dict[int, Position] = {}
        self.coordination_state = CoordinationState.CLEAR.value
        self.waiting_for: Optional[str] = None

    def compute_priority(self, grid: GridWorld) -> float:
        """
        Compute deterministic scalar priority for arbitration.

        Higher score = higher priority.
        Components:
        1. Task priority weight (1..4) * 1000.0
        2. Sub-goal urgency (Dropoff > Pickup > Idle)
        3. Proximity to goal: 10.0 / (1.0 + Manhattan distance)
        4. Deterministic tie-breaker based on numeric robot ID.
        """
        phase_weight = 200.0 if self.subgoal_phase == 'TRANSIT_TO_DROPOFF' else 100.0
        if self.subgoal_phase == 'IDLE':
            phase_weight = 0.0

        dist = grid.manhattan_distance(self.current_pos, self.goal_pos)
        prox_weight = 10.0 / (1.0 + dist)

        try:
            r_idx = int(self.robot_id.split('_')[-1])
        except Exception:
            r_idx = 0
        tie_breaker = (100.0 - r_idx) * 0.01

        return float(self.task_priority * 1000.0 + phase_weight + prox_weight + tie_breaker)


class PIBTLocalPlanner:
    """
    Modular Prioritized Local Planner based on PIBT.

    Coordinates discrete 1-step or multi-step execution windows across the fleet,
    preventing vertex collisions and edge swaps while ensuring deterministic execution.
    """

    def __init__(
        self,
        grid: GridWorld,
        reservation_table: SpaceTimeReservationTable,
    ) -> None:
        self.grid = grid
        self.table = reservation_table

    def generate_candidate_moves(
        self,
        agent: PIBTAgentState,
        curr_pos: Position,
    ) -> List[Position]:
        """
        Generate candidate moves ordered by preference.

        1. Next waypoint along M5 A* preferred path (if non-identical).
        2. 4-connected neighbors sorted by Manhattan distance to goal.
        3. Wait action (stay at curr_pos) as last option.
        """
        candidates: List[Position] = []

        # 1. Preferred path guidance
        if agent.preferred_path:
            for pt in agent.preferred_path:
                if pt != curr_pos and self.grid.is_free(pt):
                    if pt in self.grid.get_neighbors(curr_pos, allow_wait=False):
                        candidates.append(pt)
                        break

        # 2. Remaining 4-connected neighbors and wait action
        all_options = self.grid.get_neighbors(curr_pos, allow_wait=True)
        remaining = [p for p in all_options if p not in candidates]
        remaining.sort(
            key=lambda p: (
                self.grid.manhattan_distance(p, agent.goal_pos),
                0 if p == curr_pos else 1,
                p[0],
                p[1],
            )
        )
        candidates.extend(remaining)

        return candidates

    def plan_step(
        self,
        agents: Dict[str, PIBTAgentState],
        time_step: int,
    ) -> Tuple[Dict[str, Position], List[Conflict]]:
        """
        Execute one synchronous PIBT coordination step at time_step -> time_step + 1.

        Returns a tuple containing (decided_moves, conflicts).
        """
        decided_moves: Dict[str, Position] = {}
        conflicts: List[Conflict] = []
        in_process: Set[str] = set()

        # Sort agents descending by deterministic priority
        sorted_agents = sorted(
            agents.values(),
            key=lambda a: a.compute_priority(self.grid),
            reverse=True,
        )

        # Map current positions
        curr_occupants: Dict[Position, str] = {
            a.current_pos: a.robot_id for a in agents.values()
        }

        def pibt_step(
            agent_id: str,
            inherited_priority: float,
            depth: int = 0,
            pusher_pos: Optional[Position] = None,
        ) -> bool:
            if agent_id in in_process:
                # Cycle detected during recursive push
                return False
            in_process.add(agent_id)

            ag = agents[agent_id]
            curr_pos = ag.current_pos
            candidates = self.generate_candidate_moves(ag, curr_pos)

            for cand in candidates:
                # If this agent was pushed, it cannot satisfy push by staying in curr_pos
                if pusher_pos is not None and cand == curr_pos:
                    continue

                # Disallow moving into pusher's current cell (edge swap prevention)
                if pusher_pos is not None and cand == pusher_pos:
                    continue

                # Disallow static obstacles
                if not self.grid.is_free(cand):
                    continue

                # Disallow edge swap with already decided moves
                edge_swap = False
                for other_id, target in decided_moves.items():
                    other_curr = agents[other_id].current_pos
                    if curr_pos == target and cand == other_curr:
                        edge_swap = True
                        break
                if edge_swap:
                    continue

                # Disallow vertex conflict with already decided moves
                if cand in decided_moves.values():
                    continue

                # Check space-time reservation table
                conf = self.table.get_conflict(curr_pos, cand, time_step, agent_id)
                if conf is not None:
                    conflicts.append(conf)
                    continue

                # Check if cell is currently occupied by another agent
                occupant_id = curr_occupants.get(cand)
                if occupant_id is not None and occupant_id != agent_id:
                    # Occupant exists and hasn't decided yet -> attempt priority inheritance
                    if occupant_id not in decided_moves:
                        push_prio = max(
                            inherited_priority,
                            agents[occupant_id].compute_priority(self.grid),
                        )
                        success = pibt_step(
                            occupant_id, push_prio, depth + 1, pusher_pos=curr_pos,
                        )
                        if not success:
                            # Backtrack: occupant cannot clear cell, evaluate next candidate
                            ag.coordination_state = CoordinationState.BACKTRACKING.value
                            ag.waiting_for = occupant_id
                            continue
                        else:
                            ag.coordination_state = CoordinationState.CONFLICT_RESOLVED.value
                            ag.waiting_for = None
                    else:
                        # Occupant already decided to stay in this cell
                        if decided_moves[occupant_id] == cand:
                            continue

                # Target cell is viable! Commit decision for agent_id
                decided_moves[agent_id] = cand
                ag.planned_moves[time_step + 1] = cand
                in_process.remove(agent_id)
                return True

            in_process.remove(agent_id)
            # If pushed to vacate, cannot satisfy push by waiting in curr_pos
            if pusher_pos is not None:
                return False

            # All candidates failed: fallback to wait if not conflicting
            if curr_pos not in decided_moves.values():
                decided_moves[agent_id] = curr_pos
                ag.planned_moves[time_step + 1] = curr_pos
                ag.coordination_state = CoordinationState.YIELDING.value
                return True

            return False

        for ag in sorted_agents:
            if ag.robot_id not in decided_moves:
                prio = ag.compute_priority(self.grid)
                pibt_step(ag.robot_id, prio, depth=0)

        # Commit decided moves into reservation table
        for r_id, tgt in decided_moves.items():
            c_pos = agents[r_id].current_pos
            prio = agents[r_id].compute_priority(self.grid)
            if c_pos == tgt:
                self.table.reserve(tgt, time_step + 1, r_id, priority=prio)
            else:
                self.table.reserve_edge(c_pos, tgt, time_step, r_id, priority=prio)

        return decided_moves, conflicts

    def plan_step_with_telemetry(
        self,
        agents: Dict[str, PIBTAgentState],
        time_step: int,
        trigger: str = 'LOCAL_VERTEX_CONTENTION_FALLBACK',
    ) -> Tuple[Dict[str, Position], List[Conflict], Dict[str, Any]]:
        """
        Execute one synchronous PIBT step and return structured telemetry.

        Provides direct evidence traceability from actual PIBT execution.
        """
        t0 = time.perf_counter()
        moves, conflicts = self.plan_step(agents, time_step)
        duration_ms = (time.perf_counter() - t0) * 1000.0

        assigned_cells = list(moves.values())
        no_overlap = len(assigned_cells) == len(set(assigned_cells))

        telemetry = {
            'pibt_invoked': True,
            'trigger': trigger,
            'robots_involved': list(agents.keys()),
            'action': 'PRIORITIZED_PUSH_AND_YIELD',
            'result': 'SUCCESS' if no_overlap else 'CONFLICT_UNRESOLVED',
            'duration_ms': round(duration_ms, 3),
            'execution_level': 'PLANNER_LEVEL_PIBT_INTEGRATION',
            'assigned_moves': {r: list(pos) for r, pos in moves.items()},
        }
        return moves, conflicts, telemetry

    def plan_window(
        self,
        agents: Dict[str, PIBTAgentState],
        start_time_step: int,
        window_size: int,
    ) -> Dict[str, List[Position]]:
        """
        Coordinate execution window of length window_size across all agents.

        Returns map of robot_id -> list of sequential coordinated waypoints.
        """
        coordinated_paths: Dict[str, List[Position]] = {
            r_id: [a.current_pos] for r_id, a in agents.items()
        }

        # Work on local copy of agent states
        sim_agents: Dict[str, PIBTAgentState] = {}
        for r_id, a in agents.items():
            sim_agents[r_id] = PIBTAgentState(
                robot_id=a.robot_id,
                current_pos=a.current_pos,
                goal_pos=a.goal_pos,
                task_priority=a.task_priority,
                subgoal_phase=a.subgoal_phase,
                preferred_path=a.preferred_path,
            )

        for step_idx in range(window_size):
            t = start_time_step + step_idx
            moves, _ = self.plan_step(sim_agents, t)

            for r_id, next_pos in moves.items():
                coordinated_paths[r_id].append(next_pos)
                sim_agents[r_id].current_pos = next_pos
                # Advance preferred path index if matched
                if (sim_agents[r_id].preferred_path
                        and sim_agents[r_id].preferred_path[0] == next_pos):
                    sim_agents[r_id].preferred_path.pop(0)

        return coordinated_paths

