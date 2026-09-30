"""Multi-Agent Path Coordinator wrapping M5 Rolling-Horizon Planning (M6)."""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from amr_fleet_core.conflict_detector import ConflictDetector
from amr_fleet_core.coordination_models import ConflictType, Position
from amr_fleet_core.deadlock_recovery import DeadlockRecoveryManager
from amr_fleet_core.pibt_planner import PIBTAgentState, PIBTLocalPlanner
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_planner import RollingHorizonPlanner
from amr_fleet_core.wfg_deadlock import DeadlockDetector, WaitForGraph
from amr_fleet_sim.grid_world import GridWorld


class MultiAgentCoordinator:
    """
    Multi-Agent Path Coordination Layer (M6).

    Coordinates rolling-horizon execution windows across the fleet without replacing
    M5's single-agent planning engine.
    Integrates:
    - SpaceTimeReservationTable (vertex & edge reservations)
    - ConflictDetector (vertex & edge-swap conflict detection)
    - PIBTLocalPlanner (prioritized local arbitration & priority inheritance)
    - WaitForGraph & DeadlockDetector (cycle detection & persistence filtering)
    - DeadlockRecoveryManager (deterministic victim sidestepping & priority inversion)
    """

    def __init__(
        self,
        grid: Optional[GridWorld] = None,
        reservation_table: Optional[SpaceTimeReservationTable] = None,
        persistence_threshold_sec: float = 1.5,
        min_stall_cycles: int = 3,
    ) -> None:
        """Initialize coordination components and metric aggregators."""
        self.grid = grid if grid is not None else GridWorld.create_warehouse_grid(resolution=0.5)
        self.reservation_table = (
            reservation_table if reservation_table is not None
            else SpaceTimeReservationTable()
        )
        self.conflict_detector = ConflictDetector()
        self.pibt_planner = PIBTLocalPlanner(self.grid, self.reservation_table)
        self.wfg = WaitForGraph()
        self.deadlock_detector = DeadlockDetector(
            persistence_threshold_sec=persistence_threshold_sec,
            min_stall_cycles=min_stall_cycles,
        )
        self.recovery_manager = DeadlockRecoveryManager(
            self.grid, self.reservation_table,
        )

        # Performance Metrics Tracking
        self.total_coordination_cycles: int = 0
        self.total_conflicts_detected: int = 0
        self.vertex_conflicts_detected: int = 0
        self.edge_swap_conflicts_detected: int = 0
        self.conflicts_resolved: int = 0
        self.deadlocks_detected: int = 0
        self.deadlocks_recovered: int = 0
        self.last_coordination_latency_ms: float = 0.0
        self.all_latencies_ms: List[float] = []
        self.min_planned_inter_robot_distance: float = float('inf')

    def coordinate_fleet(
        self,
        fleet_planners: Dict[str, RollingHorizonPlanner],
        current_time_step: int,
        window_size: int = 4,
    ) -> Dict[str, List[Position]]:
        """
        Coordinate execution windows across all active AMRs in the fleet.

        1. Extracts single-agent A* paths from each AMR's M5 RollingHorizonPlanner.
        2. Converts planners into PIBTAgentState instances with deterministic priorities.
        3. Runs PIBT window planning with space-time reservation checks.
        4. Detects conflicts and updates WFG.
        5. Evaluates deadlocks; triggers deterministic recovery if persistent cycle found.
        6. Updates each planner's execution_path with coordinated waypoints.
        7. Prunes expired reservations from table.
        """
        start_time = time.perf_counter()
        self.total_coordination_cycles += 1

        # Prune old reservations
        self.reservation_table.release_time_before(current_time_step)

        # Build agent states from M5 planners
        agents: Dict[str, PIBTAgentState] = {}
        curr_positions_world: Dict[str, Tuple[float, float]] = {}

        for r_id, p in fleet_planners.items():
            curr_pos_grid = self.grid.to_grid(p.current_position[0], p.current_position[1])
            goal_pos_grid = (
                self.grid.to_grid(p.current_goal[0], p.current_goal[1])
                if p.current_goal else curr_pos_grid
            )

            # Extract preferred A* path in grid coordinates
            pref_grid: List[Position] = []
            for pt in p.full_path:
                g_pt = self.grid.to_grid(pt[0], pt[1])
                if not pref_grid or pref_grid[-1] != g_pt:
                    pref_grid.append(g_pt)

            task_prio = 1
            if p.active_task_idx < len(p.ordered_tasks):
                tid = p.ordered_tasks[p.active_task_idx]
                task_def = p.tasks_map.get(tid, {})
                task_prio = task_def.get('priority', 1)

            agents[r_id] = PIBTAgentState(
                robot_id=r_id,
                current_pos=curr_pos_grid,
                goal_pos=goal_pos_grid,
                task_priority=task_prio,
                subgoal_phase=p.active_phase,
                preferred_path=pref_grid,
            )
            curr_positions_world[r_id] = p.current_position

        # 1. Check raw M5 single-agent paths for potential conflicts before coordination
        raw_trajectories: Dict[str, List[Position]] = {
            r_id: a.preferred_path[:window_size + 1] if a.preferred_path else [a.current_pos]
            for r_id, a in agents.items()
        }
        pre_conflicts = self.conflict_detector.check_fleet_trajectories(
            raw_trajectories, start_time_step=current_time_step,
        )
        self.total_conflicts_detected += len(pre_conflicts)
        for c in pre_conflicts:
            if c.conflict_type == ConflictType.VERTEX.value:
                self.vertex_conflicts_detected += 1
            elif c.conflict_type == ConflictType.EDGE_SWAP.value:
                self.edge_swap_conflicts_detected += 1

        # 2. Plan coordinated window via PIBT
        coordinated_grid_paths = self.pibt_planner.plan_window(
            agents, start_time_step=current_time_step, window_size=window_size,
        )

        # 3. Update Wait-For Graph based on agents yielding or blocked
        for r_id, ag in agents.items():
            if ag.waiting_for:
                target_cand = ag.planned_moves.get(current_time_step + 1, ag.current_pos)
                self.wfg.add_wait(
                    r_id, ag.waiting_for, target_cand, current_time_step,
                )
            else:
                self.wfg.clear_robot(r_id)

        # 4. Check for deadlocks in WFG
        confirmed_deadlocks = self.deadlock_detector.update(
            self.wfg, curr_positions_world, now_sec=time.time(),
        )

        for dl in confirmed_deadlocks:
            self.deadlocks_detected += 1
            rec_ok = self.recovery_manager.execute_recovery(
                dl, agents, current_time_step,
            )
            if rec_ok:
                self.deadlocks_recovered += 1
                self.deadlock_detector.reset_cycle(dl.cycle_robot_ids)
                # Apply recovery move into coordinated paths
                for c_id in dl.cycle_robot_ids:
                    if c_id in agents and (current_time_step + 1) in agents[c_id].planned_moves:
                        rec_move = agents[c_id].planned_moves[current_time_step + 1]
                        if len(coordinated_grid_paths[c_id]) > 1:
                            coordinated_grid_paths[c_id][1] = rec_move

        # 5. Verify that coordinated paths have 0 remaining conflicts
        post_conflicts = self.conflict_detector.check_fleet_trajectories(
            coordinated_grid_paths, start_time_step=current_time_step,
        )
        resolved_this_cycle = max(0, len(pre_conflicts) - len(post_conflicts))
        self.conflicts_resolved += resolved_this_cycle

        # 6. Convert coordinated grid waypoints back to world coordinates for M5 execution
        for r_id, p in fleet_planners.items():
            g_path = coordinated_grid_paths.get(r_id, [])
            if len(g_path) > 1:
                world_exec_path = [self.grid.to_world(pt) for pt in g_path[1:]]
                p.execution_path = world_exec_path

        # 7. Record metrics
        latency_ms = (time.perf_counter() - start_time) * 1000.0
        self.last_coordination_latency_ms = latency_ms
        self.all_latencies_ms.append(latency_ms)

        # Measure planned inter-robot distance across coordinated window
        self._update_min_planned_distance(coordinated_grid_paths)

        return coordinated_grid_paths

    def _update_min_planned_distance(
        self,
        paths: Dict[str, List[Position]],
    ) -> None:
        """Compute minimum Euclidean distance across coordinated waypoints."""
        r_ids = list(paths.keys())
        n = len(r_ids)
        if n < 2:
            return

        min_len = min(len(p) for p in paths.values()) if paths else 0
        for step_idx in range(min_len):
            for i in range(n):
                for j in range(i + 1, n):
                    p1 = self.grid.to_world(paths[r_ids[i]][step_idx])
                    p2 = self.grid.to_world(paths[r_ids[j]][step_idx])
                    dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    if dist < self.min_planned_inter_robot_distance:
                        self.min_planned_inter_robot_distance = dist

    def get_metrics_summary(self) -> Dict[str, Any]:
        """Return aggregated coordination performance metrics."""
        avg_lat = (
            sum(self.all_latencies_ms) / len(self.all_latencies_ms)
            if self.all_latencies_ms else 0.0
        )
        return {
            'total_coordination_cycles': self.total_coordination_cycles,
            'total_conflicts_detected': self.total_conflicts_detected,
            'vertex_conflicts_detected': self.vertex_conflicts_detected,
            'edge_swap_conflicts_detected': self.edge_swap_conflicts_detected,
            'conflicts_resolved': self.conflicts_resolved,
            'conflict_resolution_rate': (
                (self.conflicts_resolved / self.total_conflicts_detected)
                if self.total_conflicts_detected > 0 else 1.0
            ),
            'deadlocks_detected': self.deadlocks_detected,
            'deadlocks_recovered': self.deadlocks_recovered,
            'deadlock_recovery_rate': (
                (self.deadlocks_recovered / self.deadlocks_detected)
                if self.deadlocks_detected > 0 else 1.0
            ),
            'average_coordination_latency_ms': round(avg_lat, 3),
            'min_planned_distance_m': round(self.min_planned_inter_robot_distance, 3),
        }

