"""Wait-For Graph (WFG) Representation and Persistent Deadlock Detection (M6)."""

import math
import time
from typing import Any, Dict, List, Optional, Set, Tuple

from amr_fleet_core.coordination_models import DeadlockRecord, Position


class WaitForGraph:
    """
    Directed Wait-For Graph (WFG) representing resource dependencies.

    Nodes: Robot IDs.
    Directed edge (u -> v): Robot u is waiting for Robot v to vacate or release a resource.
    """

    def __init__(self) -> None:
        """Initialize adjacency lists and edge metadata."""
        # Map: waiter_id -> Dict[blocking_id, Dict[str, Any]]
        self._adj: Dict[str, Dict[str, Dict[str, Any]]] = {}

    def add_wait(
        self,
        waiter_id: str,
        blocking_id: str,
        resource: Position,
        time_step: int,
        timestamp: Optional[float] = None,
    ) -> None:
        """Add or update a directed waiting edge waiter_id -> blocking_id."""
        if waiter_id == blocking_id:
            return
        t_now = timestamp if timestamp is not None else time.time()
        self._adj.setdefault(waiter_id, {})[blocking_id] = {
            'resource': resource,
            'time_step': time_step,
            'created_at': t_now,
        }

    def remove_wait(self, waiter_id: str, blocking_id: Optional[str] = None) -> None:
        """Remove waiting edge(s) for waiter_id."""
        if waiter_id not in self._adj:
            return
        if blocking_id is None:
            self._adj.pop(waiter_id, None)
        else:
            self._adj[waiter_id].pop(blocking_id, None)
            if not self._adj[waiter_id]:
                self._adj.pop(waiter_id, None)

    def clear_robot(self, robot_id: str) -> None:
        """Remove all incoming and outgoing edges involving robot_id."""
        self._adj.pop(robot_id, None)
        for waiter in list(self._adj.keys()):
            self._adj[waiter].pop(robot_id, None)
            if not self._adj[waiter]:
                self._adj.pop(waiter, None)

    def get_waiting_for(self, waiter_id: str) -> Optional[str]:
        """Return the primary blocking robot ID that waiter_id is waiting on."""
        outgoing = self._adj.get(waiter_id)
        if not outgoing:
            return None
        return next(iter(outgoing.keys()))

    def get_edges(self) -> List[Tuple[str, str, Position, int]]:
        """Return list of all directed edges as (waiter, blocker, resource, time_step)."""
        edges = []
        for u, targets in self._adj.items():
            for v, meta in targets.items():
                edges.append((u, v, meta['resource'], meta['time_step']))
        return edges

    def find_cycles(self) -> List[List[str]]:
        """
        Find all simple elementary cycles in the WFG using DFS.

        Returns cycles in canonical order (minimum element first) without duplicates.
        """
        visited: Set[str] = set()
        in_stack: Set[str] = set()
        stack: List[str] = []
        cycles: List[List[str]] = []
        seen_canonical: Set[Tuple[str, ...]] = set()

        def dfs(u: str) -> None:
            visited.add(u)
            in_stack.add(u)
            stack.append(u)

            for v in self._adj.get(u, {}).keys():
                if v not in visited:
                    dfs(v)
                elif v in in_stack:
                    # Cycle detected: extract subpath from v to end of stack
                    idx = stack.index(v)
                    cycle = stack[idx:]
                    # Canonical rotation: shift smallest robot_id to index 0
                    min_idx = cycle.index(min(cycle))
                    canonical = tuple(cycle[min_idx:] + cycle[:min_idx])
                    if canonical not in seen_canonical and len(canonical) >= 2:
                        seen_canonical.add(canonical)
                        cycles.append(list(canonical))

            stack.pop()
            in_stack.remove(u)

        all_nodes = set(self._adj.keys())
        for targets in self._adj.values():
            all_nodes.update(targets.keys())

        for node in sorted(all_nodes):
            if node not in visited:
                dfs(node)

        return cycles


class DeadlockDetector:
    """
    Persistent Deadlock Detection Engine.

    Distinguishes transient waits from genuine deadlocks by evaluating:
    1. WFG cycle presence.
    2. Temporal persistence threshold (time elapsed since cycle formation).
    3. Spatial no-progress condition (physical displacement < threshold).
    """

    def __init__(
        self,
        persistence_threshold_sec: float = 1.5,
        min_stall_cycles: int = 3,
        progress_tolerance_m: float = 0.08,
    ) -> None:
        """Configure deadlock detection thresholds."""
        self.persistence_threshold_sec = persistence_threshold_sec
        self.min_stall_cycles = min_stall_cycles
        self.progress_tolerance_m = progress_tolerance_m

        # Map: canonical cycle tuple -> first detected timestamp
        self._cycle_history: Dict[Tuple[str, ...], float] = {}
        # Map: canonical cycle tuple -> consecutive observations count
        self._cycle_counts: Dict[Tuple[str, ...], int] = {}
        # Map: robot_id -> (last_x, last_y)
        self._last_positions: Dict[str, Tuple[float, float]] = {}
        # Map: robot_id -> stall cycle count
        self._robot_stalls: Dict[str, int] = {}

    def update(
        self,
        wfg: WaitForGraph,
        current_positions: Dict[str, Tuple[float, float]],
        now_sec: Optional[float] = None,
    ) -> List[DeadlockRecord]:
        """
        Evaluate WFG and current robot positions for persistent deadlocks.

        Returns list of newly detected or persistent DeadlockRecord instances.
        """
        t_now = now_sec if now_sec is not None else time.time()
        active_cycles = wfg.find_cycles()
        active_canonical = {tuple(c) for c in active_cycles}

        # Update position stalls
        for r_id, pos in current_positions.items():
            last = self._last_positions.get(r_id)
            if last is not None:
                disp = math.hypot(pos[0] - last[0], pos[1] - last[1])
                if disp < self.progress_tolerance_m:
                    self._robot_stalls[r_id] = self._robot_stalls.get(r_id, 0) + 1
                else:
                    self._robot_stalls[r_id] = 0
            else:
                self._robot_stalls[r_id] = 0
            self._last_positions[r_id] = pos

        # Prune expired cycles from history
        for c in list(self._cycle_history.keys()):
            if c not in active_canonical:
                self._cycle_history.pop(c, None)
                self._cycle_counts.pop(c, None)

        confirmed_deadlocks: List[DeadlockRecord] = []

        for cycle in active_cycles:
            c_key = tuple(cycle)
            if c_key not in self._cycle_history:
                self._cycle_history[c_key] = t_now
                self._cycle_counts[c_key] = 1
            else:
                self._cycle_counts[c_key] += 1

            duration = t_now - self._cycle_history[c_key]
            counts = self._cycle_counts[c_key]

            # Verify no-progress condition across all participants
            all_stalled = all(self._robot_stalls.get(r, 0) >= 1 for r in cycle)

            # Check persistence criteria:
            # 1. Temporal persistence >= persistence_threshold_sec OR
            # 2. Cycle observed for >= min_stall_cycles
            persistent = (
                duration >= self.persistence_threshold_sec
                or counts >= self.min_stall_cycles
            )
            if persistent and all_stalled:
                rec = DeadlockRecord(
                    cycle_robot_ids=cycle,
                    root_cause=f'Cyclic waiting loop detected: {" -> ".join(cycle)} -> {cycle[0]}',
                    persistence_duration_sec=round(duration, 2),
                    recovery_action='PENDING',
                    created_at=t_now,
                )
                confirmed_deadlocks.append(rec)

        return confirmed_deadlocks

    def reset_cycle(self, cycle: List[str]) -> None:
        """Clear cycle tracking after recovery action is applied."""
        canonical = tuple(cycle)
        self._cycle_history.pop(canonical, None)
        self._cycle_counts.pop(canonical, None)
        for r in cycle:
            self._robot_stalls[r] = 0

