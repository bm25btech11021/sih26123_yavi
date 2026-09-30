"""Discrete Space-Time Reservation Table for Multi-Agent Path Coordination (M6)."""

import threading
from typing import Dict, List, Optional, Set, Tuple

from amr_fleet_core.coordination_models import Conflict, ConflictType, Position, Reservation


class SpaceTimeReservationTable:
    """
    Thread-safe Discrete Space-Time Reservation Table.

    Maintains vertex reservations (cell, time) and edge reservations ((u, v), time)
    per robot with explicit ownership, priority arbitration, and rolling-horizon expiry.

    Synchronization Assumptions:
    - Coordination state is shared among local AMR decision loops (or synchronized
      via ROS 2 message broadcasts across the fleet).
    - An edge reservation from u to v at time t guarantees that no other robot can
      traverse v to u at time t (edge-swap prevention) and that no other robot can
      occupy v at time t+1 (vertex reservation).
    """

    def __init__(self, default_horizon_window: int = 15) -> None:
        """Initialize reservation table structures and thread lock."""
        self._lock = threading.RLock()
        self.horizon_window = default_horizon_window
        # Map: (Position, time_step) -> Reservation
        self._vertex_reservations: Dict[Tuple[Position, int], Reservation] = {}
        # Map: (Position, Position, time_step) -> Reservation
        self._edge_reservations: Dict[Tuple[Position, Position, int], Reservation] = {}
        # Map: robot_id -> Set[Tuple[Position, int]]
        self._robot_vertices: Dict[str, Set[Tuple[Position, int]]] = {}
        # Map: robot_id -> Set[Tuple[Position, Position, int]]
        self._robot_edges: Dict[str, Set[Tuple[Position, Position, int]]] = {}

    def reserve(
        self,
        cell: Position,
        time_step: int,
        robot_id: str,
        priority: float = 0.0,
        duration_sec: float = 1.0,
    ) -> bool:
        """
        Reserve a vertex cell at a specific discrete time step.

        Returns True if reservation succeeded, False if already occupied by a peer.
        """
        with self._lock:
            key = (cell, time_step)
            existing = self._vertex_reservations.get(key)
            if existing is not None:
                if existing.robot_id == robot_id:
                    return True
                return False

            res = Reservation(
                robot_id=robot_id,
                from_pos=cell,
                to_pos=cell,
                time_step=time_step,
                duration_sec=duration_sec,
                is_edge=False,
                priority=priority,
            )
            self._vertex_reservations[key] = res
            self._robot_vertices.setdefault(robot_id, set()).add(key)
            return True

    def reserve_edge(
        self,
        from_pos: Position,
        to_pos: Position,
        time_step: int,
        robot_id: str,
        priority: float = 0.0,
        duration_sec: float = 1.0,
    ) -> bool:
        """
        Reserve a directed traversal from from_pos to to_pos across [time_step, time_step+1].

        Guarantees:
        1. Target vertex to_pos at time_step+1 is reserved.
        2. Reverse edge (to_pos -> from_pos) at time_step is checked to prevent edge swap.
        """
        with self._lock:
            # 1. Edge-swap check: did peer reserve to_pos -> from_pos at time_step?
            rev_key = (to_pos, from_pos, time_step)
            rev_res = self._edge_reservations.get(rev_key)
            if rev_res is not None and rev_res.robot_id != robot_id:
                return False

            # 2. Vertex conflict check at arrival time_step + 1
            v_key = (to_pos, time_step + 1)
            v_res = self._vertex_reservations.get(v_key)
            if v_res is not None and v_res.robot_id != robot_id:
                return False

            # Commit edge reservation
            edge_key = (from_pos, to_pos, time_step)
            res = Reservation(
                robot_id=robot_id,
                from_pos=from_pos,
                to_pos=to_pos,
                time_step=time_step,
                duration_sec=duration_sec,
                is_edge=True,
                priority=priority,
            )
            self._edge_reservations[edge_key] = res
            self._robot_edges.setdefault(robot_id, set()).add(edge_key)

            # Commit vertex reservation at time_step + 1
            v_reservation = Reservation(
                robot_id=robot_id,
                from_pos=to_pos,
                to_pos=to_pos,
                time_step=time_step + 1,
                duration_sec=duration_sec,
                is_edge=False,
                priority=priority,
            )
            self._vertex_reservations[v_key] = v_reservation
            self._robot_vertices.setdefault(robot_id, set()).add(v_key)
            return True

    def is_reserved(self, cell: Position, time_step: int, robot_id: str = '') -> bool:
        """Check if vertex is reserved by a robot other than robot_id."""
        with self._lock:
            res = self._vertex_reservations.get((cell, time_step))
            if res is None:
                return False
            if robot_id and res.robot_id == robot_id:
                return False
            return True

    def is_headway_conflict(
        self,
        from_pos: Position,
        to_pos: Position,
        time_step: int,
        robot_id: str = '',
        min_headway_cells: int = 2,
    ) -> bool:
        """
        Check if moving from from_pos to to_pos violates longitudinal headway against a lead robot.

        In a 0.5m grid with AMR chassis length 0.65m, adjacent cells along travel vector
        yield nominal bumper clearance of -0.15m (overlap). Minimum headway of 2 cells
        guarantees nominal center distance >= 1.0m and clearance >= 0.35m.
        """
        with self._lock:
            dx = to_pos[0] - from_pos[0]
            dy = to_pos[1] - from_pos[1]
            if dx == 0 and dy == 0:
                return False

            for step in range(1, min_headway_cells):
                ahead_cell = (to_pos[0] + dx * step, to_pos[1] + dy * step)
                res = self._vertex_reservations.get((ahead_cell, time_step))
                if res is not None and (not robot_id or res.robot_id != robot_id):
                    return True
            return False

    def get_headway_conflict(
        self,
        from_pos: Position,
        to_pos: Position,
        time_step: int,
        robot_id: str,
        min_headway_cells: int = 2,
    ) -> Optional[Conflict]:
        """Inspect if traversal violates longitudinal headway against a lead robot."""
        with self._lock:
            dx = to_pos[0] - from_pos[0]
            dy = to_pos[1] - from_pos[1]
            if dx == 0 and dy == 0:
                return None

            for step in range(1, min_headway_cells):
                ahead_cell = (to_pos[0] + dx * step, to_pos[1] + dy * step)
                res = self._vertex_reservations.get((ahead_cell, time_step))
                if res is not None and (not robot_id or res.robot_id != robot_id):
                    return Conflict(
                        conflict_type=ConflictType.VERTEX.value,
                        robot_a=robot_id,
                        robot_b=res.robot_id,
                        cell=to_pos,
                        time_step=time_step,
                        details=(
                            f'{robot_id} headway buffer conflict behind {res.robot_id} '
                            f'at {to_pos} (lead at {ahead_cell})'
                        ),
                    )
            return None

    def is_edge_conflict(
        self,
        from_pos: Position,
        to_pos: Position,
        time_step: int,
        robot_id: str = '',
    ) -> bool:
        """Check if an edge swap exists against to_pos -> from_pos at time_step."""
        with self._lock:
            rev_key = (to_pos, from_pos, time_step)
            res = self._edge_reservations.get(rev_key)
            if res is None:
                return False
            if robot_id and res.robot_id == robot_id:
                return False
            return True

    def get_reservation(self, cell: Position, time_step: int) -> Optional[Reservation]:
        """Return reservation at (cell, time_step) if present, else None."""
        with self._lock:
            return self._vertex_reservations.get((cell, time_step))

    def get_authoritative_owners(
        self, cell: Position, time_step: int,
    ) -> Set[str]:
        """Return the set of robot IDs that authoritatively own (cell, time_step)."""
        with self._lock:
            res = self._vertex_reservations.get((cell, time_step))
            if res is not None and res.robot_id:
                return {res.robot_id}
            return set()

    def get_owner(self, cell: Position, time_step: int) -> Optional[str]:
        """Return authoritative robot owner of (cell, time_step), or None if free."""
        with self._lock:
            res = self._vertex_reservations.get((cell, time_step))
            return res.robot_id if res is not None else None

    def is_owner(self, cell: Position, time_step: int, robot_id: str) -> bool:
        """Check if robot_id is the authoritative owner of (cell, time_step)."""
        with self._lock:
            res = self._vertex_reservations.get((cell, time_step))
            return bool(res is not None and res.robot_id == robot_id)

    def get_conflict(
        self,
        from_pos: Position,
        to_pos: Position,
        time_step: int,
        robot_id: str,
    ) -> Optional[Conflict]:
        """
        Inspect potential conflict if robot_id attempts traversal from_pos -> to_pos at time_step.

        Returns Conflict object if a conflict is detected, else None.
        """
        with self._lock:
            # Check edge swap
            rev_key = (to_pos, from_pos, time_step)
            edge_res = self._edge_reservations.get(rev_key)
            if edge_res is not None and edge_res.robot_id != robot_id:
                return Conflict(
                    conflict_type=ConflictType.EDGE_SWAP.value,
                    robot_a=robot_id,
                    robot_b=edge_res.robot_id,
                    cell=to_pos,
                    time_step=time_step,
                    details=(
                        f'{robot_id} ({from_pos}->{to_pos}) swaps with '
                        f'{edge_res.robot_id} ({to_pos}->{from_pos})'
                    ),
                )

            # Check vertex conflict at t+1
            v_key = (to_pos, time_step + 1)
            v_res = self._vertex_reservations.get(v_key)
            if v_res is not None and v_res.robot_id != robot_id:
                return Conflict(
                    conflict_type=ConflictType.VERTEX.value,
                    robot_a=robot_id,
                    robot_b=v_res.robot_id,
                    cell=to_pos,
                    time_step=time_step + 1,
                    details=(
                        f'{robot_id} and {v_res.robot_id} contest cell '
                        f'{to_pos} at t={time_step+1}'
                    ),
                )

            return None

    def release(self, robot_id: str) -> None:
        """Release all reservations owned by robot_id."""
        with self._lock:
            for v_key in self._robot_vertices.pop(robot_id, set()):
                self._vertex_reservations.pop(v_key, None)
            for e_key in self._robot_edges.pop(robot_id, set()):
                self._edge_reservations.pop(e_key, None)

    def release_robot(self, robot_id: str) -> None:
        """Alias for release(robot_id)."""
        self.release(robot_id)

    def release_time_before(self, min_time_step: int) -> int:
        """Expire past reservations for rolling-horizon window progression."""
        with self._lock:
            pruned_count = 0
            v_keys_to_del = [k for k in self._vertex_reservations if k[1] < min_time_step]
            for vk in v_keys_to_del:
                res = self._vertex_reservations.pop(vk)
                if res.robot_id in self._robot_vertices:
                    self._robot_vertices[res.robot_id].discard(vk)
                pruned_count += 1

            e_keys_to_del = [k for k in self._edge_reservations if k[2] < min_time_step]
            for ek in e_keys_to_del:
                res = self._edge_reservations.pop(ek)
                if res.robot_id in self._robot_edges:
                    self._robot_edges[res.robot_id].discard(ek)
                pruned_count += 1

            return pruned_count

    def get_reservations_for_robot(self, robot_id: str) -> List[Reservation]:
        """Return all active reservations owned by robot_id."""
        with self._lock:
            v_keys = self._robot_vertices.get(robot_id, set())
            e_keys = self._robot_edges.get(robot_id, set())
            out: List[Reservation] = []
            for vk in sorted(v_keys, key=lambda k: k[1]):
                if vk in self._vertex_reservations:
                    out.append(self._vertex_reservations[vk])
            for ek in sorted(e_keys, key=lambda k: k[2]):
                if ek in self._edge_reservations:
                    out.append(self._edge_reservations[ek])
            return out

    def get_all_vertex_reservations(self) -> Dict[Tuple[Position, int], Reservation]:
        """Return shallow copy of all active vertex reservations."""
        with self._lock:
            return dict(self._vertex_reservations)

    def get_all_edge_reservations(self) -> Dict[Tuple[Position, Position, int], Reservation]:
        """Return shallow copy of all active edge reservations."""
        with self._lock:
            return dict(self._edge_reservations)

    def invalidate_cells(
        self,
        cells: Set[Position],
        min_time_step: int = 0,
    ) -> List[Tuple[str, Position, int]]:
        """
        Withdraw any active reservations intersecting the specified grid cells.

        Called when dynamic obstacles or environmental aisle blockages withdraw
        cells from the traversable graph (Milestone 3).

        :param cells: Set of grid cell positions withdrawn from traversability.
        :param min_time_step: Earliest time step to consider for revocation.
        :return: List of revoked (robot_id, cell, time_step) tuples.
        """
        with self._lock:
            revoked: List[Tuple[str, Position, int]] = []

            # Invalidate vertex reservations
            v_keys_to_del = [
                k for k in self._vertex_reservations
                if k[0] in cells and k[1] >= min_time_step
            ]
            for vk in v_keys_to_del:
                res = self._vertex_reservations.pop(vk)
                if res.robot_id in self._robot_vertices:
                    self._robot_vertices[res.robot_id].discard(vk)
                revoked.append((res.robot_id, vk[0], vk[1]))

            # Invalidate edge reservations intersecting cells
            e_keys_to_del = [
                k for k in self._edge_reservations
                if (k[0] in cells or k[1] in cells) and k[2] >= min_time_step
            ]
            for ek in e_keys_to_del:
                res = self._edge_reservations.pop(ek)
                if res.robot_id in self._robot_edges:
                    self._robot_edges[res.robot_id].discard(ek)
                revoked.append((res.robot_id, ek[1], ek[2]))

            return revoked

