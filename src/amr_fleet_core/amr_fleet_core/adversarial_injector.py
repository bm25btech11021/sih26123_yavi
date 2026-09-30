"""
Dedicated Adversarial Conflict Injector for YAVI-SIH26123 Milestone 3.

This module is strictly a test instrumentation component. Its sole responsibility
is to create controlled collision-risk conditions across multi-AMR fleets for
evaluating decentralized coordination, reservation arbitration, and local safety.

Architectural Non-Invasiveness Invariant:
- The injector contains ZERO recovery logic.
- It never calls CBBA task allocation.
- It never executes A* or RHCR path replanning.
- It never executes PIBT fallback resolution.
- It never commands emergency braking directly as a recovery action.
"""

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import Any, Dict, List, Tuple

from amr_fleet_core.coordination_models import Position
from amr_fleet_core.reservation_table import SpaceTimeReservationTable


class AdversarialConflictType(Enum):
    """Enumeration of controlled adversarial conflict injection types."""

    SAME_CELL = 'SAME_CELL'
    OPPOSING_CORRIDOR = 'OPPOSING_CORRIDOR'
    CROSSING_TRAJECTORIES = 'CROSSING_TRAJECTORIES'
    RESERVATION_CONFLICT = 'RESERVATION_CONFLICT'
    HIGH_CONTENTION_INTERSECTION = 'HIGH_CONTENTION_INTERSECTION'
    COMPOUND_FAULT = 'COMPOUND_FAULT'


@dataclass
class InjectedConflict:
    """Record of an injected adversarial conflict condition."""

    conflict_id: str
    conflict_type: AdversarialConflictType
    robot_ids: List[str]
    location: Position
    time_step: int
    creation_time: float
    details: str
    parameters: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert conflict record to dictionary representation."""
        return {
            'conflict_id': self.conflict_id,
            'conflict_type': self.conflict_type.value,
            'robot_ids': self.robot_ids,
            'location': list(self.location),
            'time_step': self.time_step,
            'creation_time': self.creation_time,
            'details': self.details,
            'parameters': self.parameters,
        }


class AdversarialConflictInjector:
    """
    Dedicated Adversarial Conflict Injector for Milestone 3 testing.

    Creates collision-risk test conditions in a controlled, reproducible manner
    using fixed initial conditions and target coordinates.
    """

    def __init__(self) -> None:
        """Initialize conflict history and identifier counter."""
        self._history: List[InjectedConflict] = []
        self._counter: int = 0

    def _next_id(self, prefix: str) -> str:
        """Generate unique conflict record identifier."""
        self._counter += 1
        return f'{prefix}_{self._counter:04d}'

    def inject_same_cell_conflict(
        self,
        robot_a: str,
        robot_b: str,
        contested_cell: Position,
        time_step: int,
    ) -> InjectedConflict:
        """
        Inject a same-cell vertex contention condition (M3-C1).

        Assigns two robots to converge on the identical vertex cell at the
        same discrete planning time-step.
        """
        cid = self._next_id('CNF_SAME_CELL')
        record = InjectedConflict(
            conflict_id=cid,
            conflict_type=AdversarialConflictType.SAME_CELL,
            robot_ids=[robot_a, robot_b],
            location=contested_cell,
            time_step=time_step,
            creation_time=time.time(),
            details=f'Both {robot_a} and {robot_b} target cell {contested_cell} at t={time_step}',
            parameters={'contested_cell': contested_cell, 'arrival_step': time_step},
        )
        self._history.append(record)
        return record

    def inject_opposing_corridor_entry(
        self,
        robot_a: str,
        robot_b: str,
        corridor_cells: List[Position],
        entry_step: int = 0,
    ) -> InjectedConflict:
        """
        Inject opposing aisle entry conflict in single-track corridor (M3-C2).

        Dispatches robot_a from one end of a 1-cell wide corridor and robot_b
        from the opposite end, creating an unavoidable head-on deadlock / swap.
        """
        cid = self._next_id('CNF_OPPOSING_CORRIDOR')
        midpoint = corridor_cells[len(corridor_cells) // 2] if corridor_cells else (0, 0)
        record = InjectedConflict(
            conflict_id=cid,
            conflict_type=AdversarialConflictType.OPPOSING_CORRIDOR,
            robot_ids=[robot_a, robot_b],
            location=midpoint,
            time_step=entry_step,
            creation_time=time.time(),
            details=(
                f'{robot_a} (entry={corridor_cells[0]}) and {robot_b} '
                f'(entry={corridor_cells[-1]}) entering corridor from opposite ends'
            ),
            parameters={
                'corridor_cells': corridor_cells,
                'entry_step': entry_step,
                'corridor_length': len(corridor_cells),
            },
        )
        self._history.append(record)
        return record

    def inject_crossing_conflict(
        self,
        robot_a: str,
        robot_b: str,
        intersection_cell: Position,
        arrival_step: int,
    ) -> InjectedConflict:
        """
        Inject crossing perpendicular trajectories conflict (M3-C3).

        Configures horizontal and vertical paths intersecting at intersection_cell
        with synchronized arrival time steps.
        """
        cid = self._next_id('CNF_CROSSING')
        record = InjectedConflict(
            conflict_id=cid,
            conflict_type=AdversarialConflictType.CROSSING_TRAJECTORIES,
            robot_ids=[robot_a, robot_b],
            location=intersection_cell,
            time_step=arrival_step,
            creation_time=time.time(),
            details=(
                f'Perpendicular intersection at {intersection_cell} '
                f'between {robot_a} and {robot_b} at t={arrival_step}'
            ),
            parameters={
                'intersection_cell': intersection_cell,
                'arrival_step': arrival_step,
            },
        )
        self._history.append(record)
        return record

    def inject_reservation_conflict(
        self,
        res_table: SpaceTimeReservationTable,
        robot_a: str,
        robot_b: str,
        cell: Position,
        time_step: int,
        priority_b: float = 1.0,
    ) -> Tuple[InjectedConflict, bool]:
        """
        Inject an intentional reservation collision into SpaceTimeReservationTable (M3-C4).

        Attempts to force-reserve a cell already held by robot_a for robot_b.
        Returns the conflict record and whether the reservation table permitted it.
        """
        cid = self._next_id('CNF_RESERVATION')
        # Ensure robot_a holds the initial reservation
        res_table.reserve(cell=cell, time_step=time_step, robot_id=robot_a, priority=2.0)

        # Attempt to insert conflicting reservation for robot_b
        # We record the table's rejection/acceptance
        was_accepted = res_table.reserve(
            cell=cell, time_step=time_step, robot_id=robot_b, priority=priority_b,
        )

        record = InjectedConflict(
            conflict_id=cid,
            conflict_type=AdversarialConflictType.RESERVATION_CONFLICT,
            robot_ids=[robot_a, robot_b],
            location=cell,
            time_step=time_step,
            creation_time=time.time(),
            details=(
                f'Injected duplicate reservation for {robot_b} on {cell} '
                f'at t={time_step} (held by {robot_a}, accepted={was_accepted})'
            ),
            parameters={
                'cell': cell,
                'time_step': time_step,
                'owner': robot_a,
                'contender': robot_b,
                'accepted': was_accepted,
            },
        )
        self._history.append(record)
        return record, was_accepted

    def inject_intersection_contention(
        self,
        robots: List[str],
        intersection_center: Position,
        arrival_step: int,
    ) -> InjectedConflict:
        """
        Inject multi-agent (3+ AMR) high-contention arrival at intersection (M3-C5).

        Dispatches 3 or more AMRs toward a 4-way intersection simultaneously.
        """
        cid = self._next_id('CNF_HIGH_CONTENTION')
        record = InjectedConflict(
            conflict_id=cid,
            conflict_type=AdversarialConflictType.HIGH_CONTENTION_INTERSECTION,
            robot_ids=list(robots),
            location=intersection_center,
            time_step=arrival_step,
            creation_time=time.time(),
            details=(
                f'{len(robots)} AMRs {robots} simultaneously converging on '
                f'intersection {intersection_center} at t={arrival_step}'
            ),
            parameters={
                'robots': robots,
                'intersection_center': intersection_center,
                'arrival_step': arrival_step,
            },
        )
        self._history.append(record)
        return record

    def inject_compound_fault(
        self,
        scenario_id: str,
        fault_components: List[Dict[str, Any]],
        start_time: float,
        location: Position = (0, 0),
        time_step: int = 0,
    ) -> InjectedConflict:
        """
        Inject a compound failure condition comprising multiple simultaneous/overlapping faults.

        :param scenario_id: Identifier of the compound scenario (e.g. 'M4-A', 'M4-G')
        :param fault_components: List of component fault descriptors
        :param start_time: Epoch timestamp of scenario initiation
        :param location: Primary spatial epicenter or (0, 0)
        :param time_step: Discrete planning step
        :return: InjectedConflict record
        """
        cid = self._next_id(f'CNF_COMPOUND_{scenario_id}')
        involved_robots = sorted({
            fc['robot_id'] for fc in fault_components if 'robot_id' in fc
        })
        record = InjectedConflict(
            conflict_id=cid,
            conflict_type=AdversarialConflictType.COMPOUND_FAULT,
            robot_ids=involved_robots,
            location=location,
            time_step=time_step,
            creation_time=start_time,
            details=(
                f'Compound fault {scenario_id} with {len(fault_components)} '
                f'overlapping failure components: {[fc.get("type") for fc in fault_components]}'
            ),
            parameters={
                'scenario_id': scenario_id,
                'fault_components': fault_components,
            },
        )
        self._history.append(record)
        return record

    def get_history(self) -> List[InjectedConflict]:
        """Return shallow copy of all injected conflict records."""
        return list(self._history)

    def clear_history(self) -> None:
        """Clear recorded conflict history."""
        self._history.clear()

