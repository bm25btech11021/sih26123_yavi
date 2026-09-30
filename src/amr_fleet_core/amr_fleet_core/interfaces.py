"""Abstract interfaces for modular AMR fleet coordination algorithms."""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple


class TaskAllocator(ABC):
    """Abstract interface for task allocation algorithms (Centralized, CBBA, ACBBA)."""

    @abstractmethod
    def allocate_tasks(
        self,
        robot_states: Dict[str, Any],
        tasks: List[Dict[str, Any]],
        epoch: int,
        network_state: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, List[str]], int]:
        """
        Allocate tasks to robots.

        :param robot_states: Mapping of robot_id to current state dict.
        :param tasks: List of unassigned task dicts.
        :param epoch: Current allocation epoch.
        :param network_state: Optional connectivity or message loss metadata.
        :return: Tuple of (assignments dict {robot_id: [task_ids]}, updated epoch).
        """
        pass


class GlobalPlanner(ABC):
    """Abstract interface for lifelong MAPF planners (RHCR, GD-RHCR)."""

    @abstractmethod
    def compute_plan(
        self,
        robot_states: Dict[str, Any],
        assigned_tasks: Dict[str, List[Dict[str, Any]]],
        map_data: Any,
        reservations: Optional[Dict[str, Any]] = None,
        config: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, List[Tuple[int, int, int]]], int, float]:
        """
        Compute multi-agent path plan over a time horizon.

        :param robot_states: Mapping of robot_id to state.
        :param assigned_tasks: Mapping of robot_id to list of tasks.
        :param map_data: Map / graph representation.
        :param reservations: Space-time reservations or conflict constraints.
        :param config: Planner parameters (horizon h, window w, timeout).
        :return: Tuple of (trajectories {robot_id: [(x, y, t)]}, plan_version, validity_interval).
        """
        pass


class LocalPlanner(ABC):
    """Abstract interface for local motion and fallback planners (PIBT, NH-ORCA)."""

    @abstractmethod
    def compute_step(
        self,
        robot_id: str,
        current_state: Any,
        active_plan: Optional[List[Tuple[int, int, int]]],
        neighbors: List[Dict[str, Any]],
        constraints: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float, str]:
        """
        Compute next local action or velocity command.

        :param robot_id: Identifier of the planning robot.
        :param current_state: Robot kinematic state.
        :param active_plan: Active trajectory slice if available.
        :param neighbors: Surrounding agents and obstacles.
        :param constraints: Safety bounds or dynamic constraints.
        :return: Tuple of (linear_velocity, angular_velocity, action_type).
        """
        pass


class DeadlockManager(ABC):
    """Abstract interface for distributed deadlock detection and recovery (WFG)."""

    @abstractmethod
    def update_dependencies(
        self,
        robot_id: str,
        waiting_for_robot_id: Optional[str],
        resource_id: str,
        epoch: int,
    ) -> None:
        """Register or update a resource wait-for relationship."""
        pass

    @abstractmethod
    def detect_cycles(self) -> List[List[str]]:
        """
        Detect cycles in the dependency graph.

        :return: List of cycles, where each cycle is a list of robot_ids.
        """
        pass

    @abstractmethod
    def resolve_deadlock(self, cycle: List[str]) -> Tuple[str, str]:
        """
        Select a victim agent and resolve a detected cycle.

        :param cycle: List of robot_ids forming the dependency cycle.
        :return: Tuple of (victim_robot_id, recovery_action).
        """
        pass


class MetricsCollector(ABC):
    """Abstract interface for experiment metrics and observability."""

    @abstractmethod
    def record_event(self, event_type: str, metadata: Dict[str, Any]) -> None:
        """Record a single discrete event."""
        pass

    @abstractmethod
    def record_latency(self, metric_name: str, latency_seconds: float) -> None:
        """Record latency measurement for percentile calculation."""
        pass

    @abstractmethod
    def get_summary(self) -> Dict[str, Any]:
        """Return aggregated summary metrics (P50, P95, P99, counts)."""
        pass

