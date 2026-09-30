"""Deterministic task generator for reproducible multi-AMR warehouse workloads."""

from dataclasses import dataclass, field
import math
import random
from typing import Any, Dict, List, Optional, Tuple

from amr_fleet_core.task_model import Task, TaskPriority


# Default Benchmark Stations (Matching warehouse_grid_small.yaml)
DEFAULT_PICKUP_STATIONS: List[Tuple[float, float]] = [
    (2.0, 2.0),
    (2.0, 13.0),
    (13.0, 2.0),
    (13.0, 13.0),
]

DEFAULT_DROPOFF_STATIONS: List[Tuple[float, float]] = [
    (7.0, 7.0),
    (7.0, 8.0),
    (8.0, 7.0),
    (8.0, 8.0),
]

# Static Warehouse Obstacle Bounding Boxes (Rack Obstacles with safety inflation)
# Form: (x_min, x_max, y_min, y_max)
DEFAULT_OBSTACLE_BOUNDS: List[Tuple[float, float, float, float]] = [
    # Rack 1 (center 4.5, 5.5, size 1.2 x 3.0) -> [3.7, 5.3, 3.8, 7.2]
    (3.7, 5.3, 3.8, 7.2),
    # Rack 2 (center 4.5, 10.5, size 1.2 x 3.0) -> [3.7, 5.3, 8.8, 12.2]
    (3.7, 5.3, 8.8, 12.2),
    # Rack 3 (center 11.5, 5.5, size 1.2 x 3.0) -> [10.7, 12.3, 3.8, 7.2]
    (10.7, 12.3, 3.8, 7.2),
    # Rack 4 (center 11.5, 10.5, size 1.2 x 3.0) -> [10.7, 12.3, 8.8, 12.2]
    (10.7, 12.3, 8.8, 12.2),
]


@dataclass
class TaskGeneratorConfig:
    """Configuration parameters for deterministic task generation."""

    task_count: int = 10
    seed: int = 42
    mode: str = 'STATION_PAIR'  # 'STATION_PAIR' or 'BOUNDED_RANDOM'
    id_prefix: str = 'task'
    base_time: float = 0.0

    # Priority distribution weights
    priority_weights: Dict[str, float] = field(default_factory=lambda: {
        'LOW': 0.15,
        'NORMAL': 0.55,
        'HIGH': 0.20,
        'CRITICAL': 0.10,
    })

    # Deadline configuration
    deadline_enabled: bool = True
    deadline_fraction: float = 1.0  # Fraction of tasks that receive deadlines
    deadline_window_sec: Tuple[float, float] = (30.0, 120.0)

    # Spatial configuration
    pickup_stations: List[Tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_PICKUP_STATIONS)
    )
    dropoff_stations: List[Tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_DROPOFF_STATIONS)
    )
    map_bounds: Tuple[float, float, float, float] = (0.5, 15.5, 0.5, 15.5)
    min_travel_distance: float = 1.5

    # Staged release configuration
    staged_releases: Optional[List[Dict[str, Any]]] = None


class TaskGenerator:
    """Generates reproducible, deterministic task sets using an isolated PRNG."""

    def __init__(self, config: Optional[TaskGeneratorConfig] = None) -> None:
        self.config = config if config is not None else TaskGeneratorConfig()
        self._rng = random.Random(self.config.seed)

    def set_seed(self, seed: int) -> None:
        """Reset the internal PRNG with a new seed."""
        self.config.seed = seed
        self._rng = random.Random(seed)

    def is_valid_location(self, x: float, y: float) -> bool:
        """Check if coordinates fall inside the map and outside obstacles."""
        x_min, x_max, y_min, y_max = self.config.map_bounds
        if not (x_min <= x <= x_max and y_min <= y <= y_max):
            return False

        for ox_min, ox_max, oy_min, oy_max in DEFAULT_OBSTACLE_BOUNDS:
            if ox_min <= x <= ox_max and oy_min <= y <= oy_max:
                return False

        return True

    def _select_priority(self) -> TaskPriority:
        """Sample priority according to configured weights."""
        priorities = list(self.config.priority_weights.keys())
        weights = [self.config.priority_weights[p] for p in priorities]
        choice_str = self._rng.choices(priorities, weights=weights, k=1)[0]
        return TaskPriority.from_str(choice_str)

    def _sample_deadline(self, created_at: float) -> Optional[float]:
        """Optionally sample a deadline within the configured window."""
        if not self.config.deadline_enabled:
            return None
        if self._rng.random() > self.config.deadline_fraction:
            return None
        min_dt, max_dt = self.config.deadline_window_sec
        offset = self._rng.uniform(min_dt, max_dt)
        return round(created_at + offset, 2)

    def generate_task(self, index: int, release_time_sec: float = 0.0) -> Task:
        """Generate a single deterministic task."""
        task_id = f'{self.config.id_prefix}_{index:04d}'
        created_at = self.config.base_time

        if self.config.mode == 'STATION_PAIR':
            # Select from discrete warehouse stations
            pickup = self._rng.choice(self.config.pickup_stations)
            dropoff = self._rng.choice(self.config.dropoff_stations)
        else:
            # Random coordinates with collision and distance rejection
            attempts = 0
            while attempts < 200:
                x_min, x_max, y_min, y_max = self.config.map_bounds
                px = round(self._rng.uniform(x_min, x_max), 2)
                py = round(self._rng.uniform(y_min, y_max), 2)
                dx = round(self._rng.uniform(x_min, x_max), 2)
                dy = round(self._rng.uniform(y_min, y_max), 2)

                dist = math.hypot(dx - px, dy - py)
                if (
                    dist >= self.config.min_travel_distance
                    and self.is_valid_location(px, py)
                    and self.is_valid_location(dx, dy)
                ):
                    pickup = (px, py)
                    dropoff = (dx, dy)
                    break
                attempts += 1
            else:
                # Fallback to station pair if random sampling failed
                pickup = self._rng.choice(self.config.pickup_stations)
                dropoff = self._rng.choice(self.config.dropoff_stations)

        priority = self._select_priority()
        deadline = self._sample_deadline(created_at)

        return Task(
            task_id=task_id,
            pickup=pickup,
            dropoff=dropoff,
            priority=priority,
            created_at=created_at,
            deadline=deadline,
            metadata={'generator_seed': self.config.seed, 'mode': self.config.mode},
            release_time_sec=release_time_sec,
        )

    def generate_workload(self, count: Optional[int] = None) -> List[Task]:
        """
        Generate a full deterministic list of tasks.

        :param count: Optional override for task count.
        :return: Deterministic list of unassigned Task entities.
        """
        num_tasks = count if count is not None else self.config.task_count
        # Reset PRNG state to ensure repeatability of workload generation
        self._rng = random.Random(self.config.seed)

        release_times = [0.0] * num_tasks
        if self.config.staged_releases:
            curr_idx = 0
            for stage in self.config.staged_releases:
                r_time = float(stage.get('release_time_sec', 0.0))
                r_count = int(stage.get('count', 0))
                for _ in range(r_count):
                    if curr_idx < num_tasks:
                        release_times[curr_idx] = r_time
                        curr_idx += 1

        tasks: List[Task] = []
        for i in range(1, num_tasks + 1):
            tasks.append(self.generate_task(i, release_time_sec=release_times[i - 1]))
        return tasks

