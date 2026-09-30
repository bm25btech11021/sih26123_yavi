"""Rolling-Horizon Task Planner (RHCR Core Engine) for YAVI-SIH26123 AMR Fleet."""

from dataclasses import dataclass, field
import heapq
import math
import time
from typing import Any, Dict, List, Optional, Set, Tuple

from amr_fleet_sim.grid_world import GridWorld, Position


@dataclass
class RHConfig:
    """Configuration parameters for the Rolling-Horizon Task Planner."""

    horizon_steps: int = 10
    execution_window: int = 4
    replan_rate: float = 2.0
    sequencing_heuristic: str = 'PRIORITY_FIRST'
    grid_resolution: float = 0.5
    goal_tolerance_m: float = 0.5
    max_tasks_per_horizon: int = 4


@dataclass
class PlanningResponseData:
    """Internal structured response from the path planning interface."""

    success: bool
    status_message: str
    robot_id: str
    task_id: str
    sub_goal_type: str
    start_pose: Tuple[float, float]
    goal_pose: Tuple[float, float]
    full_path: List[Tuple[float, float]] = field(default_factory=list)
    horizon_path: List[Tuple[float, float]] = field(default_factory=list)
    execution_path: List[Tuple[float, float]] = field(default_factory=list)
    total_cost: float = 0.0
    planning_latency_ms: float = 0.0
    horizon_steps: int = 10
    execution_window: int = 4
    replan_count: int = 0


class SingleAgentAStar:
    """Deterministic 4-connected grid A* pathfinder on GridWorld."""

    def __init__(self, grid: GridWorld) -> None:
        self.grid = grid

    def find_path(
        self,
        start_grid: Position,
        goal_grid: Position,
    ) -> Optional[List[Position]]:
        """
        Compute the shortest collision-free grid path from start to goal.

        Deterministic tie-breaking is enforced by (f_score, h_score, counter, pos).
        """
        if not self.grid.in_bounds(start_grid) or not self.grid.in_bounds(goal_grid):
            return None

        actual_start = start_grid
        if not self.grid.is_free(actual_start):
            neighbors = self.grid.get_neighbors(actual_start, allow_wait=False)
            if not neighbors:
                found = False
                for r in range(1, 4):
                    for dx in range(-r, r + 1):
                        for dy in range(-r, r + 1):
                            cand = (actual_start[0] + dx, actual_start[1] + dy)
                            if self.grid.is_free(cand):
                                actual_start = cand
                                found = True
                                break
                        if found:
                            break
                    if found:
                        break
                if not found:
                    return None
            else:
                actual_start = min(
                    neighbors,
                    key=lambda p: (
                        self.grid.manhattan_distance(p, goal_grid),
                        p[0],
                        p[1],
                    ),
                )

        actual_goal = goal_grid
        if not self.grid.is_free(actual_goal):
            neighbors = self.grid.get_neighbors(actual_goal, allow_wait=False)
            if not neighbors:
                return None
            actual_goal = min(
                neighbors,
                key=lambda p: (
                    self.grid.manhattan_distance(p, actual_start),
                    p[0],
                    p[1],
                ),
            )

        if actual_start == actual_goal:
            return [actual_start]

        open_set: List[Tuple[int, int, int, Position]] = []
        counter = 0

        h_start = self.grid.manhattan_distance(actual_start, actual_goal)
        heapq.heappush(open_set, (h_start, h_start, counter, actual_start))

        came_from: Dict[Position, Position] = {}
        g_score: Dict[Position, int] = {actual_start: 0}
        closed_set: Set[Position] = set()

        while open_set:
            _, _, _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            closed_set.add(current)

            if current == actual_goal:
                path = [current]
                curr = current
                while curr in came_from:
                    curr = came_from[curr]
                    path.append(curr)
                path.reverse()
                return path

            current_g = g_score[current]
            for neighbor in self.grid.get_neighbors(current, allow_wait=False):
                if neighbor in closed_set:
                    continue

                tentative_g = current_g + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    h_val = self.grid.manhattan_distance(neighbor, actual_goal)
                    f_val = tentative_g + h_val
                    came_from[neighbor] = current
                    counter += 1
                    heapq.heappush(open_set, (f_val, h_val, counter, neighbor))

        return None


class TaskSequencer:
    """
    Determines execution sequence of tasks within a robot's assigned CBBA bundle.

    Invariant: 100% of tasks in the assigned bundle are preserved.
    Zero tasks are dropped or reassigned to other robots.
    """

    @staticmethod
    def sequence(
        bundle: List[str],
        tasks_map: Dict[str, Dict[str, Any]],
        robot_pos: Tuple[float, float],
        heuristic: str = 'PRIORITY_FIRST',
    ) -> List[str]:
        """Sequence tasks according to the specified ordering heuristic."""
        if not bundle:
            return []

        valid_tasks = [t for t in bundle if t in tasks_map]
        if not valid_tasks:
            return []

        if heuristic == 'BUNDLE_ORDER':
            return list(valid_tasks)

        if heuristic == 'PRIORITY_FIRST':
            def sort_key(t_id: str):
                t = tasks_map[t_id]
                pri = t.get('priority', 2)
                pickup = t.get('pickup', (0.0, 0.0))
                dist = math.hypot(pickup[0] - robot_pos[0], pickup[1] - robot_pos[1])
                return (-pri, round(dist, 3), t_id)

            return sorted(valid_tasks, key=sort_key)

        if heuristic == 'SHORTEST_PATH_FIRST':
            remaining = list(valid_tasks)
            ordered: List[str] = []
            curr_pos = robot_pos

            while remaining:
                next_t = min(
                    remaining,
                    key=lambda tid: (
                        math.hypot(
                            tasks_map[tid]['pickup'][0] - curr_pos[0],
                            tasks_map[tid]['pickup'][1] - curr_pos[1],
                        ),
                        -tasks_map[tid].get('priority', 2),
                        tid,
                    ),
                )
                ordered.append(next_t)
                remaining.remove(next_t)
                curr_pos = tasks_map[next_t].get('dropoff', curr_pos)

            return ordered

        if heuristic == 'DEADLINE_FIRST':
            def sort_key_deadline(t_id: str):
                t = tasks_map[t_id]
                dl = t.get('deadline')
                deadline_val = float(dl) if dl is not None else float('inf')
                pri = t.get('priority', 2)
                return (deadline_val, -pri, t_id)

            return sorted(valid_tasks, key=sort_key_deadline)

        return list(valid_tasks)


class RollingHorizonPlanner:
    """
    Rolling-Horizon Collision-Resolution Task Planner (M5 RHCR Engine).

    Maintains active task bundle, sub-goal progression, single-agent A* routes,
    horizon truncation (h), execution window selection (w), and replanning triggers.
    """

    def __init__(
        self,
        robot_id: str,
        config: Optional[RHConfig] = None,
        grid: Optional[GridWorld] = None,
    ) -> None:
        self.robot_id = robot_id
        self.config = config or RHConfig()
        self.grid = grid or GridWorld.create_warehouse_grid(
            resolution=self.config.grid_resolution,
        )
        self.astar = SingleAgentAStar(self.grid)

        # State
        self.current_position: Tuple[float, float] = (0.0, 0.0)
        self.assigned_bundle: List[str] = []
        self.ordered_tasks: List[str] = []
        self.tasks_map: Dict[str, Dict[str, Any]] = {}
        self.active_task_idx: int = 0
        self.active_phase: str = 'IDLE'
        self.current_goal: Optional[Tuple[float, float]] = None

        # Planning Paths
        self.full_path: List[Tuple[float, float]] = []
        self.horizon_path: List[Tuple[float, float]] = []
        self.execution_path: List[Tuple[float, float]] = []

        # Telemetry & Metrics
        self.replan_count: int = 0
        self.steps_executed_in_window: int = 0
        self.active_waypoint_idx: int = 1
        self.last_latency_ms: float = 0.0
        self.last_plan_success: bool = False
        self.total_distance_planned: float = 0.0
        self.last_progress_time: float = time.time()

    def update_position(self, pos: Tuple[float, float]) -> None:
        """Update localized robot position from odometry."""
        self.current_position = (round(pos[0], 3), round(pos[1], 3))

    def update_assigned_bundle(
        self,
        bundle: List[str],
        tasks_map: Dict[str, Dict[str, Any]],
    ) -> bool:
        """
        Ingest assigned tasks from M4 CBBA.

        Preserves currently executing tasks without interruption when new tasks
        are appended or bundle is expanded dynamically.
        Returns True if the bundle contents changed.
        """
        bundle_changed = set(bundle) != set(self.assigned_bundle)
        self.assigned_bundle = list(bundle)
        self.tasks_map = dict(tasks_map)

        if not bundle_changed and (not self.assigned_bundle or self.ordered_tasks):
            return False

        # If robot is actively executing an in-progress task, preserve it without interruption
        # unless it was explicitly cancelled, failed, or requeued
        if (
            self.active_phase in ('TRANSIT_TO_PICKUP', 'TRANSIT_TO_DROPOFF')
            and self.ordered_tasks
            and self.active_task_idx < len(self.ordered_tasks)
        ):
            curr_tid = self.ordered_tasks[self.active_task_idx]
            curr_status = self.tasks_map.get(curr_tid, {}).get('status', '')

            is_purged = (
                curr_tid not in self.assigned_bundle
                or curr_status in ('CANCELLED', 'FAILED', 'PENDING')
            )
            if is_purged:
                # Active task was cancelled or requeued: safely transition away from it
                completed_tasks = list(self.ordered_tasks[:self.active_task_idx])
                future_tasks = [
                    t for t in self.assigned_bundle
                    if t != curr_tid and t not in completed_tasks
                    and self.tasks_map.get(t, {}).get('status') not in (
                        'COMPLETED', 'CANCELLED', 'FAILED'
                    )
                ]
                sequenced_future = TaskSequencer.sequence(
                    future_tasks,
                    self.tasks_map,
                    self.current_position,
                    heuristic=self.config.sequencing_heuristic,
                )
                self.ordered_tasks = completed_tasks + sequenced_future
                if self.active_task_idx < len(self.ordered_tasks):
                    next_tid = self.ordered_tasks[self.active_task_idx]
                    self.active_phase = 'TRANSIT_TO_PICKUP'
                    self.current_goal = self.tasks_map[next_tid]['pickup']
                    self.replan()
                else:
                    self.active_phase = 'IDLE'
                    self.current_goal = None
                    self.full_path = []
                    self.horizon_path = []
                    self.execution_path = []
                return True

            completed_tasks = list(self.ordered_tasks[:self.active_task_idx])
            # Unstarted / newly added tasks in bundle
            future_tasks = [
                t for t in self.assigned_bundle
                if t != curr_tid and t not in completed_tasks
            ]
            if curr_tid in self.tasks_map:
                anchor_pos = self.tasks_map[curr_tid].get('dropoff', self.current_position)
            else:
                anchor_pos = self.current_position

            sequenced_future = TaskSequencer.sequence(
                future_tasks,
                self.tasks_map,
                anchor_pos,
                heuristic=self.config.sequencing_heuristic,
            )
            self.ordered_tasks = completed_tasks + [curr_tid] + sequenced_future
            # Keep active_task_idx, active_phase, and current_goal untouched
            return True

        # Robot was IDLE or uninitialized: sequence entire bundle
        uncompleted_bundle = [
            t for t in self.assigned_bundle
            if t in self.tasks_map and self.tasks_map[t].get('status') != 'COMPLETED'
        ] or list(self.assigned_bundle)

        self.ordered_tasks = TaskSequencer.sequence(
            uncompleted_bundle,
            self.tasks_map,
            self.current_position,
            heuristic=self.config.sequencing_heuristic,
        )
        self.active_task_idx = 0
        if self.ordered_tasks and self.ordered_tasks[0] in self.tasks_map:
            self.active_phase = 'TRANSIT_TO_PICKUP'
            first_t = self.tasks_map[self.ordered_tasks[0]]
            self.current_goal = first_t.get('pickup')
        else:
            self.active_phase = 'IDLE'
            self.current_goal = None
        self.replan()
        return True

    def check_subgoal_arrival(self) -> bool:
        """Check whether the robot has arrived at its active sub-goal."""
        if not self.current_goal or self.active_phase == 'IDLE':
            return False

        dist = math.hypot(
            self.current_position[0] - self.current_goal[0],
            self.current_position[1] - self.current_goal[1],
        )
        return dist <= self.config.goal_tolerance_m

    def advance_subgoal(self) -> str:
        """
        Advance task sub-goal lifecycle.

        TRANSIT_TO_PICKUP -> TRANSIT_TO_DROPOFF -> Next Task -> IDLE.
        Returns completed event ('PICKUP_REACHED', 'TASK_COMPLETED', 'ALL_COMPLETED').
        """
        if self.active_task_idx >= len(self.ordered_tasks):
            self.active_phase = 'IDLE'
            self.current_goal = None
            return 'ALL_COMPLETED'

        curr_tid = self.ordered_tasks[self.active_task_idx]
        task = self.tasks_map.get(curr_tid)
        if not task:
            self.active_phase = 'IDLE'
            self.current_goal = None
            return 'ALL_COMPLETED'

        if self.active_phase == 'TRANSIT_TO_PICKUP':
            self.active_phase = 'TRANSIT_TO_DROPOFF'
            self.current_goal = task['dropoff']
            self.replan()
            return 'PICKUP_REACHED'

        if self.active_phase == 'TRANSIT_TO_DROPOFF':
            self.active_task_idx += 1
            if self.active_task_idx < len(self.ordered_tasks):
                next_tid = self.ordered_tasks[self.active_task_idx]
                next_task = self.tasks_map.get(next_tid)
                if next_task:
                    self.active_phase = 'TRANSIT_TO_PICKUP'
                    self.current_goal = next_task['pickup']
                    self.replan()
                    return 'TASK_COMPLETED'

            self.active_phase = 'IDLE'
            self.current_goal = None
            self.full_path = []
            self.horizon_path = []
            self.execution_path = []
            return 'ALL_COMPLETED'

        return 'NONE'

    def check_replan_triggers(self) -> bool:
        """
        Check deterministic replanning triggers.

        Triggers:
        1. Sub-goal arrival.
        2. Execution window exhausted (steps_executed >= execution_window).
        3. Active path empty while tasks remain.
        """
        if self.check_subgoal_arrival():
            return True

        if self.active_phase != 'IDLE':
            if not self.full_path or not self.execution_path:
                return True
            if self.steps_executed_in_window >= self.config.execution_window:
                return True
            if self.active_waypoint_idx >= len(self.execution_path):
                return True
            if time.time() - self.last_progress_time > 3.0:
                return True

        return False

    def replan(self) -> PlanningResponseData:
        """
        Generate a rolling-horizon path plan for the current sub-goal.

        Computes single-agent A* to active sub-goal, truncates to horizon h,
        and extracts execution window w.
        """
        start_time = time.perf_counter()
        self.replan_count += 1
        self.steps_executed_in_window = 0
        self.active_waypoint_idx = 1
        self.last_progress_time = time.time()

        curr_tid = (
            self.ordered_tasks[self.active_task_idx]
            if self.active_task_idx < len(self.ordered_tasks)
            else ''
        )

        if not self.current_goal or self.active_phase == 'IDLE':
            self.full_path = []
            self.horizon_path = []
            self.execution_path = []
            self.last_plan_success = True
            self.last_latency_ms = (time.perf_counter() - start_time) * 1000.0
            return PlanningResponseData(
                success=True,
                status_message='IDLE: No active tasks to plan',
                robot_id=self.robot_id,
                task_id='',
                sub_goal_type='NONE',
                start_pose=self.current_position,
                goal_pose=self.current_position,
                full_path=[],
                horizon_path=[],
                execution_path=[],
                total_cost=0.0,
                planning_latency_ms=self.last_latency_ms,
                horizon_steps=self.config.horizon_steps,
                execution_window=self.config.execution_window,
                replan_count=self.replan_count,
            )

        start_grid = self.grid.to_grid(
            self.current_position[0], self.current_position[1],
        )
        goal_grid = self.grid.to_grid(
            self.current_goal[0], self.current_goal[1],
        )

        grid_path = self.astar.find_path(start_grid, goal_grid)
        self.last_latency_ms = (time.perf_counter() - start_time) * 1000.0

        if not grid_path:
            self.full_path = []
            self.horizon_path = []
            self.execution_path = []
            self.last_plan_success = False
            return PlanningResponseData(
                success=False,
                status_message=f'UNREACHABLE: No valid path from {start_grid} to {goal_grid}',
                robot_id=self.robot_id,
                task_id=curr_tid,
                sub_goal_type=self.active_phase,
                start_pose=self.current_position,
                goal_pose=self.current_goal,
                full_path=[],
                horizon_path=[],
                execution_path=[],
                total_cost=0.0,
                planning_latency_ms=self.last_latency_ms,
                horizon_steps=self.config.horizon_steps,
                execution_window=self.config.execution_window,
                replan_count=self.replan_count,
            )

        world_path = [self.grid.to_world(p) for p in grid_path]
        if grid_path[-1] == goal_grid and self.current_goal is not None:
            world_path[-1] = (round(self.current_goal[0], 3), round(self.current_goal[1], 3))

        if len(world_path) == 1 and self.current_goal is not None:
            # If start_grid == goal_grid but not arrived within tolerance,
            # drive remaining sub-grid distance to continuous goal.
            if not self.check_subgoal_arrival():
                world_path = [
                    self.current_position,
                    (round(self.current_goal[0], 3), round(self.current_goal[1], 3)),
                ]

        total_cost = 0.0
        for i in range(len(world_path) - 1):
            total_cost += math.hypot(
                world_path[i + 1][0] - world_path[i][0],
                world_path[i + 1][1] - world_path[i][1],
            )
        self.total_distance_planned += total_cost

        h_len = min(len(world_path), self.config.horizon_steps + 1)
        w_len = min(len(world_path), self.config.execution_window + 1)

        self.full_path = world_path
        self.horizon_path = world_path[:h_len]
        self.execution_path = world_path[:w_len]
        self.last_plan_success = True

        sub_goal_type = (
            'PICKUP' if self.active_phase == 'TRANSIT_TO_PICKUP' else 'DROPOFF'
        )

        return PlanningResponseData(
            success=True,
            status_message=f'SUCCESS: Generated {len(world_path)} step plan '
                           f'(h={h_len}, w={w_len})',
            robot_id=self.robot_id,
            task_id=curr_tid,
            sub_goal_type=sub_goal_type,
            start_pose=self.current_position,
            goal_pose=self.current_goal,
            full_path=self.full_path,
            horizon_path=self.horizon_path,
            execution_path=self.execution_path,
            total_cost=round(total_cost, 2),
            planning_latency_ms=round(self.last_latency_ms, 2),
            horizon_steps=self.config.horizon_steps,
            execution_window=self.config.execution_window,
            replan_count=self.replan_count,
        )

    def get_current_target_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Return the immediate target waypoint in world coordinates.

        Returns None if no active execution path exists.
        """
        if not self.execution_path or len(self.execution_path) <= 1:
            return None
        target_idx = min(self.active_waypoint_idx, len(self.execution_path) - 1)
        return self.execution_path[target_idx]

    def advance_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Advance one waypoint along the current execution window upon arrival.

        Returns the new target waypoint in world coordinates.
        """
        if not self.execution_path or len(self.execution_path) <= 1:
            return None

        self.active_waypoint_idx += 1
        self.steps_executed_in_window += 1
        self.last_progress_time = time.time()
        return self.get_current_target_waypoint()

    def advance_execution_step(self) -> Optional[Tuple[float, float]]:
        """
        Advance one step along current execution window (backward-compatibility).

        Returns the immediate target waypoint in world coordinates.
        """
        return self.advance_waypoint()

    def to_plan_dict(self) -> Dict[str, Any]:
        """Serialize current rolling-horizon plan state."""
        curr_tid = (
            self.ordered_tasks[self.active_task_idx]
            if self.active_task_idx < len(self.ordered_tasks)
            else ''
        )
        return {
            'robot_id': self.robot_id,
            'assigned_bundle': list(self.assigned_bundle),
            'ordered_tasks': list(self.ordered_tasks),
            'current_task_id': curr_tid,
            'current_phase': self.active_phase,
            'current_goal': list(self.current_goal) if self.current_goal else None,
            'horizon_path_len': len(self.horizon_path),
            'execution_path_len': len(self.execution_path),
            'replan_count': self.replan_count,
            'latency_ms': round(self.last_latency_ms, 2),
            'is_valid': self.last_plan_success,
        }

