"""Unit tests for deterministic task generator."""

import math

from amr_fleet_core.task_generator import (
    DEFAULT_OBSTACLE_BOUNDS,
    TaskGenerator,
    TaskGeneratorConfig,
)
from amr_fleet_core.task_model import TaskLifecycleState, TaskPriority


def test_generator_determinism_identical_seed():
    """Verify that identical seed and config produce 100% identical task sequences."""
    config1 = TaskGeneratorConfig(task_count=10, seed=12345, mode='STATION_PAIR')
    config2 = TaskGeneratorConfig(task_count=10, seed=12345, mode='STATION_PAIR')

    tasks1 = TaskGenerator(config1).generate_workload()
    tasks2 = TaskGenerator(config2).generate_workload()

    assert len(tasks1) == 10
    assert len(tasks2) == 10

    for t1, t2 in zip(tasks1, tasks2):
        assert t1.task_id == t2.task_id
        assert t1.pickup == t2.pickup
        assert t1.dropoff == t2.dropoff
        assert t1.priority == t2.priority
        assert t1.deadline == t2.deadline
        assert t1.created_at == t2.created_at


def test_generator_seed_divergence():
    """Verify that distinct seeds produce statistically divergent task workloads."""
    config1 = TaskGeneratorConfig(task_count=10, seed=100, mode='BOUNDED_RANDOM')
    config2 = TaskGeneratorConfig(task_count=10, seed=200, mode='BOUNDED_RANDOM')

    tasks1 = TaskGenerator(config1).generate_workload()
    tasks2 = TaskGenerator(config2).generate_workload()

    different_pickups = sum(1 for t1, t2 in zip(tasks1, tasks2) if t1.pickup != t2.pickup)
    assert different_pickups > 5, 'Workloads with different seeds should diverge significantly'


def test_generator_spatial_validity_and_obstacle_avoidance():
    """Verify all generated positions are within map bounds and outside obstacles."""
    config = TaskGeneratorConfig(
        task_count=50,
        seed=777,
        mode='BOUNDED_RANDOM',
        map_bounds=(0.5, 15.5, 0.5, 15.5),
        min_travel_distance=1.5,
    )
    generator = TaskGenerator(config)
    tasks = generator.generate_workload()

    assert len(tasks) == 50

    for task in tasks:
        px, py = task.pickup
        dx, dy = task.dropoff

        # Bounds check
        assert 0.5 <= px <= 15.5
        assert 0.5 <= py <= 15.5
        assert 0.5 <= dx <= 15.5
        assert 0.5 <= dy <= 15.5

        # Travel distance check
        travel_dist = math.hypot(dx - px, dy - py)
        assert travel_dist >= 1.5

        # Obstacle check
        for ox_min, ox_max, oy_min, oy_max in DEFAULT_OBSTACLE_BOUNDS:
            assert not (ox_min <= px <= ox_max and oy_min <= py <= oy_max), (
                f'Pickup ({px}, {py}) collided with obstacle box {ox_min, ox_max, oy_min, oy_max}'
            )
            assert not (ox_min <= dx <= ox_max and oy_min <= dy <= oy_max), (
                f'Dropoff ({dx}, {dy}) collided with obstacle box {ox_min, ox_max, oy_min, oy_max}'
            )


def test_generator_priority_distribution():
    """Verify generator respects configured priority weights."""
    weights = {'LOW': 0.1, 'NORMAL': 0.6, 'HIGH': 0.2, 'CRITICAL': 0.1}
    config = TaskGeneratorConfig(
        task_count=100,
        seed=42,
        priority_weights=weights,
    )
    tasks = TaskGenerator(config).generate_workload()

    counts = {p: 0 for p in TaskPriority}
    for t in tasks:
        counts[t.priority] += 1

    assert counts[TaskPriority.NORMAL] > counts[TaskPriority.LOW]
    assert counts[TaskPriority.NORMAL] > counts[TaskPriority.HIGH]
    assert counts[TaskPriority.NORMAL] > counts[TaskPriority.CRITICAL]
    assert counts[TaskPriority.CRITICAL] > 0


def test_generator_deadlines():
    """Verify deadline sampling behavior when enabled and disabled."""
    # Enabled
    cfg_on = TaskGeneratorConfig(
        task_count=10,
        seed=42,
        base_time=100.0,
        deadline_enabled=True,
        deadline_fraction=1.0,
        deadline_window_sec=(30.0, 60.0),
    )
    tasks_on = TaskGenerator(cfg_on).generate_workload()
    for t in tasks_on:
        assert t.deadline is not None
        assert 130.0 <= t.deadline <= 160.0

    # Disabled
    cfg_off = TaskGeneratorConfig(task_count=10, seed=42, deadline_enabled=False)
    tasks_off = TaskGenerator(cfg_off).generate_workload()
    for t in tasks_off:
        assert t.deadline is None


def test_generator_independence_from_robot_allocation():
    """Verify task generator NEVER pre-assigns tasks to robots."""
    config = TaskGeneratorConfig(task_count=20, seed=999)
    tasks = TaskGenerator(config).generate_workload()

    for task in tasks:
        assert task.assigned_robot_id is None
        assert task.state == TaskLifecycleState.PENDING
        assert task.assigned_at is None
        assert task.started_at is None


def test_unique_task_ids():
    """Verify all generated tasks in a workload have distinct IDs."""
    config = TaskGeneratorConfig(task_count=100, seed=888)
    tasks = TaskGenerator(config).generate_workload()
    ids = [t.task_id for t in tasks]
    assert len(ids) == len(set(ids)) == 100

