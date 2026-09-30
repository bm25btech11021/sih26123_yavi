"""Unit tests for workload configuration loading and management."""

import os
import tempfile

from amr_fleet_core.task_model import TaskPriority
from amr_fleet_core.workload import WorkloadManager
import pytest


@pytest.fixture
def workloads_dir():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    return os.path.join(ws_root, 'config', 'workloads')


def test_load_small_deterministic(workloads_dir):
    """Verify loading workload_small_deterministic.yaml."""
    path = os.path.join(workloads_dir, 'workload_small_deterministic.yaml')
    tasks = WorkloadManager.load_from_yaml(path)
    assert len(tasks) == 5
    assert tasks[0].task_id == 'task_small_0001'
    for t in tasks:
        assert t.deadline is not None


def test_load_medium_priority(workloads_dir):
    """Verify loading workload_medium_priority.yaml."""
    path = os.path.join(workloads_dir, 'workload_medium_priority.yaml')
    tasks = WorkloadManager.load_from_yaml(path)
    assert len(tasks) == 15
    assert any(t.priority in (TaskPriority.HIGH, TaskPriority.CRITICAL) for t in tasks)


def test_load_benchmark_deadlines(workloads_dir):
    """Verify loading workload_benchmark_deadlines.yaml."""
    path = os.path.join(workloads_dir, 'workload_benchmark_deadlines.yaml')
    tasks = WorkloadManager.load_from_yaml(path)
    assert len(tasks) == 25
    for t in tasks:
        assert t.deadline is not None
        assert 25.0 <= t.deadline <= 75.0


def test_load_explicit_sample(workloads_dir):
    """Verify loading workload_explicit_sample.yaml."""
    path = os.path.join(workloads_dir, 'workload_explicit_sample.yaml')
    tasks = WorkloadManager.load_from_yaml(path)
    assert len(tasks) == 3
    assert tasks[0].task_id == 'task_exp_001'
    assert tasks[0].pickup == (2.0, 2.0)
    assert tasks[0].dropoff == (8.0, 8.0)
    assert tasks[0].priority == TaskPriority.CRITICAL
    assert tasks[0].metadata.get('pallet_type') == 'EURO_PALLET'


def test_save_and_load_roundtrip():
    """Verify programmatic workload save to YAML and reload."""
    from amr_fleet_core.task_generator import TaskGenerator, TaskGeneratorConfig

    cfg = TaskGeneratorConfig(task_count=4, seed=555)
    tasks = TaskGenerator(cfg).generate_workload()

    with tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as tmp:
        tmp_path = tmp.name

    try:
        WorkloadManager.save_to_yaml(tasks, tmp_path, description='Test Temp Workload')
        restored = WorkloadManager.load_from_yaml(tmp_path)
        assert len(restored) == 4
        for original, loaded in zip(tasks, restored):
            assert original.task_id == loaded.task_id
            assert original.pickup == loaded.pickup
            assert original.dropoff == loaded.dropoff
            assert original.priority == loaded.priority
    finally:
        if os.path.isfile(tmp_path):
            os.remove(tmp_path)

