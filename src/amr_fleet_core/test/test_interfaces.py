"""Unit tests for amr_fleet_core abstract interfaces."""

from amr_fleet_core.interfaces import (
    DeadlockManager,
    GlobalPlanner,
    LocalPlanner,
    MetricsCollector,
    TaskAllocator,
)
import pytest


def test_cannot_instantiate_abstract_task_allocator():
    with pytest.raises(TypeError):
        TaskAllocator()  # pylint: disable=abstract-class-instantiated


def test_cannot_instantiate_abstract_global_planner():
    with pytest.raises(TypeError):
        GlobalPlanner()  # pylint: disable=abstract-class-instantiated


def test_cannot_instantiate_abstract_local_planner():
    with pytest.raises(TypeError):
        LocalPlanner()  # pylint: disable=abstract-class-instantiated


def test_cannot_instantiate_abstract_deadlock_manager():
    with pytest.raises(TypeError):
        DeadlockManager()  # pylint: disable=abstract-class-instantiated


def test_cannot_instantiate_abstract_metrics_collector():
    with pytest.raises(TypeError):
        MetricsCollector()  # pylint: disable=abstract-class-instantiated


def test_concrete_mock_task_allocator():
    class DummyAllocator(TaskAllocator):

        def allocate_tasks(self, robot_states, tasks, epoch, network_state=None):
            return ({r: [t['id'] for t in tasks] for r in robot_states}, epoch + 1)

    allocator = DummyAllocator()
    res, next_epoch = allocator.allocate_tasks({'robot_1': {}}, [{'id': 'task_a'}], 0)
    assert res == {'robot_1': ['task_a']}
    assert next_epoch == 1

