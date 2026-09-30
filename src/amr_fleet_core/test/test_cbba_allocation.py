"""Unit and integration tests for decentralized CBBAAllocator."""

from amr_fleet_core.cbba_agent import CBBAConfig
from amr_fleet_core.cbba_allocator import CBBAAllocator


def test_cbba_allocator_empty_inputs():
    """Verify behavior on empty robot states or task lists."""
    allocator = CBBAAllocator()
    res, next_epoch = allocator.allocate_tasks({}, [], 0)
    assert res == {}
    assert next_epoch == 1

    res_no_tasks, next_epoch = allocator.allocate_tasks({'amr_0': {}}, [], 1)
    assert res_no_tasks == {'amr_0': []}
    assert next_epoch == 2


def test_cbba_allocator_single_robot():
    """Verify single robot allocation consumes tasks up to bundle capacity."""
    config = CBBAConfig(max_bundle_size=3)
    allocator = CBBAAllocator(config=config)

    robot_states = {'amr_0': {'position': (0.0, 0.0)}}
    tasks = [
        {'task_id': f't_{i}', 'pickup': (float(i), 0.0), 'dropoff': (float(i) + 0.5, 0.0)}
        for i in range(5)
    ]

    assignments, next_epoch = allocator.allocate_tasks(robot_states, tasks, 0)
    assert next_epoch == 1
    assert len(assignments['amr_0']) == 3  # bundle cap


def test_cbba_allocator_multi_robot_no_duplicate_assignment():
    """Verify multi-robot allocation produces strictly disjoint assignments."""
    config = CBBAConfig(max_bundle_size=3)
    allocator = CBBAAllocator(config=config)

    robot_states = {
        'amr_0': {'position': (0.0, 0.0)},
        'amr_1': {'position': (10.0, 0.0)},
        'amr_2': {'position': (0.0, 10.0)},
    }
    tasks = [
        {
            'task_id': f't_{i}',
            'pickup': (float(i % 5), float(i // 5)),
            'dropoff': (float(i % 5) + 1.0, float(i // 5) + 1.0),
            'priority': (i % 4) + 1,
        }
        for i in range(8)
    ]

    assignments, _ = allocator.allocate_tasks(robot_states, tasks, 0)

    # Check convergence
    assert allocator.last_converged is True

    # Check no task is assigned to more than one robot
    assigned_tasks = []
    for r_id, b_tasks in assignments.items():
        for t in b_tasks:
            assert t not in assigned_tasks, f'Task {t} assigned more than once!'
            assigned_tasks.append(t)

    # Verify bundle capacity respected
    for r_id, b_tasks in assignments.items():
        assert len(b_tasks) <= config.max_bundle_size


def test_cbba_allocator_determinism():
    """Verify identical inputs yield 100% bitwise identical assignments across runs."""
    config = CBBAConfig(max_bundle_size=3)

    robot_states = {
        'amr_0': {'position': (0.0, 0.0)},
        'amr_1': {'position': (5.0, 5.0)},
        'amr_2': {'position': (10.0, 0.0)},
        'amr_3': {'position': (0.0, 10.0)},
        'amr_4': {'position': (10.0, 10.0)},
    }
    tasks = [
        {
            'task_id': f'task_{i:02d}',
            'pickup': (float(i * 2 % 12), float(i * 3 % 8)),
            'dropoff': (float(i * 2 % 12) + 1.0, float(i * 3 % 8) + 1.0),
            'priority': (i % 4) + 1,
        }
        for i in range(12)
    ]

    alloc_1 = CBBAAllocator(config=config)
    res_1, _ = alloc_1.allocate_tasks(robot_states, tasks, 0)

    alloc_2 = CBBAAllocator(config=config)
    res_2, _ = alloc_2.allocate_tasks(robot_states, tasks, 0)

    assert res_1 == res_2

