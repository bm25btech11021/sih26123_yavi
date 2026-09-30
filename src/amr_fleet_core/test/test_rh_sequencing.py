"""Unit tests for M5 Task Sequencing Heuristics and Bundle Preservation."""

from amr_fleet_core.rh_planner import RollingHorizonPlanner, TaskSequencer


def test_priority_first_sequencing():
    """Verify PRIORITY_FIRST sorts higher priority tasks ahead of lower priority tasks."""
    tasks = {
        'T_LOW': {
            'task_id': 'T_LOW', 'pickup': (4.0, 4.0), 'dropoff': (6.0, 6.0),
            'priority': 1, 'deadline': 100.0,
        },
        'T_HIGH': {
            'task_id': 'T_HIGH', 'pickup': (8.0, 8.0), 'dropoff': (10.0, 10.0),
            'priority': 5, 'deadline': 100.0,
        },
        'T_MED': {
            'task_id': 'T_MED', 'pickup': (12.0, 12.0), 'dropoff': (14.0, 14.0),
            'priority': 3, 'deadline': 100.0,
        },
    }
    bundle = ['T_LOW', 'T_HIGH', 'T_MED']
    seq = TaskSequencer.sequence(
        bundle, tasks, robot_pos=(0.0, 0.0), heuristic='PRIORITY_FIRST',
    )

    assert seq == ['T_HIGH', 'T_MED', 'T_LOW']


def test_shortest_path_first_sequencing():
    """Verify SHORTEST_PATH_FIRST sorts tasks nearest to the robot first."""
    tasks = {
        'T_FAR': {
            'task_id': 'T_FAR', 'pickup': (14.0, 14.0), 'dropoff': (15.0, 15.0),
            'priority': 1, 'deadline': 0.0,
        },
        'T_NEAR': {
            'task_id': 'T_NEAR', 'pickup': (3.0, 3.0), 'dropoff': (4.0, 4.0),
            'priority': 1, 'deadline': 0.0,
        },
        'T_MID': {
            'task_id': 'T_MID', 'pickup': (7.0, 7.0), 'dropoff': (8.0, 8.0),
            'priority': 1, 'deadline': 0.0,
        },
    }
    bundle = ['T_FAR', 'T_NEAR', 'T_MID']
    seq = TaskSequencer.sequence(
        bundle, tasks, robot_pos=(2.0, 2.0), heuristic='SHORTEST_PATH_FIRST',
    )

    assert seq == ['T_NEAR', 'T_MID', 'T_FAR']


def test_deadline_first_sequencing():
    """Verify DEADLINE_FIRST sorts earliest deadlines first."""
    tasks = {
        'T_LATE': {
            'task_id': 'T_LATE', 'pickup': (2.0, 2.0), 'dropoff': (3.0, 3.0),
            'priority': 1, 'deadline': 200.0,
        },
        'T_URGENT': {
            'task_id': 'T_URGENT', 'pickup': (5.0, 5.0), 'dropoff': (6.0, 6.0),
            'priority': 1, 'deadline': 50.0,
        },
        'T_MED': {
            'task_id': 'T_MED', 'pickup': (8.0, 8.0), 'dropoff': (9.0, 9.0),
            'priority': 1, 'deadline': 120.0,
        },
    }
    bundle = ['T_LATE', 'T_URGENT', 'T_MED']
    seq = TaskSequencer.sequence(
        bundle, tasks, robot_pos=(0.0, 0.0), heuristic='DEADLINE_FIRST',
    )

    assert seq == ['T_URGENT', 'T_MED', 'T_LATE']


def test_bundle_preservation():
    """Verify that task sequencing preserves bundle membership exactly (no additions or drops)."""
    tasks = {
        f'T{i}': {
            'task_id': f'T{i}',
            'pickup': (float(i), float(i)),
            'dropoff': (float(i + 1), float(i + 1)),
            'priority': i,
            'deadline': 0.0,
        }
        for i in range(5)
    }
    bundle = ['T0', 'T1', 'T2', 'T3', 'T4']
    seq = TaskSequencer.sequence(
        bundle, tasks, robot_pos=(0.0, 0.0), heuristic='PRIORITY_FIRST',
    )

    assert set(seq) == set(bundle)
    assert len(seq) == len(bundle)


def test_sub_goal_progression():
    """Verify rolling planner progresses from PICKUP -> DROPOFF -> NEXT_TASK_PICKUP -> IDLE."""
    planner = RollingHorizonPlanner(robot_id='amr_0')
    planner.update_position((0.0, 0.0))
    tasks = {
        'T1': {
            'task_id': 'T1', 'pickup': (3.0, 3.0), 'dropoff': (5.0, 5.0),
            'priority': 2, 'deadline': 0.0,
        },
        'T2': {
            'task_id': 'T2', 'pickup': (7.0, 7.0), 'dropoff': (9.0, 9.0),
            'priority': 1, 'deadline': 0.0,
        },
    }
    planner.update_assigned_bundle(['T1', 'T2'], tasks)

    assert planner.active_phase == 'TRANSIT_TO_PICKUP'
    assert planner.current_goal == (3.0, 3.0)

    # 1. Reach T1 pickup
    planner.update_position((3.0, 3.0))
    ev1 = planner.advance_subgoal()
    assert ev1 == 'PICKUP_REACHED'
    assert planner.active_phase == 'TRANSIT_TO_DROPOFF'
    assert planner.current_goal == (5.0, 5.0)

    # 2. Reach T1 dropoff
    planner.update_position((5.0, 5.0))
    ev2 = planner.advance_subgoal()
    assert ev2 == 'TASK_COMPLETED'
    assert planner.active_phase == 'TRANSIT_TO_PICKUP'
    assert planner.current_goal == (7.0, 7.0)

    # 3. Reach T2 pickup
    planner.update_position((7.0, 7.0))
    ev3 = planner.advance_subgoal()
    assert ev3 == 'PICKUP_REACHED'
    assert planner.active_phase == 'TRANSIT_TO_DROPOFF'
    assert planner.current_goal == (9.0, 9.0)

    # 4. Reach T2 dropoff
    planner.update_position((9.0, 9.0))
    ev4 = planner.advance_subgoal()
    assert ev4 == 'ALL_COMPLETED'
    assert planner.active_phase == 'IDLE'
    assert planner.current_goal is None

