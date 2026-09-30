"""Unit tests for operator live task control and allocation."""

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.task_manager_node import TaskManagerNode
from amr_fleet_core.task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskLifecycleState,
)
from amr_fleet_msgs.srv import ControlTask, CreateTask
import pytest
import rclpy


def test_task_model_operator_transitions():
    """Verify operator state transitions including CANCELLED and REQUEUE."""
    task = Task('T100', (2.0, 2.0), (5.0, 5.0), requested_robot='amr_1')
    assert task.requested_robot == 'amr_1'
    assert task.reassignment_count == 0

    # PENDING -> CANCELLED
    task.transition_to(TaskLifecycleState.CANCELLED, timestamp=10.0)
    assert task.state == TaskLifecycleState.CANCELLED
    assert task.is_terminal

    # Test ASSIGNED -> CANCELLED
    t2 = Task('T101', (2.0, 2.0), (5.0, 5.0))
    t2.transition_to(
        TaskLifecycleState.ASSIGNED, timestamp=5.0, robot_id='amr_0'
    )
    t2.transition_to(
        TaskLifecycleState.CANCELLED, timestamp=12.0, details='Operator abort'
    )
    assert t2.state == TaskLifecycleState.CANCELLED

    # Test ASSIGNED -> PENDING (Operator Requeue)
    t3 = Task('T102', (2.0, 2.0), (5.0, 5.0))
    t3.transition_to(
        TaskLifecycleState.ASSIGNED, timestamp=5.0, robot_id='amr_0'
    )
    t3.transition_to(
        TaskLifecycleState.PENDING, timestamp=15.0, details='Operator requeue'
    )
    assert t3.state == TaskLifecycleState.PENDING
    assert t3.assigned_robot_id is None
    assert t3.reassignment_count == 1

    # Test FAILED -> PENDING (Operator Requeue after recovery)
    t4 = Task('T103', (2.0, 2.0), (5.0, 5.0))
    t4.transition_to(
        TaskLifecycleState.ASSIGNED, timestamp=5.0, robot_id='amr_0'
    )
    t4.transition_to(TaskLifecycleState.FAILED, timestamp=20.0)
    assert t4.state == TaskLifecycleState.FAILED
    t4.transition_to(
        TaskLifecycleState.PENDING, timestamp=25.0, details='Operator retry'
    )
    assert t4.state == TaskLifecycleState.PENDING
    assert t4.reassignment_count == 1

    # Test serialization with requested_robot
    data = t4.to_dict()
    assert data['requested_robot'] is None
    assert data['reassignment_count'] == 1
    restored = Task.from_dict(data)
    assert restored.task_id == 'T103'
    assert restored.reassignment_count == 1


def test_task_model_invalid_requeue():
    """Verify that completed tasks cannot be requeued."""
    task = Task('T104', (2.0, 2.0), (5.0, 5.0))
    task.transition_to(
        TaskLifecycleState.ASSIGNED, timestamp=5.0, robot_id='amr_0'
    )
    task.transition_to(TaskLifecycleState.IN_PROGRESS, timestamp=10.0)
    task.transition_to(TaskLifecycleState.COMPLETED, timestamp=30.0)
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.PENDING, timestamp=35.0)


def test_task_creation_bounds_and_validation():
    """Verify coordinate bounds and validation in TaskManagerNode."""
    if not rclpy.ok():
        rclpy.init()
    node = TaskManagerNode()
    try:
        # 1. Valid Task Creation
        req = CreateTask.Request()
        req.pickup_x = 4.0
        req.pickup_y = 5.0
        req.dropoff_x = 12.0
        req.dropoff_y = 15.0
        req.priority = 2
        resp = CreateTask.Response()
        resp = node._handle_create_task(req, resp)
        assert resp.accepted is True
        assert resp.task_id.startswith('T')
        created_id = resp.task_id
        assert created_id in node.tasks

        # 2. Out of bounds (negative)
        req_bad1 = CreateTask.Request()
        req_bad1.pickup_x = -1.0
        req_bad1.pickup_y = 5.0
        req_bad1.dropoff_x = 12.0
        req_bad1.dropoff_y = 15.0
        resp_bad1 = CreateTask.Response()
        resp_bad1 = node._handle_create_task(req_bad1, resp_bad1)
        assert resp_bad1.accepted is False
        assert 'bounds' in resp_bad1.message.lower()

        # 3. Out of bounds (> 30.0)
        req_bad2 = CreateTask.Request()
        req_bad2.pickup_x = 4.0
        req_bad2.pickup_y = 5.0
        req_bad2.dropoff_x = 35.0
        req_bad2.dropoff_y = 15.0
        resp_bad2 = CreateTask.Response()
        resp_bad2 = node._handle_create_task(req_bad2, resp_bad2)
        assert resp_bad2.accepted is False
        assert 'bounds' in resp_bad2.message.lower()

        # 4. Duplicate Task ID
        req_dup = CreateTask.Request()
        req_dup.task_id = created_id
        req_dup.pickup_x = 1.0
        req_dup.pickup_y = 1.0
        req_dup.dropoff_x = 2.0
        req_dup.dropoff_y = 2.0
        resp_dup = CreateTask.Response()
        resp_dup = node._handle_create_task(req_dup, resp_dup)
        assert resp_dup.accepted is False
        assert 'already exists' in resp_dup.message.lower()

        # 5. Invalid target robot format
        req_bad_bot = CreateTask.Request()
        req_bad_bot.pickup_x = 1.0
        req_bad_bot.pickup_y = 1.0
        req_bad_bot.dropoff_x = 2.0
        req_bad_bot.dropoff_y = 2.0
        req_bad_bot.requested_robot = 'robot_unknown'
        resp_bad_bot = CreateTask.Response()
        resp_bad_bot = node._handle_create_task(req_bad_bot, resp_bad_bot)
        assert resp_bad_bot.accepted is False
        assert 'format' in resp_bad_bot.message.lower()
    finally:
        node.destroy_node()


def test_task_control_cancel_and_requeue():
    """Verify cancel and requeue actions in TaskManagerNode."""
    if not rclpy.ok():
        rclpy.init()
    node = TaskManagerNode()
    try:
        # Create a task
        create_req = CreateTask.Request()
        create_req.task_id = 'T_CTRL_01'
        create_req.pickup_x = 2.0
        create_req.pickup_y = 2.0
        create_req.dropoff_x = 8.0
        create_req.dropoff_y = 8.0
        c_resp = CreateTask.Response()
        c_resp = node._handle_create_task(create_req, c_resp)
        assert c_resp.accepted is True

        # Simulate task assignment
        node.tasks['T_CTRL_01'].transition_to(
            TaskLifecycleState.ASSIGNED, timestamp=1.0, robot_id='amr_0'
        )
        assert node.tasks['T_CTRL_01'].state == TaskLifecycleState.ASSIGNED

        # Requeue ASSIGNED task back to PENDING
        requeue_req = ControlTask.Request()
        requeue_req.task_id = 'T_CTRL_01'
        requeue_req.action = 'REQUEUE'
        requeue_resp = ControlTask.Response()
        requeue_resp = node._handle_control_task(requeue_req, requeue_resp)
        assert requeue_resp.success is True
        assert (
            node.tasks['T_CTRL_01'].state == TaskLifecycleState.PENDING
        )
        assert node.tasks['T_CTRL_01'].assigned_robot_id is None

        # Cancel PENDING task
        ctrl_req = ControlTask.Request()
        ctrl_req.task_id = 'T_CTRL_01'
        ctrl_req.action = 'CANCEL'
        ctrl_resp = ControlTask.Response()
        ctrl_resp = node._handle_control_task(ctrl_req, ctrl_resp)
        assert ctrl_resp.success is True
        assert (
            node.tasks['T_CTRL_01'].state == TaskLifecycleState.CANCELLED
        )

        # Requeue on CANCELLED task should be rejected
        req_requeue_cancelled = ControlTask.Request()
        req_requeue_cancelled.task_id = 'T_CTRL_01'
        req_requeue_cancelled.action = 'REQUEUE'
        resp_requeue_cancelled = ControlTask.Response()
        resp_requeue_cancelled = node._handle_control_task(
            req_requeue_cancelled, resp_requeue_cancelled
        )
        assert resp_requeue_cancelled.success is False

        # Unknown action
        bad_req = ControlTask.Request()
        bad_req.task_id = 'T_CTRL_01'
        bad_req.action = 'EXPLODE'
        bad_resp = ControlTask.Response()
        bad_resp = node._handle_control_task(bad_req, bad_resp)
        assert bad_resp.success is False
        assert 'unsupported' in bad_resp.message.lower()
    finally:
        node.destroy_node()


def test_cbba_direct_constraint_auction():
    """Verify CBBA ignores tasks constrained to another robot."""
    agent_0 = CBBAAgent(
        robot_id='amr_0',
        config=CBBAConfig(max_bundle_size=5),
        initial_position=(0.0, 0.0),
    )
    agent_1 = CBBAAgent(
        robot_id='amr_1',
        config=CBBAConfig(max_bundle_size=5),
        initial_position=(1.0, 1.0),
    )

    # Task T_AUTO: Unconstrained
    task_auto = {
        'task_id': 'T_AUTO',
        'pickup': (2.0, 2.0),
        'dropoff': (4.0, 4.0),
        'priority': 2,
    }
    # Task T_DIR_1: Constrained to amr_1
    task_dir1 = {
        'task_id': 'T_DIR_1',
        'pickup': (2.0, 2.0),
        'dropoff': (4.0, 4.0),
        'priority': 2,
        'requested_robot': 'amr_1',
    }

    pool = {
        'T_AUTO': task_auto,
        'T_DIR_1': task_dir1,
    }

    # Agent 0 builds bundle: should ONLY bid on T_AUTO, NOT on T_DIR_1
    agent_0.build_bundle(pool, current_time=0.0)
    assert 'T_DIR_1' not in agent_0.state.bundle
    assert 'T_AUTO' in agent_0.state.bundle

    # Agent 1 builds bundle: can bid on T_DIR_1 as it is the target robot
    agent_1.build_bundle(pool, current_time=0.0)
    assert 'T_DIR_1' in agent_1.state.bundle

