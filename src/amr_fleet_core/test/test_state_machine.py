"""Unit tests for amr_fleet_core robot state machine."""

from amr_fleet_core.state_machine import (
    InvalidStateTransitionError,
    RobotLifecycleState,
    RobotStateMachine,
)
import pytest


def test_state_machine_initialization():
    sm = RobotStateMachine('amr_0', RobotLifecycleState.UNCONFIGURED)
    assert sm.robot_id == 'amr_0'
    assert sm.current_state == RobotLifecycleState.UNCONFIGURED
    assert sm.history == []


def test_valid_lifecycle_transitions():
    sm = RobotStateMachine('amr_1', RobotLifecycleState.UNCONFIGURED)

    sm.transition_to(RobotLifecycleState.IDLE)
    assert sm.current_state == RobotLifecycleState.IDLE

    sm.transition_to(RobotLifecycleState.BIDDING)
    assert sm.current_state == RobotLifecycleState.BIDDING

    sm.transition_to(RobotLifecycleState.PLANNING)
    assert sm.current_state == RobotLifecycleState.PLANNING

    sm.transition_to(RobotLifecycleState.EXECUTING)
    assert sm.current_state == RobotLifecycleState.EXECUTING

    sm.transition_to(RobotLifecycleState.WAITING)
    assert sm.current_state == RobotLifecycleState.WAITING

    sm.transition_to(RobotLifecycleState.EXECUTING)
    assert sm.current_state == RobotLifecycleState.EXECUTING

    sm.transition_to(RobotLifecycleState.IDLE)
    assert sm.current_state == RobotLifecycleState.IDLE

    assert len(sm.history) == 7


def test_invalid_lifecycle_transition_raises():
    sm = RobotStateMachine('amr_2', RobotLifecycleState.UNCONFIGURED)

    with pytest.raises(InvalidStateTransitionError) as exc_info:
        sm.transition_to(RobotLifecycleState.EXECUTING)

    assert exc_info.value.from_state == RobotLifecycleState.UNCONFIGURED
    assert exc_info.value.to_state == RobotLifecycleState.EXECUTING
    assert sm.current_state == RobotLifecycleState.UNCONFIGURED
    assert len(sm.history) == 0


def test_fault_and_recovery():
    sm = RobotStateMachine('amr_3', RobotLifecycleState.IDLE)
    sm.transition_to(RobotLifecycleState.FAULT)
    assert sm.current_state == RobotLifecycleState.FAULT

    sm.transition_to(RobotLifecycleState.IDLE)
    assert sm.current_state == RobotLifecycleState.IDLE

