"""Core interfaces and state abstractions for YAVI-SIH26123 AMR Fleet Coordination."""

from .adaptive_compute_policy import (
    AdaptiveComputePolicy,
    AdaptiveComputePolicyConfig,
    ComputeState,
    PolicyTransitionRecord,
)
from .benchmark_manager import (
    BenchmarkConfig,
    check_obb_intersection,
    ExperimentMetadata,
    get_git_commit,
    StandardMetrics,
    StatisticalAggregator,
    StatisticalSummary,
    TerminationReason,
)
from .cbba_agent import CBBAAgent, CBBAConfig, CBBALocalState
from .cbba_allocator import CBBAAllocator
from .cbba_node import CBBANode
from .communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    DelayedMessageQueue,
    DropReason,
    PRESET_PROFILES,
)
from .compute_modes import (
    COMPUTE_CONFIGS,
    ComputeMode,
    ComputeModeConfig,
    get_compute_mode_config,
)
from .conflict_detector import ConflictDetector
from .coordination_models import (
    Conflict,
    ConflictType,
    CoordinationState,
    DeadlockRecord,
    Reservation,
)
from .deadlock_recovery import DeadlockRecoveryManager
from .fleet_state import FleetState, RobotInfo
from .interfaces import (
    DeadlockManager,
    GlobalPlanner,
    LocalPlanner,
    MetricsCollector,
    TaskAllocator,
)
from .multi_agent_coordinator import MultiAgentCoordinator
from .pibt_planner import PIBTAgentState, PIBTLocalPlanner
from .reservation_table import SpaceTimeReservationTable
from .rh_node import RollingHorizonPlannerNode
from .rh_planner import (
    PlanningResponseData,
    RHConfig,
    RollingHorizonPlanner,
    SingleAgentAStar,
    TaskSequencer,
)
from .stale_state_manager import (
    InformationState,
    PeerRecord,
    StaleStateManager,
)
from .state_machine import (
    InvalidStateTransitionError,
    RobotLifecycleState,
    RobotStateMachine,
)
from .task_generator import TaskGenerator, TaskGeneratorConfig
from .task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskEvent,
    TaskLifecycleState,
    TaskPriority,
)
from .wfg_deadlock import DeadlockDetector, WaitForGraph
from .workload import WorkloadManager

__all__ = [
    'BenchmarkConfig',
    'check_obb_intersection',
    'ExperimentMetadata',
    'StandardMetrics',
    'StatisticalAggregator',
    'StatisticalSummary',
    'TerminationReason',
    'get_git_commit',
    'TaskAllocator',
    'GlobalPlanner',
    'LocalPlanner',
    'DeadlockManager',
    'MetricsCollector',
    'RobotLifecycleState',
    'RobotStateMachine',
    'InvalidStateTransitionError',
    'FleetState',
    'RobotInfo',
    'Task',
    'TaskPriority',
    'TaskLifecycleState',
    'TaskEvent',
    'InvalidTaskTransitionError',
    'TaskGenerator',
    'TaskGeneratorConfig',
    'WorkloadManager',
    'CBBAAgent',
    'CBBAConfig',
    'CBBALocalState',
    'CBBAAllocator',
    'CBBANode',
    'RHConfig',
    'PlanningResponseData',
    'SingleAgentAStar',
    'TaskSequencer',
    'RollingHorizonPlanner',
    'RollingHorizonPlannerNode',
    'Conflict',
    'ConflictType',
    'CoordinationState',
    'DeadlockRecord',
    'Reservation',
    'ConflictDetector',
    'DeadlockRecoveryManager',
    'MultiAgentCoordinator',
    'PIBTAgentState',
    'PIBTLocalPlanner',
    'SpaceTimeReservationTable',
    'DeadlockDetector',
    'WaitForGraph',
    'CommunicationAction',
    'CommunicationImpairmentModel',
    'CommunicationProfileConfig',
    'DelayedMessageQueue',
    'DropReason',
    'PRESET_PROFILES',
    'InformationState',
    'PeerRecord',
    'StaleStateManager',
    'ComputeMode',
    'ComputeModeConfig',
    'COMPUTE_CONFIGS',
    'get_compute_mode_config',
    'ComputeState',
    'PolicyTransitionRecord',
    'AdaptiveComputePolicyConfig',
    'AdaptiveComputePolicy',
]

