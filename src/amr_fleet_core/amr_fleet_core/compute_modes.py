"""Computational mode configurations and parameters for M8B Adaptive Compute."""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Union


class ComputeMode(str, Enum):
    """Discrete computational modes for decentralized fleet coordination."""

    LOW = 'COMPUTE_MODE_LOW'
    NORMAL = 'COMPUTE_MODE_NORMAL'
    HIGH = 'COMPUTE_MODE_HIGH'

    @classmethod
    def from_string(cls, mode_str: str) -> 'ComputeMode':
        """Parse string to ComputeMode with safe fallback to NORMAL."""
        if not isinstance(mode_str, str):
            return cls.NORMAL
        cleaned = mode_str.strip().upper()
        if cleaned in ('LOW', 'COMPUTE_MODE_LOW'):
            return cls.LOW
        if cleaned in ('HIGH', 'COMPUTE_MODE_HIGH'):
            return cls.HIGH
        return cls.NORMAL


@dataclass(frozen=True)
class ComputeModeConfig:
    """Explicit parameters governing AMR planning, coordination, and auctioning."""

    mode: ComputeMode
    replan_rate: float             # RHCR planning cycle frequency (Hz)
    execution_window: int          # Steps committed before replanning (steps)
    horizon_steps: int             # Total planning horizon foresight (steps)
    consensus_rate: float          # CBBA auction bid exchange frequency (Hz)
    pibt_search_depth: int         # PIBT priority inheritance lookahead (steps)
    max_planning_budget_ms: float  # Maximum allowable planning computation (ms)
    coordination_rate: float       # Space-time reservation broadcast rate (Hz)
    description: str               # Operational rationale


# Pre-configured mode operating parameters
COMPUTE_CONFIGS: Dict[ComputeMode, ComputeModeConfig] = {
    ComputeMode.LOW: ComputeModeConfig(
        mode=ComputeMode.LOW,
        replan_rate=1.0,
        execution_window=6,
        horizon_steps=6,
        consensus_rate=2.0,
        pibt_search_depth=1,
        max_planning_budget_ms=20.0,
        coordination_rate=1.0,
        description='Reduced frequency for degraded comm or high host compute pressure',
    ),
    ComputeMode.NORMAL: ComputeModeConfig(
        mode=ComputeMode.NORMAL,
        replan_rate=2.0,
        execution_window=4,
        horizon_steps=10,
        consensus_rate=5.0,
        pibt_search_depth=1,
        max_planning_budget_ms=50.0,
        coordination_rate=2.0,
        description='Nominal M0-M7 baseline configuration',
    ),
    ComputeMode.HIGH: ComputeModeConfig(
        mode=ComputeMode.HIGH,
        replan_rate=4.0,
        execution_window=2,
        horizon_steps=14,
        consensus_rate=10.0,
        pibt_search_depth=2,
        max_planning_budget_ms=100.0,
        coordination_rate=4.0,
        description='High agility and foresight for dense spatial traffic contention',
    ),
}


def get_compute_mode_config(mode: Union[str, ComputeMode]) -> ComputeModeConfig:
    """Retrieve configuration for a mode, strictly falling back to NORMAL if invalid."""
    if isinstance(mode, str):
        mode_enum = ComputeMode.from_string(mode)
    elif isinstance(mode, ComputeMode):
        mode_enum = mode
    else:
        mode_enum = ComputeMode.NORMAL

    return COMPUTE_CONFIGS.get(mode_enum, COMPUTE_CONFIGS[ComputeMode.NORMAL])

