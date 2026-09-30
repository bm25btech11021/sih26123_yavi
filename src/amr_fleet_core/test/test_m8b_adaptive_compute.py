"""Unit and integration tests for M8B Adaptive Compute."""

from typing import List

from amr_fleet_core.adaptive_compute_policy import (
    AdaptiveComputePolicy,
    AdaptiveComputePolicyConfig,
    ComputeState,
)
from amr_fleet_core.compute_modes import (
    ComputeMode,
    ComputeModeConfig,
    get_compute_mode_config,
)


def test_compute_mode_configs() -> None:
    """Verify computational modes have distinct parameters."""
    for mode in [ComputeMode.LOW, ComputeMode.NORMAL, ComputeMode.HIGH]:
        cfg = get_compute_mode_config(mode)
        assert isinstance(cfg, ComputeModeConfig)
        assert cfg.mode == mode
        assert cfg.replan_rate > 0.0
        assert cfg.execution_window > 0
        assert cfg.horizon_steps >= cfg.execution_window
        assert cfg.consensus_rate > 0.0
        assert cfg.max_planning_budget_ms > 0.0

    # Test distinct differences between modes
    cfg_low = get_compute_mode_config(ComputeMode.LOW)
    cfg_norm = get_compute_mode_config(ComputeMode.NORMAL)
    cfg_high = get_compute_mode_config(ComputeMode.HIGH)

    assert cfg_low.replan_rate < cfg_norm.replan_rate < cfg_high.replan_rate
    assert (
        cfg_low.execution_window
        > cfg_norm.execution_window
        > cfg_high.execution_window
    )
    assert (
        cfg_low.horizon_steps
        < cfg_norm.horizon_steps
        < cfg_high.horizon_steps
    )
    assert (
        cfg_low.consensus_rate
        < cfg_norm.consensus_rate
        < cfg_high.consensus_rate
    )


def test_compute_mode_fallback() -> None:
    """Verify fallback to COMPUTE_MODE_NORMAL on invalid strings or types."""
    assert get_compute_mode_config('INVALID').mode == ComputeMode.NORMAL
    assert get_compute_mode_config('random_junk').mode == ComputeMode.NORMAL
    assert get_compute_mode_config(None).mode == ComputeMode.NORMAL
    assert get_compute_mode_config(12345).mode == ComputeMode.NORMAL


def test_policy_initial_state() -> None:
    """Verify policy initializes to COMPUTE_MODE_NORMAL."""
    policy = AdaptiveComputePolicy(robot_id='amr_0')
    assert policy.current_mode == ComputeMode.NORMAL
    assert len(policy.transition_history) == 0


def test_degraded_comm_triggers_low() -> None:
    """Verify information age exceeding threshold triggers LOW."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0, confirmation_samples=3
    )
    policy = AdaptiveComputePolicy(robot_id='amr_0', config=cfg)

    # 1st sample: candidate recorded, mode stays NORMAL
    mode, rec = policy.evaluate(
        ComputeState(timestamp=1.0, information_age_s=1.8)
    )
    assert mode == ComputeMode.NORMAL
    assert rec is None

    # 2nd sample: confirmation count = 2, mode stays NORMAL
    mode, rec = policy.evaluate(
        ComputeState(timestamp=1.5, information_age_s=1.8)
    )
    assert mode == ComputeMode.NORMAL
    assert rec is None

    # 3rd sample: confirmed -> transitions to LOW
    mode, rec = policy.evaluate(
        ComputeState(timestamp=2.0, information_age_s=1.8)
    )
    assert mode == ComputeMode.LOW
    assert rec is not None
    assert rec.new_mode == ComputeMode.LOW
    assert rec.previous_mode == ComputeMode.NORMAL
    assert rec.trigger_signal == 'information_age_s'
    assert 'STALE_DISTRIBUTED_RESERVATIONS' in rec.reason


def test_packet_loss_triggers_low() -> None:
    """Verify that packet loss exceeding threshold triggers LOW."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0, confirmation_samples=3
    )
    policy = AdaptiveComputePolicy(robot_id='amr_1', config=cfg)

    for t in [1.0, 1.5]:
        policy.evaluate(ComputeState(timestamp=t, comm_loss_rate=0.20))
    mode, rec = policy.evaluate(
        ComputeState(timestamp=2.0, comm_loss_rate=0.20)
    )

    assert mode == ComputeMode.LOW
    assert rec is not None
    assert rec.trigger_signal == 'comm_loss_rate'


def test_cpu_pressure_triggers_low() -> None:
    """Verify that host CPU saturation triggers LOW mode."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0, confirmation_samples=3
    )
    policy = AdaptiveComputePolicy(robot_id='amr_2', config=cfg)

    for t in [1.0, 1.5]:
        policy.evaluate(ComputeState(timestamp=t, cpu_utilization_pct=90.0))
    mode, rec = policy.evaluate(
        ComputeState(timestamp=2.0, cpu_utilization_pct=90.0)
    )

    assert mode == ComputeMode.LOW
    assert rec is not None
    assert rec.trigger_signal == 'cpu_utilization_pct'


def test_spatial_contention_triggers_high() -> None:
    """Verify that multiple space-time conflicts trigger HIGH mode."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0, confirmation_samples=3
    )
    policy = AdaptiveComputePolicy(robot_id='amr_3', config=cfg)

    for t in [1.0, 1.5]:
        policy.evaluate(
            ComputeState(
                timestamp=t, active_conflicts=3, information_age_s=0.2
            )
        )
    mode, rec = policy.evaluate(
        ComputeState(
            timestamp=2.0, active_conflicts=3, information_age_s=0.2
        )
    )

    assert mode == ComputeMode.HIGH
    assert rec is not None
    assert rec.new_mode == ComputeMode.HIGH
    assert rec.trigger_signal == 'active_conflicts'


def test_degraded_network_preempts_spatial_contention() -> None:
    """Verify network degradation takes precedence over traffic contention."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0, confirmation_samples=3
    )
    policy = AdaptiveComputePolicy(robot_id='amr_0', config=cfg)

    # Both degraded comm (1.8s stale) AND high conflicts (5 conflicts)
    for t in [1.0, 1.5]:
        policy.evaluate(
            ComputeState(
                timestamp=t, information_age_s=1.8, active_conflicts=5
            )
        )
    mode, rec = policy.evaluate(
        ComputeState(
            timestamp=2.0, information_age_s=1.8, active_conflicts=5
        )
    )

    # Must choose LOW, not HIGH
    assert mode == ComputeMode.LOW
    assert rec is not None
    assert rec.new_mode == ComputeMode.LOW


def test_anti_oscillation_dwell_time() -> None:
    """Verify minimum dwell time prevents premature mode switching."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=5.0, confirmation_samples=2
    )
    policy = AdaptiveComputePolicy(robot_id='amr_0', config=cfg)

    # Transition to LOW at t=2.0
    policy.evaluate(ComputeState(timestamp=1.0, information_age_s=2.0))
    mode, _ = policy.evaluate(
        ComputeState(timestamp=2.0, information_age_s=2.0)
    )
    assert mode == ComputeMode.LOW

    # At t=3.0 and t=4.0 (within 5.0s dwell), network clears & conflicts occur
    policy.evaluate(
        ComputeState(timestamp=3.0, information_age_s=0.1, active_conflicts=4)
    )
    mode, rec = policy.evaluate(
        ComputeState(timestamp=4.0, information_age_s=0.1, active_conflicts=4)
    )

    # Dwell time prevents switch to HIGH
    assert mode == ComputeMode.LOW
    assert rec is None

    # After dwell time elapses (t=7.5 and t=8.0), switch is permitted
    policy.evaluate(
        ComputeState(timestamp=7.5, information_age_s=0.1, active_conflicts=4)
    )
    mode, rec = policy.evaluate(
        ComputeState(timestamp=8.0, information_age_s=0.1, active_conflicts=4)
    )
    assert mode == ComputeMode.HIGH
    assert rec is not None


def test_anti_oscillation_transient_spikes() -> None:
    """Verify that a transient 1-sample spike does not switch mode."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0, confirmation_samples=3
    )
    policy = AdaptiveComputePolicy(robot_id='amr_0', config=cfg)

    # Sample 1: normal
    mode, _ = policy.evaluate(ComputeState(timestamp=1.0, active_conflicts=0))
    assert mode == ComputeMode.NORMAL

    # Sample 2: brief spike of 4 conflicts
    mode, rec = policy.evaluate(
        ComputeState(timestamp=1.5, active_conflicts=4)
    )
    assert mode == ComputeMode.NORMAL
    assert rec is None

    # Sample 3: back to normal (0 conflicts) -> confirmations reset
    mode, rec = policy.evaluate(
        ComputeState(timestamp=2.0, active_conflicts=0)
    )
    assert mode == ComputeMode.NORMAL
    assert rec is None
    assert policy.candidate_confirmations == 0


def test_asymmetric_hysteresis_recovery() -> None:
    """Verify that recovery from LOW requires sustained recovery values."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0,
        confirmation_samples=2,
        stale_age_low_threshold_s=1.5,
        stale_age_normal_recovery_s=0.8,
    )
    policy = AdaptiveComputePolicy(robot_id='amr_0', config=cfg)

    # Transition to LOW
    policy.evaluate(ComputeState(timestamp=1.0, information_age_s=2.0))
    mode, _ = policy.evaluate(
        ComputeState(timestamp=2.0, information_age_s=2.0)
    )
    assert mode == ComputeMode.LOW

    # Stale age drops to 1.1s (below 1.5s trigger, but above 0.8s recovery)
    policy.evaluate(ComputeState(timestamp=3.5, information_age_s=1.1))
    mode, rec = policy.evaluate(
        ComputeState(timestamp=4.0, information_age_s=1.1)
    )
    # Hysteresis keeps it in LOW
    assert mode == ComputeMode.LOW
    assert rec is None

    # Stale age drops to 0.5s (below 0.8s recovery threshold)
    policy.evaluate(ComputeState(timestamp=4.5, information_age_s=0.5))
    mode, rec = policy.evaluate(
        ComputeState(timestamp=5.0, information_age_s=0.5)
    )
    assert mode == ComputeMode.NORMAL
    assert rec is not None
    assert rec.new_mode == ComputeMode.NORMAL


def test_fail_safe_fallback() -> None:
    """Verify null state or telemetry timeout forces fallback to NORMAL."""
    cfg = AdaptiveComputePolicyConfig(
        min_dwell_time_sec=1.0,
        confirmation_samples=2,
        telemetry_timeout_s=3.0,
    )
    policy = AdaptiveComputePolicy(robot_id='amr_0', config=cfg)

    # Put into HIGH mode
    policy.evaluate(ComputeState(timestamp=1.0, active_conflicts=3))
    mode, _ = policy.evaluate(ComputeState(timestamp=1.5, active_conflicts=3))
    assert mode == ComputeMode.HIGH

    # Telemetry timeout: timestamp jumps by 10 seconds (> 3.0s timeout)
    mode, rec = policy.evaluate(
        ComputeState(timestamp=12.0, active_conflicts=3)
    )
    assert mode == ComputeMode.NORMAL
    assert rec is not None
    assert 'TELEMETRY_TIMEOUT' in rec.reason

    # Null state fallback test
    policy.current_mode = ComputeMode.LOW
    mode, rec = policy.evaluate(None)
    assert mode == ComputeMode.NORMAL
    assert 'NULL_STATE' in rec.reason


def test_deterministic_replay() -> None:
    """Verify identical input traces produce bitwise identical decisions."""
    trace: List[ComputeState] = [
        ComputeState(timestamp=1.0, active_conflicts=0, information_age_s=0.1),
        ComputeState(timestamp=1.5, active_conflicts=3, information_age_s=0.1),
        ComputeState(timestamp=2.0, active_conflicts=3, information_age_s=0.1),
        ComputeState(timestamp=2.5, active_conflicts=3, information_age_s=0.1),
        ComputeState(timestamp=6.0, active_conflicts=0, information_age_s=2.0),
        ComputeState(timestamp=6.5, active_conflicts=0, information_age_s=2.0),
        ComputeState(timestamp=7.0, active_conflicts=0, information_age_s=2.0),
    ]

    p1 = AdaptiveComputePolicy(
        robot_id='amr_0',
        config=AdaptiveComputePolicyConfig(
            min_dwell_time_sec=2.0, confirmation_samples=2
        ),
    )
    p2 = AdaptiveComputePolicy(
        robot_id='amr_0',
        config=AdaptiveComputePolicyConfig(
            min_dwell_time_sec=2.0, confirmation_samples=2
        ),
    )

    modes1 = [p1.evaluate(s)[0] for s in trace]
    modes2 = [p2.evaluate(s)[0] for s in trace]

    assert modes1 == modes2
    assert len(p1.transition_history) == len(p2.transition_history)
    for r1, r2 in zip(p1.transition_history, p2.transition_history):
        assert r1.new_mode == r2.new_mode
        assert r1.previous_mode == r2.previous_mode
        assert r1.trigger_signal == r2.trigger_signal


def test_occupancy_stats_calculation() -> None:
    """Verify occupancy stats compute percentages across active modes."""
    policy = AdaptiveComputePolicy(robot_id='amr_0')
    policy.mode_durations[ComputeMode.LOW] = 20.0
    policy.mode_durations[ComputeMode.NORMAL] = 60.0
    policy.mode_durations[ComputeMode.HIGH] = 20.0

    stats = policy.get_occupancy_stats(total_duration=100.0)
    assert stats['mode_occupancy_low_pct'] == 20.0
    assert stats['mode_occupancy_normal_pct'] == 60.0
    assert stats['mode_occupancy_high_pct'] == 20.0

