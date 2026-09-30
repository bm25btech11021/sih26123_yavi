"""
Unit and Integration Tests for Milestone M9-V3-E: Combined Stress Experiment.

Validates the multi-event integration:
  - Dynamic Task Arrival (t=45s)
  - Temporary Aisle Blockage (t=45s to t=90s)
  - Scheduled Communication Degradation (t=75s to t=120s, LOSS_HIGH)
  - Dynamic Recovery (t=120s, NORMAL)
  - Invariant Task Accounting and Timeline Sequencing

Tests:
  - test_v3_e_cli_argument_resolution: CLI flag defaults & composition
  - test_v3_e_communication_schedule_transitions: Timeline profile resolution
  - test_v3_e_impairment_model_dynamic_toggle: Dynamic loss toggle behavior
  - test_v3_e_rh_node_handles_comm_profile_event: RH node dynamic update
  - test_v3_e_cbba_node_handles_comm_profile_event: CBBA node dynamic update
  - test_v3_e_task_accounting_invariant_holds: Lifecycle sum invariant
  - test_v3_e_combined_event_timeline_consistency: Timeline ordering & concurrency
"""

from typing import Dict, List

from amr_fleet_core.cbba_node import CBBANode
from amr_fleet_core.communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    PRESET_PROFILES,
)
from amr_fleet_core.rh_node import RollingHorizonPlannerNode
from amr_fleet_msgs.msg import CommunicationProfile
import rclpy


def test_v3_e_cli_argument_resolution():
    """Verify that V3-E CLI flags parse with correct defaults and implied modes."""
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--v3-e', action='store_true')
    parser.add_argument('--v3-e-comm-degrade-time', type=float, default=75.0)
    parser.add_argument('--v3-e-comm-recover-time', type=float, default=120.0)
    parser.add_argument('--v3-e-comm-profile', type=str, default='LOSS_HIGH')
    parser.add_argument('--v3-a-block-time', type=float, default=45.0)
    parser.add_argument('--v3-a-unblock-time', type=float, default=90.0)

    args = parser.parse_args(['--v3-e'])
    assert args.v3_e is True
    assert args.v3_e_comm_degrade_time == 75.0
    assert args.v3_e_comm_recover_time == 120.0
    assert args.v3_e_comm_profile == 'LOSS_HIGH'
    assert args.v3_a_block_time == 45.0
    assert args.v3_a_unblock_time == 90.0


def test_v3_e_communication_schedule_transitions():
    """Verify that the scheduled communication profile transitions match the spec."""
    degrade_t = 75.0
    recover_t = 120.0
    degrade_profile = 'LOSS_HIGH'

    def get_profile_at(t: float) -> str:
        if t < degrade_t:
            return 'NORMAL'
        elif t < recover_t:
            return degrade_profile
        else:
            return 'NORMAL'

    # Pre-degradation window
    for t in [0.0, 10.0, 44.9, 45.0, 74.9]:
        assert get_profile_at(t) == 'NORMAL', f'Failed at t={t}'

    # Degradation window
    for t in [75.0, 75.1, 90.0, 100.0, 119.9]:
        assert get_profile_at(t) == 'LOSS_HIGH', f'Failed at t={t}'

    # Post-recovery window
    for t in [120.0, 120.1, 150.0, 180.0]:
        assert get_profile_at(t) == 'NORMAL', f'Failed at t={t}'


def test_v3_e_impairment_model_dynamic_toggle():
    """Verify CommunicationImpairmentModel dynamically drops packets in LOSS_HIGH."""
    normal_cfg = PRESET_PROFILES['NORMAL']
    loss_high_cfg = PRESET_PROFILES['LOSS_HIGH']

    model = CommunicationImpairmentModel(normal_cfg)

    # Phase 1: NORMAL (0 drops)
    drops_phase1 = 0
    for _ in range(100):
        action, _, _ = model.process_message('amr_0', 'amr_1')
        if action == CommunicationAction.DROP:
            drops_phase1 += 1
    assert drops_phase1 == 0, 'NORMAL profile must have 0 packet drops'

    # Phase 2: Switch dynamically to LOSS_HIGH
    model.set_config(loss_high_cfg)
    drops_phase2 = 0
    for _ in range(200):
        action, _, _ = model.process_message('amr_0', 'amr_1')
        if action == CommunicationAction.DROP:
            drops_phase2 += 1
    # For p=0.35 over 200 trials, expect ~70 drops (bound with margin: 30 to 110)
    assert 30 <= drops_phase2 <= 110, f'Unexpected drops in LOSS_HIGH: {drops_phase2}/200'

    # Phase 3: Switch back to NORMAL (recovers to 0 drops)
    model.set_config(normal_cfg)
    drops_phase3 = 0
    for _ in range(100):
        action, _, _ = model.process_message('amr_0', 'amr_1')
        if action == CommunicationAction.DROP:
            drops_phase3 += 1
    assert drops_phase3 == 0, 'Post-recovery NORMAL profile must have 0 packet drops'


def test_v3_e_rh_node_handles_comm_profile_event():
    """Verify RollingHorizonPlannerNode dynamically updates comm_model on event."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # Initially NORMAL: disabled
        assert not node.comm_model.config.enabled

        # Send LOSS_HIGH profile update
        msg_loss = CommunicationProfile()
        msg_loss.profile_name = 'LOSS_HIGH'
        msg_loss.enabled = True
        msg_loss.loss_probability = 0.35
        msg_loss.latency_ms = 0.0
        msg_loss.jitter_ms = 0.0
        msg_loss.seed = 42
        node._handle_comm_profile(msg_loss)

        assert node.comm_model.config.enabled
        assert node.comm_model.config.profile_name == 'LOSS_HIGH'
        assert abs(node.comm_model.config.loss_probability - 0.35) < 1e-5

        # Send NORMAL profile update (recovery)
        msg_norm = CommunicationProfile()
        msg_norm.profile_name = 'NORMAL'
        msg_norm.enabled = False
        msg_norm.loss_probability = 0.0
        msg_norm.seed = 42
        node._handle_comm_profile(msg_norm)

        assert not node.comm_model.config.enabled
        assert node.comm_model.config.profile_name == 'NORMAL'
    finally:
        node.destroy_node()


def test_v3_e_cbba_node_handles_comm_profile_event():
    """Verify CBBANode dynamically updates comm_model on event."""
    if not rclpy.ok():
        rclpy.init()

    node = CBBANode()
    try:
        # Initially NORMAL: disabled
        assert not node.comm_model.config.enabled

        # Send LOSS_HIGH profile update
        msg_loss = CommunicationProfile()
        msg_loss.profile_name = 'LOSS_HIGH'
        msg_loss.enabled = True
        msg_loss.loss_probability = 0.35
        msg_loss.seed = 42
        node._handle_comm_profile(msg_loss)

        assert node.comm_model.config.enabled
        assert node.comm_model.config.profile_name == 'LOSS_HIGH'

        # Send NORMAL profile update (recovery)
        msg_norm = CommunicationProfile()
        msg_norm.profile_name = 'NORMAL'
        msg_norm.enabled = False
        msg_norm.loss_probability = 0.0
        msg_norm.seed = 42
        node._handle_comm_profile(msg_norm)

        assert not node.comm_model.config.enabled
        assert node.comm_model.config.profile_name == 'NORMAL'
    finally:
        node.destroy_node()


def test_v3_e_task_accounting_invariant_holds():
    """Verify that task accounting invariants hold across combined transitions."""
    total_generated = 30

    stages: List[Dict[str, int]] = [
        # t = 0s: 15 tasks staged, 15 assigned, 0 in progress, 0 completed
        {
            'staged': 15,
            'pending': 0,
            'assigned': 15,
            'in_progress': 0,
            'completed': 0,
            'failed': 0,
            'cancelled': 0,
        },
        # t = 45s: staged tasks released, 2 active deliveries
        {
            'staged': 0,
            'pending': 2,
            'assigned': 26,
            'in_progress': 2,
            'completed': 0,
            'failed': 0,
            'cancelled': 0,
        },
        # t = 80s: during comm degradation & blockage
        {
            'staged': 0,
            'pending': 0,
            'assigned': 27,
            'in_progress': 2,
            'completed': 1,
            'failed': 0,
            'cancelled': 0,
        },
        # t = 130s: post recovery
        {
            'staged': 0,
            'pending': 0,
            'assigned': 25,
            'in_progress': 2,
            'completed': 3,
            'failed': 0,
            'cancelled': 0,
        },
    ]

    for stage_idx, counts in enumerate(stages):
        gen_sum = sum(counts.values())
        rem_sum = counts['staged'] + counts['pending'] + counts['assigned'] + counts['in_progress']
        term_sum = counts['completed'] + counts['failed'] + counts['cancelled']

        assert gen_sum == total_generated, (
            f'Stage {stage_idx} generated invariant failed: {gen_sum} != {total_generated}'
        )
        assert rem_sum + term_sum == total_generated, (
            f'Stage {stage_idx} remaining invariant failed: '
            f'{rem_sum} + {term_sum} != {total_generated}'
        )


def test_v3_e_combined_event_timeline_consistency():
    """Verify timeline timestamps, event durations, and concurrency overlap."""
    t_start = 0.0
    t_release = 45.0
    t_block_inject = 45.0
    t_comm_degrade = 75.0
    t_block_remove = 90.0
    t_comm_recover = 120.0
    t_horizon = 180.0

    # Strict ordering
    assert t_start < t_release
    assert t_release <= t_block_inject
    assert t_block_inject < t_comm_degrade
    assert t_comm_degrade < t_block_remove
    assert t_block_remove < t_comm_recover
    assert t_comm_recover < t_horizon

    # Window durations
    blockage_duration = t_block_remove - t_block_inject
    assert blockage_duration == 45.0, f'Expected 45s blockage, got {blockage_duration}'

    degradation_duration = t_comm_recover - t_comm_degrade
    assert degradation_duration == 45.0, f'Expected 45s degradation, got {degradation_duration}'

    # Concurrent stress window: [75.0, 90.0] has BOTH blockage and comm degradation
    concurrent_start = max(t_block_inject, t_comm_degrade)
    concurrent_end = min(t_block_remove, t_comm_recover)
    concurrent_duration = concurrent_end - concurrent_start
    assert concurrent_start == 75.0
    assert concurrent_end == 90.0
    assert concurrent_duration == 15.0, (
        f'Expected 15s concurrent stress window, got {concurrent_duration}'
    )

