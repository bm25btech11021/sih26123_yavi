"""
Communication Impairment Model for Multi-Agent Fleets (Milestone M7).

Provides configurable, reproducible communication degradation at the
communication boundary:
- Latency (fixed delay in ms)
- Jitter (latency variation around mean)
- Independent packet loss (probabilistic)
- Burst packet loss (Gilbert-Elliott Markov chain)
- Temporary communication outage (start time and duration)
- Communication partition (pairwise isolation between sub-fleets)
- Reconnection and state recovery
- Full network telemetry metrics
"""

from dataclasses import dataclass, field
from enum import Enum
import math
import random
from typing import Any, Callable, Dict, List, Optional, Set, Tuple


class DelayedMessageQueue:
    """Non-blocking queue for delayed message delivery."""

    def __init__(self) -> None:
        self._queue: List[Tuple[float, Callable[[Any], None], Any]] = []

    def schedule(
        self,
        deliver_at: float,
        callback: Callable[[Any], None],
        msg: Any,
    ) -> None:
        """Schedule a message callback for future delivery."""
        self._queue.append((deliver_at, callback, msg))

    def poll(self, current_time: float) -> int:
        """Deliver all messages whose scheduled delivery time has arrived."""
        delivered_count = 0
        still_pending = []
        for deliver_at, cb, msg in self._queue:
            if deliver_at <= current_time:
                cb(msg)
                delivered_count += 1
            else:
                still_pending.append((deliver_at, cb, msg))
        self._queue = still_pending
        return delivered_count

    def clear(self) -> None:
        """Clear pending messages."""
        self._queue.clear()

    def dispatch_ready(self, current_time: float) -> int:
        """Alias for poll: deliver all messages ready by current_time."""
        return self.poll(current_time)

    def __len__(self) -> int:
        """Return count of pending messages."""
        return len(self._queue)

    @property
    def pending_count(self) -> int:
        """Return number of pending messages."""
        return len(self._queue)


class CommunicationAction(Enum):
    """Action determined by the impairment model for an in-flight message."""

    DELIVER = 'DELIVER'
    DELAY = 'DELAY'
    DROP = 'DROP'


class DropReason(Enum):
    """Reason why a message was dropped."""

    NONE = 'NONE'
    INDEPENDENT_LOSS = 'INDEPENDENT_LOSS'
    BURST_LOSS = 'BURST_LOSS'
    OUTAGE = 'OUTAGE'
    PARTITION = 'PARTITION'


@dataclass
class CommunicationProfileConfig:
    """Configuration parameters for communication impairment."""

    profile_name: str = 'NORMAL'
    enabled: bool = False
    latency_ms: float = 0.0
    jitter_ms: float = 0.0
    loss_probability: float = 0.0
    burst_loss_probability: float = 0.0
    burst_length_mean: float = 4.0
    outage_start_s: float = 0.0
    outage_duration_s: float = 0.0
    seed: int = 42
    isolated_robots: List[str] = field(default_factory=list)


# Standard Preset Profiles
PRESET_PROFILES: Dict[str, CommunicationProfileConfig] = {
    'NORMAL': CommunicationProfileConfig(
        profile_name='NORMAL',
        enabled=False,
        latency_ms=0.0,
        jitter_ms=0.0,
        loss_probability=0.0,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'LOW_LATENCY': CommunicationProfileConfig(
        profile_name='LOW_LATENCY',
        enabled=True,
        latency_ms=100.0,
        jitter_ms=10.0,
        loss_probability=0.0,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'HIGH_LATENCY': CommunicationProfileConfig(
        profile_name='HIGH_LATENCY',
        enabled=True,
        latency_ms=400.0,
        jitter_ms=50.0,
        loss_probability=0.0,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'JITTER': CommunicationProfileConfig(
        profile_name='JITTER',
        enabled=True,
        latency_ms=150.0,
        jitter_ms=100.0,
        loss_probability=0.0,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'LOSS_LOW': CommunicationProfileConfig(
        profile_name='LOSS_LOW',
        enabled=True,
        latency_ms=0.0,
        jitter_ms=0.0,
        loss_probability=0.15,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'LOSS_HIGH': CommunicationProfileConfig(
        profile_name='LOSS_HIGH',
        enabled=True,
        latency_ms=0.0,
        jitter_ms=0.0,
        loss_probability=0.35,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'BURST_LOSS': CommunicationProfileConfig(
        profile_name='BURST_LOSS',
        enabled=True,
        latency_ms=0.0,
        jitter_ms=0.0,
        loss_probability=0.05,
        burst_loss_probability=0.25,
        burst_length_mean=4.0,
        outage_duration_s=0.0,
        seed=42,
    ),
    'OUTAGE': CommunicationProfileConfig(
        profile_name='OUTAGE',
        enabled=True,
        latency_ms=0.0,
        jitter_ms=0.0,
        loss_probability=0.0,
        burst_loss_probability=0.0,
        outage_start_s=2.0,
        outage_duration_s=5.0,
        seed=42,
    ),
    'OUTAGE_RECOVERY': CommunicationProfileConfig(
        profile_name='OUTAGE_RECOVERY',
        enabled=True,
        latency_ms=50.0,
        jitter_ms=15.0,
        loss_probability=0.05,
        burst_loss_probability=0.0,
        outage_start_s=5.0,
        outage_duration_s=5.0,
        seed=42,
    ),
    'PARTITION': CommunicationProfileConfig(
        profile_name='PARTITION',
        enabled=True,
        latency_ms=20.0,
        jitter_ms=5.0,
        loss_probability=0.0,
        burst_loss_probability=0.0,
        outage_duration_s=0.0,
        seed=42,
        isolated_robots=['amr_3', 'amr_4'],
    ),
}


class GilbertElliottModel:
    """Two-state Markov model for burst loss simulation."""

    def __init__(
        self,
        rng: random.Random,
        p_burst_start: float = 0.0,
        mean_burst_len: float = 4.0,
    ) -> None:
        self.rng = rng
        self.p_burst_start = max(0.0, min(1.0, float(p_burst_start)))
        burst_len = max(1.0, float(mean_burst_len))
        self.p_recover = 1.0 / burst_len
        self.in_bad_state: bool = False
        self.burst_events_count: int = 0

    def step(self) -> bool:
        """Transition the state and return True if packet should be dropped."""
        if not self.in_bad_state:
            if self.p_burst_start > 0.0 and self.rng.random() < self.p_burst_start:
                self.in_bad_state = True
                self.burst_events_count += 1
                return True
            return False
        else:
            if self.rng.random() < self.p_recover:
                self.in_bad_state = False
                return False
            return True


class CommunicationImpairmentModel:
    """Evaluates in-flight messages against configured network degradation parameters."""

    def __init__(self, config: Optional[CommunicationProfileConfig] = None) -> None:
        self.config = config or CommunicationProfileConfig()
        self.rng = random.Random(self.config.seed)
        self.burst_model = GilbertElliottModel(
            rng=self.rng,
            p_burst_start=self.config.burst_loss_probability,
            mean_burst_len=self.config.burst_length_mean,
        )
        self.isolated_set: Set[str] = set(self.config.isolated_robots)

        self.messages_sent: int = 0
        self.messages_delivered: int = 0
        self.messages_dropped: int = 0
        self.drop_counts: Dict[DropReason, int] = {reason: 0 for reason in DropReason}
        self.latencies_ms: List[float] = []

    def set_config(self, config: CommunicationProfileConfig) -> None:
        """Update configuration and reinitialize deterministic RNG."""
        self.config = config
        self.rng = random.Random(config.seed)
        self.burst_model = GilbertElliottModel(
            rng=self.rng,
            p_burst_start=config.burst_loss_probability,
            mean_burst_len=config.burst_length_mean,
        )
        self.isolated_set = set(config.isolated_robots)

    def load_preset(self, preset_name: str) -> bool:
        """Load a predefined profile by name."""
        upper_name = preset_name.upper()
        if upper_name in PRESET_PROFILES:
            self.set_config(PRESET_PROFILES[upper_name])
            return True
        return False

    def is_in_outage(self, current_time: float) -> bool:
        """Check if network is currently in an outage window."""
        if not self.config.enabled or self.config.outage_duration_s <= 0.0:
            return False
        start = self.config.outage_start_s
        end = start + self.config.outage_duration_s
        return start <= current_time < end

    def is_partitioned(self, sender_id: str, receiver_id: str) -> bool:
        """Check if sender and receiver are separated by a network partition."""
        if not self.isolated_set:
            return False
        sender_isolated = sender_id in self.isolated_set
        receiver_isolated = receiver_id in self.isolated_set
        return sender_isolated != receiver_isolated

    def process_message(
        self,
        sender_id: str,
        receiver_id: str,
        current_time: float = 0.0,
    ) -> Tuple[CommunicationAction, float, DropReason]:
        """Evaluate message transmission and return action, delay, and reason."""
        self.messages_sent += 1

        if not self.config.enabled:
            self.messages_delivered += 1
            return CommunicationAction.DELIVER, 0.0, DropReason.NONE

        if self.is_partitioned(sender_id, receiver_id):
            self.messages_dropped += 1
            self.drop_counts[DropReason.PARTITION] += 1
            return CommunicationAction.DROP, 0.0, DropReason.PARTITION

        if self.is_in_outage(current_time):
            self.messages_dropped += 1
            self.drop_counts[DropReason.OUTAGE] += 1
            return CommunicationAction.DROP, 0.0, DropReason.OUTAGE

        if self.burst_model.step():
            self.messages_dropped += 1
            self.drop_counts[DropReason.BURST_LOSS] += 1
            return CommunicationAction.DROP, 0.0, DropReason.BURST_LOSS

        if (
            self.config.loss_probability > 0.0
            and self.rng.random() < self.config.loss_probability
        ):
            self.messages_dropped += 1
            self.drop_counts[DropReason.INDEPENDENT_LOSS] += 1
            return CommunicationAction.DROP, 0.0, DropReason.INDEPENDENT_LOSS

        delay_ms = self.config.latency_ms
        if self.config.jitter_ms > 0.0:
            jitter = self.rng.uniform(-self.config.jitter_ms, self.config.jitter_ms)
            delay_ms = max(0.0, delay_ms + jitter)

        self.messages_delivered += 1
        self.latencies_ms.append(delay_ms)

        if delay_ms > 1.0:
            return CommunicationAction.DELAY, delay_ms, DropReason.NONE
        return CommunicationAction.DELIVER, 0.0, DropReason.NONE

    def get_metrics(self) -> Dict[str, Any]:
        """Calculate and return real-time network telemetry."""
        total = self.messages_sent
        delivered = self.messages_delivered
        dropped = self.messages_dropped
        loss_rate = float(dropped) / float(total) if total > 0 else 0.0

        if self.latencies_ms:
            sorted_lat = sorted(self.latencies_ms)
            avg_lat = sum(sorted_lat) / len(sorted_lat)
            p50_idx = int(0.50 * len(sorted_lat))
            p95_idx = min(int(0.95 * len(sorted_lat)), len(sorted_lat) - 1)
            p50_lat = sorted_lat[p50_idx]
            p95_lat = sorted_lat[p95_idx]
            variance = sum((x - avg_lat) ** 2 for x in sorted_lat) / len(sorted_lat)
            jitter_calc = math.sqrt(variance)
        else:
            avg_lat = 0.0
            p50_lat = 0.0
            p95_lat = 0.0
            jitter_calc = 0.0

        return {
            'profile_name': self.config.profile_name,
            'enabled': self.config.enabled,
            'messages_sent': total,
            'messages_delivered': delivered,
            'messages_dropped': dropped,
            'packet_loss_rate': round(loss_rate, 4),
            'avg_latency_ms': round(avg_lat, 2),
            'p50_latency_ms': round(p50_lat, 2),
            'p95_latency_ms': round(p95_lat, 2),
            'jitter_ms': round(jitter_calc, 2),
            'burst_events_count': self.burst_model.burst_events_count,
            'drop_breakdown': {
                reason.value: count for reason, count in self.drop_counts.items()
            },
        }

    def reset_metrics(self) -> None:
        """Reset telemetry counters."""
        self.messages_sent = 0
        self.messages_delivered = 0
        self.messages_dropped = 0
        self.drop_counts = {reason: 0 for reason in DropReason}
        self.latencies_ms.clear()
        self.burst_model.burst_events_count = 0

