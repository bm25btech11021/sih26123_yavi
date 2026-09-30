"""
Stale Information Tracking and State Expiration Manager (Milestone M7).

Tracks message arrival timestamps, computes information age, classifies state:
- CURRENT: Age < T_stale (e.g., < 1.5s) -> Fully valid and trusted.
- STALE: T_stale <= Age < T_expire (e.g., 1.5s to 4.0s) -> Valid with caution.
- EXPIRED: Age >= T_expire (e.g., >= 4.0s) -> Untrusted, eligible for pruning.
- MISSING: Never received from peer.

Manages remote reservation expiry and triggers reconnection reconciliation.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Tuple


class InformationState(Enum):
    """Classification of received peer information based on age."""

    CURRENT = 'CURRENT'
    STALE = 'STALE'
    EXPIRED = 'EXPIRED'
    MISSING = 'MISSING'


@dataclass
class PeerRecord:
    """Record of communication history for a specific peer robot."""

    robot_id: str
    last_sent_timestamp: float = -1.0
    last_receive_timestamp: float = -1.0
    message_count: int = 0
    last_known_payload: Any = None
    was_expired: bool = False

    def age(self, current_time: float) -> float:
        """Compute information age in seconds."""
        if self.last_receive_timestamp < 0.0:
            return float('inf')
        return max(0.0, current_time - self.last_receive_timestamp)

    def evaluate_state(
        self,
        current_time: float,
        stale_threshold_s: float = 1.5,
        expiry_threshold_s: float = 4.0,
    ) -> InformationState:
        """Evaluate state classification based on age."""
        if self.last_receive_timestamp < 0.0:
            return InformationState.MISSING
        age_val = self.age(current_time)
        if age_val < stale_threshold_s:
            return InformationState.CURRENT
        elif age_val < expiry_threshold_s:
            return InformationState.STALE
        else:
            return InformationState.EXPIRED


class StaleStateManager:
    """Manages peer freshness tracking and state expiration for an AMR node."""

    def __init__(
        self,
        local_robot_id: str,
        stale_threshold_s: float = 1.5,
        expiry_threshold_s: float = 4.0,
    ) -> None:
        self.local_robot_id = local_robot_id
        self.stale_threshold_s = float(stale_threshold_s)
        self.expiry_threshold_s = float(expiry_threshold_s)

        # Peer records per topic channel: channel -> robot_id -> PeerRecord
        self.channels: Dict[str, Dict[str, PeerRecord]] = {
            'reservations': {},
            'cbba_bids': {},
            'coordination_status': {},
            'conflicts': {},
            'deadlocks': {},
        }

        # Telemetry
        self.total_stale_observations: int = 0
        self.total_expired_events: int = 0
        self.reconnection_events_count: int = 0

    def record_incoming(
        self,
        channel: str,
        sender_id: str,
        sent_timestamp: float,
        receive_timestamp: float,
        payload: Any = None,
    ) -> Tuple[InformationState, bool]:
        """Record message arrival, update peer record, and detect reconnection."""
        if channel not in self.channels:
            self.channels[channel] = {}

        channel_records = self.channels[channel]
        is_reconnection = False

        if sender_id not in channel_records:
            record = PeerRecord(
                robot_id=sender_id,
                last_sent_timestamp=sent_timestamp,
                last_receive_timestamp=receive_timestamp,
                message_count=1,
                last_known_payload=payload,
                was_expired=False,
            )
            channel_records[sender_id] = record
            state = InformationState.CURRENT
        else:
            record = channel_records[sender_id]
            prev_state = record.evaluate_state(
                current_time=receive_timestamp,
                stale_threshold_s=self.stale_threshold_s,
                expiry_threshold_s=self.expiry_threshold_s,
            )
            if prev_state in (InformationState.EXPIRED, InformationState.MISSING):
                is_reconnection = True
                self.reconnection_events_count += 1

            record.last_sent_timestamp = sent_timestamp
            record.last_receive_timestamp = receive_timestamp
            record.message_count += 1
            record.last_known_payload = payload
            record.was_expired = False
            state = InformationState.CURRENT

        return state, is_reconnection

    def get_peer_state(
        self, channel: str, peer_id: str, current_time: float
    ) -> InformationState:
        """Get the current information state of a specific peer on a channel."""
        records = self.channels.get(channel, {})
        record = records.get(peer_id)
        if record is None:
            return InformationState.MISSING
        state = record.evaluate_state(
            current_time,
            stale_threshold_s=self.stale_threshold_s,
            expiry_threshold_s=self.expiry_threshold_s,
        )
        if state == InformationState.STALE:
            self.total_stale_observations += 1
        elif state == InformationState.EXPIRED and not record.was_expired:
            record.was_expired = True
            self.total_expired_events += 1
        return state

    def get_expired_peers(self, channel: str, current_time: float) -> List[str]:
        """Identify all peers whose state has expired on a given channel."""
        records = self.channels.get(channel, {})
        expired = []
        for p_id, rec in records.items():
            if (
                rec.evaluate_state(
                    current_time,
                    stale_threshold_s=self.stale_threshold_s,
                    expiry_threshold_s=self.expiry_threshold_s,
                )
                == InformationState.EXPIRED
            ):
                expired.append(p_id)
        return expired

    def prune_expired_reservations(
        self,
        reservation_table: Any,
        current_time: float,
    ) -> List[str]:
        """Prune reservations belonging to peers that have not communicated within threshold."""
        expired_peers = self.get_expired_peers('reservations', current_time)
        pruned_peers = []
        for p_id in expired_peers:
            if hasattr(reservation_table, 'release_robot'):
                if p_id != self.local_robot_id:
                    reservation_table.release_robot(p_id)
                    pruned_peers.append(p_id)
        return pruned_peers

    def get_stale_summary(self, current_time: float) -> Dict[str, Any]:
        """Provide diagnostic summary of information freshness across all peers."""
        summary: Dict[str, Dict[str, Dict[str, Any]]] = {}
        stale_count = 0
        expired_count = 0

        for chan_name, peer_dict in self.channels.items():
            summary[chan_name] = {}
            for p_id, rec in peer_dict.items():
                st = rec.evaluate_state(
                    current_time,
                    stale_threshold_s=self.stale_threshold_s,
                    expiry_threshold_s=self.expiry_threshold_s,
                )
                age = rec.age(current_time)
                summary[chan_name][p_id] = {
                    'state': st.value,
                    'age_s': round(age, 2),
                    'messages': rec.message_count,
                }
                if st == InformationState.STALE:
                    stale_count += 1
                elif st == InformationState.EXPIRED:
                    expired_count += 1

        return {
            'stale_count': stale_count,
            'expired_count': expired_count,
            'reconnections_count': self.reconnection_events_count,
            'channels': summary,
        }

