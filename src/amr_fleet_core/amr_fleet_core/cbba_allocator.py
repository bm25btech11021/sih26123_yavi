"""Decentralized CBBA Allocator implementing the TaskAllocator interface."""

from typing import Any, Dict, List, Optional, Tuple

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.interfaces import TaskAllocator


class CBBAAllocator(TaskAllocator):
    """
    Decentralized Task Allocator utilizing the Consensus-Based Bundle Algorithm (CBBA).

    Simulates peer-to-peer consensus rounds among independent robot agents
    without a central master solver.
    """

    def __init__(self, config: Optional[CBBAConfig] = None) -> None:
        """Initialize CBBA Allocator with optional algorithm parameters."""
        self.config = config or CBBAConfig()
        self.last_iterations: int = 0
        self.last_converged: bool = False

    def allocate_tasks(
        self,
        robot_states: Dict[str, Any],
        tasks: List[Dict[str, Any]],
        epoch: int,
        network_state: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, List[str]], int]:
        """
        Allocate tasks to robots via decentralized CBBA bundle construction and consensus.

        :param robot_states: Mapping of robot_id to robot state (position, status).
        :param tasks: List of available unassigned task dicts.
        :param epoch: Current allocation epoch.
        :param network_state: Optional network topology or communication constraints.
        :return: Tuple of (assignments dict {robot_id: [task_ids]}, updated epoch).
        """
        if not robot_states or not tasks:
            self.last_iterations = 0
            self.last_converged = True
            empty_assignments = {r_id: [] for r_id in robot_states}
            return empty_assignments, epoch + 1

        # Format task lookup map
        task_map: Dict[str, Any] = {}
        for t in tasks:
            t_id = t.get('task_id', str(t.get('id', '')))
            task_map[t_id] = {
                'task_id': t_id,
                'pickup': t.get('pickup', (0.0, 0.0)),
                'dropoff': t.get('dropoff', (0.0, 0.0)),
                'priority': t.get('priority', 2),
                'deadline': t.get('deadline'),
            }

        # Initialize independent agents
        agents: Dict[str, CBBAAgent] = {}
        sorted_robot_ids = sorted(robot_states.keys())
        for r_id in sorted_robot_ids:
            state = robot_states[r_id]
            pos = state.get('position', (0.0, 0.0))
            if isinstance(pos, dict):
                pos = (pos.get('x', 0.0), pos.get('y', 0.0))
            agents[r_id] = CBBAAgent(
                robot_id=r_id,
                config=self.config,
                initial_position=pos,
            )

        max_iterations = max(len(tasks) * len(agents) * 2, 30)
        converged = False
        iteration = 0

        while iteration < max_iterations and not converged:
            iteration += 1
            curr_time = float(epoch + iteration)

            # Phase 1: Bundle construction (independent)
            for r_id in sorted_robot_ids:
                agents[r_id].build_bundle(task_map, current_time=curr_time)

            # Phase 2: Multi-round consensus exchange until network stabilizes
            any_network_change = False
            for _ in range(len(sorted_robot_ids)):
                snapshots = {
                    r: (
                        dict(agents[r].state.winning_bids),
                        dict(agents[r].state.winning_robots),
                        dict(agents[r].state.timestamps),
                    )
                    for r in sorted_robot_ids
                }
                subround_change = False
                for r_id in sorted_robot_ids:
                    agent = agents[r_id]
                    for peer_id in sorted_robot_ids:
                        if peer_id == r_id:
                            continue
                        bids_p, robs_p, times_p = snapshots[peer_id]
                        changed = agent.resolve_conflicts(
                            peer_id=peer_id,
                            peer_iteration=iteration,
                            peer_winning_bids=bids_p,
                            peer_winning_robots=robs_p,
                            peer_timestamps=times_p,
                            task_map=task_map,
                            current_time=curr_time,
                        )
                        if changed:
                            subround_change = True
                            any_network_change = True
                if not subround_change:
                    break

            # Check fleet-wide consensus
            all_agree = True
            first_agent = agents[sorted_robot_ids[0]]
            for r_id in sorted_robot_ids[1:]:
                if not first_agent.is_consistent_with(agents[r_id]):
                    all_agree = False
                    break

            if not any_network_change and all_agree:
                converged = True
                break

        self.last_iterations = iteration
        self.last_converged = converged

        assignments: Dict[str, List[str]] = {
            r_id: list(agents[r_id].state.bundle) for r_id in sorted_robot_ids
        }
        return assignments, epoch + 1

