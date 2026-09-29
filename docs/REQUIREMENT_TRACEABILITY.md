# SIH26123 requirement traceability

This map records where the supplied implementation addresses the project needs. A source file or test is evidence of implementation intent; it does not by itself establish a measured guarantee. Run the relevant simulation and retain its configuration and output before making performance or safety claims.

| Need | Primary implementation | Verification/evidence |
|---|---|---|
| Configurable warehouse fleet | `src/amr_fleet_bringup/launch/`, `src/amr_fleet_description/`, `src/amr_fleet_sim/`, `config/robots/`, `config/maps/` | Gazebo launch and simulation package tests |
| Namespaced robot state | `src/amr_fleet_bringup/launch/`, `src/amr_fleet_msgs/` | Bringup launch/configuration tests and live ROS graph inspection |
| Distributed task bidding and ownership | `src/amr_fleet_core/amr_fleet_core/cbba_agent.py`, `cbba_allocator.py`, `cbba_node.py`, `task_model.py` | `src/amr_fleet_core/test/test_cbba_*.py` |
| Task creation, lifecycle, and reallocation | `task_manager_node.py`, `task_model.py`, `scripts/create_task.py` | `test_operator_task_control.py`, live task integration script |
| Rolling-horizon path planning | `rh_planner.py`, `rh_node.py`, `reservation_table.py` | `test_rh_*.py`, M6/M9 integration tests |
| Local conflict resolution and deadlock recovery | `pibt_planner.py`, `conflict_detector.py`, `wfg_deadlock.py`, `deadlock_recovery.py` | `test_m6_*.py`, `test_m9_v2_*.py` |
| Stale peers, communication impairments, and faults | `stale_state_manager.py`, `communication_model.py`, `fault_detector.py`, `fault_state.py`, `adversarial_injector.py` | M2/M3/M4/M7 resilience tests and scenario harnesses |
| Local obstacle sensing / stop authority | `local_obstacle_detector.py`, robot/controller launch and configuration | Safety-backstop tests; Gazebo integration still required |
| Compute-aware planning policy | `adaptive_compute_policy.py`, `compute_modes.py` | `test_m8b_adaptive_compute.py` |
| Fleet observability and adversarial operator controls | `scripts/fleet_dashboard.py`, `scripts/resilience_dashboard.py` | Dashboard tests plus manual browser/ROS integration |
| Benchmark repeatability | `scripts/run_benchmark.py`, `config/experiments/`, `config/workloads/` | Run-specific JSON/CSV/log outputs; regenerate locally |

## Scope boundaries

- This is a ROS 2/Gazebo simulation implementation. Physical robot drivers, onboard computer deployment, and hardware safety certification are not established by this repository alone.
- ROS 2 topic transport is peer-distributed in the fleet design, but the supplied launch may also start infrastructure such as a Zenoh router. Verify the exact topology and behavior under router loss before describing a deployment as brokerless or free of infrastructure single points of failure.
- The simulation fault model is not equivalent to real wireless conditions. Use measured hardware/network trials for deployment claims.
- The three supplied archives describe different systems. The separate Rust and React/Python simulators are not shipped as competing runtime stacks in this repository.


