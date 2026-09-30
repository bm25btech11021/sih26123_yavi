# YAVI SIH — SIH26123

ROS 2 and Gazebo prototype for decentralized warehouse AMR fleet coordination. This repository focuses on robot-local task bidding and planning, distributed traffic coordination, failure experiments, and operator observability.

## What is in this repository

- **ROS 2 Jazzy workspace:** five packages for fleet coordination, simulation, robot descriptions, launch/configuration, and custom messages.
- **Warehouse simulation:** Gazebo Harmonic worlds, configurable fleets, workloads, and namespaced robot launches.
- **Coordination:** CBBA task allocation, rolling-horizon space-time planning, reservation handling, PIBT conflict resolution, wait-for-graph deadlock recovery, stale-state/fault handling, and adaptive compute modes.
- **Safety and operations:** a local LiDAR braking backstop, fleet telemetry dashboard, resilience experiment console, and task creation CLI.
- **Evaluation:** package tests, benchmark and scenario-validation scripts, and experiment configuration examples.

This is a simulation prototype. Its safety mechanisms and reported benchmark outcomes apply only to the specific simulated configurations that have been run. They are not a general collision freedom proof or a certification for operating physical robots.

## Architecture

Each AMR namespace runs its own coordination nodes and maintains local state. ROS 2 topics carry bids, bundles, reservations, health and coordination updates. The task manager supplies task lifecycle services; the dashboards observe the fleet and provide operator or experiment controls. Local sensor braking has higher authority than planning and task allocation.

```text
       Task generation 
              │
              ▼
  per robot CBBA task allocation
              │
              ▼
 rolling-horizon planning + reservations
              │
              ▼
 PIBT / deadlock recovery ── local LiDAR stop
              │
              ▼
      Nav2 control → Gazebo AMRs
              │
   ROS 2 telemetry → dashboards
```

The dashboards and task manager are supervisory tools. Core robot coordination is distributed among robot nodes; dashboards are not required for motion decisions. See [the architecture](docs/ARCHITECTURE.md) and [algorithm specification](docs/ALGORITHMIC_SPECIFICATION.md).

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim 8
- `colcon`, `rosdep`, Python 3.12 and `pytest`
- For the graphical demo: RViz 2 and a working graphics stack

## Build

```bash
cd YAVI-SIH26123
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the fleet demo

The default integrated demo starts Gazebo, the adaptive fleet launch, and the two operator dashboards. It defaults to the 10-robot M9 warehouse workload.

```bash
cd YAVI-SIH26123
source /opt/ros/jazzy/setup.bash
./scripts/launch_all_in_one.sh --world warehouse_small
```

Open the fleet console at `http://localhost:8080` and the resilience experiment console at `http://localhost:8081`. For a headless simulation, add `--headless`. To choose a workload, pass `--workload config/workloads/workload_15_tasks.yaml`.

You can also run the fleet launch directly:

```bash
export ROS_DOMAIN_ID=42
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=5 world:=warehouse_small headless:=false \
  compute_mode:=ADAPTIVE comm_profile:=NORMAL \
  workload_file:=config/workloads/workload_15_tasks.yaml
```

Create a task from another sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/create_task.py --help
```


<img width="1480" height="831" alt="yavi2" src="https://github.com/user-attachments/assets/edf674fd-be57-45ff-bc3f-c9860f885bb9" />

## Tests and experiments

Run the workspace test suite after building and sourcing the overlay:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/run_tests.sh
```

Validate the compound-fault scenarios:

```bash
python3 scripts/validate_m4_compound_scenarios.py
```

Run the benchmark entry point with its help first, then select a configuration appropriate to the machine:

```bash
python3 scripts/run_benchmark.py --help
```

Every performance claim should be tied to saved run configuration, seed, logs, and metrics. The included results from the source archive were not copied into this repository; rerun experiments to produce project-owned evidence. See [runbook](docs/RUNBOOK.md), [demo guide](docs/DEMO_GUIDE.md), and [completion checklist](docs/DEFINITION_OF_DONE.md).

## Requirement traceability

| SIH26123 capability | Implementation |
|---|---|
| Multi-AMR warehouse simulation | `src/amr_fleet_bringup`, `src/amr_fleet_sim`, `src/amr_fleet_description`, `config/maps` |
| Decentralized task allocation | `src/amr_fleet_core/amr_fleet_core/cbba_*.py` |
| Rolling-horizon multi-agent coordination | `src/amr_fleet_core/amr_fleet_core/rh_*.py`, `pibt_planner.py`, `reservation_table.py` |
| Deadlock handling | `wfg_deadlock.py`, `deadlock_recovery.py` |
| Robot/network fault resilience | `fault_*.py`, `stale_state_manager.py`, `communication_model.py`, `scripts/resilience_dashboard.py` |
| Local obstacle safety | `local_obstacle_detector.py` and the robot bringup/control configuration |
| Adaptive compute | `adaptive_compute_policy.py`, `compute_modes.py` |
| Operator visibility and task control | `scripts/fleet_dashboard.py`, `task_manager_node.py`, `scripts/create_task.py` |
| Reproducible evaluation | `scripts/run_benchmark.py`, `scripts/validate_m4_compound_scenarios.py`, `config/experiments`, package tests |



## Source selection and attribution

This repository was assembled as a new YAVI project. The ROS 2 implementation was selected as the deployable system foundation. The Rust repository informed algorithm and benchmark review; the Python/React repository informed dashboard and demonstration review. Their independent runtimes were not combined because that would leave multiple competing fleet simulators rather than one integrated application. 

The supplied ROS 2 package manifests declare Apache-2.0.



