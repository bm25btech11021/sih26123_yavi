# YAVI SIH26123 // Fault Injection & Resilience Testing Dashboard

> This guide describes the simulator's experiment controller. Fault injection demonstrates selected modeled disturbances; it does not establish resilience to every real network, sensor, or actuator failure.

**Component**: Dedicated GUI Testbed & Observability Console  
**Port**: `8081` (Default)  
**Host**: `0.0.0.0` / `localhost`  
**Location**: `scripts/resilience_dashboard.py`  
**Test Suite**: `src/amr_fleet_core/test/test_resilience_dashboard.py`

---

## 1. Architectural Principles & System Decoupling

The Fault Injection & Resilience Testing Dashboard on **Port 8081** is a dedicated human-in-the-loop testbed created specifically for adversarial experimentation, fault injection, and recovery observability.

```
+-------------------------------------------------------------------------+
|                  HUMAN OPERATOR / BROWSER CLIENT                       |
|                       http://localhost:8081                             |
+------------------------------------+------------------------------------+
                                     | REST JSON / HTTP GET / POST
                                     v
+-------------------------------------------------------------------------+
|              RESILIENCE MONITOR & INJECTOR (Port 8081)                  |
|             (scripts/resilience_dashboard.py — Zero SPOF)               |
+------------------+----------------------------------+-------------------+
                   |                                  |
    ROS 2 Service  | /{robot_id}/inject_fault         | Telemetry Topics
    Calls (Clients)|                                  | (Subscribers)
                   v                                  v
+-------------------------------------------------------------------------+
|                 DECENTRALIZED AUTONOMOUS FLEET NODES                    |
|   amr_0                     amr_1                     amr_2             |
|   - FaultDetector           - FaultDetector           - FaultDetector   |
|   - CBBAAgent               - CBBAAgent               - CBBAAgent       |
|   - RHCR Planner            - RHCR Planner            - RHCR Planner    |
+-------------------------------------------------------------------------+
```

### Absolute Architectural Rule
> **Zero Recovery Logic in the Dashboard**:  
> The dashboard is strictly an **observability and injection console**. All failure detection, belief purging, task reclamation, obstacle insertion, and CBBA re-auctions occur purely inside the decentralized nodes (`fault_detector.py`, `rh_node.py`, `cbba_node.py`, `task_manager_node.py`). If the dashboard process is killed or closed, the fleet's autonomous recovery continues uninterrupted without degradation.

---

## 2. Key Dashboard Capabilities & Panels

### 2.1 Fleet Health & State Panel
- Displays live cards for all active AMRs (`amr_0`, `amr_1`, `amr_2`, ...).
- Explicit state indicators with distinctive color codes:
  - `HEALTHY`: Green badge (`#10b981`)
  - `COMM_LOSS`: Amber badge (`#f59e0b`)
  - `FAILED` / `KILL`: Red badge (`#ef4444`)
  - `ACTUATOR_FAIL`: Orange-Red badge
  - `NAVIGATION_STUCK`: Warning Yellow badge
  - `BATTERY_CRITICAL`: Deep Amber badge
  - `EMERGENCY_STOP`: Purple badge (`#8b5cf6`)
- Real-time telemetry: localized $(x, y)$ coordinates, yaw heading, linear speed, heartbeat counter, and age since last heartbeat ($t_{\text{now}} - t_{\text{last}}$).
- Active task ID and currently allocated CBBA bundle items.
- Quick action buttons per card: **Kill**, **Comm Cut**, **Restore**.

### 2.2 Active Fault Injection Console
- **Target AMR Selector**: Target any specific robot or broadcast to `ALL_ROBOTS`.
- **Fault Type Selector**:
  - `KILL`: Simulates node crash / fatal exception / SIGKILL.
  - `HEARTBEAT_TIMEOUT`: Freezes heartbeat publishing while keeping nodes alive.
  - `COMM_LOSS`: Simulates temporary network isolation / WiFi dropout.
  - `ACTUATOR_FAIL`: Simulates motor driver or gearbox failure.
  - `NAVIGATION_STUCK`: Simulates physical floor entanglement / wheel slip stall.
  - `BATTERY_CRITICAL`: Simulates sudden battery voltage drop.
  - `RESTORE`: Revives robot, resets local state, and notifies fleet of return.
  - `EMERGENCY_STOP`: Imposes immediate zero-velocity halt.
- **Duration Configuration**: Set transient duration in seconds (0.0 for permanent).
- **Fleet Emergency Stop**: Giant red **FLEET EMERGENCY STOP** button halts all robots instantly; **RESUME ALL** restores normal operations.

### 2.3 Interactive 2D Warehouse World & Chassis Obstacle Tracker
- Scalable vector rendering of warehouse layout, walls, storage racks, and pickup/dropoff stations.
- Real-time AMR markers showing heading direction, speed, and health halos.
- **Stranded Chassis Visualization**: Disabled robots are rendered with a prominent **red cross-hatched warning box** and circular keep-out radius ($0.8\text{m}$).
- Dynamic path overlays showing active AMRs rerouting around stranded chassis.

### 2.4 Decentralized 7-Stage Recovery Pipeline Tracker
Tracks the 7 canonical stages of YAVI SIH26123 resilience autonomous recovery in real time:
1. **Stage 1 (Fault Injected)**: Fault command received and dispatched ($t_0$).
2. **Stage 2 (Peer Detected)**: Surviving peer confirms heartbeat silence ($t_{\text{detect}}$).
3. **Stage 3 (Beliefs Purged)**: Failed peer's bids wiped; spacetime reservations cleared ($t_{\text{purge}}$).
4. **Stage 4 (Task Reclaimed)**: CAS precondition check executes; task $\to$ `PENDING` ($t_{\text{reclaim}}$).
5. **Stage 5 (Obstacle Inserted)**: Stranded chassis registered in GridWorld obstacle layer ($t_{\text{obs}}$).
6. **Stage 6 (Task Reassigned)**: CBBA decentralized re-auction won by surviving peer ($t_{\text{assign}}$).
7. **Stage 7 (Execution Resumed)**: Winning peer resumes physical transit to task ($t_{\text{resume}}$).

Each stage displays measured delta $\Delta t$ from fault onset.

### 2.5 Safety & Research Invariants Panel
- **Zero Task Duplication**: Continuous invariant check verifying no duplicate task ownership across evaluated cases.
- **Failed Chassis Avoidance**: Live calculation of minimum clearance distance from moving AMRs to any stranded chassis ($d_{\text{min}} \ge 0.45\text{m}$).
- **Gazebo Safety Proxy**: Confirms zero collisions via 2D Oriented Bounding Box (OBB) geometric testing.
- **Provenance Disclosure**: Explicitly labels contact detection as an odometry-based geometric proxy rather than raw bumper physics.

### 2.6 M1 Validation Scenarios Quick-Triggers
One-click execution of pre-configured research scenarios:
- `M1-A`: Single Robot Hard Kill during Task Execution
- `M1-B`: Heartbeat Timeout during Narrow Corridor Transit
- `M1-C`: Simultaneous Dual Detection & Re-auction Race
- `M1-D`: Actuator Failure with Stranded Chassis Avoidance
- `M1-E`: Operator Restoration & Re-integration into Fleet
- `M1-F`: Comm Loss vs Node Kill Distinction

### 2.7 Rolling Event Log & Data Export
- Live stream of all fault events, state transitions, and CAS operations.
- One-click export buttons:
  - **Export JSON**: Complete structured JSON telemetry log for external scripts.
  - **Export Markdown**: Formatted research report ready for inclusion in publications.

### 2.8 Network Impairment Console (Milestone 2 Extension)
- Comprehensive communication degradation modeling for adversarial network evaluation.
- Configurable preset profiles: `NORMAL`, `LOW_LATENCY`, `HIGH_LATENCY`, `JITTER`, `LOSS_LOW`, `LOSS_HIGH`, `BURST_LOSS`, `OUTAGE`, `OUTAGE_RECOVERY`, `PARTITION`.
- Parametric sliders:
  - **Latency**: 0 to 1000 ms
  - **Jitter**: 0 to 200 ms
  - **Packet Loss**: 0 to 100%
  - **Burst Loss Probability**: 0 to 100%
  - **Network Partition**: Select isolated robots (e.g. `amr_2` or `amr_3, amr_4`)
- Target AMR selector: target individual robot or fleet broadcast.

### 2.9 Dual-Mode Recovery Pipeline Stepper (M1 vs M2)
The recovery stepper dynamically adapts based on the active test mode:
- **M1 Mode (7 Stages)**: Robot Failure & Mid-Task Recovery (`FAULT_INJECTED` $\to$ `PEER_DETECTED` $\to$ `BELIEFS_PURGED` $\to$ `TASK_RECLAIMED` $\to$ `OBSTACLE_INSERTED` $\to$ `TASK_REASSIGNED` $\to$ `EXECUTION_RESUMED`).
- **M2 Mode (9 Stages)**: Network Failure, Local Autonomy & Reconnection:
  1. `NOMINAL`: Full fleet connectivity and normal coordination.
  2. `COMM_LOSS_DETECTED`: Heartbeat silence exceeds $\tau_{\text{loss}} = 1.0\text{s}$.
  3. `LOCAL_AUTONOMY_ACTIVE`: AMR continues on reserved path ($v \le 0.4\text{ m/s}$) with LiDAR active.
  4. `RESERVATION_EXPIRY_HOLD`: Space-time reservation expires; AMR enters `LOCAL_SAFETY_HOLD` ($v = 0.0\text{ m/s}$).
  5. `PEER_FAILURE_DEBOUNCE`: Outage exceeds $\tau_{\text{fail}} = 3.5\text{s}$; peer fleet confirms `FAILED`.
  6. `AUTONOMOUS_RECLAIM`: Peer fleet purges beliefs and executes CAS task re-auction.
  7. `NETWORK_RESTORED`: Communication link reconnects.
  8. `RECONCILIATION_YIELD`: Reconnected AMR reconciles timestamps and yields task without duplication.
  9. `FLEET_RECONVERGED`: Fleet state converged; zero duplicate ownership ($I_{\text{uniq}}$).

### 2.10 Live Network Telemetry & Degradation Invariants
- Live network telemetry table tracking:
  - Per-robot network status: `ONLINE`, `DEGRADED`, `COMM_LOSS`, `OFFLINE`.
  - Configured loss vs. Empirical measured loss rate ($\text{dropped}/\text{sent}$).
  - Messages sent, delivered, and dropped.
  - P50 latency, P95 latency, and jitter.
- M2 Research Invariants live verification:
  - `false_failure_rejection`: Transient outages ($\le 3.5\text{s}$) strictly reject failure declarations.
  - `reconnection_zero_duplication`: Invariant $I_{\text{uniq}}$ preserved upon reconnection.
  - `reservation_hold_on_expiry`: AMR stops immediately when space-time reservations expire.

### 2.11 M1 & M2 Validation Quick-Triggers
- **M1 Scenarios**: `M1-A` through `M1-F` (Robot Fail-Stop, Corridor Transit, Re-auction Race, Chassis Avoidance, Operator Restoration, Comm Loss vs Kill).
- **M2 Scenarios**: `M2-A` through `M2-G` and `M2-B2` (Short Outage, Outage in Assigned, Active Nav COMM_LOSS, Outage in In-Progress, Long Outage Reclaim, Reconnection Reconciliation, 35% Packet Loss, Network Partition; evaluated at `INTEGRATION / SIMULATION` execution level).

### 2.12 Environmental & Adversarial Resilience Console (Milestone 3 Extension)
- **Dynamic Aisle Blockage Controller**:
  - Live injection and clearance of single or multi-cell corridor obstructions.
  - Configurable duration (transient or persistent).
  - Visualized on 2D warehouse map as high-visibility hatched hazard zones.
- **Adversarial Trajectory & Reservation Conflict Injector**:
  - Direct execution of non-corrupting synthetic conflict conditions: `SAME_CELL`, `OPPOSING_CORRIDOR`, `CROSSING_TRAJECTORIES`, `RESERVATION_CONFLICT`, `HIGH_CONTENTION_INTERSECTION`.
  - Real-time logging of conflict detection and PIBT fallback invocations.
- **Tri-Mode Recovery Pipeline Stepper (M1 vs M2 vs M3)**:
  - Supports stepping through the 9 discrete stages of environmental resilience:
    1. `STAGE_1_OBSTACLE_INJECTED`: Blockage or conflict introduced into simulation.
    2. `STAGE_2_SENSOR_OBSERVED`: Onboard LiDAR detects obstacle ($d \le 1.5\text{m}$).
    3. `STAGE_3_LOCAL_SAFETY_EVALUATED`: Safety hold evaluated; reactive braking ($0.28\text{m}$) armed.
    4. `STAGE_4_CANDIDATE_CLEARANCE_CHECKED`: Footprint ($0.65\text{m} \times 0.45\text{m}$) and reservations verified.
    5. `STAGE_5_GRAPH_WITHDRAWN`: Cells removed from local and shared traversability graph.
    6. `STAGE_6_RESERVATIONS_INVALIDATED`: Active reservations for obstructed cells revoked.
    7. `STAGE_7_REPLAN_COMPLETED`: Single-agent A* computes valid collision-free detour.
    8. `STAGE_8_PIBT_FALLBACK_RESOLVED`: Multi-agent vertex/edge conflicts resolved via PIBT push-and-yield.
    9. `STAGE_9_EXECUTION_RESUMED`: Motion execution resumes along detour; 0 geometric overlaps.
- **M3 Validation Quick-Triggers**:
  - Interactive execution of all 9 M3 scenarios: `M3-A`, `M3-B`, `M3-B2`, `M3-C1`, `M3-C2`, `M3-C3`, `M3-C4`, `M3-C5`, `M3-H`.

### 2.13 Milestone 4 Compound Fault & Adversarial Testing Console
Milestone 4 evolves the testbed into an **Adversarial Experiment Controller** capable of composing multi-domain disturbances and monitoring the fleet's decentralized three-tier response:
- **Compound Fault Scenario Composer (Column 1)**:
  - **Robot Disturbance**: Target single or multiple AMRs (`amr_1`, `amr_0`, `amr_2`, `amr_1 & amr_2`), fault type (`KILL`, `COMM_LOSS`, `ACTUATOR_FAIL`, `HEARTBEAT_TIMEOUT`).
  - **Network Stressor**: Impairment profile (`LOSS_HIGH`, `OUTAGE`, `PARTITION`, `NORMAL`), packet loss rate ($0.0$ to $1.0$), and duration.
  - **Environmental Disturbance**: Corridor obstacle preset (`(7,4) & (7,5)`, `(7,7)`, `(5,5)`, or `NONE`).
  - **One-Click Controls**: `⚡ LAUNCH COMPOUND EXPERIMENT` (dispatches to `/api/m4/compose_and_run`) and `🔄 RESET` (restores all robots, clears all blockages, reconnects network).
- **Three-Tier Fleet Response Telemetry Panel (Column 2)**:
  - **Tier 1 (CBBA Reallocation)**: Monitors atomic CAS task reclamation (`ASSIGNED` $\to$ `PENDING`), consensus rounds, and winning peer assignment without duplicate ownership.
  - **Tier 2 (Dynamic Replanning)**: Monitors space-time reservation invalidation, alternate corridor graph withdrawal, detour length, and SingleAgentAStar replanning latency ($<0.3\,\text{ms}$).
  - **Tier 3 (Local Safety & Reactive Braking)**: Monitors the $0.28\,\text{m}$ LiDAR safety envelope, clearance distances, and the $0.8\,\text{m}$ stranded chassis keep-out exclusion boundary.
- **Formal Invariant Verification Badges (Column 3)**:
  - **$I_1$ (Task Uniqueness)**: $\forall t \in \mathcal{T},\; \sum_{i=1}^N \mathbb{I}[t \in B_i] \le 1$ (mutual exclusion of task bundles).
  - **$I_2$ (Spacetime Exclusivity)**: $\forall (x,y,t),\; |\{i \mid R_i(t)=(x,y)\}| \le 1$ (zero reservation overlaps).
  - **$I_3$ (Local LiDAR Clearance)**: $\min_{i \ne j} \|p_i - p_j\|_2 \ge 0.28\,\text{m}$ (0 physical contacts).
  - **Tiered Fault Discrimination**: $T_{\text{transient}} \le 1.5\,\text{s} \ll T_{\text{fail}} = 3.5\,\text{s}$ (non-blocking discrimination; 0 false failures).
  - **Monotonic CAS Reconnection**: Partition reconciliation yields monotonically to newer assignment timestamps.
- **M4 Benchmark Scenarios (7-Stage Recovery Stepper)**:
  - `M4-A`: Robot Failure + Dynamic Blockage (Dual Obstacle Navigation)
  - `M4-B`: Comm Loss vs Failure Discrimination (1.5s vs 3.5s; 0 false failures)
  - `M4-C`: Multiple Overlapping Robot Failures (Staggered crashes; atomic CAS task reclamation)
  - `M4-D`: Robot Failure + Sensor-Visible Obstacle (Decoupled 0.9m perception vs 0.22m reactive brake)
  - `M4-E`: Network Partition + Robot Failure (Monotonic Lamport timestamp reconciliation)
  - `M4-F`: Network Loss + Dynamic Blockage (Safe local hold upon dynamic obstruction)
  - `M4-G`: Master Compound Quad Failure (2 crashes + 50% packet loss + corridor blockage)

---

## 3. REST API Reference

| Method | Endpoint | Description | Payload Example |
| :--- | :--- | :--- | :--- |
| `GET` | `/` | Web console single-page interface | None |
| `GET` | `/api/state` | Full fleet health, telemetry, invariants, and logs | None |
| `GET` | `/api/m4/status` | Active scenario status, compound faults, and 5 formal invariants | None |
| `POST` | `/api/fault/inject` | Inject simulated robot fault | `{"robot_id": "amr_1", "fault_type": "KILL", "duration_sec": 0.0}` |
| `POST` | `/api/fault/restore` | Restore robot to healthy state (`ALL_ROBOTS` supported) | `{"robot_id": "amr_1"}` |
| `POST` | `/api/network/impairment` | Apply network impairment profile | `{"robot_id": "amr_1", "profile": "LOSS_HIGH", "loss_rate": 0.35, "delay_ms": 50}` |
| `POST` | `/api/network/reconnect` | Reconnect robot or restore normal network (`ALL_ROBOTS` supported) | `{"robot_id": "amr_1"}` |
| `POST` | `/api/environment/blockage` | Inject or clear dynamic aisle blockage (`blockage_id: "ALL"` clears all) | `{"action": "INJECT", "blockage_id": "BLK_01", "cells": [[7, 4], [7, 5]], "duration_sec": 0.0}` |
| `POST` | `/api/environment/conflict` | Inject adversarial contention condition | `{"conflict_type": "CROSSING_TRAJECTORIES", "robot_ids": ["amr_0", "amr_1"], "cell": [4, 5], "time_step": 3}` |
| `POST` | `/api/environment/step` | Advance M3 recovery pipeline stepper | `{"stage_name": "01_OBSTACLE_DETECTED"}` |
| `POST` | `/api/m4/inject_compound` | Inject multi-domain compound fault | `{"scenario_id": "M4-A", "robot_ids": ["amr_1"], "blockage_cells": [[7, 4], [7, 5]], "packet_loss_rate": 0.35}` |
| `POST` | `/api/m4/compose_and_run` | Compose custom compound experiment and trigger execution | `{"scenario_id": "CUSTOM_COMPOUND", "robot_ids": ["amr_1"], "fault_type": "KILL", "network_profile": "LOSS_HIGH", "packet_loss_rate": 0.35, "duration_sec": 5.0, "blockage_cells": [[7, 4], [7, 5]]}` |
| `POST` | `/api/fleet/estop` | Emergency stop all AMRs | None |
| `POST` | `/api/fleet/resume` | Resume all AMRs to normal operation | None |
| `POST` | `/api/scenario/trigger` | Trigger automated validation scenario (M1-A..F, M2-A..G, M3-A..H, M4-A..G) | `{"scenario_id": "M4-G"}` |
| `GET` | `/api/export` | Download JSON experiment evidence report | None |
| `GET` | `/api/export/markdown` | Download Markdown experiment evidence report | None |

---

## 4. How to Launch and Use

### Launching the Dashboard
```bash
# Sourcing environment & setting isolated ROS Domain
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Option 1: Launch in autonomous simulation testbed mode (instant run, zero external dependencies)
python3 scripts/resilience_dashboard.py --port 8081 --sim-mode

# Option 2: Launch live dashboard connected to Gazebo Harmonic fleet
python3 scripts/resilience_dashboard.py --port 8081 --world warehouse_m9_v2
```

### Accessing the Web UI
Open any modern web browser to:
```
http://localhost:8081
```
*(No external npm, node_modules, or build steps required; zero dependencies).*

### Automated Verification & Validation
```bash
# Run all 79 unit and integration tests across the 5 resilience suites:
python3 -m pytest \
  src/amr_fleet_core/test/test_fault_resilience.py \
  src/amr_fleet_core/test/test_m2_network_resilience.py \
  src/amr_fleet_core/test/test_m3_adversarial_resilience.py \
  src/amr_fleet_core/test/test_m4_compound_resilience.py \
  src/amr_fleet_core/test/test_resilience_dashboard.py -v

# Run the automated 7-benchmark M4 validation harness:
python3 scripts/validate_m4_compound_scenarios.py
```


