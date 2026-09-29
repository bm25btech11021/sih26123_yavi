# YAVI SIH26123 Operator Task Allocation & Live Task Control Guide

> **Audience**: Warehouse operators, fleet supervisors, systems engineers, and research evaluators.  
> **System**: YAVI SIH26123 Decentralized Multi-AMR Coordination System (ROS 2 Jazzy, Gazebo Harmonic, Python 3.12).

---

## 1. Architectural Overview

The **YAVI SIH26123 Operator Task Control Layer** provides human operators with real-time supervisory control over warehouse task dispatch, dynamic re-prioritization, cancellation, and requeuing—while strictly preserving **decentralized CBBA (Consensus-Based Bundle Algorithm)** as the sole task assignment and consensus mechanism.

```
       +-------------------------------------------------------------+
       |                      OPERATOR INTERFACES                     |
       |   +--------------------------+   +-----------------------+   |
       |   |  Web Dashboard (Port 8080) |   |  CLI: create_task.py  |   |
       |   +--------------------------+   +-----------------------+   |
       +------------------------------+------------------------------+
                                      |
                      ROS 2 Services: /tasks/create, /tasks/control
                                      v
       +-------------------------------------------------------------+
       |                      TASK MANAGER NODE                      |
       |  - Validates coordinates [0.0, 30.0], priority [1..4]       |
       |  - Enforces duplicate checking & auto sequential ID 'T###'  |
       |  - Instantiates Task in PENDING state (AUTO vs DIRECT)      |
       |  - Exposes /tasks/all and /tasks/available                  |
       +------------------------------+------------------------------+
                                      |
                    Topics: /tasks/available, /tasks/all
                                      v
       +-------------------------------------------------------------+
       |             DECENTRALIZED CBBA CONSENSUS LAYER              |
       |  - Each CBBANode (amr_0 .. amr_N) evaluates marginal scores |
       |  - For DIRECT tasks: non-target robots skip auction         |
       |  - For AUTO tasks: all robots bid via gossip /fleet/cbba_bids|
       |  - Winning robot claims task upon bundle convergence        |
       +------------------------------+------------------------------+
                                      |
                         Topic: /{robot_id}/bundle
                                      v
       +-------------------------------------------------------------+
       |            ROLLING HORIZON & MULTI-AGENT SAFETY             |
       |  - RHCR (Rolling-Horizon Collision-Free Route Planning)     |
       |  - Space-Time Reservations & PIBT Coordination              |
       |  - Wait-For-Graph (WFG) Deadlock Detection & Resolution     |
       |  - Local Hardware Safety Interlocks & Emergency Braking     |
       +-------------------------------------------------------------+
```

### Key Architectural Invariants
1. **Zero Centralized Bypass**: The Task Manager *never* assigns tasks directly to robots for AUTO tasks. It stamps tasks into the `PENDING` pool, leaving the decentralized auction to determine optimal bids based on robot location, queue length, and deadlines.
2. **Deterministic DIRECT Allocation**: When an operator specifies a target robot (e.g. `amr_3`), the Task Manager tags `requested_robot='amr_3'`. Non-target robots skip candidate scoring, allowing only the designated AMR to place bids.
3. **Safety Primacy**: The authority hierarchy is strictly:
   $$\text{Local Safety / Emergency Braking} \succ \text{Reservations / PIBT} \succ \text{RHCR Planning} \succ \text{Task Allocation}$$
   Task control actions never override collision avoidance, headway safety zones, or physical braking.

---

## 2. Operator CLI Tool: `scripts/create_task.py`

The CLI tool enables instantaneous command-line task injection into the running fleet.

### Command Syntax
```bash
python3 scripts/create_task.py \
  --pickup-x <FLOAT> \
  --pickup-y <FLOAT> \
  --dropoff-x <FLOAT> \
  --dropoff-y <FLOAT> \
  [--priority {LOW,NORMAL,HIGH,CRITICAL,1..4}] \
  [--task-id <STRING>] \
  [--robot <ROBOT_ID>] \
  [--deadline <FLOAT>] \
  [--timeout <FLOAT>]
```

### Argument Reference
| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--pickup-x`, `--pickup-y` | Float | *Required* | Warehouse coordinates for pickup bay $[0.0, 30.0]\,\text{m}$. |
| `--dropoff-x`, `--dropoff-y` | Float | *Required* | Warehouse coordinates for delivery hub $[0.0, 30.0]\,\text{m}$. |
| `--priority` | String/Int | `NORMAL` | Task priority: `LOW` (1), `NORMAL` (2), `HIGH` (3), `CRITICAL` (4). |
| `--task-id` | String | `""` | Custom task identifier. If omitted, sequential `T###` is generated. |
| `--robot` / `--requested-robot` | String | `""` | Constrain to specific AMR (e.g. `amr_2`). Omit for AUTO CBBA auction. |
| `--deadline` | Float | `0.0` | Deadline in simulation seconds (latencies penalized in CBBA). |
| `--timeout` | Float | `5.0` | Service response timeout in seconds. |

### CLI Usage Examples

#### Example 1: Dispatch an Autonomous CBBA Task (AUTO)
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/create_task.py \
  --pickup-x 2.5 \
  --pickup-y 4.0 \
  --dropoff-x 14.0 \
  --dropoff-y 12.0 \
  --priority HIGH
```
*Output*:
```
========================================
  TASK CREATION ACCEPTED
========================================
  Task ID:          T001
  Pickup:           (2.50, 4.00)
  Dropoff:          (14.00, 12.00)
  Priority:         HIGH
  Allocation:       AUTO (CBBA Consensus)
  Message:          Task 'T001' created successfully in PENDING state (allocation: AUTO (CBBA))
========================================
```

#### Example 2: Dispatch a Constrained Task to a Specific Robot (DIRECT)
```bash
python3 scripts/create_task.py \
  --pickup-x 8.0 \
  --pickup-y 3.0 \
  --dropoff-x 16.0 \
  --dropoff-y 6.0 \
  --priority NORMAL \
  --task-id EXPEDITED_01 \
  --robot amr_3
```

---

## 3. ROS 2 Service Interfaces

### 3.1 `/tasks/create` (`amr_fleet_msgs/srv/CreateTask`)
Exposed by `TaskManagerNode` to validate and register new tasks.

- **Request**:
  ```
  string task_id          # Custom ID, or empty for auto-generated 'T###'
  float64 pickup_x        # Warehouse X [0.0, 30.0]
  float64 pickup_y        # Warehouse Y [0.0, 30.0]
  float64 dropoff_x       # Warehouse X [0.0, 30.0]
  float64 dropoff_y       # Warehouse Y [0.0, 30.0]
  int32 priority          # 1=LOW, 2=NORMAL, 3=HIGH, 4=CRITICAL
  float64 deadline        # Optional simulation timestamp
  string requested_robot  # Target AMR ID, or empty for AUTO CBBA
  ```
- **Response**:
  ```
  bool accepted           # True if accepted into PENDING pool
  string task_id          # Assigned or generated task ID
  string message          # Rejection rationale or confirmation
  ```

### 3.2 `/tasks/control` (`amr_fleet_msgs/srv/ControlTask`)
Unified operator lifecycle control service.

- **Request**:
  ```
  string task_id          # Target task identifier
  string action           # 'CANCEL' or 'REQUEUE'
  ```
- **Response**:
  ```
  bool success            # True if lifecycle transition succeeded
  string message          # Detailed status message
  ```

### 3.3 Convenience Aliases
- `/tasks/cancel`: Routes directly to `ControlTask(action='CANCEL')`.
- `/tasks/requeue`: Routes directly to `ControlTask(action='REQUEUE')`.

---

## 4. Web Dashboard Interface (Port 8080)

The YAVI SIH26123 Web Console (`scripts/fleet_dashboard.py`) provides graphical visibility and tactile controls.

### 4.1 Launching the Console
```bash
python3 scripts/fleet_dashboard.py --port 8080
```
Open your browser to `http://localhost:8080`.

### 4.2 Interactive Features
1. **`+ CREATE TASK` Modal**:
   - Prominently located in the top navigation bar.
   - Allows typing coordinates, selecting priority, choosing between **AUTO (Auction)** and **DIRECT (Target AMR)**.
   - Provides immediate visual validation feedback (green for success, red for out-of-bounds).
2. **Live Task Registry Table (Card 04)**:
   - Displays all tasks with coordinates, priority badges, and lifecycle state.
   - **Action Buttons**:
     - `CANCEL`: Instantly cancels in-flight or assigned tasks; purges robot bundles.
     - `REQUEUE`: Re-enqueues failed or unassigned tasks back into the CBBA auction pool.
3. **CBBA Bid Inspector & Margins (Card 04B)**:
   - Clicking any task row in the table activates the inspector card.
   - Displays real-time bids from every robot in the fleet, the winning robot, and winning bid value.
   - Indicates if a task has a `[DIRECT CONSTRAINT]` applied.

---

## 5. Failure Recovery & Edge Case Handling

| Scenario | System Behavior | Operator Action |
| :--- | :--- | :--- |
| **Out of Warehouse Bounds** | Rejection with clear bounds error ($[0.0, 30.0]\,\text{m}$). | Re-enter coordinates within warehouse perimeter. |
| **Duplicate Task ID** | Request rejected; existing task preserved. | Provide unique identifier or leave blank for auto `T###`. |
| **Task Cancelled In-Flight** | Active robot finishes current waypoint cleanly, aborts remaining subgoals, purges bundle, and returns to `IDLE`. | Safe; robot automatically seeks new tasks or holds position. |
| **Temporary Aisle Blockage** | Robots automatically detour around obstacle using A* rerouting; if blocked indefinitely, task transitions to `FAILED`. | Use `REQUEUE` button on dashboard to re-auction task once path clears. |
| **Communication Drop** | CBBA agents hold local beliefs; reservations expire safely; upon reconnect, sync occurs without duplicate assignment. | System recovers autonomously; no operator action required. |


