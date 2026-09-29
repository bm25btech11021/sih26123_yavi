# YAVI SIH26123 // Fault Model & Research Invariants

> The thresholds and invariants here describe the supplied simulation model. Validate them against source, scenario outputs, and physical platform limits before presenting them as operational guarantees.

**Document Version**: 2.0  
**Target Architecture**: YAVI SIH26123 resilience (v2) Fault-Resilient Decentralized AMR Fleet Coordination  
**Date**: September 2026

---

## 1. System Scope & Fault Taxonomy

The YAVI SIH26123 resilience architecture models multi-AMR coordination under partial observability, asynchronous message delivery, and physical or computational node failures. This document defines the formal fault model, state transitions, debounce thresholds, and mathematical invariants governing fleet behavior.

### 1.1 Failure Classifications

| Failure Class | Formal Label | Manifestation | Observability | Recovery Mechanism |
| :--- | :--- | :--- | :--- | :--- |
| **Crash / Fail-Stop** | `FAILED` / `KILL` | Process termination, OS panic, power loss | Heartbeat silence $\ge 3.5\text{s}$ ($2\times$ debounce) | Autonomous: belief purge, task reclamation, CBBA re-auction, obstacle insertion |
| **Transient Comm Loss** | `COMM_LOSS` | RF shadow, antenna occlusion, WiFi dropout | Heartbeat silence $\in [1.5\text{s}, 3.5\text{s})$ | Grace period retention: tasks & reservations preserved |
| **Actuator Failure** | `ACTUATOR_FAIL` | Motor driver fault, encoder fault, stalled wheels | Telemetry self-report or zero velocity despite non-zero cmd | Autonomous: immediate stop, task reclamation, chassis obstacle |
| **Kinematic Stall** | `NAVIGATION_STUCK` | Floor slip, obstacle trapping, payload shift | Local localization stationary while path active | Autonomous: planner timeout, task reclamation |
| **Battery Critical** | `BATTERY_CRITICAL` | SoC $< 15\%$, voltage collapse | Health telemetry broadcast | Graceful preemption: return-to-charger, drop bundle |
| **Emergency Stop** | `EMERGENCY_STOP` | Hardware safety bumper or operator E-stop | Immediate broadcast or service call | Instant zero-velocity clamp; reservations retained |

---

## 2. Formal State Space & Transition System

Let the operational state of robot $R_i \in \mathcal{R}$ at time $t$ be $s_i(t) \in \mathcal{S}$, where:

$$\mathcal{S} = \{\text{HEALTHY}, \text{COMM\_LOSS}, \text{FAILED}, \text{ACTUATOR\_FAIL}, \text{NAVIGATION\_STUCK}, \text{BATTERY\_CRITICAL}, \text{EMERGENCY\_STOP}, \text{RECOVERING}\}$$

### 2.1 Timing Parameters & Thresholds

- **Heartbeat Period**: $\Delta t_{\text{hb}} = 0.5\text{s}$
- **Communication Loss Threshold**: $\tau_{\text{loss}} = 1.5\text{s}$ ($3 \times \Delta t_{\text{hb}}$)
- **Failure Detection Timeout**: $\tau_{\text{fail}} = 3.5\text{s}$ ($7 \times \Delta t_{\text{hb}}$)
- **Debounce Confirmation Samples**: $N_{\text{conf}} = 2$ consecutive evaluation cycles at $t \ge \tau_{\text{fail}}$
- **Safety Clearance Envelope**: $d_{\text{safe}} = 0.45\text{m}$ (physical robot bounding radius + safety buffer)

### 2.2 Mathematical State Transition Rules

Let $\Delta t_i = t - t_{\text{last\_hb}}(R_i)$ be the elapsed time since peer $R_j$ received a valid heartbeat from $R_i$:

1. **Health to Comm Loss**:
   $$\text{If } s_i(t) = \text{HEALTHY} \land \Delta t_i \ge \tau_{\text{loss}} \implies s_i(t^+) = \text{COMM\_LOSS}$$
2. **Comm Loss to Health (Reconnection)**:
   $$\text{If } s_i(t) = \text{COMM\_LOSS} \land \Delta t_i < \tau_{\text{loss}} \implies s_i(t^+) = \text{HEALTHY}$$
3. **Comm Loss to Failed (Timeout Confirmation)**:
   $$\text{If } s_i(t) = \text{COMM\_LOSS} \land \Delta t_i \ge \tau_{\text{fail}} \land \text{samples} \ge N_{\text{conf}} \implies s_i(t^+) = \text{FAILED}$$
4. **Immediate Fail-Stop**:
   $$\text{If explicitly reported } s_i = \text{FAILED} \implies \text{Confirmed } \text{FAILED immediately (bypass timeout)}$$
5. **Operator Restoration**:
   $$\text{If } \text{RESTORE command dispatched} \land s_i \in \{\text{FAILED}, \text{ACTUATOR\_FAIL}, \text{NAVIGATION\_STUCK}\} \implies s_i(t^+) = \text{HEALTHY}$$

---

## 3. Formal Research Invariants

The YAVI SIH26123 resilience architecture defines and enforces four fundamental research invariants across all operations:

### Invariant 1: Task Conservation & Uniqueness ($I_{\text{uniq}}$)
At any point in time $t$, no active task $T_k \in \mathcal{T}$ shall be simultaneously assigned to or executed by more than one healthy robot:

$$\forall T_k \in \mathcal{T}_{\text{active}}, \quad \left| \{ R_i \in \mathcal{R}_{\text{healthy}} \mid T_k \in \mathcal{B}_i(t) \} \right| \le 1$$

*Proof Mechanism*: Ensured by the **Compare-And-Swap (CAS)** precondition on task reclamation:
$$\text{Reclaim}(T_k, R_{\text{failed}}) \iff \left( \text{state}(T_k) \in \{\text{ASSIGNED}, \text{IN\_PROGRESS}\} \land \text{owner}(T_k) = R_{\text{failed}} \right)$$
If multiple peers attempt reclamation simultaneously, exactly one transaction succeeds; all subsequent attempts observe $\text{state} = \text{PENDING}$ and evaluate to safe no-ops.

### Invariant 2: Space-Time Reservation Exclusivity ($I_{\text{reserv}}$)
For any grid cell $c = (x, y)$ and discrete planning timestep $\tau$, at most one robot may hold a space-time reservation:

$$\forall c \in \mathcal{C}, \forall \tau \in \mathbb{N}, \quad \left| \{ R_i \in \mathcal{R} \mid (c, \tau) \in \mathcal{S}_i \} \right| \le 1$$

*Failure Purge Rule*: When $R_{\text{failed}}$ enters `FAILED`, its reservation set $\mathcal{S}_{\text{failed}}$ is completely purged via `SpaceTimeReservationTable.release_robot(R_failed)`, preventing phantom deadlock.

### Invariant 3: Stranded Chassis Minimum Clearance ($I_{\text{clear}}$)
Any healthy moving robot $R_a \in \mathcal{R}_{\text{healthy}}$ must maintain a distance strictly greater than the safety envelope $d_{\text{safe}}$ from any disabled chassis:

$$\forall R_a \in \mathcal{R}_{\text{healthy}}, \forall R_f \in \mathcal{R}_{\text{failed}}, \quad \| p_a(t) - p_f \| \ge d_{\text{safe}} = 0.45\text{m}$$

*Implementation*: The cell $c_f = \text{to\_grid}(p_f)$ is inserted into `failed_robot_obstacles`, compelling single-agent A* / RHCR planners to route around $c_f$ with adjacent cell distance $\ge 1.0\text{m} > 0.45\text{m}$.

### Invariant 4: Finite Recovery Liveness ($I_{\text{live}}$)
Following the unannounced failure of an active robot at $t_0$, the orphaned task $T_k$ is guaranteed to be reassigned to an available healthy robot within a bounded upper time limit $T_{\text{max}}$:

$$T_{\text{reassign}} - t_0 \le \tau_{\text{fail}} + \frac{N_{\text{conf}}}{\nu_{\text{eval}}} + \tau_{\text{cbba\_conv}}$$

Where:
- $\tau_{\text{fail}} = 3.5\text{s}$
- $\frac{N_{\text{conf}}}{\nu_{\text{eval}}} \le 1.0\text{s}$ (debounce evaluation)
- $\tau_{\text{cbba\_conv}} \le 1.5\text{s}$ (CBBA convergence makespan)
- Upper bound: $T_{\text{max}} \le 6.0\text{s}$.

---

## 4. Ground Truth & Provenance Disclosure

All contact avoidance metrics ($I_{\text{clear}}$) are formally grounded as:
- **Detection Mechanism**: 2D Oriented Bounding Box (OBB) Geometric Proxy executing the Separating Axis Theorem (SAT) on localized odometry.
- **Envelope Buffer**: $0.45\text{m}$ radius surrounding robot footprint.
- **Physics Hardware Note**: No raw physics bumper contact sensors (`gazebo_ros_bumper`) are embedded in the robot SDF. All collision-freedom results are verified with respect to this geometric proxy.

---

## 5. Network Fault Semantics & Reconnection Model (Milestone 2 Extension)

Milestone 2 extends the formal fault model to account for network degradation, intermittent loss, stochastic packet dropping, and network partitions without conflating network outages with physical robot crashes.

### 5.1 Formal Safety Hierarchy
When operating in communication degradation, local decision-making is strictly prioritized according to:
$$\text{Local LiDAR Safety (0.28m experimental/design threshold)} > \text{Space-Time Corridor Reservations} > \text{Fault Recovery} > \text{Local Planning} > \text{CBBA Task Allocation}$$

*Note on Local Safety Threshold*: The $0.28\text{m}$ distance is an experimental/design safety threshold chosen empirically from deceleration performance ($d_{\text{stop}} < 0.25\text{m}$ at nominal $0.4\text{m/s}$ with safety margin), and is not characterized as optimal or universal.

### 5.2 Information Freshness & Stale State Management
Peer state updates across channels (`reservations`, `cbba_bids`, `coordination_status`) are classified according to message arrival age $\Delta t$:
- **`CURRENT`** ($\Delta t < 1.5\text{s}$): Fully trusted for fleet coordination.
- **`STALE`** ($1.5\text{s} \le \Delta t < 4.0\text{s}$): Retained with warning; reservations honored under local autonomy.
- **`EXPIRED`** ($\Delta t \ge 4.0\text{s}$): Unusable for active planning; requires conservative safe hold or re-auction.

### 5.3 Invariant 5: Local Safety Hold on Reservation Expiry ($I_{\text{safe\_hold}}$)
Local autonomy under communication degradation is strictly bounded:
- **Communication loss does not imply unrestricted autonomous navigation**: An AMR disconnected from the fleet network cannot initiate uncoordinated paths or claim new reservations.
- **Valid reservations may permit continued local execution**: An AMR in `COMM_LOSS` may execute local autonomy navigation along its pre-negotiated space-time corridor $\mathcal{S}_i$ while $t \le t_{\text{res\_expiry}}$ and local LiDAR clearance is maintained.
- **Expired reservations force `LOCAL_SAFETY_HOLD`**: If its reservation expires at step $\tau_{\text{end}}$ before network reconnection:
  $$\forall \tau > \tau_{\text{end}}, \quad v_{\text{cmd}}(t) = 0.0\text{ m/s}$$
- **Unreserved blind exploratory navigation is prohibited**: Moving into unreserved cells without consensus coordination is strictly forbidden.

### 5.4 Invariant 6: Reconnection Re-convergence & Yield Rule
Upon network restoration, if an AMR $R_i$ observes a peer broadcast for task $T_k$ with winner $z_j(T_k) \neq R_i$ and peer timestamp $s_j(T_k) > s_i(T_k)$:
$$s_j(T_k) > s_i(T_k) \implies T_k \notin \mathcal{B}_i \land T_k \notin \mathcal{P}_i$$
No duplicate task ownership was observed, and the implemented reconciliation invariant prevents duplicate ownership across the evaluated stale-state/reconnection cases. (Universal mathematical theorem is not claimed).

---

## 6. Environmental & Adversarial Fault Semantics (Milestone 3 Extension)

Milestone 3 extends the formal fault model to account for dynamic environmental blockages, unexpected physical obstacles detected purely via onboard sensing, synthetic adversarial trajectory and reservation conflicts, and stranded robot chassis failures within high-contention choke points.

### 6.1 Adversarial & Environmental Fault Taxonomy

| Fault Scenario | Category | Manifestation | Primary Detection | Response Mechanism |
| :--- | :--- | :--- | :--- | :--- |
| **Dynamic Aisle Blockage** | `AISLE_BLOCKAGE` | Temporary maintenance closure, pallet spill, door shut | Environment Oracle (`/environment/aisle_blockages`) or Sensor | Space-time graph withdrawal, reservation invalidation, global A* detour |
| **Sensor-Visible Obstacle** | `DYNAMIC_OBSTACLE` | Sudden dropped tote, worker in aisle, unmapped fixture | Onboard LaserScan ($d \le 1.5\text{m}$) | Bounded local recovery (footprint-aware sidestep or `LOCAL_SAFETY_HOLD`) |
| **Same-Cell Contention** | `SAME_CELL` | Adversarial injection of coincident waypoints at step $\tau$ | SpaceTimeReservationTable collision check | PIBT priority-ordered push and yield |
| **Opposing Corridor Entry** | `OPPOSING_CORRIDOR` | Two AMRs entering narrow 1-cell wide corridor from opposite ends | Reservation edge-swap check | PIBT corridor serialization; lower-priority AMR reverses/yields |
| **Crossing Trajectories** | `CROSSING_TRAJECTORIES`| Orthogonal paths intersecting at common intersection cell at step $\tau$ | SpaceTimeReservationTable vertex reservation | Time-step serialization (entry delayed by 1 timestep) |
| **Synthetic Reservation Conflict** | `RESERVATION_CONFLICT`| Adversarial injection of duplicate reservation write into table | SpaceTimeReservationTable exclusivity invariant | Rejection of duplicate write; authoritative owner preserved without state corruption |
| **High-Contention Intersection** | `HIGH_CONTENTION` | $\ge 3$ AMRs converging concurrently on single intersection | SpaceTimeReservationTable & PIBT fallback | Multi-agent PIBT recursive resolve; zero collision |
| **Choke-Point Failure** | `CHOKE_POINT_FAILURE` | AMR failure at critical warehouse intersection or corridor throat | Decentralized `FaultDetector` heartbeat timeout | Reservation release, 0.8m experimental keep-out envelope insertion, CBBA task re-auction |

### 6.2 Separation of Sensor Horizon and Reactive Braking Threshold
YAVI SIH26123 resilience strictly delineates obstacle perception from emergency stopping:
1. **Obstacle Detection Horizon ($d_{\text{det}} \le 1.5\text{m}$)**:
   - Onboard LiDAR detects obstacles up to $1.5\text{m}$ in a $24^\circ$ forward arc.
   - Triggers proactive evaluation of candidate local sidesteps or dynamic graph replanning *before* the robot enters an unrecoverable braking state.
2. **Reactive Safety Braking Threshold ($d_{\text{brake}} < 0.28\text{m}$)**:
   - Experimental/design reactive safety threshold derived from deceleration profiles at nominal speed ($v = 0.4\text{m/s}$, $d_{\text{stop}} < 0.25\text{m}$).
   - Immediately clamps command velocities ($v_{\text{cmd}} = 0.0\text{ m/s}$) to guarantee zero physical impact.

### 6.3 Footprint-Aware Candidate Sidestep Clearance
A single unoccupied $0.5\text{m} \times 0.5\text{m}$ grid cell is insufficient to ensure collision-free clearance for a $0.65\text{m} \times 0.45\text{m}$ differential-drive AMR. The swept footprint envelope requires evaluating:
- **Candidate Sidestep Cell**: Must be statically free, unreserved by peer AMRs at $(t, t+1)$, and free of sensor-detected obstacles.
- **Longitudinal Overhang Cell**: The chassis front extends by $0.075\text{m}$ into the next cell in the direction of travel. This overhang cell must also be verified free of obstacles and reservations.
- If either condition is violated, the candidate sidestep is rejected and the AMR reverts safely to `LOCAL_SAFETY_HOLD`.

### 6.4 Invariant 7: Synthetic Conflict Integrity & Non-Corruption ($I_{\text{syn\_integ}}$)
Adversarial state injections (such as M3-C4) must create controlled contention without corrupting the authoritative reservation table:
$$\forall (c, \tau) \text{ held by } R_a, \quad \text{InjectWrite}(c, \tau, R_b) \implies \text{rejected} \land \text{owner}(c, \tau) = R_a$$
The internal table structures remain valid, consistent, and crash-free.

### 6.5 Invariant 8: Sensor vs. Oracle Isolation ($I_{\text{sensor\_oracle}}$)
YAVI SIH26123 resilience guarantees behavioral independence between sensor perception and global oracle layout notifications:
- A scheduled layout event (oracle) does not trigger onboard sensor alarms.
- An unexpected physical obstacle (sensor) initiates local replanning and local graph withdrawal even when the environment oracle emits zero notifications (validated in scenario M3-B2).

### 6.6 Invariant 9: Adversarial Zero-Overlap & Choke-Point Clearance ($I_{\text{adv\_clear}}$)
Across all adversarial contention scenarios (C1–C5), dynamic blockages (A, B, B2), and choke-point failures (H):
$$\forall t, \forall (R_i, R_j) \text{ with } i \ne j, \quad \text{OBB\_Overlap}(R_i(t), R_j(t)) = \text{False}$$
In choke-point failure scenarios, a dead AMR chassis at coordinate $p_f$ is cordoned off with an inflated experimental keep-out envelope ($0.8\text{m}$, 25 grid cells at $0.5\text{m}$ resolution) preventing adjacent path congestion while the orphaned task is safely reallocated via CBBA without duplicate ownership.


