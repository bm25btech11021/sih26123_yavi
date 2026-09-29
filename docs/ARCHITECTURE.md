# YAVI SIH26123 AMR Fleet Coordination: System Architecture

> This describes the architecture represented by the included source. It is a design and implementation map, not a certification or proof of collision-free operation. Verify behavior against the exact build, world, workload, and hardware before making performance or safety claims.

> **System Overview**: Fully decentralized, multi-agent coordination architecture for Autonomous Mobile Robots (AMRs) operating in dynamic, communication-degraded warehouse logistics environments.

---

## 1. Architectural Hierarchy & Authority Model

The software architecture is structured into a strict priority hierarchy where local physical safety controllers maintain absolute preemption authority over high-level multi-agent coordination, planning, and task allocation layers.

```
+-------------------------------------------------------------------------+
|      YAVI SIH26123 resilience ADVERSARIAL EXPERIMENT & RESILIENCE CONTROLLER (M4)       |
|                        (Web Dashboard @ Port 8081)                      |
|  [Multi-Domain Disturbance Matrix]   [Three-Tier Response Engine]       |
|  - Robot Crash / Comm Loss           - Tier 1: CBBA CAS Task Reclaim    |
|  - Packet Loss / Partition           - Tier 2: Dynamic Detour Replan    |
|  - Corridor / Aisle Blockage         - Tier 3: 20Hz LiDAR Clearance     |
|  [Formal Invariant Engine: Inv 1 (Task Mutex), Inv 2 (Resv), Inv 3 (Clear)]
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                       CROSS-CUTTING CONTROLS                            |
|                                                                         |
|  [Adaptive Compute Policy Layer]      [Decentralized Comm Fabric]       |
|  - Host CPU/RAM Monitoring            - ROS 2 Jazzy P2P Topics          |
|  - Contention & Network Telemetry     - DDS / Zenoh Micro-Broker        |
|  - Dynamic Rate/Horizon Modulation    - Loss/Latency Impairment Model   |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                       HIGH-LEVEL TASK ALLOCATION                        |
|                                                                         |
|  [Task Generation & Lifecycle Engine]                                   |
|  - Staged / Dynamic Workload Arrival                                    |
|                                                                         |
|  [Decentralized CBBA / ACBBA Consensus Layer]                           |
|  - Greedy Task Bundle Construction                                      |
|  - Distributed Consensus via Maximum-Bid Resolution                     |
|  - Dynamic Bundle Expansion without Disrupting Active Tasks             |
|  - Atomic Compare-And-Swap (CAS) Task Reclamation on Node Failure       |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                   MULTI-AGENT SPATIO-TEMPORAL PLANNING                  |
|                                                                         |
|  [Rolling-Horizon Collision Resolution (RHCR)]                          |
|  - Windowed Spatio-Temporal A* Search (h=10 steps, w=4 steps)           |
|  - Local GridWorld Map with Dynamic Obstacle Rasterization              |
|  - Station-Resource & Headway Queue Claim Logic                         |
|  - Dynamic Graph Re-routing on Corridor Invalidation (<0.1ms latency)  |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                    DECENTRALIZED COORDINATION LAYER                     |
|                                                                         |
|  [Space-Time Reservation Table]                                         |
|  - Forward Corridor Occupancy Claims [x, y, t_start, t_end]             |
|  - Autonomous TTL-Based Reservation Expiry & Invalidation Pruning       |
|                                                                         |
|  [Wait-For Graph (WFG) Deadlock Detection & Recovery]                   |
|  - Cycle Detection in Spatial Claims                                    |
|  - Deterministic Yielding & Priority-Based Evasion                      |
|                                                                         |
|  [Priority Inheritance Backtracking (PIBT) Local Coordinator]           |
|  - 1-Step Spatio-Temporal Neighbor Conflict Resolution                  |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                      LOCAL PHYSICAL SAFETY LAYER                        |
|                     (ABSOLUTE PREEMPTION AUTHORITY)                     |
|                                                                         |
|  [Reactive LiDAR Safety Braking Controller]                             |
|  - 10-20 Hz Independent Execution Loop                                  |
|  - Direct 2D LaserScan Distance Thresholding                            |
|  - Emergency Stop & Corridor Deceleration (0.28m-0.35m Safety Envelope) |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                        LOW-LEVEL ROBOT ACTUATION                        |
|                                                                         |
|  [Differential Drive Velocity Controller] -> Gazebo Harmonic DART       |
+-------------------------------------------------------------------------+
```

---

## 2. Priority of Authority & Preemption Rules

$$\text{LOCAL SAFETY} \succ \text{RESERVATION / PIBT COORDINATION} \succ \text{RHCR PLANNING} \succ \text{CBBA TASK ALLOCATION}$$

1. **Local LiDAR Safety ($10\,\text{Hz}$)**:
   - Evaluates real-time obstacle distances directly from `/scan`.
   - If an obstacle enters the protective safety bubble ($d < 0.35\,\text{m}$), the safety backstop commands $v=0, \omega=0$ immediately.
   - Decoupled from ROS 2 planning timers and immune to planning latency, network loss, or CBBA consensus states.
2. **Space-Time Reservations & PIBT ($2\text{--}5\,\text{Hz}$)**:
   - Resolves multi-robot corridor contention before physical trajectory execution.
   - If a peer's space-time reservation conflicts with the desired path, PIBT triggers a 1-step wait or detour.
3. **Rolling-Horizon Collision Resolution ($1\text{--}4\,\text{Hz}$)**:
   - Computes kinematically feasible trajectories over a short horizon ($h=10$ timesteps) and executes only the prefix ($w=4$ timesteps).
4. **CBBA Task Allocation ($2\text{--}10\,\text{Hz}$)**:
   - Builds bundles asynchronously. Executes only converged bundles (`is_converged=True`). Tasks in progress survive mid-mission reallocation.

> [!NOTE]
> This hierarchical design enforces defense-in-depth. It does not constitute a formal mathematical proof of collision-freedom under all arbitrary physical conditions, but provides robust empirical validation within the evaluated scenarios and simulation configurations.

---

## 3. Communication Fabric & Resilient Telemetry

The inter-robot communication fabric is implemented over distributed ROS 2 peer-to-peer topics:

- **CBBA Consensus**: `/{robot_id}/cbba_bid` and `/{robot_id}/cbba_bundle`
- **Space-Time Reservations**: `/fleet/reservations`
- **Aisle Blockages**: `/fleet/aisle_blockage_events`
- **Adaptive Compute Events**: `/fleet/compute_events`
- **Communication Metrics**: `/{robot_id}/comm_metrics`

### Network Degradation Tolerance
- Each node maintains a local `StaleStateManager` and `ReservationTable`.
- Heartbeat timeouts ($T_{\text{timeout}} = 1.5\,\text{s}$) automatically prune expired reservations from peers whose packets were dropped during network degradation (`LOSS_HIGH`, $p_{\text{loss}} = 0.35$).
- Reclaimed corridor reservations allow AMRs to maintain localized progress without locking the fleet in false deadlocks.

---

## 4. Adaptive Compute Allocation

The `AdaptiveComputePolicy` runs as a cross-cutting telemetry-driven policy engine that dynamically modulates computational workload across three operational modes:

| Compute Mode | Replan Rate ($f_{\text{replan}}$) | Horizon ($h$) | Execution Window ($w$) | CBBA Rate ($f_{\text{cbba}}$) | Planning Budget ($\tau$) | Trigger Conditions |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **`LOW`** | $1.0\,\text{Hz}$ | 6 steps | 6 steps | $2.0\,\text{Hz}$ | $20\,\text{ms}$ | Host CPU $>80\%$, Stale messages $>5$, or high network packet drop |
| **`NORMAL`** (Baseline) | $2.0\,\text{Hz}$ | 10 steps | 4 steps | $5.0\,\text{Hz}$ | $50\,\text{ms}$ | Nominal operations, CPU $<70\%$, healthy communication |
| **`HIGH`** | $4.0\,\text{Hz}$ | 14 steps | 2 steps | $10.0\,\text{Hz}$ | $100\,\text{ms}$ | High spatial contention (AMR density $>0.15\,\text{AMR/m}^2$) |

- Anti-oscillation guards: Minimum dwell time $T_{\text{dwell}} = 3.0\,\text{s}$, $K=3$ consecutive confirmation samples, and asymmetric hysteresis thresholds.
- Failsafe mechanism: If telemetry becomes unavailable for $>5.0\,\text{s}$, the policy safely falls back to `NORMAL`.

---

## 5. YAVI SIH26123 resilience Compound Resilience & Adversarial Scenario Architecture (Milestone M4)

Milestone 4 introduces compound fault resilience, enabling the decentralized fleet to survive concurrent, cascading failures across physical, network, and environmental domains.

### 5.1 Orthogonal 5-Dimensional State Modeling

To prevent split-brain state divergence and race conditions during simultaneous failures, each AMR $i$ maintains a 5-tuple state:
$$\mathcal{S}_i = \langle s_{health}, s_{comm}, s_{task}, s_{nav}, s_{res} \rangle$$
- $s_{health} \in \{\text{HEALTHY}, \text{COMM\_LOSS}, \text{FAILED}, \text{ESTOP}\}$
- $s_{comm} \in \{\text{CONNECTED}, \text{IMPAIRED}, \text{PARTITIONED}, \text{DISCONNECTED}\}$
- $s_{task} \in \{\text{STAGED}, \text{PENDING}, \text{ASSIGNED}, \text{IN\_PROGRESS}, \text{COMPLETED}, \text{FAILED}, \text{CANCELLED}\}$
- $s_{nav} \in \{\text{IDLE}, \text{TRANSIT}, \text{LOCAL\_SAFETY\_HOLD}, \text{SIDESTEP\_RECOVERY}, \text{ESTOP\_HOLD}\}$
- $s_{res} \in 2^{\mathcal{G} \times \mathbb{N}}$: Spacetime reservation ownership set.

### 5.2 Three-Tier Decentralized Fleet Response

When compound disruptions strike, the fleet activates three synchronized recovery tiers:
1. **Tier 1: Task Reallocation (CBBA Consensus Engine)**:
   - When node failure is confirmed ($t > 3.5\,\text{s}$), orphaned tasks are reclaimed via atomic Compare-And-Swap (`CAS(T, ASSIGNED(victim) -> PENDING)`).
   - Surviving agents dynamically re-bid without disrupting tasks already in progress.
2. **Tier 2: Dynamic Replanning (Spatiotemporal Graph)**:
   - Blocked corridors and crashed AMR positions are invalidated across local occupancy grids.
   - Forward reservations are purged instantly ($|\{i : (c, \tau) \in \mathcal{R}_i\}| \le 1$).
   - SingleAgentAStar / RHCR generates valid alternative detours in $<0.1\,\text{ms}$.
3. **Tier 3: Local Safety & Clearance (Reactive Sensor Backstop)**:
   - $20\,\text{Hz}$ onboard LiDAR obstacle scanning operating autonomously from the network.
   - Enforces minimum clearance envelope $r_{robot} = 0.28\,\text{m}$.
   - Triggers emergency reactive braking ($v=0, \omega=0$) upon sudden unmapped corridor obstruction.

### 5.3 Web-Based Adversarial Experiment Controller (Port 8081)

The system exposes an interactive operator dashboard via `scripts/resilience_dashboard.py`:
- **Real-time REST API**: `/api/m4/status`, `/api/m4/inject_compound`, `/api/m4/compose_and_run`, `/api/m4/step`, `/api/m4/clear`, `/api/m4/report`.
- **7-Stage Deterministic Recovery Stepper**: Allows deterministic step-by-step inspection of failure propagation and recovery.
- **Continuous Invariant Auditing**: Evaluates $I_1$ (Task Mutex), $I_2$ (Spacetime Reservation Exclusivity), and $I_3$ (Clearance Guarantee) continuously during adversarial testing.


