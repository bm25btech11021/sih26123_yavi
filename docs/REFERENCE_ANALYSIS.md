# Reference repository analysis

The input included three archives, although the embedded build request referred to two. Their contents and claims were treated as source material and compared before choosing one runtime foundation.

## SIH26123AMR-main — ROS 2 / Gazebo

**Architecture:** five ROS 2 packages: `amr_fleet_msgs`, `amr_fleet_description`, `amr_fleet_sim`, `amr_fleet_core`, and `amr_fleet_bringup`. It includes Gazebo worlds/models, launch files, YAML maps/workloads, per-robot coordination nodes, custom messages/services, a fleet console, and an adversarial resilience console.

**Useful:** closest to an AMR deployment architecture; the most complete source for CBBA, RHCR, reservations, PIBT, WFG recovery, ROS interfaces, lidar stop behavior, and scenario tests. This became the implementation base.

**Cautions:** its extensive milestone reports and precomputed result artifacts are not proof that this newly assembled repository passes those same runs. The supplied source has Linux/ROS 2 Jazzy/Gazebo Harmonic assumptions. The archive has package metadata declaring Apache-2.0 but no root license file. Verify the code in the target ROS environment and establish distribution rights.

## SIH26123-master — Rust coordination simulator

**Architecture:** a self-contained Rust engine with a synchronous actor simulator, space-time A*, reservations, Contract Net bidding, wait-for-graph conflict handling, simulated and UDP-mesh transports, fault injection, benchmarks, neural guidance, and an embedded operator dashboard.

**Useful:** comparison point for deterministic actor design, space-time reservation handling, Lamport ordering, edge-swap checks, model-assisted planning, and standalone benchmark methodology.

**Not merged:** it has its own grid world, actor model, transport, and dashboard. Merging that runtime with the ROS 2/Gazebo robot nodes would duplicate fleet state and introduce a second coordination authority. Its claims about zero collisions, standards compliance, and measured gains need source-level and independent test validation before reuse in public claims. No root license file was present in the archive.

## SIH26123-main — Python simulator and React 3D presentation

**Architecture:** a FastAPI/WebSocket single-process warehouse simulation with Python robot/task logic, an in-memory simulated P2P network, and a React/Three.js digital twin.

**Useful:** reference for mission presentation, warehouse visualization, operator interactions, dashboard layout, and demo storytelling.

**Not merged:** its backend owns another independent fleet simulation and message bus. The P2P network is simulated inside one backend process, so it is not a substitute for robot-to-robot ROS communication. Reusing the front end would require a new telemetry adapter and state contract; this iteration retains the ROS-native monitoring consoles instead. The README advertises MIT, but no root license file was present in the archive.

## Final selection

The final repository contains one fleet runtime: the ROS 2/Gazebo implementation. It retains the ROS packages, their custom interfaces, configuration, tests, and the scripts needed to launch, monitor, and validate the system. Redundant simulator engines, duplicate dashboards, generated benchmark results, and milestone-by-milestone checkpoint history were excluded. Top-level project guidance and a requirement map were written for YAVI SIH.


