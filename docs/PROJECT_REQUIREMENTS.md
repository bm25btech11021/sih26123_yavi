# Project requirements and scope

This summary is based on the supplied ROS 2 archive's PRD and implementation documents, plus the SIH26123 project context in the provided chat. The official problem statement itself was not present as a separate source file in this workspace, so use the official SIH document to confirm wording and acceptance thresholds.

## Functional scope

- Spawn a configurable multi-AMR warehouse simulation with namespaced robot interfaces.
- Generate and manage pickup/drop-off work with task metadata.
- Support a centralized comparison baseline and distributed CBBA-style allocation.
- Plan routes with rolling-horizon methods and coordinate shared space through reservations and local conflict resolution.
- Detect and recover from supported deadlock, blocked-aisle, stale-peer, robot, and communication fault scenarios.
- Adapt planning effort to measured compute/network conditions while preserving local safety authority.
- Expose monitoring and operator controls through dashboards and ROS services.
- Save experiment configuration and metrics for repeatable evaluation.

## Non-functional scope

- Keep the local safety path independent of task allocation and global planning.
- Avoid fixed fleet-size assumptions in the coordination interfaces.
- Run the distributed nodes on the documented ROS 2 middleware topology.
- Back throughput, reliability, and resilience statements with reproducible measurements.

## Out of scope for this repository snapshot

- Certified physical operation or a formal end-to-end safety proof.
- A commercial-grade fleet management product.
- The independent Rust and Python/React simulation engines from the other supplied archives.
- A verified public release license; the supplied archive lacked root license texts.


