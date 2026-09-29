# Completion checklist

Use this checklist for a release candidate. A checked box requires evidence from the current candidate build.

- [ ] All five ROS 2 packages resolve with `rosdep` and build on the documented distro.
- [ ] Package tests pass and `colcon test-result --verbose` is archived.
- [ ] The documented small-world launch starts the requested fleet and each robot has isolated namespaced topics/TF.
- [ ] Task creation, allocation, execution, cancellation, and supported recovery flows are exercised.
- [ ] Obstacle, communication, and robot-failure scenarios are run with configuration and output saved.
- [ ] Dashboard and resilience console connect to the live ROS graph and show current state.
- [ ] Benchmark comparisons use matched maps, tasks, seeds, robot starts, and termination rules.
- [ ] Collision checks include both same-cell and edge-swap conflicts where the benchmark model supports them.
- [ ] Performance statements include sample counts, exclusions, uncertainty, and machine/software configuration.
- [ ] Physical hardware claims are backed by separate HIL/robot evidence and are not inferred from simulation.
- [ ] The team confirms code and asset distribution rights and adds the approved root license.


