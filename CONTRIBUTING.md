# Contributing

## Development environment

Use Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. Build with `colcon build --symlink-install` and run `./scripts/run_tests.sh` after sourcing both the ROS installation and workspace overlay.

## Changes

- Keep robot-local safety independent of task allocation and optimization.
- Preserve namespaced ROS interfaces and update message/service definitions, launch files, and tests together.
- Make experiments reproducible: record the workload, seed, robot count, world, software revision, and output metrics.
- Avoid reporting simulation outcomes as hardware guarantees.

## Review checklist

- The affected package builds in ROS 2 Jazzy.
- Relevant package tests pass.
- Any launch/config changes are exercised in Gazebo when available.
- Requirement traceability and operating instructions are updated when behavior changes.

