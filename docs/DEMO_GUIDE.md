# Demo guide

This is a repeatable demonstration flow, not a prewritten claim of benchmark results.

1. Build the ROS 2 workspace and launch `warehouse_small` with five robots.
2. Show the namespaced fleet and live robot/task state in the fleet dashboard.
3. Create a task using the dashboard or `scripts/create_task.py`; show task bidding and execution.
4. Use the resilience console to block an aisle or inject a communication/robot fault supported by the selected scenario.
5. Show the resulting detection, reservation/replan or recovery events, and robot status.
6. Repeat with the same world, robot count, workload, seed, and fault timing when comparing configurations.
7. Present only measurements captured from the current run. State the simulator configuration and its limitations.

Before presenting, verify both dashboards connect, record the ROS domain and selected workload, and make sure the scenario does not depend on archived result images or reports.


