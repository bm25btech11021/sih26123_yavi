# Runbook

## Build workspace

Run from the repository root on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic installed:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Normal visual run

```bash
./scripts/launch_all_in_one.sh --world warehouse_small
```

The script defaults to the larger `warehouse_m9_v2` world. Pass `--headless` for a server-only Gazebo run, `--robots 5` to set the fleet size, or `--workload config/workloads/workload_15_tasks.yaml` to choose a workload. Run `./scripts/launch_all_in_one.sh --help` for supported options. Stop the launched processes with Ctrl+C.

The fleet dashboard is served at `http://localhost:8080`; the resilience experiment console is at `http://localhost:8081`.

## Run components separately

```bash
export ROS_DOMAIN_ID=42
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=5 world:=warehouse_small headless:=false \
  compute_mode:=ADAPTIVE comm_profile:=NORMAL \
  workload_file:=config/workloads/workload_15_tasks.yaml
```

In another terminal, source ROS and the workspace overlay, then run:

```bash
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 5 --world warehouse_small
```

The resilience console can run without Gazebo in its standalone scenario mode:

```bash
python3 scripts/resilience_dashboard.py --sim-mode --port 8081
```

## Task control

Source ROS and `install/setup.bash` in the client terminal. Use `python3 scripts/create_task.py --help` to see task creation arguments. The task manager must be running in the same ROS domain.

## Validate

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/run_tests.sh
python3 scripts/validate_m4_compound_scenarios.py
```

Store experiment configuration, seed, software revision, logs, and metrics with any reported result. This repository does not include the prior archive's generated results.


