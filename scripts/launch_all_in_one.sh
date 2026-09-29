#!/usr/bin/env bash
# ==============================================================================
# YAVI-SIH26123: ALL-IN-ONE MASTER FLEET LAUNCH SCRIPT
# ==============================================================================
# Launches all system components in a single execution:
#   1. Gazebo Harmonic 3D Physics Simulation (with GUI)
#   2. Multi-AMR Fleet Autonomy Stack (Task Manager, CBBA, Rolling-Horizon, PIBT)
#   3. RViz 2 Visualization (fleet_default.rviz with TF, LiDAR & footprints)
#   4. Fleet Observability Dashboard (Web UI on http://localhost:8080)
#   5. Resilience & Fault Injection Dashboard (Web UI on http://localhost:8081)
#
# Provides robust trap handling for clean termination on Ctrl+C.
# ==============================================================================

set -e

# Resolve directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_ROOT}"

# Default parameters
WORLD="warehouse_m9_v2"
ROBOTS=""
WORKLOAD=""
HEADLESS="false"
RVIZ="true"
DASHBOARDS="true"
PORT_FLEET=8080
PORT_RESILIENCE=8081
COMPUTE_MODE="ADAPTIVE"
COMM_PROFILE="NORMAL"

# Usage help
print_usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

YAVI-SIH26123 All-in-One Fleet Launch Script

Options:
  -w, --world <name>         Gazebo world name (warehouse_m9_v2 | warehouse_small) [default: warehouse_m9_v2]
  -n, --robots <count>       Number of AMRs (default: 10 for m9_v2, 5 for small)
  -c, --workload <file>      Path to workload YAML file
  -m, --compute-mode <mode>  Compute mode (ADAPTIVE | NORMAL | HIGH | LOW) [default: ADAPTIVE]
  -p, --comm-profile <prof>  Comm degradation profile (NORMAL | HIGH_LATENCY | JITTER | LOSS_HIGH | OUTAGE) [default: NORMAL]
  --headless                 Run Gazebo Harmonic in headless server mode (no 3D GUI)
  --no-rviz                  Do not start RViz 2 visualization
  --no-dashboards            Do not start the web dashboards
  --port-fleet <port>        Web port for Fleet Observability Dashboard [default: 8080]
  --port-resilience <port>   Web port for Resilience Dashboard [default: 8081]
  -h, --help                 Show this help message and exit

Examples:
  # Launch full big warehouse stack (10 AMRs, Gazebo GUI, RViz, Dashboards):
  ./scripts/launch_all_in_one.sh

  # Launch standard warehouse stack (5 AMRs):
  ./scripts/launch_all_in_one.sh --world warehouse_small

  # Launch headless simulation with RViz and Dashboards:
  ./scripts/launch_all_in_one.sh --headless
EOF
}

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -w|--world)
            WORLD="$2"
            shift 2
            ;;
        -n|--robots|--fleet-size)
            ROBOTS="$2"
            shift 2
            ;;
        -c|--workload)
            WORKLOAD="$2"
            shift 2
            ;;
        -m|--compute-mode)
            COMPUTE_MODE="$2"
            shift 2
            ;;
        -p|--comm-profile)
            COMM_PROFILE="$2"
            shift 2
            ;;
        --headless)
            HEADLESS="true"
            shift
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --no-dashboards)
            DASHBOARDS="false"
            shift
            ;;
        --port-fleet)
            PORT_FLEET="$2"
            shift 2
            ;;
        --port-resilience)
            PORT_RESILIENCE="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Set dynamic defaults based on selected world
CLEAN_WORLD="${WORLD%.sdf}"
if [ -z "${ROBOTS}" ]; then
    if [[ "${CLEAN_WORLD}" == *"m9"* || "${CLEAN_WORLD}" == *"32"* ]]; then
        ROBOTS=10
    else
        ROBOTS=5
    fi
fi

if [ -z "${WORKLOAD}" ]; then
    if [[ "${CLEAN_WORLD}" == *"m9_v2"* ]]; then
        WORKLOAD="config/workloads/workload_30_tasks_m9_v2.yaml"
    elif [[ "${CLEAN_WORLD}" == *"small"* ]]; then
        WORKLOAD="config/workloads/workload_15_tasks.yaml"
    else
        WORKLOAD="config/workloads/workload_30_tasks_m9_v2.yaml"
    fi
fi

# Pre-flight cleanup of old AMR processes and locked ports
echo "================================================================================"
echo " [YAVI-SIH26123] PRE-FLIGHT CHECK & INITIALIZATION"
echo "================================================================================"
echo " - Cleaning up any dangling AMR simulation, RViz, or Dashboard processes..."
pkill -9 -f "amr_fleet" 2>/dev/null || true
pkill -9 -f "fleet_dashboard" 2>/dev/null || true
pkill -9 -f "resilience_dashboard" 2>/dev/null || true
pkill -9 -f "warehouse_m9" 2>/dev/null || true
pkill -9 -f "warehouse_small" 2>/dev/null || true
pkill -9 -f "fleet_default.rviz" 2>/dev/null || true
pkill -9 -f "m8b_adaptive_fleet" 2>/dev/null || true
pkill -9 -f "cbba_node" 2>/dev/null || true
pkill -9 -f "rh_node" 2>/dev/null || true
pkill -9 -f "task_manager_node" 2>/dev/null || true
pkill -9 -f "warehouse_visualizer" 2>/dev/null || true
sleep 1

# Environment Setup
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
echo " - Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (isolated DDS domain)"

# Environment Setup
echo " - Sourcing ROS 2 Jazzy and Workspace Overlay..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
else
    echo "[WARN] ${WORKSPACE_ROOT}/install/setup.bash not found. Building workspace..."
    colcon build --symlink-install
    source "${WORKSPACE_ROOT}/install/setup.bash"
fi

export PYTHONPATH="${WORKSPACE_ROOT}/src/amr_fleet_core:${WORKSPACE_ROOT}/src/amr_fleet_sim:${PYTHONPATH}"
export GZ_SIM_RESOURCE_PATH="${WORKSPACE_ROOT}/src/amr_fleet_bringup/worlds:${WORKSPACE_ROOT}/src/amr_fleet_bringup/models:${GZ_SIM_RESOURCE_PATH}"

# Process tracking and cleanup trap
PIDS=()
CLEANED_UP=0

cleanup() {
    if [ "$CLEANED_UP" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1
    echo ""
    echo "================================================================================"
    echo " [YAVI-SIH26123] SHUTTING DOWN ALL FLEET SERVICES..."
    echo "================================================================================"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGTERM "$pid" 2>/dev/null || true
        fi
    done

    # Give processes a brief moment to shut down cleanly
    sleep 2

    # Force-kill any lingering processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    pkill -9 -f "amr_fleet" 2>/dev/null || true
    pkill -9 -f "fleet_dashboard" 2>/dev/null || true
    pkill -9 -f "resilience_dashboard" 2>/dev/null || true
    pkill -9 -f "warehouse_m9" 2>/dev/null || true
    pkill -9 -f "warehouse_small" 2>/dev/null || true
    pkill -9 -f "fleet_default.rviz" 2>/dev/null || true
    pkill -9 -f "m8b_adaptive_fleet" 2>/dev/null || true
    pkill -9 -f "cbba_node" 2>/dev/null || true
    pkill -9 -f "rh_node" 2>/dev/null || true
    pkill -9 -f "task_manager_node" 2>/dev/null || true
    pkill -9 -f "warehouse_visualizer" 2>/dev/null || true

    echo " [YAVI-SIH26123] All simulation and dashboard processes cleanly terminated."
    echo "================================================================================"
}

trap cleanup SIGINT SIGTERM EXIT

# Step 1: Launch Multi-AMR Fleet Simulation (Gazebo + Core Nodes)
echo " - [1/4] Starting Gazebo Harmonic & AMR Autonomy Stack (${ROBOTS} AMRs in ${CLEAN_WORLD})..."
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
    robot_count:="${ROBOTS}" \
    world:="${CLEAN_WORLD}" \
    headless:="${HEADLESS}" \
    rviz:=false \
    compute_mode:="${COMPUTE_MODE}" \
    comm_profile:="${COMM_PROFILE}" \
    workload_file:="${WORKLOAD}" > /tmp/nrdas_ros_launch.log 2>&1 &
ROS_LAUNCH_PID=$!
PIDS+=("$ROS_LAUNCH_PID")

# Brief stagger to let Gazebo start physics server & clock bridge before visualizers connect
echo " - Waiting 4 seconds for Gazebo physics server & clock bridge initialization..."
sleep 4

# Step 2 & 3: Launch Dashboards
if [ "$DASHBOARDS" = "true" ]; then
    echo " - [2/4] Starting Fleet Observability Dashboard on Port ${PORT_FLEET}..."
    python3 -u scripts/fleet_dashboard.py \
        --port "${PORT_FLEET}" \
        --fleet-size "${ROBOTS}" \
        --world "${CLEAN_WORLD}" > /tmp/nrdas_fleet_dashboard.log 2>&1 &
    FLEET_DASH_PID=$!
    PIDS+=("$FLEET_DASH_PID")

    echo " - [3/4] Starting Resilience & Fault Injection Dashboard on Port ${PORT_RESILIENCE}..."
    python3 -u scripts/resilience_dashboard.py \
        --port "${PORT_RESILIENCE}" \
        --fleet-size "${ROBOTS}" \
        --world "${CLEAN_WORLD}" > /tmp/nrdas_resilience_dashboard.log 2>&1 &
    RESILIENCE_DASH_PID=$!
    PIDS+=("$RESILIENCE_DASH_PID")
fi

# Step 4: Launch RViz 2 Visualization
if [ "$RVIZ" = "true" ]; then
    echo " - [4/4] Starting RViz 2 Fleet Visualization..."
    RVIZ_CONFIG="${WORKSPACE_ROOT}/install/amr_fleet_bringup/share/amr_fleet_bringup/rviz/fleet_default.rviz"
    if [ ! -f "$RVIZ_CONFIG" ]; then
        RVIZ_CONFIG="${WORKSPACE_ROOT}/src/amr_fleet_bringup/rviz/fleet_default.rviz"
    fi
    rviz2 -d "${RVIZ_CONFIG}" > /tmp/nrdas_rviz2.log 2>&1 &
    RVIZ_PID=$!
    PIDS+=("$RVIZ_PID")
fi

# Banner Display
cat << EOF

================================================================================
          YAVI-SIH26123: ALL-IN-ONE AMR FLEET AUTONOMY SUITE LAUNCHED
================================================================================
  World:                   ${CLEAN_WORLD}
  Fleet Size:              ${ROBOTS} AMRs (amr_0 .. amr_$((ROBOTS - 1)))
  Workload Schedule:       $(basename "${WORKLOAD}")
  Compute Policy:          ${COMPUTE_MODE}
  Communication Profile:   ${COMM_PROFILE}

  Active Visualizers:
  * Gazebo 3D World:       $([ "$HEADLESS" = "false" ] && echo "Running (3D GUI Window)" || echo "Running (Headless Server)")
  * RViz 2 Visualization:  $([ "$RVIZ" = "true" ] && echo "Running (fleet_default.rviz)" || echo "Disabled")

  Interactive Web Dashboards:
  * Fleet Observability:   http://localhost:${PORT_FLEET}
  * Resilience Console:    http://localhost:${PORT_RESILIENCE}

  Service Log Outputs:
  * ROS & Gazebo:          tail -f /tmp/nrdas_ros_launch.log
  * Fleet Dashboard:       tail -f /tmp/nrdas_fleet_dashboard.log
  * Resilience Dashboard:  tail -f /tmp/nrdas_resilience_dashboard.log
  * RViz 2:                tail -f /tmp/nrdas_rviz2.log

  >>> Press [Ctrl + C] in this terminal to gracefully terminate all services. <<<
================================================================================

EOF

# Keep script running and wait for launch process or interruption
wait "$ROS_LAUNCH_PID"

