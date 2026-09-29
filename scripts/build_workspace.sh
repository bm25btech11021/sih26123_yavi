#!/usr/bin/env bash
# Deterministic colcon build script for YAVI-SIH26123 AMR Fleet
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo " Building YAVI-SIH26123 AMR Fleet Workspace: $WORKSPACE_ROOT"
echo "============================================================"

cd "$WORKSPACE_ROOT"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # Sourcing ROS 2 Jazzy underlay
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u 2>/dev/null || true
fi

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

echo "============================================================"
echo " Workspace built successfully!"
echo " Source install/setup.bash to use packages."
echo "============================================================"

