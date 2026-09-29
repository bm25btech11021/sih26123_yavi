#!/usr/bin/env bash
# Test execution and validation script for YAVI-SIH26123 AMR Fleet
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo " Testing YAVI-SIH26123 AMR Fleet Packages: $WORKSPACE_ROOT"
echo "============================================================"

cd "$WORKSPACE_ROOT"

set +u
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi
set -u 2>/dev/null || true

colcon test --event-handlers console_direct+

echo "============================================================"
echo " Test Results Summary:"
echo "============================================================"
colcon test-result --all --verbose

