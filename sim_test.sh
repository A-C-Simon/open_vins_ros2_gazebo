#!/bin/bash
# One-command OpenVINS simulation + RViz visualization.
# Usage:
#   ./sim_test.sh [path/to/estimator_config.yaml]
# Run from anywhere, the script cds to the workspace root itself.
set -e

# Find workspace root: ascend to first dir with install/setup.bash AND src/.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=""
D="$SCRIPT_DIR"
while [ "$D" != "/" ]; do
  if [ -f "$D/install/setup.bash" ] && [ -d "$D/src" ]; then
    WS="$D"
    break
  fi
  D="$(dirname "$D")"
done
WS="${WS:-/home/ac/ros2_ws}"
cd "$WS"

# 1. Environment
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  echo "ERROR: $WS/install/setup.bash not found. Did you 'colcon build'?" >&2
  exit 1
fi

# 2. Config (default = rpng_sim, override with $1)
CONFIG="${1:-src/open_vins/config/rpng_sim/estimator_config.yaml}"
if [ ! -f "$CONFIG" ]; then
  echo "ERROR: config not found: $CONFIG" >&2
  exit 1
fi

# 3. RViz config (installed share first, source fallback)
RVIZ_CONFIG="$WS/install/ov_msckf/share/ov_msckf/launch/display_ros2.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
  RVIZ_CONFIG="$WS/src/open_vins/ov_msckf/launch/display_ros2.rviz"
fi
if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "WARNING: rviz config not found, running simulation without rviz." >&2
  RVIZ_CONFIG=""
fi

# 4. Simulation binary (build tree first, ros2 run fallback)
SIM_BIN="$WS/build/ov_msckf/run_simulation"
if [ ! -x "$SIM_BIN" ]; then
  SIM_BIN="ros2 run ov_msckf run_simulation"
  USE_ROS2_RUN=1
else
  USE_ROS2_RUN=0
fi

echo "WS:     $WS"
echo "Config: $CONFIG"
echo "Rviz:   ${RVIZ_CONFIG:-<disabled>}"

cleanup() {
  echo ""
  echo "Shutting down..."
  jobs -p | xargs -r kill 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# 5. Launch simulation + rviz in parallel
if [ "$USE_ROS2_RUN" -eq 1 ]; then
  # shellcheck disable=SC2086
  $SIM_BIN "$CONFIG" &
else
  "$SIM_BIN" "$CONFIG" &
fi
SIM_PID=$!

if [ -n "$RVIZ_CONFIG" ]; then
  sleep 2  # let sim node start publishing first
  rviz2 -d "$RVIZ_CONFIG" --ros-args --log-level warn &
  RVIZ_PID=$!
fi

wait $SIM_PID
