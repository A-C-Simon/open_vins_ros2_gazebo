#!/bin/bash
# Gazebo rover test runner with visualization.
# Defaults: auto-drive ON (teleop can override), rviz2 ON, gazebo GUI ON.
#
# Driving modes are EXCLUSIVE (proven pattern): exactly one node owns /cmd_vel.
# A second writer (even an idle keyboard node spamming zeros) would fight it.
# Usage:
#   ./gazebo_test.sh [--auto|--no-auto] [--rviz|--no-rviz] [--teleop|--teleop-only|--no-teleop] [--gui|--headless] [--dry-run]
#
# Examples:
#   ./gazebo_test.sh                          # auto-drive + rviz + gui
#   ./gazebo_test.sh --teleop                 # arrow-key manual driving (auto off)
#   ./gazebo_test.sh --teleop-only            # same as --teleop
#   ./gazebo_test.sh --headless --no-rviz     # server only (low resource / CI)
set -e

AUTO=true
RVIZ=true
GUI=true
TELEOP=false
DRYRUN=false

usage() {
  sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# //; s/^#//'
}

for arg in "$@"; do
  case "$arg" in
    --auto) AUTO=true ;;
    --no-auto) AUTO=false ;;
    --rviz|--rviz2) RVIZ=true ;;
    --no-rviz|--no-rviz2) RVIZ=false ;;
    --teleop|--teleop-only) AUTO=false; TELEOP=true ;;
    --no-teleop) TELEOP=false ;;
    --gui) GUI=true ;;
    --headless|--no-gui) GUI=false ;;
    --dry-run) DRYRUN=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $arg (see --help)" >&2; exit 1 ;;
  esac
done

# Resolve workspace root: ascend from script location to the first dir that
# looks like a workspace (has install/setup.bash AND a src/ subdir).
# The src/ check avoids stray install trees (e.g. src/install).
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

# Drive traffic lives on its own ROS domain by default. The default domain
# on this machine is shared by every sim and keyboard node, and any stray
# /cmd_vel writer there (even an idle keyboard spamming zeros) wins
# intermittently and freezes the rover. Override with e.g.
# ROVER_DOMAIN_ID=8 ./gazebo_test.sh to rejoin the default domain.
if [ -z "${ROVER_DOMAIN_ID:-}" ]; then
  ROVER_DOMAIN_ID=42
fi
export ROS_DOMAIN_ID="$ROVER_DOMAIN_ID"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (rover traffic isolated; same export is needed"
echo "in any other terminal that talks to the rover, e.g.:"
echo "  export ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"

source /opt/ros/humble/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "ERROR: $WS/install/setup.bash missing. Build first." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

# Make sure our package is built (cheap if already built)
if [ ! -d "$WS/install/ov_rover_sim" ]; then
  echo "ov_rover_sim not built, building..."
  colcon build --packages-select ov_rover_sim --symlink-install
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

LAUNCH_CMD="ros2 launch ov_rover_sim rover_sim.launch.py auto:=$AUTO rviz:=$RVIZ gui:=$GUI"
TELEOP_CMD="ros2 run ov_rover_sim key_teleop.py"

echo "WS:     $WS"
echo "Launch: $LAUNCH_CMD"
echo "Teleop: $TELEOP (exclusive: auto is off while teleop runs)"

if [ "$DRYRUN" = true ]; then
  echo "(dry-run, not launching)"
  $TELEOP && echo "would also run: $TELEOP_CMD" || true
  exit 0
fi

# Pre-flight: /cmd_vel must be free. A foreign writer (forgotten keyboard
# node, another sim in the same ROS domain) wins intermittently and the
# rover stutters or freezes while everything looks fine.
echo "Pre-flight: checking /cmd_vel is free..."
BUSY=$(python3 -c "
import rclpy
from rclpy.node import Node
rclpy.init()
n = Node('preflight')
import time
found = set()
t0 = time.time()
while time.time() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        for i in n.get_publishers_info_by_topic('/cmd_vel'):
            if i.node_name not in ('preflight', '_NODE_NAME_UNKNOWN_'):
                found.add(i.node_name)
    except Exception:
        pass
n.destroy_node()
rclpy.shutdown()
print(' '.join(sorted(found)))
" 2>/dev/null)
if [ -n "$BUSY" ]; then
  echo "WARNING: /cmd_vel already has publishers: $BUSY" >&2
  echo "Another session (or a stale keyboard/auto node) owns the drive topic." >&2
  echo "Kill it (or give each setup its own ROS_DOMAIN_ID) or the rover will fight it." >&2
  read -r -p "Continue anyway? [y/N] " yn
  case "$yn" in
    [Yy]*) ;;
    *) echo "Aborted."; exit 1 ;;
  esac
else
  echo "/cmd_vel is free."
fi

cleanup() {
  echo ""
  echo "Shutting down gazebo test..."
  jobs -p | xargs -r kill 2>/dev/null || true
  # rviz2 included: Ctrl+C must take down everything, including separately started rviz2.
  # Also kill stale drivers/teleops: a forgotten keyboard node spamming zero
  # /cmd_vel vetoes every other driver on this ROS domain (last-writer-wins).
  pkill -f "gzserver|gzclient|rviz2" 2>/dev/null || true
  pkill -f "auto_loop\.py|key_teleop\.py|teleop_twist_keyboard" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# Start simulation in background
$LAUNCH_CMD 2>&1 | tee /tmp/gazebo_test.log &
LAUNCH_PID=$!

if [ "$TELEOP" = true ]; then
  echo ""
  echo "Arrow-key teleop active (exclusive /cmd_vel owner)."
  echo "Focus this terminal and use arrow keys. Ctrl+C quits everything."
  sleep 3
  # foreground so keyboard input works; auto yields via teleop_timeout
  # shellcheck disable=SC2086
  $TELEOP_CMD
else
  echo ""
  echo "Running. auto=$AUTO rviz=$RVIZ gui=$GUI"
  echo "Manual override anytime: $TELEOP_CMD"
  wait $LAUNCH_PID
fi
