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

# Stale-server guard: a leftover gzserver fights the new run over the
# gazebo master port and ROS topics. Refuse to start dirty (never kill
# other sessions' servers for them; print how instead).
if pgrep -x gzserver > /dev/null; then
  echo "ERROR: a gzserver process is already running - refusing dirty start." >&2
  echo "Inspect: pgrep -af gzserver" >&2
  echo "If it is a leftover, kill it yourself: pkill -x gzserver" >&2
  exit 1
fi

# Headless cameras need an X server to render into. With no display,
# Gazebo camera sensors silently produce nothing, so start Xvfb.
have_display() {
  [ -n "${DISPLAY:-}" ] || return 1
  command -v xdpyinfo > /dev/null 2>&1 || return 0
  xdpyinfo -display "$DISPLAY" > /dev/null 2>&1
}
start_xvfb() {
  # A display whose lock exists but has no live server behind it is stale
  # (crashed runs leave them); only a live server blocks reuse.
  local d
  for d in $(seq 99 130); do
    if [ -e "/tmp/.X${d}-lock" ] && pgrep -f "Xvfb :${d} " > /dev/null; then
      continue
    fi
    rm -f "/tmp/.X${d}-lock" "/tmp/.X11-unix/X${d}"
    Xvfb ":$d" -screen 0 1280x1024x24 &
    XVFB_PID=$!
    sleep 1
    if kill -0 "$XVFB_PID" 2>/dev/null && xdpyinfo -display ":$d" > /dev/null 2>&1; then
      export DISPLAY=":$d"
      echo "Started Xvfb on $DISPLAY (pid $XVFB_PID) for headless camera rendering."
      return 0
    fi
    kill "$XVFB_PID" 2>/dev/null || true
    unset XVFB_PID
  done
  echo "WARNING: no working X display; cameras will not render headless." >&2
  return 1
}
if [ "$GUI" = false ] && ! have_display; then
  if command -v Xvfb > /dev/null && command -v xdpyinfo > /dev/null; then
    start_xvfb || true
  else
    echo "WARNING: no X server and no Xvfb/xdpyinfo; cameras will not render headless." >&2
  fi
fi

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

set -m # job control, so `jobs -p` also works when stdin is not a terminal
cleanup() {
  # run once (the EXIT trap would otherwise repeat the INT trap's work)
  [ -n "${CLEANED:-}" ] && return 0
  CLEANED=1
  echo ""
  echo "Shutting down gazebo test..."
  # precise kill first: the whole launch process group. That takes down
  # ros2 launch itself plus robot_state_publisher and every node it
  # started, without touching anyone else's processes.
  if [ -n "${LAUNCH_PID:-}" ]; then
    LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')
    [ -n "$LAUNCH_PGID" ] && kill -- "-$LAUNCH_PGID" 2>/dev/null || true
  fi
  jobs -p | xargs -r kill 2>/dev/null || true
  # Kill only OUR processes (our world file / our node scripts). Never use
  # broad patterns here: another session's servers share this machine and
  # killing them (or their X servers) breaks that session's cameras/sim.
  # Stale drivers/teleops of ours: a forgotten keyboard node spamming zero
  # /cmd_vel vetoes every other driver on this ROS domain (last-writer-wins).
  pkill -f "auto_loop\.py|key_teleop\.py" 2>/dev/null || true
  pkill -f "rviz2.*\.rviz" 2>/dev/null || true
  # gzserver ignores SIGTERM: escalate what is still ours, then SIGKILL it.
  sleep 2
  [ -n "${XVFB_PID:-}" ] && kill "$XVFB_PID" 2>/dev/null || true
  pkill -INT -f "small_room.world" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "small_room.world" 2>/dev/null || true
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
  # foreground so keyboard input works (exclusive mode: auto is off)
  # shellcheck disable=SC2086
  $TELEOP_CMD
else
  echo ""
  echo "Running. auto=$AUTO rviz=$RVIZ gui=$GUI"
  echo "Manual override anytime: $TELEOP_CMD"
  wait $LAUNCH_PID
fi
