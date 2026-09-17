#!/bin/bash
# Gazebo rover test runner with OpenVINS visualization.
# Defaults: auto-drive ON (circle), OpenVINS estimator ON, rviz2 ON, gazebo GUI ON.
#
# Driving modes are EXCLUSIVE (proven pattern): exactly one node owns /cmd_vel.
# A second writer (even an idle keyboard node spamming zeros) would fight it.
# Auto mode defaults to a gentle circle: VIO needs rotation plus
# translation at all times, and stop-turns in place starve it of parallax.
# Usage:
#   ./gazebo.sh [--auto|--no-auto] [--circle|--square] [--vins|--no-vins] [--rviz|--no-rviz] [--teleop|--teleop-only|--no-teleop] [--gui|--headless] [--dry-run]
#
# Examples:
#   ./gazebo.sh                          # circle + VIO + rviz + gui
#   ./gazebo.sh --square                 # old stop-turn square loop
#   ./gazebo.sh --teleop                 # arrow-key manual driving (auto off), VIO still runs
#   ./gazebo.sh --teleop-only            # same as --teleop
#   ./gazebo.sh --no-vins                # rover only, no estimator
#   ./gazebo.sh --headless --no-rviz     # server only (low resource / CI)
set -e

AUTO=true
MODE=circle
VINS=true
RVIZ=true
GUI=true
TELEOP=false
DRYRUN=false

usage() {
  sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# //; s/^#//'
}

for arg in "$@"; do
  case "$arg" in
    --auto) AUTO=true ;;
    --no-auto) AUTO=false ;;
    --circle) MODE=circle ;;
    --square) MODE=square ;;
    --vins) VINS=true ;;
    --no-vins) VINS=false ;;
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
# ROVER_DOMAIN_ID=8 ./gazebo.sh to rejoin the default domain.
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

LAUNCH_CMD="ros2 launch ov_rover_sim rover_sim.launch.py auto:=$AUTO mode:=$MODE rviz:=$RVIZ gui:=$GUI"
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
  # Let the X server itself pick a free display number atomically
  # (-displayfd). Scanning lock files races when two sessions start at
  # once: both grab :100, the loser dies, and later the winner's cleanup
  # kills the display out from under the loser (fatal XIO mid-run).
  local dispfile="/tmp/xvfb_display_$$.txt"
  rm -f "$dispfile"
  Xvfb -displayfd 3 -screen 0 1280x1024x24 3>"$dispfile" &
  XVFB_PID=$!
  local i d
  for i in $(seq 1 50); do
    [ -s "$dispfile" ] && break
    sleep 0.1
    kill -0 "$XVFB_PID" 2>/dev/null || break
  done
  d=$(cat "$dispfile" 2>/dev/null)
  rm -f "$dispfile"
  if [ -n "$d" ] && kill -0 "$XVFB_PID" 2>/dev/null \
      && xdpyinfo -display ":$d" > /dev/null 2>&1; then
    export DISPLAY=":$d"
    echo "Started Xvfb on $DISPLAY (pid $XVFB_PID) for headless camera rendering."
    return 0
  fi
  kill "$XVFB_PID" 2>/dev/null || true
  unset XVFB_PID
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

VINS_CONFIG="$WS/src/open_vins/ov_rover_sim/config/rover_stereo/estimator_config.yaml"

echo "WS:     $WS"
echo "Launch: $LAUNCH_CMD"
echo "Teleop: $TELEOP (exclusive: auto is off while teleop runs)"
echo "Vins:   $VINS (config: $VINS_CONFIG)"

if [ "$DRYRUN" = true ]; then
  echo "(dry-run, not launching)"
  $TELEOP && echo "would also run: $TELEOP_CMD" || true
  $VINS && echo "would also run: ros2 run ov_msckf run_subscribe_msckf \"$VINS_CONFIG\" --ros-args -r __ns:=/ov_msckf" || true
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
  # align_frames.py normally exits on its own, but a mid-run Ctrl+C must
  # still take it (and everything else) down so no zombie keeps the
  # domain's topics alive after the terminal is gone.
  pkill -f "auto_loop\.py|key_teleop\.py|align_frames\.py|run_subscribe_msckf" 2>/dev/null || true
  pkill -f "rviz2.*\.rviz" 2>/dev/null || true
  # gzserver ignores SIGTERM: escalate what is still ours, then SIGKILL it.
  sleep 2
  [ -n "${XVFB_PID:-}" ] && kill "$XVFB_PID" 2>/dev/null || true
  pkill -INT -f "small_room.world" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "small_room.world" 2>/dev/null || true
}
trap cleanup INT TERM HUP EXIT

# Start simulation in background
$LAUNCH_CMD 2>&1 | tee /tmp/gazebo_test.log &
LAUNCH_PID=$!

# Start the OpenVINS estimator on the rover stream (background). It needs
# the sim up first (topics live; the rover holds still 15 s sim for clean
# contact AND for a static estimator init). Start the estimator EARLY
# (12 s wall) so it is up during the stillness: with no jerk-wait
# (init_imu_thresh 0) it statically initializes on the still data
# (exact zero velocity, gravity, stillness biases) instead of gambling
# a dynamic init on planar motion (which dumps yaw rate into gyro bias
# and fits rotation as sideways velocity). Driving starts after.
# Namespace matches the ov_msckf launch file, so topics land under /ov_msckf/.
if [ "$VINS" = true ]; then
  if [ ! -f "$VINS_CONFIG" ]; then
    echo "ERROR: estimator config not found: $VINS_CONFIG" >&2
    exit 1
  fi
    echo "Waiting for the sim to come up, then starting OpenVINS..."
    sleep 12
    ros2 run ov_msckf run_subscribe_msckf "$VINS_CONFIG" --ros-args -r __ns:=/ov_msckf \
      -p use_sim_time:=true > /tmp/ov_msckf.log 2>&1 &
  VINS_PID=$!
  echo "OpenVINS running (log: /tmp/ov_msckf.log, topics under /ov_msckf/)."
fi

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
  echo "Running. auto=$AUTO mode=$MODE vins=$VINS rviz=$RVIZ gui=$GUI"
  echo "Manual override anytime: $TELEOP_CMD"
  wait $LAUNCH_PID
fi
