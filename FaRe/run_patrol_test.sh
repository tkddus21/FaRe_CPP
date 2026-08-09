#!/usr/bin/env bash
# Runs one patrol and captures everything it produced into its own results/testN
# directory, so repeated runs can be compared instead of overwriting each other.
#
# Expects the world, the robot and the navigation stack to already be running
# (see README "Steps for Online Navigation"). This script only owns the
# recording and the patrol itself.
#
# Runs are named <date>_<time>_<label> so they sort chronologically, never
# collide, and say what was being tested:
#
#   ./run_patrol_test.sh                     # -> results/20260729_0130_aws_waffle_pi
#   FARE_MAP=house ./run_patrol_test.sh      # -> results/turtlebot3_house/20260729_0130_house_waffle_pi
#   ./run_patrol_test.sh infl03              # -> results/20260729_0130_infl03
#   RECORD_SCAN=1 ./run_patrol_test.sh       # also record /scan (much larger bag)
set -uo pipefail

FARE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Topics worth keeping. costmap and costmap_updates go together: costmap_2d
# publishes the full grid once and then only sends deltas, so without the
# updates topic a playback shows the costmap frozen at its initial state.
TOPICS=(
  /map
  /amcl_pose
  /odom
  /tf
  /tf_static
  /move_base/status
  /move_base/current_goal
  /move_base/NavfnROS/plan
  /move_base/DWAPlannerROS/global_plan
  /move_base/DWAPlannerROS/local_plan
  /move_base/global_costmap/costmap
  /move_base/global_costmap/costmap_updates
  /move_base/local_costmap/costmap
  /move_base/local_costmap/costmap_updates
)
[ "${RECORD_SCAN:-0}" = "1" ] && TOPICS+=(/scan)

die() { echo "ERROR: $*" >&2; exit 1; }

# --- pre-flight ------------------------------------------------------------
# Everything that can abort the run is checked before the run directory is
# created, so a failed attempt does not burn a test number.
# Each of these has cost us a wasted run at some point, so check up front
# rather than discovering the problem 12 minutes in.

# The map is chosen by FARE_MAP (see FaRe/config.py), and each map keeps its own
# output_dir. Read it from the config rather than assuming results/, so a house
# run cannot drive the AWS run's waypoints or overwrite its artefacts.
CFG="$(python3 -c "
import sys
sys.path.insert(0, '$FARE_DIR')
from config import config, MAP_NAME
print(config['output_dir'])
print(MAP_NAME)
print(config['yaml_filename'])
")" || die "could not read FaRe/config.py (bad FARE_MAP?)"
{ read -r RESULTS_DIR; read -r FARE_MAP_NAME; read -r MAP_YAML; } <<< "$CFG"

rosnode list 2>/dev/null | grep -q '/move_base' \
  || die "move_base is not running - start the navigation stack first"
[ -f "$RESULTS_DIR/wp_ori_data.txt" ] \
  || die "no waypoints at $RESULTS_DIR/wp_ori_data.txt - run Surveillance.py first"

echo "==> FARE_MAP=$FARE_MAP_NAME  ($MAP_YAML)"

MODEL="${TURTLEBOT3_MODEL:-unset}"
echo "==> TURTLEBOT3_MODEL=$MODEL"

# AMCL must actually be localised. A diverged filter produces a run that looks
# like a navigation failure but is really a measurement failure.
echo "==> checking AMCL against Gazebo ground truth"
python3 - "$MODEL" <<'PY' || die "AMCL check failed - re-run set_initial_pose.py"
import math, sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState

model = f"turtlebot3_{sys.argv[1]}"
rospy.init_node('amcl_precheck', anonymous=True, disable_signals=True)
try:
    rospy.wait_for_service('/gazebo/get_model_state', timeout=15)
    gt = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(model, '')
    est = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=20)
except Exception as exc:
    print(f"    could not read poses: {exc}")
    sys.exit(1)

err = math.hypot(est.pose.pose.position.x - gt.pose.position.x,
                 est.pose.pose.position.y - gt.pose.position.y)
print(f"    ground truth ({gt.pose.position.x:.3f}, {gt.pose.position.y:.3f})  "
      f"AMCL ({est.pose.pose.position.x:.3f}, {est.pose.pose.position.y:.3f})  "
      f"error {err:.3f} m")
sys.exit(0 if err < 0.5 else 1)
PY

# --- pick the run directory ------------------------------------------------
# <date>_<time>_<label>, e.g. 20260729_0130_aws_waffle_pi. The timestamp sorts
# runs chronologically and can never collide; the label says what was being
# tested, which a bare test1/test2 does not. Defaults to map + robot model, the
# two things that usually differ between runs - with more than one map in play,
# a directory named just "waffle_pi" no longer identifies the run.
LABEL="${1:-${FARE_MAP_NAME}_${MODEL}}"
LABEL="${LABEL//[^A-Za-z0-9._-]/_}"
RUN_DIR="$RESULTS_DIR/$(date '+%Y%m%d_%H%M')_$LABEL"
[ -e "$RUN_DIR" ] && die "$RUN_DIR already exists; wait a minute or pass another label"
mkdir -p "$RUN_DIR" || die "cannot create $RUN_DIR"
echo "==> run directory: $RUN_DIR"

# --- record ----------------------------------------------------------------
BAG="$RUN_DIR/patrol.bag"
echo "==> recording ${#TOPICS[@]} topics -> $BAG"
rosbag record -O "$BAG" "${TOPICS[@]}" >"$RUN_DIR/rosbag.log" 2>&1 &
BAG_PID=$!
sleep 8   # let subscriptions establish before the first latched messages fly by
kill -0 "$BAG_PID" 2>/dev/null || die "rosbag record died; see $RUN_DIR/rosbag.log"

# --- patrol ----------------------------------------------------------------
echo "==> running PatrolSim"
( cd "$FARE_DIR/.." && python3 -u FaRe/PatrolSim.py ) 2>&1 | tee "$RUN_DIR/patrol_console.log"

# --- stop recording --------------------------------------------------------
# SIGINT, never SIGKILL: rosbag writes its index on shutdown, and a killed bag
# is unindexed and may be unplayable.
echo "==> stopping rosbag (SIGINT)"
kill -INT "$BAG_PID" 2>/dev/null
for _ in $(seq 1 40); do
  [ -f "$BAG" ] && break
  sleep 2
done
if [ -f "$BAG" ]; then
  echo "==> bag closed: $(du -h "$BAG" | cut -f1)"
else
  echo "WARNING: $BAG not finalised; check for a .active file and run 'rosbag reindex'" >&2
fi

# --- collect the artefacts this run produced -------------------------------
for f in patrol_log.csv wp_ori_data.txt metrics.csv waypoint_clearance.csv \
         coverage_map.png coverage_grid.npy path.png; do
  [ -f "$RESULTS_DIR/$f" ] && cp "$RESULTS_DIR/$f" "$RUN_DIR/"
done

# --- record the conditions, so the numbers stay interpretable later --------
{
  echo "date              : $(date '+%F %T')"
  echo "TURTLEBOT3_MODEL  : $MODEL"
  echo "FARE_MAP          : $FARE_MAP_NAME"
  echo "map               : $MAP_YAML"
  echo "recorded topics   : ${#TOPICS[@]}"
  for p in footprint footprint_padding; do
    echo "$p : $(rosparam get "/move_base/global_costmap/$p" 2>/dev/null)"
  done
  for p in inflation_radius cost_scaling_factor; do
    echo "$p : $(rosparam get "/move_base/global_costmap/inflation_layer/$p" 2>/dev/null)"
  done
  echo "max_vel_x         : $(rosparam get /move_base/DWAPlannerROS/max_vel_x 2>/dev/null)"
  echo "git               : $(git -C "$FARE_DIR" rev-parse --short HEAD 2>/dev/null)"
} > "$RUN_DIR/run_info.txt"

# --- summarise -------------------------------------------------------------
if [ -f "$RUN_DIR/patrol_log.csv" ]; then
  python3 - "$RUN_DIR/patrol_log.csv" <<'PY' | tee -a "$RUN_DIR/run_info.txt"
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
ok = sum(1 for r in rows if r['status'] == 'SUCCEEDED')
print(f"result            : {ok}/{len(rows)} SUCCEEDED")
for r in rows:
    if r['status'] != 'SUCCEEDED':
        print(f"  failed #{r['index']} ({r['world_x']}, {r['world_y']}) "
              f"{r['status']} {r['duration_s']}s")
PY
fi

echo
echo "==> done. everything for this run is in $RUN_DIR"
echo "    replay:  rosbag play --clock -d 5 -r 3 $BAG"
