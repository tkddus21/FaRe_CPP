#!/usr/bin/env python3
"""Drives the robot around until gmapping has seen the whole house.

Replaces the teleop pass in the README's mapping procedure. It picks frontiers -
free cells that touch unmapped space - off gmapping's live /map and sends the
best one to move_base, until none are left.

This is the one node turtlebot3_slam/launch/turtlebot3_frontier_exploration.launch
would have supplied. That launch file needs the frontier_exploration package,
which was never released for Noetic; everything else it starts is in
launch/house_mapping.launch.

Two things here exist because of how FaRe reads a map afterwards:

  * The goal sent for a frontier is not the frontier itself but the nearest cell
    with enough clearance for the costmap footprint. Frontiers hug walls and
    doorways, so a frontier cell is rarely somewhere move_base can park, and
    every unreachable goal costs a timeout plus a recovery.

  * After every reached goal the robot spins in place. gmapping's maxUrange is
    3.0 m, so a room wider than that keeps unmapped area in its middle that no
    wall-following pass ever fills in. The README warns about exactly this:
    diagnose_waypoints.py treats unmapped area as a barrier, so waypoints are
    never placed there.

Run house_sim.launch and house_mapping.launch first, then:

    python3 FaRe/explore_for_mapping.py

Then save the map where the "house" preset in FaRe/config.py expects it:

    rosrun map_server map_saver -f ~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map
"""

import argparse
import ast
import math
import time

import numpy as np
import rospy
import actionlib
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import binary_dilation, distance_transform_edt, label
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

# Deliberately does not import FaRe/config.py. That module resolves the map it is
# configured for at import time and exits if the map is missing - which is the
# state this script exists to get out of.

# OccupancyGrid convention: -1 unknown, 0 free, 100 occupied. gmapping only ever
# emits those three, but threshold anyway so this survives a different SLAM node.
UNKNOWN = -1
OCCUPIED_FROM = 65

FALLBACK_CLEARANCE = 0.28  # metres, waffle_pi circumscribed radius plus padding

# 4 cells at 0.05 m is a 0.2 m opening. Small, deliberately: a doorway seen edge-on
# shows up as a handful of cells, and dropping it strands the room behind it.
MIN_CLUSTER_CELLS = 4
GOAL_SEARCH_RADIUS = 1.5  # metres to look outward from a frontier for a standable cell
# Shorter than PatrolSim's 120 s, but not by much. 60 s was tried and cost more than
# it saved: goals across the house legitimately need longer, and every one cut short
# counts as a failure and burns an attempt against a frontier that was fine.
GOAL_TIMEOUT = 90.0
BLACKLIST_RADIUS = 0.6   # metres; a failed frontier poisons its neighbourhood too
BLACKLIST_COOLDOWN = 120.0   # seconds before a failed frontier is worth another try
# Generous, because a frontier that fails is usually telling you about the robot's
# current position rather than about itself, and the map keeps changing under it.
# At 3 a run ended at 44.3 sq.m: the three clusters visible at that moment had each
# failed three times, which says nothing about the rooms still behind them.
MAX_FRONTIER_ATTEMPTS = 6
DONE_AFTER_EMPTY = 3     # consecutive empty scans before calling it finished

STALL_DISTANCE = 0.02    # metres of travel that still counts as moving
STALL_SECONDS = 20.0     # give up on a goal after this long without moving
MAX_HARD_STUCK = 4       # consecutive failed recoveries before abandoning the run

SPIN_SPEED = 0.8         # rad/s
ESCAPE_SPEED = 0.10      # m/s, signed at use; faster than PatrolSim's 0.08 because
ESCAPE_TIME = 3.0        # seconds - a wedge needs a shove, not a nudge
FREED_DISTANCE = 0.05    # metres; less than this and the escape did not work

STATUS_NAMES = {v: k for k, v in vars(GoalStatus).items()
                if isinstance(v, int) and k.isupper()}


class MapView:
    """Latest /map, held as a numpy grid with the metadata needed to index it."""

    def __init__(self):
        self.grid = None
        self.info = None
        rospy.Subscriber('/map', OccupancyGrid, self._cb, queue_size=1)

    def _cb(self, msg):
        # OccupancyGrid is row-major from the origin cell, and the origin is the
        # bottom-left corner - no vertical flip here. map_saver adds the flip when
        # it writes the pgm, which is why PatrolSim has to undo one and this
        # does not.
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self.info = msg.info

    def cell_to_world(self, row, col):
        return (self.info.origin.position.x + (col + 0.5) * self.info.resolution,
                self.info.origin.position.y + (row + 0.5) * self.info.resolution)

    def counts(self):
        return (int((self.grid == UNKNOWN).sum()),
                int((self.grid >= 0).sum() - (self.grid >= OCCUPIED_FROM).sum()),
                int((self.grid >= OCCUPIED_FROM).sum()))


def min_clearance():
    """How much room a frontier needs, taken from the costmap move_base is using.

    The circumscribed radius, not the inscribed one: a goal is a pose the robot
    has to be able to turn on the spot at, and the footprint is not a circle.
    Read from the parameter server rather than FaRe/config.py so this reflects
    the footprint actually in force, padding included.
    """
    footprint = rospy.get_param('/move_base/global_costmap/footprint', None)
    # costmap_2d writes the footprint back as its string form, not as the nested
    # list the yaml declared, so this has to survive both.
    if isinstance(footprint, str):
        try:
            footprint = ast.literal_eval(footprint)
        except (ValueError, SyntaxError):
            footprint = None
    if not footprint:
        rospy.logwarn(f"no usable footprint param, assuming {FALLBACK_CLEARANCE} m")
        return FALLBACK_CLEARANCE
    padding = rospy.get_param('/move_base/global_costmap/footprint_padding', 0.0)
    return max(math.hypot(vx, vy) for vx, vy in footprint) + padding


def find_frontier_clusters(view, clearance_m):
    """Reachable frontier clusters as (size, world_x, world_y).

    A frontier cell is free and orthogonally adjacent to unmapped space.

    Where the frontier is and where the robot can stand to see it are two
    different questions, and conflating them is why an earlier version of this
    ran out of goals after one room. A frontier is a boundary: it hugs walls and
    threads through half-seen doorways, so almost every frontier cell is within a
    footprint radius of something. Filtering the cells themselves by clearance
    left clusters of 8 cells where 12 were needed, and exploration stopped with
    the house 44 sq.m mapped.

    So cluster the raw frontier, then for each cluster look outward for the
    nearest cell the robot can actually occupy. The frontier says where to look;
    that cell is where to stand.
    """
    grid, res = view.grid, view.info.resolution

    free = grid == 0
    unknown = grid == UNKNOWN
    obstacle = grid >= OCCUPIED_FROM

    cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
    frontier = free & binary_dilation(unknown, structure=cross)
    if not frontier.any():
        return []

    clearance = distance_transform_edt(~obstacle) * res
    standable = free & (clearance >= clearance_m)
    if not standable.any():
        return []
    stand_rows, stand_cols = np.nonzero(standable)

    search_cells = GOAL_SEARCH_RADIUS / res
    labelled, count = label(frontier, structure=np.ones((3, 3), dtype=bool))
    clusters = []
    for i in range(1, count + 1):
        rows, cols = np.nonzero(labelled == i)
        if rows.size < MIN_CLUSTER_CELLS:
            continue
        cr, cc = rows.mean(), cols.mean()
        d2 = (stand_rows - cr) ** 2 + (stand_cols - cc) ** 2
        nearest = int(np.argmin(d2))
        if d2[nearest] > search_cells ** 2:
            continue  # nothing to stand on near this frontier; leave it
        x, y = view.cell_to_world(stand_rows[nearest], stand_cols[nearest])
        clusters.append((rows.size, x, y))
    return clusters


def blacklist_entry(blacklist, x, y):
    """The blacklist record covering (x, y), or None."""
    for entry in blacklist:
        if math.hypot(x - entry['x'], y - entry['y']) < BLACKLIST_RADIUS:
            return entry
    return None


def record_failure(blacklist, x, y):
    entry = blacklist_entry(blacklist, x, y)
    if entry:
        entry['fails'] += 1
        entry['when'] = time.time()
    else:
        blacklist.append({'x': x, 'y': y, 'fails': 1, 'when': time.time()})


def pick_goal(clusters, robot_xy, blacklist):
    """Best frontier: big, and close. Returns None when nothing is worth trying now.

    Dividing by distance rather than subtracting it keeps the choice scale-free,
    so a big room across the house still wins over a scrap in the next doorway,
    but only while it stays big.

    A failed frontier is set aside for BLACKLIST_COOLDOWN and then retried, up to
    MAX_FRONTIER_ATTEMPTS. Retrying matters because most failures are not a
    property of the frontier: move_base plans from wherever the robot happens to
    be, and the same goal often succeeds from the next room. Permanently
    blacklisting on the first abort is what ended one run at 81.8 sq.m with eight
    frontiers still open.
    """
    rx, ry = robot_xy
    now = time.time()
    best, best_score = None, 0.0
    for size, x, y in clusters:
        entry = blacklist_entry(blacklist, x, y)
        if entry and (entry['fails'] >= MAX_FRONTIER_ATTEMPTS
                      or now - entry['when'] < BLACKLIST_COOLDOWN):
            continue
        dist = math.hypot(x - rx, y - ry)
        score = size / (dist + 1.0)
        if score > best_score:
            best, best_score = (x, y), score
    return best


def all_exhausted(clusters, blacklist):
    """True only when every remaining frontier has been given up on for good.

    The distinction this draws is the point: "nothing selectable right now"
    because failures are cooling down is not the same as "the house is mapped",
    and treating them the same ends the run with walls still open.
    """
    return all(
        (lambda e: e is not None and e['fails'] >= MAX_FRONTIER_ATTEMPTS)(
            blacklist_entry(blacklist, x, y))
        for _, x, y in clusters)


def spin_in_place(cmd_vel, turns=1.0):
    """One full rotation under direct velocity control.

    move_base has no "look around here" action, and its rotate recovery refuses
    to run when it thinks a collision is possible - which is common in the tight
    spots where the extra look matters most.
    """
    twist = Twist()
    twist.angular.z = SPIN_SPEED
    rate = rospy.Rate(10)
    for _ in range(int(turns * 2 * math.pi / SPIN_SPEED * 10)):
        if rospy.is_shutdown():
            break
        cmd_vel.publish(twist)
        rate.sleep()
    cmd_vel.publish(Twist())


def scan_clearance(deg_from, deg_to):
    """Closest laser return in a sector, in metres. 0 degrees is straight ahead."""
    try:
        scan = rospy.wait_for_message('/scan', LaserScan, timeout=2.0)
    except rospy.ROSException:
        return None
    ranges = np.asarray(scan.ranges, dtype=float)
    ranges[~np.isfinite(ranges)] = scan.range_max
    n = len(ranges)
    idx = [int(round((math.radians(a) - scan.angle_min) / scan.angle_increment)) % n
           for a in range(deg_from, deg_to)]
    return float(ranges[idx].min())


def recover(client, cmd_vel, listener):
    """Drive out of a wedge, in whichever direction there is room to go.

    PatrolSim reverses unconditionally, which is right for its failure: a patrol
    drives forwards into a pinch. Exploration wedges the other way round just as
    often - goals sit at the edge of known space, so the robot backs into corners
    it has never seen - and reversing into what is already touching the bumper
    just grinds. Measured on a real wedge here: 0.17 m of rear clearance against
    a 0.205 m rear overhang, 5 s of reversing bought 3 cm, and one second
    forwards freed it.

    So ask the laser which way is open and go that way, then confirm against tf
    that the robot actually moved rather than trusting the command.
    """
    client.cancel_all_goals()
    try:
        rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
    except rospy.ServiceException as exc:
        rospy.logwarn(f"clear_costmaps failed: {exc}")

    front = scan_clearance(-40, 40)
    rear = scan_clearance(140, 220)
    # Compare against the overhang each way, not against each other: the
    # footprint is not symmetric (front +0.077, rear -0.205 on a waffle_pi).
    speed = ESCAPE_SPEED
    if front is not None and rear is not None and (rear - 0.205) > (front - 0.077):
        speed = -ESCAPE_SPEED
    rospy.loginfo(f"recovery: front {front} m, rear {rear} m -> "
                  f"{'forward' if speed > 0 else 'reverse'}")

    rate = rospy.Rate(10)

    def drive(linear, angular, seconds):
        """Hold a velocity for a while and report how far the robot actually got."""
        start = robot_pose(listener)
        twist = Twist()
        twist.linear.x, twist.angular.z = linear, angular
        for _ in range(int(seconds * 10)):
            if rospy.is_shutdown():
                break
            cmd_vel.publish(twist)
            rate.sleep()
        cmd_vel.publish(Twist())
        rospy.sleep(0.3)
        end = robot_pose(listener)
        if not (start and end):
            return 0.0
        return math.hypot(end[0] - start[0], end[1] - start[1])

    # Escalate: the way with room, then the other way, then wiggle. Each step is
    # only tried because the previous one measurably failed, so a robot that is
    # merely paused does not get thrashed.
    moved = drive(speed, 0.0, ESCAPE_TIME)
    if moved < FREED_DISTANCE:
        moved = max(moved, drive(-speed, 0.0, ESCAPE_TIME))
    if moved < FREED_DISTANCE:
        # Alternating hard turns change which part of the footprint is loaded
        # against the obstacle. Straight pushes cannot do that, and a wedge that
        # resists both directions is usually a corner contact rather than a wall.
        left = scan_clearance(60, 120) or 0.0
        right = scan_clearance(240, 300) or 0.0
        turn = SPIN_SPEED if left >= right else -SPIN_SPEED
        for sign in (1, -1, 1):
            moved = max(moved, drive(speed * 0.5, turn * sign, 1.2))
            if moved >= FREED_DISTANCE:
                break

    rospy.loginfo(f"recovery moved {moved:.3f} m")
    return moved >= FREED_DISTANCE


def await_goal(client, listener):
    """Wait out a goal, but stop early once the robot has clearly stopped moving.

    move_base only gives up after its own patience and recovery behaviours run
    out, which took the full GOAL_TIMEOUT on every wedge in the last run. The
    robot had not moved for a minute by then; the outcome was already decided.
    Watching tf instead turns a 90 s stall into a 20 s one, and the earlier the
    recovery runs the more likely it still has room to work with.
    """
    deadline = time.time() + GOAL_TIMEOUT
    last_pose = robot_pose(listener)
    last_move = time.time()

    while time.time() < deadline and not rospy.is_shutdown():
        if client.wait_for_result(rospy.Duration(1.0)):
            return client.get_state()

        pose = robot_pose(listener)
        if pose and last_pose:
            if math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1]) > STALL_DISTANCE:
                last_move = time.time()
        last_pose = pose or last_pose

        if time.time() - last_move > STALL_SECONDS:
            rospy.logwarn(f"no movement for {STALL_SECONDS:.0f}s - abandoning this goal")
            client.cancel_all_goals()
            rospy.sleep(0.5)
            return GoalStatus.ABORTED

    client.cancel_all_goals()
    return GoalStatus.LOST


def robot_pose(listener):
    try:
        (trans, _) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        return trans[0], trans[1]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--timeout', type=float, default=1800.0,
                        help='give up after this many seconds (default: 1800)')
    parser.add_argument('--no-spin', action='store_true',
                        help='skip the look-around spin at each goal (faster, worse map)')
    args = parser.parse_args()

    rospy.init_node('explore_for_mapping')
    view = MapView()
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("waiting for move_base action server")
    if not client.wait_for_server(rospy.Duration(60)):
        rospy.logerr("move_base never came up - is house_mapping.launch running?")
        return

    rospy.loginfo("waiting for /map from gmapping")
    while view.grid is None and not rospy.is_shutdown():
        rospy.sleep(0.5)

    clearance_m = min_clearance()
    rospy.loginfo(f"frontiers need {clearance_m:.3f} m of clearance")

    # The first scan only sees one room, so let gmapping settle before trusting a
    # frontier count to tell us we are finished.
    spin_in_place(cmd_vel)

    started = time.time()
    blacklist = []
    empty_scans = 0
    reached = failed = hard_stuck = exhausted_scans = 0

    while not rospy.is_shutdown():
        if time.time() - started > args.timeout:
            rospy.logwarn(f"timeout after {args.timeout:.0f}s - stopping with "
                          "the map as it stands")
            break

        pose = robot_pose(listener)
        if pose is None:
            rospy.logwarn_throttle(5, "no map->base_footprint transform yet")
            rospy.sleep(1.0)
            continue

        clusters = find_frontier_clusters(view, clearance_m)
        goal = pick_goal(clusters, pose, blacklist)
        unknown, free, occupied = view.counts()
        rospy.loginfo(f"map {free} free / {occupied} occupied / {unknown} unknown "
                      f"| {len(clusters)} frontiers | {reached} reached, {failed} failed")

        if goal is None:
            if not clusters:
                empty_scans += 1
                if empty_scans >= DONE_AFTER_EMPTY:
                    rospy.loginfo("no frontiers left - the house is mapped")
                    break
                # Not necessarily finished: a frontier can be hidden behind a door
                # the robot is facing away from. Look around before believing it.
                spin_in_place(cmd_vel)
                continue
            if all_exhausted(clusters, blacklist):
                exhausted_scans += 1
                if exhausted_scans >= DONE_AFTER_EMPTY:
                    rospy.logwarn(
                        f"{len(clusters)} frontiers remain but each has failed "
                        f"{MAX_FRONTIER_ATTEMPTS} times - stopping. The map will "
                        "have open edges where those frontiers are.")
                    break
                # Not yet. The set of clusters is recomputed from a map that is
                # still growing, so "every cluster I can see right now is spent"
                # is a snapshot, not a conclusion - one more look often splits a
                # spent cluster into new ones behind it.
                spin_in_place(cmd_vel)
                continue
            exhausted_scans = 0
            # Frontiers exist and are merely cooling down. Spin rather than
            # counting this as finished; the extra look often fills them in anyway.
            spin_in_place(cmd_vel)
            continue
        # Both counters measure *consecutive* dead ends, so finding a goal at all
        # resets them. Without this they accumulate across a run that is making
        # progress and eventually stop it on evidence gathered minutes apart.
        empty_scans = exhausted_scans = 0

        x, y = goal
        yaw = math.atan2(y - pose[1], x - pose[0])
        rospy.loginfo(f"-> frontier at ({x:.2f}, {y:.2f})")

        msg = MoveBaseGoal()
        msg.target_pose.header.frame_id = 'map'
        msg.target_pose.header.stamp = rospy.Time.now()
        msg.target_pose.pose.position.x = x
        msg.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        (msg.target_pose.pose.orientation.x, msg.target_pose.pose.orientation.y,
         msg.target_pose.pose.orientation.z, msg.target_pose.pose.orientation.w) = q

        client.send_goal(msg)
        state = await_goal(client, listener)

        if state == GoalStatus.SUCCEEDED:
            reached += 1
            hard_stuck = 0
            if not args.no_spin:
                spin_in_place(cmd_vel)
        else:
            failed += 1
            record_failure(blacklist, x, y)
            tries = blacklist_entry(blacklist, x, y)['fails']
            rospy.logwarn(f"frontier {STATUS_NAMES.get(state, state)} "
                          f"({tries}/{MAX_FRONTIER_ATTEMPTS} attempts)")
            if recover(client, cmd_vel, listener):
                hard_stuck = 0
            else:
                hard_stuck += 1
                if hard_stuck >= MAX_HARD_STUCK:
                    rospy.logerr(
                        f"{hard_stuck} recoveries in a row moved the robot less than "
                        f"{FREED_DISTANCE} m - it is wedged and cannot free itself. "
                        "Stopping so the map so far can still be saved.")
                    break

    cmd_vel.publish(Twist())
    unknown, free, occupied = view.counts()
    area = free * view.info.resolution ** 2
    rospy.loginfo(f"done: {reached} frontiers reached, {failed} failed, "
                  f"{area:.1f} sq.m mapped")
    rospy.loginfo("save it with: rosrun map_server map_saver -f "
                  "~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
