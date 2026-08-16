#!/usr/bin/env python3
"""Drives a fixed tour of the turtlebot3_house so gmapping sees all of it.

Frontier exploration (explore_for_mapping.py) decides where to go from the map it
has built so far, which makes it good at finding rooms but bad at finishing them:
it stops as soon as no free cell touches unmapped space, and a wall seen once from
four metres away satisfies that test while still being a smear.

This does the opposite. The house is a known, static world, so its geometry is not
something to discover - it is in
turtlebot3_gazebo/models/turtlebot3_house/model.sdf. Read the walls from there,
lay a grid over the rooms they enclose, and visit every cell. Coverage stops being
a search problem and becomes a list.

The grid spacing is deliberately below gmapping's 3.0 m maxUrange, so every part of
every room is seen from close enough for the scan to land, not just from a doorway.

Run house_sim.launch and house_mapping.launch first, then:

    python3 FaRe/tour_house_for_mapping.py

It is safe to run after explore_for_mapping.py and adds to whatever that built -
gmapping keeps accumulating as long as it is the same node.
"""

import argparse
import math
import os
import time
import xml.etree.ElementTree as ET

import rospy
import actionlib
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

MODEL_SDF = os.path.expanduser(
    '~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/'
    'turtlebot3_house/model.sdf')

# The rooms, as axis-aligned boxes in world metres, read off the wall centres in
# model.sdf. The house is a wide upper body with two wings hanging south from it -
# the shape is not one rectangle, and treating it as one sends the robot at goals
# in the garden between the wings.
# Order matters: the tour finishes one region before starting the next, and the
# wings only connect through the main body. Left, middle, right means every
# transition is between neighbours.
REGIONS = [
    (-7.4, -3.80, -5.25, 0.85),  # left wing
    (-7.4, -0.05, 7.4, 5.15),    # main body
    (5.00, -5.15, 7.4, -0.30),   # right wing
]

GRID_SPACING = 1.2    # metres; well below gmapping's 3.0 m maxUrange on purpose
WALL_MARGIN = 0.45    # metres to keep clear of a wall; waffle_pi circumscribes 0.257
# Sized for the first goal, not the typical one. Consecutive tour points are
# GRID_SPACING apart and take seconds, but the serpentine starts in a corner and the
# robot starts wherever the last run left it: the opening leg can be the full
# diagonal of the house, which at the waffle_pi's 0.26 m/s is over a minute before
# any turning. At 75 s that first goal timed out 1 m short of arriving.
GOAL_TIMEOUT = 150.0
STALL_SECONDS = 20.0
STALL_DISTANCE = 0.02
SPIN_SPEED = 0.8
ESCAPE_SPEED = 0.10
ESCAPE_TIME = 3.0

STATUS_NAMES = {v: k for k, v in vars(GoalStatus).items()
                if isinstance(v, int) and k.isupper()}


def wall_segments(path):
    """Wall centre-lines as ((x1, y1), (x2, y2)) in world metres.

    Each Wall_* link carries a pose and a box size; the box's x extent is the wall
    length and the pose yaw is its direction, so the segment is the centre plus and
    minus half the length along that direction.
    """
    root = ET.parse(path).getroot()
    segments = []
    for link in root.iter('link'):
        name = link.get('name') or ''
        if not name.startswith('Wall'):
            continue
        pose = link.find('pose')
        if pose is None:
            continue
        px, py, _, _, _, yaw = (float(v) for v in pose.text.split())
        size = None
        for box in link.iter('box'):
            s = box.find('size')
            if s is not None:
                size = [float(v) for v in s.text.split()]
                break
        if not size:
            continue
        half = size[0] / 2.0
        dx, dy = math.cos(yaw) * half, math.sin(yaw) * half
        segments.append(((px - dx, py - dy), (px + dx, py + dy)))
    return segments


def distance_to_segment(px, py, seg):
    (x1, y1), (x2, y2) = seg
    vx, vy = x2 - x1, y2 - y1
    length2 = vx * vx + vy * vy
    if length2 == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / length2))
    return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))


def tour_points(segments):
    """Grid points inside the rooms and clear of the walls, in a serpentine order.

    One grid over the whole house, not one per region. Fitting a grid to each
    region put its rows on the region's own edges, which are walls: the row at
    y = -0.05 sat 0.12 m from the wall at y = -0.17 and every point in it was
    rejected, leaving 7 points for the entire house and none at all in the wings.
    A single grid at a fixed pitch has no such relationship to the geometry.

    Serpentine rather than nearest-neighbour: the aim is to sweep each room and
    leave, not to minimise travel. A greedy nearest-neighbour tour ends up
    ping-ponging between rooms through the same doorway.
    """
    xs = [r[0] for r in REGIONS] + [r[2] for r in REGIONS]
    ys = [r[1] for r in REGIONS] + [r[3] for r in REGIONS]
    gx0, gx1, gy0, gy1 = min(xs), max(xs), min(ys), max(ys)
    nx = int((gx1 - gx0) / GRID_SPACING)
    ny = int((gy1 - gy0) / GRID_SPACING)

    points, seen = [], []
    for rx0, ry0, rx1, ry1 in REGIONS:
        rows = []
        for j in range(ny + 1):
            py = gy0 + j * GRID_SPACING
            if not ry0 <= py <= ry1:
                continue
            row = []
            for i in range(nx + 1):
                px = gx0 + i * GRID_SPACING
                if not rx0 <= px <= rx1:
                    continue
                if all(distance_to_segment(px, py, s) >= WALL_MARGIN
                       for s in segments):
                    row.append((px, py))
            rows.append(row)
        for j, row in enumerate(rows):
            for p in (row if j % 2 == 0 else list(reversed(row))):
                # Regions meet at their seams; a point visited in the previous
                # region is not worth a second goal.
                if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.5 for q in seen):
                    points.append(p)
                    seen.append(p)
    return points


def robot_pose(listener):
    try:
        trans, _ = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        return trans[0], trans[1]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None


def spin_in_place(cmd_vel):
    twist = Twist()
    twist.angular.z = SPIN_SPEED
    rate = rospy.Rate(10)
    for _ in range(int(2 * math.pi / SPIN_SPEED * 10)):
        if rospy.is_shutdown():
            break
        cmd_vel.publish(twist)
        rate.sleep()
    cmd_vel.publish(Twist())


def await_goal(client, listener):
    """Wait out a goal, giving up early once the robot has stopped moving."""
    deadline = time.time() + GOAL_TIMEOUT
    last_pose = robot_pose(listener)
    last_move = time.time()
    while time.time() < deadline and not rospy.is_shutdown():
        if client.wait_for_result(rospy.Duration(1.0)):
            return client.get_state()
        pose = robot_pose(listener)
        if pose and last_pose:
            if math.hypot(pose[0] - last_pose[0],
                          pose[1] - last_pose[1]) > STALL_DISTANCE:
                last_move = time.time()
        last_pose = pose or last_pose
        if time.time() - last_move > STALL_SECONDS:
            client.cancel_all_goals()
            rospy.sleep(0.5)
            return GoalStatus.ABORTED
    client.cancel_all_goals()
    return GoalStatus.LOST


def unwedge(cmd_vel, listener):
    """Nudge out of a contact, whichever way tf says actually moves the robot."""
    try:
        rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
    except rospy.ServiceException:
        pass
    rate = rospy.Rate(10)
    for speed in (-ESCAPE_SPEED, ESCAPE_SPEED):
        before = robot_pose(listener)
        twist = Twist()
        twist.linear.x = speed
        for _ in range(int(ESCAPE_TIME * 10)):
            if rospy.is_shutdown():
                return
            cmd_vel.publish(twist)
            rate.sleep()
        cmd_vel.publish(Twist())
        rospy.sleep(0.3)
        after = robot_pose(listener)
        if before and after and math.hypot(after[0] - before[0],
                                           after[1] - before[1]) > 0.05:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--dry-run', action='store_true',
                        help='print the tour and exit, without needing move_base')
    parser.add_argument('--no-spin', action='store_true',
                        help='skip the look-around spin at each point')
    args = parser.parse_args()

    segments = wall_segments(MODEL_SDF)
    points = tour_points(segments)
    print(f"{len(segments)} walls -> {len(points)} tour points")
    if args.dry_run:
        for i, (x, y) in enumerate(points):
            print(f"  {i:3d}  ({x:6.2f}, {y:6.2f})")
        return

    rospy.init_node('tour_house_for_mapping')
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    listener = tf.TransformListener()
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("waiting for move_base action server")
    if not client.wait_for_server(rospy.Duration(60)):
        rospy.logerr("move_base never came up - is house_mapping.launch running?")
        return
    rospy.wait_for_message('/map', OccupancyGrid, timeout=60)

    reached = failed = 0
    for i, (x, y) in enumerate(points):
        if rospy.is_shutdown():
            break
        pose = robot_pose(listener)
        while pose is None and not rospy.is_shutdown():
            rospy.sleep(1.0)
            pose = robot_pose(listener)
        yaw = math.atan2(y - pose[1], x - pose[0])

        msg = MoveBaseGoal()
        msg.target_pose.header.frame_id = 'map'
        msg.target_pose.header.stamp = rospy.Time.now()
        msg.target_pose.pose.position.x = x
        msg.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        (msg.target_pose.pose.orientation.x, msg.target_pose.pose.orientation.y,
         msg.target_pose.pose.orientation.z, msg.target_pose.pose.orientation.w) = q

        rospy.loginfo(f"[{i + 1}/{len(points)}] -> ({x:.2f}, {y:.2f})")
        client.send_goal(msg)
        state = await_goal(client, listener)

        if state == GoalStatus.SUCCEEDED:
            reached += 1
            if not args.no_spin:
                spin_in_place(cmd_vel)
        else:
            failed += 1
            # A point can be unreachable simply because furniture stands on it -
            # model.sdf's walls say nothing about the sofa. Skipping is correct;
            # the neighbouring points still cover the room.
            rospy.logwarn(f"[{i + 1}/{len(points)}] {STATUS_NAMES.get(state, state)}"
                          f" - skipping ({reached} reached, {failed} failed)")
            unwedge(cmd_vel, listener)

    cmd_vel.publish(Twist())
    rospy.loginfo(f"tour done: {reached} reached, {failed} skipped")
    rospy.loginfo("save it with: rosrun map_server map_saver -f "
                  "~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
