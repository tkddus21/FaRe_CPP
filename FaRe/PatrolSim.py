#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
import csv
import time
import yaml
import os
from config import config
from MAP import Map_generator
map_generator = Map_generator()
output_dir = config['output_dir']
pgm_filename = config['pgm_filename']
yaml_filename = config['yaml_filename']
map_data = map_generator.load_yaml(yaml_filename)
waypoints_path = os.path.join(output_dir,'wp_ori_data.txt')
patrol_log_path = os.path.join(output_dir,'patrol_log.csv')

GOAL_TIMEOUT = 120.0  # seconds; without this a single unreachable goal stalls the run forever
STATUS_NAMES = {v: k for k, v in vars(GoalStatus).items() if isinstance(v, int) and k.isupper()}

BACKUP_SPEED = -0.08  # m/s
BACKUP_TIME = 2.0     # seconds, i.e. reverse ~16 cm


def recover(client, cmd_vel_pub):
    """Un-wedge the robot after a failed goal.

    On cluttered maps the robot can drive into a gap barely wider than itself.
    move_base's own rotate recovery then refuses to act ("can't rotate in place
    because there is a potential collision"), so it stays stuck and *every*
    later goal aborts too. Reversing under direct velocity control backs it out
    of the pinch, which move_base cannot do for itself.
    """
    client.cancel_all_goals()

    try:
        rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
    except rospy.ServiceException as exc:
        rospy.logwarn(f"clear_costmaps failed: {exc}")

    twist = Twist()
    twist.linear.x = BACKUP_SPEED
    rate = rospy.Rate(10)
    for _ in range(int(BACKUP_TIME * 10)):
        cmd_vel_pub.publish(twist)
        rate.sleep()
    cmd_vel_pub.publish(Twist())  # stop
    rospy.loginfo("recovery: cleared costmaps and backed up")

def send_goal(client, x, y, theta):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(GOAL_TIMEOUT)):
        client.cancel_goal()
        return 'TIMEOUT'

    return STATUS_NAMES.get(client.get_state(), str(client.get_state()))


def grid_to_world_coords(wp, map_data):
    resolution = map_data['resolution']
    origin = map_data['origin']
    map_height = map_generator.load_pgm(pgm_filename).shape[0]

    # FaRe stores waypoints as (row, col) i.e. (y_pixel, x_pixel), matching
    # numpy's array[row, col] indexing used throughout Scout_Multi_Processing.py.
    # ROS map_server's pgm convention has row 0 at the TOP of the image, but the
    # map origin refers to the BOTTOM-LEFT pixel, so the row axis must be
    # flipped when converting to world y (row and col were previously swapped
    # here with no flip, sending goals to the wrong physical location).
    world_coords = []
    for row, col in wp:
        x = col * resolution + origin[0]
        y = (map_height - 1 - row) * resolution + origin[1]
        world_coords.append((x, y))
    return world_coords


def patrol():
    rospy.init_node('patrol_waypoints')
    with open(waypoints_path, 'r') as f:
        lines = f.readlines()
        wp = eval(lines[0].split('=')[1].strip())  # Waypoints from occupancy grid
        ori = eval(lines[1].split('=')[1].strip())  # Orientations in radians

    if len(wp) != len(ori):
        rospy.logerr(f"waypoint/orientation length mismatch: {len(wp)} vs {len(ori)}. "
                     "Re-run Surveillance.py to regenerate wp_ori_data.txt.")
        return

    # Convert waypoints from grid cells to world coordinates
    world_waypoints = grid_to_world_coords(wp, map_data)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("waiting for move_base action server")
    client.wait_for_server()
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    with open(patrol_log_path, 'w', newline='') as log_file:
        writer = csv.writer(log_file)
        writer.writerow(['index', 'grid_row', 'grid_col', 'world_x', 'world_y',
                         'yaw_rad', 'status', 'duration_s'])
        try:
            for i, (x, y) in enumerate(world_waypoints):
                rospy.loginfo(f"Sending goal {i+1}/{len(world_waypoints)}: "
                              f"({x:.2f}, {y:.2f}, {ori[i]:.2f})")
                start = time.time()
                status = send_goal(client, x, y, ori[i])
                elapsed = time.time() - start

                rospy.loginfo(f"goal {i+1} -> {status} in {elapsed:.1f}s")
                writer.writerow([i, wp[i][0], wp[i][1], round(x, 3), round(y, 3),
                                 round(ori[i], 4), status, round(elapsed, 1)])
                log_file.flush()

                # A failure usually means the robot is wedged; without this the
                # remaining goals all fail against a robot that cannot move.
                if status != 'SUCCEEDED':
                    recover(client, cmd_vel_pub)

            rospy.loginfo(f"Patrolling is finished, log saved to {patrol_log_path}")

        except rospy.ROSInterruptException:
            rospy.loginfo("Stopping patrolling due to interruption")

if __name__ == '__main__':
    try:
        patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")
