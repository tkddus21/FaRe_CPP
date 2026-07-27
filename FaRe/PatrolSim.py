#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
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

def send_goal(x, y, theta):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

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
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Failed to reach the goal")
    elif client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached the goal")
    else:
        rospy.logwarn("Goal did not succeed, status: %s", client.get_state())


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

    # Convert waypoints from grid cells to world coordinates
    world_waypoints = grid_to_world_coords(wp, map_data)
    try:
        for i, (x, y) in enumerate(world_waypoints):
            # wp includes a final "return to start" point with no matching
            # entry in ori (one shorter); reuse the last known orientation for it.
            theta = ori[i] if i < len(ori) else ori[-1]
            rospy.loginfo(f"Sending goal {i+1}/{len(world_waypoints)}: ({x}, {y}, {theta})")
            send_goal(x, y, theta)

        rospy.loginfo("Patrolling is finished")

    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping patrolling due to interruption")

if __name__ == '__main__':
    try:
        patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")
