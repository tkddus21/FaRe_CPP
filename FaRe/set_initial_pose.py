#!/usr/bin/env python3
"""Seeds AMCL with the robot's ground-truth pose from Gazebo.

Replaces clicking "2D Pose Estimate" in rviz, so the patrol can be launched
without a GUI. Run this after turtlebot3_navigation.launch is up and before
PatrolSim.py; without an initial pose AMCL never localises and move_base
rejects every goal.
"""

import argparse
import os

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    default_model = f"turtlebot3_{os.environ.get('TURTLEBOT3_MODEL', 'burger')}"
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=default_model,
                        help='Gazebo model name (default: from TURTLEBOT3_MODEL)')
    args = parser.parse_args()

    rospy.init_node('set_initial_pose', anonymous=True)

    rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    state = get_state(args.model, '')
    if not state.success:
        rospy.logerr(f"model '{args.model}' not found in Gazebo")
        return

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose = state.pose
    # Modest covariance: confident enough to converge, loose enough that AMCL
    # still corrects itself against the laser scan.
    msg.pose.covariance[0] = 0.05   # x
    msg.pose.covariance[7] = 0.05   # y
    msg.pose.covariance[35] = 0.02  # yaw

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    rospy.sleep(1.5)  # let the latched publisher connect to amcl
    for _ in range(3):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.sleep(0.5)

    p = state.pose.position
    rospy.loginfo(f"seeded /initialpose from Gazebo: x={p.x:.3f} y={p.y:.3f}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
