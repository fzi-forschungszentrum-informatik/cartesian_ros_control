#!/usr/bin/env python
"""
Simple action client for testing PassThroughControllers

Use this to fire-off a quick trajectory goal for testing.
The trajectory will last 10 seconds.

"""

from __future__ import print_function
import rospy
import sys
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def test():
    client = actionlib.SimpleActionClient(
        '/hw_interface/forward_joint_trajectories/follow_joint_trajectory',
        FollowJointTrajectoryAction)

    client.wait_for_server()

    # Drive to a single point for testing
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    p = JointTrajectoryPoint()
    p.positions = [0, -1.570796, 1.570796, 0, -1.570796, 0]  # arbitrary
    p.time_from_start = rospy.Duration(10)  # arbitrary
    goal.trajectory.points.append(p)

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('action_test_client')
        result = test()
        print("Result: {}".format(result))
    except rospy.ROSInterruptException:
        pass
