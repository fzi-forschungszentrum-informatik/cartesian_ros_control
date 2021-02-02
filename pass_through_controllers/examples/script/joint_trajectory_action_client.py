#!/usr/bin/env python
"""
Simple action client for testing joint-based PassThroughControllers

Use this to fire-off a quick random trajectory goal for testing.
The trajectory will last 10 seconds.

"""

from __future__ import print_function
import rospy
import actionlib
import signal
import sys
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class Client(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            '/hw_interface/forward_joint_trajectories/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def test(self):
        """ Drive to random joint positions

        This test builds a random goal configuration by sampling uniformly in
        [-pi, +pi] for each joint.  It then drives there within 10 seconds.
        """
        goal = FollowJointTrajectoryGoal()
        duration = 10
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        goal.trajectory.joint_names = joint_names

        p = JointTrajectoryPoint()
        p.positions = [(np.random.random_sample() * 2 - 1) * np.pi for i in range(len(joint_names))]
        p.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(p)

        self.client.send_goal(goal)
        self.client.wait_for_result()

        return self.client.get_result()

    def clean_shutdown(self, msg=None):
        """ Cancel goal on Ctrl-C """
        self.client.cancel_goal()
        if msg is not None:
            print(msg)
        sys.exit(0)


if __name__ == '__main__':

    try:
        rospy.init_node('action_test_client')
        client = Client()
        signal.signal(signal.SIGINT, lambda sig, frame: client.clean_shutdown("\nGoal canceled."))
        result = client.test()
        print("Result: {}".format(result))

    except rospy.ROSInterruptException:
        pass
