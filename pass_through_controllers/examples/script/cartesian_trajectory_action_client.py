#!/usr/bin/env python
"""
Simple action client for testing Cartesian-based PassThroughControllers

Use this to fire-off a quick random Cartesian trajectory goal for testing.
The trajectory will last 10 seconds.

"""

from __future__ import print_function
import rospy
import actionlib
import signal
import sys
import os
import numpy as np
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL


class Client(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            '/hw_interface/forward_cartesian_trajectories/follow_cartesian_trajectory',
            FollowCartesianTrajectoryAction)
        self.client.wait_for_server()

        # Suppress spam output of urdf parsing.
        # urdf_parser_py is unhappy with various visual tags in the robot_description.
        tmp = sys.stderr
        sys.stderr = open(os.devnull, 'w')
        robot = URDF.from_parameter_server()
        sys.stderr = tmp

        _, tree = treeFromUrdfModel(robot)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(tree.getChain('base_link', 'tool0'))

    def test(self):
        """ Follow two-point, random Cartesian trajectory

        This samples uniformly in [-pi, +pi] for each joint to compute two
        random poses within the robots reach.  It then traverses these points
        within 10 seconds.
        """
        def random_point():
            p_kdl = PyKDL.Frame()
            joints = PyKDL.JntArray(6)
            for i in range(6):
                joints[i] = (np.random.random_sample() * 2 - 1) * np.pi
            self.fk_solver.JntToCart(joints, p_kdl)

            p = CartesianTrajectoryPoint()
            p.pose.position.x = p_kdl.p[0]
            p.pose.position.y = p_kdl.p[1]
            p.pose.position.z = p_kdl.p[2]
            q = PyKDL.Rotation.GetQuaternion(p_kdl.M)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            return p

        # Random 2-point trajectory
        duration = 10
        p1 = random_point()
        p2 = random_point()
        p1.time_from_start = rospy.Duration(0.5 * duration)
        p2.time_from_start = rospy.Duration(duration)

        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory.points.append(p1)
        goal.trajectory.points.append(p2)

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
