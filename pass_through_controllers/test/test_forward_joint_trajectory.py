#!/usr/bin/env python
import unittest
import rospy
import actionlib
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
import tf
import time
import copy
from collections import OrderedDict

PKG = 'pass_through_controllers'
NAME = 'test_joint_trajectory_forwarding'


class IntegrationTest(unittest.TestCase):

    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        rospy.init_node(NAME)

        timeout = rospy.Duration(3)

        self.client = actionlib.SimpleActionClient(
            "/hw_interface/forward_joint_trajectories/follow_joint_trajectory", FollowJointTrajectoryAction)
        try:
            self.client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail("Could not reach controller action. Msg: {}".format(err))

        self.switch_robot_ctrl = rospy.ServiceProxy('/robot/controller_manager/switch_controller', SwitchController)
        try:
            self.switch_robot_ctrl.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach robot controller switch service. Msg: {}".format(err))

        self.switch_forward_ctrl = rospy.ServiceProxy('/hw_interface/controller_manager/switch_controller', SwitchController)
        try:
            self.switch_forward_ctrl.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach hw_interface controller switch service. Msg: {}".format(err))

        self.listener = tf.TransformListener()

        # Correctly ordered joint names with reference joint state
        self.joint_map = OrderedDict()
        self.joint_map['joint1'] = 0
        self.joint_map['joint2'] = -2.0
        self.joint_map['joint3'] = 2.26
        self.joint_map['joint4'] = -0.2513274122872
        self.joint_map['joint5'] = 1.57
        self.joint_map['joint6'] = 0.0

        # Cartesian reference pose for this joint state (x,y,z, qx, qy, qz, qw)
        self.p_ref = [0.354, 0.180, 0.390, 0.502, 0.502, 0.498, 0.498]

    def test_normal_execution(self):
        """ Test the basic functionality by moving to a defined joint state """
        self.switch_to_joint_control()

        # Move to reference joint state
        start_joint_state = FollowJointTrajectoryGoal()
        start_joint_state.trajectory.joint_names = self.joint_map.keys()

        start_joint_state.trajectory.points = [
            JointTrajectoryPoint(positions=self.joint_map.values(), time_from_start=rospy.Duration(3.0))]
        start_joint_state.goal_time_tolerance = rospy.Duration(1)
        self.client.send_goal(start_joint_state)
        self.client.wait_for_result()

        time.sleep(1)
        self.check_reached()
        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)

    def test_mixed_joint_order(self):
        """ Test whether a mixed-up joint order gets executed correctly """
        self.switch_to_joint_control()

        # Move to reference joint state with different joint order
        joint_names = ['joint6', 'joint4', 'joint3', 'joint1', 'joint5', 'joint2']
        q_start = [self.joint_map[i] for i in joint_names]

        start_joint_state = FollowJointTrajectoryGoal()
        start_joint_state.trajectory.joint_names = joint_names

        start_joint_state.trajectory.points = [
            JointTrajectoryPoint(positions=q_start, time_from_start=rospy.Duration(3.0))]
        start_joint_state.goal_time_tolerance = rospy.Duration(1)
        self.client.send_goal(start_joint_state)
        self.client.wait_for_result()

        time.sleep(1)
        self.check_reached()
        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)

    def test_wrong_joint_names(self):
        """ Test whether a trajectory with wrong joint names fails """
        self.switch_to_joint_control()

        # Unknown joint names
        joint_names = ['a', 'b', 'c', 'd', 'e', 'f']

        start_joint_state = FollowJointTrajectoryGoal()
        start_joint_state.trajectory.joint_names = joint_names

        start_joint_state.trajectory.points = [
            JointTrajectoryPoint(positions=self.joint_map.values(), time_from_start=rospy.Duration(3.0))]
        start_joint_state.goal_time_tolerance = rospy.Duration(1)
        self.client.send_goal(start_joint_state)
        self.client.wait_for_result()

        time.sleep(1)
        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.INVALID_GOAL)

    def switch_to_joint_control(self):
        """ Assume possibly running Cartesian controllers"""

        # Prepare robot dummy
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['cartesian_trajectory_controller']
        srv.start_controllers = ['joint_trajectory_controller']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_robot_ctrl(srv)

        # Prepare passthrough controller
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['forward_cartesian_trajectories']
        srv.start_controllers = ['forward_joint_trajectories']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_forward_ctrl(srv)

    def check_reached(self):
        """ Check whether we reached our target within TF lookup accuracy """
        try:
            (trans, rot) = self.listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        n = 3  # places accuracy
        self.assertAlmostEqual(self.p_ref[0], trans[0], n)
        self.assertAlmostEqual(self.p_ref[1], trans[1], n)
        self.assertAlmostEqual(self.p_ref[2], trans[2], n)
        self.assertAlmostEqual(self.p_ref[3], rot[0], n)
        self.assertAlmostEqual(self.p_ref[4], rot[1], n)
        self.assertAlmostEqual(self.p_ref[5], rot[2], n)
        self.assertAlmostEqual(self.p_ref[6], rot[3], n)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
