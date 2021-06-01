#!/usr/bin/env python
import unittest
import rospy
import actionlib
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController

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

    def test_normal_execution(self):
        """ Test the basic functionality by moving to a single joint state """
        self.switch_to_joint_control()
        self.move()
        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)

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

    def move(self):
        """ Move to an arbitrary joint state within 3 seconds. """

        q_start = [0, -2.0, 2.26, -0.2513274122872, 1.57, 0.0]  # From base to tip
        start_joint_state = FollowJointTrajectoryGoal()
        start_joint_state.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6']

        start_joint_state.trajectory.points = [
            JointTrajectoryPoint(positions=q_start, time_from_start=rospy.Duration(3.0))]
        self.client.send_goal(start_joint_state)
        self.client.wait_for_result()


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
