#!/usr/bin/env python
import unittest
import copy
import rospy
import actionlib
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    FollowCartesianTrajectoryResult,
    CartesianTrajectoryPoint)
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController

PKG = 'pass_through_controllers'
NAME = 'test_cartesian_trajectory_forwarding'


class IntegrationTest(unittest.TestCase):

    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        rospy.init_node(NAME)

        timeout = rospy.Duration(3)

        self.set_joints = actionlib.SimpleActionClient(
            "/robot/joint_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        if not self.set_joints.wait_for_server(timeout):
            self.fail("Could not reach robot action.")

        self.client = actionlib.SimpleActionClient(
            '/hw_interface/forward_cartesian_trajectories/follow_cartesian_trajectory',
            FollowCartesianTrajectoryAction)
        if not self.client.wait_for_server(timeout):
            self.fail("Could not reach hw_interface action.")

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
        """ Test the basic functionality by moving on a straight line """
        self.move_to_start()
        self.switch_to_cartesian_control()
        self.move()
        self.assertEqual(self.client.get_result().error_code,
                         FollowCartesianTrajectoryResult.SUCCESSFUL)

    def move_to_start(self):
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['cartesian_trajectory_controller']
        srv.start_controllers = ['joint_trajectory_controller']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_robot_ctrl(srv)

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
        self.set_joints.send_goal(
            start_joint_state)
        self.set_joints.wait_for_result()

    def switch_to_cartesian_control(self):
        """ Assume possibly running joint controllers"""

        # Prepare robot dummy
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['joint_trajectory_controller']
        srv.start_controllers = ['cartesian_trajectory_controller']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_robot_ctrl(srv)

        # Prepare passthrough controller
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['forward_joint_trajectories']
        srv.start_controllers = ['forward_cartesian_trajectories']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_forward_ctrl(srv)

    def move(self):
        """ Move on a straight line """

        # Cartesian end-effector pose that corresponds to the start joint
        # configuration
        start = CartesianTrajectoryPoint()
        start.pose.position.x = 0.354
        start.pose.position.y = 0.180
        start.pose.position.z = 0.390
        start.pose.orientation.x = 0.502
        start.pose.orientation.y = 0.502
        start.pose.orientation.z = 0.498
        start.pose.orientation.w = 0.498

        duration = 5

        p1 = copy.deepcopy(start)
        p1.pose.position.z += 0.3
        p1.time_from_start = rospy.Duration(duration)

        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory.points.append(p1)

        self.client.send_goal(goal)
        self.client.wait_for_result()


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
