// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controllers.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include "control_msgs/FollowJointTrajectoryGoal.h"
#include <controller_interface/multi_interface_controller.h>
#include <memory>
#include <pass_through_controllers/trajectory_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace joint_trajectory_controllers {

class PassThroughController
  : public controller_interface::MultiInterfaceController<
      hardware_interface::JointTrajectoryInterface,
      hardware_interface::JointStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);


  std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> m_action_server;

  /**
   * @brief Callback method for new action goals
   *
   * This method calls the \a on_new_cmd callback from the
   * hardware_interface::TrajectoryHandle<JointTrajectory>.
   * Implementers of the ROS-control HW can choose how the trajectory goal is
   * forwarded to the robot for interpolation.
   *
   * Things in detail:
   * - New goals are only accepted when the controller is running
   * - Switching the controller (stopping) cancels running goals
   * - The goal's success is monitored in update()
   *
   * Further info on how the simple action server works is given
   * <a href="http://wiki.ros.org/actionlib/DetailedDescription">here</a>.
   *
   * @param goal The trajectory goal
   */
  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  /**
   * @brief Callback method for preempt requests
   *
   * This method gets called on every preempt request that happens either
   * directly upon a client request or indirectly when receiving a new goal
   * while another is still active.
   *
   * This method calls the \a on_cancel callback from the hardware_interface::TrajectoryHandle<JointTrajectory>.
   * Implementers of the ROS-control HW can implement how this notification is
   * handled by the robot vendor control.
   *
   * Also check 
   * <a href="https://answers.ros.org/question/333316/actionlib-preempt-vs-cancel/">this info</a>.
   * on the simple action server's preemption policy:
   */
  void preemptCB();

private:
  std::unique_ptr<hardware_interface::JointTrajectoryHandle> m_trajectory_handle;
  std::vector<hardware_interface::JointStateHandle> m_joint_handles;
};
} // namespace joint_trajectory_controllers






namespace cartesian_trajectory_controllers {

class PassThroughController
  : public controller_interface::MultiInterfaceController<
      hardware_interface::CartesianTrajectoryInterface,
      cartesian_ros_control::CartesianStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:
};

} // namespace cartesian_trajectory_controllers
