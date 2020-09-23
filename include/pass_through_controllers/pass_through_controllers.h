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
#include "control_msgs/JointTolerance.h"
#include <controller_interface/controller.h>
#include <memory>
#include <pass_through_controllers/trajectory_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <vector>

namespace joint_trajectory_controllers {

class PassThroughController
  : public controller_interface::Controller<hardware_interface::JointTrajectoryInterface>
{
public:
  bool init(hardware_interface::JointTrajectoryInterface* traj_interface,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);


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
  /**
   * @brief  Monitor the trajectory execution
   *
   * @param feedback The feedback to use for evaluating tolerances
   */
  void monitorExecution(const hardware_interface::JointTrajectoryFeedback& feedback);

  /**
   * @brief Check if tolerances are met
   *
   * @param error The error to check
   * @param tolerances The tolerances to check against
   *
   * @return False if any of the errors exceeds its tolerance, else true
   */
  bool withinTolerances(const trajectory_msgs::JointTrajectoryPoint& error,
                        const std::vector<control_msgs::JointTolerance>& tolerances);


  /**
   * @brief Gets called when the action goal's time is up.
   */
  void timesUpCB(const ros::TimerEvent& event);

  bool m_done;
  std::vector<std::string> m_joint_names;
  std::vector<control_msgs::JointTolerance> m_path_tolerances;
  std::vector<control_msgs::JointTolerance> m_goal_tolerances;
  std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> m_action_server;
  std::unique_ptr<hardware_interface::JointTrajectoryHandle> m_trajectory_handle;
};
} // namespace joint_trajectory_controllers






namespace cartesian_trajectory_controllers {

class PassThroughController
  : public controller_interface::Controller<hardware_interface::CartesianTrajectoryInterface>
{
public:
  bool init(hardware_interface::CartesianTrajectoryInterface* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:
};

} // namespace cartesian_trajectory_controllers
