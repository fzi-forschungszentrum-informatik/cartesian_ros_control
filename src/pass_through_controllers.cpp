// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controllers.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryResult.h"
#include "ros/duration.h"
#include <pass_through_controllers/trajectory_interface.h>
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/pass_through_controllers.h>

namespace joint_trajectory_controllers
{
  bool PassThroughController::init(hardware_interface::JointTrajectoryInterface* traj_interface,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    // Get names of joints from the parameter server
    std::vector<std::string> joint_names;
    if (!controller_nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace()
                                         << "/joints from parameter server");
      return false;
    }

    try
    {
      traj_interface->setResources(joint_names);
      m_trajectory_handle = std::make_unique<hardware_interface::JointTrajectoryHandle>(
          traj_interface->getHandle("joint_trajectory_handle"));
    }
    catch (hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM(
          "joint_trajectory_controllersPassThroughController: Exception getting trajectory handle from interface: "
          << ex.what());
      return false;
    }

    // Action server
    m_action_server.reset(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
          controller_nh,
          "follow_joint_trajectory",
          std::bind(&PassThroughController::executeCB, this, std::placeholders::_1),
          false));

    // This is the cleanest method to notify the vendor robot driver of
    // preempted requests.
    m_action_server->registerPreemptCallback(
        std::bind(&PassThroughController::preemptCB, this));

    m_action_server->start();

    return true;
  }

  void PassThroughController::starting(const ros::Time& time)
  {
  }

  void PassThroughController::stopping(const ros::Time& time)
  {
    if (m_action_server->isActive())
    {
      // Set canceled flag in the action result
      m_action_server->setPreempted();

      // Stop trajectory interpolation on the robot
      m_trajectory_handle->cancelCommand();
    }

  }

  void PassThroughController::update(const ros::Time& time, const ros::Duration& period)
  {
    if (m_action_server->isActive())
    {
      hardware_interface::JointTrajectoryFeedback f = m_trajectory_handle->getFeedback();
      m_action_server->publishFeedback(f);

      // Check tolerances on each call and set terminal conditions for the
      // action server if special criteria are met.
      // Also set the m_done flag once that happens.
      monitorExecution(f);
    }
  }

  void PassThroughController::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    // Upon entering this callback, the simple action server has already
    // preempted the previously active goal (if any) and has accepted the new goal.

    if (!this->isRunning())
    {
      ROS_ERROR("Can't accept new action goals. Controller is not running.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      m_action_server->setAborted(result);
      return;
    }
    
    // Notify the  vendor robot control.
    m_trajectory_handle->setCommand(*goal);
    m_done = false;

    while (!m_done)
    {
      ros::Duration d(0.01);
      d.sleep();
    }

    // When done, the action server is in one of the three states:
    // 1) succeeded: managed in update()
    // 2) aborted: managed in update()
    // 3) preempted: managed in preemptCB()
  }

  void PassThroughController::preemptCB()
  {
    // Notify the vendor robot control.
    m_trajectory_handle->cancelCommand();

    control_msgs::FollowJointTrajectoryResult result;
    result.error_string = "preempted";
    m_action_server->setPreempted(result);

    m_done = true;
  }

  void PassThroughController::monitorExecution(
    const hardware_interface::JointTrajectoryFeedback& feedback)
  {
    // TODO: Check path_tolerance
    {
      //control_msgs::FollowJointTrajectoryResult result;
      //result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      // m_action_server->setAborted(result);
      // m_done = true;
      // return;
    }

    // TODO: Check goal_tolerance and goal_time_tolerance
    {
      //m_action_server->setSucceeded();
      // m_done = true;
      // return;
    }
    {
      // control_msgs::FollowJointTrajectoryResult result;
      // result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      //m_action_server->setAborted(result);
      // m_done = true;
      // return;
    }

  }
}

PLUGINLIB_EXPORT_CLASS(joint_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)




namespace cartesian_trajectory_controllers
{
  bool PassThroughController::init(hardware_interface::CartesianTrajectoryInterface* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    return true;
  }

  void PassThroughController::starting(const ros::Time& time)
  {
  }

  void PassThroughController::stopping(const ros::Time& time)
  {
  }

  void PassThroughController::update(const ros::Time& time, const ros::Duration& period)
  {
  }

}


PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)
