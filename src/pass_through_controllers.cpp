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
#include <pass_through_controllers/trajectory_interface.h>
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/pass_through_controllers.h>
#include <hardware_interface/joint_state_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>

namespace joint_trajectory_controllers
{
  bool PassThroughController::init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    // Get names of readable joints from the parameter server
    std::vector<std::string> joint_names;
    if (!controller_nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace()
                                         << "/joints from parameter server");
      return false;
    }

    // Get handle for trajectory forwarding from the hardware interface
    auto* traj_interface = robot_hw->get<hardware_interface::JointTrajectoryInterface>();
    if (traj_interface == nullptr)
    {
      ROS_ERROR("joint_trajectory_controllers/PassThroughController: Error getting "
          "trajectory interface from hardware");
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

    // Get the joint state handles for computing action feedback
    auto* joint_state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
    if (joint_state_interface == nullptr)
    {
      ROS_ERROR_STREAM("joint_trajectory_controllersPassThroughController: Error getting joint "
                       "state interface from hardware");
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      try
      {
        m_joint_handles.push_back(joint_state_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& ex)
      {
        ROS_ERROR_STREAM("joint_trajectory_controllersPassThroughController: Exception getting "
                         "joint state handles: "
                         << ex.what());
        return false;
      }
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
      m_action_server->publishFeedback(
          m_trajectory_handle->getFeedback());

      // TODO: Monitor tolerances of execution
      // and set goal result to success once finished.
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
    
    m_trajectory_handle->setCommand(*goal);

    // TODO: Wait for the robot
    m_action_server->setSucceeded();
  }

  void PassThroughController::preemptCB()
  {
    // Notify the vendor robot control.
    m_trajectory_handle->cancelCommand();
  }

}

PLUGINLIB_EXPORT_CLASS(joint_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)




namespace cartesian_trajectory_controllers
{
  bool PassThroughController::init(hardware_interface::RobotHW* robot_hw,
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
