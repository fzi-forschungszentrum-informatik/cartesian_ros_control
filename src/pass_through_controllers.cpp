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
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/trajectory_interface.h>
#include <pass_through_controllers/pass_through_controllers.h>
#include <string>

// Other
#include "ros/duration.h"
#include "ros/timer.h"

namespace trajectory_controllers {

  template <class TrajectoryInterface>
  bool PassThroughController<TrajectoryInterface>::init(TrajectoryInterface* traj_interface,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    // Get names of joints from the parameter server
    if (!controller_nh.getParam("joints", m_joint_names))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace()
                                         << "/joints from parameter server");
      return false;
    }

    try
    {
      traj_interface->setResources(m_joint_names);
      m_trajectory_handle = std::make_unique<typename Base::TrajectoryHandle>(
          traj_interface->getHandle(Base::TrajectoryHandle::getName()));
    }
    catch (hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM(
          controller_nh.getNamespace() << ": Exception getting trajectory handle from interface: "
          << ex.what());
      return false;
    }

    // Action server
    m_action_server.reset(new actionlib::SimpleActionServer<typename Base::FollowTrajectoryAction>(
          controller_nh,
          "follow_trajectory", // TODO: joint vs Cartesian
          std::bind(&PassThroughController::executeCB, this, std::placeholders::_1),
          false));

    // This is the cleanest method to notify the vendor robot driver of
    // preempted requests.
    m_action_server->registerPreemptCallback(
        std::bind(&PassThroughController::preemptCB, this));

    m_action_server->start();

    return true;
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::starting(const ros::Time& time)
  {
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::stopping(const ros::Time& time)
  {
    if (m_action_server->isActive())
    {
      // Set canceled flag in the action result
      m_action_server->setPreempted();

      // Stop trajectory interpolation on the robot
      m_trajectory_handle->cancelCommand();
    }

  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::update(const ros::Time& time, const ros::Duration& period)
  {
    if (m_action_server->isActive())
    {
      typename Base::TrajectoryFeedback f = m_trajectory_handle->getFeedback();
      m_action_server->publishFeedback(f);

      // Check tolerances on each call and set terminal conditions for the
      // action server if special criteria are met.
      // Also set the m_done flag once that happens.
      monitorExecution(f);
    }
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::executeCB(const typename Base::GoalConstPtr& goal)
  {
    // Upon entering this callback, the simple action server has already
    // preempted the previously active goal (if any) and has accepted the new goal.

    if (!this->isRunning())
    {
      ROS_ERROR("Can't accept new action goals. Controller is not running.");
      typename Base::FollowTrajectoryResult result;
      result.error_code = Base::FollowTrajectoryResult::INVALID_GOAL;
      m_action_server->setAborted(result);
      return;
    }

    // Only allow correct tolerances if given
    if (!goalValid(goal))
    {
      return;
    }
    m_path_tolerances = goal->path_tolerance;
    m_goal_tolerances = goal->goal_tolerance;
    
    // Notify the  vendor robot control.
    m_done = false;
    m_trajectory_handle->setCommand(*goal);

    // Start timer
    ros::Duration action_duration =
      goal->trajectory.points.back().time_from_start + goal->goal_time_tolerance;
    ros::Timer timer =
      ros::NodeHandle().createTimer(action_duration, &PassThroughController::timesUpCB, this, true);

    while (!m_done)
    {
      ros::Duration(0.01).sleep();
    }

    // When done, the action server is in one of the three states:
    // 1) succeeded: managed in update()
    // 2) aborted: managed in update()
    // 3) preempted: managed in preemptCB()
  }


  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::preemptCB()
  {
    // Notify the vendor robot control.
    m_trajectory_handle->cancelCommand();

    typename Base::FollowTrajectoryResult result;
    result.error_string = "preempted";
    m_action_server->setPreempted(result);

    m_done = true;
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::monitorExecution(
    const typename Base::TrajectoryFeedback& feedback)
  {
    // Abort if any of the joints exceeds its path tolerance
    if (!withinTolerances(feedback.error, m_path_tolerances))
    {
      typename Base::FollowTrajectoryResult result;
      result.error_code = Base::FollowTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      m_action_server->setAborted(result);
      m_done = true;
      return;
    }
  }

  template <>
  bool PassThroughController<hardware_interface::JointTrajectoryInterface>::withinTolerances(
    const TrajectoryPoint& error,
    const Tolerance& tolerances)
  {
    // Precondition
    if (!tolerances.empty())
    {
      assert(error.positions.size() == tolerances.size() &&
          error.velocities.size() == tolerances.size() &&
          error.accelerations.size() == tolerances.size());
    }

    for (size_t i = 0; i < tolerances.size(); ++i)
    {
      // > 0.0 means initialized
      if ((m_path_tolerances[i].position > 0.0 &&
           std::abs(error.positions[i]) > m_path_tolerances[i].position) ||
          (m_path_tolerances[i].velocity > 0.0 &&
           std::abs(error.velocities[i]) > m_path_tolerances[i].velocity) ||
          (m_path_tolerances[i].acceleration > 0.0 &&
           std::abs(error.accelerations[i]) > m_path_tolerances[i].acceleration))
      {
        return false;
      }
    }
    return true;
  }

  template <>
  bool PassThroughController<hardware_interface::CartesianTrajectoryInterface>::withinTolerances(
    const typename Base::TrajectoryPoint& error, const typename Base::Tolerance& tolerances)
  {
    // Precondition
    //if (!tolerances.empty())
    //{
    //  assert(error.positions.size() == tolerances.size() &&
    //      error.velocities.size() == tolerances.size() &&
    //      error.accelerations.size() == tolerances.size());
    //}

    //for (size_t i = 0; i < tolerances.size(); ++i)
    //{
    //  // > 0.0 means initialized
    //  if ((m_path_tolerances[i].position > 0.0 &&
    //       std::abs(error.positions[i]) > m_path_tolerances[i].position) ||
    //      (m_path_tolerances[i].velocity > 0.0 &&
    //       std::abs(error.velocities[i]) > m_path_tolerances[i].velocity) ||
    //      (m_path_tolerances[i].acceleration > 0.0 &&
    //       std::abs(error.accelerations[i]) > m_path_tolerances[i].acceleration))
    //  {
    //    return false;
    //  }
    //}
    return true;
  }

  template <>
  bool PassThroughController<hardware_interface::JointTrajectoryInterface>::goalValid(
    const typename Base::GoalConstPtr& goal)
  {
    if ((!goal->path_tolerance.empty() && goal->path_tolerance.size() != m_joint_names.size()) ||
        (!goal->goal_tolerance.empty() && goal->goal_tolerance.size() != m_joint_names.size()))
    {
      ROS_ERROR("Given tolerances must match the number of joints");
      typename Base::FollowTrajectoryResult result;
      result.error_code = Base::FollowTrajectoryResult::INVALID_GOAL;
      m_action_server->setAborted(result);
      return false;
    }
    return true;
  }

  template <>
  bool PassThroughController<hardware_interface::CartesianTrajectoryInterface>::goalValid(
    const typename Base::GoalConstPtr& goal)
  {
    // TODO: Implement
    return true;
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::timesUpCB(const ros::TimerEvent& event)
  {
    // Use most recent feedback
    typename Base::TrajectoryPoint p = m_trajectory_handle->getFeedback().error;
    typename Base::FollowTrajectoryResult result;

    // Abort if any of the joints exceeds its goal tolerance
    if (!withinTolerances(p, m_goal_tolerances))
    {
      result.error_code = Base::FollowTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      m_action_server->setAborted(result);

      // TODO: Preempt vendor control?
      // When the time is up on the ROS side, the vendor trajectory controller
      // might still be running.
      // What's a suitable policy for this in ROS?
    }
    else // Succeed
    {
      result.error_code = Base::FollowTrajectoryResult::SUCCESSFUL;
      m_action_server->setSucceeded(result);
    }

    m_done = true;
  }


}


// Exports

namespace joint_trajectory_controllers {

using PassThroughController =
  trajectory_controllers::PassThroughController<hardware_interface::JointTrajectoryInterface>;
}

namespace cartesian_trajectory_controllers {

using PassThroughController =
  trajectory_controllers::PassThroughController<hardware_interface::CartesianTrajectoryInterface>;
}

PLUGINLIB_EXPORT_CLASS(joint_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)
