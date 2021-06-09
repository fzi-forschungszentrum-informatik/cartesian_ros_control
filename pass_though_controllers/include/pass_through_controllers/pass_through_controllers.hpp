// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controllers.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <pass_through_controllers/trajectory_interface.h>
#include <pass_through_controllers/pass_through_controllers.h>

// Other
#include "ros/duration.h"
#include "ros/timer.h"
#include "speed_scaling_interface/speed_scaling_interface.h"
#include <sstream>
#include <string>

namespace trajectory_controllers
{
template <class TrajectoryInterface>
bool PassThroughController<TrajectoryInterface>::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
                                                      ros::NodeHandle& controller_nh)
{
  // Get names of joints from the parameter server
  if (!controller_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace() << "/joints from parameter server");
    return false;
  }

  // Availability checked by MultiInterfaceController
  trajectory_interface_ = hw->get<TrajectoryInterface>();
  if (!trajectory_interface_)
  {
    ROS_ERROR_STREAM(controller_nh.getNamespace() << ": No suitable trajectory interface found.");
    return false;
  }

  trajectory_interface_->setResources(joint_names_);

  // Use speed scaling interface if available (optional).
  auto speed_scaling_interface = hw->get<hardware_interface::SpeedScalingInterface>();
  if (!speed_scaling_interface)
  {
    ROS_INFO_STREAM(controller_nh.getNamespace() << ": Your RobotHW seems not to provide speed scaling. Starting "
                                                    "without this feature.");
    speed_scaling_ = nullptr;
  }
  else
  {
    speed_scaling_ = std::make_unique<hardware_interface::SpeedScalingHandle>(speed_scaling_interface->getHandle("speed"
                                                                                                                 "_scal"
                                                                                                                 "ing_"
                                                                                                                 "facto"
                                                                                                                 "r"));
  }

  // Action server
  action_server_.reset(new actionlib::SimpleActionServer<typename Base::FollowTrajectoryAction>(
      controller_nh,
      std::is_same<TrajectoryInterface, hardware_interface::JointTrajectoryInterface>::value ? "follow_joint_"
                                                                                               "trajectory" :
                                                                                               "follow_cartesian_"
                                                                                               "trajectory",
      std::bind(&PassThroughController::executeCB, this, std::placeholders::_1), false));

  // This is the cleanest method to notify the vendor robot driver of
  // preempted requests.
  action_server_->registerPreemptCallback(std::bind(&PassThroughController::preemptCB, this));

  // Register callback for when hardware finishes (prematurely)
  trajectory_interface_->registerDoneCallback(std::bind(&PassThroughController::doneCB, this, std::placeholders::_1));

  action_server_->start();

  return true;
}

template <class TrajectoryInterface>
void PassThroughController<TrajectoryInterface>::starting(const ros::Time& time)
{
  done_ = true;  // wait with update() until the next goal comes in.
}

template <class TrajectoryInterface>
void PassThroughController<TrajectoryInterface>::stopping(const ros::Time& time)
{
  if (action_server_->isActive())
  {
    // Stop trajectory interpolation on the robot
    trajectory_interface_->setCancel();

    typename Base::FollowTrajectoryResult result;
    result.error_string = "Controller stopped.";
    result.error_code = Base::FollowTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    action_server_->setAborted(result);
    done_ = true;
  }
}

template <class TrajectoryInterface>
void PassThroughController<TrajectoryInterface>::update(const ros::Time& time, const ros::Duration& period)
{
  if (action_server_->isActive() && !done_)
  {
    // Measure action duration and apply speed scaling if available.
    const double factor = (speed_scaling_) ? *speed_scaling_->getScalingFactor() : 1.0;
    action_duration_.current += period * factor;

    typename Base::TrajectoryFeedback f = trajectory_interface_->getFeedback();
    action_server_->publishFeedback(f);

    // Check tolerances on each call and set terminal conditions for the
    // action server if special criteria are met.
    // Also set the done_ flag once that happens.
    monitorExecution(f);

    // Time is up.
    if (action_duration_.current >= action_duration_.target)
    {
      if (!done_)
      {
        ROS_WARN_THROTTLE(3, "The trajectory should be finished by now. "
                             "Something might be wrong with the robot. "
                             "You might want to cancel this goal.");
      }
    }
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
    action_server_->setAborted(result);
    return;
  }

  // Only allow correct tolerances if given
  if (!isValid(goal))
  {
    return;
  }
  path_tolerances_ = goal->path_tolerance;
  goal_tolerances_ = goal->goal_tolerance;

  // Notify the  vendor robot control.
  if (!trajectory_interface_->setGoal(*goal))
  {
    ROS_ERROR("Trajectory goal is invalid.");
    typename Base::FollowTrajectoryResult result;
    result.error_code = Base::FollowTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result);
    return;
  }

  // Time keeping
  action_duration_.current = ros::Duration(0.0);
  action_duration_.target = goal->trajectory.points.back().time_from_start + goal->goal_time_tolerance;

  done_ = false;
  while (!done_)
  {
    ros::Duration(0.01).sleep();
  }

  // When done, the action server is in one of the three states:
  // 1) succeeded: managed in doneCB()
  // 2) aborted: managed in update() or in doneCB()
  // 3) preempted: managed in preemptCB()
}

template <class TrajectoryInterface>
void PassThroughController<TrajectoryInterface>::preemptCB()
{
  if (action_server_->isActive())
  {
    // Notify the vendor robot control.
    trajectory_interface_->setCancel();
  }
}

template <class TrajectoryInterface>
void PassThroughController<TrajectoryInterface>::monitorExecution(const typename Base::TrajectoryFeedback& feedback)
{
  // Preempt if any of the joints exceeds its path tolerance
  if (!withinTolerances(feedback.error, path_tolerances_))
  {
    trajectory_interface_->setCancel();
  }
}

template <>
bool PassThroughController<hardware_interface::JointTrajectoryInterface>::withinTolerances(const TrajectoryPoint& error,
                                                                                           const Tolerance& tolerances)
{
  // Check each user-given tolerance field individually.
  // Fail if either the tolerance is exceeded or if the robot driver does not
  // provide semantically correct data (conservative fail).
  for (size_t i = 0; i < tolerances.size(); ++i)
  {
    if (tolerances[i].position > 0.0)
    {
      if (error.positions.size() == tolerances.size())
      {
        return std::abs(error.positions[i]) <= tolerances[i].position;
      }
      ROS_WARN("Position tolerances specified, but not fully supported by the driver implementation.");
      return false;
    }

    if (tolerances[i].velocity > 0.0)
    {
      if (error.velocities.size() == tolerances.size())
      {
        return std::abs(error.velocities[i]) <= tolerances[i].velocity;
      }
      ROS_WARN("Velocity tolerances specified, but not fully supported by the driver implementation.");
      return false;
    }

    if (tolerances[i].acceleration > 0.0)
    {
      if (error.accelerations.size() == tolerances.size())
      {
        return std::abs(error.accelerations[i]) <= tolerances[i].acceleration;
      }
      ROS_WARN("Acceleration tolerances  specified, but not fully supported by the driver implementation.");
      return false;
    }
  }

  // Every error is ok for uninitialized tolerances
  return true;
}

template <>
bool PassThroughController<hardware_interface::CartesianTrajectoryInterface>::withinTolerances(
    const typename Base::TrajectoryPoint& error, const typename Base::Tolerance& tolerances)
{
  // Every error is ok for uninitialized tolerances
  Base::Tolerance uninitialized;
  std::stringstream str_1;
  std::stringstream str_2;
  str_1 << tolerances;
  str_2 << uninitialized;

  if (str_1.str() == str_2.str())
  {
    return true;
  }

  auto not_within_limits = [](auto& a, auto& b) { return a.x > b.x || a.y > b.y || a.z > b.z; };

  // Check each individual dimension separately.
  if (not_within_limits(error.pose.position, tolerances.position_error) ||
      not_within_limits(error.pose.orientation, tolerances.orientation_error) ||
      not_within_limits(error.twist.linear, tolerances.twist_error.linear) ||
      not_within_limits(error.twist.angular, tolerances.twist_error.angular) ||
      not_within_limits(error.acceleration.linear, tolerances.acceleration_error.linear) ||
      not_within_limits(error.acceleration.angular, tolerances.acceleration_error.angular))
  {
    return false;
  }

  return true;
}

template <>
bool PassThroughController<hardware_interface::JointTrajectoryInterface>::isValid(
    const typename Base::GoalConstPtr& goal)
{
  // If tolerances are given, they must be given for all joints.
  if ((!goal->path_tolerance.empty() && goal->path_tolerance.size() != joint_names_.size()) ||
      (!goal->goal_tolerance.empty() && goal->goal_tolerance.size() != joint_names_.size()))
  {
    ROS_ERROR("Given tolerances must match the number of joints");
    typename Base::FollowTrajectoryResult result;
    result.error_code = Base::FollowTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result);
    return false;
  }
  return true;
}

template <>
bool PassThroughController<hardware_interface::CartesianTrajectoryInterface>::isValid(
    const typename Base::GoalConstPtr& goal)
{
  // No plausibility check required. All possible user inputs seem fine for now.
  return true;
}

template <class TrajectoryInterface>
void PassThroughController<TrajectoryInterface>::doneCB(const hardware_interface::ExecutionState& state)
{
  typename Base::FollowTrajectoryResult result;

  if (!action_server_->isActive())
  {
    return;
  }

  switch (state)
  {
    case hardware_interface::ExecutionState::ABORTED: {
      result.error_string = "Trajectory aborted by the robot. Something unexpected happened.";
      result.error_code = Base::FollowTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setAborted(result);
    }
    break;

    case hardware_interface::ExecutionState::PREEMPTED: {
      result.error_string = "Trajectory preempted. Possible reasons: user request | path tolerances fail.";
      result.error_code = Base::FollowTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setPreempted(result);
    }
    break;

    case hardware_interface::ExecutionState::SUCCESS: {
      // Check if we meet the goal tolerances.
      // The most recent feedback gets us the current state.
      typename Base::TrajectoryPoint p = trajectory_interface_->getFeedback().error;
      if (!withinTolerances(p, goal_tolerances_))
      {
        result.error_string = "Trajectory finished execution but failed goal tolerances";
        result.error_code = Base::FollowTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
        action_server_->setAborted(result);
      }
      else
      {
        result.error_string = "Trajectory execution successful";
        result.error_code = Base::FollowTrajectoryResult::SUCCESSFUL;
        action_server_->setSucceeded(result);
      }
    }
    break;

    default:
      result.error_string = "Trajectory finished in unknown state.";
      result.error_code = Base::FollowTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setAborted(result);
      break;
  }

  done_ = true;
}

}  // namespace trajectory_controllers
